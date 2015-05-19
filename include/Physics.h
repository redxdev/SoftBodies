#pragma once

#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"

// Much of this code is based on Chris Hecker's 3d physics sample, which can be
// found at http://chrishecker.com/Rigid_Body_Dynamics.
// This version ports some of his code to Cinder and adds inter-body collision.

#define MAX_BOUNDING_VERTICES 20
#define NUM_CONFIGURATIONS 2
#define NUM_WALLS 6
#define NUM_BODIES 1
#define WORLD_SIZE 8

#define NUM_SPRING_CONNECTIONS 4
#define NUM_SPRING_X 10
#define NUM_SPRING_Y 10

// This wasn't immediately clear when I originally looked at Hecker's code,
// but the configuration stuff is how the simulation keeps track of both
// the current and previous states. Every frame, the current configuration is
// switched and rebuilt based on the previous state. Next frame, the previous
// state is overwritten with the new current state. Now that I think about it,
// it's a pretty clever way to deal with this.
struct BodyConfiguration
{
	ci::Vec3f CMPosition;
	ci::Matrix33f Orientation;

	ci::Vec3f CMVelocity;
	ci::Vec3f AngularMomentum;

	ci::Vec3f CMForce;
	ci::Vec3f Torque;

	ci::Matrix33f InverseWorldInertiaTensor;
	ci::Vec3f AngularVelocity;

	ci::Vec3f BoundingVertices[MAX_BOUNDING_VERTICES];
};

struct SphereBody
{
	float OneOverMass;
	ci::Matrix33f InverseBodyInertiaTensor;
	float CoefficientOfRestitution;

	BodyConfiguration Configurations[NUM_CONFIGURATIONS];
};

struct SpringConfiguration
{
	ci::Vec3f Position;
	ci::Vec3f Velocity;
	ci::Vec3f Force;
};

struct SpringNode
{
	bool LockedToWorld = false;

	SpringConfiguration Configurations[NUM_CONFIGURATIONS];

	SpringNode* ConnectedNodes[NUM_SPRING_CONNECTIONS];
};

enum class CollisionState
{
	Penetrating,
	Colliding,
	Clear
};

struct Wall
{
	ci::Vec3f Normal;
	float d;
};

class SimulationWorld
{
public:
	SimulationWorld(float WorldX, float WorldY, float WorldZ);
	~SimulationWorld();

	void Simulate(float DeltaTime);
	void Render();

	void ResetSprings();

	ci::Vec3f Gravity;
	float SpringLength = 0.3f;
	float SpringConstant = 10.f;
	bool Wireframe = true;
	bool DrawCloth = false;
	bool RandomLockedPoints = false;
	bool NoLockedPoints = false;
	bool EnableBall = false;

private:
	void InitializeBodies();

	CollisionState State;

	int SpringCollisionX;
	int SpringCollisionY;
	int SphereCollisionIndex;

	int SourceConfigurationIndex;
	int TargetConfigurationIndex;

	void ComputeForces(int ConfigurationIndex);
	void Integrate(float DeltaTime);
	CollisionState CheckForCollisions(int ConfigurationIndex);
	void ResolveCollisions(int ConfigurationIndex);

	Wall Walls[NUM_WALLS];

	SphereBody Spheres[NUM_BODIES];
	SpringNode Nodes[NUM_SPRING_X][NUM_SPRING_Y];

	int WorldSize = WORLD_SIZE;
};