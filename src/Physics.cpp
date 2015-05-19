#include "Physics.h"

#include <assert.h>

using namespace ci;

const float Kdl = 0.04f;
const float Kda = 0.01;
const float NoKdl = 0.002f;
const float NoKda = 0.001;

Matrix33f CreateSkewSymmetric(const Vec3f& Vec)
{
	Matrix33f M;
	M.at(0, 0) = 0.f;		M.at(0, 1) = -Vec.z;;	M.at(0, 2) = Vec.y;
	M.at(1, 0) = Vec.z;		M.at(1, 1) = 0.f;		M.at(1, 2) = -Vec.x;
	M.at(2, 0) = -Vec.y;	M.at(2, 1) = Vec.x;		M.at(2, 2) = 0.f;

	return M;
}

void OrthonormalizeOrientation(Matrix33f& Orientation)
{
	Vec3f X(Orientation.at(0, 0), Orientation.at(1, 0), Orientation.at(2, 0));
	Vec3f Y(Orientation.at(0, 1), Orientation.at(1, 1), Orientation.at(2, 1));
	Vec3f Z;

	X.normalize();
	Z = X.cross(Y).normalized();
	Y = Z.cross(X).normalized();

	Orientation.at(0, 0) = X[0];	Orientation.at(0, 1) = Y[0];	Orientation.at(0, 2) = Z[0];
	Orientation.at(1, 0) = X[1];	Orientation.at(1, 1) = Y[1];	Orientation.at(1, 2) = Z[1];
	Orientation.at(2, 0) = X[2];	Orientation.at(2, 1) = Y[2];	Orientation.at(2, 2) = Z[2];
}

void CreateOpenGLTransform(const Matrix33f& Orientation, const Vec3f& Position, float* M)
{
	float(*M44)[4] = (float(*)[4])M;
	int i;

	for (i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			M44[j][i] = Orientation.at(i, j);
		}
	}

	M44[0][3] = M44[1][3] = M44[2][3] = 0.0f; M44[3][3] = 1.0f;

	for (i = 0; i < 3; ++i)
	{
		M44[3][i] = Position[i];
	}
}

float GenerateUnitRandomReal()
{
	float Random = ((float)rand()) / ((float)RAND_MAX);
	assert(Random >= 0.f && Random <= 1.f);
	return Random;
}

float GenerateReasonableRandomReal()
{
	return 0.1f + 0.2f*GenerateUnitRandomReal();
}

void SimulationWorld::InitializeBodies()
{
	ResetSprings();
}

SimulationWorld::SimulationWorld(float WorldX, float WorldY, float WorldZ)
	: SourceConfigurationIndex(0), TargetConfigurationIndex(1)
{
	Gravity = Vec3f(0, 0, -10);

	InitializeBodies();

	// initialize walls
	Walls[0].Normal = Vec3f(0, -1, 0);
	Walls[0].d = WorldY / 2.f;
	Walls[1].Normal = Vec3f(0, 1, 0);
	Walls[1].d = WorldY / 2.f;

	Walls[2].Normal = Vec3f(-1, 0, 0);
	Walls[2].d = WorldX / 2.f;
	Walls[3].Normal = Vec3f(1, 0, 0);
	Walls[3].d = WorldX / 2.f;

	Walls[4].Normal = Vec3f(0, 0, -1);
	Walls[4].d = WorldZ / 2.f;
	Walls[5].Normal = Vec3f(0, 0, 1);
	Walls[5].d = WorldZ / 2.f;
}

SimulationWorld::~SimulationWorld()
{
	
}

void SimulationWorld::Render()
{
	glEnable(GL_LIGHTING);

	if (EnableBall)
	{
		glColor3f(0.8f, 0.f, 0.3f);
		for (int i = 0; i < NUM_BODIES; ++i)
		{
			Matrix33f& Orientation = Spheres[i].Configurations[SourceConfigurationIndex].Orientation;
			Vec3f& CMPosition = Spheres[i].Configurations[SourceConfigurationIndex].CMPosition;
			float M[16];

			glPushMatrix();
				CreateOpenGLTransform(Orientation, CMPosition, M);
				glMultMatrixf(M);
				gl::drawSphere(Vec3f(0, 0, 0), 1.f, 24);
			glPopMatrix();
		}
	}

	glColor3f(0.8f, 0.f, 0.f);
	for (int X = 0; X < NUM_SPRING_X - 1; ++X)
	{
		for (int Y = 0; Y < NUM_SPRING_Y - 1; ++Y)
		{
			auto& A = Nodes[X][Y];
			auto& B = Nodes[X + 1][Y];
			auto& C = Nodes[X][Y + 1];
			auto& D = Nodes[X + 1][Y + 1];

			if (DrawCloth)
			{
				glBegin(GL_TRIANGLE_STRIP);
					glVertex3f(A.Configurations[SourceConfigurationIndex].Position);
					glVertex3f(B.Configurations[SourceConfigurationIndex].Position);
					glVertex3f(C.Configurations[SourceConfigurationIndex].Position);
					glVertex3f(D.Configurations[SourceConfigurationIndex].Position);
				glEnd();
			}
		}
	}
	glDisable(GL_LIGHTING);

	if (Wireframe)
	{
		for (int NX = 0; NX < NUM_SPRING_X; ++NX)
		{
			for (int NY = 0; NY < NUM_SPRING_Y; ++NY)
			{

				auto& Node = Nodes[NX][NY];
				glColor3f(1.f, 1.f, 1.f);
				for (int Connected = 0; Connected < NUM_SPRING_CONNECTIONS; ++Connected)
				{
					auto* OtherNode = Node.ConnectedNodes[Connected];
					if (OtherNode == nullptr)
						continue;

					gl::drawLine(Node.Configurations[SourceConfigurationIndex].Position, OtherNode->Configurations[SourceConfigurationIndex].Position);
				}
			}
		}
	}

	assert(NUM_WALLS == 6);

	/*
	// draw walls
	glColor3f(1.0f, 1.0f, 1.0f);
	// do a big linestrip to get most of edges
	glBegin(GL_LINE_STRIP);
		glVertex3f(-WorldSize / 2.f, -WorldSize / 2.f, -WorldSize / 2.f);
		glVertex3f(-WorldSize / 2.f, -WorldSize / 2.f, WorldSize / 2.f);
		glVertex3f(-WorldSize / 2.f, WorldSize / 2.f, WorldSize / 2.f);
		glVertex3f(WorldSize / 2.f, WorldSize / 2.f, WorldSize / 2.f);
		glVertex3f(WorldSize / 2.f, -WorldSize / 2.f, WorldSize / 2.f);
		glVertex3f(WorldSize / 2.f, -WorldSize / 2.f, -WorldSize / 2.f);
	glEnd();
	// fill in the stragglers
	glBegin(GL_LINES);
		glVertex3f(WorldSize / 2.f, -WorldSize / 2.f, WorldSize / 2.f);
		glVertex3f(-WorldSize / 2.f, -WorldSize / 2.f, WorldSize / 2.f);

		glVertex3f(WorldSize / 2.f, WorldSize / 2.f, WorldSize / 2.f);
		glVertex3f(WorldSize / 2.f, WorldSize / 2.f, -WorldSize / 2.f);

		glVertex3f(-WorldSize / 2.f, WorldSize / 2.f, WorldSize / 2.f);
		glVertex3f(-WorldSize / 2.f, WorldSize / 2.f, -WorldSize / 2.f);
	glEnd();

	// draw floor
	glBegin(GL_LINES);
		glVertex3f(WorldSize / 2.f, WorldSize / 2.f, -WorldSize / 2.f);
		glVertex3f(-WorldSize / 2.f, WorldSize / 2.f, -WorldSize / 2.f);

		glVertex3f(-WorldSize / 2.f, WorldSize / 2.f, -WorldSize / 2.f);
		glVertex3f(-WorldSize / 2.f, -WorldSize / 2.f, -WorldSize / 2.f);

		glVertex3f(-WorldSize / 2.f, -WorldSize / 2.f, -WorldSize / 2.f);
		glVertex3f(WorldSize / 2.f, -WorldSize / 2.f, -WorldSize / 2.f);

		glVertex3f(WorldSize / 2.f, -WorldSize / 2.f, -WorldSize / 2.f);
		glVertex3f(WorldSize / 2.f, WorldSize / 2.f, -WorldSize / 2.f);
	glEnd();*/
}

void SimulationWorld::Simulate(float DeltaTime)
{
	float CurrentTime = 0;
	float TargetTime = DeltaTime;
	while (CurrentTime < DeltaTime)
	{
		ComputeForces(SourceConfigurationIndex);

		Integrate(TargetTime - CurrentTime);

		CheckForCollisions(TargetConfigurationIndex);

		if (State == CollisionState::Penetrating)
		{
			TargetTime = (CurrentTime + TargetTime) / 2.f;

			assert(fabs(TargetTime - CurrentTime) > EPSILON);
		}
		else
		{
			if (State == CollisionState::Colliding)
			{
				int Counter = 0;
				do
				{
					ResolveCollisions(TargetConfigurationIndex);
					Counter++;
				}
				while (CheckForCollisions(TargetConfigurationIndex) == CollisionState::Colliding && Counter < 1000);

				//assert(Counter < 100);
			}

			CurrentTime = TargetTime;
			TargetTime = DeltaTime;

			SourceConfigurationIndex = SourceConfigurationIndex ? 0 : 1;
			TargetConfigurationIndex = TargetConfigurationIndex ? 0 : 1;
		}
	}
}

void SimulationWorld::ComputeForces(int ConfigurationIndex)
{
	if (EnableBall)
	{
		for (int Counter = 0; Counter < NUM_BODIES; ++Counter)
		{
			SphereBody& Body = Spheres[Counter];
			BodyConfiguration& Configuration = Body.Configurations[ConfigurationIndex];

			Configuration.Torque = Vec3f(0, 0, 0);
			Configuration.CMForce = Vec3f(0, 0, 0);

			Configuration.CMForce += Gravity / Body.OneOverMass;

			// damping
			Configuration.CMForce += -Kdl * Configuration.CMVelocity;
			Configuration.Torque += -Kda * Configuration.AngularVelocity;

			// no damping
			/*Configuration.CMForce += -NoKdl * Configuration.CMVelocity;
			Configuration.Torque += -NoKda * Configuration.AngularVelocity;*/
		}
	}

	for (int NX = 0; NX < NUM_SPRING_X; ++NX)
	{
		for (int NY = 0; NY < NUM_SPRING_Y; ++NY)
		{
			auto& Node = Nodes[NX][NY];
			auto& Config = Node.Configurations[ConfigurationIndex];

			if (Node.LockedToWorld) // cannot move locked nodes
				continue;

			Config.Force = Vec3f(0, 0, 0);

			Config.Force += Gravity / 5.f;

			for (int Index = 0; Index < NUM_SPRING_CONNECTIONS; ++Index)
			{
				auto* Other = Node.ConnectedNodes[Index];
				if (Other == nullptr)
					continue;

				Vec3f MyPos = Config.Position;
				Vec3f OtherPos = Other->Configurations[ConfigurationIndex].Position;
				Vec3f Direction = OtherPos - MyPos;
				float Distance = MyPos.distance(OtherPos);

				float Diff = (SpringLength - Distance) / Distance;
				Vec3f P = Direction * Diff;

				//Config.Force += Direction * (SpringConstant * (Distance - SpringLength));
				Config.Force -= P * SpringConstant;
			}

			// Damping
			Config.Force += -Config.Velocity;
		}
	}
}

void SimulationWorld::Integrate(float DeltaTime)
{
	if (EnableBall)
	{
		for (int Counter = 0; Counter < NUM_BODIES; ++Counter)
		{
			BodyConfiguration& Source = Spheres[Counter].Configurations[SourceConfigurationIndex];
			BodyConfiguration& Target = Spheres[Counter].Configurations[TargetConfigurationIndex];

			Target.CMPosition = Source.CMPosition + DeltaTime * Source.CMVelocity;

			Target.Orientation = Source.Orientation + CreateSkewSymmetric(Source.AngularVelocity) * Source.Orientation * DeltaTime;

			Target.CMVelocity = Source.CMVelocity + (DeltaTime * Spheres[Counter].OneOverMass) * Source.CMForce;

			Target.AngularMomentum = Source.AngularMomentum + DeltaTime * Source.Torque;

			OrthonormalizeOrientation(Target.Orientation);

			Target.InverseWorldInertiaTensor = Target.Orientation * Spheres[Counter].InverseBodyInertiaTensor * Target.Orientation.transposed();

			Target.AngularVelocity = Target.InverseWorldInertiaTensor * Target.AngularMomentum;
		}
	}

	for (int NX = 0; NX < NUM_SPRING_X; ++NX)
	{
		for (int NY = 0; NY < NUM_SPRING_Y; ++NY)
		{
			auto& Node = Nodes[NX][NY];
			auto& Source = Node.Configurations[SourceConfigurationIndex];
			auto& Target = Node.Configurations[TargetConfigurationIndex];

			if (Node.LockedToWorld) // cannot move locked nodes
				continue;

			Target.Position = Source.Position + DeltaTime * Source.Velocity;
			Target.Velocity = Source.Velocity + DeltaTime * Source.Force;
		}
	}
}

CollisionState SimulationWorld::CheckForCollisions(int ConfigurationIndex)
{
	State = CollisionState::Clear;

	const float DepthEpsilon = 0.001f;

	/**
	for (int BodyIndex = 0; BodyIndex < NUM_BODIES && State != CollisionState::Penetrating; ++BodyIndex)
	{
		RigidBody& Body = Bodies[BodyIndex];
		BodyConfiguration& Configuration = Body.Configurations[ConfigurationIndex];

		for (unsigned int Counter = 0; Counter < Body.NumberOfBoundingVertices && State != CollisionState::Penetrating; ++Counter)
		{
			Vec3f Position = Configuration.BoundingVertices[Counter];
			Vec3f U = Position - Configuration.CMPosition;

			Vec3f Velocity = Configuration.CMVelocity + Configuration.AngularVelocity.cross(U);

			for (int WallIndex = 0; WallIndex < NUM_WALLS && State != CollisionState::Penetrating; ++WallIndex)
			{
				Wall& W = Walls[WallIndex];

				float axbyczd = Position.dot(W.Normal) + W.d; // that's hard to type :(

				if (axbyczd < -DepthEpsilon)
				{
					State = CollisionState::Penetrating;
				}
				else if (axbyczd < DepthEpsilon)
				{
					float RelativeVelocity = W.Normal.dot(Velocity);

					if (RelativeVelocity < 0.f)
					{
						State = CollisionState::Colliding;
						CollisionNormal = W.Normal;
						CollidingCornerIndex = Counter;
						CollidingBodyIndex = BodyIndex;
					}
				}
			}
		}
	}**/

	if (EnableBall)
	{
		for (int SI = 0; SI < NUM_BODIES && State != CollisionState::Penetrating; ++SI)
		{
			auto& Body = Spheres[SI];
			auto& Configuration = Body.Configurations[ConfigurationIndex];

			for (int X = 0; X < NUM_SPRING_X && State != CollisionState::Penetrating; ++X)
			{
				for (int Y = 0; Y < NUM_SPRING_Y && State != CollisionState::Penetrating; ++Y)
				{
					auto& Node = Nodes[X][Y];
					auto& NConfig = Node.Configurations[ConfigurationIndex];
					float Dist = Configuration.CMPosition.distance(NConfig.Position);
					if (Dist < 1.f)
					{
						Vec3f N = (Configuration.CMPosition - NConfig.Position).normalized();
						float RV = N.dot(Configuration.CMVelocity);
						if (RV < 0.f)
						{
							State = CollisionState::Colliding;
							SpringCollisionX = X;
							SpringCollisionY = Y;
							SphereCollisionIndex = SI;
						}
					}
					else if (Dist > 1.f && Dist < 1.f + DepthEpsilon)
					{
						/**
						State = CollisionState::Colliding;
						SpringCollisionX = X;
						SpringCollisionY = Y;
						SphereCollisionIndex = SI;**/
					}
				}
			}
		}
	}

	return State;
}

void SimulationWorld::ResolveCollisions(int ConfigurationIndex)
{
	auto& Body = Spheres[SphereCollisionIndex];
	auto& Config = Body.Configurations[ConfigurationIndex];

	auto& Node = Nodes[SpringCollisionX][SpringCollisionY];
	auto& NConfig = Node.Configurations[ConfigurationIndex];

	float Dist = Config.CMPosition.distance(NConfig.Position);
	float PenDist = 1.f - Dist;
	Vec3f Force = (NConfig.Position - Config.CMPosition).normalized() * PenDist;

	Config.CMVelocity -= Force * Body.OneOverMass;
	Config.AngularMomentum -= (NConfig.Position - Config.CMPosition).cross(Force * Body.OneOverMass) * 100000.f;
	Config.AngularVelocity = Config.InverseWorldInertiaTensor * Config.AngularMomentum;

	NConfig.Velocity += Force;
}

void SimulationWorld::ResetSprings()
{
	for (int BodyIndex = 0; BodyIndex < NUM_BODIES; BodyIndex++)
	{
		SphereBody& Body = Spheres[BodyIndex];

		float d2 = 0.2;
		float dX2 = d2;
		float dY2 = d2;
		float dZ2 = d2;

		Vec3f Position = Vec3f(0.f, 0.f, 5.f);
		Body.Configurations[0].CMPosition = Position;
		Body.Configurations[1].CMPosition = Position;

		Body.Configurations[0].CMVelocity = Vec3f(0, 0, 0);
		Body.Configurations[1].CMVelocity = Vec3f(0, 0, 0);

		float Density = 0.4f;
		float Mass = 8.f*Density*dX2*dY2*dZ2;
		assert(Mass > 0.f);

		Body.OneOverMass = 1.f / Mass;
		Body.InverseBodyInertiaTensor.at(0, 0) = 3.f / (Mass*(dY2*dY2 + dZ2*dZ2));
		Body.InverseBodyInertiaTensor.at(1, 1) = 3.f / (Mass*(dX2*dX2 + dZ2*dZ2));
		Body.InverseBodyInertiaTensor.at(2, 2) = 3.f / (Mass*(dX2*dX2 + dY2*dY2));

		Body.CoefficientOfRestitution = 1.f;
	}

	for (int NX = 0; NX < NUM_SPRING_X; ++NX)
	{
		for (int NY = 0; NY < NUM_SPRING_Y; ++NY)
		{
			auto& Node = Nodes[NX][NY];
			for (int Config = 0; Config < NUM_CONFIGURATIONS; ++Config)
			{
				Node.Configurations[Config].Position = Vec3f(-5.f + (10.f / NUM_SPRING_X) * NX, -5.f + (10.f / NUM_SPRING_Y) * NY, 0.5f);
				Node.Configurations[Config].Velocity = Vec3f(0, 0, 0);
				Node.Configurations[Config].Force = Vec3f(0, 0, 0);
			}

			if (NX != 0)
			{
				Node.ConnectedNodes[0] = &Nodes[NX - 1][NY];
			}
			else
			{
				Node.ConnectedNodes[0] = nullptr;
			}

			if (NX != NUM_SPRING_X - 1)
			{
				Node.ConnectedNodes[1] = &Nodes[NX + 1][NY];
			}
			else
			{
				Node.ConnectedNodes[1] = nullptr;
			}

			if (NY != 0)
			{
				Node.ConnectedNodes[2] = &Nodes[NX][NY - 1];
			}
			else
			{
				Node.ConnectedNodes[2] = nullptr;
			}

			if (NY != NUM_SPRING_Y - 1)
			{
				Node.ConnectedNodes[3] = &Nodes[NX][NY + 1];
			}
			else
			{
				Node.ConnectedNodes[3] = nullptr;
			}

			if (NoLockedPoints)
			{
				Node.LockedToWorld = false;
			}
			else
			{
				if (RandomLockedPoints)
				{
					if (GenerateUnitRandomReal() < 0.05)
					{
						Node.LockedToWorld = true;
					}
					else
					{
						Node.LockedToWorld = false;
					}
				}
				else
				{
					if (NX == 0 && NY == 0 || NX == NUM_SPRING_X - 1 && NY == 0 || NX == 0 && NY == NUM_SPRING_Y - 1 || NX == NUM_SPRING_X - 1 && NY == NUM_SPRING_Y - 1)
					{
						Node.LockedToWorld = true;
					}
					else
					{
						Node.LockedToWorld = false;
					}
				}
			}
		}
	}
}
