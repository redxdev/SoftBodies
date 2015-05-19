#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/Camera.h"
#include "cinder/params/Params.h"

#include "Physics.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class PhysicsApp : public AppNative {
public:
	void setup();
	void mouseDown( MouseEvent event );	
	void update();
	void draw();

private:
	CameraPersp Cam;
	SimulationWorld* World;
	float lastTime = 0;

	params::InterfaceGl Params;
};

void PhysicsApp::setup()
{
	srand(time(NULL));

	glDisable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_FLAT);

	float lightPosition[4] = {0.0f, 0.0f, 1.0f, 0.0f};
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	Cam.setPerspective(45.0f, getWindowAspectRatio(), 0.1f, 100.0f);
	Cam.lookAt(Vec3f(-15, 0, 0), Vec3f(0, 0, 0), Vec3f(0, 0, 1));

	World = new SimulationWorld(WORLD_SIZE, WORLD_SIZE, WORLD_SIZE);

	Params = params::InterfaceGl("Simulation", Vec2i(225, 200));
	Params.addParam("Gravity", &World->Gravity, "axisx=-z axisy=-x axisz=y");
	Params.addSeparator();
	Params.addParam("Enable Ball", &World->EnableBall);
	Params.addParam("Wireframe", &World->Wireframe);
	Params.addParam("Draw Cloth", &World->DrawCloth);
	Params.addParam("Spring Length", &World->SpringLength, "min=0.1 max=5.0");
	Params.addParam("Spring Constant", &World->SpringConstant, "min=0.1 max=10");
	Params.addParam("Random Locked Points", &World->RandomLockedPoints);
	Params.addParam("No Locked Points", &World->NoLockedPoints);
	Params.addButton("Reset Simulation", std::bind(&SimulationWorld::ResetSprings, World));
}

void PhysicsApp::mouseDown( MouseEvent event )
{
}

void PhysicsApp::update()
{
	float delta = getElapsedSeconds() - lastTime;
	lastTime = getElapsedSeconds();
	World->Simulate(delta);
}

void PhysicsApp::draw()
{
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) );

	gl::pushMatrices();
	gl::setMatrices(Cam);

	World->Render();

	gl::popMatrices();

	Params.draw();
}

CINDER_APP_NATIVE( PhysicsApp, RendererGl )
