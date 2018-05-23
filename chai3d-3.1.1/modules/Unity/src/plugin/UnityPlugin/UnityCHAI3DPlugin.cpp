//------------------------------------------------------------------------------
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#include "UnityCHAI3DPlugin.h"
//------------------------------------------------------------------------------

// every b sec spam the console with a for 1s
//#define printSometimes(s, interval) if((std::time(0)/1000) % interval) printf("%s", s); 

#ifdef CONSOLE_SPAM
#define printVec(name, vec) std::cout << name << ':' << vec.str() << std::endl;
#define printJoss(name, s)  std::cout << name << ':' << s << std::endl;
#else
#define printVec(name, vec);
#endif

extern "C" {
	// a world that contains all objects of the virtual environment
	cWorld* world;

	// a haptic device handler
	cHapticDeviceHandler* handler;

	// a pointer to the current haptic device
	cGenericHapticDevicePtr hapticDevice;

	// a virtual tool representing the haptic device in the scene
	Needle* tool;

	// define the radius of the tool (sphere)
	double toolRadius;

	// indicates if the haptic simulation currently running
	bool simulationRunning = true;

	// indicates if the haptic simulation has terminated
	bool simulationFinished = false;

	// frequency counter to measure the simulation haptic rate
	cFrequencyCounter frequencyCounter;

	// get spec of haptic device
	cHapticDeviceInfo hapticDeviceInfo;


	bool prepareHaptics(double hapticScale, double toolRadius)
	{
#ifdef _DEBUG
		FILE* pConsole;
		AllocConsole();
		freopen_s(&pConsole, "CONOUT$", "wb", stdout);

		time_t currentTime;
		time(&currentTime);

		std::cout << "Joss wuz here yo. Time: " << currentTime << std::endl;
#endif

		//--------------------------------------------------------------------------
		// WORLD
		//--------------------------------------------------------------------------

		// create a new world.
		world = new cWorld();

		//--------------------------------------------------------------------------
		// HAPTIC DEVICES / TOOLS
		//--------------------------------------------------------------------------

		// create a haptic device handler
		handler = new cHapticDeviceHandler();

		// get access to the first available haptic device
		if (!handler->getDevice(hapticDevice, 0))
			return false;

		// retrieve information about the current haptic device
		hapticDeviceInfo = hapticDevice->getSpecifications();

#ifdef _DEBUG
		std::cout << "===[ device info ]===" << std::endl; 
		std::cout << "model id: " << hapticDeviceInfo.m_model << std::endl;
		std::cout << "model name: " << hapticDeviceInfo.m_modelName << std::endl;
		std::cout << "manufacturer: " << hapticDeviceInfo.m_manufacturerName << std::endl;
#endif

		// create a 3D tool and add it to the world
		tool = new Needle(world);
		world->addChild(tool);

		// connect the haptic device to the tool
		tool->setHapticDevice(hapticDevice);

		// define a radius for the tool (sphere)
		tool->setRadius(toolRadius);

		// enable if objects in the scene are going to rotate of translate
		// or possibly collide against the tool. If the environment
		// is entirely static, you can set this parameter to "false"
		tool->enableDynamicObjects(true);

		// map the physical workspace of the haptic device to a larger virtual workspace.
		tool->setWorkspaceRadius(hapticScale);

		// haptic forces are enabled only if small forces are first sent to the device;
		// this mode avoids the force spike that occurs when the application starts when 
		// the tool is located inside an object for instance. 
		tool->setWaitForSmallForce(true);

		// start the haptic tool
		tool->start();

		// temp
		double pos[3] = { 0.0 };
		setSpringProperties(true, pos, 0.01, 0.5, 3.0);

		return true;
	}

	void startHaptics(void)
	{
		//--------------------------------------------------------------------------
		// START SIMULATION
		//--------------------------------------------------------------------------

		// create a thread which starts the main haptics rendering loop
		cThread* hapticsThread = new cThread();
		hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
	}

	void stopHaptics(void)
	{
		// stop the simulation
		simulationRunning = false;

		// wait for graphics and haptics loops to terminate
		while (!simulationFinished) { cSleepMs(100); }

		// close haptic device
		tool->stop();


#ifdef _DEBUG
		FreeConsole();
#endif
	}

	void updateHaptics(void)
	{
		// reset clock
		cPrecisionClock clock;
		clock.reset();

		// simulation in now running
		simulationRunning = true;
		simulationFinished = false;

		// main haptic simulation loop
		while (simulationRunning)
		{
			/////////////////////////////////////////////////////////////////////
			// SIMULATION TIME
			/////////////////////////////////////////////////////////////////////

			// stop the simulation clock
			clock.stop();

			// read the time increment in seconds
			double timeInterval = clock.getCurrentTimeSeconds();

			// restart the simulation clock
			clock.reset();
			clock.start();

			// update frequency counter
			frequencyCounter.signal(1);

			/////////////////////////////////////////////////////////////////////
			// HAPTIC FORCE COMPUTATION
			/////////////////////////////////////////////////////////////////////

			// compute global reference frames for each object
			world->computeGlobalPositions(true);

			// update position and orientation of tool
			tool->updateFromDevice();

			// compute interaction forces
			tool->computeInteractionForces();

			// send forces to haptic device
			tool->applyToDevice();
		}

		// exit haptics thread
		simulationFinished = true;
	}

	void getProxyPosition(double outPosArray[])
	{
		if (simulationRunning)
		{
			outPosArray[0] = tool->m_hapticPoint->getGlobalPosProxy().x();
			outPosArray[1] = tool->m_hapticPoint->getGlobalPosProxy().y();
			outPosArray[2] = tool->m_hapticPoint->getGlobalPosProxy().z();
			convertXYZFromCHAI3D(outPosArray);
		}
		else
		{
			outPosArray[0] = 0;
			outPosArray[1] = 0;
			outPosArray[2] = 0;
		}
	}

	void getDevicePosition(double outPosArray[])
	{
		if (simulationRunning)
		{
			outPosArray[0] = tool->m_hapticPoint->getGlobalPosGoal().x();
			outPosArray[1] = tool->m_hapticPoint->getGlobalPosGoal().y();
			outPosArray[2] = tool->m_hapticPoint->getGlobalPosGoal().z();
			convertXYZFromCHAI3D(outPosArray);
		}
		else
		{
			outPosArray[0] = 0;
			outPosArray[1] = 0;
			outPosArray[2] = 0;
		}
	}

	bool isTouching(int objectId)
	{
		return tool->m_hapticPoint->isInContact(world->getChild(objectId + 1));
	}

	bool isButtonPressed(int buttonId)
	{
		return tool->getUserSwitch(buttonId);
	}

	void addObject(double objectPos[], double objectScale[], double objectRotation[], double vertPos[][3], double normals[][3], int vertNum, int triPos[][3], int triNum)
	{
		// read the scale factor between the physical workspace of the haptic
		// device and the virtual workspace defined for the tool
		double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

		// stiffness properties
		double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

		cMesh* object = new cMesh();

		// set vertices
		for (int i = 0; i < vertNum; i++)
		{
			int vertex = object->newVertex();

			convertXYZToCHAI3D(vertPos[i]);
			cVector3d vertPosVecotor3 = cVector3d(vertPos[i][0], vertPos[i][1], vertPos[i][2]);
			convertXYZToCHAI3D(normals[i]);
			cVector3d vertNormalVecotor3 = cVector3d(normals[i][0], normals[i][1], normals[i][2]);

			object->m_vertices->setLocalPos(vertex, vertPosVecotor3);
			object->m_vertices->setNormal(vertex, vertNormalVecotor3);
		}

		// set triangles
		for (int i = 0; i < triNum; i++)
		{
			object->newTriangle(triPos[i][2], triPos[i][1], triPos[i][0]);
		}

		// add object to world
		world->addChild(object);

		// set the position of the object at the center of the world
		convertXYZToCHAI3D(objectPos);
		object->setLocalPos(objectPos[0], objectPos[1], objectPos[2]);

		// scale object
		object->scaleXYZ(objectScale[2], objectScale[0], objectScale[1]);

		// rotate object
		object->rotateExtrinsicEulerAnglesDeg(objectRotation[2], -1 * objectRotation[0], -1 * objectRotation[1], C_EULER_ORDER_XYZ);

		// define a default stiffness for the object
		object->m_material->setStiffness(0.3 * maxStiffness);

		// define some static friction
		object->m_material->setStaticFriction(0.5);

		// define some dynamic friction
		object->m_material->setDynamicFriction(0.5);

		// render triangles haptically
		object->m_material->setHapticTriangleSides(true, false);

		// disable culling
		object->setUseCulling(false, true);

		// compute a boundary box
		object->computeBoundaryBox(true);

		// compute collision detection algorithm
		object->createAABBCollisionDetector(toolRadius);
	}

	void translateObjects(double translation[])
	{
		convertXYZToCHAI3D(translation);
		int num = world->getNumChildren();
		for (int i = 0; i < num; i++)
		{
			if (tool != world->getChild(i))
			{
				world->getChild(i)->translate(translation[0], translation[1], translation[2]);
			}
		}
	}

	void setHapticPosition(double position[])
	{
		convertXYZToCHAI3D(position);
		tool->setLocalPos(position[0], position[1], position[2]);
	}

	void setHapticRotation(double rotation[])
	{
		tool->setLocalRot(cMatrix3d(rotation[2], -1 * rotation[0], -1 * rotation[1], C_EULER_ORDER_XYZ));
	}

	void setGlobalForce(double force[])
	{
		convertXYZToCHAI3D(force);
		tool->setDeviceGlobalForce(cVector3d(force));
	}

	void setSpringProperties(bool enabled, double position[], double minDist, double maxDist, double maxForce)
	{
		convertXYZToCHAI3D(position);
		tool->springProperties.enabled = enabled;
		tool->springProperties.restPosition = cVector3d(position);
		tool->springProperties.minDist = minDist;
		tool->springProperties.maxDist = maxDist;
		tool->springProperties.maxForce = maxForce;
	}

	//--------------------------------------------------------------------------
	// Utils
	//--------------------------------------------------------------------------

	void convertXYZFromCHAI3D(double inputXYZ[])
	{
		double val0 = inputXYZ[0];
		double val1 = inputXYZ[1];
		double val2 = inputXYZ[2];

		inputXYZ[0] = val1;
		inputXYZ[1] = val2;
		inputXYZ[2] = -1 * val0;
	}

	void convertXYZToCHAI3D(double inputXYZ[])
	{
		double val0 = inputXYZ[0];
		double val1 = inputXYZ[1];
		double val2 = inputXYZ[2];

		inputXYZ[0] = -1 * val2;
		inputXYZ[1] = val0;
		inputXYZ[2] = val1;
	}

}


inline cVector3d Needle::computeSpringForce()
{
	cVector3d position = m_hapticPoint->m_algorithmFingerProxy->getProxyGlobalPosition();

	//std::cout << "joss local pos: " << position.str() << std::endl;

	// transform spring anchor position into local space
	cVector3d springRestPosition = springProperties.restPosition;

	

	cVector3d displacementToTarget = springRestPosition - position;
	double dist = displacementToTarget.length();
	displacementToTarget.normalize();


	double forceMagnitude = lmapd(dist, springProperties.minDist, springProperties.maxDist, 0.0, springProperties.maxForce);
	forceMagnitude = cClamp(forceMagnitude, 0.0, springProperties.maxForce);

	cVector3d springForce = displacementToTarget * forceMagnitude;
	//std::cout << "joss springForce: " << springForce.str() << std::endl;

	return springForce;
}

void Needle::computeInteractionForces()
{
	// compute interaction forces at haptic point in global coordinates
	cVector3d interactionForce = m_hapticPoint->computeInteractionForces(m_deviceGlobalPos,
		m_deviceGlobalRot,
		m_deviceGlobalLinVel,
		m_deviceGlobalAngVel);
	cVector3d globalTorque(0.0, 0.0, 0.0);

	//printVec("interactionForce before spring", interactionForce)

	// apply custom spring effect
	if (springProperties.enabled)
	{
		interactionForce += computeSpringForce();
	}

	// update computed forces to tool

	printVec("interactionForce", interactionForce)

	//printf("joss force: %s\n", interactionForce.str()); //m_deviceGlobalForce.str());
	setDeviceGlobalForce(interactionForce);
	setDeviceGlobalTorque(globalTorque);
	setGripperForce(0.0);
}


inline double lmapd(float from, float fromMin, float fromMax, float toMin, float toMax)
{
	double fromAbs = from - fromMin;
	double fromMaxAbs = fromMax - fromMin;
	double normal = fromAbs / fromMaxAbs;
	double toMaxAbs = toMax - toMin;
	double toAbs = toMaxAbs * normal;
	double to = toAbs + toMin;
	return to;
}

Needle::Needle(cWorld * a_parentWorld) : cToolCursor(a_parentWorld)
{
}
