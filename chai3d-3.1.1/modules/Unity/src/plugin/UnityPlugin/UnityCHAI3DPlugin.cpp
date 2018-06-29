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

// quality-of-life debugging preprocessor macros, because i'm lazy - joss
#define HAPTIC_DEBUG

#ifdef HAPTIC_DEBUG
#define PRINTLN(x) std::cout << x << std::endl;
#define PRINTWAIT(x, time) std::cout << x << std::endl; Sleep(time);
#else
#define PRINTLN(x) ; 
#define PRINTWAIT(x, time) ;
#endif

namespace NeedleSimPlugin
{
	extern "C" {

		// a world that contains all objects of the virtual environment
		cWorld* world;

		// clock
		cPrecisionClock toolClock;

		double deltaTime;

		// a haptic device handler
		cHapticDeviceHandler* handler;

		// a pointer to the current haptic device
		cGenericHapticDevicePtr* hapticDevice;

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

		HapticLayerContainer patient;

#ifdef HAPTIC_DEBUG
		bool lastForceEngagedState(false);
#endif

		bool prepareHaptics(double hapticScale)
		{
#ifdef HAPTIC_DEBUG
			FILE* pConsole;
			AllocConsole();
			freopen_s(&pConsole, "CONOUT$", "wb", stdout);
#endif

			//--------------------------------------------------------------------------
			// WORLD
			//--------------------------------------------------------------------------

			world = new cWorld();

			//--------------------------------------------------------------------------
			// HAPTIC DEVICES / TOOLS
			//--------------------------------------------------------------------------

			// create a haptic device handler
			handler = new cHapticDeviceHandler();

			// instantiate the shared pointer to the device. The reason it is created and deleted is because it does not leave scope when you stop the play session in Unity.
			hapticDevice = new cGenericHapticDevicePtr();

			// get access to the first available haptic device
			if (!handler->getDevice(*hapticDevice, 0))
				return false;

			// retrieve information about the current haptic device
			hapticDeviceInfo = (*hapticDevice)->getSpecifications();

			// create a 3D tool and add it to the world
			tool = new Needle(world);
			world->addChild(tool);

			// connect the haptic device to the tool
			tool->setHapticDevice(*hapticDevice);

			// define the radius of the tool (sphere)
			toolRadius = 0.1;

			// define a radius for the tool. Also used for collision bounding box creation
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

#ifdef HAPTIC_DEBUG
			lastForceEngagedState = tool->isForceEngaged();

			std::cout << "===[ world initialized ]===" << std::endl;
			std::cout << "model id: " << hapticDeviceInfo.m_model << std::endl;
			std::cout << "model name: " << hapticDeviceInfo.m_modelName << std::endl;
			std::cout << "manufacturer: " << hapticDeviceInfo.m_manufacturerName << std::endl;
			std::cout << "===============================\n" << std::endl;
#endif
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
			(*hapticDevice)->close();
			tool->stop();


			//--------------------------------------------------------------------------
			// Memory cleanup
			//--------------------------------------------------------------------------

			delete handler;
			handler = nullptr;

			// clean up world. This will also delete the tool, because the tool is a child of the world
			world->deleteAllChildren();
			delete world;
			world = nullptr;
			tool = nullptr;

			// clean up device shared pointer
			delete hapticDevice;

#ifdef HAPTIC_DEBUG
			FreeConsole();
#endif
		}

		void updateHaptics(void)
		{
			// reset clock
			toolClock.reset();

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
				toolClock.stop();

				// read the time increment in seconds
				deltaTime = toolClock.getCurrentTimeSeconds();

				// restart the simulation clock
				toolClock.reset();
				toolClock.start();

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

#ifdef HAPTIC_DEBUG

				// report on when the haptic force feedback is enabled/disabled
				{
					bool isEngaged = tool->isForceEngaged();
					if (isEngaged != lastForceEngagedState)
					{
						std::cout << "Force feedback engaged?: " << to_string(isEngaged) << std::endl;
					}
					lastForceEngagedState = isEngaged;
				}

				// report on framerate
				{
					static double time(0.0);
					static int num_samples(0);
					time += deltaTime;
					num_samples++;

					if (num_samples == 10000) // reports approx. every 10sec
					{
						std::cout << "avg framerate: " << time / num_samples << std::endl;
						num_samples = 0;
						time = 0.0;
					}
				}

#endif HAPTIC_DEBUG
			}

			// exit haptics thread
			simulationFinished = true;
		}

		void getProxyPosition(double outPosArray[])
		{
			if (simulationRunning)
			{
				outPosArray[0] = tool->m_hapticTip->getGlobalPosProxy().x();
				outPosArray[1] = tool->m_hapticTip->getGlobalPosProxy().y();
				outPosArray[2] = tool->m_hapticTip->getGlobalPosProxy().z();
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
				outPosArray[0] = tool->m_hapticTip->getGlobalPosGoal().x();
				outPosArray[1] = tool->m_hapticTip->getGlobalPosGoal().y();
				outPosArray[2] = tool->m_hapticTip->getGlobalPosGoal().z();
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
			return tool->m_hapticTip->isInContact(world->getChild(objectId));
		}

		bool isButtonPressed(int buttonId)
		{
			return tool->getUserSwitch(buttonId);
		}

		void setToolRadius(double a_toolRadius)
		{
			toolRadius = a_toolRadius;
			tool->setRadius(a_toolRadius);
		}

		int addObject(double objectPos[], double objectScale[], double objectRotation[], double vertPos[][3], double normals[][3], int vertNum, int triPos[][3], int triNum)
		{
			// read the scale factor between the physical workspace of the haptic
			// device and the virtual workspace defined for the tool
			double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

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

			// set the position of the object at the center of the world
			convertXYZToCHAI3D(objectPos);
			object->setLocalPos(objectPos[0], objectPos[1], objectPos[2]);

			// scale object
			object->scaleXYZ(objectScale[2], objectScale[0], objectScale[1]);

			// rotate object
			object->rotateExtrinsicEulerAnglesDeg(objectRotation[2], -1 * objectRotation[0], -1 * objectRotation[1], C_EULER_ORDER_XYZ);

			// stiffness property
			double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

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

			// add object to world
			world->addChild(object);

			int objectID = (world->getNumChildren() - 1);

			return objectID;
		}

		int addBoxObject(double objectPos[], double objectScale[], double objectRotation[])
		{
			// create the box and include the size
			convertXYZToCHAI3D(objectScale);
			cShapeBox* box = new cShapeBox(objectScale[0], objectScale[1], objectScale[2]);

			// read the scale factor between the physical workspace of the haptic
			// device and the virtual workspace defined for the tool
			double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

			// stiffness property
			double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

			// define a default stiffness for the object
			box->m_material->setStiffness(0.5 * maxStiffness);//);

			// define some static friction
			box->m_material->setStaticFriction(0.5);

			// define some dynamic friction
			box->m_material->setDynamicFriction(0.5);

			// set the position of the object at the center of the world
			convertXYZToCHAI3D(objectPos);
			box->setLocalPos(objectPos[0], objectPos[1], objectPos[2]);

			// rotate object. don't mind the weird order of rotations. that is because it is converting from unity's coordinate system to chai3d's
			box->rotateExtrinsicEulerAnglesDeg(objectRotation[2], -1 * objectRotation[0], -1 * objectRotation[1], C_EULER_ORDER_XYZ);

			box->createEffectSurface();

			//// compute a boundary box
			box->computeBoundaryBox(true);

			world->addChild(box);

			// return an ID number
			int objectID = (world->getNumChildren() - 1); // child 0 is the tool itself.
			std::cout << "box created! ID: " << objectID << std::endl;
			return objectID;
		}

		void setObjectProperties(int objectID, double stiffness, double friction_static, double friction_dynamic, double viscosity, double penetrationForce)
		{
			cGenericObject* object = world->getChild(objectID);

			// read the scale factor between the physical workspace of the haptic
			// device and the virtual workspace defined for the tool
			double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

			// stiffness properties
			double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

			// stiffness for the object in N/m
			object->m_material->setStiffness(stiffness * maxStiffness);

			// coefficient of static friction
			object->m_material->setStaticFriction(friction_static);

			// coefficient of dynamic friction
			object->m_material->setDynamicFriction(friction_dynamic);

			// viscosity (only active if a viscosity effect is also on it)
			object->m_material->setViscosity(viscosity);

			// force to penetrate
			object->m_material->setPenetrationForceThreshold(penetrationForce);
		}

		void addViscosityEffect(int objectID, double viscosity)
		{
			cGenericObject* object = world->getChild(objectID);

			//create and link effect
			object->addEffect(new cEffectViscosity(object));
			object->m_material->setViscosity(viscosity); // set property
		}

		void addMembraneEffect(int objectID, double a_resistance, double a_friction_static, double a_friction_dynamic, double maxForce, double distanceToMaxForce, double a_springMass, double a_penetrationThreshold)
		{
			PRINTLN("adding membrane effect!")

				// get object by index in the world scope
				cGenericObject* object = world->getChild(objectID);

			// create effect and set properties
			SpringyMembrane* effect = new SpringyMembrane(object);
			effect->m_resistance = a_resistance;
			effect->m_friction_dynamic = a_friction_dynamic;
			effect->m_friction_static = a_friction_static;
			effect->spring.maxForce = maxForce;
			effect->spring.maxDist = distanceToMaxForce;
			effect->m_springMass = a_springMass;
			effect->m_penetrationThreshold = a_penetrationThreshold;

			PRINTLN("membrane effect added!")

				// link it to the object
				object->addEffect(effect);
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

		void setNeedleAxialConstraint(bool enabled, double position[], double direction[], double minDist, double maxDist, double maxForce, double damping)
		{
			convertXYZToCHAI3D(position);
			convertXYZToCHAI3D(direction);

			tool->axialConstraint.enabled = enabled;
			tool->axialConstraint.position = cVector3d(position);
			tool->axialConstraint.direction = cVector3d(direction);
			tool->axialConstraint.minDist = minDist;
			tool->axialConstraint.maxDist = maxDist;
			tool->axialConstraint.maxForce = maxForce;
			tool->axialConstraint.damping = damping;
		}

		void setNeedleAxialSpring(bool enabled, double position[], double direction[], double minDist, double maxDist, double maxForce, double damping)
		{
			convertXYZToCHAI3D(position);
			convertXYZToCHAI3D(direction);

			tool->axialSpring.enabled = enabled;
			tool->axialSpring.position = cVector3d(position);
			tool->axialSpring.direction = cVector3d(direction);
			tool->axialSpring.minDist = minDist;
			tool->axialSpring.maxDist = maxDist;
			tool->axialSpring.maxForce = maxForce;
			tool->axialSpring.damping = damping;
		}

		void setHapticEntryPoint(double position[], double direction[])
		{
			convertXYZToCHAI3D(position);
			convertXYZToCHAI3D(direction);

			patient.m_entryPoint = cVector3d(position);
			patient.m_entryDirection = cVector3d(direction);
			//PRINTLN("entry point set!")
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



		///////////////////////////////////////////
		// Needle
		///////////////////////////////////////////

		void Needle::computeInteractionForces()
		{
			// for each haptic point compute the interaction force
			// and combine their overall contribution to compute the output force
			// and torque to be sent to the haptic device

			// initialize variables
			cVector3d interactionForce(0.0, 0.0, 0.0);
			cVector3d globalTorque(0.0, 0.0, 0.0);

			//int numContactPoint = (int)(m_hapticPoints.size());
			//for (int i = 0; i< numContactPoint; i++)
			//{
			//	// get next haptic point
			//	cHapticPoint* nextContactPoint = m_hapticPoints[i];
			//
			//	// compute force at haptic point as well as new proxy position
			//	cVector3d t_force = nextContactPoint->computeInteractionForces(m_deviceGlobalPos,
			//		m_deviceGlobalRot,
			//		m_deviceGlobalLinVel,
			//		m_deviceGlobalAngVel);
			//
			//	cVector3d t_pos = nextContactPoint->getGlobalPosProxy();
			//
			//	// combine force contributions together
			//	interactionForce.add(t_force);
			//}


			// compute interaction forces at haptic point in global coordinates
			interactionForce = m_hapticTip->computeInteractionForces(m_deviceGlobalPos,
				m_deviceGlobalRot,
				m_deviceGlobalLinVel,
				m_deviceGlobalAngVel);

			cVector3d devicePos = m_hapticTip->m_algorithmFingerProxy->getDeviceGlobalPosition();

			// apply custom spring effect
			if (springProperties.enabled)
			{
				interactionForce += computeSpringForce(devicePos, springProperties.restPosition, springProperties.minDist, springProperties.maxDist, springProperties.maxForce);
			}

			if (axialConstraint.enabled)
			{
				cVector3d axialConstraintForce = computeAxialConstraintForce(devicePos, axialConstraint.position, axialConstraint.direction, axialConstraint.minDist, axialConstraint.maxDist, axialConstraint.maxForce, axialConstraint.damping);
				// might need a better mapping to allow axial constraint to do its job while other forces apply. Look to diminishing returns from video games? 

				interactionForce += axialConstraintForce;
			}

			if (axialSpring.enabled)
			{
				cVector3d axialConstraintForce = computeAxialSpringForce(devicePos, axialConstraint.position, axialConstraint.direction, axialConstraint.minDist, axialConstraint.maxDist, axialConstraint.maxForce, axialConstraint.damping);
				// might need a better mapping to allow axial constraint to do its job while other forces apply. Look to diminishing returns from video games? 

				interactionForce += axialConstraintForce;
			}

			interactionForce += patient.computeForces(devicePos);

			// avoid trying to apply more force than possible
			interactionForce.clamp(hapticDeviceInfo.m_maxLinearForce);

			setDeviceGlobalForce(interactionForce);
			setDeviceGlobalTorque(globalTorque);
			setGripperForce(0.0);
		}

		bool Needle::isForceEngaged()
		{
			return m_forceEngaged;
		}

		Needle::Needle(cWorld * a_parentWorld) : cGenericTool(a_parentWorld)
		{
			int numPoints = 1;
			for (int i = 0; i < numPoints; i++)
			{
				cHapticPoint* newPoint = new cHapticPoint(this);

				m_hapticPoints.push_back(newPoint);
			}

			m_hapticTip = m_hapticPoints[0];
		}

		Needle::~Needle()
		{
			for (auto point : m_hapticPoints)
			{
				delete point;
				point = nullptr;
			}

			m_hapticPoints.clear();
		}

	} // extern 

	inline cVector3d computeSpringForce(const cVector3d& position, cVector3d& springRestPosition, double& minDist, double& maxDist, double& maxForce)
	{
		cVector3d displacementToTarget = springRestPosition - position;
		double dist = displacementToTarget.length();
		displacementToTarget.normalize();

		double forceMagnitude = lmapd(dist, minDist, maxDist, 0.0, maxForce);
		forceMagnitude = cClamp(forceMagnitude, 0.0, maxForce);

		cVector3d springForce = displacementToTarget * forceMagnitude;

		return springForce;
	}

	inline double lmapd(const double& from, const double& fromMin, const double& fromMax, const double& toMin, const double& toMax)
	{
		double fromAbs = from - fromMin;
		double fromMaxAbs = fromMax - fromMin;
		double normal = fromAbs / fromMaxAbs;
		double toMaxAbs = toMax - toMin;
		double toAbs = toMaxAbs * normal;
		double to = toAbs + toMin;
		return to;
	}

#undef JOSSDEBUG

	SpringyMembrane::SpringyMembrane(cGenericObject * a_parent) : cGenericEffect(a_parent)
	{
		spring.enabled = true;
		spring.restPosition = cVector3d(0.0, 0.0, 0.0);
		spring.minDist = 0.0;
		spring.maxDist = 0.01;
		spring.maxForce = 9.0;
		m_wasInsideLastFrame = false;
		m_membranePenetrated = false;
	}

	SpringyMembrane::~SpringyMembrane()
	{
	}

	bool SpringyMembrane::computeForce(const cVector3d & a_toolPos,
		const cVector3d & a_toolVel,
		const unsigned int & a_toolID,
		cVector3d & a_reactionForce)
	{
		if (m_parent->m_interactionInside)
		{
			// if this is the first penetration
			if (!m_wasInsideLastFrame)
			{
				m_wasInsideLastFrame = true;

				// initialize the spring on the device position. 
				// the tool is at first pulled back towards its initial entry point
				spring.restPosition = a_toolPos; //tool->m_hapticTip->getLocalPosGoal();
			}

			double devicePos[3];
			getDevicePosition(devicePos);
			cVector3d pos(devicePos);

			// apply force onto the haptic device, pulling the tool toward the spring tail, wherever it may be
			cVector3d force = computeSpringForce(pos, spring.restPosition, spring.minDist, spring.maxDist, spring.maxForce);
			a_reactionForce = force;

			double forceMag = force.length();

			if (forceMag >= m_penetrationThreshold && !m_membranePenetrated)
			{
				m_membranePenetrated = true;
				PRINTLN("membrane penetrated!")
			}

			//if penetrated, allow the spring to move.
			if (m_membranePenetrated)
			{
				double maxStaticFrictionOutput = m_resistance * m_friction_static;

				if (forceMag > maxStaticFrictionOutput)
				{
					m_useDynamicFriction = true;
				}

				// static friction
				if (!m_useDynamicFriction)
				{
					// don't move. Do nothing.
				}
				else // dynamic friction
				{
					double maxDynamicFrictionOutput = m_resistance * m_friction_dynamic;

					double netForceMag = max(0.0, forceMag - maxDynamicFrictionOutput);

					if (netForceMag <= 0.00001)
					{
						m_useDynamicFriction = false;
						PRINTLN("back to static friction!")
					}

					// the spring tail moves toward the tool
					double acceleration = netForceMag / m_springMass;

					// don't bother with actual velocity. just move the spring along the acceleration vector.
					cVector3d velocity = -acceleration * cNormalize(force) * deltaTime;

					spring.restPosition += velocity * deltaTime;
				}
			}
		}
		else
		{
			m_wasInsideLastFrame = false;
			m_membranePenetrated = false;

			a_reactionForce.zero();
			return false;
		}
	}

	bool SpringyMembrane::isPenetrated()
	{
		return m_membranePenetrated;
	}

	//cPenetrablePoint::cPenetrablePoint(cGenericTool * a_parentTool) : cHapticPoint(a_parentTool)
	//{
	//	// replace finger-proxy algorithm used for modelling contacts with 
	//	// cMesh objects.
	//	delete m_algorithmFingerProxy;
	//	m_algorithmFingerProxy = new cAlgorithmFingerProxyPuncture();
	//}
	//
	//cPenetrablePoint::~cPenetrablePoint()
	//{
	//}

	inline cVector3d computeAxialConstraintForce(cVector3d position, cVector3d & targetPos, cVector3d & targetDir, double & minDist, double & maxDist, double & maxForce, double& kDamping)
	{
		cVector3d displacementToTarget = targetPos - position;
		displacementToTarget = displacementToTarget.projectToPlane(targetDir);
		double dist = displacementToTarget.length();
		displacementToTarget.normalize();

		// spring
		double forceMagnitude = lmapd(dist, minDist, maxDist, 0.0, maxForce);
		cVector3d springForce = displacementToTarget * forceMagnitude;

		// speed damping
		cVector3d vel = tool->getDeviceGlobalLinVel().projectToPlane(targetDir);;
		springForce -= kDamping * vel * maxForce;

		springForce.clamp(maxForce);
		return springForce;
	}

	inline cVector3d computeAxialSpringForce(cVector3d position, cVector3d & targetPos, cVector3d & targetDir, double & minDist, double & maxDist, double & maxForce, double & kDamping)
	{
		cVector3d displacementToTarget = targetPos - position;
		displacementToTarget = displacementToTarget.projectToVector(targetDir);
		double dist = displacementToTarget.length();
		displacementToTarget.normalize();

		// spring
		double forceMagnitude = lmapd(dist, minDist, maxDist, 0.0, maxForce);
		cVector3d springForce = displacementToTarget * forceMagnitude;

		// speed damping
		cVector3d vel = tool->getDeviceGlobalLinVel().projectToVector(targetDir);;
		springForce -= kDamping * vel * maxForce;

		springForce.clamp(maxForce);
		return springForce;
	}


	HapticLayerContainer::HapticLayerContainer()
	{
		m_lastLayerPenetrated = -1;


		//create a default dataset with skin.

		// add material
		HapticLayer skin = HapticLayer();
		skin.m_restingDepth = 0.0;
		skin.m_frictionForce = 3.0;
		skin.setStiffnessByExponentAndDistance(2.0, 0.005, 7.0);

		m_layerMaterials.push_back(skin);


		//// add data for where the material is
		//std::list <util::NodeGraphTableEntry<unsigned int>> layerList;
		//
		//{// first layer start
		//
		//	util::NodeGraphTableEntry<unsigned int> layer;
		//	float distance = 0.0f;
		//	layer.distanceAlongPath = distance;
		//	layer.t = distance;
		//	layer.val = 0;
		//	layerList.push_back(layer);
		//}
		//
		//{// first layer end
		//
		//	util::NodeGraphTableEntry<unsigned int> layer;
		//	float distance = 1.0f;
		//	layer.distanceAlongPath = distance;
		//	layer.t = distance;
		//	layer.val = 0;
		//	layerList.push_back(layer);
		//}
		//
		//m_layerLUT.m_data.push_back(layerList);
	}

	HapticLayerContainer::~HapticLayerContainer()
	{
	}

	inline cVector3d HapticLayerContainer::computeForces(cVector3d & devicePosition, double forceScalar)
	{
		// what is the displacement parallel to the layers?
		cVector3d displacement = (devicePosition - m_entryPoint);

		double penetrationDepth = displacement.dot(m_entryDirection);

		double netForceMagnitude = 0.0;

		// for each layer, determine if the layer is already being penetrated.
		// If it has, then apply the friction force. If not, then apply the tension force.
		int numLayers = m_layerMaterials.size();
		for (int layerIdx = 0; layerIdx < numLayers; layerIdx++)
		{
			auto layer = &m_layerMaterials[layerIdx];

			if (m_lastLayerPenetrated >= layerIdx)
			{
				// penetrated behaviour
				double force = 0.0; // Joss TODO: compute this force

				// if behind the material's depth
				if (penetrationDepth < layer->m_restingDepth)
				{
					//it is no longer penetrated.
					m_lastLayerPenetrated = max(m_lastLayerPenetrated - 1, -1);
					//any materials after it are also no longer penetrated.
				}

				//PRINTLN("last layer: " << m_lastLayerPenetrated << "| pen depth: " << penetrationDepth)

				netForceMagnitude += force;

				//PRINTLN("inside layer" + layerIdx)
			}
			else
			{
				// not-yet-penetrated behaviour

				double displacement = penetrationDepth - layer->m_restingDepth;

				// determine where the haptic tip is in relation to the layer's rest position in 1D.
				double force = layer->computeTensionForce(displacement);


				// if the force is enough to penetrate it, go through.
				if(force >= layer->m_penetrationThreshold)
				{
					m_lastLayerPenetrated = layerIdx;

					PRINTLN("penetrated layer" << layerIdx)
				}
				else
				{

					//PRINTLN("pushing on layer" << layerIdx)
				}

				netForceMagnitude += force;
			}
		}

		cVector3d outputForce = -m_entryDirection;
		outputForce.normalize();
		outputForce.mul(netForceMagnitude);

		return outputForce;
	}
	HapticLayer::HapticLayer()
	{
		m_stiffnessExponent = 1.0;
		m_frictionForce = 0.1;
		m_penetrationThreshold = 1.0;
		m_displacementPoint = 0.0;
		m_restingDepth = 0.0;
	}
	double HapticLayer::computeTensionForce(const double& displacement)
	{
		double outputForce = 0.0;

		// the layer is receiving pressure
		if (displacement > 0.0)
		{
			outputForce = min(m_stiffness * std::pow(displacement, m_stiffnessExponent), m_penetrationThreshold);
		}

		return outputForce;
	}

	void HapticLayer::setStiffnessByExponentAndDistance(double a_stiffnessExponent, double distance, double a_penetrationThreshold)
	{
		m_stiffnessExponent = a_stiffnessExponent;
		m_penetrationThreshold = a_penetrationThreshold;
		m_stiffness = m_penetrationThreshold / (std::pow(distance, m_stiffnessExponent));
	}

	//inline void HapticLayer::setProperties(const double & a_stiffness, const double & a_stiffnessExponent, const double & a_penetrationThreshold)
	//{
	//	m_stiffness = a_stiffness;
	//	m_stiffnessExponent = a_stiffnessExponent;
	//	m_penetrationThreshold = a_penetrationThreshold;
	//}
}