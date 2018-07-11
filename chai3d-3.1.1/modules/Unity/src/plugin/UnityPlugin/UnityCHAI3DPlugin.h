#ifdef FUNCDLL_EXPORT
#define FUNCDLL_API __declspec(dllexport) 
#else
#define FUNCDLL_API __declspec(dllimport) 
#endif

#include "Path.h"

namespace NeedleSimPlugin
{
	//class cPenetrablePoint : public cHapticPoint
	//{
	//	cAlgorithmFingerProxyPuncture
	//
	//	cPenetrablePoint(cGenericTool* a_parentTool);
	//	virtual ~cPenetrablePoint();
	//};

	extern "C"
	{

		struct Spring
		{
			bool enabled = false;
			cVector3d restPosition;
			double minDist;
			double maxDist;
			double maxForce;

		};

		struct AxialSpring
		{
			bool enabled = false;
			cVector3d direction;
			cVector3d position;
			double minDist;
			double maxDist;
			double maxForce;
			double damping;
		};

		// 1-dimensional layer. Has a state
		class HapticLayer
		{
		public:
			HapticLayer(const double& a_stiffness, const double& a_stiffnessExponent, const double& a_maxFrictionForce, const double& a_penetrationThreshold, const double& a_malleability, const double& a_depth);
			HapticLayer();

			~HapticLayer() {}

			// force applied before penetration
			// uses formula: outputForce = stiffness * displacement^stiffnessExponent
			// caps at penetration threshold; will not output any higher than that
			double computeTensionForce(const double& displacement);

			// force applied after penetration. implements a mass-spring model where the contact point can move
			double computeFrictionForce(const double& displacement);

			// the resting depth of the layer, relative to the entry point of the needle
			double m_restingDepth; 

			//inline void setProperties(const double& a_stiffness, const double& a_stiffnessExponent, const double& a_penetrationThreshold);

			// sets the scalar m_stiffness such that at a displacement of {distance} units, the maximum force will be achieved.
			void setStiffnessByExponentAndDistance(double a_stiffnessExponent, double a_distance, double a_penetrationThreshold);

			// called when the layer is penetrated
			void onEnterLayer();

			// called when leaving the layer
			void onExitLayer();


			//// pre-penetration parameters ///////////
			
			// scalar from 0 to inf
			double m_stiffness;

			// behaviour of the force function. e.g. at 1 it is linear. at 2 it is quadratic. at (0, 1) it is logarithmic. at 0 it is constant
			double m_stiffnessExponent; 
			
			double m_penetrationThreshold; // maximum force before penetration
			


			//// post-penetration parameters ///////////
			double m_maxFrictionForce; // max friction force once penetrated
			double m_malleability;

			// where is the layer is in contact with a penetrator such as a needle, relative to its rest position
			double m_displacementPoint;
		};

		class HapticLayerContainer
		{
		public:
			HapticLayerContainer();
			~HapticLayerContainer();

			// create numLayers HapticLayers
			void initialize(unsigned int numLayers);

		public:
			inline cVector3d computeForces(cVector3d& devicePosition, double forceScalar = 1.0);

			//LUT of layers. does not use multiple intervals. Only one.

			// this LUT maps indices of entries in the layer materials container
			//util::Path<unsigned int> m_layerLUT; 
			
			// holds the material properties and states
			std::vector<HapticLayer> m_layerMaterials;

			// set from Unity
			cVector3d m_entryDirection;
			cVector3d m_entryPoint;

			// index of the last layer material that was penetrated. During needle insertion this keeps track of which layers are penetrated and which are not.
			int m_lastLayerPenetrated; // an index of -1 means no layers are being penetrated
			
			// number of layers to iterate through
			int m_numLayersInUse;
		
			bool m_enabled;

		};

		//Custom simulation tool
		class Needle : public cGenericTool
		{

		public:
			Needle(cWorld* a_parentWorld);
			~Needle();

			cHapticPoint* m_hapticTip; // tip of the needle

			Spring springProperties;

			// to constrain movement to a specific axis at a specific location in worldspace
			AxialSpring axialConstraint;

			AxialSpring axialSpring;
			
			bool isForceEngaged();

			void computeInteractionForces() override;
			
		private:

			cVector3d m_lastForceApplied;
		};

		// a spring that moves. one end, the "head", is attached to the haptic tool, the "tail" is attached to a mass that has static and dynamic friction modelled on it.
		class SpringyMembrane : public cGenericEffect
		{
		public:
			SpringyMembrane(cGenericObject* a_parent);
			~SpringyMembrane();

			virtual bool computeForce(const cVector3d &a_toolPos, const cVector3d &a_toolVel, const unsigned int &a_toolID, cVector3d &a_reactionForce);

			bool isPenetrated();

			///////////////////////
			// properties

			Spring spring; // for now, this spring affects both the membrane penetration itself and its properties after penetration

			double m_penetrationThreshold = 7.0; // force required to penetrate the tissue
			double m_resistance = 9.0; // amount of friction force the spring tail can muster. set this to the maximum device force output
			double m_friction_static = 0.2; // how much it takes to move the spring at all 0.5 means half of resistance value
			double m_friction_dynamic = 0.1; // affects how slowly the spring tail catches up to the device position
			double m_springMass = 1.0;

		private:

			bool m_useDynamicFriction = false;

			// for tracking the change from not touching to touching
			bool m_wasInsideLastFrame = false;

			// not the same as m_wasInsideLastFrame
			bool m_membranePenetrated = false;
		};

		FUNCDLL_API bool prepareHaptics(double hapticScale);

		FUNCDLL_API void startHaptics(void);
		FUNCDLL_API void stopHaptics(void);
		void updateHaptics(void);

		FUNCDLL_API void getProxyPosition(double outPosArray[]);
		FUNCDLL_API void getDevicePosition(double outPosArray[]);

		// check if the tool is touching a particular object by ID.			object IDs start at 1 because the haptic tool is the first child of the world (index 0)
		FUNCDLL_API bool isTouching(int objectId);
		FUNCDLL_API bool isButtonPressed(int buttonId);

		FUNCDLL_API void setToolRadius(double a_toolRadius);

		// No removing objects is allowed, since these IDs work by assuming everything is added sequentially as a child of the world, and never removed.
		// return a world ID to the added object (literally just its index in the cWorld children vector).
		// will also break if you try to parent objects using Chai3d's parenting system, since it is assumed all objects are part children of the world.
		FUNCDLL_API int addObject(double objectPos[], double objectScale[], double objectRotation[], double vertPos[][3], double normals[][3], int vertNum, int tris[][3], int triNum);

		FUNCDLL_API int addBoxObject(double objectPos[], double objectScale[], double objectRotation[]);

		FUNCDLL_API void setObjectProperties(int objectID, double stiffness, double friction_static, double friction_dynamic, double viscosity, double penetrationForce);

		FUNCDLL_API void addViscosityEffect(int objectID, double viscosity);
		FUNCDLL_API void addMembraneEffect(int objectID, double a_resistance, double a_friction_static, double a_friction_dynamic, double maxForce, double distanceToMaxForce, double a_springMass, double a_penetrationThreshold);

		// move all objects in the world
		FUNCDLL_API void translateObjects(double translation[]);

		// set position of workspace
		FUNCDLL_API void setHapticPosition(double position[]);

		// set orientation of workspace
		FUNCDLL_API void setHapticRotation(double rotation[]);

		FUNCDLL_API void setGlobalForce(double force[]);
		FUNCDLL_API void setSpringProperties(bool enabled, double position[], double minDist, double maxDist, double maxForce);
		
		// set constraint to only allow movement along a specific axis given by a direction vector. passing 0,0,0 will disable the constraint
		FUNCDLL_API void setNeedleAxialConstraint(bool enabled, double position[], double direction[], double minDist, double maxDist, double maxForce, double damping);
		
		FUNCDLL_API void setNeedleAxialSpring(bool enabled, double position[], double direction[], double minDist, double maxDist, double maxForce, double damping);

		FUNCDLL_API void setHapticEntryPoint(double position[], double direction[]);


		FUNCDLL_API void clearHapticLayersFromPatient();
		FUNCDLL_API void addHapticLayerToPatient(double a_stiffness, double a_stiffnessExponent, double a_maxFrictionForce, double a_penetrationThreshold, double a_resistanceToMovement, double a_depth);

		FUNCDLL_API void setPatientLayersEnabled(bool a_enabled);

		//set the number of layers that are present in the needle's path into the patient
		FUNCDLL_API void setPatientNumLayersToUse(int a_numLayers);

		FUNCDLL_API void setHapticLayerProperties(int layerIndex, double a_stiffness, double a_stiffnessExponent, double a_maxFrictionForce, double a_penetrationThreshold, double a_resistanceToMovement, double a_depth);

		// Like Unity, Chai3D uses a right handed coordinate system, but -z x y
		FUNCDLL_API void convertXYZFromCHAI3D(double inputXYZ[]);
		FUNCDLL_API void convertXYZToCHAI3D(double inputXYZ[]);
	}

	inline double lmapd(const double& from, const double& fromMin, const double& fromMax, const double& toMin, const double& toMax);

	template<typename T, typename R>
	inline R lmap(const T& from, const T& fromMin, const T& fromMax, const R& toMin, const R& toMax)
	{
		T fromAbs = from - fromMin;
		T fromMaxAbs = fromMax - fromMin;
		double normal = fromAbs / fromMaxAbs;
		R toMaxAbs = toMax - toMin;
		R toAbs = toMaxAbs * normal;
		R to = toAbs + toMin;
		return to;
	}

	inline cVector3d computeSpringForce(const cVector3d& position, cVector3d& targetPos, double& minDist, double& maxDist, double& maxForce);

	template<typename T, typename R>
	inline R lerp(T from, T to, double tValue)
	{
		return (1.0 - tValue) * from + (tValue * to);
	}

	template<typename T>
	inline T lerp(T from, T to, double tValue)
	{
		return (1.0 - tValue) * from + (tValue * to);
	}

	inline cVector3d lerpd(const cVector3d& from, const cVector3d& to, double tValue)
	{
		return (1.0 - tValue) * from + (tValue * to);
	}

	// applies perpendicular to direction
	inline cVector3d computeAxialConstraintForce(cVector3d position, cVector3d& targetPos, cVector3d& targetDir, double& minDist, double& maxDist, double& maxForce, double& kDamping);

	// applies forces parallel to direction
	inline cVector3d computeAxialSpringForce(cVector3d position, cVector3d& targetPos, cVector3d& targetDir, double& minDist, double& maxDist, double& maxForce, double& kDamping);

	// applies speed damping effect
	inline cVector3d computeDampingEffect(const double& kDamping);
}