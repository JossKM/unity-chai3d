#ifdef FUNCDLL_EXPORT
#define FUNCDLL_API __declspec(dllexport) 
#else
#define FUNCDLL_API __declspec(dllimport) 
#endif

namespace NeedleSimPlugin
{

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

		//Custom simulation tool
		class Needle : public cToolCursor
		{
		public:
			Needle(cWorld* a_parentWorld);

			Spring springProperties;

			//double axialConstraintPriority = 50.0;
			//double interactionForcePriority = 50.0;

			// to constrain movement to a specific axis at a specific location in worldspace
			struct AxialConstraint
			{
				bool enabled = false;
				cVector3d direction;
				cVector3d position;
				double minDist;
				double maxDist;
				double maxForce;

			} axialConstraint;

			inline cVector3d computeAxialConstraintForce(cVector3d& targetPos, cVector3d& targetDir, double& minDist, double& maxDist, double& maxForce);


			void computeInteractionForces() override;

			bool isForceEngaged();
			
		private:
			bool m_isPenetrating = false;
		};

		// a spring that moves. the "head" is attached to the haptic tool, the "tail" is attached to a mass that has static and dynamic friction modelled on it.
		class SpringyMembrane : public cGenericEffect
		{
		public:
			SpringyMembrane(cGenericObject* a_parent);
			~SpringyMembrane();

			virtual bool computeForce(const cVector3d &a_toolPos, const cVector3d &a_toolVel, const unsigned int &a_toolID, cVector3d &a_reactionForce);

			Spring spring;

			double penetrationThreshold;
			double m_resistance = 9.0; // amount of friction force the spring tail can muster. set this to the maximum device force output
			double m_friction_static = 0.2; // how much it takes to move the spring at all 0.5 means half of resistance value
			// how slowly the spring tail catches up to the device position
			double m_friction_dynamic = 0.1;
			double m_springMass = 1.0;

			// at 1.0 friction the spring does not move.
			// at 0 friction the spring 

		private:
			bool m_wasInsideLastFrame = false;

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

		// No removing objects is allowed, since these IDs work by assuming everything is added sequentially as a child of the world, and never removed.
		// return a world ID to the added object (literally just its index in the cWorld children vector).
		// will also break if you try to parent objects using Chai3d's parenting system, since it is assumed all objects are part children of the world.
		FUNCDLL_API int addObject(double objectPos[], double objectScale[], double objectRotation[], double vertPos[][3], double normals[][3], int vertNum, int tris[][3], int triNum);

		FUNCDLL_API void setObjectProperties(int objectID, double stiffness, double friction_static, double friction_dynamic, double viscosity);

		FUNCDLL_API void addViscosityEffect(int objectID, double viscosity);
		FUNCDLL_API void addMembraneEffect(int objectID, double a_resistance, double a_friction_static, double a_friction_dynamic, double maxForce, double distanceToMaxForce, double a_springMass);

		// move all objects in the world
		FUNCDLL_API void translateObjects(double translation[]);

		// set position of workspace
		FUNCDLL_API void setHapticPosition(double position[]);

		// set orientation of workspace
		FUNCDLL_API void setHapticRotation(double rotation[]);

		// set constraint to only allow movement along a specific axis given by a direction vector. passing 0,0,0 will disable the constraint
		//FUNCDLL_API void setAxialConstraint(double direction[]);

		FUNCDLL_API void setGlobalForce(double force[]);

		FUNCDLL_API void setSpringProperties(bool enabled, double position[], double minDist, double maxDist, double maxForce);
		FUNCDLL_API void setAxialConstraint(bool enabled, double position[], double direction[], double minDist, double maxDist, double maxForce);

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
	inline R lerpd(T from, T to, double tValue)
	{
		return (1.0 - tValue) * from + (tValue * to);
	}

	inline cVector3d lerpd(const cVector3d& from, const cVector3d& to, double tValue)
	{
		return (1.0 - tValue) * from + (tValue * to);
	}
}