#ifdef FUNCDLL_EXPORT
#define FUNCDLL_API __declspec(dllexport) 
#else
#define FUNCDLL_API __declspec(dllimport) 
#endif


extern "C" {

	class Needle : public cToolCursor
	{
	public:
		Needle(cWorld* a_parentWorld);

		struct SpringProperties
		{
			bool enabled = false;
			cVector3d restPosition;
			double minDist;
			double maxDist;
			double maxForce;

		} springProperties;

		inline cVector3d computeSpringForce();

		void computeInteractionForces() override;
	};

	FUNCDLL_API bool prepareHaptics(double hapticScale, double toolRadius);
	FUNCDLL_API void startHaptics(void);
	FUNCDLL_API void stopHaptics(void);
	void updateHaptics(void);

	FUNCDLL_API void getProxyPosition(double outPosArray[]);
	FUNCDLL_API void getDevicePosition(double outPosArray[]);

	FUNCDLL_API bool isTouching(int objectId);
	FUNCDLL_API bool isButtonPressed(int buttonId);

	FUNCDLL_API void addObject(double objectPos[], double objectScale[], double objectRotation[], double vertPos[][3], double normals[][3], int vertNum, int tris[][3], int triNum);

	FUNCDLL_API void translateObjects(double translation[]);

	// set position of workspace
	FUNCDLL_API void setHapticPosition(double position[]);

	// set orientation of workspace
	FUNCDLL_API void setHapticRotation(double rotation[]);

	FUNCDLL_API void testHaptics(void);

	// set constraint to only allow movement along a specific axis given by a direction vector. passing 0,0,0 will disable the constraint
	//FUNCDLL_API void setAxialConstraint(double direction[]);
	
	FUNCDLL_API void setGlobalForce(double force[]);

	// spring is in local coordinates
	FUNCDLL_API void setSpringProperties(bool enabled, double position[], double minDist, double maxDist, double maxForce);

	// Like Unity, Chai3D uses a right handed coordinate system, but -z x y
	FUNCDLL_API void convertXYZFromCHAI3D(double inputXYZ[]);
	FUNCDLL_API void convertXYZToCHAI3D(double inputXYZ[]);
}

inline double lmapd(float from, float fromMin, float fromMax, float toMin, float toMax);
