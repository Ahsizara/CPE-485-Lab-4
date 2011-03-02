#pragma once

#include "wirobotsdk.h"
#include "Map.h"
#include "ImageProcessingTools.h"
#include "wirobotsdk.h"
#include "Node.h"

#define NO_CONTROL -32768
#define M_PWM 0
#define M_POSITION 1
#define M_VELOCITY 2
#define cFULL_COUNT 32767
#define cWHOLE_RANGE 1200

const int scale=					1;
const float robotWidth=				scale*0.262f;
const float robotHeight=			scale*0.225f;
const float wheelRadius=			scale*0.085f;
const float wheelWidth=				scale*0.032f;
const float topdim1=				scale*0.07f;
const float wheelCoverdim1=			scale*0.02f;
const float wheelCoverdim2=			scale*0.02f;
const float wheelCoverdim3=			scale*0.11f;
const float wheelCoverdim4=			scale*0.08f;
const float robotGroundClearance=	scale*0.03f;
const float cameraWidth=			scale*0.06f;
const float cameradim1=				scale*0.02f;
const float antennaHeight=			scale*0.3f;
const float antennaRadius=			scale*0.005f;

const int encoderResolution=		1200;
const int encoderMax=				32767;
const float maxVelocity=			0.1f;//0.075f;

const double Kpho =					1;
const double Kalpha =				2;
const double Kbeta =				-0.5;//-1;
const double alphaTrackingAccuracy=	0.05;
const double betaTrackingAccuracy=	0.05;
const double phoTrackingAccuracy=	0.05;
const double pi=					3.1416;
const int bigNumber=					99999;

const int CONTROLLERTYPE_MANUALCONTROL=                 0;
const int CONTROLLERTYPE_POINTTRACKER=                 1;
const int CONTROLLERTYPE_EXPERIMENT=                 2;


const float initialX=				0.0f;
const float initialY=				0.0f;
const float initialT=				0.0f;

const int reflectY					=		-1;
const int reflectT					=		-1;

const int ROBOT_TYPE_SIMULATED				=	0;
const int ROBOT_TYPE_REAL				=	1;

// Particle constants
const float pColorSize=				0.6f;
const float particleBaseRadius=		0.02f;
const float estimateBaseRadius=		0.04f;
const float K_distance=				0.05f;
const float K_angle=				0.1f;
const float K_wheelRandomness=		0.75f;
const int numWeightDivisions=		2;
const int numParticles=				300; //1200;
const int numRandomParticles=		(int)(0.5*numParticles);
const float sonarRadius=			(float)(0.9*robotWidth/2);
const float IRRadius=				(float)(0.6*sonarRadius);
const float IRMaxRange=				70.0f;
const float IRMinRange=				10.0f;
const float sonarMaxRange=			200.0f;
const float sonarMinRange=			1.0f;
const float sonarWeightScale=		5.0f;
const int numXCells=				20;
const int numYCells=				20;
const int maxNumNodes=				5000;


class Robot 
{
// Konstruktion
public:
	Robot();	// Standard-Konstruktor
	Robot(float _x, float _y, float _t);	// Standard-Konstruktor

protected:



// Implementierung
public:
	double x, y, t;
	double x_est, y_est, t_est;
	double x_ost, y_ost, t_ost;
	double x_start, y_start, t_start;
	double x_goal, y_goal, t_goal;
	int controllerType;			
	double desiredX, desiredY,desiredT;
	int robotType;
	Map robotMap;
	bool imageThreshold[imageSizeX*imageSizeY];
	ImageProcessingTools imageProcTools;
	double tiltAngle, zoom;
	BOOL displayParticles, displayNodes, displaySimRobot;

	// MotionPlanner variables
	bool motionPlanRequired;
	Node nodeList[maxNumNodes];
	int numNodes;

	// Localization/Controller variables
	double currentEncoderPulse1;
	double currentEncoderPulse2;
	double lastEncoderPulse1;
	double lastEncoderPulse2;
	double wheelDistanceR;
	double wheelDistanceL;
	double angleTravelled, distanceTravelled;
	double desiredWheelSpeedR, desiredWheelSpeedL;
	double diffEncoder1, diffEncoder2;
	int weightParticleSet[100000];


	// Particle Filter variables
	double sonar1, sonar2, sonar3, IR1, IR2, IR3, IR4, IR5, IR6, IR7;
	typedef struct PARTICLE				//structure for passing to the controlling function
	{
		double x,y,t,w;
	} PARTICLE;
	
	PARTICLE particles[numParticles]; // Used to estimate state
	PARTICLE propagatedParticles[numParticles]; // Used as intermediate particles in filter


	// Motion Planner Variables 
	double samplingCellSizeX, samplingCellSizeY;
	int numOccupiedCells;
	int occupiedCellsList[numXCells*numYCells];
	int numNodesInCell[numXCells*numYCells];
	Node NodesInCells[numXCells*numYCells][100];
	Node trajList[maxNumNodes];
	int trajSize, trajCurrentNode;



	void Initialize();
	void RunControlLoop(CWiRobotSDK* m_MOTSDK_rob);
	void LocalizeRealStateWithOdometry(CWiRobotSDK*	m_MOTSDK_rob);
	void LocalizeEstStateWithParticleFilter(CWiRobotSDK*	m_MOTSDK_rob);
	void MotionPrediction(CWiRobotSDK*	m_MOTSDK_rob);
	void WallPositioning(CWiRobotSDK*	m_MOTSDK_rob);
	void TrackPoint(CWiRobotSDK*	m_MOTSDK_rob);
	void TrackTrajectory(CWiRobotSDK*	m_MOTSDK_rob);
	void MotionPlanner(CWiRobotSDK*	m_MOTSDK_rob);
	void CalculateWeight(int p);
	double RandomGaussian();
	void InitializeParticles();
	void SetStartPos(int p);
	void SetRandomPos(int p);
	double Sgn(double a);
	int GetCellNumber (double x , double y);
	void AddNode (Node n);
	void BuildTraj (Node goalNode);
	boolean makeTraj;
	double calcGaus(double mu,double stdv,double x);


protected:

};

