#include "stdafx.h"
#include "Robot.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>

using namespace std;

int count = 0;
/************************ CONSTRUCTORS ***********************/
Robot :: Robot (){
	x = initialX;
	y = initialY;
	t = initialT;
	Initialize();
}

Robot :: Robot (float _x, float _y, float _t){
	x = _x;
	y = _y;
	t = _t;
	Initialize();
}

void Robot :: Initialize()
{
	// Set estimated state
	x_est = x_ost = initialX;
	y_est = y_ost = initialY;
	t_est = t_ost = initialT;
						
	// Set desired state
	desiredX = initialX;
	desiredY = initialY;
	desiredT = initialT;

	// Reset Localization Variables
	currentEncoderPulse1 = 0;
	currentEncoderPulse2 = 0;
	lastEncoderPulse1 = 0;
	lastEncoderPulse2 = 0;
	wheelDistanceR = 0;
	wheelDistanceL = 0;


	// Set default x80sdk to simulator mode
	robotType = ROBOT_TYPE_SIMULATED;

	// Stop all experiments by default
	controllerType = CONTROLLERTYPE_MANUALCONTROL;

	// Set random start for particles
	InitializeParticles();

	// Set default to no motionPlanRequired
	motionPlanRequired = false;
	makeTraj = false;


	// Set visual display
	tiltAngle = 25.0;
	zoom = 2.0;
	displayParticles = true;
	displayNodes = true;
	displaySimRobot = true;
}




/************************ MAIN CONTROL LOOP ***********************/


// This is the main control function called from the control loop
// in the RoboticsLabDlg application. This is called at every time
// step.
// Students should choose what type of localization and control 
// method to use. 

void Robot :: RunControlLoop(CWiRobotSDK*	m_MOTSDK_rob)
{
	
	// ****************** Additional Student Code: Start ************

	// Students can select what type of localization and control
	// functions to call here. For lab 1, we just call the function
	// WallPositioning to have the robot maintain a constant distance
	// to the wall (see lab manual).


	// Localize	
	MotionPrediction(m_MOTSDK_rob);
	LocalizeRealStateWithOdometry(m_MOTSDK_rob);
	LocalizeEstStateWithParticleFilter(m_MOTSDK_rob);


	// If using the point tracker, call the function
	if (controllerType == CONTROLLERTYPE_POINTTRACKER)
	{
		
		// Check if we need to create a new trajectory
		if (motionPlanRequired){
			MotionPlanner(m_MOTSDK_rob);
			motionPlanRequired = false;
		}

		//TrackPoint(m_MOTSDK_rob);


		// Follow the trajectory
		makeTraj = true;
		TrackTrajectory(m_MOTSDK_rob);

		// ONLY for lab 1, we call this function. Otherwise, comment it out. 
		//WallPositioning(m_MOTSDK_rob);


	}




	// ****************** Additional Student Code: End   ************

	// Sleep to approximate 20 Hz update rate
	Sleep(50);
}




// This function will set the position of the robot with respect
// to a wall.

void Robot :: WallPositioning(CWiRobotSDK*	m_MOTSDK_rob)
{

	// Read Central ultrasonic range sensor
	sonar2 = 0.01*m_MOTSDK_rob->GetSensorSonar2 (x, y, t);

	// ****************** Additional Student Code: Start ************

	// Put code here to calculated desiredWheelSpeedR and 
	// desiredWheelSpeedL. Make sure the robot does not exceed 
	// maxVelocity!!!!!!!!!!!!




	// ****************** Additional Student Code: End   ************


	// Send Control signals, put negative on left wheel control
	m_MOTSDK_rob->DcMotorVelocityNonTimeCtrAll((short)desiredWheelSpeedR,(short)desiredWheelSpeedL,NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL);
	 
}



/************************ LOCALIZATION ***********************/



// This function will grab the most recent encoder measurements
// from either the simulator or the robot (whichever is activated)
// and use those measurements to predict the RELATIVE forward 
// motion and rotation of the robot. These are referred to as
// distanceTravelled and angleTravelled respectively.

void Robot :: MotionPrediction(CWiRobotSDK*	m_MOTSDK_rob)
{


	// ****************** Additional Student Code: Start ************

	// Put code here to calculated distanceTravelled and angleTravelled.
	// You can set and use variables like diffEncoder1, currentEncoderPulse1,
	// wheelDistanceL, wheelRadius, encoderResolution etc. These are defined
	// in the Robot.h file.

	currentEncoderPulse1 = m_MOTSDK_rob->GetEncoderPulse1();
	currentEncoderPulse2 = m_MOTSDK_rob->GetEncoderPulse2();

	diffEncoder1 = currentEncoderPulse1 - lastEncoderPulse1;
	diffEncoder2 = currentEncoderPulse2 - lastEncoderPulse2;

	if(diffEncoder1 > 15000)
	{
		diffEncoder1-=encoderMax;
	}
	if(diffEncoder1 < -15000)
	{
		diffEncoder1+=encoderMax;
	}
	if(diffEncoder2 > 15000)
	{
		diffEncoder2-=encoderMax;
	}
	if(diffEncoder2 < -15000)
	{
		diffEncoder2+=encoderMax;
	}
	
	lastEncoderPulse1 = currentEncoderPulse1;
	lastEncoderPulse2 = currentEncoderPulse2;

	wheelDistanceL = -(2 * 3.14 * wheelRadius * (diffEncoder1/encoderResolution));
	wheelDistanceR = 2 * 3.14 * wheelRadius * (diffEncoder2/encoderResolution);

	distanceTravelled = (wheelDistanceL + wheelDistanceR) / 2;
	angleTravelled = (wheelDistanceR - wheelDistanceL) / (robotWidth);

	// ****************** Additional Student Code: End   ************

}



// This function will Localize the robot, i.e. set the robot position
// defined by x,y,t using the last position with angleTravelled and
// distance travelled.

void Robot :: LocalizeRealStateWithOdometry(CWiRobotSDK*	m_MOTSDK_rob)
{
	// ****************** Additional Student Code: Start ************

	// Put code here to calculate x,y,t and x_est, y_est, t_est.

	x += distanceTravelled * cos(t + angleTravelled/2);
	y += distanceTravelled * sin(t + angleTravelled/2);
	t += angleTravelled;
	x_ost = x;
	y_ost = y;
	t_ost = t;

	// ****************** Additional Student Code: End   ************

}



// Instead of localizing with just odometry, another lab will use
// the range sensors within a particle filter. Like the odometry 
// localization, this function will also use the angleTravelled and
// distance travelled. 
// Unlike the odometry localization, we will set the state estimates
// here: x_est, y_est, t_est.

void Robot :: LocalizeEstStateWithParticleFilter(CWiRobotSDK*	m_MOTSDK_rob)
{

	// Get current sensor readings in meters
	sonar1 = 0.01*m_MOTSDK_rob->GetSensorSonar1 (x, y, t);	// Left facing at 45 deg
	sonar2 = 0.01*m_MOTSDK_rob->GetSensorSonar2 (x, y, t);	// Forward facing at 0 deg
	sonar3 = 0.01*m_MOTSDK_rob->GetSensorSonar3 (x, y, t);	// Right facing at -45 deg

	IR1 = 0.01*m_MOTSDK_rob->GetSensorIRRange (x, y, t);	// Left Facing at 45 deg
	IR2 = 0.01*m_MOTSDK_rob->GetCustomAD3 (x, y, t);		// Left Facing at 15 deg
	IR3 = 0.01*m_MOTSDK_rob->GetCustomAD4 (x, y, t);		// Right Facing at -15 deg
	IR4 = 0.01*m_MOTSDK_rob->GetCustomAD5 (x, y, t);		// Right Facing at -45 deg
	IR5 = 0.01*m_MOTSDK_rob->GetCustomAD6 (x, y, t);		// Right Facing at -90 deg
	IR6 = 0.01*m_MOTSDK_rob->GetCustomAD7 (x, y, t);		// Backward Facing at 180 deg
	IR7 = 0.01*m_MOTSDK_rob->GetCustomAD8 (x, y, t);		// Left Facing at 90 deg
	
	// ****************** Additional Student Code: Start ************

	// Put code here to calculated x_est, y_est, t_est.
	int i;
	double tempWheelDistL, tempWheelDistR, tempDistTraveled, tempAngleTraveled;
	
	//As long as the wheels are moving, propagate particles
	if(((wheelDistanceL > 0.0001 || wheelDistanceL < -0.0001) && (wheelDistanceR > 0.0001 || wheelDistanceR < -0.0001)))
	{
		//For each particle, move with the robot + random gaussian. Calculate the weight afterwards.
		for(i = 0; i < numParticles; i++)
		{
			//Add the random gaussian
			tempWheelDistL = wheelDistanceL + .005*RandomGaussian();
			tempWheelDistR = wheelDistanceR + .005*RandomGaussian();
			
			//Calculate the distance
			tempDistTraveled = (tempWheelDistL + tempWheelDistR) / 2;
			tempAngleTraveled = (tempWheelDistR - tempWheelDistL) / (robotWidth);
			propagatedParticles[i].x = particles[i].x + tempDistTraveled * cos(particles[i].t + tempAngleTraveled/2);
			propagatedParticles[i].y = particles[i].y + tempDistTraveled * sin(particles[i].t + tempAngleTraveled/2);
			propagatedParticles[i].t = particles[i].t + tempAngleTraveled;
			propagatedParticles[i].w = particles[i].w;

			//Calculate the weight
			CalculateWeight(i);
		}


		vector<int> samplingVector;
		double tot = 0.0;
		int hwp = 0; //heightest weighted particle
		for(i = 0; i < numParticles; i++){
			tot += propagatedParticles[i].w;
		}

		for(i = 0; i < numParticles; i++)
		{
			propagatedParticles[i].w /= tot;
			//samplingVector.assign((int)(propagatedParticles[i].w * 100) + 1,i);
			int eger = (int)(propagatedParticles[i].w * numParticles);
			
			if(eger>10) eger = 10;
			if(eger<1) eger = 1;
			
			for(int j = 0; j < eger; j++)
			{
				samplingVector.push_back(i);
			}
		}
		int samplingSize = samplingVector.size();
		for(i = 0; i < numParticles; i++)
		{
			particles[i].x = propagatedParticles[i].x;
			particles[i].y = propagatedParticles[i].y;
			particles[i].t = propagatedParticles[i].t;
			particles[i].w = propagatedParticles[i].w;
			/*	
			int randomIndex = rand() % samplingSize;
			if(i%10 != 0 && propagatedParticles[i].w != 0)
			{
				particles[i].x = propagatedParticles[samplingVector[randomIndex]].x;
				particles[i].y = propagatedParticles[samplingVector[randomIndex]].y;
				particles[i].t = propagatedParticles[samplingVector[randomIndex]].t;
				particles[i].w = propagatedParticles[samplingVector[randomIndex]].w;
			}
			else
			{
				particles[i].x = propagatedParticles[i].x;
				particles[i].y = propagatedParticles[i].y;
				particles[i].t = propagatedParticles[i].t;
				particles[i].w = propagatedParticles[i].w;
				//SetRandomPos(i);
			}
			*/
		}
		double avg = tot/(double)numParticles;
		int ran = 0;
		for(i=0;i<numParticles;i++){
			if(particles[i].w > particles[hwp].w){
				hwp = i;
			}
			if(particles[i].w < (.5*avg))
			{
				//SetRandomPos(i);
				particles[i].w = avg;
				ran++;
			}
		}
		if(!ran){
			for(i=0;i<numParticles;i++){
				if(particles[i].w<.9*avg){
					//SetRandomPos(i);
					particles[i].w = avg;
				}
			}
		}
		if(particles[hwp].w>avg){
			x_est = particles[hwp].x;
			y_est = particles[hwp].y;
			t_est = particles[hwp].t;
		}
		else{
			x_est = x_ost;
			y_est = y_ost;
			t_est = t_ost;
		}
	}

	// ****************** Additional Student Code: End   ************

}



// Particle filters work by setting the weight associated with each
// particle, according to the difference between the real robot 
// range measurements and the predicted measurements associated 
// with the particle.
// This function should calculate the weight associated with particle p.

void Robot :: CalculateWeight(int p)
{
	double weight = propagatedParticles[p].w;

	// ****************** Additional Student Code: Start ************

	// Put code here to calculated weight. Feel free to use the
	// function robotMap.GetClosestWallDistance from Map.cpp.

	//If the IR sensor is more than max, try to use the sonar, but don't modify the weight significantly
	double diffDistance = 0;
	double frontSonar = sonar2*100;
	double rightIRFront = IR3*100;
	int didthis = 0;
	double mu;
	
	if(propagatedParticles[p].w == NULL)
	{
		propagatedParticles[p].w = 1;
	}
	if (propagatedParticles[p].x > robotMap.maxX || propagatedParticles[p].x < robotMap.minX ||
		propagatedParticles[p].y > robotMap.maxY || propagatedParticles[p].y < robotMap.minY)
	{
		propagatedParticles[p].w = 0;
	}
		
	const double IRRangeAngleFrontFarLeft = 3.1416/2;
	const double IRRangeAngleFrontMidLeft = 3.1416/4;
	const double IRRangeAngleFrontJustLeft = 3.1416/12;
	const double IRRangeAngleFrontJustRight = -3.1416/12;
	const double IRRangeAngleFrontMidRight = -3.1416/4;
	const double IRRangeAngleFrontFarRight = -3.1416/2;

	const double IRRangeAngleBack = 3.1416;


	//weight = propagatedParticles[p].w;
	double IRs[] = {IR1,IR2,IR3,IR4,IR5,IR6,IR7};
	double ang[] = {IRRangeAngleFrontMidLeft,IRRangeAngleFrontJustLeft,IRRangeAngleFrontJustRight,IRRangeAngleFrontMidRight,IRRangeAngleFrontFarRight,IRRangeAngleBack,IRRangeAngleFrontFarLeft};

	for(int i=0;i<7;i++){
		if(IRs[i]*100.0 < IRMaxRange && IRs[i]*100.0 > IRMinRange){
			mu = 105.7*robotMap.GetClosestWallDistance(propagatedParticles[p].x, propagatedParticles[p].y, propagatedParticles[p].t+ang[i]) - 13.446;
			weight *= calcGaus(mu,25.0,IRs[i]*100.0);
			didthis += 1;
		/*
			ofstream afile;
		
			afile.open ("ourLog.txt", ios::app);
			afile << i << "\t" << weight << "\tD:" << calcGaus(mu,25.0,IRs[i]*100.0) << "\tA:" << IRs[i]*100.0 << "\tP:" << mu << endl;
			//afile << p << " " << leftIRFront << endl;
			afile.close ();
		*/
		}
	}
	/*
	if(false)
	{
		if(frontSonar > IRMaxRange && frontSonar < sonarMaxRange)
		{
			//diffDistance = fabs(frontSonar - robotMap.GetClosestWallDistance(propagatedParticles[p].x, propagatedParticles[p].y, propagatedParticles[p].t));
			//if(diffDistance < sonarMaxRange)
			//{
				//weight = propagatedParticles[p].w * 1/diffDistance; //TODO: Gaussian multiplier for weight
				mu = 105.7*robotMap.GetClosestWallDistance(propagatedParticles[p].x, propagatedParticles[p].y, propagatedParticles[p].t)-13.446;
				weight = calcGaus(mu,25.0,frontSonar);
 				didthis = 3;
			//}
			//else
			//{
			//	weight = 0;
			//}
		}
		double PHSonar = sonar1*100.0; //Place Holder
		if(PHSonar > IRMaxRange && PHSonar < sonarMaxRange)
		{
			mu = 105.7*robotMap.GetClosestWallDistance(propagatedParticles[p].x, propagatedParticles[p].y, propagatedParticles[p].t+.785)-13.446;
			weight *= calcGaus(mu,25.0,PHSonar);
			didthis = 3;
		}
		PHSonar = sonar3*100.0; //Place Holder
		if(PHSonar > IRMaxRange && PHSonar < sonarMaxRange)
		{
			mu = 105.7*robotMap.GetClosestWallDistance(propagatedParticles[p].x, propagatedParticles[p].y, propagatedParticles[p].t-.785)-13.446;
			weight *= calcGaus(mu,25.0,PHSonar);
			didthis = 3;
		}
	}
	*/

	//Use [sonar1, sonar2, sonar3, IR1, IR2, IR3, IR4, IR5, IR6, IR7] and GetClosestWallDistance

	// ****************** Additional Student Code: End   ************

	weight *= calcGaus(x_ost,.1,propagatedParticles[p].x);
	weight *= calcGaus(y_ost,.1,propagatedParticles[p].y);
	
	if(p==1){
		didthis = didthis;
	}
	propagatedParticles[p].w = weight;
}



// This function is used to initialize the particle states 
// for particle filtering. It should pick a random location in the 
// environment for each particle by calling SetRandomPos

void Robot :: InitializeParticles() {

	// Set particles in random locations and orientations within environment
	for (int i=0; i< numParticles; i++){

		// Either set the particles at known start position [0 0 0],  
		// or set particles at random locations.
		//SetRandomPos(i);
		SetStartPos(i);
	}
}



// For particle p, this function will select a valid position. It should
// select the position randomly, with equal likelihood of being anywhere 
// in the environement. Should work for rectangular environments to make 
// things easier.

void Robot :: SetRandomPos(int p){

	// ****************** Additional Student Code: Start ************

	// Put code here to calculated the position, orientation of 
	// particles[p]. Feel free to use the rand() function. 
	// It might be helpful to use boundaries defined in the
	// map.cpp file (e.g. robotMap.minX)
	particles[p].x = ((double)rand() / (double)RAND_MAX)*(robotMap.maxX - robotMap.minX) + robotMap.minX;
	particles[p].y = ((double)rand() / (double)RAND_MAX)*(robotMap.maxY - robotMap.minY) + robotMap.minY;
	particles[p].t = ((double)rand() / (double)RAND_MAX)*(2*pi) - pi;
	particles[p].w = 1.0;

	// ****************** Additional Student Code: End   ************
}


// For particle p, this function will select a start predefined position. 

void Robot :: SetStartPos(int p){
	particles[p].x = initialX;
	particles[p].y = initialY;
	particles[p].t = initialT;
	particles[p].w = 1.0;
}



// Random number generator with gaussian distribution
// Often random guassian numbers are used in particle filters. This
// function might help.

double Robot :: RandomGaussian()
{
	double U1, U2, V1, V2;
	double S = 2.0;
	while(S >= 1.0) 
	{
		U1 = double(rand())/double(RAND_MAX);
		U2 = double(rand())/double(RAND_MAX);
		V1 = 2.0*U1-1.0;
		V2 = 2.0*U2-1.0;
		S = pow(V1,2) + pow(V2,2);
	}
	double gauss = V1*sqrt((-2.0*log(S))/S);
	return gauss;
}



// Get the sign of a number
double Robot :: Sgn(double a)
{
	if (a>0)
        return 1.;
	else if (a<0)
        return -1.;
	else
        return 0.;
}




/************************ POINT TRACKING ***********************/


// This function is used to implement Point tracking. It 
// uses the desired state: desiredX, desiredY, desiredT
// to calculate the desired wheel speeds that should be
// sent to the robot or simulator

void Robot :: TrackPoint(CWiRobotSDK*	m_MOTSDK_rob)
{

	bool pointTracked = false;


	// ****************** Additional Student Code: Start ************

	// Put code here to calculated desiredWheelSpeedR and 
	// desiredWheelSpeedL. Make sure the robot does not exceed 
	// maxVelocity!!!!!!!!!!!!

	
	double KKpho = .5;
	double KKalpha = .5;
	double KKbeta = -.2;

	double targetXDistance = desiredX - x_est;
	double targetYDistance = desiredY - y_est;
	double pho;
	double alpha, beta;
	double desiredV, desiredW;	


	//get distance to target
	pho = sqrt( targetXDistance * targetXDistance + targetYDistance * targetYDistance);
	alpha = atan2(targetYDistance, targetXDistance) - t_est;

	if (targetXDistance > -.05 && targetXDistance < .05 && 
		targetYDistance > -.05 && targetYDistance < .05)
		pho = 0;
	
	if(alpha < -pi/2 || alpha > pi/2)
	{
		pho *= -1;
		alpha = atan2(-targetYDistance, -targetXDistance) - t_est;
		beta = desiredT - atan2(-targetYDistance, -targetXDistance);
	} else
	{
		beta = desiredT - atan2(targetYDistance, targetXDistance);
	}
	if (pho < .08 && pho > -.08)
	{
		alpha = 0;
		beta = t_est - desiredT;
	}

	if(alpha < -pi)
		alpha += 2 * pi;
	if(alpha > pi)
		alpha -= 2 * pi;
	if(beta < -pi)
		beta += 2 * pi;
	if(beta > pi)
		beta -= 2 * pi;
	
	desiredV = KKpho * pho;
	
	if(pho > .08 || pho < -.08)
	{
		desiredW = KKalpha * alpha + KKbeta * beta;
	}
	else
	{
		desiredV = 0;
		desiredW = KKbeta * beta;
	}

	

//	desiredV = (desiredV>1)?1:desiredV;
//	desiredV = (desiredV<-1)?1:desiredV;
//	if(desiredW>1){desiredW=1;}
//	if(desiredW<-1){desiredW=-1;}

	desiredWheelSpeedL = (desiredW * robotWidth/2 + desiredV) / wheelRadius;
	desiredWheelSpeedR = (desiredW * robotWidth/2 - desiredV) / wheelRadius;
	desiredWheelSpeedL *= 1200/(2*pi);
	desiredWheelSpeedR *= 1200/(2*pi);
		


	//if(count % 5 == 0)
	//{
	//	afile.open ("ourLog.txt", ios::app);
	//	afile << setprecision(4) << x_est  << "\t" << y_est << "\t" << t_est << "\t" << pho << "\t" << alpha << "\t" << beta << "\n";
	//	afile.close ();
	//}
	//count++;




	// ****************** Additional Student Code: End   ************


	// Check to see if point is already tracked to desired Accuracy
	if (pointTracked) {
			
		// Switch back to manual control
		controllerType = CONTROLLERTYPE_MANUALCONTROL;
			
		// Send Zero Control signals
		m_MOTSDK_rob->DcMotorVelocityNonTimeCtrAll(0,0,NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL);

	// Point not tracked yet, so calculate new control signal
	} else {
		// Send Control signals, put negative on left wheel control
		m_MOTSDK_rob->DcMotorVelocityNonTimeCtrAll((short)desiredWheelSpeedR,(short)desiredWheelSpeedL,NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL);
	} 
	
}



// This function is used to implement trajectory tracking. 
// A straight line trajectory is created that connects the 
// robots current state to the desird position. This function 
// will call the TrackPoint function to force the robot to
// follow the new trajectory.

void Robot :: TrackTrajectory(CWiRobotSDK*	m_MOTSDK_rob)
{

	// ****************** Additional Student Code: Start ************

	// Put code here to call TrackPoint, where the point to be tracked
	// is continually moved along the line that connects the start position
	// defined by x_start, y_start to the desired position desiredX, 
	// desiredY;


	if(makeTraj)
	{
		int i;
		nodeList[0].x = nodeList[0].y = nodeList[0].lastNode = nodeList[0].nodeIndex = 0;
		for(i = 1; nodeList[i].nodeIndex != 0; i++)
			nodeList[i].x = nodeList[i].y = nodeList[i].lastNode = nodeList[i].nodeIndex = 0;

		float targetXDistance = desiredX - x_start;
		float targetYDistance = desiredY - y_start;
		float pho = sqrt( targetXDistance * targetXDistance + targetYDistance * targetYDistance);
		numNodes = (pho/.2)+.5;
		float xDiff = targetXDistance/numNodes;
		float yDiff = targetYDistance/numNodes;

		nodeList[0] = *(new Node(x_start, y_start, 0, 0));
		
		for(i = 1; i < numNodes; i++)
		{
			nodeList[i] = *(new Node(desiredX - xDiff*(numNodes-i), desiredY - yDiff*(numNodes-i), i,i-1 ));
		
		}
		nodeList[numNodes-1] = *(new Node(desiredX, desiredY, numNodes-1, numNodes-2));

		BuildTraj(nodeList[numNodes-1]);
		makeTraj = false;
	}

	// Check to see if we should go to next node
	double distToCurrentNode = sqrt(pow(x_est-trajList[trajCurrentNode].x,2)+pow(y_est-trajList[trajCurrentNode].y,2));
	if (distToCurrentNode< 0.1 && trajCurrentNode+1<trajSize){
		trajCurrentNode ++;
		desiredX = trajList[trajCurrentNode].x;
		desiredY = trajList[trajCurrentNode].y;
		desiredT = 0;		
	}
	if(trajCurrentNode+1==trajSize)
	{
		desiredT = t_goal;
	}

	TrackPoint(m_MOTSDK_rob);



	// ****************** Additional Student Code: End   ************

}




/************************ MOTION PLANNING ***********************/

// This function is used to implement weighted sampling in 
// when randomly selecting nodes to expand from in the PRM.
// The work environment is divided into a grid of cells.
// This function returns the cell number.

int Robot :: GetCellNumber (double x , double y)
{
	return (int) floor( (x-minWorkspaceX)/samplingCellSizeX) + (int)(floor((y-minWorkspaceY)/samplingCellSizeY)*numXCells);
}


// This function is also used to implement weighted sampling in 
// when randomly selecting nodes to expand from in the PRM.
// When new nodes for the PRM are generated, they must be added
// to a variety of memory locations.
// First, the node is stored in a list of nodes specific to a grid
// cell. If this is the first node in that grid cell, the list of 
// occupied cells is updated. Then, the node is stored in a general
// list that keeps track of all nodes for building the final
// trajectory.

void Robot :: AddNode (Node n)
{
	int cellNumber = GetCellNumber(n.x, n.y);
	if (numNodesInCell[cellNumber] == 0){
		occupiedCellsList[numOccupiedCells] = cellNumber;
		numOccupiedCells ++;
	}
	NodesInCells[cellNumber][numNodesInCell[cellNumber]] = n;
	numNodesInCell[cellNumber]++;

	// Add to nodelist
	nodeList[numNodes] = n;
	numNodes++;

	return;
}


// Given the goal node, this function will recursively add the
// parent node to a trajectory until the start node is reached.
// The result is a list of nodes that connect the start node to
// the goal node with collision free edges.

void Robot :: BuildTraj (Node goalNode)
{
	Node tempList[maxNumNodes];
	tempList[0] = goalNode;
	int i=1;

	// Make backwards traj by looking at parent of every child node
	while(tempList[i-1].nodeIndex !=0 )
	{
		tempList[i] = nodeList[tempList[i-1].lastNode];
		i++;
	}

	// Reverse trajectory order
	for (int j=0; j<i; j++){
		trajList[j] = tempList[i-j-1];
	}

	// Set size of trajectory and initialize node counter
	trajSize = i;
	trajCurrentNode = 0;

	return;
}


// This function houses the core motion planner. This function
// will generate a new trajectory when called. Students must 
// add their code here.

void Robot :: MotionPlanner(CWiRobotSDK*	m_MOTSDK_rob)
{


	// Initialize sampling grid cell variables for weighted
	// random selection of nodes to expand.
	samplingCellSizeX = (maxWorkspaceX-minWorkspaceX)/numXCells;
	samplingCellSizeY = (maxWorkspaceY-minWorkspaceY)/numYCells;
	numOccupiedCells = 0;
	for (int i=0; i<numXCells*numYCells; i++)
		numNodesInCell[i] = 0;
	numNodes = 0;


	// ****************** Additional Student Code: Start ************

	// Put code here to expand the PRM until the goal node is reached,
	// or until a max number of iterations is reached.


	// Create and add the start Node


	// Create the goal node


	// Loop until path created
	bool pathFound = false;
	int maxIterations = maxNumNodes;
	int iterations = 0;



	while (iterations < maxIterations && !pathFound) {

		// Choose node for expansion


		// Expand Node


		// Check for collision


			// Add to nodelist


			// Check for goal region

	

		// Increment number of iterations
		iterations ++;
	}


	// ****************** Additional Student Code: End   ************

	// Create the trajectory to follow
	//BuildTraj(goalNode);

}

double Robot :: calcGaus(double mu, double stdv, double x)
{
	//Mu = Mean?
	//STDV = Standard Deviation
	//x = ?
	if (mu>IRMaxRange) return calcGaus(0,stdv,70);
	double frac = 1.0/(sqrt(2.0*pi*stdv*stdv));
	double ex = exp(-(x-mu)*(x-mu)/(2.0*stdv*stdv));
	double mint = frac*ex;
	if(mint<0.0) mint = 1.0;
	return mint;
}





