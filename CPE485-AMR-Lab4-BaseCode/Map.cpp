// Map.cpp

#include "stdafx.h"
#include "Math.h"
#include "Map.h"

Map :: Map (){

	// This is hard coding at its worst. Just edit the file to put in
	// segments of the environment your robot is working in. This is
	// used both for visual display and for localization.

	// ****************** Additional Student Code: Start ************
	
	// Change hard code here to change map:

	numMapSegments = 9;
	mapSegmentCorners[0][0][0] = -2.0;//-1.5
	mapSegmentCorners[0][0][1] = -2.0;//-1.5
	mapSegmentCorners[0][1][0] = -2.0;//-1.5
	mapSegmentCorners[0][1][1] = 2.0;//1.5

	mapSegmentCorners[1][0][0] = -2.0;//1.5
	mapSegmentCorners[1][0][1] = 2.0;//0.5
	mapSegmentCorners[1][1][0] = 2.0;//1.5
	mapSegmentCorners[1][1][1] = 2.0;//-1.5

	mapSegmentCorners[2][0][0] = 2.0;//1.0
	mapSegmentCorners[2][0][1] = 2.0;//-0.5
	mapSegmentCorners[2][1][0] = 2.0;//-0.25
	mapSegmentCorners[2][1][1] = -2.0;//-0.5

	mapSegmentCorners[3][0][0] = 2.0;//-0.25
	mapSegmentCorners[3][0][1] = -2.0;//-1.0
	mapSegmentCorners[3][1][0] = -1.2;//-0.25
	mapSegmentCorners[3][1][1] = -2.0;//1.0

	mapSegmentCorners[4][0][0] = -1.2;//-1.5
	mapSegmentCorners[4][0][1] = -2.0;//-1.5
	mapSegmentCorners[4][1][0] = -1.2;//-1.5
	mapSegmentCorners[4][1][1] = 1.2;//1.5

	mapSegmentCorners[5][0][0] = -1.2;//1.5
	mapSegmentCorners[5][0][1] = 1.2;//0.5
	mapSegmentCorners[5][1][0] = 1.2;//1.5
	mapSegmentCorners[5][1][1] = 1.2;//-1.5

	mapSegmentCorners[6][0][0] = 1.2;//1.0
	mapSegmentCorners[6][0][1] = 1.2;//-0.5
	mapSegmentCorners[6][1][0] = 1.2;//-0.25
	mapSegmentCorners[6][1][1] = -1.2;//-0.5

	mapSegmentCorners[7][0][0] = -0.4;//-0.25
	mapSegmentCorners[7][0][1] = -1.2;//-1.0
	mapSegmentCorners[7][1][0] = -0.4;//-0.25
	mapSegmentCorners[7][1][1] = 1.2;//1.0

	mapSegmentCorners[8][0][0] = 0.4;//-0.25
	mapSegmentCorners[8][0][1] = -2.0;//-1.0
	mapSegmentCorners[8][1][0] = 0.4;//-0.25
	mapSegmentCorners[8][1][1] = 0.4;//1.0

	// ****************** Additional Student Code: End   ************


	// Set map parameters
	// These will be useful in your future coding.
	minX = 9999; minY = 9999; maxX=-9999; maxY=-9999;
	for (int i=0; i< numMapSegments; i++){
		
		// Set extreme values
		minX = min(minX, min(mapSegmentCorners[i][0][0],mapSegmentCorners[i][1][0]));
		minY = min(minY, min(mapSegmentCorners[i][0][1],mapSegmentCorners[i][1][1]));
		maxX = max(maxX, max(mapSegmentCorners[i][0][0],mapSegmentCorners[i][1][0]));
		maxY = max(maxY, max(mapSegmentCorners[i][0][1],mapSegmentCorners[i][1][1]));
		
		// Set wall segments to be horizontal
		slopes[i] = (mapSegmentCorners[i][0][1]-mapSegmentCorners[i][1][1])/(0.001+mapSegmentCorners[i][0][0]-mapSegmentCorners[i][1][0]);
		intercepts[i] = mapSegmentCorners[i][0][1] - slopes[i]*mapSegmentCorners[i][0][0];

		// Set wall segment lengths
		segmentSizes[i] = sqrt(pow(mapSegmentCorners[i][0][0]-mapSegmentCorners[i][1][0],2)+pow(mapSegmentCorners[i][0][1]-mapSegmentCorners[i][1][1],2));
	}
}




// This function is used in your particle filter localization lab. Find 
// the range measurement to a segment given the ROBOT POSITION (x, y) and 
// SENSOR ORIENTATION (t)
double Map :: GetWallDistance(double x, double y, double t, int segment){

	// Set wall vars
	double X1 = mapSegmentCorners[segment][0][0];
	double Y1 = mapSegmentCorners[segment][0][1];
	double X2 = mapSegmentCorners[segment][1][0];
	double Y2 = mapSegmentCorners[segment][1][1];
	double dist = 9999;

	//Range t
	if (t>Pi) t -= 2*Pi; else if (t<-Pi) t += 2*Pi;


	// ****************** Additional Student Code: Start ************	
	double tSlope = tan(t);
	
	double xIntercept = (tSlope*x + Y1 - y - slopes[segment]*X1)/(tSlope-slopes[segment]);
	double yIntercept = tSlope*(xIntercept-x) + y;

	double xDiff = x - xIntercept;
	double yDiff = y - yIntercept;

	dist = sqrt(xDiff * xDiff + yDiff * yDiff);

	//ensure intercept is on wall

	//dist from 1 point
	double delta1 = sqrt( (xIntercept - X1)*(xIntercept - X1) + (yIntercept - Y1)*(yIntercept - Y1) );
	
	//dist from other point
	double delta2 = sqrt( (xIntercept - X2)*(xIntercept - X2) + (yIntercept - Y2)*(yIntercept - Y2) );

	//segment length
	double segLen = sqrt( (X1-X2)*(X1-X2) + (Y1-Y2)*(Y1-Y2) );

	if (delta1 > segLen || delta2 > segLen)
		dist = 9999;

	//ensure intercept is infront of us
	if (abs(atan2(yIntercept - y, xIntercept - x)-t) > .01)
		dist = 9999;


	// ****************** Additional Student Code: End   ************

	return dist;
}


// This function is used in particle filter localization to find the
// range to the closest wall segment, for a robot located
// at position x, y with sensor with orientation t.

double Map :: GetClosestWallDistance(double x, double y, double t){

	double minDist = 9999;

	// ****************** Additional Student Code: Start ************

	// Put code here that loops through segments, calling the
	// function GetWallDistance.
	int i;
	double distTemp;
	for(i = 0; i < numMapSegments; i++)
	{
		distTemp = GetWallDistance(x, y, t, i);
		if(distTemp < minDist)
			minDist = distTemp;
	}

	// ****************** Additional Student Code: End   ************

	return minDist;
}

// This function is called from the motion planner. It is
// used to check for collisions between an edge between
// nodes n1 and n2, and a wall segment.
// The function uses an iterative approach by moving along
// the edge a "safe" distance, defined to be the shortest distance 
// to the wall segment, until the end of the edge is reached or 
// a collision occurs.

bool Map :: CollisionFound(Node n1, Node n2, double tol){


	// Check that within boundaries
	if (n2.x > maxWorkspaceX || n2.x < minWorkspaceX || n2.y > maxWorkspaceY || n2.y < minWorkspaceY)
		return true;


	// Check for collision with walls
	double theta = atan2(n2.y-n1.y, n2.x-n1.x);
	double edgeSize = sqrt(pow(n2.y-n1.y, 2)+pow(n2.x-n1.x,2));
	double sinTheta = sin(theta);
	double cosTheta = cos(theta);

	// Loop through segments
	for (int segment=0; segment< numMapSegments; segment++) {

		double distTravelledOnEdge = 0;
		double ex = n1.x, ey = n1.y;
		double distToSegment;
		while (distTravelledOnEdge-tol < edgeSize){
			distToSegment = GetWallDistance(ex, ey, segment, tol, n2.x, n2.y);
			if (distToSegment-tol < 0.05)
				return true;
			ex += cosTheta*distToSegment;
			ey += sinTheta*distToSegment;
			distTravelledOnEdge +=distToSegment;
		}

	}
	return false;
}


// This function will calculate the length of the perpendicular 
// connecting point x,y to the wall segment. If the perpendicular
// does not hit the segment, a large number is returned.

double Map :: GetWallDistance(double x, double y, int segment, double tol, double n2x, double n2y){

	// Set wall vars
	double X1 = mapSegmentCorners[segment][0][0];
	double Y1 = mapSegmentCorners[segment][0][1];
	double X2 = mapSegmentCorners[segment][1][0];
	double Y2 = mapSegmentCorners[segment][1][1];
	double dist = 9999;

	// Put code here to calculated dist.
	// Calculate slope and intercept
	double angleSegmentPerpendicular = Pi/2 + atan((Y2-Y1)/(0.000001+X2-X1));
	double m = tan(angleSegmentPerpendicular);
	double b = y - m*x;

	// Get line intersection
	double x_intersect = (b-intercepts[segment])/(slopes[segment]-m);
	double y_intersect = m*x_intersect + b;

	// Check to see if intersection LIES within segment
	double dist_intersect_corner1 = sqrt(pow(x_intersect-X1,2) + pow(y_intersect-Y1,2));
	double dist_intersect_corner2 = sqrt(pow(x_intersect-X2,2) + pow(y_intersect-Y2,2));
	if (dist_intersect_corner1 <= (segmentSizes[segment]+tol) && dist_intersect_corner2 <= (segmentSizes[segment]+tol) ){
		dist = sqrt(pow(x-x_intersect,2) + pow(y-y_intersect,2));
	}
	
	// Check for distance to corners (for case where no intersection with segment
	double dist_point_corner1 = sqrt(pow(x-X1,2) + pow(y-Y1,2));
	double dist_point_corner2 = sqrt(pow(x-X2,2) + pow(y-Y2,2));
	dist = min(dist, dist_point_corner1); 
	dist = min(dist, dist_point_corner2); 

	return dist;
}


