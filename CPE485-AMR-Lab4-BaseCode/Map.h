// Map.h


#pragma once

#include "Landmark.h"
#include "Node.h"

// Evironment Constants
const int maxNumLandmarks=			10;
const int maxNumMapSegments=		10;
const float wallHeight=				0.4f;
const float minWorkspaceX=			-2.0f;
const float maxWorkspaceX=			2.0f;
const float minWorkspaceY=			-2.0f;
const float maxWorkspaceY=			2.0f;

// Particle Filter Constants
const double Pi=					3.1416;



class Map 
{
// Construction
public:
	Map();	// Standard-Constructor
	Map(bool preCodedMap);	

protected:



// Implementing
public:
	int numLandmarks;
	Landmark landmarks[maxNumLandmarks]; 
	int numMapSegments;
	double mapSegmentCorners[maxNumMapSegments][2][2];
	double slopes[maxNumMapSegments];
	double intercepts[maxNumMapSegments];
	double segmentSizes[maxNumMapSegments];
	double minX, maxX, minY, maxY;

	void AddLandmark(Landmark _landmark);
	double GetWallDistance(double x, double y, double t, int segment);
	double GetWallDistance(double x, double y, int segment, double tol, double n2x, double n2y);
	double GetClosestWallDistance(double x, double y, double t);
	bool CollisionFound(Node n1, Node n2, double tol);

protected:

};
