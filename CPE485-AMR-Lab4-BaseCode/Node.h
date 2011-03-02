// Node.h


#pragma once




class Node 
{
// Construction
public:
	Node();	// Standard-Constructor
	Node(double x, double y, int nodeIndex, int lastNode);	

protected:



// Implementing
public:
	double x, y;
	int nodeIndex, lastNode;

protected:

};
