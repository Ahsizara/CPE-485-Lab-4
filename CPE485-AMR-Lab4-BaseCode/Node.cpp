// Node.cpp

#include "stdafx.h"
#include "Math.h"
#include "Node.h"

Node :: Node (){

	x = 0;
	y = 0;
	lastNode = 0;
	nodeIndex = 0;
}

Node :: Node (double _x, double _y, int _nodeIndex, int _lastNode){

	x = _x;
	y = _y;
	nodeIndex = _nodeIndex;
	lastNode = _lastNode;
}



