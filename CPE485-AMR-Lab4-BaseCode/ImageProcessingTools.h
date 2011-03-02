#pragma once

#include "wirobotsdk.h"
#include "Map.h"
#include "Math.h"



const int imageSizeX=				176;
const int imageSizeY=				144;
const int imageSizeC=				3;
const float colorTolerance=			20.0f;
const int heightCalcWindowSize=		2;


class ImageProcessingTools 
{
// Konstruktion
public:
	ImageProcessingTools();	// Standard-Konstruktor

protected:



// Implementierung
public:
	Landmark measuredLandmarks[100];
	int numMeasuredLandmarks;

	void rgb2hsv(COLORREF c1, double* pixelColorHSV);
	void findLandmarks(char* ptr);
	double CalculateBearing(double landmarkCenter);
	double CalculateRange(double landmarkHeight);

protected:

};

