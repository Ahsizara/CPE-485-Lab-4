// Landmark.h


#pragma once


const float coneRadius=				0.1f;
const float coneBaseRadius=			(float) (1.2*coneRadius);
const float coneHeight=				0.3f;

class Landmark 
{
// Konstruktion
public:
	Landmark();	// Standard-Konstruktor
	Landmark(float _x, float _y);	// Standard-Konstruktor

protected:



// Implementierung
public:
	double x, y, t;
	double x_image, y_image, w_image, h_image;
	double x_est, y_est;
	double x_pixel, h_pixel;
	double range, bearing;

protected:

};
