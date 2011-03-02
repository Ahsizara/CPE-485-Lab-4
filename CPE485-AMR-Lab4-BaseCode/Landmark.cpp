// Landmark.cpp

#include "stdafx.h"
#include "Landmark.h"

Landmark :: Landmark (){
	x = 0;
	y = 0;
	x_est = 0;
	y_est = 0;
	w_image = 0;
	h_image = 0;

}

Landmark :: Landmark (float _x, float _y){
	x = _x;
	y = _y;
	x_est = 0;
	y_est = 0;
	w_image = 0;
	h_image = 0;
			
}