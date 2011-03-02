#include "stdafx.h"
#include "ImageProcessingTools.h"

ImageProcessingTools :: ImageProcessingTools (){
}


void ImageProcessingTools::findLandmarks(char *ptr)
{
	// Make Histograms for each column
	int i,j, pixelIndex;
	double pixelColorHSV[imageSizeC];
	bool imageThreshold[imageSizeX][imageSizeY];
	int YHistogram[imageSizeX];

	// Loop through each x value adding all pixels that pass threshold
	for ( i = 0; i< imageSizeX; i++) {
		YHistogram[i] = 0;
		for (j = 0; j<imageSizeY-50; j++) {
			pixelIndex = (j *imageSizeX + i) * 3;
			COLORREF pixelColorRGB = RGB(ptr[pixelIndex],ptr[pixelIndex+1],ptr[pixelIndex+2]);
			rgb2hsv(pixelColorRGB, pixelColorHSV);
		
			double colorDiff = fabs(pixelColorHSV[0] - (-8.0f));
			if ((colorDiff <  colorTolerance) && (pixelColorHSV[1] > 0.6f) && (pixelColorHSV[2] > 0.6f)){
				imageThreshold[i][j] = true;
				YHistogram[i]++;
			} else {
				imageThreshold[i][j] = false;
			}
		}
	}



	// Look for blobs using histogram
	int blobWidth[100];
	int blobStart[100];
	double blobCenter[100];
	double blobHeight[100];
	int numRowBlobs = 0;
	bool inBlob = false;
	int blobStartX = -100;
	int blobEndX = 0;

	// Make sure that end is detected
	YHistogram[imageSizeX-1] = 0;

	// For each row, look for on edges and off edges
	for (i = 1; i< imageSizeX; i++) {

		// Check to see if color is yellow
		if (YHistogram[i] > 5){
				
			// Check if start of blob
			if (!inBlob){
					
				// Record edge of blob
				inBlob = true;
				blobStartX = i;
			} 

		// Check to see if end of blob in this column
		} else if (inBlob){

			// Record the end of blob if big enough
			blobEndX = i;
			if (blobEndX - blobStartX > 10) {

				// Update blob arrays
				blobStart[numRowBlobs] = blobStartX;
				blobWidth[numRowBlobs] =  - (blobStartX - blobEndX);
				blobCenter[numRowBlobs] = (blobEndX + blobStartX)/2;
				numRowBlobs++;
			}
			inBlob = false;			
		} 
	}



	// Loop through blobs to get heights
	for (i = 0; i<numRowBlobs; i++){
		blobHeight[i] = 0;
		for (j = max(0,((int)blobCenter[i])-heightCalcWindowSize); j < min(imageSizeX,((int)blobCenter[i])+heightCalcWindowSize); j++){
			blobHeight[i] = blobHeight[i] + YHistogram[j]; 
		}
		blobHeight[i] = blobHeight[i]/(2*heightCalcWindowSize);
	}




	// Create Landmark Measurements
	numMeasuredLandmarks = numRowBlobs;
	for (i = 0; i<numMeasuredLandmarks; i++){
		measuredLandmarks[i].x_pixel = blobCenter[i];
		measuredLandmarks[i].h_pixel = blobHeight[i];
		measuredLandmarks[i].range = CalculateRange(blobHeight[i]);
		measuredLandmarks[i].bearing = CalculateBearing(blobCenter[i]);
		measuredLandmarks[i].x_est = measuredLandmarks[i].bearing;
		measuredLandmarks[i].y_est = measuredLandmarks[i].range;
	}

}

double ImageProcessingTools::CalculateBearing(double landmarkCenter)
{	

	return landmarkCenter;
}


double ImageProcessingTools::CalculateRange(double landmarkHeight)
{	

	return landmarkHeight;
}



void ImageProcessingTools::rgb2hsv(COLORREF c1, double* pixelColorHSV)
{
	double themin,themax,delta;
	double R,G,B;

	R = GetRValue(c1)/255.0f;
	G = GetGValue(c1)/255.0f;
	B = GetBValue(c1)/255.0f;

	themin = min(R,min(G,B));
	themax = max(R,max(G,B));
	delta = themax - themin;
	
	pixelColorHSV[2] = themax;
	pixelColorHSV[1] = 0;
	if (themax > 0)
      pixelColorHSV[1] = delta / themax;
	pixelColorHSV[0] = 0;
   
	if (delta > 0) {
		if (themax == R && themax != G)
			pixelColorHSV[0] += (G - B) / delta;
		if (themax == G && themax != B)
			pixelColorHSV[0] += (2 + (B - R) / delta);
		if (themax == B && themax != R)
			pixelColorHSV[0] += (4 + (R - G) / delta);
      pixelColorHSV[0] *= 60.0;
   }
}
