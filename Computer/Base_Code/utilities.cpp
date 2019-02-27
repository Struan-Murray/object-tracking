#include "imageTrackingGlobals.h"

#include <opencv2/core.hpp> // OpenCV's core functionality, definition of Mat
#include <opencv2/highgui.hpp> // Variation of Qt built into OpenCV, manages GUI
#include <opencv2/imgproc.hpp> // OpenCV's image processing library

#include <chrono>
#include <iostream>
#include <string>

using namespace cv;

bool confirm()
{
	std::string choice{""};

	std::cin >> choice;
	std::cin.ignore();

	if(choice.at(0)=='Y' || choice.at(0)=='y' || choice.at(0)=='J' || choice.at(0)=='j' || choice.at(0)=='1' || choice.at(0)=='S' || choice.at(0)=='s' || choice.at(0)=='C' || choice.at(0)=='c')
	{
		return true;
	}
	else{return false;}

	return false;
}

std::string printFormattedTime(std::chrono::high_resolution_clock::time_point time_start, std::chrono::high_resolution_clock::time_point time_end) // XSTRUANCHECKED
{
	intmax_t a = std::chrono::duration_cast<std::chrono::nanoseconds>(time_end-time_start).count();
	if(a == 0){return "CLOCK ERROR (Reported 0 Nanoseconds)";}
	else if(a == 1){return std::to_string(a) + " Nanosecond";}
	else if(a <= 9999){return std::to_string(a) + " Nanoseconds";}
	else if(a <= 9999999){return std::to_string(a/1000) + " Microseconds";}
	else if(a <= 9999999999){return std::to_string(a/1000000) + " Milliseconds";}
	else{return std::to_string(a/1000000000) + " Seconds";}
	return "That's a nasty error";
}

intmax_t getNanoTime(std::chrono::high_resolution_clock::time_point time_start, std::chrono::high_resolution_clock::time_point time_end) // XSTRUANCHECKED
{
	intmax_t a = std::chrono::duration_cast<std::chrono::nanoseconds>(time_end-time_start).count();
	if(a == 0){return -1;}
	else{return a;}
}

void createTrackbars(int& startProgram) // Creates window with option to start program XSTRUANCHECKED
{
	// Create memory to store trackbar and trackbar window name
	std::string TrackbarWindowName{"Confirm Object Centered"};
	std::string TrackbarName{"Object Centered"};

	// Create window for trackbars
	namedWindow(TrackbarWindowName, 0);

	// Place trackbar in window, store trackbar value in variable 'proceed'
	// Trackbar value is between 0 and 1
	createTrackbar(TrackbarName, TrackbarWindowName, &startProgram, 1, 0);
}

void drawObject(int x, int y, int radius, Mat &frame) // Draws a target marker at point specified by inputs to function XSTRUANCHECKED
{
	#if GUI == 1
	// Draws a centrepoint around x,y with diameter equal to radius (Half the size of the circle)
	drawMarker(frame, Point(x, y), Scalar(255, 0, 0), MARKER_TYPE, radius, (radius/50)+1);

	#if MINIMAL == 0
	// Draws a circle around x,y with radius
	circle(frame, Point(x, y), radius, Scalar(255, 0, 0), 2);
	// Displays current co-ordinates of the object next to marker
	putText(frame, std::to_string(x) + "," + std::to_string(y), Point(x, y + 30), 1, 1, Scalar(255, 0, 0), 2);
	#endif
	#endif
}

void morphOps(Mat &thresh) // Takes raw binary image (thresh) and erodes noise, then dilates what remains XSTRUANCHECKED
{
	// Create element structure that will erode the binary image.
	Mat opElement = getStructuringElement(EROSION_TYPE, Size(erosionMagnitude, erosionMagnitude));

	erode(thresh, thresh, opElement); // Erode binary image

	#if SIMPLE_OPTIMISATIONS == 0
	// Update element structure to dilate image
	opElement = getStructuringElement(DILATION_TYPE, Size(dilationMagnitude, dilationMagnitude));
	#endif

	dilate(thresh, thresh, opElement); // Dilate binary image
}
