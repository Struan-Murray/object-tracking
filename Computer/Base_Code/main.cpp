﻿/* Object Tracking code written in C++.
 * Original Author: 	Allesandro Rigola (Windows)
 * Current Author:  	Struan Murray (Linux) (https://github.com/Struan-Murray)
 *
 * Requres OpenCV to be installed. See https://github.com/Struan-Murray/OpenCV-Install
 * for Linux install script.
 */

// Built in libraries

#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <deque>
#include <chrono>
#include <fstream>
#include <thread>

// OpenCV libraries

#include <opencv2/core.hpp> // OpenCV's core functionality, definition of Mat
#include <opencv2/highgui.hpp> // Variation of Qt built into OpenCV, manages GUI
#include <opencv2/imgproc.hpp> // OpenCV's image processing library

using namespace cv;
using namespace std;

// Custom libraries

#include "IntervalCheckTimer.h"
#include "basic_speed_PID.h"
//#include "SerialPort.h" // Microsoft Windows only Serial port manager for interfacing with microcontroller. XWINDOWS

// Enabling of features

#define GUI 1 // Enabling of GUI (Master ON/OFF)
#define MINIMAL_GUI 0 // Runs minimal GUI, less stats on screen

#define SIMPLE_OPTIMISATIONS 0 // Cuts out unnecesary operations
#define EXPENSIVE_OPTIMISATIONS 0 // Cuts out important operations

#define MARKER_TYPE MARKER_STAR // Marker to place on tracked object

#define EROSION_TYPE MORPH_ELLIPSE // Target Erosion Type
const int erosionMagnitude = 6;
#define DILATION_TYPE MORPH_ELLIPSE // Target Dilation Type
const int dilationMagnitude = 16;

// Calculated Globals

bool objectFound = false; // Global marker for found object
bool arduinoConnected = false; // Global marker for Arduino connection
int startProgram = false; // Global wait for user to start program
int counter = 0; // Counter for use with direction calculation

std::string xDirection = "NULL"; // Current X direction of tracked object
std::string yDirection = "NULL"; // Current Y direction of tracked object
int dx{0}, dy{0}; // Current X and Y direction of object numerically

#define SEPERATOR ","

// Globals

float Kp1, Ki1;
float Kp2, Ki2;
float Kp3, Ki3;
int Y_MAX = 256;
int Y_MIN = 0;
int U_MAX = 256;
int U_MIN = 0;
int V_MAX = 256;
int V_MIN = 0;
int YUVMIN = 16; //value dictated by colour space limitation
int YMAX = 235; //value dictated by colour space limitation
int UVMAX = 240; //value dictated by colour space limitation
int found = 0;
int change[2] = { 0, 0 };

string output = "";
int frames_u = 190;
int frames_v = 190;
long inside = 0;
long outside = 0;
int delta2 = 1;
int temp_u1;
int temp_u2;
int temp_v1;
int temp_v2;
int dis_u;
int dis_v;
int lockA;
int U_MIN2;
int U_MAX2;
int V_MIN2;
int V_MAX2;

int alpha, bravo, delta, bamma; //gamma > bamma
int window_base;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;
int a, b, c, d;
int mode2 = 0;
int x_out, y_out;
int radium;
int counthere = 0;
int prima = 1;
basic_speed_PID PID_turn1(0, 0, 0, -10, 10);
basic_speed_PID PID_turn2(0, 0, 0, -10, 10);
basic_speed_PID PID_speed(0, 0, 0, -5 , 5 );
IntervalCheckTimer timer;

/* ----------------------- Main Code Begins ----------------------- */

std::string printFormattedTime(std::chrono::high_resolution_clock::time_point, std::chrono::high_resolution_clock::time_point); // Prints time with appropriate units XSTRUANCHECKED
intmax_t getNanoTime(std::chrono::high_resolution_clock::time_point, std::chrono::high_resolution_clock::time_point); // Returns time in microseconds XSTRUANCHECKED
void createTrackbars(); // Creates window with option to start program XSTRUANCHECKED
void drawObject(int , int , int, Mat&); // Draws a target marker at point specified by inputs to function XSTRUANCHECKED
void morphOps(Mat&); // Takes raw binary image (thresh) and erodes noise, then dilates what remains XSTRUANCHECKED
void displayDirection(int_fast16_t, int_fast16_t, int_fast16_t*, int_fast16_t*); // Calculates direction of object over several frames XSTRUANCHECKED

// TO BE CHECKED

char mapTurn(int); // x-axis XCHANGED from char* to char XARDUINO
char mapTurn2(int); // y-axis XCHANGED from char* to char XARDUINO
char mapMove(int); // XCHANGED from char* to char XARDUINO
void PID_dist(double &Kp,double &Ki,double &Kd, int radium, float range); // Modifies PID values based on distance
int trackFilteredObject(int &x, int &y, int &radius, Mat threshold, Mat &cameraFeed, const int minArea, const int maxArea); // Establishes borders and center of object
void targetAquired(Mat &imgOriginal, Mat &threshold, VideoCapture capWebcam, char charCheckForEscKey);
void checkturn(int x, int &drift_Turn); // x-axis
void checkturn2(int y, int &drift_Turn2); // y-axis
void adjuster_U(Mat imgYUV, int delta, int fine, int interval, int radium, Rect r);
void adjuster_V(Mat imgYUV, int delta, int fine, int interval, int radium, Rect r);
void facendi(Mat imgYUV, Mat imgOriginal, int range, int fine, int x, int y);

int main()
{
	/* External Setup */
	/* Camera Init */

	int_fast16_t cameraCount = 0; // Number of cameras connected to computer.
	VideoCapture capWebcam[64]; // Declare a VideoCapture object and associate to webcam, 0 => use 1st webcam.

	std::cout << "Opening Webcam(s)...\n";

	while(capWebcam[cameraCount].open(cameraCount) == true){cameraCount++;} // Check all webcams connected to computer and assign addresses.

	if(cameraCount < 1)
	{
		perror("STRUAN: Camera not detected, see error file for debugging.\nCOMPUTER");
		return -1;
	}

	std::cout << "\nAVAILABLE CAMERAS\n";
	for(int i = 0; i < 80; i++){std::cout << "-";}
	std::cout << "\n";

	for(int_fast16_t a = 0; a < cameraCount; ++a)
	{
		capWebcam[a].set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
		capWebcam[a].set(CAP_PROP_FRAME_HEIGHT, INT_MAX);
		capWebcam[a].set(CAP_PROP_FRAME_WIDTH, INT_MAX);
		capWebcam[a].set(CAP_PROP_FPS, INT_MAX);

		int32_t fourcc = capWebcam[a].get(CAP_PROP_FOURCC);
		std::string fourcc_str = format("%c%c%c%c", fourcc & 255, (fourcc >> 8) & 255, (fourcc >> 16) & 255, (fourcc >> 24) & 255);

		std::cout << "CAMERA " << a << "\n";
		std::cout << "Maximum settings are as follows:\n";
		std::cout << "FPS: " << capWebcam[a].get(CAP_PROP_FPS) << "\n";
		std::cout << "Resolution: " << capWebcam[a].get(CAP_PROP_FRAME_WIDTH) << "*" << capWebcam[a].get(CAP_PROP_FRAME_HEIGHT) << "\n";
		std::cout << "Camera Format: " << fourcc_str << "\n";

		for(int i = 0; i < 80; i++){std::cout << "-";}
		std::cout << "\n";
	}

	std::cout << "\n";



	/* Local Setup */
	/* Camera Init */

	int_fast16_t selectedCamera = 0; // Camera selected for program to use.

	if(cameraCount > 1)
	{
		std::cout << "Please select which camera to use: ";
		std::cin >> selectedCamera;
		std::cout << "\n";

		if(selectedCamera < 0 || selectedCamera >= cameraCount){selectedCamera = 0;}
		else{}
	}
	else{}

	std::cout << "Camera used = " << selectedCamera << "\n";
	std::cout << "\n";

	const int frameHeight = 480;
	const double aspectRatio = (double)4/(double)3;
	const int frameWidth = (int)(aspectRatio * (double)frameHeight);
	int fps = 120;
	const int minimumObjectArea = frameHeight * frameWidth / 1000;
	const int maximumObjectArea = frameHeight * frameWidth / 1.5;

	if (capWebcam[selectedCamera].isOpened() == true) // Check if VideoCapture object was associated to webcam successfully
	{
		std::cout << "Setting camera properties...\n";

		capWebcam[selectedCamera].set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
		capWebcam[selectedCamera].set(CAP_PROP_FRAME_WIDTH, frameWidth);
		capWebcam[selectedCamera].set(CAP_PROP_FRAME_HEIGHT, frameHeight);
		capWebcam[selectedCamera].set(CAP_PROP_FPS, fps);

		fps = capWebcam[selectedCamera].get(CAP_PROP_FPS);

		int32_t fourcc = capWebcam[selectedCamera].get(CAP_PROP_FOURCC);
		std::string fourcc_str = format("%c%c%c%c", fourcc & 255, (fourcc >> 8) & 255, (fourcc >> 16) & 255, (fourcc >> 24) & 255);

		for(int i = 0; i < 80; i++){std::cout << "-";}
		std::cout << "\n";

		std::cout << "CAMERA " << selectedCamera << " stats have been set as follows:\n";
		std::cout << "FPS: " << capWebcam[selectedCamera].get(CAP_PROP_FPS) << "\n";
		std::cout << "Resolution: " << capWebcam[selectedCamera].get(CAP_PROP_FRAME_WIDTH) << "*" << capWebcam[selectedCamera].get(CAP_PROP_FRAME_HEIGHT) << "\n";
		std::cout << "Camera Format: " << fourcc_str << "\n";
		std::cout << "RGB Flag: " << capWebcam[selectedCamera].get(CAP_PROP_CONVERT_RGB) << "\n";
		std::cout << "Complexity: " << ((double)frameHeight*(double)frameWidth/307200.0) << "\n";

		for(int i = 0; i < 80; i++){std::cout << "-";}
		std::cout << "\n\n";
	}
	else
	{
		perror("STRUAN: Camera failed to open, quitting.\nCOMPUTER");
		return -2;
	}



	/* Log File initialisation */

	ofstream statsFile; // File pointer to contain performance data.
	intmax_t programStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

	std::cout << "Opening performance file...\n";
	statsFile.open("Log_" + std::to_string(programStartTime) + ".csv",ios::out | ios::trunc);

	if (statsFile.is_open() == true) // Check if performance file was opened
	{
		std::cout << "Performace Log file opened.\n\n";
		statsFile << "FPS:" << SEPERATOR << capWebcam[selectedCamera].get(CAP_PROP_FPS) << "\n";
		statsFile << "ResolutionHeight:" << SEPERATOR  << capWebcam[selectedCamera].get(CAP_PROP_FRAME_HEIGHT) << "\n";
		statsFile << "ResolutionWidth:" << SEPERATOR << capWebcam[selectedCamera].get(CAP_PROP_FRAME_WIDTH) << "\n";
		statsFile << "\n";
		statsFile << "ObjectFound" << SEPERATOR << "WebcamRead" << SEPERATOR <<
		"GaussianBlur" << SEPERATOR << "ColourConversion" << SEPERATOR <<
		"SelectMode" << SEPERATOR << "InRangeMorphOpsDisplayDirection" <<
		SEPERATOR << "TrackFilteredObject" << SEPERATOR << "PIDDist" <<
		SEPERATOR << "Facendi" << SEPERATOR << "PutText" << SEPERATOR <<
		"ShowImage" << SEPERATOR << "TimeWaitKey\n";
	}
	else
	{
		perror("STRUAN: File not opened, check filesystem is accessible to program.\nCOMPUTER");
		return -3;
	}



	/* Arduino connection */

	std::cout << "Connecting to arduino...\n";
	//SerialPort arduino("\\\\.\\COM3");

	if (false /*arduino.isConnected()/**/)
	{
		std::cout << "Arduino Connection Established\n\n";
	}
	else
	{
		perror("STRUAN: Arduino not connected. Camera movement is disabled. Port name is likely incorrect, or Arduino is not connected.\nCOMPUTER");
		std::cout << "\n";
	}



	/* History and data aquisition */

	int_fast16_t bufferX[2] = {0,0}; // List of last tracked X coordinates.
	int_fast16_t bufferY[2] = {0,0}; // List of last tracked Y coordinatess.


	double Kp{0.03};
	double Ki{1.0};
	double Kd{0.0};

	int drift_Turn = 0;
	int drift_Turn2 = 0;
	//int drift_Move = 0;      //PID parameters are produced automatically as the programme executes

	PID_turn1.set_gainvals(Kp, Kd, Ki);
	PID_turn2.set_gainvals(Kp, Kd, Ki);

	Mat imgOriginal;
	Mat threshold;
	Mat imgYUV;
	Mat borderImage;

	//bool trackObjects = true;

	int x = 0;
	int y = 0;
	int radius = 0;
	//double ratio = 0;
	char charCheckForEscKey = 0;

	targetAquired(imgOriginal, threshold, capWebcam[selectedCamera], charCheckForEscKey);

	std::cout << "Target found";
	timer.setInterCheck(100);

	namedWindow("Threshold", WINDOW_NORMAL);
	namedWindow("imgOriginal", WINDOW_NORMAL); // CV_WINDOW_NORMAL which allows resizing the window

	auto time_begin = std::chrono::high_resolution_clock::now();
	auto time_start = std::chrono::high_resolution_clock::now();
	auto time_end = std::chrono::high_resolution_clock::now();

	std::string print[12];
	// Main loop

	std::cout << "Starting Main Loop\n";
	while (charCheckForEscKey != '0' && capWebcam[selectedCamera].isOpened())
	{
		time_begin = std::chrono::high_resolution_clock::now();
		time_start = std::chrono::high_resolution_clock::now();

		capWebcam[selectedCamera].read(imgOriginal);

		time_end = std::chrono::high_resolution_clock::now();
		print[0] = std::to_string(getNanoTime(time_start, time_end));

		time_start = std::chrono::high_resolution_clock::now();

		GaussianBlur(imgOriginal, imgOriginal, cv::Size(5, 5), 0);

		time_end = std::chrono::high_resolution_clock::now();
		print[1] = std::to_string(getNanoTime(time_start, time_end));

		time_start = std::chrono::high_resolution_clock::now();

		cvtColor(imgOriginal, imgYUV, COLOR_RGB2YCrCb);

		time_end = std::chrono::high_resolution_clock::now();
		print[2] = std::to_string(getNanoTime(time_start, time_end));

		// Avoids suppression of the intervals
		time_start = std::chrono::high_resolution_clock::now();

		if (U_MAX == 16)
		{
			if (V_MAX == 0)
			{
				mode2 = 1;
				U_MAX = 255;
				V_MAX = 255;
			}
			else
			{
				mode2 = 0;
			}
		}

		time_end = std::chrono::high_resolution_clock::now();
		print[3] = std::to_string(getNanoTime(time_start, time_end));
		time_start = std::chrono::high_resolution_clock::now();

		putText(imgOriginal, " Delta: " + std::to_string(delta2), Point(0, 100), 1, 1, Scalar(0, 0, 255), 2);
		putText(imgOriginal, " Lost frames: " + std::to_string(lockA), Point(0, 130), 1, 1, Scalar(0, 0, 255), 2);

		inRange(imgYUV, cv::Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);

		morphOps(threshold);
		displayDirection(x, y, bufferX, bufferY);

		time_end = std::chrono::high_resolution_clock::now();
		print[4] = std::to_string(getNanoTime(time_start, time_end));

		time_start = std::chrono::high_resolution_clock::now();

		if (trackFilteredObject(x, y, radius, threshold, imgOriginal, minimumObjectArea, maximumObjectArea) == 1)
		{
			lockA++; // keeps count of how many frames tracking has been lost for
		}
		else
		{
			lockA = 0; // when object is found again variable is reset to zero
		}

		time_end = std::chrono::high_resolution_clock::now();
		print[5] = std::to_string(getNanoTime(time_start, time_end));

		time_start = std::chrono::high_resolution_clock::now();

		radium = radius;
		PID_dist(Kp, Ki, Kd, radium, 0.2); // adjusts PID values depending on radium ob object

		time_end = std::chrono::high_resolution_clock::now();
		print[6] = std::to_string(getNanoTime(time_start, time_end));

		time_start = std::chrono::high_resolution_clock::now();

		facendi(imgYUV, imgOriginal, 20, 10, x_out, y_out);

		time_end = std::chrono::high_resolution_clock::now();
		print[7] = std::to_string(getNanoTime(time_start, time_end));

		time_start = std::chrono::high_resolution_clock::now();

		#if MINIMAL == 0
		putText(imgOriginal,"Radius: " + std::to_string(radium), Point(20, 250), 1, 1, Scalar(0, 0, 255), 2);
		putText(imgOriginal, "Kp: " + to_string(Kp1) + " Ki: " + to_string(Ki1) + " Kd: " + to_string(Kd), Point(150, 470), 1, 1, Scalar(0, 0, 255), 2);
		putText(imgOriginal, "X Direction: " + xDirection + " (" + std::to_string(dx) + ")", Point(0, 400), 1, 1, Scalar(0, 0, 255), 2);
		putText(imgOriginal, "Y Direction: " + yDirection + " (" + std::to_string(dy) + ")", Point(0, 420), 1, 1, Scalar(0, 0, 255), 2);
		putText(imgOriginal, "U_MIN: " + std::to_string(U_MIN) + "    V_MIN: " + std::to_string(V_MIN) + "    Y_MIN: " + std::to_string(Y_MIN) , Point(180, 15), 1, 1, Scalar(0, 0, 255), 2);
		putText(imgOriginal, "U_MAX: " + std::to_string(U_MAX) + "    V_MAX: " + std::to_string(V_MAX) + "    Y_MAX: " + std::to_string(Y_MAX), Point(180, 30), 1, 1, Scalar(0, 0, 255), 2);
		//putText(imgOriginal, "In: " + to_string(inside) + " Out: " + to_string(outside), Point(5, 15), 1, 1, Scalar(0, 0, 255), 2);
		//putText(imgOriginal, "Ratio: " + to_string((inside / (outside + delta2))), Point(25, 35), 1, 1, Scalar(0, 0, 255), 2);
		#endif

		time_end = std::chrono::high_resolution_clock::now();
		print[8] = std::to_string(getNanoTime(time_start, time_end));

		time_start = std::chrono::high_resolution_clock::now();

		imshow("imgOriginal", imgOriginal);
		imshow("Threshold", threshold);

		time_end = std::chrono::high_resolution_clock::now();
		print[9] = std::to_string(getNanoTime(time_start, time_end));

		time_start = std::chrono::high_resolution_clock::now();

		if (timer.isMinChekTimeElapsedAndUpdate())
		{
			checkturn(x, drift_Turn);
			//checkDrive(radius, drift_Move);
			checkturn2(y, drift_Turn2);
		}
		charCheckForEscKey = cv::waitKey(10);

		time_end = std::chrono::high_resolution_clock::now();
		print[10] = std::to_string(getNanoTime(time_start, time_end));

		intmax_t frameTime = getNanoTime(time_begin, time_end);

		if(frameTime < 1000000000/fps){std::this_thread::sleep_for(std::chrono::nanoseconds(1000000000/fps - frameTime));}
		else{}

		time_end = std::chrono::high_resolution_clock::now();
		print[11] = std::to_string(getNanoTime(time_begin, time_end));


		std::cout << "FPS " << 1000000000 / getNanoTime(time_begin, time_end) << "\n\n";

		if(statsFile.is_open())
		{
			statsFile << std::to_string(objectFound) << SEPERATOR;
			for(int_fast16_t i = 0; i < 12; ++i)
			{
				statsFile << print[i];
				statsFile << SEPERATOR;
			}
			statsFile << "\n";
		}

	}

	statsFile.close();
	destroyAllWindows();
	std::cout << "Finished\n";
	return 0;
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

void createTrackbars() // Creates window with option to start program XSTRUANCHECKED
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

void displayDirection(int_fast16_t x, int_fast16_t y, int_fast16_t* bufferX, int_fast16_t* bufferY) // Calculates direction of object over several frames XSTRUANCHECKED
{
	if(counter <= 0)
	{
		counter = 1;
		bufferX[0] = x;
		bufferY[0] = y;
	}
	else if (counter >= 3) // Frames to be skipped - 1 (3 = 4 frames)
	{
		counter = 0;
		dx = x - bufferX[0];
		dy = y - bufferY[0];

		if (abs(dx) > 10)
		{
			if (dx > 0){xDirection = "Left";}
			else{xDirection = "Right";}
		}
		else{xDirection = "Null";}

		if (abs(dy) > 10)
		{
			if (dy > 0){yDirection = "Down";}
			else{yDirection = "Up";}
		}
		else{yDirection = "Null";}
	}
	else{counter++;}

	return;
}

// TO BE CHECKED

char mapTurn(int drift)   // x-axis XCHANGED from char* to char
{
	switch (drift)
	{
	case 0:
		return '0';
	case 1:
		return 'a';
	case 2:
		return 'b';
	case 3:
		return 'c';
	case 4:
		return 'd';
	case 5:
		return 'e';
	case 6:
		return 'f';
	case 7:
		return 'g';
	case 8:
		return 'h';
	case 9:
		return 'i';
	case 10:
		return 'j';
	case -1:
		return 'k';
	case -2:
		return 'l';
	case -3:
		return 'm';
	case -4:
		return 'n';
	case -5:
		return 'o';
	case -6:
		return 'p';
	case -7:
		return 'q';
	case -8:
		return 'r';
	case -9:
		return 's';
	case -10:
		return 't';
	default:
		cout << "Print x";
		return 'x';
	}
}

char mapTurn2(int drift)   // y-axis XCHANGED from char* to char
{
	switch (drift)
	{
	case 0:
		return '0';
	case 1:
		return 'A';
	case 2:
		return 'B';
	case 3:
		return 'C';
	case 4:
		return 'D';
	case 5:
		return 'E';
	case 6:
		return 'F';
	case 7:
		return 'G';
	case 8:
		return 'H';
	case 9:
		return 'I';
	case 10:
		return 'J';
	case -1:
		return 'K';
	case -2:
		return 'L';
	case -3:
		return 'M';
	case -4:
		return 'N';
	case -5:
		return 'O';
	case -6:
		return 'P';
	case -7:
		return 'Q';
	case -8:
		return 'R';
	case -9:
		return 'S';
	case -10:
		return 'T';
	default:
		cout << "Print x";
		return 'x';
	}
}

char mapMove(int drift) // XCHANGED from char* to char
{
	switch (drift)
	{
	case 0:
		return 'z';
	case 1:
		return '1';
	case 2:
		return '2';
	case 3:
		return '3';
	case 4:
		return '4';
	case 5:
		return '5';
	case -1:
		return '6';
	case -2:
		return '7';
	case -3:
		return '8';
	case -4:
		return '9';
	case -5:
		return ':';
	default:
		return 'x';
	}
}

/* Changes the response of the PID depending on the size of the tracked object,
 * avoids sudden movements when target is too close and tracks faster when
 * object is far away.*/
void PID_dist(double &Kp,double &Ki,double &Kd, int radium, float range)
{
	Kp2 = Kp + Kp * range;
	Ki2 = Ki + Ki * range;
	Kp3 = Kp - Kp * range;
	Ki3 = Ki - Ki * range;

	if (radium < 20) // most dynamic response when object is small
	{
		Kp1 = Kp2;
		Ki1 = Ki2;
	}
	else if (radium > 100) //least dynamic response when object is big
	{
		Kp1 = Kp3;
		Ki1 = Ki3;
	}
	else //linear response between two intervals
	{
		Kp1 = ((Kp3 - Kp2) * (radium - 20)) / 80 + Kp2;
		Ki1 = ((Ki3 - Ki2) * (radium - 20)) / 80 + Ki2;
	}

	PID_turn1.set_gainvals(Kp1, Kd, Ki1);
	PID_turn2.set_gainvals(Kp1, Kd, Ki1);
}

int trackFilteredObject(int &x, int &y, int &radius, Mat threshold, Mat &cameraFeed, const int minArea, const int maxArea) // Establishes borders and center of object
{
	Mat temp;
	threshold.copyTo(temp);
	vector<vector<Point>> contours; // These two vectors needed for output of findContours
	vector<Vec4i> hierarchy;
	objectFound = false;

	findContours(temp, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE); // Find contours of filtered image using openCV findContours function

	if (hierarchy.size() > 0)
	{
		int numObjects = hierarchy.size();
		if (numObjects < MAX_NUM_OBJECTS) // If number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		{
			for (int index = 0; index >= 0; index = hierarchy[index][0])
			{
				Moments moment = moments((cv::Mat)contours[index]); // Use moments method to find our filtered object
				double area = moment.m00;

				/* If the area is less than 20 px by 20px then it is probably just noise
				 * if the area is larger than 2/3 of the image size, probably just a bad filter
				 * we only want the object with the largest area so we safe a reference area each
				 * iteration and compare it to the area in the next iteration. */
				if (area > minArea && area < maxArea )
				{
					x = moment.m10 / area;
					y = moment.m01 / area;
					objectFound = true;
					radius = sqrt(area / 3.14) *1.15;
					x_out = x;
					y_out = y;
				}
				else{objectFound = false;}
			}

			if (objectFound == true) //let user know you found an object
			{
				putText(cameraFeed, "Tracking Object", Point(0, 65), 2, 1, Scalar(0, 0, 255), 2);
				drawObject(x, y, radius, cameraFeed); //draw object location on screen
			}
		}
		else{putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);}
	}
	if (objectFound == false)
	{
		putText(cameraFeed, "Tracking Lost", Point(0, 65), 2, 1, Scalar(0, 0, 255), 2);
		return 1;
	}
	else
	{
		return 0;
	}
}

void targetAquired(Mat &imgOriginal, Mat &threshold, VideoCapture capWebcam, char charCheckForEscKey)
{
	bool useMorphOps = true;
	Mat imgYUV;

	while (charCheckForEscKey != 27 && capWebcam.isOpened())
	{		// until the Esc key is pressed or webcam connection is lost
		bool blnFrameReadSuccessfully = capWebcam.read(imgOriginal);		// get next frame

		if (!blnFrameReadSuccessfully || imgOriginal.empty())
		{		// if frame not read successfully
			std::cout << "error: frame not read from webcam\n";	// print error message to std out
			break;	// and jump out of while loop
		}

		GaussianBlur(imgOriginal, imgOriginal, Size(5, 5), 0);

		cvtColor(imgOriginal, imgYUV, COLOR_RGB2YUV); //XCHANGE COLOR_RGB2YCrCb > COLOR_RGB2HSV

		Rect r = Rect(160, 160, 320, 160);                       //draws rectangle at start
		rectangle(imgOriginal, r, Scalar(75, 0, 255), 5, 8, 0);
		putText(imgOriginal,"Object identification" , Point(10, 50), 2, 1, Scalar(75, 0, 255), 2);
		namedWindow("imgOriginal", WINDOW_NORMAL);	// note: you can use CV_WINDOW_NORMAL which allows resizing the window
		namedWindow("imgYUV", WINDOW_NORMAL);

		imshow("imgOriginal", imgOriginal);
		imshow("imgYUV", imgYUV);

		Mat borderImage;

		createTrackbars(); //returns startProgram = 1 when button is pressed
		//Begin of calibration function ---------only run once at startup---------------------------------------
		if (startProgram == true)
		{
			c = 320;
			d = 160;
			int old_U_MIN = 0;
			int old_U_MAX = 0;
			int old_V_MIN = 0;
			int old_V_MAX = 0;
			//int old_Y_MIN = 0;
			//int old_Y_MAX = 0;
			float old_max = 0;  //all values up to this one are initialised as 0 and rewritten at first iteration
			int delta = 100;    //to be added to denominator of max function
			int fine = 2;       // the higher, the lower the level of accuracy in calibration mode

			float max = 0;
			inside = 0;
			outside = 0;

			for (U_MIN = YUVMIN; U_MIN <= UVMAX; U_MIN = U_MIN + fine)
			{
				inRange(imgYUV, Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);
				Mat cropedImage = threshold(Rect(160, 160, c, d));
				threshold.copyTo(borderImage);
				rectangle(borderImage, r, Scalar(0, 0, 0), -1, 8, 0);
				inside = countNonZero(cropedImage);
				outside = countNonZero(borderImage);
				max = inside/(outside + delta);
				if (max > old_max)
				{
					old_U_MIN = U_MIN;
					old_max = max;
				}
			}

			U_MIN = old_U_MIN;
			old_max = 0;
			inside = 0;
			outside = 0;


			for (U_MAX = UVMAX; U_MAX >= YUVMIN; U_MAX = U_MAX - fine)
			{
				inRange(imgYUV, Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);
				Mat cropedImage = threshold(Rect(160, 160, c, d));
				threshold.copyTo(borderImage);
				rectangle(borderImage, r, Scalar(0, 0, 0), -1, 8, 0);
				inside = countNonZero(cropedImage);
				outside = countNonZero(borderImage);
				max = inside / (outside + delta);
				if (max > old_max)
				{
					old_U_MAX = U_MAX;
					old_max = max;
				}
			}

			U_MAX = old_U_MAX;
			old_max = 0;
			inside = 0;
			outside = 0;

			for (V_MIN = YUVMIN; V_MIN <= UVMAX; V_MIN = V_MIN + fine)
			{
				inRange(imgYUV, Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);
				Mat cropedImage = threshold(Rect(160, 160, c, d));
				threshold.copyTo(borderImage);
				rectangle(borderImage, r, Scalar(0, 0, 0), -1, 8, 0);
				inside = countNonZero(cropedImage);
				outside = countNonZero(borderImage);
				max = inside / (outside + delta);
				if (max > old_max)
				{
					old_V_MIN = V_MIN;
					old_max = max;
				}
			}

			V_MIN = old_V_MIN;
			old_max = 0;
			inside = 0;
			outside = 0;

			for (V_MAX = UVMAX; V_MAX >= YUVMIN; V_MAX = V_MAX - fine)
			{
				inRange(imgYUV, Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);
				Mat cropedImage = threshold(Rect(160, 160, c, d));
				threshold.copyTo(borderImage);
				rectangle(borderImage, r, Scalar(0, 0, 0), -1, 8, 0);
				inside = countNonZero(cropedImage);
				outside = countNonZero(borderImage);
				max = inside / (outside + delta);
				if (max > old_max)
				{
					old_V_MAX = V_MAX;
					old_max = max;
				}
			}
			V_MAX = old_V_MAX;
			old_max = 0;
			Y_MIN = 0;
			Y_MAX = 255;

			found = 1;  //this allows to move on from calibration to tracking mode

		//end of calibration function ------------------------------------------
		}
		inRange(imgYUV, Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);
		//produces image with the calibrate paramters that have been produced

		//imshow("cropedImage", cropedImage); //Ale
		//imshow("borderImage", borderImage); //Ale
		//end of ale's part

		//Mat Gaussian = threshold.clone();
		//Mat Dilate = threshold.clone();
		//GaussianBlur(Gaussian, Gaussian, Size(5, 5), 0);

		if (useMorphOps)
		{
			morphOps(threshold);
		}
		/*
		Mat Combo = Dilate.clone();
		GaussianBlur(Combo, Combo, Size(5, 5), 0);
		*/
		namedWindow("Threshold", WINDOW_NORMAL);
		// putText(threshold, "Ratio: " + std::to_string(max), Point(0, 350), 1, 1, Scalar(255, 255, 255), 2);
		imshow("Threshold", threshold);
		/*
		namedWindow("Gaussian", CV_WINDOW_AUTOSIZE);
		imshow("Gaussian", Gaussian);

		namedWindow("Dilate", CV_WINDOW_AUTOSIZE);
		imshow("Dilate", Dilate);

		namedWindow("Combo", CV_WINDOW_AUTOSIZE);
		imshow("Combo", Combo);
		*/

		charCheckForEscKey = waitKey(30);			// delay (in ms) and get key press, if any
		if (found == 1)
		{
			destroyAllWindows();
			return;
		}
	}
}

void checkturn(int x, int &drift_Turn)  // x-axis
{
	int old_drift = drift_Turn;
	int change_Drift;

	if (objectFound == false)
	{
		//arduino.writeSerialPort(mapTurn(11), 2);
	}
	if (objectFound == true)
	{
		drift_Turn = PID_turn1.ComputePID_output(320, x);
		change_Drift = drift_Turn - old_drift;
		if (change_Drift != 0)
		{
			//arduino.writeSerialPort(mapTurn(drift_Turn), 2);
		}
	}
}

void checkturn2(int y, int &drift_Turn2)     // y-axis
{
	int old_drift2 = drift_Turn2;
	int change_Drift2;

	if (objectFound == false)
	{
		//arduino.writeSerialPort(mapTurn(11), 2);
	}
	if (objectFound == true)
	{
		drift_Turn2 = PID_turn2.ComputePID_output(240, y);
		change_Drift2 = drift_Turn2 - old_drift2;
		if (change_Drift2 != 0)
		{
			//arduino.writeSerialPort(mapTurn2(drift_Turn2), 2);
		}
	}
}

void adjuster_U(Mat imgYUV, int delta, int fine, int interval, int radium, Rect r)
{
	int half_int = int(interval / 2);
	int U_MIN_low = 0;
	int U_MAX_low = 0;
	int U_MIN_high = 0;
	int U_MAX_high = 0;
	inside = 0;
	outside = 0;
	Mat threshold;
	Mat borderImage;
	//preparing variables to pass
	/*
	if (U_MIN < 0) {
		U_MIN = 0;
	}
	if (V_MIN < 0) {
		V_MIN = 0;
	}
	if (U_MAX > 255) {
		U_MAX = 255;
	}
	if (V_MAX > 255) {
		V_MAX = 255;
	}
	*/

	//dealing with U component
	if ((U_MIN > half_int) && (U_MIN < (UVMAX - half_int)))
	{
		U_MIN_low = U_MIN - half_int;
		U_MIN_high = U_MIN + half_int;
	}
	else
	{
		U_MIN_low = YUVMIN;
		U_MIN_high = interval - 1;
	}

	if ((U_MAX > (UVMAX - interval)))
	{
		U_MAX_low = UVMAX - interval;
		U_MAX_high = UVMAX;
	}
	else
	{
		U_MAX_low = U_MAX - half_int;
		U_MAX_high = U_MAX + half_int;
	}
	/*

	//taking absolute values

	U_MAX_low = abs(U_MAX_low);
	U_MAX_high = abs(U_MAX_high);
	U_MIN_low = abs(U_MIN_low);
	U_MIN_high = abs(U_MIN_high);

	//U Component

	if (U_MAX_low < U_MIN_high) { swap(U_MAX_low, U_MIN_high); }
	if (U_MAX_low < U_MIN_low) { swap(U_MAX_low, U_MIN_low); }
	if (U_MAX_high < U_MIN_high) { swap(U_MAX_high, U_MIN_high); }
	if (U_MAX_high < U_MIN_low) { swap(U_MAX_high, U_MIN_low); }
	*/

	int old_U_MIN = YUVMIN;
	int old_U_MAX = YUVMIN;

	float max = 0;
	float old_max = 0;  //all values up to this one are initialised as 0 and rewritten at first iteration
						// DELTA to be added to denominator of max function
						// FINE the higher the lower the level of accuracy in calibration mode

	if (mode2) // when this mode enters windows are forced to be wide-open and
	{          // as a result of this fine parameter is incresed to reduce computation weight
		U_MAX_low = 0;
		U_MAX_high = 255;
		U_MIN_low = 0;
		U_MIN_high = 255;
		//fine = fine * 2; // allows faster track-back of target
	}

	delta = delta * radium + 1; // delta is computed
	delta2 = delta;

	Mat cropedImage;

	for (U_MAX = U_MAX_low; U_MAX <= U_MAX_high; U_MAX = U_MAX + fine)
	{
		inRange(imgYUV, Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);
		cropedImage = threshold(Rect(a, b, c, d));
		threshold.copyTo(borderImage);
		rectangle(borderImage, r, Scalar(0, 0, 0), -1, 8, 0);
		inside = countNonZero(cropedImage);
		outside = countNonZero(borderImage);
		max = inside / (outside + delta);
		if (max > old_max)
		{
			old_U_MAX = U_MAX;
			old_max = max;
		}
	}
	U_MAX = old_U_MAX;
	old_max = 0;
	inside = 0;
	outside = 0;
	for (U_MIN = U_MIN_low; U_MIN <= U_MIN_high; U_MIN = U_MIN + fine)
	{
		inRange(imgYUV, Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);
		cropedImage = threshold(Rect(a, b, c, d));
		threshold.copyTo(borderImage);
		rectangle(borderImage, r, Scalar(0, 0, 0), -1, 8, 0);
		inside = countNonZero(cropedImage);
		outside = countNonZero(borderImage);
		max = inside / (outside + delta);
		if (max > old_max)
		{
			old_U_MIN = U_MIN;
			old_max = max;
		}
	}

	U_MIN = old_U_MIN;
	old_max = 0;
}

void adjuster_V(Mat imgYUV, int delta, int fine, int interval, int radium, Rect r)
{
	int half_int = int(interval / 2);
	int V_MIN_low = 0;
	int V_MAX_low = 0;
	int V_MIN_high = 0;
	int V_MAX_high = 0;
	Mat threshold;
	Mat borderImage;
	inside = 0;
	outside = 0;

	if ((V_MIN > half_int) && (V_MIN < (UVMAX - half_int)))
	{
		V_MIN_low = V_MIN - half_int;
		V_MIN_high = V_MIN + half_int;
	}
	else
	{
		V_MIN_low = YUVMIN;
		V_MIN_high = interval - 1;
	}
	if (V_MAX >(UVMAX - interval))
	{
		V_MAX_low = UVMAX - interval;
		V_MAX_high = UVMAX;
	}
	else
	{
		V_MAX_low = V_MAX - half_int;
		V_MAX_high = V_MAX + half_int;
	}

	int old_V_MIN = 0;
	int old_V_MAX = 0;

	float max = 0;
	float old_max = 0;  //all values up to this one are initialised as 0 and rewritten at first iteration
							// DELTA to be added to denominator of max function
							// FINE the higher the lower the level of accuracy in calibration mode

	if (mode2)
	{	// when this mode enters windows are forced to be wide-open and
		// as a result of this fine parameter is incresed to reduce computation weight
		V_MAX_low = 0;
		V_MAX_high = 255;
		V_MIN_low = 0;
		V_MIN_high = 255;
		//fine = fine * 2;
	}

	//------------------------------delta is computed---------
	delta = delta * radium  + 1;
	delta2 = delta;
	//---------------------------------------------------------------------------
	Mat cropedImage;

	for (V_MAX = V_MAX_low; V_MAX <= V_MAX_high; V_MAX = V_MAX + fine)
	{
		inRange(imgYUV, Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);
		cropedImage = threshold(Rect(a, b, c, d));
		threshold.copyTo(borderImage);
		rectangle(borderImage, r, Scalar(0, 0, 0), -1, 8, 0);
		inside = countNonZero(cropedImage);
		outside = countNonZero(borderImage);
		max = inside / (outside + delta);
		if (max > old_max)
		{
			old_V_MAX = V_MAX;
			old_max = max;
		}
	}
	V_MAX = old_V_MAX;
	old_max = 0;
	inside = 0;
	outside = 0;

	for (V_MIN = V_MIN_low; V_MIN <= V_MIN_high; V_MIN = V_MIN + fine)
	{
		inRange(imgYUV, Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);
		cropedImage = threshold(Rect(a, b, c, d));
		threshold.copyTo(borderImage);
		rectangle(borderImage, r, Scalar(0, 0, 0), -1, 8, 0);
		inside = countNonZero(cropedImage);
		outside = countNonZero(borderImage);
		max = inside / (outside + delta);
		if (max > old_max)
		{
			old_V_MIN = V_MIN;
			old_max = max;
		}
	}

	V_MIN = old_V_MIN;
	old_max = 0;
	Y_MAX = 255;
	Y_MIN = 0;
	//selctor for mode2, when activated it will force wide open bands and a less fine adjusting
	//not to reduce performace of the runnning programme. Mode 2 is entered only when U and V both collapse.
	//if (Y_MAX == 0) {
}

// groups together and manages all the adjusting functions
void facendi(Mat imgYUV, Mat imgOriginal, int range, int fine, int x, int y)
{
	auto time_start = std::chrono::high_resolution_clock::now();

	int multiplier = 2;
	c = radium * multiplier + window_base;
	if (c > 460)
	{    //limits maximum size of adjusting rectangle
		c = 460;
	}

	d = c;
	a = (640 - c) / 2;
	b = (480 - c) / 2;
	Rect r = Rect(a, b, c, d);
	rectangle(imgOriginal, r, Scalar(0, 0, 255), 5, 8, 0);
	int x_low = a + radium;
	int y_low = b + radium;
	int x_high = a + c - radium ;
	int y_high = b + d - radium;
	int x_now = x  ;
	int y_now = y  ;
	int adj = 0;
	putText(imgOriginal, "X: " + std::to_string(x_low) + "/"+ std::to_string(x_now) + "/" + std::to_string(x_high), Point(10, 300), 1, 1, Scalar(0, 0, 255), 2);
	putText(imgOriginal, "Y: " + std::to_string(y_low) + "/" + std::to_string(y_now) + "/" + std::to_string(y_high), Point(10, 320), 1, 1, Scalar(0, 0, 255), 2);

	if (counthere == 0)
	{
		counthere = 4;
		if ((x_now > x_low) && (x_now < x_high))
		{
			if ((y_now > y_low) && (y_now < y_high))
			{
				adj = 1 ;
			}
		}
	}

	counthere--;

	if (lockA > 3)
	{
		adj = 1;
		window_base = 350;
		fine = 10;
	}
	else
	{
		window_base = 120;
	}
	auto time_end = std::chrono::high_resolution_clock::now();
	std::cout << printFormattedTime(time_start, time_end) << "\n";
	if (adj == 1)
	{
		time_start = std::chrono::high_resolution_clock::now();
		adjuster_U(imgYUV, 1, fine, range, radium, r);
		time_end = std::chrono::high_resolution_clock::now();
		std::cout << "U adjust: " << printFormattedTime(time_start, time_end) << "\n";

		time_start = std::chrono::high_resolution_clock::now();
		adjuster_V(imgYUV, 1, fine, range, radium, r);
		time_end = std::chrono::high_resolution_clock::now();
		std::cout << "V adjust: " << printFormattedTime(time_start, time_end) << "\n";

		time_start = std::chrono::high_resolution_clock::now();
		putText(imgOriginal, "Adjstusting__U", Point(0, 200), 1, 1, Scalar(0, 0, 255), 2);
		putText(imgOriginal, "Adjstusting__V", Point(0, 220), 1, 1, Scalar(0, 0, 255), 2);
		time_end = std::chrono::high_resolution_clock::now();
		std::cout << "Printing: " << printFormattedTime(time_start, time_end) << "\n";
	}
}
