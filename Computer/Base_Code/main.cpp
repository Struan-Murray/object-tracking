
//#define _SCL_SECURE_NO_WARNINGS
// Libraries
//#pragma comment(lib,"opencv_world330.lib")
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "basic_speed_PID.h"
#include "IntervalCheckTimer.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <deque>
//#include "SerialPort.h"

using namespace cv;
using namespace std;

//Globals
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
int proceed = 0;
int change[2] = { 0, 0 };
int counter;
string direction = "";
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
int  U_MAX2;
int V_MIN2;
int V_MAX2;

int alpha, bravo, delta, bamma; //gamma > bamma
int window_base;
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20 * 20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 1.5;
bool objectFound = false;
int a, b, c, d;
int mode2 = 0; 
int x_out, y_out;
int radium;
int counthere = 0;
int prima = 1;
basic_speed_PID PID_turn(0,0,0,-10,10);
basic_speed_PID PID_turn2(0, 0, 0, -10, 10);

basic_speed_PID PID_speed(0, 0, 0, -5, 5);
IntervalCheckTimer timer;
//SerialPort arduino("\\\\.\\COM3");

///////////////////////////////////////////////////////////////////////////////////////////////////

string intToString(int number) {
	std::stringstream ss;
	ss << number;
	return ss.str();
}


void createTrackbars() {
	//create window for trackbars
	namedWindow("trackbarWindowName", 0);
	//create memory to store trackbar name on window
	char TrackbarName[50];

	sprintf(TrackbarName, "Centered", proceed); //XCHANGE sprintf_s > sprintf

	createTrackbar("Centered", "trackbarWindowName", &proceed, 1, 0);


}

void morphOps(Mat &thresh) {

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(4, 4));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(6, 6));

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);


	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);


}
void drawObject(int x, int y, int radius, Mat &frame) {

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

	//UPDATE:JUNE 18TH, 2013
	//added 'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

	circle(frame, Point(x, y), radius, Scalar(255, 0, 0), 2);
	if (y - 25>0)
		line(frame, Point(x, y), Point(x, y - 25), Scalar(255, 0, 0), 2);
	else line(frame, Point(x, y), Point(x, 0), Scalar(255, 0, 0), 2);
	if (y + 25<FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), Scalar(255, 0, 0), 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(255, 0, 0), 2);
	if (x - 25>0)
		line(frame, Point(x, y), Point(x - 25, y), Scalar(255, 0, 0), 2);
	else line(frame, Point(x, y), Point(0, y), Scalar(255, 0, 0), 2);
	if (x + 25<FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), Scalar(255, 0, 0), 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(255, 0, 0), 2);

	putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(255, 0, 0), 2);

}



int trackFilteredObject(int &x, int &y, int &radius, Mat threshold, Mat &cameraFeed) {

	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	double refArea = 0;
	objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects<MAX_NUM_OBJECTS) {
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea) {
					x = moment.m10 / area;
					y = moment.m01 / area;
					objectFound = true;
					refArea = area;
					radius = sqrt(refArea / 3.14) *1.15;
					x_out = x;
					y_out = y;
				}
				else objectFound = false;


			}
			//let user know you found an object
			if (objectFound == true) {
				putText(cameraFeed, "Tracking Object", Point(0, 65), 2, 1, Scalar(0, 0, 255), 2);
				//draw object location on screen
				drawObject(x, y, radius, cameraFeed);
				
			}
		}
		else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
	}
	if (objectFound == false) {
		putText(cameraFeed, "Tracking Lost", Point(0, 65), 2, 1, Scalar(0, 0, 255), 2);
		return 1;
		
	}
}
void displayDirection(int x, int y, int bufferX[4], int bufferY[4], Mat &imgOriginal)
{
	bufferX[counter] = x;
	bufferY[counter] = y;
	if (counter >= 4)
	{
		counter = 0;
		//cout << endl << endl << endl << "ENTER PRINTING LOOP" << endl << endl << endl;
		int dx = bufferX[3] - bufferX[0];
		int dy = bufferY[3] - bufferY[0];
		putText(imgOriginal, intToString(dx) + "," + intToString(dy), Point(0,350), 1, 1, Scalar(0, 0, 255), 2);
		if (dx > 20)
		{
			direction = "Left";
		}
		if (abs(dx) > 20)
		{
			if (dx > 0)
				direction = "Left";
			else
				direction = "Right";
			return;
		}

		if (abs(dy) > 20)
		{
			if (dy > 0)
				direction = "Down";
			else
				direction = "Up";
			return;
		}
		//cout << direction;


	}
	counter = counter + 1;
}

void targetAquired(Mat &imgOriginal, Mat &threshold, VideoCapture capWebcam, char charCheckForEscKey)
{
	bool useMorphOps = true;
	Mat imgYUV;


	while (charCheckForEscKey != 27 && capWebcam.isOpened()) {		// until the Esc key is pressed or webcam connection is lost

		bool blnFrameReadSuccessfully = capWebcam.read(imgOriginal);		// get next frame



		if (!blnFrameReadSuccessfully || imgOriginal.empty()) {		// if frame not read successfully

			cout << "error: frame not read from webcam\n";		// print error message to std out

			break;													// and jump out of while loop

		}

		GaussianBlur(imgOriginal, imgOriginal, Size(5, 5), 0);

		cvtColor(imgOriginal, imgYUV, COLOR_RGB2YCrCb);

		Rect r = Rect(160, 160, 320, 160);                       //draws rectangle at start 
		rectangle(imgOriginal, r, Scalar(75, 0, 255), 5, 8, 0);
		putText(imgOriginal,"Object identification" , Point(10, 50), 2, 1, Scalar(75, 0, 255), 2);

		namedWindow("imgOriginal", WINDOW_NORMAL);	// note: you can use CV_WINDOW_NORMAL which allows resizing the window
		namedWindow("imgYUV", WINDOW_NORMAL);


		imshow("imgOriginal", imgOriginal);
		imshow("imgYUV", imgYUV);

		Mat borderImage; 
		
		
		//imshow("cropedImage", cropedImage); //Ale
		createTrackbars();   //returns proceed = 1 when button is pressed
		
		
		
		//Begin of calibration function ---------only run once at startup---------------------------------------
		if (proceed == 1) {
			c = 320;
			d = 160;
			int old_U_MIN = 0;
			int old_U_MAX = 0;
			int old_V_MIN = 0;
			int old_V_MAX = 0;
			int old_Y_MIN = 0;
			int old_Y_MAX = 0;
			float old_max = 0;  //all values up to this one are initialised as 0 and rewritten at first iteration
			int delta = 100;    //to be added to denominator of max function 
			int fine = 2;       // the higher, the lower the level of accuracy in calibration mode
			
			float max = 0;
			inside = 0;
			outside = 0;

			for (U_MIN = YUVMIN; U_MIN <= UVMAX; U_MIN = U_MIN + fine) {
				inRange(imgYUV, Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);
				Mat cropedImage = threshold(Rect(160, 160, c, d));   
				threshold.copyTo(borderImage); 
				rectangle(borderImage, r, Scalar(0, 0, 0), -1, 8, 0);  
				inside = countNonZero(cropedImage);
				outside = countNonZero(borderImage);
				max = inside/(outside + delta);
				if (max > old_max) {
					old_U_MIN = U_MIN;
					old_max = max;
				}}

			U_MIN = old_U_MIN;
			old_max = 0;
			inside = 0;
			outside = 0;


			for (U_MAX = UVMAX; U_MAX >= YUVMIN; U_MAX = U_MAX - fine) {
				inRange(imgYUV, Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);
				Mat cropedImage = threshold(Rect(160, 160, c, d));   
				threshold.copyTo(borderImage); 
				rectangle(borderImage, r, Scalar(0, 0, 0), -1, 8, 0);  
				inside = countNonZero(cropedImage);
				outside = countNonZero(borderImage);
				max = inside / (outside + delta);
				if (max > old_max) {
					old_U_MAX = U_MAX;
					old_max = max;
				}}

			U_MAX = old_U_MAX;
			old_max = 0;
			inside = 0;
			outside = 0;

			for (V_MIN = YUVMIN; V_MIN <= UVMAX; V_MIN = V_MIN + fine) {
				inRange(imgYUV, Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);
				Mat cropedImage = threshold(Rect(160, 160, c, d));  
				threshold.copyTo(borderImage); 
				rectangle(borderImage, r, Scalar(0, 0, 0), -1, 8, 0);  
				inside = countNonZero(cropedImage);
				outside = countNonZero(borderImage);
				max = inside / (outside + delta);
				if (max > old_max) {
					old_V_MIN = V_MIN;
					old_max = max;
				}}

			V_MIN = old_V_MIN;
			old_max = 0;
			inside = 0;
			outside = 0;

			for (V_MAX = UVMAX; V_MAX >= YUVMIN; V_MAX = V_MAX - fine) {
				inRange(imgYUV, Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);
				Mat cropedImage = threshold(Rect(160, 160, c, d));   
				threshold.copyTo(borderImage); 
				rectangle(borderImage, r, Scalar(0, 0, 0), -1, 8, 0); 
				inside = countNonZero(cropedImage);
				outside = countNonZero(borderImage);
				max = inside / (outside + delta);
				if (max > old_max) {
					old_V_MAX = V_MAX;
					old_max = max;
				}
			}
			V_MAX = old_V_MAX;
			old_max = 0;
			
			
			Y_MIN = 0;
			Y_MAX = 255;

			found = 1;  //this allows to move on from calibration to tracking mode 

		//end of calibration function -----------------------------------------------------------------------------------------------------

			
		}
		inRange(imgYUV, Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);
		//produces image with the calibrate paramters that have been produced
		
	//	imshow("cropedImage", cropedImage); //Ale
	//	imshow("borderImage", borderImage); //Ale
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
	//	putText(threshold, "Ratio: " + intToString(max), Point(0, 350), 1, 1, Scalar(255, 255, 255), 2);
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

char* mapTurn(int drift)   // x-axis
{

	switch (drift)
	{
	case 0:
		return "0";
	case 1:
		return "a";
	case 2:
		return "b";
	case 3:
		return "c";
	case 4:
		return "d";
	case 5:
		return "e";
	case 6:
		return "f";
	case 7:
		return"g";
	case 8:
		return "h";
	case 9:
		return "i";
	case 10:
		return "j";
	case -1:
		return "k";
	case -2:
		return "l";
	case -3:
		return "m";
	case -4:
		return "n";
	case -5:
		return "o";
	case -6:
		return "p";
	case -7:
		return "q";
	case -8:
		return "r";
	case -9:
		return "s";
	case -10:
		return "t";
	default:
		cout << "Print x";
		return "x";
	}
}

char* mapTurn2(int drift)   // y-axis
{

	switch (drift)
	{
	case 0:
		return "0";
	case 1:
		return "A";
	case 2:
		return "B";
	case 3:
		return "C";
	case 4:
		return "D";
	case 5:
		return "E";
	case 6:
		return "F";
	case 7:
		return"G";
	case 8:
		return "H";
	case 9:
		return "I";
	case 10:
		return "J";
	case -1:
		return "K";
	case -2:
		return "L";
	case -3:
		return "M";
	case -4:
		return "N";
	case -5:
		return "O";
	case -6:
		return "P";
	case -7:
		return "Q";
	case -8:
		return "R";
	case -9:
		return "S";
	case -10:
		return "T";
	default:
		cout << "Print x";
		return "x";
	}
}
char* mapMove(int drift)
{
	switch (drift)
	{
	case 0:
		return "z";
	case 1:
		return "1";
	case 2:
		return "2";
	case 3:
		return "3";
	case 4:
		return "4";
	case 5:
		return "5";
	case -1:
		return "6";
	case -2:
		return "7";
	case -3:
		return "8";
	case -4:
		return "9";
	case -5:
		return ":";
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
		drift_Turn = PID_turn.ComputePID_output(320, x);
		//printf("Moving x by: ");
		//cout << drift_Turn;
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
		//printf("Moving y by: ");
		//cout << drift_Turn2;
		change_Drift2 = drift_Turn2 - old_drift2;
		if (change_Drift2 != 0)
		{
			//arduino.writeSerialPort(mapTurn2(drift_Turn2), 2);
		}
	}
}


void adjuster_U(Mat imgYUV, int delta, int fine, int interval, int radium, int window_base, Rect r) {

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
	if ((U_MIN > half_int) && (U_MIN < (UVMAX - half_int))) {
		U_MIN_low = U_MIN - half_int;
		U_MIN_high = U_MIN + half_int;
	}

	else {
		U_MIN_low = YUVMIN;
		U_MIN_high = interval - 1;
	}

	if ((U_MAX > (UVMAX - interval))) {
		U_MAX_low = UVMAX - interval;
		U_MAX_high = UVMAX;
	}

	else {
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

	if (mode2) {                // when this mode enters windows are forced to be wide-open and 
								// as a result of this fine parameter is incresed to reduce computation weight
		

		U_MAX_low = 0;
		U_MAX_high = 255;
		U_MIN_low = 0;
		U_MIN_high = 255;

		//fine = fine * 2; // allows faster track-back of target 
	}
	//------------------------------delta is computed---------
	delta = delta * radium + 1;
	
	delta2 = delta;
	//---------------------------------------------------------------------------

	Mat cropedImage;


	for (U_MAX = U_MAX_low; U_MAX <= U_MAX_high; U_MAX = U_MAX + fine) {
		inRange(imgYUV, Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);
		cropedImage = threshold(Rect(a, b, c, d));
		threshold.copyTo(borderImage);
		rectangle(borderImage, r, Scalar(0, 0, 0), -1, 8, 0);
		inside = countNonZero(cropedImage);
		outside = countNonZero(borderImage);
		max = inside / (outside + delta);
		if (max > old_max) {
			old_U_MAX = U_MAX;
			old_max = max;
		}
	}
	U_MAX = old_U_MAX;
	old_max = 0;
	inside = 0;
	outside = 0;



	for (U_MIN = U_MIN_low; U_MIN <= U_MIN_high; U_MIN = U_MIN + fine) {
		inRange(imgYUV, Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);
		cropedImage = threshold(Rect(a, b, c, d));
		threshold.copyTo(borderImage);
		rectangle(borderImage, r, Scalar(0, 0, 0), -1, 8, 0);
		inside = countNonZero(cropedImage);
		outside = countNonZero(borderImage);
		max = inside / (outside + delta);
		if (max > old_max) {
			old_U_MIN = U_MIN;
			old_max = max;
		}

	}

	U_MIN = old_U_MIN;
	old_max = 0;
	

	}
	






	void adjuster_V(Mat imgYUV, int delta, int fine, int interval, int radium, int window_base, Rect r) {
		int half_int = int(interval / 2);
		int V_MIN_low = 0;
		int V_MAX_low = 0;
		int V_MIN_high = 0;
		int V_MAX_high = 0;
		Mat threshold;
		Mat borderImage;
		inside = 0;
		outside = 0;
		
		if ((V_MIN > half_int) && (V_MIN < (UVMAX - half_int))) {
			V_MIN_low = V_MIN - half_int;
			V_MIN_high = V_MIN + half_int;
		}

		else {
			V_MIN_low = YUVMIN;
			V_MIN_high = interval - 1;
		}

		if (V_MAX >(UVMAX - interval)) {
			V_MAX_low = UVMAX - interval;
			V_MAX_high = UVMAX;
		}

		else {
			V_MAX_low = V_MAX - half_int;
			V_MAX_high = V_MAX + half_int;
		}

		
		int old_V_MIN = 0;
		int old_V_MAX = 0;

		float max = 0;
		float old_max = 0;  //all values up to this one are initialised as 0 and rewritten at first iteration
							// DELTA to be added to denominator of max function 
							// FINE the higher the lower the level of accuracy in calibration mode

		if (mode2) {                // when this mode enters windows are forced to be wide-open and 
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

		for (V_MAX = V_MAX_low; V_MAX <= V_MAX_high; V_MAX = V_MAX + fine) {
			inRange(imgYUV, Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);
			cropedImage = threshold(Rect(a, b, c, d));
			threshold.copyTo(borderImage);
			rectangle(borderImage, r, Scalar(0, 0, 0), -1, 8, 0);
			inside = countNonZero(cropedImage);
			outside = countNonZero(borderImage);
			max = inside / (outside + delta);
			if (max > old_max) {
				old_V_MAX = V_MAX;
				old_max = max;
			}
		}
		V_MAX = old_V_MAX;
		old_max = 0;
		inside = 0;
		outside = 0;


		for (V_MIN = V_MIN_low; V_MIN <= V_MIN_high; V_MIN = V_MIN + fine) {
			inRange(imgYUV, Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);
			cropedImage = threshold(Rect(a, b, c, d));
			threshold.copyTo(borderImage);
			rectangle(borderImage, r, Scalar(0, 0, 0), -1, 8, 0);
			inside = countNonZero(cropedImage);
			outside = countNonZero(borderImage);
			max = inside / (outside + delta);
			if (max > old_max) {
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
//}
/*


bool check(Mat threshold) {
	float value = 0;
	Mat cropedImage = threshold(Rect(160, 96, 320, 288));
	value = countNonZero(cropedImage);
	putText(threshold, "In whites are: " + to_string(value), Point(0, 200), 1, 1, Scalar(255, 255, 255), 2);
	if (value < 5000)
		return 1;
	else
		return 0;
}




*/


	// groups together and manages all the adjusting functions 
	void facendi(Mat imgYUV, Mat imgOriginal, int range, int fine, int x, int y) {  
		
		int multiplier = 2;
		c = radium * multiplier + window_base;

		if (c > 460) {    //limits maximum size of adjusting rectangle
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
		putText(imgOriginal, "X: " + intToString(x_low) + "/"+ intToString(x_now) + "/" + intToString(x_high), Point(10, 300), 1, 1, Scalar(0, 0, 255), 2);
		putText(imgOriginal, "Y: " + intToString(y_low) + "/" + intToString(y_now) + "/" + intToString(y_high), Point(10, 320), 1, 1, Scalar(0, 0, 255), 2);
		
		if (counthere == 0){
			counthere = 4;
		if ((x_now > x_low) && (x_now < x_high)) {
			if ((y_now > y_low) && (y_now < y_high)) {
				adj = 1 ;
			}}}

		counthere--;

		if (lockA > 3) {
			adj = 1;
			window_base = 350;
			fine = 10;
		}
		else {
			window_base = 120;
		}
		if (adj == 1) {
			adjuster_U(imgYUV, 1, fine, range, radium, window_base, r);
			putText(imgOriginal, "Adjstusting__U", Point(0, 200), 1, 1, Scalar(0, 0, 255), 2);
			adjuster_V(imgYUV, 1, fine, range, radium, window_base, r);
			putText(imgOriginal, "Adjstusting__V", Point(0, 220), 1, 1, Scalar(0, 0, 255), 2);
		}
	}

	//changes the response of the PID depending on the size of the tracked object, avoids sudden 
	//movemetns when target is too close and tracks faster when object is far away.

	void PID_dist(double &Kp,double &Ki,double &Kd, int radium, float range) {  

		Kp2 = Kp + Kp * range;
		Ki2 = Ki + Ki * range;
		Kp3 = Kp - Kp * range;
		Ki3 = Ki - Ki * range;

		if (radium < 20) {    // most dynamic response when object is small
			Kp1 = Kp2;
			Ki1 = Ki2;
		}
		else if (radium > 100) {  //least dynamic response when object is big
			Kp1 = Kp3;
			Ki1 = Ki3;
			} 

		else {                                      //linear response between two intervals
			Kp1 = ((Kp3 - Kp2) * (radium - 20)) / 80 + Kp2;
			Ki1 = ((Ki3 - Ki2) * (radium - 20)) / 80 + Ki2;
			
		}
		

		PID_turn.set_gainvals(Kp1, Kd, Ki1);
		PID_turn2.set_gainvals(Kp1, Kd, Ki1);
	}
	






int main() {
	if (/*arduino.isConnected()*/1) cout << "Connection Established" << endl;
	else cout << "ERROR, check port name";
	VideoCapture capWebcam(0);// declare a VideoCapture object and associate to webcam, 0 => use 1st webcam
	double Ki, Kd, Kp;
	int drift_Turn = 0;
	int drift_Turn2 = 0;
	int drift_Move = 0;      //PID parameters are produced automatically as the programme executes
	 /*
	cout << "Enter the value for the controller: \n";
	cout << "Kp = ";
	cin >> Kp;
	cout << "Ki = ";
	cin >> Ki;
	cout << "Kd = ";
	cin >> Kd;
	*/
	Kp = 0.03;
	Ki = 1;
	Kd = 0;
	PID_turn.set_gainvals(Kp, Kd, Ki);
	PID_turn2.set_gainvals(Kp, Kd, Ki);
	
	
	
	if (capWebcam.isOpened() == false) {				// check if VideoCapture object was associated to webcam successfully
		cout << "error: capWebcam not accessed successfully\n\n";	// if not, print error message to std out
		return(0);														// and exit program
	}


	Mat imgOriginal;
	Mat threshold;
	Mat imgYUV;
	Mat borderImage;


	bool trackObjects = true;

	int x = 0;
	int y = 0;
	int radius = 0;
	int bufferX[4];
	int bufferY[4];
	float ratio = 0;
	char charCheckForEscKey = 0;

	targetAquired(imgOriginal, threshold, capWebcam, charCheckForEscKey);

	cout << "Target found";
	timer.setInterCheck(100);

	while (charCheckForEscKey != 27 && capWebcam.isOpened())
	{
		capWebcam.read(imgOriginal);
		GaussianBlur(imgOriginal, imgOriginal, cv::Size(5, 5), 0);
		cvtColor(imgOriginal, imgYUV, COLOR_RGB2YCrCb);
		namedWindow("imgOriginal", WINDOW_NORMAL);	// note: you can use CV_WINDOW_NORMAL which allows resizing the window
	
		// Avoids suppression of the intervals
		
		if (U_MAX == 16) {
			if (V_MAX == 0) {
				mode2 = 1;
				U_MAX = 255;
				V_MAX = 255;
			
			}
			else {
				mode2 = 0;
			}
		}

		
		putText(imgOriginal, " Delta: " + intToString(delta2), Point(0, 100), 1, 1, Scalar(0, 0, 255), 2);
		putText(imgOriginal, " Lost frames: " + intToString(lockA), Point(0, 130), 1, 1, Scalar(0, 0, 255), 2);
		
		
		inRange(imgYUV, cv::Scalar(Y_MIN, U_MIN, V_MIN), cv::Scalar(Y_MAX, U_MAX, V_MAX), threshold);


	    morphOps(threshold);
		displayDirection(x, y, bufferX, bufferY, imgOriginal);
		namedWindow("Threshold", WINDOW_NORMAL);

		if (trackFilteredObject(x, y, radius, threshold, imgOriginal) == 1) {
			lockA++;  // keeps count of how many frames tracking has been lost for
		}
		else {
			lockA = 0;  // when object is found again variable is reset to zero 
		}
		
		radium = radius; 
		putText(imgOriginal,"Radius: " + intToString(radium), Point(20, 250), 1, 1, Scalar(0, 0, 255), 2);
		PID_dist(Kp, Ki, Kd, radium, 0.2);  //adjusts PID values depending on radium ob object 
		putText(imgOriginal, "Kp: " + to_string(Kp1) + " Ki: " + to_string(Ki1) + " Kd: " + to_string(Kd), Point(150, 470), 1, 1, Scalar(0, 0, 255), 2);
		facendi(imgYUV, imgOriginal, 20, 5, x_out, y_out);

		putText(imgOriginal, direction, Point(0, 400), 1, 1, Scalar(0, 0, 255), 2);
		putText(imgOriginal, " U_MIN: " + intToString(U_MIN) + "    V_MIN: " + intToString(V_MIN) + "    Y_MIN: " + intToString(Y_MIN) , Point(180, 15), 1, 1, Scalar(0, 0, 255), 2);
		putText(imgOriginal, " U_MAX: " + intToString(U_MAX) + "    V_MAX: " + intToString(V_MAX) + "    Y_MAX: " + intToString(Y_MAX), Point(180, 30), 1, 1, Scalar(0, 0, 255), 2);
		//putText(imgOriginal, "In: " + to_string(inside) + " Out: " + to_string(outside), Point(5, 15), 1, 1, Scalar(0, 0, 255), 2);
		//putText(imgOriginal, "Ratio: " + to_string((inside / (outside + delta2))), Point(25, 35), 1, 1, Scalar(0, 0, 255), 2);
		
		imshow("imgOriginal", imgOriginal);
		imshow("Threshold", threshold);
		if (timer.isMinChekTimeElapsedAndUpdate())
		{
			checkturn(x, drift_Turn);
			//checkDrive(radius, drift_Move);
			checkturn2(y, drift_Turn2);
		}
		charCheckForEscKey = cv::waitKey(30);
		//cout << x << " , " << y << endl;
		//cout << radius << endl;
	}
	// end while
	return (0);

}  


