#include "selectCamera.h"
#include "imageTrackingGlobals.h"
#include <opencv2/videoio.hpp>
#include <iostream>

using namespace cv;

int selectCamera(VideoCapture& cameraReturned, int& fps)
{
	VideoCapture cameraChecked[64]; // Declare a VideoCapture object and associate to webcam, 0 => use 1st webcam.
	int cameraCount = 0; // Number of cameras connected to computer.
	int selectedCamera = 0; // Camera selected for program to use.

	std::cout << "Opening Webcam(s)...\n";

	while(cameraChecked[cameraCount].open(cameraCount) == true){cameraCount++;} // Check all webcams connected to computer and assign addresses.

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
		cameraChecked[a].set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
		cameraChecked[a].set(CAP_PROP_FRAME_HEIGHT, INT_MAX);
		cameraChecked[a].set(CAP_PROP_FRAME_WIDTH, INT_MAX);
		cameraChecked[a].set(CAP_PROP_FPS, INT_MAX);

		int32_t fourcc = cameraChecked[a].get(CAP_PROP_FOURCC);
		std::string fourcc_str = format("%c%c%c%c", fourcc & 255, (fourcc >> 8) & 255, (fourcc >> 16) & 255, (fourcc >> 24) & 255);

		std::cout << "CAMERA " << a << "\n";
		std::cout << "Maximum settings are as follows:\n";
		std::cout << "FPS: " << cameraChecked[a].get(CAP_PROP_FPS) << "\n";
		std::cout << "Resolution: " << cameraChecked[a].get(CAP_PROP_FRAME_WIDTH) << "*" << cameraChecked[a].get(CAP_PROP_FRAME_HEIGHT) << "\n";
		std::cout << "Camera Format: " << fourcc_str << "\n";

		for(int i = 0; i < 80; i++){std::cout << "-";}
		std::cout << "\n";
	}

	std::cout << "\n";

	if(cameraCount > 1)
	{
		std::cout << "Please select which camera to use: ";
		std::cin >> selectedCamera;
		std::cin.ignore();
		std::cout << "\n";

		if(selectedCamera < 0 || selectedCamera >= cameraCount){selectedCamera = 0;}
		else{}
	}
	else{}

	cameraReturned = cameraChecked[selectedCamera];

	std::cout << "Camera used = " << selectedCamera << "\n";
	std::cout << "\n";

	if (cameraReturned.isOpened() == true) // Check if VideoCapture object was associated to webcam successfully
	{
		std::cout << "Setting camera properties...\n";

		cameraReturned.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
		cameraReturned.set(CAP_PROP_FRAME_WIDTH, frameWidth);
		cameraReturned.set(CAP_PROP_FRAME_HEIGHT, frameHeight);
		cameraReturned.set(CAP_PROP_FPS, fps);

		fps = cameraReturned.get(CAP_PROP_FPS);

		int32_t fourcc = cameraReturned.get(CAP_PROP_FOURCC);
		std::string fourcc_str = format("%c%c%c%c", fourcc & 255, (fourcc >> 8) & 255, (fourcc >> 16) & 255, (fourcc >> 24) & 255);

		for(int i = 0; i < 80; i++){std::cout << "-";}
		std::cout << "\n";

		std::cout << "CAMERA " << selectedCamera << " stats have been set as follows:\n";
		std::cout << "FPS: " << cameraReturned.get(CAP_PROP_FPS) << "\n";
		std::cout << "Resolution: " << cameraReturned.get(CAP_PROP_FRAME_WIDTH) << "*" << cameraReturned.get(CAP_PROP_FRAME_HEIGHT) << "\n";
		std::cout << "Camera Format: " << fourcc_str << "\n";
		std::cout << "RGB Flag: " << cameraReturned.get(CAP_PROP_CONVERT_RGB) << "\n";
		std::cout << "Complexity: " << ((double)frameHeight*(double)frameWidth/307200.0) << "\n";

		for(int i = 0; i < 80; i++){std::cout << "-";}
		std::cout << "\n\n";
	}
	else
	{
		perror("STRUAN: Camera failed to open, quitting.\nCOMPUTER");
		return -2;
	}
	return 0;
}
