#ifndef INITIATELOGFILE_H
#define INITIATELOGFILE_H

#include "imageTrackingGlobals.h"
#include <opencv2/videoio.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#if __GNUC__ >= 8
	#include <cstdlib>
	#include <filesystem>
	namespace fs = std::filesystem;
#endif

int initiateLogFile(cv::VideoCapture& camera, std::ofstream& statsFile)
{
	intmax_t programStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

	std::cout << "Opening performance file...\n";

	#if __GNUC__ >= 8
		fs::create_directory("logs");
	#endif

	statsFile.open("logs/Log_" + std::to_string(programStartTime) + ".csv", std::ios::out | std::ios::trunc);

	if (statsFile.is_open() == true) // Check if performance file was opened
	{
		std::cout << "Performace Log file opened.\n\n";
		statsFile << "FPS:" << seperator << camera.get(cv::CAP_PROP_FPS) << "\n";
		statsFile << "ResolutionHeight:" << seperator  << camera.get(cv::CAP_PROP_FRAME_HEIGHT) << "\n";
		statsFile << "ResolutionWidth:" << seperator << camera.get(cv::CAP_PROP_FRAME_WIDTH) << "\n";
		statsFile << "\n";

		statsFile <<
		"ObjectTracked" << seperator <<
		"WebcamRead" << seperator <<
		"GaussianBlur" << seperator <<
		"ColourConversion" << seperator <<
		"SelectMode" << seperator <<
		"InRangeMorphOpsDisplayDirection" << seperator <<
		"TrackFilteredObject" << seperator <<
		"PIDDist" << seperator <<
		"Facendi" << seperator <<
		"PutText" << seperator <<
		"ShowImage" << seperator <<
		"TimeWaitKey" << seperator <<
		"Frame" << seperator <<
		"MorphOps" << seperator <<
		"DisplayDirection" << "\n";
	}
	else
	{
		perror("STRUAN: File not opened, check filesystem is accessible to program and \"logs\" directory exists.\nCOMPUTER");
		return -3;
	}
	return 0;
}

#endif
