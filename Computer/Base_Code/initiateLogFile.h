#ifndef INITIATELOGFILE_H
#define INITIATELOGFILE_H

#include <opencv2/videoio.hpp>
#include <fstream>

int initiateLogFile(cv::VideoCapture&, std::ofstream&);

#endif
