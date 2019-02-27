#ifndef UTILITIES_H
#define UTILITIES_H

#include <opencv2/core.hpp> // OpenCV's core functionality, definition of Mat

#include <chrono>
#include <string>

bool confirm();
std::string printFormattedTime(std::chrono::high_resolution_clock::time_point, std::chrono::high_resolution_clock::time_point); // Prints time with appropriate units XSTRUANCHECKED
intmax_t getNanoTime(std::chrono::high_resolution_clock::time_point, std::chrono::high_resolution_clock::time_point); // Returns time in microseconds XSTRUANCHECKED
void createTrackbars(int&); // Creates window with option to start program XSTRUANCHECKED
void drawObject(int , int , int, cv::Mat&); // Draws a target marker at point specified by inputs to function XSTRUANCHECKED
void morphOps(cv::Mat&); // Takes raw binary image (thresh) and erodes noise, then dilates what remains XSTRUANCHECKED
void displayDirection(int_fast16_t, int_fast16_t, int_fast16_t*, int_fast16_t*); // Calculates direction of object over several frames XSTRUANCHECKED

#endif
