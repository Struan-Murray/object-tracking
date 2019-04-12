#ifndef IMAGETRACKINGGLOBALS_H
#define IMAGETRACKINGGLOBALS_H

// Enabling of features

#define GUI 1 // Enabling of GUI (Master ON/OFF)
#define MINIMAL_GUI 0 // Runs minimal GUI, less stats on screen

#define SIMPLE_OPTIMISATIONS 0 // Cuts out unnecesary operations
#define EXPENSIVE_OPTIMISATIONS 0 // Cuts out important operations

#define EROSION_TYPE MORPH_RECT // Target Erosion Type
const int erosionMagnitude = 6;
#define DILATION_TYPE MORPH_RECT // Target Dilation Type
const int dilationMagnitude = 6;

#define MARKER_TYPE MARKER_STAR // Marker to place on tracked object

const double aspectRatio = (double)4/(double)3;
const int frameHeight = 480;
const int frameWidth = (int)(aspectRatio * (double)frameHeight);

const char seperator = ',';

#endif
