#pragma once
#ifndef ROAD_WITH_TRACK_H
#define ROAD_WITH_TRACK_H


#include "agv_navigation.h"

class road_with_track {
public:
	vector<Vec4i> agv_track_detection(const Mat&);
	double agv_contourSize(const vector<Point>&);
};


#endif