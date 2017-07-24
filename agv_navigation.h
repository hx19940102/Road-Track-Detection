#pragma once
#ifndef AGV_NAVIGATION_H
#define AGV_NAVIGATION_H


#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;


class agv_navigation {
public:
	double f_dx, f_dy, c_x, c_y;
	agv_navigation() {};
};


#endif