/********************************************************************
* Project Name : Road track detection for agv                       *
* Description  : Track the road for navigation                      *
* Author       : Kris Huang                                         *
* Date         : July 06, 2017                                      *
* Notes        : This file is road_with_track.h                     *
*                for road with traffic lanes				     	*
* CTI One Corporation, 3679 Enochs Street, Santa Clara, CA 95051    *
*********************************************************************/


#pragma once
#ifndef ROAD_WITH_TRACK_H
#define ROAD_WITH_TRACK_H


#include "agv_navigation.h"


class road_with_track:public agv_navigation {
private:
	Point target;
public:
	road_with_track(double a, double b, double c, double d) {
		f_dx = a; f_dy = b; c_x = c; c_y = d;
		target = Point(0, 0);
	 }
	Point agv_track_detection(Mat&);
	double agv_contourSize(const vector<Point>&);
	pair<double, double> agv_linear_regression(const vector<Point>&);
	double agv_navigate(const Mat&);
};


#endif