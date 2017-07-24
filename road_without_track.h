/********************************************************************
* Project Name : Road track detection for agv                       *
* Description  : Track the road for navigation                      *
* Author       : Kris Huang                                         *
* Date         : July 06, 2017                                      *
* Notes        : This file is road_without_track.h                  *
*                for road without traffic lanes				     	*
* CTI One Corporation, 3679 Enochs Street, Santa Clara, CA 95051    *
*********************************************************************/


#pragma once
#ifndef ROAD_WITHOUT_TRACK_H
#define ROAD_WITHOUT_TRACK_H


#include "road_with_track.h"

class findroad {
private:
	Mat path;
public:
	void setpath(Mat& image);
	Mat getroad(Mat &image);
};

class road_without_track : public agv_navigation {
private:
	pair<double, double> track;
public:
	road_without_track(double a, double b, double c, double d) {
		f_dx = a; f_dy = b; c_x = c; c_y = d;
		track = pair<double, double>(0., 0.); }
	Point agv_track_detection(Mat&);
	double agv_contourSize(const vector<Point>&);
	pair<vector<Point>, vector<Point>> agv_find_border_point(const Mat&, const Mat&);
	pair<double, double> agv_linear_regression(const vector<Point>&);
	vector<double> agv_quadratic_regression(const vector<Point>&);
	void agv_reject_outliers(vector<Point>&);
	double agv_navigate(const Mat&);
	Mat agv_shadow_removal(const Mat &);
};


#endif