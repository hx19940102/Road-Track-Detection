/********************************************************************
* Project Name : Road track detection for agv                       *
* Description  : Track the road for navigation                      *
* Author       : Kris Huang                                         *
* Date         : July 06, 2017                                      *
* Notes        : This file is the main.cpp     				     	*
*********************************************************************/


#include "road_without_track.h"


#define F_DX 806
#define F_DY 806
#define C_X 340
#define C_Y 256


int main() {
	while (1) {
		//Mat img = imread("road5.jpg");
		int mode = 0;
		cout << "Input 1 for road with track, 2 for road without track, 3 for using camera, 4 for quit : " << endl;
		cin >> mode;
		cout << endl;
		if (mode == 1) {
			string video_name = "";
			cout << "Name of Video : " << endl;
			cin >> video_name;
			VideoCapture cam(video_name);
			if (!cam.isOpened()) continue;
			road_with_track* test = new road_with_track(500, F_DY, C_X, C_Y);
			Mat frame;
			while (1) {
				cam >> frame;
				if (frame.empty()) break;
				double angle = test->agv_navigate(frame);
				if (angle > 0) cout << "Trun Right : " << angle << " degree" << endl;
				if (angle < 0) cout << "Trun Left : " << -angle << " degree" << endl;
			}
		}
		if (mode == 2) {
			string video_name = "";
			cout << "Name of Video : " << endl;
			cin >> video_name;
			video_name = "ENOCHS_Videos//" + video_name;
			VideoCapture cam(video_name);
			if (!cam.isOpened()) continue;
			road_without_track* test = new road_without_track(1103 / 4, F_DY, C_X, C_Y);
			Mat frame;
			while (1) {
				cam >> frame;
				if (frame.empty()) break;
				double angle = test->agv_navigate(frame);
				if (angle > 0) cout << "Trun Right : " << angle << " degree" << endl;
				if (angle < 0) cout << "Trun Left : " << -angle << " degree" << endl;
			}
		}
		if (mode == 3) {
			VideoCapture cam(0);
			if (!cam.isOpened()) continue;
			road_without_track* test = new road_without_track(F_DX, F_DY, C_X, C_Y);
			Mat frame;
			while (1) {
				cam >> frame;
				if (frame.empty()) break;
				double angle = test->agv_navigate(frame);
				if (angle > 0) cout << "Trun Right : " << angle << " degree" << endl;
				if (angle < 0) cout << "Trun Left : " << -angle << " degree" << endl;
			}
		}
		if (mode == 4) break;
		cout << endl;
	}
	return 0;
}