/********************************************************************
* Project Name : Road track detection for agv                       *
* Description  : Track the road for navigation                      *
* Author       : Kris Huang                                         *
* Revision     : KH and HL, version 2.0;                            *
* Date         : July 07, 2017                                      *
* Notes        : This file is road_without_track.cpp                *
*                for road without traffic lanes		                *
* Caution (HL and KH, July 7th, 17) :                               *
* This program currently does not work for the the heavy shadows    *
* casted by the sunshine, see the testing image and video;          *
* image xxxx2650.jpg and ENOCHS_110.MOV                             *
* To solve this problem:                                            *
*  1. add stereo vision depth map; and                              *
*  2. Lidar depth map;                                              *
*     if the input from vision is an obstacle                       *
*        && input from the stereo is far away and/or same as the    *
*           neighbouring path region                                *
*     then the "obstacle" is ignored;                               *
*        && the path is calculated based on partial video info and  *
*           stereo vision                                           *
* CTI One Corporation, 3679 Enochs Street, Santa Clara, CA 95051    *
* Note:                                                             *
*  processing principles:                                           *
*  1. PROXIMITY;                                                    *
*  2. Good continuation;                                            *
*  3. Symmetry;                                                     *
*  4. Similarity;                                                   *
*  5. Closure;                                                      *
*  6. Simplicity;                                                   *
*********************************************************************/


#include "road_without_track.h"

#define    sigmaX    1.0          //Gaussian kernel 
#define    sigmaY    1.0
#define    gaussian_kernel_size    5
#define    img_resize_ratio        1 //0.5, or 0.25, or 0.125 
#define    img_thresh    100


Point road_without_track::agv_track_detection(Mat &img)
{
	// preprocessing 
	// Resize the image -- preprocessing stage 1 
	resize(img, img, Size(img.cols * img_resize_ratio, img.rows * img_resize_ratio), 
		0, 0, INTER_LINEAR);


	// Gaussian blur decreases noise -- preprocessing stage 2 
	Mat img_copy;
	img.copyTo(img_copy);
	Size gaussian_kernel = Size(gaussian_kernel_size, gaussian_kernel_size);
	GaussianBlur(img_copy, img_copy, gaussian_kernel, sigmaX, sigmaY);


	// Removal of shadows
	Mat shadow_mask = agv_shadow_removal(img_copy);


	// Binarization of grey scale -- preprocessing stage 3 
	Mat img_gray;
	cvtColor(img_copy, img_gray, CV_BGR2GRAY);
	threshold(img_gray, img_gray, img_thresh, 255, CV_THRESH_OTSU);


	// Image morphology processing -- preprocessing stage 4 
	Mat img_erode, img_dilate;
	erode(img_gray, img_erode, Mat(), Point(2, 2), 7);     // removal of smaller binarized regions  
	dilate(img_gray, img_dilate, Mat(), Point(2, 2), 7);   // connecting regions 
	threshold(img_dilate, img_dilate, 1, 50, THRESH_BINARY_INV); // ??? change 0 to 50 by offset 

	Mat path_trace(img_gray.size(), CV_8U, Scalar(0));
	path_trace = img_erode + img_dilate;

	findroad road;
	road.setpath(path_trace);

	// Rule 4: similarity (color) 
	// Rule 1: proximity (region) 
	Mat road_found = road.getroad(img);
	road_found.convertTo(road_found, CV_8U); // water shade algorithm, result back to road_found 
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 1));
	dilate(road_found, road_found, element, Point(0, 0), 3);
	erode(road_found, road_found, element, Point(0, 0), 3);
	imshow("road", road_found);
	imshow("shadow", shadow_mask);


	pair<vector<Point>, vector<Point>> points = agv_find_border_point(road_found, shadow_mask); //find the buttom points 


	// Divide into left part points and right part points
	// and remove outliers based on rule 2 : good continuous
	vector<Point> left_points = points.first;
	agv_reject_outliers(left_points);
	vector<Point> right_points = points.second;
	agv_reject_outliers(right_points);
	for (unsigned int i = 0; i < left_points.size(); i++) circle(img, left_points[i], 4, Scalar(0, 0, 255), 3);
	for (unsigned int i = 0; i < right_points.size(); i++) circle(img, right_points[i], 4, Scalar(0, 0, 255), 3);


	// rule 3: symmetry for left and right
	// rule 6: simplicity for regression 
	for (unsigned int i = 0; i < right_points.size(); i++) right_points[i].x = img.cols - 1 - right_points[i].x;
	pair<double, double> left_bottom_line = agv_linear_regression(left_points);
	pair<double, double> right_bottom_line = agv_linear_regression(right_points);
	Point left_pt1(0, left_bottom_line.second),
		left_pt2(img.cols * 0.4, img.cols * 0.4 * left_bottom_line.first + left_bottom_line.second),
		right_pt1(img.cols - 1, right_bottom_line.second),
		right_pt2(img.cols - 1 - img.cols * 0.4, img.cols * 0.4 * right_bottom_line.first + right_bottom_line.second);
	line(img, left_pt1, left_pt2, Scalar(0, 255, 0), 3);
	line(img, right_pt1, right_pt2, Scalar(0, 255, 0), 3);

	int y_max = min(left_pt1.y, right_pt1.y),
		y_min = max(left_pt2.y, right_pt2.y);
	int y_m = (y_max + y_min) / 2;
	int x_m = ((y_m - left_bottom_line.second) / left_bottom_line.first +
		(img.cols - 1 - (y_m - right_bottom_line.second) / right_bottom_line.first)) / 2;

	Point path_central_point(-1, -1),
		camera_central_point(img.cols / 2, img.rows - 1);
	vector<Point> central_curve_points;
	if (y_max < y_min || x_m < 0 || x_m >= img.cols) {
		path_central_point.x = img.cols / 2;
		path_central_point.y = img.rows / 2;
	}
	else {
		path_central_point.x = x_m;
		path_central_point.y = y_m;
	}
	
	
	/* Find left anf right bottom curve
	vector<double> left_curve = agv_quadratic_regression(left_points);
	vector<double> right_curve = agv_quadratic_regression(right_points);
	vector<Point2f> left_curve_points, right_curve_points;
	unsigned int i = 0;
	for (int i = 0; i < img.cols * 0.35; i++) {
		Point temp_left(i, left_curve[0] + left_curve[1] * i + left_curve[2] * i * i);
		left_curve_points.push_back(temp_left);
		Point temp_right(img.cols - 1 - i, right_curve[0] + right_curve[1] * i + right_curve[2] * i * i);
		right_curve_points.push_back(temp_right);
	}


	// Search central point of the road
	int y_i = max(left_curve_points[(int)left_curve_points.size() - 1].y, right_curve_points[(int)right_curve_points.size() - 1].y);
	int y_r = min(left_curve_points[0].y, right_curve_points[0].y);
	int y_m = (y_i + y_r) / 2;
	int x_i_l = -1, x_m_l = -1, x_r_l = -1;
	for (unsigned int i = 0; i < left_curve_points.size(); i++) {
		if (left_curve_points[i].y == y_i) x_i_l = left_curve_points[i].x;
		if (left_curve_points[i].y == y_m) x_m_l = left_curve_points[i].x;
		if (left_curve_points[i].y == y_r) x_r_l = left_curve_points[i].x;
	}
	int x_i_r = -1, x_m_r = -1, x_r_r = -1;
	for (unsigned int i = 0; i < right_curve_points.size(); i++) {
		if (right_curve_points[i].y == y_i) x_i_r = right_curve_points[i].x;
		if (right_curve_points[i].y == y_m) x_m_r = right_curve_points[i].x;
		if (right_curve_points[i].y == y_r) x_r_r = right_curve_points[i].x;
	}


	Point path_central_point(-1, -1),
		camera_central_point(img.cols / 2, img.rows - 1);
	vector<Point> central_curve_points;
	if (x_i_l == -1 || x_i_r == -1 || x_r_l == -1 || x_r_r == -1) {
		path_central_point.x = img.cols / 2;
		path_central_point.y = img.rows / 2;
	}
	else {
		path_central_point.x = (x_m_r + x_m_l) / 2;
		path_central_point.y = y_m;
	}
	*/
	

	/*
	line(img, Point(0, left_bottom_line.second), Point(img.cols / 2,
		left_bottom_line.first * img.cols / 2 + left_bottom_line.second),
		Scalar(0, 255, 0), 2);  // linear interpolation, but quadratic is better; change to quadratic fitting 
	line(img, Point(img.cols / 2, right_bottom_line.first * img.cols / 2
		+ right_bottom_line.second), Point(img.cols - 1, right_bottom_line.first * (img.cols - 1)
			+ right_bottom_line.second), Scalar(0, 255, 0), 2); // for the right side of the image, same 2nd order needed 
	*/
	//Mat curve_left(left_curve_points, true);
	//curve_left.convertTo(curve_left, CV_32S);
	//Mat curve_right(right_curve_points, true);
	//curve_right.convertTo(curve_right, CV_32S);
	//Mat curve_central(central_curve_points, true);
	//curve_central.convertTo(curve_central, CV_32S);
	//polylines(img, curve_left, false, Scalar(0, 255, 0), 2);
	//polylines(img, curve_right, false, Scalar(0, 255, 0), 2);
	circle(img, path_central_point, 3, Scalar(255, 0, 0), 3);
	line(img, camera_central_point, path_central_point, Scalar(255, 255, 0), 3);
	//polylines(img, curve_central, false, Scalar(255, 0, 0), 2);

	// rule 5: closure -- no need for that

	return path_central_point;
}


double road_without_track::agv_contourSize(const vector<Point>& contour)
{
	return contourArea(contour);
}


pair<vector<Point>, vector<Point>> road_without_track::agv_find_border_point(const Mat & road, const Mat & shadow_mask)
{
	const int step_width = road.cols * 0.02;
	vector<Point> left_points;
	for (unsigned int i = 0; i < 20 * step_width; i += step_width) {
		for (unsigned int j = road.rows - 1; j > 0; j--) {
			int pixel = road.at<uchar>(j, i);
			if (pixel > 0 && pixel < 255) {
				left_points.push_back(Point(i, j)); break;
			}
		}
	}

	vector<Point> right_points;
	for (unsigned int i = road.cols - 1; i > road.cols - 1 - 20 * step_width; i -= step_width) {
		for (unsigned int j = road.rows - 1; j > 0; j--) {
			int pixel = road.at<uchar>(j, i);
			if (pixel > 0 && pixel < 255)
			{
				right_points.push_back(Point(i, j)); break;
			}
		}
	}

	return pair<vector<Point>, vector<Point>>(left_points, right_points);
}


pair<double, double> road_without_track::agv_linear_regression(const vector<Point>& points)
{
	if ((int)points.size() < 2) return pair<double, double>(0, 0);
	double avg_x = 0., avg_y = 0.;
	const int size = (int)points.size();
	for (unsigned int i = 0; i < size; i++) {
		avg_x += points[i].x;
		avg_y += points[i].y;
	}
	avg_x = avg_x / size; avg_y = avg_y / size;
	double k1 = 0, k2 = 0;
	for (unsigned int i = 0; i < size; i++) {
		k1 += (points[i].x - avg_x) * (points[i].y - avg_y);
		k2 += (points[i].x - avg_x) * (points[i].x - avg_x);
	}
	double k = k1 / k2; double b = avg_y - k * avg_x;
	return pair<double, double>(k, b);
}


vector<double> road_without_track::agv_quadratic_regression(const vector<Point>& points)
{
	if ((int)points.size() < 3) return vector<double>(3, 0);
	// Fit a quadratic function based a set of points with least-square method
	const int n = (int)points.size();
	double x_i = 0, x_i2 = 0, x_i3 = 0, x_i4 = 0;
	double y_i = 0, x_i_y_i = 0, x_i2_y_i = 0;
	for (unsigned int i = 0; i < points.size(); i++) {
		x_i += points[i].x;
		x_i2 += points[i].x * points[i].x;
		x_i3 += points[i].x * points[i].x * points[i].x;
		x_i4 += points[i].x * points[i].x * points[i].x * points[i].x;
		y_i += points[i].y;
		x_i_y_i += points[i].x * points[i].y;
		x_i2_y_i += points[i].x * points[i].x * points[i].y;
	}
	Mat A(3, 3, CV_64F);
	A.at<double>(0, 0) = n; A.at<double>(0, 1) = x_i; A.at<double>(0, 2) = x_i2;
	A.at<double>(1, 0) = x_i; A.at<double>(1, 1) = x_i2; A.at<double>(1, 2) = x_i3;
	A.at<double>(2, 0) = x_i2; A.at<double>(2, 1) = x_i3; A.at<double>(2, 2) = x_i4;
	Mat b(3, 1, CV_64F);
	b.at<double>(0, 0) = y_i; b.at<double>(1, 0) = x_i_y_i; b.at<double>(2, 0) = x_i2_y_i;

	Mat A_inverse;
	invert(A, A_inverse);
	Mat x = A_inverse * b;
	vector<double> res(3, 0.);
	res[0] = x.at<double>(0, 0); res[1] = x.at<double>(1, 0); res[2] = x.at<double>(2, 0);

	return res;
}


void road_without_track::agv_reject_outliers(vector<Point>& points)
{
	for (unsigned int i = 1; (i + 1) < points.size(); i++) {
		if ((points[i].y > points[i - 1].y && points[i].y > points[i + 1].y) || (points[i].y < points[i - 1].y && points[i].y < points[i + 1].y)) {
			points.erase(points.begin() + i);
			i--;
		}
	}
}


double road_without_track::agv_navigate(const Mat& img)
{
	Mat img_copy; img.copyTo(img_copy);
	const Point p = agv_track_detection(img_copy);
	double k = (p.x - img_copy.cols / 2) / f_dx;
	double angle = atan(k) * 180 / CV_PI;
	if (p.x == img.cols / 2) angle = 0;
	
	if (angle > 0) {
		putText(img_copy, "Turn Right " + to_string(angle) + " degree", Point(50, 50),
			FONT_HERSHEY_COMPLEX, .8, Scalar(0, 255, 0), 2);
	}
	else if(angle < 0) {
		putText(img_copy, "Turn Left " + to_string(-angle) + " degree", Point(50, 50),
			FONT_HERSHEY_COMPLEX, .8, Scalar(0, 0, 255), 2);
	}
	else {
		putText(img_copy, "Keep Straight", cvPoint(50, 50),
			FONT_HERSHEY_COMPLEX, .8, Scalar(0, 0, 0), 2);
	}

	imshow("img", img_copy);
	waitKey(1);

	return angle;
}


// Not used yet
Mat road_without_track::agv_shadow_removal(const Mat & img)
{
	// We define a shadow pixel as any pixel whose brightness falls
	// below a minimum threshold, and whose blue content is greater
	// than its red or green content(measured as 8 bits in each of the
	// three channels)
	Mat img_copy; img.copyTo(img_copy);

	Mat img_gray;
	cvtColor(img_copy, img_gray, CV_BGR2GRAY);
	Mat rgbs[3];
	split(img_copy, rgbs);

	int thres = 50;
	Mat shadow_mask = Mat::ones(img_copy.rows, img_copy.cols, CV_8UC1) * 255;
	for (int i = img_copy.rows * 0.3; i < img_copy.rows; i++) {
		for (int j = 0; j < img_copy.cols; j++) {
			if ((img_gray.at<uchar>(i, j) < thres) &&
				(rgbs[0].at<uchar>(i, j) > rgbs[1].at<uchar>(i, j)) &&
				(rgbs[0].at<uchar>(i, j) > rgbs[2].at<uchar>(i, j))) {
				shadow_mask.at<uchar>(i, j) = 0;
			}
		}
	}

	//imshow("img", img_copy);
	//imshow("mask", shadow_mask);
	//waitKey(1);
	return shadow_mask;
}


void findroad::setpath(Mat & image)
{
	image.convertTo(path, CV_32S);
}


Mat findroad::getroad(Mat & image)
{
	watershed(image, path);
	path.convertTo(path, CV_8U);
	return path;
}
