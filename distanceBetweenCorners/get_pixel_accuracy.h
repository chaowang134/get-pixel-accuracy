#pragma once

//#include "pch.h"
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

/**
@function get_average_distance_use_opencv 计算照片上相邻角点间的像素距离

@params filename 标定用图片的完整路径
@params board_row 标定板每行的方格数
@params board_col 标定板每列的方格数

@notice 该函数调用了opencv的findChessboardCorners()因此有诸多限制条件:
		1. 需要拍摄完整的标定板、标定板不能被部分遮挡
		2. 需要提前知道标定板的规格
**/
//float get_average_distance_use_opencv(string file, int board_row, int board_col);


void get_mpp(float* mpp_x, float* mpp_y,
	string file, int board_row, int board_col, float square_size
);
//
//float calDistance(Point2f point0, Point2f point1);
//
//float calDistance(vector<Point2f> imgPoints, Size boardSize);