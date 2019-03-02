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
@function get_average_distance_use_opencv ������Ƭ�����ڽǵ������ؾ���

@params filename �궨��ͼƬ������·��
@params board_row �궨��ÿ�еķ�����
@params board_col �궨��ÿ�еķ�����

@notice �ú���������opencv��findChessboardCorners()����������������:
		1. ��Ҫ���������ı궨�塢�궨�岻�ܱ������ڵ�
		2. ��Ҫ��ǰ֪���궨��Ĺ��
**/
//float get_average_distance_use_opencv(string file, int board_row, int board_col);


void get_mpp(float* mpp_x, float* mpp_y,
	string file, int board_row, int board_col, float square_size
);
//
//float calDistance(Point2f point0, Point2f point1);
//
//float calDistance(vector<Point2f> imgPoints, Size boardSize);