#pragma once

#include "get_pixel_accuracy.h"
using namespace cv;
using namespace std;


int main()
{
	string file = "E:/Source/repos/PointCloud/calibration_img/01.jpg";
	//----- test get_average_distance_use_opencv() -----------
	//get_average_distance_use_opencv("E:/Source/repos/PointCloud/calibration_img/01.jpg", 5, 8);
	//get_average_distance_use_opencv("E:/captures/0225/CAP_0_84.jpg", 9, 12);

	// ----- test get_mmp() ----------
	float mpp_x = 0, mpp_y = 0;
	get_mpp(&mpp_x, &mpp_y, file, 5, 8, 21.5);
	cout << mpp_x << ", " << mpp_y << endl;
}