#pragma once

#include "get_pixel_accuracy.h"
using namespace cv;
using namespace std;


//int main()
//{
//	string file = "E:/Source/repos/PointCloud/calibration_img/01.jpg";
//	//----- test get_average_distance_use_opencv() -----------
//	//get_average_distance_use_opencv("E:/Source/repos/PointCloud/calibration_img/01.jpg", 5, 8);
//	//get_average_distance_use_opencv("E:/captures/0225/CAP_0_84.jpg", 9, 12);
//
//	// ----- test get_mmp() ----------
//	float mpp_x = 0, mpp_y = 0;
//	get_mpp(&mpp_x, &mpp_y, file, 5, 8, 21.5);
//	cout << mpp_x << ", " << mpp_y << endl;
//}

/*
@function get_mpp 计算图片的像素精度

@params mpp_x  输出水平方向的像素精度
@params mpp_y  输出垂直方向的像素精度
@params file   图片的路径
@params board_row  标定板水平方向的方格数
@params board_col  标定板竖直方向的方格数
@params square_size  标定方格的边长
*/
void get_mpp(float* mpp_x, float* mpp_y, 
	string file, int board_row, int board_col, float square_size
)
{
	//cout << "measure pix distance between corners" << endl;
	// -------------open image-----------------
	Mat imgInput = imread(file);
	Mat imgGray;
	cvtColor(imgInput, imgGray, COLOR_RGB2GRAY);

	// -------find chessboard corners--------------
	Size boardSize = Size(board_row - 1, board_col - 1);
	vector<Point2f> imgPointsRaw;

	if (!findChessboardCorners(imgGray, boardSize, imgPointsRaw,
		CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK))
	{
		cout << file << "中检测不到角点\n";
		exit(1);
	}

	// ------- Subpix refine ---------
	vector<Point2f>	imgPointsSubpix(imgPointsRaw);

	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	cornerSubPix(imgGray, imgPointsSubpix, Size(5, 5), Size(-1, -1), termcrit);

	// 显示角点位置
	//drawChessboardCorners(imgInput, boardSize, imgPointsSubpix, true);
	//imshow("Camera Calibration", imgInput);
	//waitKey();

	// 输出角点坐标
	//cout << imgPointsRaw << endl;
	//cout << imgPointsSubpix << endl;

	// s
	//for (int i = 0; i < imgPointsSubpix.size(); )
	//{
	//	for (int j = 0; j < board_row - 1; ++j)
	//	{
	//		cout << "\t" << imgPointsSubpix[i];
	//		++i;
	//	}
	//	cout << "\n";
	//}

	// 找到角落上的四个角点
	Point2f D, C, A, B;
	D = imgPointsSubpix[0];
	C = imgPointsSubpix[(board_row - 2)];
	A = imgPointsSubpix[(board_col - 2)*(board_row - 1)];
	B = imgPointsSubpix[(board_col - 2)*(board_row - 1) + (board_row - 2)];

	//cout << D << endl;
	//cout << C << endl;
	//cout << A << endl;
	//cout << B << endl;

	// 计算角点之间的物理距离
	float board_width  = square_size * (board_row - 2);
	float board_height = square_size * (board_col - 2);

	// 令 k = mpp_y/mmp_x，计算k的平均值
	float k_A = -(board_height * (B.x - A.x)) / (board_width*(D.y - A.y));
	float k_C = -(board_height * (C.x - D.x)) / (board_width*(C.y - B.y));
	float k_D = -(board_height * (C.x - D.x)) / (board_width*(D.y - A.y));
	float k_B = -(board_height * (B.x - A.x)) / (board_width*(C.y - B.y));
	float k = (k_A + k_C + k_B + k_D) / 4;


	//cout << k_A << ", " << k_B << ", " << k_C << ", " << k_D << endl;
	//cout << k << endl;
	// 计算对角线的长度
	float diagonal_x = (abs(B.x - D.x) + abs(C.x - A.x)) / 2;
	float diagonal_y = (abs(B.y - D.y) + abs(A.y - C.y)) / 2;

	*mpp_x = sqrt((board_width*board_width + board_height * board_height) / (powf(diagonal_x, 2) + powf(k*diagonal_y, 2)));
	*mpp_y = k * *mpp_x;
}

//float get_average_distance_use_opencv(string file, int board_row, int board_col)
//{
//	cout << "measure pix distance between corners" << endl;
//	// -------------open image-----------------
//	Mat imgInput = imread(file);
//	Mat imgGray;
//	cvtColor(imgInput, imgGray, COLOR_RGB2GRAY);
//
//	if (imgGray.empty())
//	{
//		cout << "No valid image input." << endl;
//		char c = getchar();
//		return -1;
//	}
//
//
//
//	// -------find chessboard corners--------------
//	Size boardSize = Size(board_row - 1, board_col - 1);
//	vector<Point2f> imgPointsRaw;
//	
//	if (!findChessboardCorners(imgGray, boardSize, imgPointsRaw, 
//		CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE+ CALIB_CB_FAST_CHECK))
//	{
//		cout << file << "中检测不到角点\n";
//		exit(1);
//	}
//	else // 亚像素精确化
//	{
//		vector<Point2f>	imgPointsSubpix(imgPointsRaw);
//
//		TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
//		cornerSubPix(imgGray, imgPointsSubpix, Size(5, 5), Size(-1, -1), termcrit);
//
//		//// 显示角点位置
//		//drawChessboardCorners(imgInput, boardSize, imgPointsSubpix, true);
//		//imshow("Camera Calibration", imgInput);
//		
//		//// 输出角点坐标
//		//cout << imgPointsRaw << endl;
//		//cout << imgPointsSubpix << endl;
//
//		// 计算相邻角点之间的距离
//		return calDistance(imgPointsSubpix, boardSize);
//	}
//
//	return -1.0;
//}
//
//float calDistance(Point2f point0, Point2f point1)
//{
//	float distance;
//
//	distance = powf((point0.x - point1.x), 2) + powf((point0.y - point1.y), 2);
//	distance = sqrtf(distance);
//
//	return distance;
//}
//
//float calDistance(vector<Point2f> imgPoints, Size boardSize)
//{
//	// --------check input----------
//	if (imgPoints.size() != boardSize.width * boardSize.height)
//	{
//		cout << "calDistance 传入的参数不匹配！";
//		exit(0);
//	}
//
//	// --------calculate the average of all distances
//	float distanesTotal = 0;
//
//	int count = 0;
//	for (int i = 0; i < boardSize.height; i++)
//	{
//		for (int j = 0; j < boardSize.width; j++)
//		{
//			int idx = i * boardSize.width + j;
//			if (j != boardSize.width - 1)
//			{
//				float dis = calDistance(imgPoints[idx], imgPoints[idx + 1]);
//				distanesTotal += dis;
//				cout << ++count << ":" << dis << "\t";
//			}
//			if (i != boardSize.height - 1)
//			{
//				float dis = calDistance(imgPoints[idx], imgPoints[idx + boardSize.width]);
//				distanesTotal += dis;
//				cout << ++count << ":" << dis << "\t";
//			}
//		}
//	}
//
//	int pointsCount = 2 * (boardSize.height * boardSize.width) - boardSize.height - boardSize.width;
//	cout << "average_distance: " << distanesTotal / pointsCount;
//	return distanesTotal / pointsCount;
//}