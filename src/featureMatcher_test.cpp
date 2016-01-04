/*
 * featureMatcher_test.cpp
 *
 *  Created on: Sep 22, 2015
 *      Author: nicolas
 */

#include <vector>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "klt_point_handling.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv )
{
	Mat Img_L = imread( argv[1], 1 );
	if ( !Img_L.data )
	{
		printf("No left image data \n");
		return -1;
	}
	cv::Mat Img_L_g;
	cv::cvtColor(Img_L, Img_L_g, CV_BGR2GRAY);

	Mat Img_R = imread( argv[2], 1 );
	if ( !Img_R.data )
	{
		printf("No right image data \n");
		return -1;
	}
	cv::Mat Img_R_g;
	cv::cvtColor(Img_R, Img_R_g, CV_BGR2GRAY);

	vector<int> updateVect(80, 2);
	vector<double> z_all_l;
	vector<double> z_all_r;

	handle_points_klt(
			Img_L_g,
			Img_R_g,
			z_all_l,
			z_all_r,
			updateVect);

	return 0;

}



