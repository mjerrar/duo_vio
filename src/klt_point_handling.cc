#include "klt_point_handling.h"

#include <algorithm>
#include <list>
#include <vector>

#include <iostream>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>

using namespace std;
using namespace cv;

//"persistent" local variables
static cv::Mat prev_img;
static std::vector<cv::Point2f> prev_corners;
static std::vector<cv::Point2f> prev_corners_right;
static std::vector<unsigned char> prev_status(100, 0);
static cv::FastFeatureDetector detector;
static cv::OrbDescriptorExtractor extractor;

//local functions
static void initMorePoints(const cv::Mat &img_l, const cv::Mat &img_r, std::vector<int> &updateVect, vector<double> &z_all_l, vector<double> &z_all_r);
bool stereoMatch(const cv::Mat &img_l, const cv::Mat &img_r, std::vector<cv::KeyPoint> &keypointsL, std::vector<cv::Point2f> &leftPoints, std::vector<cv::Point2f> &rightPoints);
bool stereoMatchOpticalFlow(const cv::Mat &img_l, const cv::Mat &img_r, std::vector<cv::KeyPoint> &keypointsL, std::vector<cv::Point2f> &leftPoints, std::vector<cv::Point2f> &rightPoints);
bool compareMatch(const DMatch &first, const DMatch &second);
bool compareKeypoints(const KeyPoint &first, const KeyPoint &second);

// local functions for KLT version
static void initMorePoints_KLT(const cv::Mat &img_l, const cv::Mat &img_r,	vector<int> &updateVect, vector<double> &z_all_l, vector<double> &z_all_r, int gridSizeX, int gridSizeY, double minDistSqr, double minScore);
static double shiTomasiScore(const cv::Mat &image, const cv::Point &position, int halfWindow);

//corners (z_all_l, z_all_r) and status are output variables
void handle_points_klt(
		const cv::Mat &img_l,
		const cv::Mat &img_r,
		vector<double> &z_all_l,
		vector<double> &z_all_r,
		vector<int> &updateVect)
{
//	clock_t t1 = clock();

	unsigned int numPoints = updateVect.size();
	z_all_l.resize(numPoints * 2);
	std::fill(z_all_l.begin(), z_all_l.end(), -100.0);

	z_all_r.resize(numPoints * 2);
	std::fill(z_all_r.begin(), z_all_r.end(), -100.0);

	for (size_t i = 0; i < updateVect.size() && i <numPoints; ++i)
	{
		if(updateVect[i] == 1)
		{
			prev_status[i] = 1;
		} else {
			prev_status[i] = 0; // if updateVect[i] == 0 feature is inactive, == 2 request new feature
		}
	}

	std::vector<unsigned char> status;
	std::vector<cv::Point2f> cur_corners;
	std::vector<float> error;

	if (!prev_img.empty())
	{
		if (!prev_corners.empty())
		{
			cv::calcOpticalFlowPyrLK(prev_img, img_l, prev_corners, cur_corners, status, error, cv::Size(9,9), 3);
			prev_corners = cur_corners;


			for (size_t i = 0; i < prev_corners.size() && i < numPoints; ++i)
			{
				if(!(prev_status[i] && status[i]))
					prev_status[i] = 0;

				if (prev_status[i] == 1)
				{
					z_all_l[2*i+0] = prev_corners[i].x;
					z_all_l[2*i+1] = prev_corners[i].y;
					updateVect[i] = 1;
				} else {
					if (updateVect[i] == 1) // be careful not to overwrite 2s in updateVect
						updateVect[i] = 0;
				}
			}
		}
	}

	img_l.copyTo(prev_img);

	bool init_new_points_with_KLT = false;

	if (!img_r.empty())
	{
		//check if we need more points
		if (init_new_points_with_KLT)
		{
			const int gridSizeX = 4;
			const int gridSizeY = 4;
			const double minDistSqr = 10*10;
			const double minScore = 4;

			initMorePoints_KLT(img_l, img_r, updateVect, z_all_l, z_all_r, gridSizeX, gridSizeY, minDistSqr, minScore);
		} else {
			initMorePoints(img_l, img_r, updateVect, z_all_l, z_all_r);
		}
	} else {
		printf("Right image is empty!\n");
	}

//	clock_t t2 = clock();
//	printf("Point tracker took: %d clicks, %f msec\n", int(t2 - t1), 1000*float(t2 - t1)/CLOCKS_PER_SEC);
}

// ==== local functions, hidden from outside this file ====

static void initMorePoints(
		const cv::Mat &img_l,
		const cv::Mat &img_r,
		vector<int> &updateVect,
		vector<double> &z_all_l,
		vector<double> &z_all_r)
{
	if (!img_l.data)
		throw "Left image is invalid";
	if (!img_r.data)
		throw "Right image is invalid";

	unsigned int targetNumPoints = 0;
	// count the features that need to be initialized
	for (int i = 0; i < updateVect.size(); i++)
	{
		if (updateVect[i] == 2) // 2 means VIO requested stereo measurement
			targetNumPoints++;
	}

	if(!targetNumPoints)
		return;

	std::vector<cv::KeyPoint> keypointsL, keypointsR, goodKeypointsL, unusedKeypoints;
	cv::Mat descriptorsL, descriptorsR;

	int numBinsX = 4;
	int numBinsY = 4;
	int binWidth = img_l.cols/numBinsX;
	int binHeight = img_l.rows/numBinsY;
	int targetFeaturesPerBin = (updateVect.size()-1)/(numBinsX * numBinsY)+1; // total number of features that should be in each bin

	int featuresPerBin[numBinsX][numBinsY];
	memset( featuresPerBin, 0, numBinsX*numBinsY*sizeof(int) ); // set everything to 0

	// count the number of active features in each bin
	for (int i = 0; i < prev_corners.size(); i++)
	{
		int binX = prev_corners[i].x / binWidth;
		int binY = prev_corners[i].y / binHeight;

		featuresPerBin[binX][binY]++;
	}

	unsigned int dist = binWidth/targetFeaturesPerBin;
	// go through each cell and detect features
	for (int x = 0; x < numBinsX; x++)
	{
		for (int y = 0; y < numBinsY; y++)
		{
			int neededFeatures = max(0, targetFeaturesPerBin - featuresPerBin[x][y]);
//			printf("needed features: %d\n", neededFeatures);
			if (neededFeatures)
			{
				int col_from = x*binWidth;
				int col_to   = min((x+1)*binWidth, img_l.cols);
				int row_from = y*binHeight;
				int row_to   = min((y+1)*binHeight, img_l.rows);

//				printf("bin (%d %d) x: (%d %d), y: (%d %d)\n", x, y, col_from, col_to, row_from, row_to);

				std::vector<cv::KeyPoint> keypoints, goodKeypointsBin;
				detector.detect(img_l.rowRange(row_from, row_to).colRange(col_from, col_to), keypoints);

				sort(keypoints.begin(), keypoints.end(), compareKeypoints);

				// add bin offsets to the points
				for (int i = 0; i < keypoints.size(); i++)
				{
					keypoints[i].pt.x += col_from;
					keypoints[i].pt.y += row_from;
				}

//				printf("detected %d keypoints\n", keypoints.size());

				// check if the new features are far enough from existing points
				int newPtIdx = 0;
				for (; newPtIdx < keypoints.size(); newPtIdx++)
				{
					int new_pt_x = keypoints[newPtIdx].pt.x;
					int new_pt_y = keypoints[newPtIdx].pt.y;

					bool far_enough = true;
					for (int j = 0; j < prev_corners.size(); j++)
					{
						int existing_pt_x = prev_corners[j].x;
						int existing_pt_y = prev_corners[j].y;
						if(abs(existing_pt_x - new_pt_x) < dist && abs(existing_pt_y - new_pt_y) < dist)
						{
//							printf("Discarding new point %d at (%d, %d) because it's too close to existing point %d at (%d, %d)\n", j, new_pt_x, new_pt_y, j, existing_pt_x, existing_pt_y);
							far_enough = false;
							break;
						}
					}
					if (far_enough)
					{
						// check if the new feature is too close to a new one
						for (int j = 0; j < goodKeypointsBin.size(); j++)
						{
							int existing_pt_x = goodKeypointsBin[j].pt.x;
							int existing_pt_y = goodKeypointsBin[j].pt.y;
							if(abs(existing_pt_x - new_pt_x) < dist && abs(existing_pt_y - new_pt_y) < dist)
							{
//								printf("Discarding new point %d at (%d, %d) because it's too close to another new point %d at (%d, %d)\n", j, new_pt_x, new_pt_y, j+1, existing_pt_x, existing_pt_y);
								far_enough = false;
								break;
							}
						}
						if (far_enough)
						{
							goodKeypointsBin.push_back(keypoints[newPtIdx]);
//							printf("Found good feature at (%d, %d)\n", new_pt_x, new_pt_y);
							if(goodKeypointsBin.size() == neededFeatures)
								break;
						}
					}
				}
//				printf("found %d good keypoints in this bin\n", goodKeypointsBin.size());
				// insert the good points into the vector containing the new points of the whole image
				goodKeypointsL.insert(goodKeypointsL.end(), goodKeypointsBin.begin(), goodKeypointsBin.end());
				// save the unused keypoints for later
				if (newPtIdx < keypoints.size()-1)
				{
					unusedKeypoints.insert(unusedKeypoints.end(), keypoints.begin()+newPtIdx, keypoints.end());
				}
			}
		}
	}

	if (goodKeypointsL.size() < targetNumPoints)
	{
//		printf("need %d more features, have %d unused ones to try\n", targetNumPoints-goodKeypointsL.size(), unusedKeypoints.size());
		// try to insert new points that were not used in the bins
		sort(unusedKeypoints.begin(), unusedKeypoints.end(), compareKeypoints);

		dist /= 2; // reduce the distance criterion

		for (int newPtIdx = 0; newPtIdx < unusedKeypoints.size(); newPtIdx++)
		{
			int new_pt_x = unusedKeypoints[newPtIdx].pt.x;
			int new_pt_y = unusedKeypoints[newPtIdx].pt.y;

			bool far_enough = true;
			for (int j = 0; j < prev_corners.size(); j++)
			{
				int existing_pt_x = prev_corners[j].x;
				int existing_pt_y = prev_corners[j].y;
				if(abs(existing_pt_x - new_pt_x) < dist && abs(existing_pt_y - new_pt_y) < dist)
				{
					//							printf("Discarding new point %d at (%d, %d) because it's too close to existing point %d at (%d, %d)\n", j, new_pt_x, new_pt_y, j, existing_pt_x, existing_pt_y);
					far_enough = false;
					break;
				}
			}
			if (far_enough)
			{
				// check if the new feature is too close to a new one
				for (int j = 0; j < goodKeypointsL.size(); j++)
				{
					int existing_pt_x = goodKeypointsL[j].pt.x;
					int existing_pt_y = goodKeypointsL[j].pt.y;
					if(abs(existing_pt_x - new_pt_x) < dist && abs(existing_pt_y - new_pt_y) < dist)
					{
						//								printf("Discarding new point %d at (%d, %d) because it's too close to another new point %d at (%d, %d)\n", j, new_pt_x, new_pt_y, j+1, existing_pt_x, existing_pt_y);
						far_enough = false;
						break;
					}
				}
				if (far_enough)
				{
					goodKeypointsL.push_back(unusedKeypoints[newPtIdx]);
					//							printf("Found good feature at (%d, %d)\n", new_pt_x, new_pt_y);
					if(goodKeypointsL.size() == targetNumPoints)
						break;
				}
			}
		}
//		printf("%d new features with %d requested after adding from unused ones\n", goodKeypointsL.size(), targetNumPoints);
	}

	if (goodKeypointsL.empty())
	{
		for (int i = 0; i < updateVect.size(); i++)
		{
			if (updateVect[i] == 2)
				updateVect[i] = 0;
		}
		return;
	}

	std::vector<cv::Point2f> leftPoints, rightPoints;

	if (!stereoMatchOpticalFlow(img_l, img_r, goodKeypointsL, leftPoints, rightPoints))
	{
		for (int i = 0; i < updateVect.size(); i++)
		{
			if (updateVect[i] == 2)
				updateVect[i] = 0;
		}
		return;
	}
	if (leftPoints.size() != rightPoints.size())
			printf("Left and right points have different sizes: left %d, right %d\n", (int) leftPoints.size(), (int) rightPoints.size());

	if (leftPoints.size() != targetNumPoints)
		printf("Number of good matches: %d, desired: %d\n", (int) leftPoints.size(), targetNumPoints);

	if (prev_corners.size() < updateVect.size())
			prev_corners.resize(updateVect.size());
	int matches_idx = 0;
	for (int i = 0; i < updateVect.size(); i++)
	{
		if(updateVect[i] == 2)
		{
			if (matches_idx < leftPoints.size())
			{
				prev_corners[i] = leftPoints[matches_idx];
				prev_status[i] = 1;

				z_all_l[i*2 + 0] = leftPoints[matches_idx].x;
				z_all_l[i*2 + 1] = leftPoints[matches_idx].y;

				z_all_r[i*2 + 0] = rightPoints[matches_idx].x;
				z_all_r[i*2 + 1] = rightPoints[matches_idx].y;

				matches_idx++;
			} else {
				updateVect[i] = 0;
			}
		}
	}
}


bool stereoMatch(const cv::Mat &img_l, const cv::Mat &img_r, std::vector<cv::KeyPoint> &keypointsL, std::vector<cv::Point2f> &leftPoints, std::vector<cv::Point2f> &rightPoints)
{
	std::vector<cv::KeyPoint> keypointsR;
	cv::Mat descriptorsL, descriptorsR;
	detector.detect(img_r, keypointsR);

	extractor.compute(img_l, keypointsL, descriptorsL);
	extractor.compute(img_r, keypointsR, descriptorsR);

	if ( descriptorsL.empty() )
	{
		printf("WARNING: Left descriptor empty\n");
		return false;
	}
	if ( descriptorsR.empty() )
	{
		printf("WARNING: Right descriptor empty\n");
		return false;
	}

	BFMatcher matcher(cv::NORM_HAMMING, true); // BFMatcher appears to be faster than FlannBasedMatcher
	std::vector< DMatch > matches;
	matcher.match( descriptorsR, descriptorsL, matches );

	if (!matches.empty())
	{
		// sort the matches by distance (i.e. quality)
		sort(matches.begin(), matches.end(), compareMatch);

		for (int i = 0; i < matches.size(); i++)
		{
			leftPoints.push_back(keypointsL[matches[i].trainIdx].pt);
			rightPoints.push_back(keypointsR[matches[i].queryIdx].pt);
		}

		// get sub pixel accurate points
		Size winSize = Size( 5, 5 );
		Size zeroZone = Size( -1, -1 );
		TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );
		cornerSubPix( img_l, leftPoints, winSize, zeroZone, criteria );
		cornerSubPix( img_r, rightPoints, winSize, zeroZone, criteria );
	}

	return true;
}

bool stereoMatchOpticalFlow(const cv::Mat &img_l, const cv::Mat &img_r, std::vector<cv::KeyPoint> &keypointsL, std::vector<cv::Point2f> &leftPoints, std::vector<cv::Point2f> &rightPoints)
{
	if (keypointsL.empty())
		return false;

	std::vector<cv::Point2f> leftPoints_flow, rightPoints_flow;
	for (int i = 0; i < keypointsL.size(); i++)
	{
		leftPoints_flow.push_back(keypointsL[i].pt);
	}
	// get sub pixel accurate points
	Size winSize = Size( 5, 5 );
	Size zeroZone = Size( -1, -1 );
	TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );
	cornerSubPix( img_l, leftPoints_flow, winSize, zeroZone, criteria );

	std::vector<unsigned char> statusRight;
	std::vector<float> error;

	cv::calcOpticalFlowPyrLK(img_l, img_r, leftPoints_flow, rightPoints_flow, statusRight, error, cv::Size(13,13), 4);

	for (int i = 0; i < leftPoints_flow.size(); i++)
	{
		if (statusRight[i])
		{
			leftPoints.push_back(leftPoints_flow[i]);
			rightPoints.push_back(rightPoints_flow[i]);
		}
	}

	return true;
}

bool compareMatch(const DMatch &first, const DMatch &second)
{
	return first.distance < second.distance;
}

bool compareKeypoints(const cv::KeyPoint &first, const cv::KeyPoint &second)
{
	return first.response > second.response;
}

//////////////////////////////////////////////////////////////////////
// INIT MORE POINTS USING KLT
struct GridOrder
{
	GridOrder() : maxScore(0), gx(0), gy(0) {}

	struct PointWithScore
	{
		PointWithScore(double x, double y) : x(x), y(y), score(0) {}
		double x, y, score;
	};

	double maxScore;
	std::vector<PointWithScore> points;
	unsigned char gx, gy;
};

static bool sort_func(const std::pair<size_t, size_t>& lhs, const std::pair<size_t, size_t>& rhs)
{
	return lhs.second < rhs.second;
}

static void initMorePoints_KLT(
		const cv::Mat &img_l,
		const cv::Mat &img_r,
		vector<int> &updateVect,
		vector<double> &z_all_l,
		vector<double> &z_all_r,
		int gridSizeX,
		int gridSizeY,
		double minDistSqr,
		double minScore)
{
	unsigned int targetNewPoints = 0;
	// count the features that need to be initialized
	for (int i = 0; i < updateVect.size(); i++)
	{
		if (updateVect[i] == 2) // 2 means VIO requested stereo measurement
			targetNewPoints++;
	}

	if (!targetNewPoints)
		return;

	if (prev_corners.size() < updateVect.size())
			prev_corners.resize(updateVect.size());
	//	printf("updateVect: %d\n", updateVect.size());
	//	printf("z_all_l: %d\n", z_all_l.size());
	//	printf("z_all_r: %d\n", z_all_r.size());
	//	printf("prev_corners: %d\n", prev_corners.size());

	// count the existing points
	unsigned int existingPoints = 0;
	for (int i = 0; i < prev_status.size(); i++)
	{
		if (prev_status[i] == 1)
			existingPoints++;
	}
	unsigned int targetNumPoints = existingPoints + targetNewPoints;

	unsigned int newAnchorTreshold = 0;  //do not init new points until at least this amount is missing

	//first compute points per grid cell
	std::vector<GridOrder> gridPointCount(gridSizeX*gridSizeY);

	const int gridPixX = (int)img_l.cols / gridSizeX;
	const int gridPixY = (int)img_l.rows / gridSizeY;
	const size_t numPointsPerGridCell = (targetNumPoints-1) / gridPointCount.size() + 1;

	//init gridPointCount
	for(int gy = 0; gy < gridSizeY; gy++)
		for(int gx = 0; gx < gridSizeX; gx++)
		{
			gridPointCount[gy*gridSizeX+gx].gx = gx;
			gridPointCount[gy*gridSizeX+gx].gy = gy;
			gridPointCount[gy*gridSizeX+gx].maxScore = 0;
		}

	//compute number of points in grid
	unsigned int numPoints = 0;
	for(size_t i = 0; i < prev_status.size(); i++)
	{
		if (prev_status[i] > 0)
		{
			double px = prev_corners[i].x;
			double py = prev_corners[i].y;
			int gx = px / gridPixX;
			int gy = py / gridPixY;
			if (gx >= 0 && gx < gridSizeX && gy >= 0 && gy < gridSizeY)
			{
				gridPointCount[gy*gridSizeX+gx].points.push_back(GridOrder::PointWithScore(px, py));
			}
			numPoints++;
		}
	}

	//check if enough points are missing to initialize a new anchor
	if (targetNumPoints - numPoints < newAnchorTreshold) return;

	//then sort the grid cells by occupancy
	std::list<std::pair<size_t, size_t> > gridOrder;
	for (size_t i = 0; i < gridPointCount.size(); i++)
	{
		gridOrder.push_back(std::make_pair(i, gridPointCount[i].points.size()));
	}
	gridOrder.sort(sort_func);

	std::vector<cv::Point2f> newPoints;

	int patchSize = 5;
	int halfpatchSize = patchSize/2;

	//now process the grid cells in ascending occupancy order
	for(std::list<std::pair<size_t, size_t> >::iterator it = gridOrder.begin(); it != gridOrder.end(); ++it)
	{
		size_t gridIdx = it->first;
		int gx = gridPointCount[gridIdx].gx;
		int gy = gridPointCount[gridIdx].gy;

		std::vector<GridOrder::PointWithScore> &curPoints = gridPointCount[gridIdx].points;

		//as long as there can be more points try to add more
		if (targetNumPoints > numPoints && curPoints.size() < numPointsPerGridCell)
		{
			//special handling for gridCells at image border
			int to = halfpatchSize, bo = -halfpatchSize, lo = halfpatchSize, ro = -halfpatchSize;
			if (gx == 0) lo = halfpatchSize+1;
			if (gy == 0) to = halfpatchSize+1;
			if (gx == gridSizeX-1) ro = -(halfpatchSize+1);
			if (gy == gridSizeY-1) bo = -(halfpatchSize+1);

			int gto = gy*gridPixY+to;
			int gbo = (gy+1)*gridPixY+bo;
			int glo = gx*gridPixX+lo;
			int gro = (gx+1)*gridPixX+ro;

			std::vector<cv::Point2f> corners;
			cv::goodFeaturesToTrack(img_l.rowRange(gto, gbo).colRange(glo, gro), corners, numPointsPerGridCell*4, 0.03, 10);

			//sort corners by descending score to first pick the best ones...
			std::vector<std::pair<double, size_t> > score2index(corners.size());
			for (size_t i = 0; i < corners.size(); i++)
			{
				score2index[i] = std::make_pair(shiTomasiScore(img_l, corners[i], patchSize), i);
			}
			sort(score2index.begin(), score2index.end());

			//now check for every corner if they are valid and useful etc...
			for (size_t ii = 0; ii < corners.size(); ii++)
			{
				size_t i = score2index[ii].second;
				corners[i].x += glo;
				corners[i].y += gto;

				//not near existing corners
				bool valid = true;
				for (size_t e = 0; e < prev_status.size(); e++)
				{
					if (prev_status[e] > 0)
					{
						double dx = prev_corners[e].x-corners[i].x;
						double dy = prev_corners[e].y-corners[i].y;
						double dist = dx*dx+dy*dy;
						if (dist < minDistSqr)
						{
							valid = false;
							break;
						}
					}
				}
				if (valid == true)
				{
					curPoints.push_back(GridOrder::PointWithScore(corners[i].x, corners[i].y));
					newPoints.push_back(cv::Point2f(corners[i].x, corners[i].y));
					numPoints++;
					if (numPoints == targetNumPoints || curPoints.size() == numPointsPerGridCell)
						break;
				}
			}
		}
	}

	//now just add all newly added points to the filter data structures

	size_t j = 0;
	for (size_t i = 0; i < newPoints.size(); i++)
	{
		//search for the first free position in the points array
		for (; j < prev_status.size(); j++)
		{
			if (updateVect[j] == 2)
			{
				prev_corners[j] = newPoints[i];
				i++;
			}
		}
		//stop if no more slots for new points
		if (j == targetNumPoints) break;
		j++;
	}

	std::vector<unsigned char> statusRight;
	std::vector<cv::Point2f> newPoints_right;
	std::vector<float> error;
	cv::calcOpticalFlowPyrLK(img_l, img_r, newPoints, newPoints_right, statusRight, error, cv::Size(13,13), 4);

	int newPoints_idx = 0;
	for (int i = 0; i < updateVect.size(); i++)
	{
		if (updateVect[i] == 2)
		{
			while(newPoints_idx < newPoints.size() && statusRight[newPoints_idx] != 1)
			{
				newPoints_idx++;
			}
			if (newPoints_idx < newPoints.size())
			{
				if (statusRight[newPoints_idx] == 1)
				{
					prev_corners[i] = newPoints[newPoints_idx];
					prev_status[i] = 1;

					z_all_l[i*2 + 0] = newPoints[newPoints_idx].x;
					z_all_l[i*2 + 1] = newPoints[newPoints_idx].y;

					z_all_r[i*2 + 0] = newPoints_right[newPoints_idx].x;
					z_all_r[i*2 + 1] = newPoints_right[newPoints_idx].y;
				}
				newPoints_idx++;
			} else {
				updateVect[i] = 0;
			}
		}
	}

}

static double shiTomasiScore(const cv::Mat &image, const cv::Point &position, int halfWindow)
{
	int iXX = 0;
	int iYY = 0;
	int iXY = 0;

	int x_start = position.x - halfWindow;
	int y_start = position.y - halfWindow;
	int x_end = position.x + halfWindow;
	int y_end = position.y + halfWindow;

	if (x_start <= 0 || y_start <= 0 || x_end >= image.cols || y_end >= image.rows) return 0.;

	for(int y = y_start; y < y_end; y++)
		for(int x = x_start; x < x_end; x++)
		{
			int dx = (int)image.at<unsigned char>(y,x+1) - (int)image.at<unsigned char>(y,x-1);
			int dy = (int)image.at<unsigned char>(y+1,x) - (int)image.at<unsigned char>(y-1,x);
			iXX += dx*dx;
			iYY += dy*dy;
			iXY += dx*dy;
		}

	double nPixels_inv = 1. / (2*(x_end - x_start + 1)*(y_end - y_start + 1));
	double dXX = iXX * nPixels_inv;
	double dYY = iYY * nPixels_inv;
	double dXY = iXY * nPixels_inv;

	// Find and return smaller eigenvalue:
	return 0.5 * (dXX + dYY - sqrt( (dXX + dYY) * (dXX + dYY) - 4. * (dXX * dYY - dXY * dXY) ));
}
