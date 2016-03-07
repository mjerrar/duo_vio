/*
 * BKTracker.h
 *      Point tracker that initializes new stereo features using Fast/Brisk
 *      and tracks features over time in the left camera using KLT.
 */

#ifndef VIO_ROS_SRC_BKTRACKER_H_
#define VIO_ROS_SRC_BKTRACKER_H_

#include "FeatureTracker.h"

#include <algorithm>
#include <list>
#include <vector>

#include <iostream>

class BKTracker : public FeatureTracker
{
private:
	cv::Mat prev_img;
	std::vector<cv::Point2f> prev_corners;
	cv::FastFeatureDetector detector;
	cv::BriefDescriptorExtractor extractor;

	cv::Size KLT_patch_size;
	unsigned int KLT_levels;

public:
	BKTracker(unsigned int num_points_,
			unsigned int detector_threshold_,
			unsigned int descriptor_length_,
			unsigned int KLT_patch_size_,
			unsigned int KLT_levels_);
	~BKTracker();

	void update(const cv::Mat &img_l,
				const cv::Mat &img_r,
				std::vector<double> &z_all_l,
				std::vector<double> &z_all_r,
				std::vector<int> &status);

private:
	void initMorePoints(
			const cv::Mat &img_l,
			const cv::Mat &img_r,
			std::vector<int> &status,
			std::vector<double> &z_l,
			std::vector<double> &z_r);

	static bool compareMatch(const cv::DMatch &first, const cv::DMatch &second);

	static bool compareKeypoints(const cv::KeyPoint &first, const cv::KeyPoint &second);
};

#endif /* VIO_ROS_SRC_BKTRACKER_H_ */
