/*
 * FeatureTracker.h
 *      Abstract base class for all feature trackers
 */

#ifndef VIO_ROS_SRC_FEATURETRACKER_H_
#define VIO_ROS_SRC_FEATURETRACKER_H_

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

class FeatureTracker
{
protected:
	unsigned int num_points;
	cv::Mat prev_img;
	std::vector<unsigned char> prev_status;

public:
	FeatureTracker(unsigned int num_points_) : num_points(num_points_), prev_status(num_points, 0){};
	virtual ~FeatureTracker(){};

	// The interface through which all inheriting feature trackers will be used
	virtual void update(
			const cv::Mat &img_l,
			const cv::Mat &img_r,
			std::vector<double> &z_all_l,
			std::vector<double> &z_all_r,
			std::vector<int> &status) = 0;
};

#endif /* VIO_ROS_SRC_FEATURETRACKER_H_ */
