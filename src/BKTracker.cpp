#include "BKTracker.h"

BKTracker::BKTracker(unsigned int num_points_,
		unsigned int detector_threshold_,
		unsigned int descriptor_length_,
		unsigned int KLT_patch_size_,
		unsigned int KLT_levels_)
: FeatureTracker(num_points_),
  detector(detector_threshold_),
  extractor(descriptor_length_),
  prev_corners(num_points_, cv::Point2f(0,0)),
  KLT_patch_size(KLT_patch_size_, KLT_patch_size_),
  KLT_levels(KLT_levels_)
{
}

BKTracker::~BKTracker()
{
}

// Update the feature tracks
// Input        img_l:  The image of the left camera
// Input        img_r:  The image of the right camera
// Output       z_l:    The pixel coordinates of the tracked features in the left image
// Output       z_r:    The pixel coordinates of the tracked features in the right image
// Input/output status: Status of the feature:
//                      0 == lost feature;
//                      1 == tracked feature (only tracked in left image);
//                      2 == new feature (with stereo measurements)

void BKTracker::update(
		const cv::Mat &img_l,
		const cv::Mat &img_r,
		std::vector<double> &z_l,
		std::vector<double> &z_r,
		std::vector<int> &status)
{
	z_l.resize(num_points * 2);
	std::fill(z_l.begin(), z_l.end(), -100.0);

	z_r.resize(num_points * 2);
	std::fill(z_r.begin(), z_r.end(), -100.0);

	// check if VIO rejected any features
	for (size_t i = 0; i < num_points; ++i)
	{
		if(status[i] == 1)
		{
			prev_status[i] = 1;
		} else {
			prev_status[i] = 0; // if status[i] == 0 feature is inactive, == 2 request new feature
		}
	}

	std::vector<unsigned char> new_status;
	std::vector<cv::Point2f> new_corners;
	std::vector<float> error;

	if (!prev_img.empty() && !prev_corners.empty())
	{
		cv::calcOpticalFlowPyrLK(prev_img, img_l, prev_corners, new_corners, new_status, error, KLT_patch_size, KLT_levels);
		prev_corners = new_corners;

		for (size_t i = 0; i < num_points; ++i)
		{
			if(!(prev_status[i] && status[i]))
				prev_status[i] = 0;

			if (prev_status[i] == 1)
			{
				z_l[2*i+0] = prev_corners[i].x;
				z_l[2*i+1] = prev_corners[i].y;
				status[i] = 1;
			} else {
				if (status[i] == 1) // be careful not to overwrite 2s in status
					status[i] = 0;
			}
		}
	}

	img_l.copyTo(prev_img);

	bool init_new_points_with_KLT = false;

	if (!img_r.empty())
	{
		initMorePoints(img_l, img_r, status, z_l, z_r);
	} else {
		printf("Right image is empty!\n");
	}
}

void BKTracker::initMorePoints(
		const cv::Mat &img_l,
		const cv::Mat &img_r,
		std::vector<int> &status,
		std::vector<double> &z_l,
		std::vector<double> &z_r)
{

	// count the features that need to be initialized
	unsigned int targetNumPoints = 0;
	for (int i = 0; i < status.size(); i++)
	{
		if (status[i] == 2) // 2 means VIO requested stereo measurement
			targetNumPoints++;
	}

	if(!targetNumPoints)
		return;

	std::vector<cv::KeyPoint> keypointsL, keypointsR;
	cv::Mat descriptorsL, descriptorsR;

	// find the best features in the left image
	detector.detect(img_l, keypointsL);
	sort(keypointsL.begin(), keypointsL.end(), compareKeypoints);
	keypointsL.erase(keypointsL.begin() + std::min(targetNumPoints*10, (unsigned int) keypointsL.size()), keypointsL.end());


	extractor.compute(img_l, keypointsL, descriptorsL);

	// find features in the right image
	detector.detect(img_r, keypointsR);
	extractor.compute(img_r, keypointsR, descriptorsR);

	if ( descriptorsL.empty() )
	{
		printf("WARNING: Left descriptor empty\n");
		return;
	}
	if ( descriptorsR.empty() )
	{
		printf("WARNING: Right descriptor empty\n");
		return;
	}

	// match the features
	cv::BFMatcher matcher(cv::NORM_HAMMING, true);
	std::vector< cv::DMatch > matches;
	matcher.match(descriptorsR, descriptorsL, matches);

	// sort the matches by distance (i.e. quality)
	std::sort(matches.begin(), matches.end(), compareMatch);

	double max_dist = matches.back().distance;
	double min_dist = matches.front().distance;

	// make sure new features are far enough from existing features and other new features
	std::vector< cv::DMatch > good_matches;

	unsigned int dist = 10; // distance in pixels the feature needs to be away from any existing features
	int new_pt_x, new_pt_y, existing_pt_x, existing_pt_y;
	bool feature_is_distinct;

	for( int i = 0; i < matches.size(); i++ )
	{
		if(matches[i].distance <= std::max(max_dist/2, 0.02) )
		{
			new_pt_x = keypointsL[matches[i].trainIdx].pt.x;
			new_pt_y = keypointsL[matches[i].trainIdx].pt.y;

			feature_is_distinct = true;

			// check against existing features
			for (int j = 0; j < prev_corners.size(); j++)
			{
				existing_pt_x = prev_corners[j].x;
				existing_pt_y = prev_corners[j].y;
				if(abs(existing_pt_x - new_pt_x) < dist && abs(existing_pt_y - new_pt_y) < dist)
				{
					feature_is_distinct = false;
					break;
				}
			}

			if (feature_is_distinct)
			{
				// check against new, already inserted features
				for (int j = 0; j < good_matches.size(); j++)
				{
					existing_pt_x = keypointsL[matches[j].trainIdx].pt.x;
					existing_pt_y = keypointsL[matches[j].trainIdx].pt.y;
					if(abs(existing_pt_x - new_pt_x) < dist && abs(existing_pt_y - new_pt_y) < dist)
					{
						feature_is_distinct = false;
						break;
					}
				}

				if (feature_is_distinct)
				{
					good_matches.push_back(matches[i]);
					if(good_matches.size() == targetNumPoints)
						break;
				}
			}
		} else {
			break; // all following matches will also fail because matches are sorted
		}
	}

	if (good_matches.size() != targetNumPoints)
		printf("Number of good matches: %d, desired: %d\n", (int) good_matches.size(), targetNumPoints);

	int good_matches_idx = 0;
	for (int status_idx = 0; status_idx < status.size(); status_idx++)
	{
		if(status[status_idx] == 2)
		{
			if (good_matches_idx < good_matches.size())
			{
				prev_corners[status_idx] = keypointsL[good_matches[good_matches_idx].trainIdx].pt;
				prev_status[status_idx] = 1;

				z_l[status_idx*2 + 0] = keypointsL[good_matches[good_matches_idx].trainIdx].pt.x;
				z_l[status_idx*2 + 1] = keypointsL[good_matches[good_matches_idx].trainIdx].pt.y;

				z_r[status_idx*2 + 0] = keypointsR[good_matches[good_matches_idx].queryIdx].pt.x;
				z_r[status_idx*2 + 1] = keypointsR[good_matches[good_matches_idx].queryIdx].pt.y;

				good_matches_idx++;
			} else {
				status[status_idx] = 0;
			}
		}
	}
}

bool BKTracker::compareMatch(const cv::DMatch &first, const cv::DMatch &second)
{
	return first.distance < second.distance;
}

bool BKTracker::compareKeypoints(const cv::KeyPoint &first, const cv::KeyPoint &second)
{
	return first.response > second.response;
}

