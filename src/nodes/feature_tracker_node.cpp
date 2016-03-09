/****************************************************************************
 *
 *   Copyright (c) 2015-2016 AIT, ETH Zurich. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name AIT nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/*
 * feature_tracker_node.cpp
 *
 *  Created on: Mar 9, 2016
 *      Author: nicolas
 */

#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include "Precision.h"
#include "trackFeatures.h"
#include "vio_ros/TrackFeatures.h"

bool track(vio_ros::TrackFeatures::Request &req, vio_ros::TrackFeatures::Response &res);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "feature_tracker");

    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("track_features", track);
    ROS_INFO("Ready to track features.");

    ros::spin();
    return 0;
}


bool track(vio_ros::TrackFeatures::Request &req, vio_ros::TrackFeatures::Response &res)
{
    cv_bridge::CvImagePtr left_image;
    cv_bridge::CvImagePtr right_image;

    try {
        left_image = cv_bridge::toCvCopy(req.left_image, "mono8");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Error while converting left image to OpenCV: %s", e.what());
    }
    try {
        right_image = cv_bridge::toCvCopy(req.right_image, "mono8");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Error while converting right image to OpenCV: %s", e.what());
    }

    trackFeatures(left_image->image, right_image->image, res.features_l, res.features_r, req.update_vect, req.stereo);
    res.update_vect = req.update_vect;
    return true;
}
