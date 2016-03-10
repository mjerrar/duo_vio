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
 * VIO.cpp
 *
 *  Created on: Mar 10, 2016
 *      Author: nicolas
 */

#include "VIO.h"

VIO::VIO() :
                reset_(false),
                params_set_(false),
                is_initialized_(true) {
    SLAM_initialize();
    int_dummy_.resize(matlab_consts::numTrackFeatures, 0);
    float_dummy_.resize(matlab_consts::numTrackFeatures * 2, 0);
    anchor_poses_dummy_.resize(matlab_consts::numAnchors);
}

VIO::VIO(DUOParameters duoParam, NoiseParameters noiseParam, VIOParameters vioParam) :
                duoParam_(duoParam),
                noiseParam_(noiseParam),
                vioParam_(vioParam_),
                reset_(false),
                params_set_(true) {
    if (!is_initialized_) {
        SLAM_initialize();
        is_initialized_ = true;
    }
    int_dummy_.resize(matlab_consts::numTrackFeatures, 0);
    float_dummy_.resize(matlab_consts::numTrackFeatures * 2, 0);
    anchor_poses_dummy_.resize(matlab_consts::numAnchors);
}

VIO::~VIO() {
    SLAM_terminate();
}

void VIO::predict(const VIOMeasurements &meas, double dt) {
    if (!params_set_)
        throw "VIO parameters not set yet";

    SLAM(&int_dummy_[0], &float_dummy_[0], &float_dummy_[0], dt, &meas, &duoParam_, &noiseParam_, &vioParam_, false, reset_, &robot_state_dummy_,
            &float_dummy_[0], &anchor_poses_dummy_[0], &float_dummy_[0]);
    reset_ = false;

}

void VIO::update(std::vector<int> &update_vect, std::vector<FloatType> &feautres_l, std::vector<FloatType> &feautres_r, RobotState &robotState,
        std::vector<FloatType> &map, std::vector<AnchorPose> &anchor_poses, std::vector<FloatType> &delayedStatus) {

    if (!params_set_)
        throw "VIO parameters not set yet";

    assert(update_vect.size() == matlab_consts::numTrackFeatures);
    assert(feautres_l.size() == matlab_consts::numTrackFeatures * 2);
    assert(feautres_r.size() == matlab_consts::numTrackFeatures * 2);
    assert(anchor_poses.size() == matlab_consts::numAnchors);
    assert(delayedStatus.size() == matlab_consts::numTrackFeatures);

    SLAM(&update_vect[0], &feautres_l[0], &feautres_r[0], 0.0, &vio_eas_dummy_, &duoParam_, &noiseParam_, &vioParam_, true, false, &robotState, &map[0],
            &anchor_poses[0], &delayedStatus[0]);

}

void VIO::reset() {
    reset_ = true;
}

bool VIO::getParams(DUOParameters &duoParam, NoiseParameters &noiseParam, VIOParameters &vioParam) {
    if (!params_set_)
        return false;

    duoParam = duoParam_;
    noiseParam = noiseParam_;
    vioParam = vioParam_;

    return true;
}

void VIO::setParams(DUOParameters duoParam, NoiseParameters noiseParam, VIOParameters vioParam) {
    duoParam_ = duoParam;
    noiseParam_ = noiseParam;
    vioParam_ = vioParam;

    params_set_ = true;
}
