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
 * VIO.h
 *
 *  Created on: Mar 10, 2016
 *      Author: nicolas
 */

#ifndef SRC_VIO_H_
#define SRC_VIO_H_

#include <vector>

#include "Precision.h"
#include "SLAM_includes.h"
#include "matlab_consts.h"

class VIO {
    bool reset_;
    bool params_set_;
    bool is_initialized_;

    DUOParameters duoParam_;
    NoiseParameters noiseParam_;
    VIOParameters vioParam_;

    std::vector<int> int_dummy_;
    std::vector<FloatType> float_dummy_;
    std::vector<AnchorPose> anchor_poses_dummy_;
    VIOMeasurements vio_eas_dummy_;
    RobotState robot_state_dummy_;

public:
    VIO();
    VIO(DUOParameters duoParam, NoiseParameters noiseParam, VIOParameters vioParam);
    virtual ~VIO();
    void predict(const VIOMeasurements &meas, double dt);
    void update(std::vector<int> &update_vect, std::vector<FloatType> &feautres_l, std::vector<FloatType> &feautres_r, RobotState &robotState,
            std::vector<FloatType> &map, std::vector<AnchorPose> &anchor_poses, std::vector<FloatType> &delayedStatus);
    void reset();
    bool getParams(DUOParameters &duoParam, NoiseParameters &noiseParam, VIOParameters &vioParam);
    void setParams(DUOParameters duoParam, NoiseParameters noiseParam, VIOParameters vioParam);
};

#endif /* SRC_VIO_H_ */
