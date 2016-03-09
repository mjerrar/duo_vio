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
 * IMULowpass.cpp
 *
 *  Created on: Feb 24, 2016
 *      Author: nicolas
 */

#include "IMULowpass.h"

IMULowpass::IMULowpass() :
                first_time_(true),
                smoothing_factor_(1.0) {
//  filtered_meas_.acc = {0.0, 0.0, 0.0};
//  filtered_meas_.gyr = {0.0, 0.0, 0.0};
}

IMULowpass::IMULowpass(double smoothing_factor) :
                first_time_(true),
                smoothing_factor_(1.0) {
    setSmoothingFactor(smoothing_factor);
}

IMULowpass::~IMULowpass() {
}

void IMULowpass::setSmoothingFactor(double smoothing_factor) {
    if (smoothing_factor < 0.0)
        smoothing_factor = 0.0;
    if (smoothing_factor > 1.0)
        smoothing_factor = 1.0;
    smoothing_factor_ = smoothing_factor;
}

void IMULowpass::put(const VIOMeasurements &meas) {
    if (first_time_) {
        filtered_meas_ = meas;
        first_time_ = false;
    } else {
        for (int i = 0; i < 3; i++) {
            filtered_meas_.acc[i] = smoothing_factor_ * meas.acc[i] + (1.0 - smoothing_factor_) * filtered_meas_.acc[i];
            filtered_meas_.gyr[i] = smoothing_factor_ * meas.gyr[i] + (1.0 - smoothing_factor_) * filtered_meas_.gyr[i];
        }
    }
}

void IMULowpass::get(VIOMeasurements &meas) {
    meas = filtered_meas_;
}
