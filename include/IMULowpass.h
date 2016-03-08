/*
 * IMULowpass.h
 *
 *  Created on: Feb 24, 2016
 *      Author: nicolas
 */

#ifndef VIO_ROS_SRC_IMULOWPASS_H_
#define VIO_ROS_SRC_IMULOWPASS_H_

#include "InterfaceStructs.h"
#include "Precision.h"

class IMULowpass {
    bool first_time_;
    double smoothing_factor_;
    VIOMeasurements filtered_meas_;
 public:
    IMULowpass();
    IMULowpass(double smoothing_factor);
    virtual ~IMULowpass();
    void setSmoothingFactor(double smoothing_factor);
    void put(const VIOMeasurements &meas);
    void get(VIOMeasurements &meas);
};

#endif /* VIO_ROS_SRC_IMULOWPASS_H_ */
