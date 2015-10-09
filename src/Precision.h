/*
 * PrecisionSwitch.h
 *
 *  Created on: Oct 8, 2015
 *      Author: nicolas
 */

#ifndef VIO_ROS_SRC_PRECISION_H_
#define VIO_ROS_SRC_PRECISION_H_

//#define SINGLE_PRECISION // uncomment this line for single precision

#ifdef SINGLE_PRECISION
typedef float FloatType;
#else
typedef double FloatType;
#endif


#endif /* VIO_ROS_SRC_PRECISION_H_ */
