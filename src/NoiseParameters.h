/*
 * NoiseParameters.h
 *
 *  Created on: Aug 25, 2015
 *      Author: nicolas
 */

#ifndef VIO_ROS_SRC_NOISEPARAMETERS_H_
#define VIO_ROS_SRC_NOISEPARAMETERS_H_


struct NoiseParameters
{
	double process_noise[4];
	double image_noise[2];
	double orientation_noise;
	double pressure_noise;
	double sigmaInit;
};


#endif /* VIO_ROS_SRC_NOISEPARAMETERS_H_ */
