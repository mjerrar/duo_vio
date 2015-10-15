/*
 * InterfaceStructs.h
 *
 *  Created on: Aug 25, 2015
 *      Author: nicolas
 */

#ifndef VIO_ROS_SRC_INTERFACESTRUCTS_H_
#define VIO_ROS_SRC_INTERFACESTRUCTS_H_

#include <yaml-cpp/yaml.h>
#include <string>
#include <ros/console.h>
#include "Precision.h"


// VIOParameters
// =========================================================
struct VIOParameters
{
	int num_points_per_anchor;
	int num_anchors;
	int max_ekf_iterations;
	bool fixed_feature;
	bool delayed_initialization;
	bool mono;
	bool RANSAC;
};

// ControllerGains
// =========================================================
struct ControllerGains
{
	FloatType Kp_xy;
	FloatType Ki_xy;
	FloatType Kd_xy;
	FloatType Kp_z;
	FloatType Ki_z;
	FloatType Kd_z;
	FloatType Kp_yaw;
	FloatType Kd_yaw;
	FloatType i_lim;
};

// ProcessNoise
// =========================================================
struct ProcessNoise
{
	FloatType qv;
	FloatType qw;
	FloatType qao;
	FloatType qwo;
	FloatType qR_ci;
};

// NoiseParamters
// =========================================================
struct NoiseParameters
{
	ProcessNoise process_noise;
	FloatType image_noise;
	FloatType inv_depth_initial_unc;
	FloatType gyro_bias_initial_unc[3];
	FloatType acc_bias_initial_unc[3];
};

// VIOMeasurements
// =========================================================
struct VIOMeasurements
{
	FloatType gyr_duo[3];
	FloatType acc_duo[3];
};

// IMUOffset
// =========================================================
struct IMUState
{
	FloatType pos[3];
	FloatType att[4];
	FloatType gyro_bias[3];
	FloatType acc_bias[3];
};

// RobotState
// =========================================================
struct RobotState
{
	FloatType pos[3];
	FloatType att[4];
	FloatType vel[3];
	IMUState IMU;
};

// AnchorPose
// =========================================================
struct AnchorPose
{
	FloatType pos[3];
	FloatType att[4];
};

// ReferenceCommand
// =========================================================
struct ReferenceCommand
{
	FloatType position[4]; // x, y, z, yaw
	FloatType velocity[4];
};

// Kalibr_params
// =========================================================
struct Kalibr_params // parameters used in Kalibr
{
	FloatType update_rate;
	FloatType accelerometer_noise_density;
	FloatType accelerometer_random_walk;
	FloatType gyroscope_noise_density;
	FloatType gyroscope_random_walk;
};

// cameraParameters
// =========================================================
struct CameraParameters // parameters of one camera
{
	FloatType RadialDistortion[3];
	FloatType TangentialDistortion[2];
	FloatType FocalLength[2];
	FloatType PrincipalPoint[2];
	enum {PLUMB_BOB = 0, ATAN = 1};
	int DistortionModel;
};

struct DUOParameters // parameters of both cameras plus stereo calibration
{
	CameraParameters CameraParameters1;
	CameraParameters CameraParameters2;

	FloatType r_lr[3];
	FloatType R_lr[9];
	FloatType R_rl[9];
	FloatType R_ci[9];
	FloatType t_ci[3];
	FloatType gyro_bias[3];
	FloatType acc_bias[3];
	FloatType time_shift;

	Kalibr_params kalibr_params;

};

inline DUOParameters parseYaml(const YAML::Node& node)
{
	DUOParameters v;

	YAML::Node r_lr = node["r_lr"];

	for (std::size_t i = 0; i < r_lr.size(); i++)
	{
		v.r_lr[i] = r_lr[i][0].as<double>();
	}

	YAML::Node R_lr = node["R_lr"];
	for (std::size_t i = 0; i < R_lr.size(); i++)
	{
		YAML::Node row = R_lr[i];
		for (std::size_t j = 0; j < row.size(); j++) // matlab is column major
			v.R_lr[i + j*3] = row[j].as<double>();
	}

	YAML::Node R_rl = node["R_rl"];
	for (std::size_t i = 0; i < R_rl.size(); i++)
	{
		YAML::Node row = R_rl[i];
		for (std::size_t j = 0; j < row.size(); j++) // matlab is column major
			v.R_rl[i + j*3] = row[j].as<double>();
	}

	if(const YAML::Node R_ci = node["R_ci"]) {
		for (std::size_t i = 0; i < R_ci.size(); i++)
		{
			YAML::Node row = R_ci[i];
			for (std::size_t j = 0; j < row.size(); j++) // matlab is column major
				v.R_ci[i + j*3] = row[j].as<double>();
		}
	} else {
		ROS_WARN("Did not find R_ci, using default");
		for (std::size_t i = 0; i < 3; i++)
		{
			for (std::size_t j = 0; j < 3; j++)
			{
				if (i == j)
					v.R_ci[i + j*3] = 1.0;
				else
					v.R_ci[i + j*3] = 0.0;
			}
		}
	}

	if(const YAML::Node t_ci = node["t_ci"]) {
		for (std::size_t i = 0; i < t_ci.size(); i++)
		{
			v.t_ci[i] = t_ci[i][0].as<double>();
		}
	} else {
		ROS_WARN("Did not find t_ci, using default");
		for (std::size_t i = 0; i < 3; i++)
		{
			v.t_ci[i] = 0.0;
		}
	}

	if(const YAML::Node gyro_bias = node["gyro_bias"]) {
		for (std::size_t i = 0; i < gyro_bias.size(); i++)
		{
			v.gyro_bias[i] = gyro_bias[i][0].as<double>();
		}
	} else {
		ROS_WARN("Did not find gyro_bias, using default");
		for (std::size_t i = 0; i < 3; i++)
		{
			v.gyro_bias[i] = 0.0;
		}
	}

	if(const YAML::Node acc_bias = node["acc_bias"]) {
		for (std::size_t i = 0; i < acc_bias.size(); i++)
		{
			v.acc_bias[i] = acc_bias[i][0].as<double>();
		}
	} else {
		ROS_WARN("Did not find acc_bias, using default");
		for (std::size_t i = 0; i < 3; i++)
		{
			v.acc_bias[i] = 0.0;
		}
	}

	if(const YAML::Node time_shift = node["time_shift"]) {
		v.time_shift = time_shift.as<double>();
	} else {
		ROS_WARN("Did not find time_shift, using default");
		v.time_shift = 0.0;
	}

	std::string plumb_bob = "plumb_bob";
	std::string atan = "atan";


	// camera 1
	YAML::Node DistortionModel = node["CameraParameters1"]["DistortionModel"];
	if (!atan.compare(DistortionModel.as<std::string>()))
		v.CameraParameters1.DistortionModel = v.CameraParameters1.ATAN;
	else
		v.CameraParameters1.DistortionModel = v.CameraParameters1.PLUMB_BOB;

	YAML::Node RadialDistortion = node["CameraParameters1"]["RadialDistortion"];
	for (std::size_t i = 0; i < RadialDistortion.size(); i++)
		v.CameraParameters1.RadialDistortion[i] = RadialDistortion[i].as<double>();
	for (std::size_t i = RadialDistortion.size(); i < 3; i++)
		v.CameraParameters1.RadialDistortion[i] = 0.0;

	YAML::Node TangentialDistortion = node["CameraParameters1"]["TangentialDistortion"];
	for (std::size_t i = 0; i < TangentialDistortion.size(); i++)
		v.CameraParameters1.TangentialDistortion[i] = TangentialDistortion[i].as<double>();

	YAML::Node FocalLength = node["CameraParameters1"]["FocalLength"];
	for (std::size_t i = 0; i < FocalLength.size(); i++)
		v.CameraParameters1.FocalLength[i] = FocalLength[i].as<double>();

	YAML::Node PrincipalPoint = node["CameraParameters1"]["PrincipalPoint"];
	for (std::size_t i = 0; i < PrincipalPoint.size(); i++)
		v.CameraParameters1.PrincipalPoint[i] = PrincipalPoint[i].as<double>();

	// camera 2
	DistortionModel = node["CameraParameters2"]["DistortionModel"];
	if (!atan.compare(DistortionModel.as<std::string>()))
		v.CameraParameters2.DistortionModel = v.CameraParameters2.ATAN;
	else
		v.CameraParameters2.DistortionModel = v.CameraParameters2.PLUMB_BOB;

	RadialDistortion = node["CameraParameters2"]["RadialDistortion"];
	for (std::size_t i = 0; i < RadialDistortion.size(); i++)
		v.CameraParameters2.RadialDistortion[i] = RadialDistortion[i].as<double>();
	for (std::size_t i = RadialDistortion.size(); i < 3; i++)
		v.CameraParameters2.RadialDistortion[i] = 0.0;

	TangentialDistortion = node["CameraParameters2"]["TangentialDistortion"];
	for (std::size_t i = 0; i < TangentialDistortion.size(); i++)
		v.CameraParameters2.TangentialDistortion[i] = TangentialDistortion[i].as<double>();

	FocalLength = node["CameraParameters2"]["FocalLength"];
	for (std::size_t i = 0; i < FocalLength.size(); i++)
		v.CameraParameters2.FocalLength[i] = FocalLength[i].as<double>();

	PrincipalPoint = node["CameraParameters2"]["PrincipalPoint"];
	for (std::size_t i = 0; i < PrincipalPoint.size(); i++)
		v.CameraParameters2.PrincipalPoint[i] = PrincipalPoint[i].as<double>();

	if(const YAML::Node n = node["Kalibr_params"]["update_rate"])
		v.kalibr_params.update_rate = n.as<double>();

	if(const YAML::Node n = node["Kalibr_params"]["accelerometer_noise_density"])
		v.kalibr_params.accelerometer_noise_density = n.as<double>();

	if(const YAML::Node n = node["Kalibr_params"]["accelerometer_random_walk"])
		v.kalibr_params.accelerometer_random_walk = n.as<double>();

	if(const YAML::Node n = node["Kalibr_params"]["gyroscope_noise_density"])
		v.kalibr_params.gyroscope_noise_density = n.as<double>();

	if(const YAML::Node n = node["Kalibr_params"]["gyroscope_random_walk"])
		v.kalibr_params.gyroscope_random_walk = n.as<double>();

	return v;
}




#endif /* VIO_ROS_SRC_INTERFACESTRUCTS_H_ */
