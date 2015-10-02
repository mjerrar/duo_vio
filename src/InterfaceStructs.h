/*
 * InterfaceStructs.h
 *
 *  Created on: Aug 25, 2015
 *      Author: nicolas
 */

#ifndef VIO_ROS_SRC_INTERFACESTRUCTS_H_
#define VIO_ROS_SRC_INTERFACESTRUCTS_H_

#include <yaml-cpp/yaml.h>
#include <ros/console.h>


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
};

// ControllerGains
// =========================================================
struct ControllerGains
{
	double Kp_xy;
	double Ki_xy;
	double Kd_xy;
	double Kp_z;
	double Ki_z;
	double Kd_z;
	double Kp_yaw;
	double Kd_yaw;
	double i_lim;
};

// ProcessNoise
// =========================================================
struct ProcessNoise
{
	double qv;
	double qw;
	double qao;
	double qwo;
	double qR_ci;
};

// NoiseParamters
// =========================================================
struct NoiseParameters
{
	ProcessNoise process_noise;
	double image_noise[2];
	double sigmaInit;
};

// VIOMeasurements
// =========================================================
struct VIOMeasurements
{
	double gyr_duo[3];
	double acc_duo[3];
};

// IMUOffset
// =========================================================
struct IMUState
{
	double pos[3];
	double att[4];
	double gyro_bias[3];
	double acc_bias[3];
};

// RobotState
// =========================================================
struct RobotState
{
	double pos[3];
	double att[4];
	double vel[3];
	IMUState IMU;
};

// AnchorPose
// =========================================================
struct AnchorPose
{
	double pos[3];
	double att[4];
};

// ReferenceCommand
// =========================================================
struct ReferenceCommand
{
	double position[4]; // x, y, z, yaw
	double velocity[4];
};

// cameraParameters
// =========================================================
struct CameraParameters //  parameters of one camera
{
	double RadialDistortion[3];
	double TangentialDistortion[2];
	double FocalLength[2];
	double PrincipalPoint[2];
};

struct DUOParameters // parameters of both cameras plus stereo calibration
{
	CameraParameters CameraParameters1;
	CameraParameters CameraParameters2;

	double r_lr[3];
	double R_lr[9];
	double R_rl[9];
	double R_ci[9];
	double t_ci[3];
	double gyro_bias[3];
	double acc_bias[3];
	double time_shift;

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
			v.r_lr[i] = t_ci[i][0].as<double>();
		}
	} else {
		ROS_WARN("Did not find t_ci, using default");
		for (std::size_t i = 0; i < 3; i++)
		{
			v.r_lr[i] = 0.0;
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


	// camera 1
	YAML::Node RadialDistortion = node["CameraParameters1"]["RadialDistortion"];
	for (std::size_t i = 0; i < RadialDistortion.size(); i++)
	{
		v.CameraParameters1.RadialDistortion[i] = RadialDistortion[i].as<double>();
	}

	YAML::Node TangentialDistortion = node["CameraParameters1"]["TangentialDistortion"];
	for (std::size_t i = 0; i < TangentialDistortion.size(); i++)
	{
		v.CameraParameters1.TangentialDistortion[i] = TangentialDistortion[i].as<double>();
	}

	YAML::Node FocalLength = node["CameraParameters1"]["FocalLength"];
	for (std::size_t i = 0; i < FocalLength.size(); i++)
	{
		v.CameraParameters1.FocalLength[i] = FocalLength[i].as<double>();
	}

	YAML::Node PrincipalPoint = node["CameraParameters1"]["PrincipalPoint"];
	for (std::size_t i = 0; i < PrincipalPoint.size(); i++)
	{
		v.CameraParameters1.PrincipalPoint[i] = PrincipalPoint[i].as<double>();
	}

	// camera 2
	RadialDistortion = node["CameraParameters2"]["RadialDistortion"];
	for (std::size_t i = 0; i < RadialDistortion.size(); i++)
	{
		v.CameraParameters2.RadialDistortion[i] = RadialDistortion[i].as<double>();
	}

	TangentialDistortion = node["CameraParameters2"]["TangentialDistortion"];
	for (std::size_t i = 0; i < TangentialDistortion.size(); i++)
	{
		v.CameraParameters2.TangentialDistortion[i] = TangentialDistortion[i].as<double>();
	}

	FocalLength = node["CameraParameters2"]["FocalLength"];
	for (std::size_t i = 0; i < FocalLength.size(); i++)
	{
		v.CameraParameters2.FocalLength[i] = FocalLength[i].as<double>();
	}

	PrincipalPoint = node["CameraParameters2"]["PrincipalPoint"];
	for (std::size_t i = 0; i < PrincipalPoint.size(); i++)
	{
		v.CameraParameters2.PrincipalPoint[i] = PrincipalPoint[i].as<double>();
	}

	return v;
}




#endif /* VIO_ROS_SRC_INTERFACESTRUCTS_H_ */
