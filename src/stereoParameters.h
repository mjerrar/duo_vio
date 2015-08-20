/*
 * cameraParameters.h
 *
 *  Created on: Aug 17, 2015
 *      Author: nicolas
 */

#ifndef VIO_ROS_CAMERAPARAMETERS_H_
#define VIO_ROS_CAMERAPARAMETERS_H_

#include <yaml-cpp/yaml.h>

struct cameraParameters //  parameters of one camera
{
	double RadialDistortion[3];
	double TangentialDistortion[2];
	double FocalLength[2];
	double PrincipalPoint[2];
};

struct stereoParameters // parameters of both cameras plus stereo calibration
{
	cameraParameters CameraParameters1;
	cameraParameters CameraParameters2;

	double r_lr[3];
	double R_lr[9];
	double R_rl[9];
};

inline stereoParameters parseYaml(const YAML::Node& node)
{
	stereoParameters v;

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

#endif /* VIO_ROS_CAMERAPARAMETERS_H_ */
