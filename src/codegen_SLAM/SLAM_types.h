//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_types.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 02-Nov-2015 16:25:43
//
#ifndef __SLAM_TYPES_H__
#define __SLAM_TYPES_H__

// Include Files
#include "rtwtypes.h"

// Type Definitions
typedef struct {
  double acc_duo[3];
  double gyr_duo[3];
} struct0_T;

typedef struct {
  int ATAN;
  int PLUMB_BOB;
  double FocalLength[2];
  double PrincipalPoint[2];
  double RadialDistortion[3];
  int DistortionModel;
} struct2_T;

typedef struct {
  struct2_T CameraParameters1;
  struct2_T CameraParameters2;
  double r_lr[3];
  double R_lr[9];
  double R_rl[9];
  double R_ci[9];
  double t_ci[3];
  double gyro_bias[3];
  double acc_bias[3];
  double time_shift;
} struct1_T;

typedef struct {
  double qv;
  double qw;
  double qao;
  double qwo;
  double qR_ci;
} struct4_T;

typedef struct {
  struct4_T process_noise;
  double gyro_bias_initial_unc[3];
  double acc_bias_initial_unc[3];
  double image_noise;
  double inv_depth_initial_unc;
} struct3_T;

typedef struct {
  int num_points_per_anchor;
  int num_anchors;
  int max_ekf_iterations;
  boolean_T fixed_feature;
  boolean_T delayed_initialization;
  boolean_T mono;
  boolean_T full_stereo;
  boolean_T RANSAC;
} struct5_T;

typedef struct {
  double gyro_bias[3];
  double acc_bias[3];
  double pos[3];
  double att[4];
} struct7_T;

typedef struct {
  double pos[3];
  double att[4];
  double vel[3];
  struct7_T IMU;
} struct6_T;

typedef struct {
  double pos[3];
  double att[4];
} struct8_T;

#endif

//
// File trailer for SLAM_types.h
//
// [EOF]
//
