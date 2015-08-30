//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_updIT.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 30-Aug-2015 14:58:54
//
#ifndef __SLAM_UPDIT_H__
#define __SLAM_UPDIT_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void SLAM_updIT(emxArray_real_T *P_apr, emxArray_real_T *b_xt, const
  double c_cameraParams_CameraParameters[3], const double
  d_cameraParams_CameraParameters[2], const double
  e_cameraParams_CameraParameters[2], const double
  f_cameraParams_CameraParameters[3], const double
  g_cameraParams_CameraParameters[2], const double
  h_cameraParams_CameraParameters[2], const double cameraParams_r_lr[3], const
  double cameraParams_R_lr[9], const double cameraParams_R_rl[9], double
  updateVect[16], const double z_all_l[32], const double z_all_r[32], const
  double noiseParameters_image_noise[2], double noiseParameters_sigmaInit,
  double c_noiseParameters_orientation_n, double noiseParameters_pressure_noise,
  double noiseParameters_ext_pos_noise, double noiseParameters_ext_att_noise,
  const VIOMeasurements *measurements, double b_height_offset_pressure, const
  VIOParameters b_VIOParameters, emxArray_real_T *h_u_apo, emxArray_real_T *map);
extern void SLAM_updIT_free();
extern void SLAM_updIT_init();

#endif

//
// File trailer for SLAM_updIT.h
//
// [EOF]
//
