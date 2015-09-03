//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_updIT.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 03-Sep-2015 23:49:51
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
  updateVect[24], const double z_all_l[48], const double z_all_r[48], const
  NoiseParameters *noiseParameters, const VIOMeasurements *measurements, double
  b_height_offset_pressure, const VIOParameters b_VIOParameters, emxArray_real_T
  *h_u_apo, emxArray_real_T *map);
extern void SLAM_updIT_free();
extern void SLAM_updIT_init();

#endif

//
// File trailer for SLAM_updIT.h
//
// [EOF]
//
