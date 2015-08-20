//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_updIT.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 20-Aug-2015 19:17:37
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
  double c_cameraparams_CameraParameters[3], const double
  d_cameraparams_CameraParameters[2], const double
  e_cameraparams_CameraParameters[2], const double
  f_cameraparams_CameraParameters[3], const double
  g_cameraparams_CameraParameters[2], const double
  h_cameraparams_CameraParameters[2], const double cameraparams_r_lr[3], const
  double cameraparams_R_lr[9], const double cameraparams_R_rl[9], double
  updateVect[16], const double z_all_l[32], const double z_all_r[32], const
  double imNoise[2], const double IMU_measurements[23], double
  numPointsPerAnchor, double numAnchors, double height_offset_pressure,
  emxArray_real_T *h_u_apo, emxArray_real_T *map);
extern void SLAM_updIT_free();
extern void SLAM_updIT_init();

#endif

//
// File trailer for SLAM_updIT.h
//
// [EOF]
//
