//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: predictMeasurement_stereo.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 28-Aug-2015 11:22:17
//
#ifndef __PREDICTMEASUREMENT_STEREO_H__
#define __PREDICTMEASUREMENT_STEREO_H__

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
extern void predictMeasurement_stereo(const double fp_l[3], const double
  c_cameraparams_CameraParameters[3], const double
  d_cameraparams_CameraParameters[2], const double
  e_cameraparams_CameraParameters[2], const double
  f_cameraparams_CameraParameters[3], const double
  g_cameraparams_CameraParameters[2], const double
  h_cameraparams_CameraParameters[2], const double cameraparams_r_lr[3], const
  double cameraparams_R_rl[9], double h_u_l[2], double h_u_r[2]);

#endif

//
// File trailer for predictMeasurement_stereo.h
//
// [EOF]
//
