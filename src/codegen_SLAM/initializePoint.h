//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: initializePoint.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 07-Oct-2015 10:22:33
//
#ifndef __INITIALIZEPOINT_H__
#define __INITIALIZEPOINT_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void initializePoint(const double z_u_l[2], const double z_u_r[2], const
  double c_cameraparams_CameraParameters[2], const double
  d_cameraparams_CameraParameters[2], const double
  e_cameraparams_CameraParameters[2], const double
  f_cameraparams_CameraParameters[2], const double cameraparams_r_lr[3], const
  double cameraparams_R_lr[9], double fp[3], double b_m[6], boolean_T *success);

#endif

//
// File trailer for initializePoint.h
//
// [EOF]
//
