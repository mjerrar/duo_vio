//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: initializePoint.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Sep-2015 22:40:16
//
#ifndef __INITIALIZEPOINT_H__
#define __INITIALIZEPOINT_H__

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
extern double d_eml_xnrm2(int n, const double x[30], int ix0);
extern void initializePoint(const emxArray_real_T *b_xt, const double
  c_cameraparams_CameraParameters[3], const double
  d_cameraparams_CameraParameters[2], const double
  e_cameraparams_CameraParameters[2], const double
  f_cameraparams_CameraParameters[3], const double
  g_cameraparams_CameraParameters[2], const double
  h_cameraparams_CameraParameters[2], const double cameraparams_r_lr[3], const
  double cameraparams_R_lr[9], const double z_l[2], const double z_r[2], double
  fp[3], double m_out[3]);

#endif

//
// File trailer for initializePoint.h
//
// [EOF]
//
