//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: predictMeasurementStereoDistorted.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Oct-2015 15:29:53
//
#ifndef __PREDICTMEASUREMENTSTEREODISTORTED_H__
#define __PREDICTMEASUREMENTSTEREODISTORTED_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void c_predictMeasurementStereoDisto(const double fp_l[3], const double
  c_stereoParams_CameraParameters[2], const double
  d_stereoParams_CameraParameters[2], const double
  e_stereoParams_CameraParameters[3], const double
  f_stereoParams_CameraParameters[2], const double
  g_stereoParams_CameraParameters[2], const double
  h_stereoParams_CameraParameters[3], const double stereoParams_r_lr[3], const
  double stereoParams_R_rl[9], double h_d_l[2], double h_d_r[2]);

#endif

//
// File trailer for predictMeasurementStereoDistorted.h
//
// [EOF]
//
