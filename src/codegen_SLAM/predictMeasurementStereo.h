//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: predictMeasurementStereo.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Oct-2015 15:29:53
//
#ifndef __PREDICTMEASUREMENTSTEREO_H__
#define __PREDICTMEASUREMENTSTEREO_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void predictMeasurementStereo(const double fp_l[3], const double
  c_stereoParams_CameraParameters[2], const double
  d_stereoParams_CameraParameters[2], const double
  e_stereoParams_CameraParameters[2], const double
  f_stereoParams_CameraParameters[2], const double stereoParams_r_lr[3], const
  double stereoParams_R_rl[9], double h_cin_l[2], double h_cin_r[2]);

#endif

//
// File trailer for predictMeasurementStereo.h
//
// [EOF]
//
