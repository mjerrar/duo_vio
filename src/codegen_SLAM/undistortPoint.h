//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: undistortPoint.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Sep-2015 10:04:04
//
#ifndef __UNDISTORTPOINT_H__
#define __UNDISTORTPOINT_H__

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
extern void undistortPoint(const double pt_d[2], const double
  c_cameraParameters_RadialDistor[3], const double cameraParameters_FocalLength
  [2], const double cameraParameters_PrincipalPoint[2], double pt_u[2]);

#endif

//
// File trailer for undistortPoint.h
//
// [EOF]
//
