//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: undistortPoint.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 07-Oct-2015 10:22:33
//
#ifndef __UNDISTORTPOINT_H__
#define __UNDISTORTPOINT_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void undistortPoint(const double pt_d_data[], const int pt_d_size[1],
  const double cameraparams_FocalLength[2], const double
  cameraparams_PrincipalPoint[2], const double cameraparams_RadialDistortion[3],
  double pt_u_data[], int pt_u_size[1]);

#endif

//
// File trailer for undistortPoint.h
//
// [EOF]
//
