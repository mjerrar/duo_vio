//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: undistortPoint.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 31-Aug-2015 09:51:22
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
extern void undistortPoint(const double pt_d_data[], const int pt_d_size[1],
  const double c_cameraParameters_CameraParame[3], const double
  d_cameraParameters_CameraParame[2], const double
  e_cameraParameters_CameraParame[2], double pt_u_data[], int pt_u_size[1]);

#endif

//
// File trailer for undistortPoint.h
//
// [EOF]
//
