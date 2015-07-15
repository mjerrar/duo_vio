//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getJacobianAndResidual_std.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 15-Jul-2015 17:00:42
//
#ifndef __GETJACOBIANANDRESIDUAL_STD_H__
#define __GETJACOBIANANDRESIDUAL_STD_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void getJacobianAndResidual_std(const emxArray_real_T *b_xt, double
  errorStateSize, const double z_all_l[64], const double z_all_r[64], const
  double b_map[96], const double indMeas_data[], const int indMeas_size[1],
  const double cameraparams_r_lr[3], const double cameraparams_R_lr[9], double
  r_data[], int r_size[1], emxArray_real_T *H_xc, double h_u_data[], int
  h_u_size[1]);

#endif

//
// File trailer for getJacobianAndResidual_std.h
//
// [EOF]
//
