//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_pred_euler.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Oct-2015 15:29:53
//
#ifndef __SLAM_PRED_EULER_H__
#define __SLAM_PRED_EULER_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void SLAM_pred_euler(emxArray_real_T *P_apo, f_struct_T *x, double dt,
  double processNoise_qv, double processNoise_qw, double processNoise_qao,
  double processNoise_qwo, double processNoise_qR_ci, const double
  measurements_acc_duo[3], const double measurements_gyr_duo[3], double
  c_numStates);

#endif

//
// File trailer for SLAM_pred_euler.h
//
// [EOF]
//
