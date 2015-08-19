//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_pred_model.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 11:35:06
//
#ifndef __SLAM_PRED_MODEL_H__
#define __SLAM_PRED_MODEL_H__

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
extern void SLAM_pred_model(struct_T *b_SLAM_data, double dt, const double
  b_processNoise[4], const double IMU_measurements[23]);

#endif

//
// File trailer for SLAM_pred_model.h
//
// [EOF]
//
