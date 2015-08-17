//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_pred.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 17-Aug-2015 20:14:43
//
#ifndef __SLAM_PRED_H__
#define __SLAM_PRED_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void SLAM_pred(emxArray_real_T *P_apo, emxArray_real_T *x, double dt,
                      const double processNoise[4], const double
                      IMU_measurements[23], double c_numStates);

#endif

//
// File trailer for SLAM_pred.h
//
// [EOF]
//
