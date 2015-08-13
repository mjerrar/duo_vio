//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 13-Aug-2015 19:21:30
//
#ifndef __SLAM_H__
#define __SLAM_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Type Definitions
#include <stdio.h>

// Function Declarations
extern void SLAM(double updateVect[16], const double z_all_l[32], const double
                 z_all_r[32], double dt, const double processNoise[4], double
                 IMU_measurements[23], const double imNoise[2], double
                 numPointsPerAnchor, double numAnchors, emxArray_real_T
                 *h_u_apo_out, emxArray_real_T *xt_out, emxArray_real_T
                 *P_apo_out, emxArray_real_T *map_out);
extern void SLAM_initialize();
extern void SLAM_terminate();
extern emxArray_real_T *emxCreateND_real_T(int b_numDimensions, int *b_size);
extern emxArray_real_T *emxCreateWrapperND_real_T(double *b_data, int
  b_numDimensions, int *b_size);
extern emxArray_real_T *emxCreateWrapper_real_T(double *b_data, int rows, int
  cols);
extern emxArray_real_T *emxCreate_real_T(int rows, int cols);
extern void emxDestroyArray_real_T(emxArray_real_T *emxArray);
extern void emxInitArray_real_T(emxArray_real_T **pEmxArray, int b_numDimensions);

#endif

//
// File trailer for SLAM.h
//
// [EOF]
//
