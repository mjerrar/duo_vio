//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 30-Jun-2015 14:23:01
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

// Function Declarations
extern void SLAM(double updateVect[32], const double z_all[96], const double
                 cameraparams[4], double dt, const double processNoise[3], const
                 double IMU_measurements[9], const double imNoise[3], double
                 numPointsPerAnchor, double numAnchors, double h_u_apo[96],
                 emxArray_real_T *xt_out, emxArray_real_T *P_apo_out, double
                 b_map[96]);
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
