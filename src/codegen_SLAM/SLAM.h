//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 11-Jun-2015 14:38:13
//
#ifndef __SLAM_H__
#define __SLAM_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void SLAM(const double updateVect[16], const double z_all[48], const
                 double cameraparams[4], double dt, const double processNoise[4],
                 const double IMU_measurements[9], const double imNoise[3],
                 double numPointsPerAnchor, double numAnchors, emxArray_real_T
                 *h_u_apo, emxArray_real_T *xt_out, double updateVect_out[16],
                 emxArray_real_T *anchor_u_out, emxArray_real_T *anchor_pose_out,
                 emxArray_real_T *P_apo_out);
extern void SLAM_initialize();
extern void SLAM_terminate();
extern emxArray_real_T *emxCreateND_real_T(int numDimensions, int *size);
extern emxArray_real_T *emxCreateWrapperND_real_T(double *data, int
  numDimensions, int *size);
extern emxArray_real_T *emxCreateWrapper_real_T(double *data, int rows, int cols);
extern emxArray_real_T *emxCreate_real_T(int rows, int cols);
extern void emxDestroyArray_real_T(emxArray_real_T *emxArray);
extern void emxInitArray_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#endif

//
// File trailer for SLAM.h
//
// [EOF]
//
