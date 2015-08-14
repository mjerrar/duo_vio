//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_emxAPI.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 14-Aug-2015 15:27:50
//
#ifndef __SLAM_EMXAPI_H__
#define __SLAM_EMXAPI_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
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
// File trailer for SLAM_emxAPI.h
//
// [EOF]
//
