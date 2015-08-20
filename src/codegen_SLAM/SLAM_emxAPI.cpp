//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_emxAPI.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 20-Aug-2015 15:34:21
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_emxAPI.h"
#include "SLAM_emxutil.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : int b_numDimensions
//                int *b_size
// Return Type  : emxArray_real_T *
//
emxArray_real_T *emxCreateND_real_T(int b_numDimensions, int *b_size)
{
  emxArray_real_T *emx;
  int numEl;
  int i;
  b_emxInit_real_T(&emx, b_numDimensions);
  numEl = 1;
  for (i = 0; i < b_numDimensions; i++) {
    numEl *= b_size[i];
    emx->size[i] = b_size[i];
  }

  emx->data = (double *)calloc((unsigned int)numEl, sizeof(double));
  emx->numDimensions = b_numDimensions;
  emx->allocatedSize = numEl;
  return emx;
}

//
// Arguments    : double *b_data
//                int b_numDimensions
//                int *b_size
// Return Type  : emxArray_real_T *
//
emxArray_real_T *emxCreateWrapperND_real_T(double *b_data, int b_numDimensions,
  int *b_size)
{
  emxArray_real_T *emx;
  int numEl;
  int i;
  b_emxInit_real_T(&emx, b_numDimensions);
  numEl = 1;
  for (i = 0; i < b_numDimensions; i++) {
    numEl *= b_size[i];
    emx->size[i] = b_size[i];
  }

  emx->data = b_data;
  emx->numDimensions = b_numDimensions;
  emx->allocatedSize = numEl;
  emx->canFreeData = false;
  return emx;
}

//
// Arguments    : double *b_data
//                int rows
//                int cols
// Return Type  : emxArray_real_T *
//
emxArray_real_T *emxCreateWrapper_real_T(double *b_data, int rows, int cols)
{
  emxArray_real_T *emx;
  int b_size[2];
  int numEl;
  int i;
  b_size[0] = rows;
  b_size[1] = cols;
  b_emxInit_real_T(&emx, 2);
  numEl = 1;
  for (i = 0; i < 2; i++) {
    numEl *= b_size[i];
    emx->size[i] = b_size[i];
  }

  emx->data = b_data;
  emx->numDimensions = 2;
  emx->allocatedSize = numEl;
  emx->canFreeData = false;
  return emx;
}

//
// Arguments    : int rows
//                int cols
// Return Type  : emxArray_real_T *
//
emxArray_real_T *emxCreate_real_T(int rows, int cols)
{
  emxArray_real_T *emx;
  int b_size[2];
  int numEl;
  int i;
  b_size[0] = rows;
  b_size[1] = cols;
  b_emxInit_real_T(&emx, 2);
  numEl = 1;
  for (i = 0; i < 2; i++) {
    numEl *= b_size[i];
    emx->size[i] = b_size[i];
  }

  emx->data = (double *)calloc((unsigned int)numEl, sizeof(double));
  emx->numDimensions = 2;
  emx->allocatedSize = numEl;
  return emx;
}

//
// Arguments    : emxArray_real_T *emxArray
// Return Type  : void
//
void emxDestroyArray_real_T(emxArray_real_T *emxArray)
{
  emxFree_real_T(&emxArray);
}

//
// Arguments    : emxArray_real_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
void emxInitArray_real_T(emxArray_real_T **pEmxArray, int b_numDimensions)
{
  b_emxInit_real_T(pEmxArray, b_numDimensions);
}

//
// File trailer for SLAM_emxAPI.cpp
//
// [EOF]
//
