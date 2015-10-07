//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_emxutil.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 07-Oct-2015 10:22:33
//
#ifndef __SLAM_EMXUTIL_H__
#define __SLAM_EMXUTIL_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void b_emxFreeStruct_struct_T(f_struct_T *pStruct);
extern void b_emxInitStruct_struct_T(e_struct_T *pStruct);
extern void b_emxInit_int32_T(emxArray_int32_T **pEmxArray, int b_numDimensions);
extern void b_emxInit_real_T(emxArray_real_T **pEmxArray, int b_numDimensions);
extern void c_emxFree_struct_T(emxArray_c_struct_T **pEmxArray);
extern void c_emxInit_struct_T(emxArray_c_struct_T **pEmxArray, int
  b_numDimensions);
extern void emxCopyStruct_struct_T(e_struct_T *dst, const e_struct_T *src);
extern void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
extern void emxEnsureCapacity_struct_T(emxArray_b_struct_T *emxArray, int
  oldNumel);
extern void emxFreeStruct_struct_T(e_struct_T *pStruct);
extern void emxFree_AnchorPose(emxArray_AnchorPose **pEmxArray);
extern void emxFree_boolean_T(emxArray_boolean_T **pEmxArray);
extern void emxFree_int32_T(emxArray_int32_T **pEmxArray);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxInitStruct_struct_T(f_struct_T *pStruct);
extern void emxInit_AnchorPose(emxArray_AnchorPose **pEmxArray, int
  b_numDimensions);
extern void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int
  b_numDimensions);
extern void emxInit_int32_T(emxArray_int32_T **pEmxArray, int b_numDimensions);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int b_numDimensions);

#endif

//
// File trailer for SLAM_emxutil.h
//
// [EOF]
//
