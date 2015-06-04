//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: rectifyStereo.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Jun-2015 16:07:13
//
#ifndef __RECTIFYSTEREO_H__
#define __RECTIFYSTEREO_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "rectifyStereo_types.h"

// Function Declarations
extern emxArray_uint8_T *emxCreateND_uint8_T(int numDimensions, int *size);
extern emxArray_uint8_T *emxCreateWrapperND_uint8_T(unsigned char *data, int
  numDimensions, int *size);
extern emxArray_uint8_T *emxCreateWrapper_uint8_T(unsigned char *data, int rows,
  int cols);
extern emxArray_uint8_T *emxCreate_uint8_T(int rows, int cols);
extern void emxDestroyArray_uint8_T(emxArray_uint8_T *emxArray);
extern void emxInitArray_uint8_T(emxArray_uint8_T **pEmxArray, int numDimensions);
extern void rectifyStereo(const unsigned char ImGrayR_lr[360960], const unsigned
  char ImGrayR_rr[360960], const struct0_T *stereoParamStruct, const struct4_T
  *stereoParamsSecondStruct, emxArray_uint8_T *ImGrayR_l, emxArray_uint8_T
  *ImGrayR_r, double cameraparams_out[4]);
extern void rectifyStereo_initialize();
extern void rectifyStereo_terminate();

#endif

//
// File trailer for rectifyStereo.h
//
// [EOF]
//
