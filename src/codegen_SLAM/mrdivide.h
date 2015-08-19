//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mrdivide.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 10:03:40
//
#ifndef __MRDIVIDE_H__
#define __MRDIVIDE_H__

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
extern void b_mrdivide(emxArray_real_T *A, const emxArray_real_T *B);
extern void c_mrdivide(emxArray_real_T *A, const emxArray_real_T *B);
extern double eml_xnrm2(int n, const emxArray_real_T *x, int ix0);
extern void mrdivide(const double A[2], const double B[4], double y[2]);

#endif

//
// File trailer for mrdivide.h
//
// [EOF]
//
