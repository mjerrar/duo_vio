//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mrdivide.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 03-Sep-2015 22:44:17
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
extern void b_mrdivide(const double A[2], const double B[4], double y[2]);
extern void mrdivide(const emxArray_real_T *A, const double B_data[], const int
                     B_size[2], emxArray_real_T *y);

#endif

//
// File trailer for mrdivide.h
//
// [EOF]
//
