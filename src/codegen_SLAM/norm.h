//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: norm.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 25-Aug-2015 18:09:06
//
#ifndef __NORM_H__
#define __NORM_H__

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
extern double b_norm(const double x[4]);
extern double c_norm(const double x[2]);
extern double d_norm(const emxArray_real_T *x);
extern double norm(const double x[3]);

#endif

//
// File trailer for norm.h
//
// [EOF]
//
