//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: any.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 28-Aug-2015 20:07:15
//
#ifndef __ANY_H__
#define __ANY_H__

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
extern boolean_T any(const boolean_T x[16]);
extern void b_any(const emxArray_boolean_T *x, boolean_T y[16]);
extern boolean_T c_any(const boolean_T x[3]);
extern boolean_T d_any(const emxArray_boolean_T *x);

#endif

//
// File trailer for any.h
//
// [EOF]
//
