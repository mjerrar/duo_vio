//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Att_pred.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 30-Aug-2015 16:19:29
//
#ifndef __ATT_PRED_H__
#define __ATT_PRED_H__

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
extern void Att_pred(const double x_data[], const double P_data[], const int
                     P_size[2], const double w[3], double q, double dt, double
                     x[4], double b_P[9]);

#endif

//
// File trailer for Att_pred.h
//
// [EOF]
//
