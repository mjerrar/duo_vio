//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Att_pred.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 31-Jul-2015 14:58:50
//
#ifndef __ATT_PRED_H__
#define __ATT_PRED_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void Att_pred(double x[4], double b_P[9], const double w[3], double q,
                     double dt);

#endif

//
// File trailer for Att_pred.h
//
// [EOF]
//
