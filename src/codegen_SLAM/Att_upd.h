//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Att_upd.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Sep-2015 23:21:02
//
#ifndef __ATT_UPD_H__
#define __ATT_UPD_H__

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
extern void Att_upd(double x[4], double b_P[9], const double z[3], double n,
                    double dt);

#endif

//
// File trailer for Att_upd.h
//
// [EOF]
//
