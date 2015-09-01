//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Att_upd.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 01-Sep-2015 21:43:27
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
extern void Att_upd(const double x_data[], double P_data[], int P_size[2], const
                    double z[3], double n, double dt, double x[4]);

#endif

//
// File trailer for Att_upd.h
//
// [EOF]
//
