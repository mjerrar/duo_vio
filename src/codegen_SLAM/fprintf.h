//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fprintf.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 03-Sep-2015 23:12:57
//
#ifndef __FPRINTF_H__
#define __FPRINTF_H__

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
extern void b_fprintf(signed char formatSpec);
extern void d_fprintf(signed char formatSpec);
extern void f_fprintf(int formatSpec, int varargin_1);
extern void h_fprintf();
extern void j_fprintf(double formatSpec);
extern void l_fprintf(double formatSpec, double varargin_1, double varargin_2,
                      double varargin_3, double varargin_4, double varargin_5,
                      double varargin_6, double varargin_7);

#endif

//
// File trailer for fprintf.h
//
// [EOF]
//
