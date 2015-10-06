//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fprintf.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Oct-2015 15:29:53
//
#ifndef __FPRINTF_H__
#define __FPRINTF_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void ab_fprintf(int formatSpec);
extern void b_fprintf();
extern void cb_fprintf(const char formatSpec_data[], const int formatSpec_size[2]);
extern void d_fprintf(double formatSpec);
extern void eb_fprintf(const char formatSpec_data[], const int formatSpec_size[2]);
extern void f_fprintf(double formatSpec);
extern void gb_fprintf(const char formatSpec_data[], const int formatSpec_size[2]);
extern void h_fprintf(double formatSpec);
extern void ib_fprintf(const char formatSpec_data[], const int formatSpec_size[2]);
extern void j_fprintf(double formatSpec);
extern void kb_fprintf(double formatSpec, double varargin_1, double varargin_2);
extern void l_fprintf(double formatSpec);
extern void mb_fprintf(double formatSpec, double varargin_1, double varargin_2);
extern void n_fprintf(double formatSpec, double varargin_1, double varargin_2);
extern void p_fprintf(double formatSpec, double varargin_1, double varargin_2);
extern void r_fprintf(double formatSpec);
extern void t_fprintf(double formatSpec);
extern void v_fprintf(int formatSpec);
extern void x_fprintf(int formatSpec);

#endif

//
// File trailer for fprintf.h
//
// [EOF]
//
