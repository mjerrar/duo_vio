//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: predictMeasurementMono.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 05-Oct-2015 20:16:23
//
#ifndef __PREDICTMEASUREMENTMONO_H__
#define __PREDICTMEASUREMENTMONO_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Function Declarations
extern void predictMeasurementMono(const double fp[3], const double
  cameraparams_FocalLength[2], const double cameraparams_PrincipalPoint[2],
  double px[2]);

#endif

//
// File trailer for predictMeasurementMono.h
//
// [EOF]
//
