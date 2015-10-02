//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_data.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//
#ifndef __SLAM_DATA_H__
#define __SLAM_DATA_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "SLAM_types.h"

// Variable Declarations
extern double numStates;
extern double numStatesxt;
extern double numPointsPerAnchor;
extern double numAnchors;
extern double numTrackFeatures;
extern double minFeatureThreshold;
extern emxArray_boolean_T *triangulation_success;
extern double debug_level;

#endif

//
// File trailer for SLAM_data.h
//
// [EOF]
//
