//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_initialize.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 15-Jul-2015 17:00:42
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_initialize.h"
#include "SLAM_updIT.h"
#include "SLAM_data.h"
#include <stdio.h>

// Named Constants
#define b_normalGravity                (true)
#define b_trailSize                    (0.0)
#define b_numStates                    (12.0)
#define b_numTrackFeatures             (32.0)
#define b_numStatesxt                  (13.0)
#define b_baselineThresold             (0.3)
#define b_minFeatureThreshold          (10.0)

// Variable Definitions
static double baselineThresold;
static double minFeatureThreshold;
static boolean_T normalGravity;

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void SLAM_initialize()
{
  rt_InitInfAndNaN(8U);
  normalGravity = b_normalGravity;
  minFeatureThreshold = b_minFeatureThreshold;
  baselineThresold = b_baselineThresold;
  numStatesxt = b_numStatesxt;
  numTrackFeatures = b_numTrackFeatures;
  numStates = b_numStates;
  trailSize = b_trailSize;
  initialized_not_empty_init();
  SLAM_init();
  SLAM_updIT_init();
}

//
// File trailer for SLAM_initialize.cpp
//
// [EOF]
//
