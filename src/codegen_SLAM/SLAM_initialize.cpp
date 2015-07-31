//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_initialize.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 31-Jul-2015 14:58:50
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_initialize.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "SLAM_updIT.h"
#include "SLAM_data.h"
#include <stdio.h>

// Named Constants
#define b_normalGravity                (true)
#define b_numStates                    (12.0)
#define b_numTrackFeatures             (32.0)
#define b_numStatesxt                  (13.0)
#define b_minFeatureThreshold          (2.6666666666666665)
#define b_numAnchors                   (4.0)
#define b_numStatesPerAnchor           (10.0)
#define b_maxEKFIterations             (10.0)

// Variable Definitions
static double numTrackFeatures;

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void SLAM_initialize()
{
  rt_InitInfAndNaN(8U);
  normalGravity = b_normalGravity;
  maxEKFIterations = b_maxEKFIterations;
  numStatesPerAnchor = b_numStatesPerAnchor;
  numAnchors = b_numAnchors;
  minFeatureThreshold = b_minFeatureThreshold;
  numStatesxt = b_numStatesxt;
  numTrackFeatures = b_numTrackFeatures;
  numStates = b_numStates;
  initialized_not_empty_init();
  SLAM_init();
  SLAM_updIT_init();
  c_eml_rand_mt19937ar_stateful_i();
}

//
// File trailer for SLAM_initialize.cpp
//
// [EOF]
//
