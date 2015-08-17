//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_initialize.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 17-Aug-2015 14:51:33
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
#define b_gravityUpdate                (true)
#define b_useAirPressure               (false)
#define b_normalGravity                (true)
#define b_numStates                    (12.0)
#define b_numTrackFeatures             (16.0)
#define b_numStatesxt                  (13.0)
#define b_minFeatureThreshold          (2.0)
#define b_sigma_Init                   (1.0E-5)
#define b_maxEKFIterations             (1.0)
#define b_gravAlignNoise               (0.01)

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
  useAirPressure = b_useAirPressure;
  gravAlignNoise = b_gravAlignNoise;
  gravityUpdate = b_gravityUpdate;
  maxEKFIterations = b_maxEKFIterations;
  sigma_Init = b_sigma_Init;
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
