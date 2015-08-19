//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_initialize.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 10:03:40
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_initialize.h"
#include "SLAM_data.h"
#include <stdio.h>

// Named Constants
#define b_gravityUpdate                (false)
#define b_useAirPressure               (false)
#define b_numStates                    (12.0)
#define b_numStatesxt                  (13.0)
#define b_minFeatureThreshold          (1.3333333333333333)
#define b_numTrackFeatures             (16.0)
#define b_sigma_Init                   (0.001)
#define b_gravAlignNoise               (10.0)

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
  useAirPressure = b_useAirPressure;
  gravAlignNoise = b_gravAlignNoise;
  gravityUpdate = b_gravityUpdate;
  sigma_Init = b_sigma_Init;
  numTrackFeatures = b_numTrackFeatures;
  minFeatureThreshold = b_minFeatureThreshold;
  numStatesxt = b_numStatesxt;
  numStates = b_numStates;
  init_counter_not_empty_init();
  SLAM_init();
}

//
// File trailer for SLAM_initialize.cpp
//
// [EOF]
//
