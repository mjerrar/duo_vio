//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_initialize.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Sep-2015 16:33:03
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_initialize.h"
#include "SLAM_updIT.h"
#include "SLAM_pred.h"
#include "SLAM_data.h"
#include <ros/console.h>
#include <stdio.h>

// Named Constants
#define b_numStates                    (15.0)
#define b_numStatesxt                  (16.0)
#define b_minFeatureThreshold          (3.0)

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void SLAM_initialize()
{
  rt_InitInfAndNaN(8U);
  last_imu_not_empty_init();
  minFeatureThreshold = b_minFeatureThreshold;
  numStatesxt = b_numStatesxt;
  numStates = b_numStates;
  initialized_not_empty_init();
  SLAM_init();
  SLAM_updIT_init();
}

//
// File trailer for SLAM_initialize.cpp
//
// [EOF]
//
