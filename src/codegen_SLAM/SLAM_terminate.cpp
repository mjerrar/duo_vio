//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_terminate.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_terminate.h"
#include "SLAM_emxutil.h"
#include "SLAM_data.h"
#include <ros/console.h>

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void SLAM_terminate()
{
  SLAM_free();
  emxFree_boolean_T(&triangulation_success);
}

//
// File trailer for SLAM_terminate.cpp
//
// [EOF]
//
