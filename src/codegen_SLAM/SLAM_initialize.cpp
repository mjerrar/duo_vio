//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_initialize.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Oct-2015 15:29:53
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_initialize.h"
#include "SLAM_emxutil.h"
#include "SLAM_data.h"
#include <ros/console.h>
#include <stdio.h>

// Named Constants
#define b_numStates                    (21.0)
#define b_numStatesxt                  (22.0)
#define b_numPointsPerAnchor           (8.0)
#define b_numAnchors                   (5.0)
#define b_numTrackFeatures             (40.0)
#define b_debug_level                  (2.0)
#define b_minFeatureThreshold          (4.0)

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void SLAM_initialize()
{
  int i0;
  static const boolean_T c_triangulation_success[11] = { true, true, true, true,
    true, true, true, true, true, false, true };

  rt_InitInfAndNaN(8U);
  emxInit_boolean_T(&triangulation_success, 1);
  i0 = triangulation_success->size[0];
  triangulation_success->size[0] = 11;
  emxEnsureCapacity((emxArray__common *)triangulation_success, i0, (int)sizeof
                    (boolean_T));
  for (i0 = 0; i0 < 11; i0++) {
    triangulation_success->data[i0] = c_triangulation_success[i0];
  }

  minFeatureThreshold = b_minFeatureThreshold;
  debug_level = b_debug_level;
  numTrackFeatures = b_numTrackFeatures;
  numAnchors = b_numAnchors;
  numPointsPerAnchor = b_numPointsPerAnchor;
  numStatesxt = b_numStatesxt;
  numStates = b_numStates;
  initialized_not_empty_init();
  SLAM_init();
}

//
// File trailer for SLAM_initialize.cpp
//
// [EOF]
//
