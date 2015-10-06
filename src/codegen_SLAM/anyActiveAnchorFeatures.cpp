//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: anyActiveAnchorFeatures.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Oct-2015 15:29:53
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "anyActiveAnchorFeatures.h"
#include "SLAM_data.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// Arguments    : const emxArray_struct_T *anchor_state_feature_states
// Return Type  : boolean_T
//
boolean_T anyActiveAnchorFeatures(const emxArray_struct_T
  *anchor_state_feature_states)
{
  boolean_T ret;
  int featureIdx;
  boolean_T exitg1;
  ret = false;
  featureIdx = 0;
  exitg1 = false;
  while ((!exitg1) && (featureIdx <= (int)numPointsPerAnchor - 1)) {
    if (anchor_state_feature_states->data[featureIdx].status == 1.0) {
      ret = true;
      exitg1 = true;
    } else {
      featureIdx++;
    }
  }

  return ret;
}

//
// File trailer for anyActiveAnchorFeatures.cpp
//
// [EOF]
//
