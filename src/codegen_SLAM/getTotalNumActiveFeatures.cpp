//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getTotalNumActiveFeatures.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Oct-2015 15:29:53
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getTotalNumActiveFeatures.h"
#include "getNumFeaturesOfType.h"
#include "SLAM_data.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// getTotalNumActiveFeatures Get the number of active features of all anchors
// Arguments    : const emxArray_b_struct_T *xt_anchor_states
// Return Type  : double
//
double getTotalNumActiveFeatures(const emxArray_b_struct_T *xt_anchor_states)
{
  double n;
  int i20;
  int anchorIdx;
  double b_n;
  n = 0.0;
  i20 = (int)numAnchors;
  for (anchorIdx = 0; anchorIdx < i20; anchorIdx++) {
    // getNumActiveFeatures Get the number of active features of an anchor
    b_n = getNumFeaturesOfType(xt_anchor_states->data[anchorIdx].feature_states,
      1.0);
    n += b_n;
  }

  return n;
}

//
// File trailer for getTotalNumActiveFeatures.cpp
//
// [EOF]
//
