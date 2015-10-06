//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getTotalNumDelayedFeatures.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 05-Oct-2015 20:16:23
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getTotalNumDelayedFeatures.h"
#include "getNumFeaturesOfType.h"
#include "SLAM_emxutil.h"
#include "SLAM_data.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// getTotalNumDelayedFeatures Get the number of delayed features of all anchors
// Arguments    : const emxArray_b_struct_T *xt_anchor_states
// Return Type  : double
//
double getTotalNumDelayedFeatures(const emxArray_b_struct_T *xt_anchor_states)
{
  double n;
  int i21;
  int anchorIdx;
  e_struct_T expl_temp;
  n = 0.0;
  i21 = (int)numAnchors;
  anchorIdx = 0;
  b_emxInitStruct_struct_T(&expl_temp);
  while (anchorIdx <= i21 - 1) {
    emxCopyStruct_struct_T(&expl_temp, &xt_anchor_states->data[anchorIdx]);
    n += getNumFeaturesOfType(expl_temp.feature_states, 2.0);
    anchorIdx++;
  }

  emxFreeStruct_struct_T(&expl_temp);
  return n;
}

//
// File trailer for getTotalNumDelayedFeatures.cpp
//
// [EOF]
//
