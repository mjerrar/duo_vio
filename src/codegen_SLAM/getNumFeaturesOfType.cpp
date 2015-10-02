//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getNumFeaturesOfType.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getNumFeaturesOfType.h"
#include "SLAM_data.h"
#include <ros/console.h>

// Function Definitions

//
// getNumFeaturesOfType Get the number of features of type type of an anchor
//  type can be a scalar or a row vector of types
// Arguments    : const emxArray_struct_T *anchor_state_feature_states
//                double type
// Return Type  : double
//
double getNumFeaturesOfType(const emxArray_struct_T *anchor_state_feature_states,
  double type)
{
  double n;
  int featureIdx;
  boolean_T b0;
  n = 0.0;
  for (featureIdx = 0; featureIdx < (int)numPointsPerAnchor; featureIdx++) {
    b0 = !(anchor_state_feature_states->data[featureIdx].status == type);
    if (!b0) {
      n++;
    }
  }

  return n;
}

//
// File trailer for getNumFeaturesOfType.cpp
//
// [EOF]
//
