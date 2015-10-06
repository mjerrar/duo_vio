//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getNumValidFeatures.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 05-Oct-2015 20:16:23
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "getNumValidFeatures.h"
#include "SLAM_data.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// getNumValidFeatures Get the number of valid features of an anchor
// Arguments    : const emxArray_struct_T *anchor_state_feature_states
// Return Type  : double
//
double getNumValidFeatures(const emxArray_struct_T *anchor_state_feature_states)
{
  double n;
  int featureIdx;
  boolean_T x[2];
  int k;
  boolean_T y;
  boolean_T exitg1;

  // getNumFeaturesOfType Get the number of features of type type of an anchor
  //  type can be a scalar or a row vector of types
  n = 0.0;
  for (featureIdx = 0; featureIdx < (int)numPointsPerAnchor; featureIdx++) {
    for (k = 0; k < 2; k++) {
      x[k] = (anchor_state_feature_states->data[featureIdx].status == 1.0 +
              (double)k);
    }

    y = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 2)) {
      if (!!x[k]) {
        y = true;
        exitg1 = true;
      } else {
        k++;
      }
    }

    if (y) {
      n++;
    }
  }

  return n;
}

//
// File trailer for getNumValidFeatures.cpp
//
// [EOF]
//
