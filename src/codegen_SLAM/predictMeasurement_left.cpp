//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: predictMeasurement_left.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 03-Sep-2015 23:49:51
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "predictMeasurement_left.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// PREDICTMEASUREMENT Predict the measurement of a feature given in the left
// camera frame
//    Get the normalized pixel coordinates where a feature given in the left camera
//    frame
// Arguments    : const double fp_l[3]
//                double px_n[2]
// Return Type  : void
//
void predictMeasurement_left(const double fp_l[3], double px_n[2])
{
  int i;

  //  normalized feature in camera frame
  for (i = 0; i < 2; i++) {
    px_n[i] = fp_l[i] / fp_l[2];
  }

  //  undistorted measurement in normalized pixels
}

//
// File trailer for predictMeasurement_left.cpp
//
// [EOF]
//
