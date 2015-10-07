//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: predictMeasurementMono.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 07-Oct-2015 10:22:33
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "predictMeasurementMono.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// predictMeasurementLeft Predict the measurement of a feature given in the left
// camera frame
//    Get the normalized pixel coordinates where a feature given in the left camera
//    frame
// Arguments    : const double fp[3]
//                const double cameraparams_FocalLength[2]
//                const double cameraparams_PrincipalPoint[2]
//                double px[2]
// Return Type  : void
//
void predictMeasurementMono(const double fp[3], const double
  cameraparams_FocalLength[2], const double cameraparams_PrincipalPoint[2],
  double px[2])
{
  double h_c_n_l[2];
  int i;
  for (i = 0; i < 2; i++) {
    h_c_n_l[i] = fp[i] / fp[2];
  }

  //  normalized feature in camera frame
  px[0] = h_c_n_l[0] * cameraparams_FocalLength[0] +
    cameraparams_PrincipalPoint[0];
  px[1] = h_c_n_l[1] * cameraparams_FocalLength[1] +
    cameraparams_PrincipalPoint[1];
}

//
// File trailer for predictMeasurementMono.cpp
//
// [EOF]
//
