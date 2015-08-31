//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: get_r_u.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 31-Aug-2015 09:51:22
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "get_r_u.h"
#include "predictMeasurement_stereo.h"
#include "SLAM_rtwutil.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// get_r_u Get undistorted radius from distorted radius
//    Get the pixel radius of the undistorted pixels from a distorted pixel
//    radius and distortion parameters
// Arguments    : double k1
//                double k2
//                double k3
//                double rd
// Return Type  : double
//
double get_r_u(double k1, double k2, double k3, double rd)
{
  double ru;
  int i;
  boolean_T exitg1;
  double diff;
  char cv2[28];
  static const char cv3[28] = { 'n', 'e', 'g', 'a', 't', 'i', 'v', 'e', ' ', 'u',
    'n', 'd', 'i', 's', 't', 'o', 'r', 't', 'e', 'd', ' ', 'r', 'a', 'd', 'i',
    'u', 's', '\x00' };

  ru = rd;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 10)) {
    diff = ((((ru + k1 * rt_powd_snf(ru, 3.0)) + k2 * rt_powd_snf(ru, 5.0)) + k3
             * rt_powd_snf(ru, 7.0)) - rd) / (((1.0 + 3.0 * k1 * (ru * ru)) +
      5.0 * k2 * rt_powd_snf(ru, 4.0)) + 7.0 * k3 * rt_powd_snf(ru, 6.0));
    ru -= diff;
    if ((diff < 1.0E-6) && (diff > -1.0E-6)) {
      exitg1 = true;
    } else {
      i++;
    }
  }

  if (ru < 0.0) {
    // #coder
    // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
    for (i = 0; i < 28; i++) {
      cv2[i] = cv3[i];
    }

    ROS_ERROR(cv2);
  }

  return ru;
}

//
// File trailer for get_r_u.cpp
//
// [EOF]
//
