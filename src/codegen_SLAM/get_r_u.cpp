//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: get_r_u.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 03-Sep-2015 21:31:09
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "get_r_u.h"
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
  double x;
  double rd_sq;
  int i;
  boolean_T exitg1;
  double a;
  double b_a;
  double diff;
  char cv2[46];
  static const char cv3[46] = { 'n', 'e', 'g', 'a', 't', 'i', 'v', 'e', ' ', 'u',
    'n', 'd', 'i', 's', 't', 'o', 'r', 't', 'e', 'd', ' ', 'r', 'a', 'd', 'i',
    'u', 's', '.', ' ', 'r', 'd', ' ', '=', ' ', '%', 'f', ',', ' ', 'r', 'u',
    ' ', '=', ' ', '%', 'f', '\x00' };

  x = rd * rd;
  rd_sq = x;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 100)) {
    a = ((1.0 + k1 * x) + k2 * (x * x)) + k3 * rt_powd_snf(x, 3.0);
    b_a = (1.0 + k1 * x) + k2 * (x * x) * k3 * rt_powd_snf(x, 3.0);
    diff = (x * (a * a) - rd_sq) / (b_a * b_a + 2.0 * x * (((1.0 + 2.0 * k1 * x)
      + 2.0 * k2 * (x * x)) + 2.0 * k3 * rt_powd_snf(x, 3.0)));
    x -= diff;
    if ((diff < 1.0E-6) && (diff > -1.0E-6)) {
      exitg1 = true;
    } else {
      i++;
    }
  }

  if (x < 0.0) {
    // #coder
    // ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
    for (i = 0; i < 46; i++) {
      cv2[i] = cv3[i];
    }

    ROS_WARN(cv2, rd, x);
  }

  return sqrt(x);
}

//
// File trailer for get_r_u.cpp
//
// [EOF]
//
