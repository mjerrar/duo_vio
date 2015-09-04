//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ros_error.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Sep-2015 11:04:36
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "ros_error.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// #coder
// ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
// Arguments    : void
// Return Type  : void
//
void ros_error()
{
  char cv0[38];
  int i2;
  static const char cv1[38] = { 'A', 'n', 'c', 'h', 'o', 'r', ' ', 'f', 'e', 'a',
    't', 'u', 'r', 'e', ' ', 'm', 'a', 't', 'r', 'i', 'x', ' ', 'i', 's', ' ',
    'i', 'n', 'c', 'o', 'n', 's', 'i', 's', 't', 'e', 'n', 't', '\x00' };

  for (i2 = 0; i2 < 38; i2++) {
    cv0[i2] = cv1[i2];
  }

  ROS_ERROR(cv0);
}

//
// File trailer for ros_error.cpp
//
// [EOF]
//
