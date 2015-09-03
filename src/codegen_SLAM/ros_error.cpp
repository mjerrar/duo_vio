//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ros_error.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 03-Sep-2015 17:44:13
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
  int i3;
  static const char cv1[38] = { 'A', 'n', 'c', 'h', 'o', 'r', ' ', 'f', 'e', 'a',
    't', 'u', 'r', 'e', ' ', 'm', 'a', 't', 'r', 'i', 'x', ' ', 'i', 's', ' ',
    'i', 'n', 'c', 'o', 'n', 's', 'i', 's', 't', 'e', 'n', 't', '\x00' };

  for (i3 = 0; i3 < 38; i3++) {
    cv0[i3] = cv1[i3];
  }

  ROS_ERROR(cv0);
}

//
// File trailer for ros_error.cpp
//
// [EOF]
//
