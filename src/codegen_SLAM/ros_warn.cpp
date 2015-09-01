//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ros_warn.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 01-Sep-2015 22:19:36
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "ros_warn.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// #coder
// ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
// Arguments    : void
// Return Type  : void
//
void ros_warn()
{
  char cv10[38];
  int i12;
  static const char cv11[38] = { '1', '-', 'p', 'o', 'i', 'n', 't', ' ', 'R',
    'A', 'N', 'S', 'A', 'C', ' ', 'r', 'e', 'j', 'e', 'c', 't', 'e', 'd', ' ',
    'a', 'l', 'l', ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e', 's', '!', '\x00' };

  for (i12 = 0; i12 < 38; i12++) {
    cv10[i12] = cv11[i12];
  }

  ROS_WARN(cv10);
}

//
// File trailer for ros_warn.cpp
//
// [EOF]
//
