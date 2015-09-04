//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ros_warn.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Sep-2015 16:58:25
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
  char cv6[38];
  int i7;
  static const char cv7[38] = { '1', '-', 'p', 'o', 'i', 'n', 't', ' ', 'R', 'A',
    'N', 'S', 'A', 'C', ' ', 'r', 'e', 'j', 'e', 'c', 't', 'e', 'd', ' ', 'a',
    'l', 'l', ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e', 's', '!', '\x00' };

  for (i7 = 0; i7 < 38; i7++) {
    cv6[i7] = cv7[i7];
  }

  ROS_WARN(cv6);
}

//
// File trailer for ros_warn.cpp
//
// [EOF]
//
