//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ros_warn.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 01-Sep-2015 21:43:27
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
void b_ros_warn()
{
  char cv8[16];
  int i13;
  static const char cv9[16] = { 'S', 'L', 'A', 'M', ' ', 's', 'o', 'f', 't', ' ',
    'r', 'e', 's', 'e', 't', '\x00' };

  for (i13 = 0; i13 < 16; i13++) {
    cv8[i13] = cv9[i13];
  }

  ROS_WARN(cv8);
}

//
// #coder
// ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
// Arguments    : void
// Return Type  : void
//
void ros_warn()
{
  char cv6[38];
  int i10;
  static const char cv7[38] = { '1', '-', 'p', 'o', 'i', 'n', 't', ' ', 'R', 'A',
    'N', 'S', 'A', 'C', ' ', 'r', 'e', 'j', 'e', 'c', 't', 'e', 'd', ' ', 'a',
    'l', 'l', ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e', 's', '!', '\x00' };

  for (i10 = 0; i10 < 38; i10++) {
    cv6[i10] = cv7[i10];
  }

  ROS_WARN(cv6);
}

//
// File trailer for ros_warn.cpp
//
// [EOF]
//
