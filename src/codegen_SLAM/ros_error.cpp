//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ros_error.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 01-Sep-2015 22:19:36
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
void b_ros_error()
{
  char cv8[25];
  int i7;
  static const char cv9[25] = { 'm', 'a', 'p', ' ', 'f', 'e', 'a', 't', 'u', 'r',
    'e', ' ', 'i', 's', ' ', 'n', 'o', 't', ' ', 'v', 'a', 'l', 'i', 'd', '\x00'
  };

  for (i7 = 0; i7 < 25; i7++) {
    cv8[i7] = cv9[i7];
  }

  ROS_ERROR(cv8);
}

//
// #coder
// ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
// Arguments    : void
// Return Type  : void
//
void ros_error()
{
  char cv4[38];
  int i5;
  static const char cv5[38] = { 'A', 'n', 'c', 'h', 'o', 'r', ' ', 'f', 'e', 'a',
    't', 'u', 'r', 'e', ' ', 'm', 'a', 't', 'r', 'i', 'x', ' ', 'i', 's', ' ',
    'i', 'n', 'c', 'o', 'n', 's', 'i', 's', 't', 'e', 'n', 't', '\x00' };

  for (i5 = 0; i5 < 38; i5++) {
    cv4[i5] = cv5[i5];
  }

  ROS_ERROR(cv4);
}

//
// File trailer for ros_error.cpp
//
// [EOF]
//
