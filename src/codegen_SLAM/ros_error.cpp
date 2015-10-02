//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ros_error.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "ros_error.h"
#include <ros/console.h>

// Function Definitions

//
// #coder
// ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
// Arguments    : void
// Return Type  : void
//
void b_ros_error()
{
  char cv24[41];
  int i26;
  static const char cv25[41] = { 'p', 'i', 'c', 'k', 'e', 'd', ' ', 'a', 'n',
    ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', 'w', 'i', 't', 'h', ' ', 'n', 'o',
    ' ', 'a', 'c', 't', 'i', 'v', 'e', ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e',
    's', '\x00' };

  for (i26 = 0; i26 < 41; i26++) {
    cv24[i26] = cv25[i26];
  }

  ROS_ERROR(cv24);
}

//
// #coder
// ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
// Arguments    : void
// Return Type  : void
//
void ros_error()
{
  char cv4[27];
  int i11;
  static const char cv5[27] = { 'p', 'i', 'c', 'k', 'e', 'd', ' ', 'a', 'n', ' ',
    'i', 'n', 'a', 'c', 't', 'i', 'v', 'e', ' ', 'f', 'e', 'a', 't', 'u', 'r',
    'e', '\x00' };

  for (i11 = 0; i11 < 27; i11++) {
    cv4[i11] = cv5[i11];
  }

  ROS_ERROR(cv4);
}

//
// File trailer for ros_error.cpp
//
// [EOF]
//
