//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ros_info.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 01-Sep-2015 22:19:36
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "ros_info.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : void
// Return Type  : void
//
void b_ros_info()
{
  char cv2[38];
  int i1;
  static const char cv3[38] = { 'I', 'n', 'i', 't', 'i', 'a', 'l', 'i', 'z', 'i',
    'n', 'g', ' ', 'o', 'r', 'i', 'e', 'n', 't', 'a', 't', 'i', 'o', 'n', ' ',
    'w', 'i', 't', 'h', ' ', 'D', 'U', 'O', ' ', 'I', 'M', 'U', '\x00' };

  for (i1 = 0; i1 < 38; i1++) {
    cv2[i1] = cv3[i1];
  }

  ROS_INFO(cv2);
}

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : void
// Return Type  : void
//
void ros_info()
{
  char cv0[46];
  int i0;
  static const char cv1[46] = { 'I', 'n', 'i', 't', 'i', 'a', 'l', 'i', 'z', 'i',
    'n', 'g', ' ', 'o', 'r', 'i', 'e', 'n', 't', 'a', 't', 'i', 'o', 'n', ' ',
    'w', 'i', 't', 'h', ' ', 'F', 'M', 'U', ' ', 'o', 'r', 'i', 'e', 'n', 't',
    'a', 't', 'i', 'o', 'n', '\x00' };

  for (i0 = 0; i0 < 46; i0++) {
    cv0[i0] = cv1[i0];
  }

  ROS_INFO(cv0);
}

//
// File trailer for ros_info.cpp
//
// [EOF]
//
