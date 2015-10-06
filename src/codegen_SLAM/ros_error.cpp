//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ros_error.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 05-Oct-2015 20:16:23
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
  char cv6[14];
  int i17;
  static const char cv7[14] = { 'i', 'n', 'c', 'o', 'n', 's', 'i', 's', 't', 'e',
    'n', 'c', 'y', '\x00' };

  for (i17 = 0; i17 < 14; i17++) {
    cv6[i17] = cv7[i17];
  }

  ROS_ERROR(cv6);
}

//
// #coder
// ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
// Arguments    : void
// Return Type  : void
//
void ros_error()
{
  char cv2[27];
  int i14;
  static const char cv3[27] = { 'p', 'i', 'c', 'k', 'e', 'd', ' ', 'a', 'n', ' ',
    'i', 'n', 'a', 'c', 't', 'i', 'v', 'e', ' ', 'f', 'e', 'a', 't', 'u', 'r',
    'e', '\x00' };

  for (i14 = 0; i14 < 27; i14++) {
    cv2[i14] = cv3[i14];
  }

  ROS_ERROR(cv2);
}

//
// File trailer for ros_error.cpp
//
// [EOF]
//
