//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ros_info.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 05-Oct-2015 20:16:23
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "ros_info.h"
#include "SLAM_data.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : double varargin_1
//                double varargin_2
//                double varargin_3
// Return Type  : void
//
void b_ros_info(double varargin_1, double varargin_2, double varargin_3)
{
  char cv4[44];
  int i15;
  static const char cv5[44] = { 'F', 'i', 'x', 'i', 'n', 'g', ' ', 'f', 'e', 'a',
    't', 'u', 'r', 'e', ' ', '%', 'i', ' ', '(', 'f', 'e', 'a', 't', 'u', 'r',
    'e', ' ', '%', 'i', ' ', 'o', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ',
    '%', 'i', ')', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i15 = 0; i15 < 44; i15++) {
      cv4[i15] = cv5[i15];
    }

    ROS_INFO(cv4, varargin_1, varargin_2, varargin_3);
  }
}

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : double varargin_1
// Return Type  : void
//
void c_ros_info(double varargin_1)
{
  char cv8[45];
  int i19;
  static const char cv9[45] = { 'F', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%', 'd',
    ' ', 'i', 's', ' ', 't', 'o', 'o', ' ', 'f', 'a', 'r', ' ', 'a', 'w', 'a',
    'y', ' ', 't', 'o', ' ', 't', 'r', 'i', 'a', 'n', 'g', 'u', 'l', 'a', 't',
    'e', '.', '\\', 'n', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i19 = 0; i19 < 45; i19++) {
      cv8[i19] = cv9[i19];
    }

    ROS_INFO(cv8, varargin_1);
  }
}

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : double varargin_1
//                double varargin_2
//                double varargin_3
// Return Type  : void
//
void d_ros_info(double varargin_1, double varargin_2, double varargin_3)
{
  char cv10[51];
  int i22;
  static const char cv11[51] = { 'F', 'o', 'r', 'c', 'i', 'n', 'g', ' ', 'a',
    'c', 't', 'i', 'v', 'a', 't', 'i', 'o', 'n', ' ', 'o', 'f', ' ', 'f', 'e',
    'a', 't', 'u', 'r', 'e', ' ', '%', 'i', ' ', '(', '%', 'i', ' ', 'o', 'n',
    ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ')', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i22 = 0; i22 < 51; i22++) {
      cv10[i22] = cv11[i22];
    }

    ROS_INFO(cv10, varargin_1, varargin_2, varargin_3);
  }
}

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : double varargin_1
//                double varargin_2
//                double varargin_3
// Return Type  : void
//
void ros_info(double varargin_1, double varargin_2, double varargin_3)
{
  char cv0[54];
  int i13;
  static const char cv1[54] = { 'F', 'i', 'x', 'e', 'd', ' ', 'f', 'e', 'a', 't',
    'u', 'r', 'e', ' ', '%', 'i', ' ', '(', '%', 'i', ' ', 'o', 'n', ' ', 'a',
    'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ')', ' ', 'i', 's', ' ', 'n', 'o',
    ' ', 'l', 'o', 'n', 'g', 'e', 'r', ' ', 'v', 'a', 'l', 'i', 'd', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i13 = 0; i13 < 54; i13++) {
      cv0[i13] = cv1[i13];
    }

    ROS_INFO(cv0, varargin_1, varargin_2, varargin_3);
  }
}

//
// File trailer for ros_info.cpp
//
// [EOF]
//
