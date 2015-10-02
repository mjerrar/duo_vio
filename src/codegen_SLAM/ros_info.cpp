//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ros_info.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "ros_info.h"
#include "SLAM_data.h"
#include <ros/console.h>

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
  char cv2[54];
  int i10;
  static const char cv3[54] = { 'F', 'i', 'x', 'e', 'd', ' ', 'f', 'e', 'a', 't',
    'u', 'r', 'e', ' ', '%', 'i', ' ', '(', '%', 'i', ' ', 'o', 'n', ' ', 'a',
    'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ')', ' ', 'i', 's', ' ', 'n', 'o',
    ' ', 'l', 'o', 'n', 'g', 'e', 'r', ' ', 'v', 'a', 'l', 'i', 'd', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i10 = 0; i10 < 54; i10++) {
      cv2[i10] = cv3[i10];
    }

    ROS_INFO(cv2, varargin_1, varargin_2, varargin_3);
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
void c_ros_info(double varargin_1, double varargin_2, double varargin_3)
{
  char cv6[44];
  int i12;
  static const char cv7[44] = { 'F', 'i', 'x', 'i', 'n', 'g', ' ', 'f', 'e', 'a',
    't', 'u', 'r', 'e', ' ', '%', 'i', ' ', '(', 'f', 'e', 'a', 't', 'u', 'r',
    'e', ' ', '%', 'i', ' ', 'o', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ',
    '%', 'i', ')', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i12 = 0; i12 < 44; i12++) {
      cv6[i12] = cv7[i12];
    }

    ROS_INFO(cv6, varargin_1, varargin_2, varargin_3);
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
  char cv8[24];
  int i14;
  static const char cv9[24] = { 'R', 'e', 'j', 'e', 'c', 't', 'i', 'n', 'g', ' ',
    '%', 'i', ' ', '(', '%', 'i', ' ', 'o', 'n', ' ', '%', 'i', ')', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i14 = 0; i14 < 24; i14++) {
      cv8[i14] = cv9[i14];
    }

    ROS_INFO(cv8, varargin_1, varargin_2, varargin_3);
  }
}

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                int varargin_2
// Return Type  : void
//
void e_ros_info(int varargin_1, int varargin_2)
{
  char cv10[44];
  int i16;
  static const char cv11[44] = { 's', 'u', 'c', 'c', 'e', 's', 's', 'f', 'u',
    'l', 'l', 'y', ' ', 't', 'r', 'i', 'a', 'n', 'g', 'u', 'l', 'a', 't', 'e',
    'd', ' ', '%', 'd', ' ', 'o', 'f', ' ', '%', 'd', ' ', 'f', 'e', 'a', 't',
    'u', 'r', 'e', 's', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i16 = 0; i16 < 44; i16++) {
      cv10[i16] = cv11[i16];
    }

    ROS_INFO(cv10, varargin_1, varargin_2);
  }
}

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : double varargin_1
// Return Type  : void
//
void f_ros_info(double varargin_1)
{
  char cv12[52];
  int i17;
  static const char cv13[52] = { 'I', 'n', 'i', 't', 'i', 'a', 'l', 'i', 'z',
    'i', 'n', 'g', ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ',', ' ',
    'w', 'h', 'i', 'c', 'h', ' ', 'w', 'a', 's', ' ', 't', 'h', 'e', ' ', 'o',
    'r', 'i', 'g', 'i', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i17 = 0; i17 < 52; i17++) {
      cv12[i17] = cv13[i17];
    }

    ROS_INFO(cv12, varargin_1);
  }
}

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : double varargin_1
// Return Type  : void
//
void g_ros_info(double varargin_1)
{
  char cv14[45];
  int i18;
  static const char cv15[45] = { 'F', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%',
    'd', ' ', 'i', 's', ' ', 't', 'o', 'o', ' ', 'f', 'a', 'r', ' ', 'a', 'w',
    'a', 'y', ' ', 't', 'o', ' ', 't', 'r', 'i', 'a', 'n', 'g', 'u', 'l', 'a',
    't', 'e', '.', '\\', 'n', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i18 = 0; i18 < 45; i18++) {
      cv14[i18] = cv15[i18];
    }

    ROS_INFO(cv14, varargin_1);
  }
}

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                int varargin_2
//                int varargin_3
// Return Type  : void
//
void h_ros_info(int varargin_1, int varargin_2, int varargin_3)
{
  char cv16[48];
  int i19;
  static const char cv17[48] = { 'I', 'n', 's', 'e', 'r', 't', 'i', 'n', 'g',
    ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%', 'd', ' ', 'a', 's', ' ',
    'f', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%', 'i', ' ', 'o', 'n', ' ', 'a',
    'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i19 = 0; i19 < 48; i19++) {
      cv16[i19] = cv17[i19];
    }

    ROS_INFO(cv16, varargin_1, varargin_2, varargin_3);
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
void i_ros_info(double varargin_1, double varargin_2, double varargin_3)
{
  char cv18[43];
  int i20;
  static const char cv19[43] = { 'F', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%',
    'i', ' ', '(', '%', 'i', ' ', 'o', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r',
    ' ', '%', 'i', ')', ' ', 'h', 'a', 's', ' ', 'c', 'o', 'n', 'v', 'e', 'r',
    'g', 'e', 'd', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i20 = 0; i20 < 43; i20++) {
      cv18[i20] = cv19[i20];
    }

    ROS_INFO(cv18, varargin_1, varargin_2, varargin_3);
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
void j_ros_info(double varargin_1, double varargin_2, double varargin_3)
{
  char cv20[51];
  int i23;
  static const char cv21[51] = { 'F', 'o', 'r', 'c', 'i', 'n', 'g', ' ', 'a',
    'c', 't', 'i', 'v', 'a', 't', 'i', 'o', 'n', ' ', 'o', 'f', ' ', 'f', 'e',
    'a', 't', 'u', 'r', 'e', ' ', '%', 'i', ' ', '(', '%', 'i', ' ', 'o', 'n',
    ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ')', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i23 = 0; i23 < 51; i23++) {
      cv20[i23] = cv21[i23];
    }

    ROS_INFO(cv20, varargin_1, varargin_2, varargin_3);
  }
}

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : void
// Return Type  : void
//
void k_ros_info()
{
  char cv22[24];
  int i24;
  static const char cv23[24] = { 'r', 'e', 'q', 'u', 'e', 's', 't', 'i', 'n',
    'g', ' ', 'n', 'e', 'w', ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e', 's', '\x00'
  };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i24 = 0; i24 < 24; i24++) {
      cv22[i24] = cv23[i24];
    }

    ROS_INFO(cv22);
  }
}

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : double varargin_1
// Return Type  : void
//
void l_ros_info(double varargin_1)
{
  char cv26[28];
  int i27;
  static const char cv27[28] = { 'S', 'e', 't', 't', 'i', 'n', 'g', ' ', 'a',
    'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ' ', 'a', 's', ' ', 'o', 'r', 'i',
    'g', 'i', 'n', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i27 = 0; i27 < 28; i27++) {
      cv26[i27] = cv27[i27];
    }

    ROS_INFO(cv26, varargin_1);
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
  char cv0[43];
  int i9;
  static const char cv1[43] = { 'L', 'o', 's', 't', ' ', 'f', 'e', 'a', 't', 'u',
    'r', 'e', ' ', '%', 'i', ',', ' ', 'w', 'h', 'i', 'c', 'h', ' ', 'w', 'a',
    's', ' ', '%', 'i', ' ', 'o', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ',
    '%', 'i', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0) {
    for (i9 = 0; i9 < 43; i9++) {
      cv0[i9] = cv1[i9];
    }

    ROS_INFO(cv0, varargin_1, varargin_2, varargin_3);
  }
}

//
// File trailer for ros_info.cpp
//
// [EOF]
//
