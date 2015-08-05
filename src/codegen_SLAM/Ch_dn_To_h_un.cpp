//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Ch_dn_To_h_un.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 05-Aug-2015 16:03:26
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "Ch_dn_To_h_un.h"
#include "predictMeasurement_stereo.h"
#include "SLAM_rtwutil.h"
#include <stdio.h>

// Function Definitions

//
// UNTITLED Summary of this function goes here
//   computs the derivatives with respect to the normalised undistorted point
// Arguments    : double x
//                double y
//                double h_dn_l_To_h_un_l[4]
// Return Type  : void
//
void Ch_dn_To_h_un(double x, double y, double h_dn_l_To_h_un_l[4])
{
  double x2;
  double y2;
  double a;
  x2 = x * x;
  y2 = y * y;
  a = x2 + y2;
  h_dn_l_To_h_un_l[0] = (((1.0 + -0.401250937863819 * (x2 + y2)) +
    0.193995986374922 * (a * a)) + -0.0528652448245847 * rt_powd_snf(x2 + y2,
    4.0)) + x * ((-0.802501875727638 * x + 0.775983945499688 * x * (x2 + y2)) +
                 -0.42292195859667758 * x * rt_powd_snf(x2 + y2, 3.0));
  h_dn_l_To_h_un_l[2] = 2.0 * x * y * ((-0.401250937863819 + 0.387991972749844 *
    (x2 + y2)) + -0.21146097929833879 * rt_powd_snf(x2 + y2, 3.0));
  h_dn_l_To_h_un_l[1] = 2.0 * x * y * ((-0.401250937863819 + 0.387991972749844 *
    (x2 + y2)) + -0.21146097929833879 * rt_powd_snf(x2 + y2, 3.0));
  a = x2 + y2;
  h_dn_l_To_h_un_l[3] = (((1.0 + -0.401250937863819 * (x2 + y2)) +
    0.193995986374922 * (a * a)) + -0.0528652448245847 * rt_powd_snf(x2 + y2,
    4.0)) + y * ((-0.802501875727638 * y + 0.775983945499688 * y * (x2 + y2)) +
                 -0.42292195859667758 * y * rt_powd_snf(x2 + y2, 3.0));

  //  h_dn_l_To_h_un_l(1,1)=1 + k1 * (3*  x2 + y2) + k2 * (5 * x2^2 + 6 * x2 * y2 + y2^2); 
  //  h_dn_l_To_h_un_l(1,2)=2 * x * y * (k1 + 2 * k2 * (x2 + y2));
  //  h_dn_l_To_h_un_l(2,1)=2 * x * y * (k1 + 2 * k2  *(x2 + y2));
  //  h_dn_l_To_h_un_l(2,2)=1 + k1 * (x2 + 3 * y2) + k2 * (x2^2 + 6 * x2 * y2 + 5 * y2^2); 
}

//
// File trailer for Ch_dn_To_h_un.cpp
//
// [EOF]
//
