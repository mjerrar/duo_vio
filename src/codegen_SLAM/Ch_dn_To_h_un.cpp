//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Ch_dn_To_h_un.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 28-Aug-2015 19:03:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "Ch_dn_To_h_un.h"
#include "predictMeasurement_stereo.h"
#include "SLAM_rtwutil.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// UNTITLED Summary of this function goes here
//   computs the derivatives with respect to the normalised undistorted point
// Arguments    : double k1
//                double k2
//                double k3
//                double x
//                double y
//                double h_dn_l_To_h_un_l[4]
// Return Type  : void
//
void Ch_dn_To_h_un(double k1, double k2, double k3, double x, double y, double
                   h_dn_l_To_h_un_l[4])
{
  double x2;
  double y2;
  double a;
  x2 = x * x;
  y2 = y * y;
  a = x2 + y2;
  h_dn_l_To_h_un_l[0] = (((1.0 + k1 * (x2 + y2)) + k2 * (a * a)) + k3 *
    rt_powd_snf(x2 + y2, 4.0)) + x * ((2.0 * k1 * x + 4.0 * k2 * x * (x2 + y2))
    + 8.0 * k3 * x * rt_powd_snf(x2 + y2, 3.0));
  h_dn_l_To_h_un_l[2] = 2.0 * x * y * ((k1 + 2.0 * k2 * (x2 + y2)) + 4.0 * k3 *
    rt_powd_snf(x2 + y2, 3.0));
  h_dn_l_To_h_un_l[1] = 2.0 * x * y * ((k1 + 2.0 * k2 * (x2 + y2)) + 4.0 * k3 *
    rt_powd_snf(x2 + y2, 3.0));
  a = x2 + y2;
  h_dn_l_To_h_un_l[3] = (((1.0 + k1 * (x2 + y2)) + k2 * (a * a)) + k3 *
    rt_powd_snf(x2 + y2, 4.0)) + y * ((2.0 * k1 * y + 4.0 * k2 * y * (x2 + y2))
    + 8.0 * k3 * y * rt_powd_snf(x2 + y2, 3.0));

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
