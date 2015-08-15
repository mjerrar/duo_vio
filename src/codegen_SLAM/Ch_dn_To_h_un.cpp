//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Ch_dn_To_h_un.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 15-Aug-2015 13:16:41
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
  h_dn_l_To_h_un_l[0] = (((1.0 + -0.414085141240295 * (x2 + y2)) +
    0.236451305145822 * (a * a)) + -0.0871296995623235 * rt_powd_snf(x2 + y2,
    4.0)) + x * ((-0.82817028248059 * x + 0.945805220583288 * x * (x2 + y2)) +
                 -0.697037596498588 * x * rt_powd_snf(x2 + y2, 3.0));
  h_dn_l_To_h_un_l[2] = 2.0 * x * y * ((-0.414085141240295 + 0.472902610291644 *
    (x2 + y2)) + -0.348518798249294 * rt_powd_snf(x2 + y2, 3.0));
  h_dn_l_To_h_un_l[1] = 2.0 * x * y * ((-0.414085141240295 + 0.472902610291644 *
    (x2 + y2)) + -0.348518798249294 * rt_powd_snf(x2 + y2, 3.0));
  a = x2 + y2;
  h_dn_l_To_h_un_l[3] = (((1.0 + -0.414085141240295 * (x2 + y2)) +
    0.236451305145822 * (a * a)) + -0.0871296995623235 * rt_powd_snf(x2 + y2,
    4.0)) + y * ((-0.82817028248059 * y + 0.945805220583288 * y * (x2 + y2)) +
                 -0.697037596498588 * y * rt_powd_snf(x2 + y2, 3.0));

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
