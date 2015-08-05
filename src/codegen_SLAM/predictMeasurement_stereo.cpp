//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: predictMeasurement_stereo.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 05-Aug-2015 15:44:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "predictMeasurement_stereo.h"
#include "SLAM_rtwutil.h"
#include <stdio.h>

// Function Definitions

//
// PREDICTMEASUREMENT Predict the measurement of a feature given in the left
// camera frame
//    Get the pixel coordinates where a feature given in the left camera
//    frame would be visible in both cameras
// Arguments    : const double fp_l[3]
//                const double cameraparams_r_lr[3]
//                const double cameraparams_R_rl[9]
//                double h_u_l[2]
//                double h_u_r[2]
// Return Type  : void
//
void predictMeasurement_stereo(const double fp_l[3], const double
  cameraparams_r_lr[3], const double cameraparams_R_rl[9], double h_u_l[2],
  double h_u_r[2])
{
  double fp_r[3];
  int i;
  double b_fp_r;
  int i13;
  double h_cin_l[3];
  double radsq_l;
  double b;
  double radsq_r;

  //  camera parameters for the left and right camera
  //  r_lr = cameraparams.r_lr;
  //  R_lr = cameraparams.R_lr;
  //  R_rl=R_lr';
  for (i = 0; i < 3; i++) {
    b_fp_r = 0.0;
    for (i13 = 0; i13 < 3; i13++) {
      b_fp_r += cameraparams_R_rl[i + 3 * i13] * fp_l[i13];
    }

    fp_r[i] = b_fp_r - cameraparams_r_lr[i];
  }

  //  if fp_l(3) == 0
  //      ROS_ERROR('h_ci_l(3)==0')
  //  end
  //  if fp_r(3) == 0
  //      ROS_ERROR('h_ci_r(3)==0')
  //  end
  b_fp_r = fp_r[2];
  for (i = 0; i < 3; i++) {
    h_cin_l[i] = fp_l[i] / fp_l[2];
    fp_r[i] /= b_fp_r;
  }

  //  if any(isnan(h_cin_l))
  //      ROS_ERROR('h_cin_l')
  //      ROS_ERROR('h_ci_l(3) = %f', h_cin_l(3))
  //  end
  //  if any(isnan(h_cin_r))
  //      ROS_ERROR('h_cin_r')
  //      ROS_ERROR('h_ci_r(3) = %f', h_cin_r(3))
  //  end
  //  h_uin_l = h_cin_l(1:2);
  //  h_uin_r = h_cin_r(1:2);
  //  if any(isnan(h_uin_l))
  //      ROS_ERROR('h_uin_l')
  //  end
  //  if any(isnan(h_uin_r))
  //      ROS_ERROR('h_uin_r')
  //  end
  //  rad_l=sqrt(h_uin_l(1)^2+h_uin_l(2)^2);
  //  rad_r=sqrt(h_uin_r(1)^2+h_uin_r(2)^2);
  radsq_l = h_cin_l[0] * h_cin_l[0] + h_cin_l[1] * h_cin_l[1];
  b = ((1.0 + -0.41042183437511 * radsq_l) + 0.221265330302933 * (radsq_l *
        radsq_l)) + -0.0619354263198911 * rt_powd_snf(radsq_l, 4.0);
  radsq_r = fp_r[0] * fp_r[0] + fp_r[1] * fp_r[1];
  b_fp_r = ((1.0 + -0.403776514266162 * radsq_r) + 0.212756636184267 * (radsq_r *
             radsq_r)) + -0.0594355697055105 * rt_powd_snf(radsq_r, 4.0);
  for (i = 0; i < 3; i++) {
    h_cin_l[i] *= b;
    fp_r[i] *= b_fp_r;
  }

  //  if any(isnan(rad_l))
  //      ROS_ERROR('rad_l')
  //  end
  //  if any(isnan(rad_r))
  //      ROS_ERROR('rad_r')
  //  end
  //  if any(isnan(h_din_l))
  //      ROS_ERROR('h_din_l')
  //  end
  //  if any(isnan(h_din_r))
  //      ROS_ERROR('h_din_r')
  //  end
  h_u_l[0] = 370.467750713497 + 537.083588825387 * h_cin_l[0];
  h_u_l[1] = 226.640025353723 + 539.036743091681 * h_cin_l[1];
  h_u_r[0] = 390.576377367293 + 538.369043959143 * fp_r[0];
  h_u_r[1] = 218.634996583338 + 539.436438304934 * fp_r[1];

  //  if any(isnan(h_u_l))
  //      ROS_ERROR('h_di_l')
  //  end
  //  if any(isnan(h_u_r))
  //      ROS_ERROR('h_di_r')
  //  end
}

//
// File trailer for predictMeasurement_stereo.cpp
//
// [EOF]
//
