//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: undistortPoint.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Oct-2015 15:29:53
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "undistortPoint.h"
#include "predictMeasurementStereoDistorted.h"
#include "SLAM_rtwutil.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// UNDISTORTPOINT Undistort a point (or points) that are from a camera with
// the calibration cameraparams
//    Undistort a point or set of points from one camera. Depending on the
//    camera model used to calibrate the camera, the appropriate undistortion
//    is applied
// Arguments    : const double pt_d_data[]
//                const int pt_d_size[1]
//                const double cameraparams_FocalLength[2]
//                const double cameraparams_PrincipalPoint[2]
//                const double cameraparams_RadialDistortion[3]
//                double pt_u_data[]
//                int pt_u_size[1]
// Return Type  : void
//
void undistortPoint(const double pt_d_data[], const int pt_d_size[1], const
                    double cameraparams_FocalLength[2], const double
                    cameraparams_PrincipalPoint[2], const double
                    cameraparams_RadialDistortion[3], double pt_u_data[], int
                    pt_u_size[1])
{
  int loop_ub;
  int i12;
  double fx;
  double fy;
  double Cx;
  double Cy;
  double k1;
  double k2;
  double k3;
  double d1;
  int i;
  double pt_d_n[2];
  double r_d_sq;
  double r_u_sq;
  boolean_T exitg1;
  double a;
  double b_a;
  double diff;
  double coeff;

  // % Plumb Bob
  pt_u_size[0] = pt_d_size[0];
  loop_ub = pt_d_size[0];
  for (i12 = 0; i12 < loop_ub; i12++) {
    pt_u_data[i12] = pt_d_data[i12];
  }

  fx = cameraparams_FocalLength[0];
  fy = cameraparams_FocalLength[1];
  Cx = cameraparams_PrincipalPoint[0];
  Cy = cameraparams_PrincipalPoint[1];
  k1 = cameraparams_RadialDistortion[0];
  k2 = cameraparams_RadialDistortion[1];
  k3 = cameraparams_RadialDistortion[2];
  d1 = (double)pt_d_size[0] / 2.0;
  for (i = 0; i < (int)d1; i++) {
    pt_d_n[0] = (pt_d_data[i << 1] - Cx) / fx;
    pt_d_n[1] = (pt_d_data[(i << 1) + 1] - Cy) / fy;
    r_d_sq = pt_d_n[0] * pt_d_n[0] + pt_d_n[1] * pt_d_n[1];

    // get_r_u Get undistorted radius from distorted radius
    //    Get the pixel radius of the undistorted pixels from a distorted pixel
    //    radius and distortion parameters
    r_u_sq = r_d_sq;
    loop_ub = 0;
    exitg1 = false;
    while ((!exitg1) && (loop_ub < 100)) {
      a = ((1.0 + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq)) + k3 * rt_powd_snf
        (r_u_sq, 3.0);
      b_a = (1.0 + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq) * k3 * rt_powd_snf
        (r_u_sq, 3.0);
      diff = (r_u_sq * (a * a) - r_d_sq) / (b_a * b_a + 2.0 * r_u_sq * (((1.0 +
        2.0 * k1 * r_u_sq) + 2.0 * k2 * (r_u_sq * r_u_sq)) + 2.0 * k3 *
        rt_powd_snf(r_u_sq, 3.0)));
      r_u_sq -= diff;
      if ((diff < 1.0E-6) && (diff > -1.0E-6)) {
        exitg1 = true;
      } else {
        loop_ub++;
      }
    }

    coeff = ((1.0 + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq)) + k3 * rt_powd_snf
      (r_u_sq, 3.0);
    for (i12 = 0; i12 < 2; i12++) {
      pt_d_n[i12] /= coeff;
    }

    loop_ub = i << 1;
    pt_u_data[loop_ub] = pt_d_n[0] * fx + Cx;
    pt_u_data[1 + loop_ub] = pt_d_n[1] * fy + Cy;
  }
}

//
// File trailer for undistortPoint.cpp
//
// [EOF]
//
