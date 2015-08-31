//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: undistortPoint.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 31-Aug-2015 20:50:24
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "undistortPoint.h"
#include "predictMeasurement_stereo.h"
#include "get_r_u.h"
#include "SLAM_rtwutil.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// undistortPoint Undistort a distorted pixel point
//    pt_d is a distorted pixel point (or a column vector of distorted points)
//    from a camera with camera parameters cameraParameters.
//    pt_u is a undistorted pixel point (or a column vector of undistorted points)
//    in normalized pixel coordinates, i.e.
//    the principal point is at (0, 0), the focal length is (1, 1)
// Arguments    : const double pt_d_data[]
//                const int pt_d_size[1]
//                const double c_cameraParameters_CameraParame[3]
//                const double d_cameraParameters_CameraParame[2]
//                const double e_cameraParameters_CameraParame[2]
//                double pt_u_data[]
//                int pt_u_size[1]
// Return Type  : void
//
void undistortPoint(const double pt_d_data[], const int pt_d_size[1], const
                    double c_cameraParameters_CameraParame[3], const double
                    d_cameraParameters_CameraParame[2], const double
                    e_cameraParameters_CameraParame[2], double pt_u_data[], int
                    pt_u_size[1])
{
  int loop_ub;
  int i;
  double k1_l;
  double k2_l;
  double k3_l;
  double d0;
  double pt_d_n_idx_0;
  double pt_d_n_idx_1;
  double r_u;
  double coeff;
  pt_u_size[0] = pt_d_size[0];
  loop_ub = pt_d_size[0];
  for (i = 0; i < loop_ub; i++) {
    pt_u_data[i] = pt_d_data[i];
  }

  k1_l = c_cameraParameters_CameraParame[0];
  k2_l = c_cameraParameters_CameraParame[1];
  k3_l = c_cameraParameters_CameraParame[2];
  d0 = (double)pt_d_size[0] / 2.0;
  for (loop_ub = 0; loop_ub < (int)d0; loop_ub++) {
    pt_d_n_idx_0 = (pt_d_data[loop_ub << 1] - e_cameraParameters_CameraParame[0])
      / d_cameraParameters_CameraParame[0];
    pt_d_n_idx_1 = (pt_d_data[(loop_ub << 1) + 1] -
                    e_cameraParameters_CameraParame[1]) /
      d_cameraParameters_CameraParame[1];
    r_u = get_r_u(k1_l, k2_l, k3_l, sqrt(pt_d_n_idx_0 * pt_d_n_idx_0 +
      pt_d_n_idx_1 * pt_d_n_idx_1));
    coeff = ((1.0 + k1_l * (r_u * r_u)) + k2_l * rt_powd_snf(r_u, 4.0)) + k3_l *
      rt_powd_snf(r_u, 6.0);
    i = loop_ub << 1;
    pt_u_data[i] = pt_d_n_idx_0 / coeff;
    pt_u_data[1 + i] = pt_d_n_idx_1 / coeff;
  }
}

//
// File trailer for undistortPoint.cpp
//
// [EOF]
//
