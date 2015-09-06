//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: undistortPoint.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Sep-2015 10:04:04
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
// Arguments    : const double pt_d[2]
//                const double c_cameraParameters_RadialDistor[3]
//                const double cameraParameters_FocalLength[2]
//                const double cameraParameters_PrincipalPoint[2]
//                double pt_u[2]
// Return Type  : void
//
void undistortPoint(const double pt_d[2], const double
                    c_cameraParameters_RadialDistor[3], const double
                    cameraParameters_FocalLength[2], const double
                    cameraParameters_PrincipalPoint[2], double pt_u[2])
{
  double pt_d_n_idx_0;
  double pt_d_n_idx_1;
  double r_u;
  double coeff;
  pt_d_n_idx_0 = (pt_d[0] - cameraParameters_PrincipalPoint[0]) /
    cameraParameters_FocalLength[0];
  pt_d_n_idx_1 = (pt_d[1] - cameraParameters_PrincipalPoint[1]) /
    cameraParameters_FocalLength[1];
  r_u = get_r_u(c_cameraParameters_RadialDistor[0],
                c_cameraParameters_RadialDistor[1],
                c_cameraParameters_RadialDistor[2], sqrt(pt_d_n_idx_0 *
    pt_d_n_idx_0 + pt_d_n_idx_1 * pt_d_n_idx_1));
  coeff = ((1.0 + c_cameraParameters_RadialDistor[0] * (r_u * r_u)) +
           c_cameraParameters_RadialDistor[1] * rt_powd_snf(r_u, 4.0)) +
    c_cameraParameters_RadialDistor[2] * rt_powd_snf(r_u, 6.0);
  pt_u[0] = pt_d_n_idx_0 / coeff;
  pt_u[1] = pt_d_n_idx_1 / coeff;
}

//
// File trailer for undistortPoint.cpp
//
// [EOF]
//
