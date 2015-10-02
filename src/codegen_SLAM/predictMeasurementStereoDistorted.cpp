//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: predictMeasurementStereoDistorted.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "predictMeasurementStereoDistorted.h"
#include "SLAM_rtwutil.h"
#include <ros/console.h>

// Function Declarations
static void predictMeasurementDistortedPB(const double fp[3], const double
  cameraParams_FocalLength[2], const double cameraParams_PrincipalPoint[2],
  const double cameraParams_RadialDistortion[3], double h_d[2]);

// Function Definitions

//
// Arguments    : const double fp[3]
//                const double cameraParams_FocalLength[2]
//                const double cameraParams_PrincipalPoint[2]
//                const double cameraParams_RadialDistortion[3]
//                double h_d[2]
// Return Type  : void
//
static void predictMeasurementDistortedPB(const double fp[3], const double
  cameraParams_FocalLength[2], const double cameraParams_PrincipalPoint[2],
  const double cameraParams_RadialDistortion[3], double h_d[2])
{
  double h_cin[2];
  int i;
  double radsq;
  double b;
  for (i = 0; i < 2; i++) {
    h_cin[i] = fp[i] / fp[2];
  }

  radsq = h_cin[0] * h_cin[0] + h_cin[1] * h_cin[1];
  b = ((1.0 + cameraParams_RadialDistortion[0] * radsq) +
       cameraParams_RadialDistortion[1] * (radsq * radsq)) +
    cameraParams_RadialDistortion[2] * rt_powd_snf(radsq, 4.0);
  for (i = 0; i < 2; i++) {
    h_cin[i] *= b;
  }

  h_d[0] = cameraParams_PrincipalPoint[0] + cameraParams_FocalLength[0] * h_cin
    [0];
  h_d[1] = cameraParams_PrincipalPoint[1] + cameraParams_FocalLength[1] * h_cin
    [1];
}

//
// PREDICTMEASUREMENT Predict the measurement of a feature given in the left
// camera frame
//    Get the pixel coordinates where a feature given in the left camera
//    frame would be visible in both cameras. Depending on the
//    camera model used to calibrate the camera, the appropriate undistortion
//    is applied
// Arguments    : const double fp_l[3]
//                const double c_stereoParams_CameraParameters[2]
//                const double d_stereoParams_CameraParameters[2]
//                const double e_stereoParams_CameraParameters[3]
//                const double f_stereoParams_CameraParameters[2]
//                const double g_stereoParams_CameraParameters[2]
//                const double h_stereoParams_CameraParameters[3]
//                const double stereoParams_r_lr[3]
//                const double stereoParams_R_rl[9]
//                double h_d_l[2]
//                double h_d_r[2]
// Return Type  : void
//
void c_predictMeasurementStereoDisto(const double fp_l[3], const double
  c_stereoParams_CameraParameters[2], const double
  d_stereoParams_CameraParameters[2], const double
  e_stereoParams_CameraParameters[3], const double
  f_stereoParams_CameraParameters[2], const double
  g_stereoParams_CameraParameters[2], const double
  h_stereoParams_CameraParameters[3], const double stereoParams_r_lr[3], const
  double stereoParams_R_rl[9], double h_d_l[2], double h_d_r[2])
{
  double b_stereoParams_R_rl[3];
  int i28;
  double d3;
  int i29;
  predictMeasurementDistortedPB(fp_l, c_stereoParams_CameraParameters,
    d_stereoParams_CameraParameters, e_stereoParams_CameraParameters, h_d_l);
  for (i28 = 0; i28 < 3; i28++) {
    d3 = 0.0;
    for (i29 = 0; i29 < 3; i29++) {
      d3 += stereoParams_R_rl[i28 + 3 * i29] * fp_l[i29];
    }

    b_stereoParams_R_rl[i28] = d3 - stereoParams_r_lr[i28];
  }

  predictMeasurementDistortedPB(b_stereoParams_R_rl,
    f_stereoParams_CameraParameters, g_stereoParams_CameraParameters,
    h_stereoParams_CameraParameters, h_d_r);
}

//
// File trailer for predictMeasurementStereoDistorted.cpp
//
// [EOF]
//
