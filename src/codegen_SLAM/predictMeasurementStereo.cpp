//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: predictMeasurementStereo.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "predictMeasurementStereo.h"
#include <ros/console.h>

// Function Definitions

//
// predictMeasurementStereo Predict the measurement of a feature given in the left
// camera frame
//    Get the pixel undistorted coordinates where a feature given in the left camera
//    frame would be visible in both cameras
// Arguments    : const double fp_l[3]
//                const double c_stereoParams_CameraParameters[2]
//                const double d_stereoParams_CameraParameters[2]
//                const double e_stereoParams_CameraParameters[2]
//                const double f_stereoParams_CameraParameters[2]
//                const double stereoParams_r_lr[3]
//                const double stereoParams_R_rl[9]
//                double h_cin_l[2]
//                double h_cin_r[2]
// Return Type  : void
//
void predictMeasurementStereo(const double fp_l[3], const double
  c_stereoParams_CameraParameters[2], const double
  d_stereoParams_CameraParameters[2], const double
  e_stereoParams_CameraParameters[2], const double
  f_stereoParams_CameraParameters[2], const double stereoParams_r_lr[3], const
  double stereoParams_R_rl[9], double h_cin_l[2], double h_cin_r[2])
{
  double h_c_n_l[2];
  int i;
  double fp_r[3];
  double d2;
  int i15;

  // predictMeasurementLeft Predict the measurement of a feature given in the left 
  // camera frame
  //    Get the normalized pixel coordinates where a feature given in the left camera 
  //    frame
  for (i = 0; i < 2; i++) {
    h_c_n_l[i] = fp_l[i] / fp_l[2];
  }

  //  normalized feature in camera frame
  h_cin_l[0] = h_c_n_l[0] * c_stereoParams_CameraParameters[0] +
    d_stereoParams_CameraParameters[0];
  h_cin_l[1] = h_c_n_l[1] * c_stereoParams_CameraParameters[1] +
    d_stereoParams_CameraParameters[1];
  for (i = 0; i < 3; i++) {
    d2 = 0.0;
    for (i15 = 0; i15 < 3; i15++) {
      d2 += stereoParams_R_rl[i + 3 * i15] * fp_l[i15];
    }

    fp_r[i] = d2 - stereoParams_r_lr[i];
  }

  // predictMeasurementLeft Predict the measurement of a feature given in the left 
  // camera frame
  //    Get the normalized pixel coordinates where a feature given in the left camera 
  //    frame
  for (i = 0; i < 2; i++) {
    h_c_n_l[i] = fp_r[i] / fp_r[2];
  }

  //  normalized feature in camera frame
  h_cin_r[0] = h_c_n_l[0] * e_stereoParams_CameraParameters[0] +
    f_stereoParams_CameraParameters[0];
  h_cin_r[1] = h_c_n_l[1] * e_stereoParams_CameraParameters[1] +
    f_stereoParams_CameraParameters[1];
}

//
// File trailer for predictMeasurementStereo.cpp
//
// [EOF]
//
