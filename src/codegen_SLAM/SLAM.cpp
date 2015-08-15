//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 15-Aug-2015 13:16:41
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "SLAM_emxutil.h"
#include "fprintf.h"
#include "SLAM_updIT.h"
#include "predictMeasurement_stereo.h"
#include "SLAM_pred.h"
#include "blkdiag.h"
#include "repmat.h"
#include "QuatFromRotJ.h"
#include "norm.h"
#include "SLAM_rtwutil.h"
#include "SLAM_data.h"
#include <stdio.h>

// Type Definitions
typedef struct {
  char Name[30];
  char Version[3];
  char Release[8];
  char Date[11];
} struct_T;

typedef struct {
  double RadialDistortion[3];
  double TangentialDistortion[2];
  boolean_T EstimateSkew;
  double NumRadialDistortionCoefficients;
  boolean_T EstimateTangentialDistortion;
  double NumPatterns;
  double IntrinsicMatrix[9];
  double FocalLength[2];
  double PrincipalPoint[2];
  double Skew;
  double MeanReprojectionError;
  double IntrinsicMatrixInternal[9];
  struct_T Version;
} b_struct_T;

typedef struct {
  double Xmap[4];
  double Ymap[4];
  double XmapSingle[4];
  double YmapSingle[4];
  double NewOrigin[2];
} c_struct_T;

typedef struct {
  double T[9];
  double Dimensionality;
} d_struct_T;

typedef struct {
  d_struct_T H1;
  d_struct_T H2;
  double Q[16];
  double XBounds[2];
  double YBounds[2];
  boolean_T Initialized;
  double RectifiedImageSize[2];
} e_struct_T;

typedef struct {
  b_struct_T CameraParameters1;
  b_struct_T CameraParameters2;
  double RotationOfCamera2[9];
  double TranslationOfCamera2[3];
  double FundamentalMatrix[9];
  double EssentialMatrix[9];
  double MeanReprojectionError;
  double NumPatterns;
  double WorldPoints[80];
  char WorldUnits[2];
  c_struct_T RectifyMap1;
  c_struct_T RectifyMap2;
  e_struct_T RectificationParams;
  struct_T Version;
  double r_lr[3];
  double R_lr[9];
  double R_rl[9];
} f_struct_T;

// Variable Definitions
static boolean_T initialized_not_empty;
static emxArray_real_T *xt;
static emxArray_real_T *P;
static f_struct_T cameraparams;
static double delayBuffer_k[84];
static double init_counter;
static double P_att[9];
static double x_att[4];

// Function Definitions

//
// Arguments    : double updateVect[16]
//                const double z_all_l[32]
//                const double z_all_r[32]
//                double dt
//                const double processNoise[4]
//                double IMU_measurements[23]
//                const double imNoise[2]
//                double numPointsPerAnchor
//                double numAnchors
//                emxArray_real_T *h_u_apo_out
//                emxArray_real_T *xt_out
//                emxArray_real_T *P_apo_out
//                emxArray_real_T *map_out
// Return Type  : void
//
void SLAM(double updateVect[16], const double z_all_l[32], const double z_all_r
          [32], double dt, const double processNoise[4], double
          IMU_measurements[23], const double imNoise[2], double
          numPointsPerAnchor, double numAnchors, emxArray_real_T *h_u_apo_out,
          emxArray_real_T *xt_out, emxArray_real_T *P_apo_out, emxArray_real_T
          *map_out)
{
  double B;
  double z_n_b[3];
  int k;
  double y_n_b[3];
  int i13;
  double x_n_b[3];
  double b_x_n_b[9];
  static const double dv34[3] = { -0.414085141240295, 0.236451305145822,
    -0.0871296995623235 };

  static const double dv35[9] = { 268.155648020127, 0.0, 155.972717007495, 0.0,
    268.867732741683, 113.206085625994, 0.0, 0.0, 1.0 };

  static const double dv36[9] = { 268.155648020127, 0.0, 0.0, 0.0,
    268.867732741683, 0.0, 155.972717007495, 113.206085625994, 1.0 };

  static const char cv0[30] = { 'C', 'o', 'm', 'p', 'u', 't', 'e', 'r', ' ', 'V',
    'i', 's', 'i', 'o', 'n', ' ', 'S', 'y', 's', 't', 'e', 'm', ' ', 'T', 'o',
    'o', 'l', 'b', 'o', 'x' };

  static const char cv1[3] = { '6', '.', '2' };

  static const char cv2[8] = { '(', 'R', '2', '0', '1', '5', 'a', ')' };

  static const char cv3[11] = { '0', '9', '-', 'F', 'e', 'b', '-', '2', '0', '1',
    '5' };

  static const double dv37[3] = { -0.410786366925601, 0.222940449996276,
    -0.0755554113677893 };

  static const double dv38[9] = { 268.839577384212, 0.0, 167.100031218981, 0.0,
    269.510643351885, 107.901779803044, 0.0, 0.0, 1.0 };

  static const double dv39[9] = { 268.839577384212, 0.0, 0.0, 0.0,
    269.510643351885, 0.0, 167.100031218981, 107.901779803044, 1.0 };

  static const double dv40[9] = { 0.99986163923822, -0.00276356090334069,
    0.0164032042904086, 0.00285714926837293, 0.999979759652258,
    -0.00568480288764035, -0.0163871619848462, 0.00573088273711954,
    0.999849297596961 };

  static const double dv41[3] = { -29.5590877728364, -0.181335935104241,
    0.253273247293606 };

  static const double dv42[9] = { 3.15512205825112E-8, 3.49016833440671E-6,
    0.000299677211697942, -1.02113807360447E-5, 2.30896601934282E-6,
    -0.108449956429672, 0.000172793765317693, 0.108866982432858,
    0.732410597409066 };

  static const double dv43[9] = { 0.0022745543446244, 0.252237261907493,
    0.182760086623752, -0.738101959590127, 0.167313948053832, -29.5504827175476,
    -0.262999184633548, 29.5579713827866, 0.172371247184576 };

  static const short iv9[80] = { 0, 0, 0, 0, 0, 120, 120, 120, 120, 120, 240,
    240, 240, 240, 240, 360, 360, 360, 360, 360, 480, 480, 480, 480, 480, 600,
    600, 600, 600, 600, 720, 720, 720, 720, 720, 840, 840, 840, 840, 840, 0, 120,
    240, 360, 480, 0, 120, 240, 360, 480, 0, 120, 240, 360, 480, 0, 120, 240,
    360, 480, 0, 120, 240, 360, 480, 0, 120, 240, 360, 480, 0, 120, 240, 360,
    480, 0, 120, 240, 360, 480 };

  static const signed char iv10[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const signed char iv11[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 1 };

  static const double dv44[3] = { 0.0295590877728364, 0.000181335935104241,
    -0.000253273247293606 };

  static const double dv45[9] = { 0.99986163923822, 0.00285714926837293,
    -0.0163871619848462, -0.00276356090334069, 0.999979759652258,
    0.00573088273711954, 0.0164032042904086, -0.00568480288764035,
    0.999849297596961 };

  emxArray_real_T *r4;
  emxArray_real_T *r5;
  emxArray_real_T *r6;
  emxArray_real_T *r7;
  static const signed char y[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const double b_y[9] = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01
  };

  double dv46[3];
  static const double a[3] = { -0.0132, -0.4398, 0.2673 };

  double b_a[3];
  double c_a[3];
  static const double d_a[9] = { -0.0077435500000000435, 0.0086606800000000872,
    0.99989175999999991, -0.99987860000000006, -0.010132990000000119,
    -0.0076556800000000536, 0.010066000000000075, -0.9998704,
    0.0087384500000000087 };

  int tmp_data[16];

  //  persistents for attitude estimator
  //  for coder
  // % imu hack
  // % finish imu hack
  if (!initialized_not_empty) {
    //  initialization for attitude filter
    // delayBuffer_k_1=[0;0;0;0;0;0];
    B = norm(*(double (*)[3])&IMU_measurements[3]);
    for (k = 0; k < 3; k++) {
      z_n_b[k] = IMU_measurements[k + 3] / B;
    }

    // m_n_b=IMU_measurements(11:13);
    y_n_b[0] = z_n_b[1] * 0.0 - z_n_b[2] * 0.0;
    y_n_b[1] = z_n_b[2] - z_n_b[0] * 0.0;
    y_n_b[2] = z_n_b[0] * 0.0 - z_n_b[1];
    B = norm(y_n_b);
    for (i13 = 0; i13 < 3; i13++) {
      y_n_b[i13] /= B;
    }

    x_n_b[0] = y_n_b[1] * z_n_b[2] - y_n_b[2] * z_n_b[1];
    x_n_b[1] = y_n_b[2] * z_n_b[0] - y_n_b[0] * z_n_b[2];
    x_n_b[2] = y_n_b[0] * z_n_b[1] - y_n_b[1] * z_n_b[0];
    B = norm(x_n_b);
    for (i13 = 0; i13 < 3; i13++) {
      b_x_n_b[i13] = x_n_b[i13] / B;
      b_x_n_b[3 + i13] = y_n_b[i13];
      b_x_n_b[6 + i13] = z_n_b[i13];
    }

    QuatFromRotJ(b_x_n_b, x_att);
    memset(&P_att[0], 0, 9U * sizeof(double));

    //  other initialization
    initialized_not_empty = true;
    for (k = 0; k < 3; k++) {
      P_att[k + 3 * k] = 1.0;

      //  Autogenerated function that constructs and returns a hard coded struct. 
      //  Generated on 07-Aug-2015 09:51:40.
      cameraparams.CameraParameters1.RadialDistortion[k] = dv34[k];
    }

    for (i13 = 0; i13 < 2; i13++) {
      cameraparams.CameraParameters1.TangentialDistortion[i13] = 0.0;
    }

    cameraparams.CameraParameters1.EstimateSkew = false;
    cameraparams.CameraParameters1.NumRadialDistortionCoefficients = 3.0;
    cameraparams.CameraParameters1.EstimateTangentialDistortion = false;
    cameraparams.CameraParameters1.NumPatterns = 103.0;
    for (i13 = 0; i13 < 9; i13++) {
      cameraparams.CameraParameters1.IntrinsicMatrix[i13] = dv35[i13];
    }

    for (i13 = 0; i13 < 2; i13++) {
      cameraparams.CameraParameters1.FocalLength[i13] = 268.155648020127 +
        0.71208472155603886 * (double)i13;
      cameraparams.CameraParameters1.PrincipalPoint[i13] = 155.972717007495 +
        -42.766631381500986 * (double)i13;
    }

    cameraparams.CameraParameters1.Skew = 0.0;
    cameraparams.CameraParameters1.MeanReprojectionError = 0.178947558747657;
    for (i13 = 0; i13 < 9; i13++) {
      cameraparams.CameraParameters1.IntrinsicMatrixInternal[i13] = dv36[i13];
    }

    for (i13 = 0; i13 < 30; i13++) {
      cameraparams.CameraParameters1.Version.Name[i13] = cv0[i13];
    }

    for (i13 = 0; i13 < 3; i13++) {
      cameraparams.CameraParameters1.Version.Version[i13] = cv1[i13];
    }

    for (i13 = 0; i13 < 8; i13++) {
      cameraparams.CameraParameters1.Version.Release[i13] = cv2[i13];
    }

    for (i13 = 0; i13 < 11; i13++) {
      cameraparams.CameraParameters1.Version.Date[i13] = cv3[i13];
    }

    for (i13 = 0; i13 < 3; i13++) {
      cameraparams.CameraParameters2.RadialDistortion[i13] = dv37[i13];
    }

    for (i13 = 0; i13 < 2; i13++) {
      cameraparams.CameraParameters2.TangentialDistortion[i13] = 0.0;
    }

    cameraparams.CameraParameters2.EstimateSkew = false;
    cameraparams.CameraParameters2.NumRadialDistortionCoefficients = 3.0;
    cameraparams.CameraParameters2.EstimateTangentialDistortion = false;
    cameraparams.CameraParameters2.NumPatterns = 103.0;
    for (i13 = 0; i13 < 9; i13++) {
      cameraparams.CameraParameters2.IntrinsicMatrix[i13] = dv38[i13];
    }

    for (i13 = 0; i13 < 2; i13++) {
      cameraparams.CameraParameters2.FocalLength[i13] = 268.839577384212 +
        0.67106596767297333 * (double)i13;
      cameraparams.CameraParameters2.PrincipalPoint[i13] = 167.100031218981 +
        -59.198251415937008 * (double)i13;
    }

    cameraparams.CameraParameters2.Skew = 0.0;
    cameraparams.CameraParameters2.MeanReprojectionError = 0.176667688316263;
    for (i13 = 0; i13 < 9; i13++) {
      cameraparams.CameraParameters2.IntrinsicMatrixInternal[i13] = dv39[i13];
    }

    for (i13 = 0; i13 < 30; i13++) {
      cameraparams.CameraParameters2.Version.Name[i13] = cv0[i13];
    }

    for (i13 = 0; i13 < 3; i13++) {
      cameraparams.CameraParameters2.Version.Version[i13] = cv1[i13];
    }

    for (i13 = 0; i13 < 8; i13++) {
      cameraparams.CameraParameters2.Version.Release[i13] = cv2[i13];
    }

    for (i13 = 0; i13 < 11; i13++) {
      cameraparams.CameraParameters2.Version.Date[i13] = cv3[i13];
    }

    for (i13 = 0; i13 < 9; i13++) {
      cameraparams.RotationOfCamera2[i13] = dv40[i13];
    }

    for (i13 = 0; i13 < 3; i13++) {
      cameraparams.TranslationOfCamera2[i13] = dv41[i13];
    }

    for (i13 = 0; i13 < 9; i13++) {
      cameraparams.FundamentalMatrix[i13] = dv42[i13];
      cameraparams.EssentialMatrix[i13] = dv43[i13];
    }

    cameraparams.MeanReprojectionError = 0.17780762353196;
    cameraparams.NumPatterns = 103.0;
    for (i13 = 0; i13 < 80; i13++) {
      cameraparams.WorldPoints[i13] = iv9[i13];
    }

    for (i13 = 0; i13 < 2; i13++) {
      cameraparams.WorldUnits[i13] = 'm';
    }

    for (i13 = 0; i13 < 4; i13++) {
      cameraparams.RectifyMap1.Xmap[i13] = 0.0;
      cameraparams.RectifyMap1.Ymap[i13] = 0.0;
      cameraparams.RectifyMap1.XmapSingle[i13] = 0.0;
      cameraparams.RectifyMap1.YmapSingle[i13] = 0.0;
    }

    for (i13 = 0; i13 < 2; i13++) {
      cameraparams.RectifyMap1.NewOrigin[i13] = 0.0;
    }

    for (i13 = 0; i13 < 4; i13++) {
      cameraparams.RectifyMap2.Xmap[i13] = 0.0;
      cameraparams.RectifyMap2.Ymap[i13] = 0.0;
      cameraparams.RectifyMap2.XmapSingle[i13] = 0.0;
      cameraparams.RectifyMap2.YmapSingle[i13] = 0.0;
    }

    for (i13 = 0; i13 < 2; i13++) {
      cameraparams.RectifyMap2.NewOrigin[i13] = 0.0;
    }

    for (i13 = 0; i13 < 9; i13++) {
      cameraparams.RectificationParams.H1.T[i13] = iv10[i13];
    }

    cameraparams.RectificationParams.H1.Dimensionality = 2.0;
    for (i13 = 0; i13 < 9; i13++) {
      cameraparams.RectificationParams.H2.T[i13] = iv10[i13];
    }

    cameraparams.RectificationParams.H2.Dimensionality = 2.0;
    for (i13 = 0; i13 < 16; i13++) {
      cameraparams.RectificationParams.Q[i13] = iv11[i13];
    }

    for (i13 = 0; i13 < 2; i13++) {
      cameraparams.RectificationParams.XBounds[i13] = 0.0;
      cameraparams.RectificationParams.YBounds[i13] = 0.0;
    }

    cameraparams.RectificationParams.Initialized = true;
    for (i13 = 0; i13 < 2; i13++) {
      cameraparams.RectificationParams.RectifiedImageSize[i13] = 1.0;
    }

    for (i13 = 0; i13 < 30; i13++) {
      cameraparams.Version.Name[i13] = cv0[i13];
    }

    for (i13 = 0; i13 < 3; i13++) {
      cameraparams.Version.Version[i13] = cv1[i13];
    }

    for (i13 = 0; i13 < 8; i13++) {
      cameraparams.Version.Release[i13] = cv2[i13];
    }

    for (i13 = 0; i13 < 11; i13++) {
      cameraparams.Version.Date[i13] = cv3[i13];
    }

    for (k = 0; k < 3; k++) {
      cameraparams.r_lr[k] = dv44[k];
    }

    for (i13 = 0; i13 < 9; i13++) {
      cameraparams.R_lr[i13] = dv40[i13];
      cameraparams.R_rl[i13] = dv45[i13];
    }

    b_emxInit_real_T(&r4, 1);
    i13 = r4->size[0];
    r4->size[0] = 7 + (int)numPointsPerAnchor;
    emxEnsureCapacity((emxArray__common *)r4, i13, (int)sizeof(double));
    r4->data[0] = 0.0;
    r4->data[1] = 0.0;
    r4->data[2] = 0.0;
    r4->data[3] = 0.0;
    r4->data[4] = 0.0;
    r4->data[5] = 0.0;
    r4->data[6] = 1.0;
    k = (int)numPointsPerAnchor;
    for (i13 = 0; i13 < k; i13++) {
      r4->data[i13 + 7] = 0.0;
    }

    b_emxInit_real_T(&r5, 1);
    repmat(r4, numAnchors, r5);
    i13 = xt->size[0];
    xt->size[0] = 13 + r5->size[0];
    emxEnsureCapacity((emxArray__common *)xt, i13, (int)sizeof(double));
    xt->data[0] = 0.0;
    xt->data[1] = 0.0;
    xt->data[2] = 0.0;
    xt->data[3] = 0.0;
    xt->data[4] = 0.0;
    xt->data[5] = 0.0;
    xt->data[6] = 1.0;
    xt->data[7] = 0.0;
    xt->data[8] = 0.0;
    xt->data[9] = 0.0;
    emxFree_real_T(&r4);
    for (i13 = 0; i13 < 3; i13++) {
      xt->data[i13 + 10] = 0.0 * IMU_measurements[i13];
    }

    k = r5->size[0];
    for (i13 = 0; i13 < k; i13++) {
      xt->data[i13 + 13] = r5->data[i13];
    }

    emxFree_real_T(&r5);
    emxInit_real_T(&r6, 2);

    //  initial real vector
    B = numAnchors * (6.0 + numPointsPerAnchor);
    i13 = r6->size[0] * r6->size[1];
    r6->size[0] = (int)numStates;
    r6->size[1] = (int)numStates;
    emxEnsureCapacity((emxArray__common *)r6, i13, (int)sizeof(double));
    k = (int)numStates * (int)numStates;
    for (i13 = 0; i13 < k; i13++) {
      r6->data[i13] = 0.0;
    }

    emxInit_real_T(&r7, 2);
    i13 = r7->size[0] * r7->size[1];
    r7->size[0] = (int)B;
    r7->size[1] = (int)B;
    emxEnsureCapacity((emxArray__common *)r7, i13, (int)sizeof(double));
    k = (int)B * (int)B;
    for (i13 = 0; i13 < k; i13++) {
      r7->data[i13] = 0.0;
    }

    blkdiag(r6, r7, P);

    //  initial error state covariance
    emxFree_real_T(&r7);
    emxFree_real_T(&r6);
    for (i13 = 0; i13 < 3; i13++) {
      for (k = 0; k < 3; k++) {
        P->data[k + P->size[0] * i13] = 0.0;
      }
    }

    //  position
    for (i13 = 0; i13 < 3; i13++) {
      for (k = 0; k < 3; k++) {
        P->data[(k + P->size[0] * (3 + i13)) + 3] = y[k + 3 * i13];
      }
    }

    //  orientation
    //      P(4:6,4:6) = R_iw_init * diag([1 1 0]) * R_iw_init';
    //      P(4:6,4:6) = diag([0 0 1]);
    for (i13 = 0; i13 < 3; i13++) {
      for (k = 0; k < 3; k++) {
        P->data[(k + P->size[0] * (6 + i13)) + 6] = 0.0;
      }
    }

    //  velocity
    for (i13 = 0; i13 < 3; i13++) {
      for (k = 0; k < 3; k++) {
        P->data[(k + P->size[0] * (9 + i13)) + 9] = b_y[k + 3 * i13];
      }
    }

    //  gyro bias
  }

  for (k = 0; k < 13; k++) {
    delayBuffer_k[1 + k] = delayBuffer_k[k];
  }

  for (i13 = 0; i13 < 3; i13++) {
    delayBuffer_k[i13] = 0.0 * IMU_measurements[13 + i13];
  }

  for (i13 = 0; i13 < 3; i13++) {
    delayBuffer_k[i13 + 3] = 0.0 * IMU_measurements[16 + i13];
  }

  for (i13 = 0; i13 < 3; i13++) {
    dv46[i13] = delayBuffer_k[78 + i13] + a[i13];
  }

  for (i13 = 0; i13 < 3; i13++) {
    b_a[i13] = 0.0;
    for (k = 0; k < 3; k++) {
      b_a[i13] += d_a[i13 + 3 * k] * dv46[k];
    }

    IMU_measurements[i13] = b_a[i13];
    c_a[i13] = 0.0;
    for (k = 0; k < 3; k++) {
      c_a[i13] += d_a[i13 + 3 * k] * delayBuffer_k[k + 81];
    }
  }

  for (i13 = 0; i13 < 3; i13++) {
    IMU_measurements[3 + i13] = c_a[i13];
  }

  if (init_counter == 0.0) {
    //  done initializing attitude. Insert the estimated attitude and the covariance into the whole state, request features 
    b_fprintf();
    for (k = 0; k < 16; k++) {
      updateVect[k] = 0.0;
    }

    if (1.0 > numPointsPerAnchor) {
      k = 0;
    } else {
      k = (int)numPointsPerAnchor;
    }

    for (i13 = 0; i13 < k; i13++) {
      tmp_data[i13] = i13;
    }

    for (i13 = 0; i13 < k; i13++) {
      updateVect[tmp_data[i13]] = 2.0;
    }

    for (i13 = 0; i13 < 4; i13++) {
      xt->data[3 + i13] = x_att[i13];
    }

    for (i13 = 0; i13 < 3; i13++) {
      for (k = 0; k < 3; k++) {
        P->data[(k + P->size[0] * (3 + i13)) + 3] = P_att[k + 3 * i13];
      }
    }

    i13 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i13, (int)sizeof(double));
    k = xt->size[0];
    for (i13 = 0; i13 < k; i13++) {
      xt_out->data[i13] = xt->data[i13];
    }

    for (i13 = 0; i13 < 4; i13++) {
      xt_out->data[3 + i13] = x_att[i13];
    }

    i13 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i13, (int)sizeof(double));
    k = P->size[0] * P->size[1];
    for (i13 = 0; i13 < k; i13++) {
      P_apo_out->data[i13] = P->data[i13];
    }

    i13 = h_u_apo_out->size[0];
    h_u_apo_out->size[0] = 64;
    emxEnsureCapacity((emxArray__common *)h_u_apo_out, i13, (int)sizeof(double));
    for (i13 = 0; i13 < 64; i13++) {
      h_u_apo_out->data[i13] = -100.0;
    }

    i13 = map_out->size[0] * map_out->size[1];
    map_out->size[0] = 3;
    map_out->size[1] = 16;
    emxEnsureCapacity((emxArray__common *)map_out, i13, (int)sizeof(double));
    for (i13 = 0; i13 < 48; i13++) {
      map_out->data[i13] = rtNaN;
    }

    init_counter = 1.0;
  } else if (init_counter == 1.0) {
    SLAM_updIT(P, xt, cameraparams.r_lr, cameraparams.R_lr, cameraparams.R_rl,
               updateVect, z_all_l, z_all_r, imNoise, IMU_measurements,
               numPointsPerAnchor, numAnchors, (1.0 - rt_powd_snf
                (IMU_measurements[9] / 101325.0, 0.190284)) * 145366.45,
               h_u_apo_out, map_out);
    i13 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i13, (int)sizeof(double));
    k = xt->size[0];
    for (i13 = 0; i13 < k; i13++) {
      xt_out->data[i13] = xt->data[i13];
    }

    i13 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i13, (int)sizeof(double));
    k = P->size[0] * P->size[1];
    for (i13 = 0; i13 < k; i13++) {
      P_apo_out->data[i13] = P->data[i13];
    }

    init_counter = 2.0;
  } else {
    SLAM_pred(P, xt, dt, processNoise, IMU_measurements, numStates);

    //  [xt,P] =  SLAM_pred_euler(P, xt, dt, processNoise, IMU_measurements, numStates); 
    SLAM_updIT(P, xt, cameraparams.r_lr, cameraparams.R_lr, cameraparams.R_rl,
               updateVect, z_all_l, z_all_r, imNoise, IMU_measurements,
               numPointsPerAnchor, numAnchors, 0.0, h_u_apo_out, map_out);
    i13 = xt_out->size[0];
    xt_out->size[0] = xt->size[0];
    emxEnsureCapacity((emxArray__common *)xt_out, i13, (int)sizeof(double));
    k = xt->size[0];
    for (i13 = 0; i13 < k; i13++) {
      xt_out->data[i13] = xt->data[i13];
    }

    i13 = P_apo_out->size[0] * P_apo_out->size[1];
    P_apo_out->size[0] = P->size[0];
    P_apo_out->size[1] = P->size[1];
    emxEnsureCapacity((emxArray__common *)P_apo_out, i13, (int)sizeof(double));
    k = P->size[0] * P->size[1];
    for (i13 = 0; i13 < k; i13++) {
      P_apo_out->data[i13] = P->data[i13];
    }

    // % output asserts for coder
  }
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_free()
{
  emxFree_real_T(&P);
  emxFree_real_T(&xt);
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_init()
{
  emxInit_real_T(&P, 2);
  b_emxInit_real_T(&xt, 1);
  init_counter = 0.0;
  memset(&delayBuffer_k[0], 0, 84U * sizeof(double));
}

//
// Arguments    : void
// Return Type  : void
//
void initialized_not_empty_init()
{
  initialized_not_empty = false;
}

//
// File trailer for SLAM.cpp
//
// [EOF]
//
