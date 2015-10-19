//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 19-Oct-2015 15:58:38
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include <ros/console.h>
#include <stdio.h>

// Type Definitions
#include <stdio.h>
#ifndef struct_emxArray__common
#define struct_emxArray__common

struct emxArray__common
{
  void *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray__common

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T

struct emxArray_int32_T
{
  int *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_int32_T

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_real_T

typedef struct {
  double pos[3];
  double att[4];
  double gyro_bias[3];
  double acc_bias[3];
} b_struct_T;

typedef struct {
  b_struct_T IMU;
  double pos[3];
  double att[4];
  double vel[3];
} c_struct_T;

typedef struct {
  int anchor_idx;
  double pos[3];
  double att[4];
} d_struct_T;

typedef struct {
  double inverse_depth;
  double m[3];
  double scaled_map_point[3];
  int status;
  int status_idx;
  int P_idx;
} e_struct_T;

typedef struct {
  double pos[3];
  double att[4];
  int P_idx[6];
  e_struct_T feature_states[8];
} f_struct_T;

typedef struct {
  c_struct_T robot_state;
  int fixed_feature;
  d_struct_T origin;
  f_struct_T anchor_states[5];
} g_struct_T;

typedef struct {
  double pos[3];
  double att[4];
} struct_T;

// Named Constants
#define b_debug_level                  (2.0)

// Variable Definitions
static boolean_T initialized_not_empty;
static g_struct_T xt;
static double P[8281];
static double h_u[160];
static double map[120];
static double delayedStatus[40];
static double debug_level;

// Function Declarations
static void OnePointRANSAC_EKF(g_struct_T *b_xt, double b_P[8281], const double
  z_u_l[80], const double cameraparams_FocalLength[2], const double
  cameraparams_PrincipalPoint[2], double noiseParameters_image_noise, int
  c_VIOParameters_max_ekf_iterati, boolean_T c_VIOParameters_delayed_initial,
  boolean_T VIOParameters_RANSAC, int updateVect[40]);
static void QuatFromRotJ(const double R[9], double Q[4]);
static void SLAM_pred_euler(double P_apo[8281], g_struct_T *x, double dt, double
  processNoise_qv, double processNoise_qw, double processNoise_qao, double
  processNoise_qwo, double processNoise_qR_ci, const double
  measurements_acc_duo[3], const double measurements_gyr_duo[3]);
static void SLAM_upd(double P_apr[8281], g_struct_T *b_xt, double
                     c_cameraParams_CameraParameters, const double
                     d_cameraParams_CameraParameters[2], const double
                     e_cameraParams_CameraParameters[2], const double
                     f_cameraParams_CameraParameters[3], double
                     g_cameraParams_CameraParameters, double
                     h_cameraParams_CameraParameters, const double
                     i_cameraParams_CameraParameters[2], const double
                     j_cameraParams_CameraParameters[2], const double
                     k_cameraParams_CameraParameters[3], double
                     l_cameraParams_CameraParameters, const double
                     cameraParams_r_lr[3], const double cameraParams_R_lr[9],
                     const double cameraParams_R_rl[9], int updateVect[40],
                     double z_all_l[80], double z_all_r[80], double
                     noiseParameters_image_noise, double
                     c_noiseParameters_inv_depth_ini, const VIOParameters
                     b_VIOParameters, double h_u_apo[160], double b_map[120],
                     double b_delayedStatus[40]);
static boolean_T any(const boolean_T x[40]);
static boolean_T anyActiveAnchorFeatures(const e_struct_T
  anchor_state_feature_states[8]);
static boolean_T b_any(const boolean_T x[3]);
static void b_diag(const double v[15], double d[225]);
static void b_eye(double I[441]);
static double b_fprintf();
static void b_merge(emxArray_int32_T *idx, emxArray_real_T *x, int offset, int
                    np, int nq, emxArray_int32_T *iwork, emxArray_real_T *xwork);
static void b_mrdivide(const double A[2], const double B[4], double y[2]);
static double b_norm(const double x[4]);
static void b_ros_error();
static void b_ros_info(int varargin_1, int varargin_2, int varargin_3);
static void b_ros_warn(int varargin_1, int varargin_2, int varargin_3);
static void b_sort(double x[8], int idx[8]);
static void b_xgeqp3(double A[30], double tau[5], int jpvt[5]);
static double b_xnrm2(int n, const emxArray_real_T *x, int ix0);
static boolean_T c_any(const double x[5]);
static void c_eye(double I[8281]);
static double c_fprintf(double varargin_1);
static double c_norm(const double x[2]);
static void c_predictMeasurementStereoDisto(const double fp_l[3], double
  c_stereoParams_CameraParameters, const double d_stereoParams_CameraParameters
  [2], const double e_stereoParams_CameraParameters[2], const double
  f_stereoParams_CameraParameters[3], double g_stereoParams_CameraParameters,
  double h_stereoParams_CameraParameters, const double
  i_stereoParams_CameraParameters[2], const double
  j_stereoParams_CameraParameters[2], const double
  k_stereoParams_CameraParameters[3], double l_stereoParams_CameraParameters,
  const double stereoParams_r_lr[3], const double stereoParams_R_rl[9], double
  h_d_l[2], double h_d_r[2]);
static void c_ros_info(int varargin_1, int varargin_2);
static void c_ros_warn();
static void c_sort(emxArray_real_T *x, emxArray_int32_T *idx);
static double c_xnrm2(int n, const emxArray_real_T *x, int ix0);
static void cast(const struct_T x[5], AnchorPose y[5]);
static void cross(const double a[3], const double b[3], double c[3]);
static void d_eye(double I[8281]);
static double d_fprintf(double varargin_1);
static void d_ros_info(int varargin_1);
static void d_ros_warn(int varargin_1, double varargin_2);
static void d_sort(emxArray_real_T *x, int dim, emxArray_int32_T *idx);
static double d_xnrm2(int n, const double x[30], int ix0);
static double det(const double x[36]);
static void diag(const double v[3], double d[9]);
static double e_fprintf(double varargin_1);
static void e_ros_info(int varargin_1);
static void e_ros_warn(signed char varargin_1);
static void eml_signed_integer_colon(int b, emxArray_int32_T *y);
static void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
static void emxFree_int32_T(emxArray_int32_T **pEmxArray);
static void emxFree_real_T(emxArray_real_T **pEmxArray);
static void emxInit_int32_T(emxArray_int32_T **pEmxArray, int b_numDimensions);
static void emxInit_int32_T1(emxArray_int32_T **pEmxArray, int b_numDimensions);
static void emxInit_real_T(emxArray_real_T **pEmxArray, int b_numDimensions);
static void emxInit_real_T1(emxArray_real_T **pEmxArray, int b_numDimensions);
static void eye(double I[9]);
static double f_fprintf(double varargin_1);
static void f_ros_info(int varargin_1, double varargin_2, double varargin_3);
static void f_ros_warn(int varargin_1, double varargin_2);
static void fileManager(FILE * *f, boolean_T *a);
static double g_fprintf(double varargin_1);
static void g_ros_info(int varargin_1);
static void g_ros_warn(double varargin_1);
static void getAnchorPoses(const double xt_origin_pos[3], const double
  xt_origin_att[4], const f_struct_T xt_anchor_states[5], struct_T anchor_poses
  [5]);
static void getH_R_res(const double xt_robot_state_pos[3], const double
  xt_robot_state_att[4], const f_struct_T xt_anchor_states[5], const double
  z_all_l[80], const boolean_T b_status[40], const double
  cameraparams_FocalLength[2], const double cameraparams_PrincipalPoint[2],
  double noiseParameters_image_noise, emxArray_real_T *r, emxArray_real_T *H,
  emxArray_real_T *R);
static void getMap(const double xt_origin_pos[3], const double xt_origin_att[4],
                   const f_struct_T xt_anchor_states[5], double b_map[120]);
static double getNumValidFeatures(const e_struct_T anchor_state_feature_states[8]);
static void getScaledMap(g_struct_T *b_xt);
static double getTotalNumActiveFeatures(const f_struct_T xt_anchor_states[5]);
static double getTotalNumDelayedFeatures(const f_struct_T xt_anchor_states[5]);
static void getWorldState(const double xt_robot_state_IMU_pos[3], const double
  xt_robot_state_IMU_att[4], const double xt_robot_state_IMU_gyro_bias[3], const
  double xt_robot_state_IMU_acc_bias[3], const double xt_robot_state_pos[3],
  const double xt_robot_state_att[4], const double xt_robot_state_vel[3], const
  double xt_origin_pos[3], const double xt_origin_att[4], double
  world_state_pos[3], double world_state_att[4], double world_state_vel[3],
  double world_state_IMU_gyro_bias[3], double world_state_IMU_acc_bias[3],
  double world_state_IMU_pos[3], double world_state_IMU_att[4]);
static double h_fprintf(double varargin_1, double varargin_2, double varargin_3);
static void h_ros_info(int varargin_1);
static void h_ros_warn(int varargin_1, int varargin_2, int varargin_3);
static double i_fprintf(double varargin_1, double varargin_2, double varargin_3);
static void i_ros_warn(int varargin_1);
static void initializePoint(const double z_u_l[2], const double z_u_r[2], const
  double c_cameraparams_CameraParameters[2], const double
  d_cameraparams_CameraParameters[2], const double
  e_cameraparams_CameraParameters[2], const double
  f_cameraparams_CameraParameters[2], const double cameraparams_r_lr[3], const
  double cameraparams_R_lr[9], double fp[3], double b_m[6], boolean_T *success);
static double j_fprintf(double varargin_1);
static void j_ros_warn();
static double k_fprintf(double varargin_1);
static double l_fprintf(int varargin_1);
static void lusolve(const emxArray_real_T *A, emxArray_real_T *B);
static double m_fprintf(int varargin_1);
static double median(const double x_data[], const int x_size[1]);
static void merge(int idx[8], double x[8], int offset, int np, int nq, int
                  iwork[8], double xwork[8]);
static void merge_block(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
  int n, int preSortLevel, emxArray_int32_T *iwork, emxArray_real_T *xwork);
static void mrdivide(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *y);
static void multiplyIdx(const double idx_data[], const int idx_size[1], double
  idx_mult_data[], int idx_mult_size[1]);
static double n_fprintf(int varargin_1);
static double norm(const double x[3]);
static double o_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static double p_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static void predictMeasurementDistortedAtan(const double fp[3], const double
  cameraParams_FocalLength[2], const double cameraParams_PrincipalPoint[2],
  const double cameraParams_RadialDistortion[3], double h_d[2]);
static void predictMeasurementDistortedPB(const double fp[3], const double
  cameraParams_FocalLength[2], const double cameraParams_PrincipalPoint[2],
  const double cameraParams_RadialDistortion[3], double h_d[2]);
static void predictMeasurementMono(const double fp[3], const double
  cameraparams_FocalLength[2], const double cameraparams_PrincipalPoint[2],
  double px[2]);
static void predictMeasurementStereo(const double fp_l[3], const double
  c_stereoParams_CameraParameters[2], const double
  d_stereoParams_CameraParameters[2], const double
  e_stereoParams_CameraParameters[2], const double
  f_stereoParams_CameraParameters[2], const double stereoParams_r_lr[3], const
  double stereoParams_R_rl[9], double h_cin_l[2], double h_cin_r[2]);
static void printParams(double c_noiseParameters_process_noise, double
  d_noiseParameters_process_noise, double e_noiseParameters_process_noise,
  double f_noiseParameters_process_noise, double g_noiseParameters_process_noise,
  const double c_noiseParameters_gyro_bias_ini[3], const double
  c_noiseParameters_acc_bias_init[3], double noiseParameters_image_noise, double
  c_noiseParameters_inv_depth_ini, int c_VIOParameters_num_points_per_, int
  VIOParameters_num_anchors, int c_VIOParameters_max_ekf_iterati, boolean_T
  VIOParameters_fixed_feature, boolean_T c_VIOParameters_delayed_initial,
  boolean_T VIOParameters_mono, boolean_T VIOParameters_RANSAC);
static double q_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static void quatPlusThetaJ(const double dtheta[3], double dq[4]);
static void quatmultJ(const double q[4], const double p[4], double qp[4]);
static double r_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static void rdivide(const double x[3], double y, double z[3]);
static void ros_error();
static void ros_info(int varargin_1, int varargin_2, int varargin_3);
static void ros_warn(int varargin_1, double varargin_2);
static double rt_powd_snf(double u0, double u1);
static double rt_roundd_snf(double u);
static double s_fprintf(int varargin_1);
static void sort(double x[8], int idx[8]);
static void sortIdx(emxArray_real_T *x, emxArray_int32_T *idx);
static void undistortPoint(const double pt_d_data[], const int pt_d_size[1],
  double cameraparams_ATAN, const double cameraparams_FocalLength[2], const
  double cameraparams_PrincipalPoint[2], const double
  cameraparams_RadialDistortion[3], double cameraparams_DistortionModel, double
  pt_u_data[], int pt_u_size[1]);
static void undistortPointAtan(const double pt_d_data[], const int pt_d_size[1],
  const double cameraparams_FocalLength[2], const double
  cameraparams_PrincipalPoint[2], const double cameraparams_RadialDistortion[3],
  double pt_u_data[], int pt_u_size[1]);
static void xgeqp3(emxArray_real_T *A, emxArray_real_T *tau, emxArray_int32_T
                   *jpvt);
static double xnrm2(const double x[3]);
static void xscal(int n, double a, emxArray_real_T *x, int ix0);

// Function Definitions

//
// OnePointRANSAC_EKF Perform a 1-point RANSAC outlier rejection and update
// the state
// Arguments    : g_struct_T *b_xt
//                double b_P[8281]
//                const double z_u_l[80]
//                const double cameraparams_FocalLength[2]
//                const double cameraparams_PrincipalPoint[2]
//                double noiseParameters_image_noise
//                int c_VIOParameters_max_ekf_iterati
//                boolean_T c_VIOParameters_delayed_initial
//                boolean_T VIOParameters_RANSAC
//                int updateVect[40]
// Return Type  : void
//
static void OnePointRANSAC_EKF(g_struct_T *b_xt, double b_P[8281], const double
  z_u_l[80], const double cameraparams_FocalLength[2], const double
  cameraparams_PrincipalPoint[2], double noiseParameters_image_noise, int
  c_VIOParameters_max_ekf_iterati, boolean_T c_VIOParameters_delayed_initial,
  boolean_T VIOParameters_RANSAC, int updateVect[40])
{
  int ii;
  boolean_T activeFeatures[40];
  boolean_T delayedFeatures[40];
  int anchorIdx;
  int featureIdx;
  boolean_T LI_inlier_status[40];
  emxArray_real_T *S;
  emxArray_real_T *K;
  emxArray_real_T *r;
  emxArray_real_T *H;
  emxArray_real_T *R;
  int idx;
  signed char ii_data[40];
  boolean_T exitg2;
  boolean_T guard2 = false;
  int loop_ub;
  int i53;
  signed char hyp_ind_data[40];
  double num_hyp;
  int hyp_it;
  emxArray_real_T *C;
  emxArray_real_T *y;
  emxArray_real_T *b_y;
  emxArray_real_T *b_C;
  boolean_T HI_inlierCandidates[40];
  boolean_T HI_inlierStatus[40];
  unsigned int H_idx_0;
  int b_m;
  int br;
  int i54;
  int ic;
  int ar;
  int ib;
  int ia;
  double a[2];
  double x_apo[91];
  double b_x_apo;
  double dv17[4];
  double c_xt[4];
  double att_cov[9];
  double r_wc[3];
  double anchorPos[3];
  double c_x_apo[3];
  double dv18[4];
  double gryro_bias_cov[9];
  double rho;
  long i55;
  double z_u[2];
  double b_rho[3];
  double d10;
  double b_r[2];
  double dv19[2];
  int k;
  emxArray_real_T *c_y;
  emxArray_real_T *d_y;
  emxArray_real_T *b_S;
  double dv20[4];
  static double e_y[8281];
  static double dv21[8281];
  static double dv22[8281];
  static double dv23[8281];
  boolean_T b_HI_inlierCandidates;
  boolean_T exitg1;
  boolean_T guard1 = false;
  int ii_size_idx_0;
  emxArray_real_T *b_K;
  emxArray_real_T *b_H;
  int it;
  emxArray_real_T *c_C;
  emxArray_real_T *d_C;
  emxArray_real_T *f_y;
  emxArray_real_T *g_y;
  emxArray_real_T *h_y;
  emxArray_real_T *e_C;
  emxArray_real_T *c_H;
  emxArray_real_T *d_H;
  emxArray_real_T *e_H;
  double x_apo_data[91];
  double b_x_apo_data;
  double dv24[4];
  emxArray_real_T *f_C;
  double dv25[8281];
  emxArray_real_T *i_y;
  emxArray_real_T *g_C;
  emxArray_real_T *j_y;
  emxArray_real_T *h_C;
  double dv26[8281];
  double rejected_ratio;
  char cv52[66];
  static const char cv53[66] = { '1', '-', 'p', 'o', 'i', 'n', 't', ' ', 'R',
    'A', 'N', 'S', 'A', 'C', ' ', 'r', 'e', 'j', 'e', 'c', 't', 'e', 'd', ' ',
    '%', 'd', '%', '%', ' ', 'o', 'f', ' ', 'a', 'l', 'l', ' ', 'f', 'e', 'a',
    't', 'u', 'r', 'e', 's', '!', ' ', 'R', 'e', 's', 'e', 't', 't', 'i', 'n',
    'g', ' ', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', '.', '\x00' };

  double acc_bias_cov[9];
  double origin_att_cov[9];
  double R_ci_cov[9];
  static const signed char k_y[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  // 'OnePointRANSAC_EKF:5' numStatesPerAnchor = 6 + numPointsPerAnchor;
  // 'OnePointRANSAC_EKF:7' mahalanobis_thresh = 6;
  //  HI mahalanobis gate
  // 'OnePointRANSAC_EKF:8' LI_residual_thresh = 3;
  // 'OnePointRANSAC_EKF:9' LI_min_support_thresh = 3;
  // 'OnePointRANSAC_EKF:11' residualDim = 2;
  // 'OnePointRANSAC_EKF:13' activeFeatures = false(size(updateVect));
  // 'OnePointRANSAC_EKF:14' delayedFeatures = activeFeatures;
  for (ii = 0; ii < 40; ii++) {
    activeFeatures[ii] = false;
    delayedFeatures[ii] = false;
  }

  // 'OnePointRANSAC_EKF:15' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    // 'OnePointRANSAC_EKF:16' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      // 'OnePointRANSAC_EKF:17' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 1 
      if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status == 1)
      {
        // 'OnePointRANSAC_EKF:18' activeFeatures(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = 1; 
        activeFeatures[b_xt->anchor_states[anchorIdx].feature_states[featureIdx]
          .status_idx - 1] = true;
      } else {
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
            2) {
          // 'OnePointRANSAC_EKF:19' elseif xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 
          // 'OnePointRANSAC_EKF:20' delayedFeatures(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = 1; 
          delayedFeatures[b_xt->anchor_states[anchorIdx]
            .feature_states[featureIdx].status_idx - 1] = true;
        }
      }
    }
  }

  // 'OnePointRANSAC_EKF:25' activeFeatures = activeFeatures & (updateVect==1);
  // 'OnePointRANSAC_EKF:26' delayedFeatures = delayedFeatures & (updateVect==1); 
  // 'OnePointRANSAC_EKF:28' LI_inlier_status = false(size(updateVect));
  for (ii = 0; ii < 40; ii++) {
    LI_inlier_status[ii] = false;
    activeFeatures[ii] = (activeFeatures[ii] && (updateVect[ii] == 1));
    delayedFeatures[ii] = (delayedFeatures[ii] && (updateVect[ii] == 1));
  }

  // % B 1-point hypotheses generation and evaluation
  // 'OnePointRANSAC_EKF:31' if VIOParameters.RANSAC
  emxInit_real_T1(&S, 2);
  emxInit_real_T1(&K, 2);
  emxInit_real_T(&r, 1);
  emxInit_real_T1(&H, 2);
  emxInit_real_T1(&R, 2);
  if (VIOParameters_RANSAC) {
    //  build the map according to the current estimate
    // 'OnePointRANSAC_EKF:33' xt = getScaledMap(xt);
    getScaledMap(b_xt);

    //  randomly permute the active feature indices for 1-point RANSAC
    // 'OnePointRANSAC_EKF:35' hyp_ind = find(activeFeatures);
    idx = 0;
    ii = 1;
    exitg2 = false;
    while ((!exitg2) && (ii < 41)) {
      guard2 = false;
      if (activeFeatures[ii - 1]) {
        idx++;
        ii_data[idx - 1] = (signed char)ii;
        if (idx >= 40) {
          exitg2 = true;
        } else {
          guard2 = true;
        }
      } else {
        guard2 = true;
      }

      if (guard2) {
        ii++;
      }
    }

    if (1 > idx) {
      loop_ub = 0;
    } else {
      loop_ub = idx;
    }

    for (i53 = 0; i53 < loop_ub; i53++) {
      hyp_ind_data[i53] = ii_data[i53];
    }

    //  hyp_ind = hyp_ind(randperm(length(hyp_ind)));
    // 'OnePointRANSAC_EKF:38' hyp_status = LI_inlier_status;
    // 'OnePointRANSAC_EKF:39' LI_inlier_status_i = LI_inlier_status;
    // 'OnePointRANSAC_EKF:41' num_hyp = length(hyp_ind);
    if (1 > idx) {
      num_hyp = 0.0;
    } else {
      num_hyp = idx;
    }

    // 'OnePointRANSAC_EKF:42' hyp_it = 1;
    hyp_it = 1;

    // 'OnePointRANSAC_EKF:43' while hyp_it < num_hyp && hyp_it < length(hyp_ind) 
    emxInit_real_T1(&C, 2);
    emxInit_real_T1(&y, 2);
    emxInit_real_T1(&b_y, 2);
    emxInit_real_T1(&b_C, 2);
    while ((hyp_it < num_hyp) && (hyp_it < loop_ub)) {
      // 'OnePointRANSAC_EKF:44' hyp_idx = hyp_ind(hyp_it);
      // 'OnePointRANSAC_EKF:46' LI_inlier_status_i(:) = false;
      // 'OnePointRANSAC_EKF:48' hyp_status(:) = false;
      for (ii = 0; ii < 40; ii++) {
        HI_inlierCandidates[ii] = false;
        HI_inlierStatus[ii] = false;
      }

      // 'OnePointRANSAC_EKF:49' hyp_status(hyp_idx) = true;
      HI_inlierStatus[hyp_ind_data[hyp_it - 1] - 1] = true;

      //  used to signal which feature to compute the derivatives for
      // 'OnePointRANSAC_EKF:50' [r, H, R] = getH_R_res(xt, z_u_l, hyp_status, cameraparams, noiseParameters); 
      getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att,
                 b_xt->anchor_states, z_u_l, HI_inlierStatus,
                 cameraparams_FocalLength, cameraparams_PrincipalPoint,
                 noiseParameters_image_noise, r, H, R);

      // 'OnePointRANSAC_EKF:52' S = (H*P*H'+R);
      H_idx_0 = (unsigned int)H->size[0];
      i53 = y->size[0] * y->size[1];
      y->size[0] = (int)H_idx_0;
      y->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)y, i53, (int)sizeof(double));
      b_m = H->size[0];
      i53 = y->size[0] * y->size[1];
      y->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)y, i53, (int)sizeof(double));
      for (i53 = 0; i53 < 91; i53++) {
        br = y->size[0];
        for (i54 = 0; i54 < br; i54++) {
          y->data[i54 + y->size[0] * i53] = 0.0;
        }
      }

      if (H->size[0] == 0) {
      } else {
        ii = H->size[0] * 90;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i53 = idx + b_m;
          for (ic = idx; ic + 1 <= i53; ic++) {
            y->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (b_P[ib] != 0.0) {
              ia = ar;
              i53 = idx + b_m;
              for (ic = idx; ic + 1 <= i53; ic++) {
                ia++;
                y->data[ic] += b_P[ib] * H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i53 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)K, i53, (int)sizeof(double));
      br = H->size[0];
      for (i53 = 0; i53 < br; i53++) {
        for (i54 = 0; i54 < 91; i54++) {
          K->data[i54 + K->size[0] * i53] = H->data[i53 + H->size[0] * i54];
        }
      }

      a[0] = (unsigned int)y->size[0];
      a[1] = (unsigned int)K->size[1];
      i53 = C->size[0] * C->size[1];
      C->size[0] = (int)a[0];
      C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)C, i53, (int)sizeof(double));
      b_m = y->size[0];
      i53 = C->size[0] * C->size[1];
      emxEnsureCapacity((emxArray__common *)C, i53, (int)sizeof(double));
      br = C->size[1];
      for (i53 = 0; i53 < br; i53++) {
        ii = C->size[0];
        for (i54 = 0; i54 < ii; i54++) {
          C->data[i54 + C->size[0] * i53] = 0.0;
        }
      }

      if ((y->size[0] == 0) || (K->size[1] == 0)) {
      } else {
        ii = y->size[0] * (K->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i53 = idx + b_m;
          for (ic = idx; ic + 1 <= i53; ic++) {
            C->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              i53 = idx + b_m;
              for (ic = idx; ic + 1 <= i53; ic++) {
                ia++;
                C->data[ic] += K->data[ib] * y->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      // 'OnePointRANSAC_EKF:53' K = (P*H')/S;
      i53 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)K, i53, (int)sizeof(double));
      br = H->size[0];
      for (i53 = 0; i53 < br; i53++) {
        for (i54 = 0; i54 < 91; i54++) {
          K->data[i54 + K->size[0] * i53] = H->data[i53 + H->size[0] * i54];
        }
      }

      H_idx_0 = (unsigned int)K->size[1];
      i53 = b_y->size[0] * b_y->size[1];
      b_y->size[0] = 91;
      b_y->size[1] = (int)H_idx_0;
      b_y->size[0] = 91;
      emxEnsureCapacity((emxArray__common *)b_y, i53, (int)sizeof(double));
      br = b_y->size[1];
      for (i53 = 0; i53 < br; i53++) {
        for (i54 = 0; i54 < 91; i54++) {
          b_y->data[i54 + b_y->size[0] * i53] = 0.0;
        }
      }

      if (K->size[1] == 0) {
      } else {
        ii = 91 * (K->size[1] - 1);
        for (idx = 0; idx <= ii; idx += 91) {
          for (ic = idx; ic + 1 <= idx + 91; ic++) {
            b_y->data[ic] = 0.0;
          }
        }

        br = 0;
        for (idx = 0; idx <= ii; idx += 91) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              for (ic = idx; ic + 1 <= idx + 91; ic++) {
                ia++;
                b_y->data[ic] += K->data[ib] * b_P[ia - 1];
              }
            }

            ar += 91;
          }

          br += 91;
        }
      }

      i53 = b_C->size[0] * b_C->size[1];
      b_C->size[0] = C->size[0];
      b_C->size[1] = C->size[1];
      emxEnsureCapacity((emxArray__common *)b_C, i53, (int)sizeof(double));
      br = C->size[0] * C->size[1];
      for (i53 = 0; i53 < br; i53++) {
        b_C->data[i53] = C->data[i53] + R->data[i53];
      }

      mrdivide(b_y, b_C, K);

      // 'OnePointRANSAC_EKF:55' x_apo = K*r;
      if ((K->size[1] == 1) || (r->size[0] == 1)) {
        for (i53 = 0; i53 < 91; i53++) {
          x_apo[i53] = 0.0;
          br = K->size[1];
          for (i54 = 0; i54 < br; i54++) {
            b_x_apo = x_apo[i53] + K->data[i53 + K->size[0] * i54] * r->data[i54];
            x_apo[i53] = b_x_apo;
          }
        }
      } else {
        memset(&x_apo[0], 0, 91U * sizeof(double));
        ar = 0;
        for (ib = 0; ib + 1 <= K->size[1]; ib++) {
          if (r->data[ib] != 0.0) {
            ia = ar;
            for (ic = 0; ic < 91; ic++) {
              ia++;
              b_x_apo = x_apo[ic] + r->data[ib] * K->data[ia - 1];
              x_apo[ic] = b_x_apo;
            }
          }

          ar += 91;
        }
      }

      // 'OnePointRANSAC_EKF:57' R_cw = RotFromQuatJ(quatmultJ(quatPlusThetaJ(x_apo(4:6)), xt.robot_state.att)); 
      quatPlusThetaJ(*(double (*)[3])&x_apo[3], dv17);
      quatmultJ(dv17, b_xt->robot_state.att, c_xt);

      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
      // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
      // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
      att_cov[0] = ((c_xt[0] * c_xt[0] - c_xt[1] * c_xt[1]) - c_xt[2] * c_xt[2])
        + c_xt[3] * c_xt[3];
      att_cov[3] = 2.0 * (c_xt[0] * c_xt[1] + c_xt[2] * c_xt[3]);
      att_cov[6] = 2.0 * (c_xt[0] * c_xt[2] - c_xt[1] * c_xt[3]);
      att_cov[1] = 2.0 * (c_xt[0] * c_xt[1] - c_xt[2] * c_xt[3]);
      att_cov[4] = ((-(c_xt[0] * c_xt[0]) + c_xt[1] * c_xt[1]) - c_xt[2] * c_xt
                    [2]) + c_xt[3] * c_xt[3];
      att_cov[7] = 2.0 * (c_xt[1] * c_xt[2] + c_xt[0] * c_xt[3]);
      att_cov[2] = 2.0 * (c_xt[0] * c_xt[2] + c_xt[1] * c_xt[3]);
      att_cov[5] = 2.0 * (c_xt[1] * c_xt[2] - c_xt[0] * c_xt[3]);
      att_cov[8] = ((-(c_xt[0] * c_xt[0]) - c_xt[1] * c_xt[1]) + c_xt[2] * c_xt
                    [2]) + c_xt[3] * c_xt[3];

      // 'OnePointRANSAC_EKF:58' r_wc = xt.robot_state.pos + x_apo(1:3);
      for (ii = 0; ii < 3; ii++) {
        r_wc[ii] = b_xt->robot_state.pos[ii] + x_apo[ii];
      }

      // 'OnePointRANSAC_EKF:60' for anchorIdx = 1:numAnchors
      for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
        // 'OnePointRANSAC_EKF:61' anchorPos = xt.anchor_states(anchorIdx).pos + x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:3)); 
        i53 = anchorIdx * 14;

        // 'OnePointRANSAC_EKF:62' anchorRot = RotFromQuatJ(quatmultJ(quatPlusThetaJ(x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(4:6))), xt.anchor_states(anchorIdx).att)); 
        i54 = anchorIdx * 14;
        for (idx = 0; idx < 3; idx++) {
          anchorPos[idx] = b_xt->anchor_states[anchorIdx].pos[idx] + x_apo[(idx
            + i53) + 21];
          c_x_apo[idx] = x_apo[(idx + i54) + 24];
        }

        quatPlusThetaJ(c_x_apo, dv18);
        quatmultJ(dv18, b_xt->anchor_states[anchorIdx].att, c_xt);

        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
        // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
        // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
        gryro_bias_cov[0] = ((c_xt[0] * c_xt[0] - c_xt[1] * c_xt[1]) - c_xt[2] *
                             c_xt[2]) + c_xt[3] * c_xt[3];
        gryro_bias_cov[3] = 2.0 * (c_xt[0] * c_xt[1] + c_xt[2] * c_xt[3]);
        gryro_bias_cov[6] = 2.0 * (c_xt[0] * c_xt[2] - c_xt[1] * c_xt[3]);
        gryro_bias_cov[1] = 2.0 * (c_xt[0] * c_xt[1] - c_xt[2] * c_xt[3]);
        gryro_bias_cov[4] = ((-(c_xt[0] * c_xt[0]) + c_xt[1] * c_xt[1]) - c_xt[2]
                             * c_xt[2]) + c_xt[3] * c_xt[3];
        gryro_bias_cov[7] = 2.0 * (c_xt[1] * c_xt[2] + c_xt[0] * c_xt[3]);
        gryro_bias_cov[2] = 2.0 * (c_xt[0] * c_xt[2] + c_xt[1] * c_xt[3]);
        gryro_bias_cov[5] = 2.0 * (c_xt[1] * c_xt[2] - c_xt[0] * c_xt[3]);
        gryro_bias_cov[8] = ((-(c_xt[0] * c_xt[0]) - c_xt[1] * c_xt[1]) + c_xt[2]
                             * c_xt[2]) + c_xt[3] * c_xt[3];

        // 'OnePointRANSAC_EKF:64' for featureIdx = 1:numPointsPerAnchor
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          // 'OnePointRANSAC_EKF:65' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 1 
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              1) {
            //  only update active features
            // 'OnePointRANSAC_EKF:66' rho = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth + x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
            rho = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              inverse_depth + x_apo[(anchorIdx * 14 + featureIdx) + 27];

            // 'OnePointRANSAC_EKF:67' m = xt.anchor_states(anchorIdx).feature_states(featureIdx).m; 
            // 'OnePointRANSAC_EKF:68' fp_scaled = R_cw*(rho*anchorPos + anchorRot'*m - r_wc*rho); 
            // 'OnePointRANSAC_EKF:69' h_u = predictMeasurementMono(fp_scaled, cameraparams); 
            // 'OnePointRANSAC_EKF:70' z_u = z_u_l((xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx-1)*2 + int32(1:2)); 
            i55 = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              status_idx - 1L;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i53 = (int)i55;
            if (i53 > 1073741823) {
              ii = MAX_int32_T;
            } else if (i53 <= -1073741824) {
              ii = MIN_int32_T;
            } else {
              ii = i53 << 1;
            }

            for (i53 = 0; i53 < 2; i53++) {
              i55 = (long)ii + (1 + i53);
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              z_u[i53] = z_u_l[(int)i55 - 1];
            }

            // 'OnePointRANSAC_EKF:71' innov = norm(h_u - z_u);
            // 'OnePointRANSAC_EKF:72' if innov < LI_residual_thresh
            for (i53 = 0; i53 < 3; i53++) {
              d10 = 0.0;
              for (i54 = 0; i54 < 3; i54++) {
                d10 += gryro_bias_cov[i54 + 3 * i53] * b_xt->
                  anchor_states[anchorIdx].feature_states[featureIdx].m[i54];
              }

              b_rho[i53] = (rho * anchorPos[i53] + d10) - r_wc[i53] * rho;
            }

            for (i53 = 0; i53 < 3; i53++) {
              c_x_apo[i53] = 0.0;
              for (i54 = 0; i54 < 3; i54++) {
                c_x_apo[i53] += att_cov[i53 + 3 * i54] * b_rho[i54];
              }
            }

            predictMeasurementMono(c_x_apo, cameraparams_FocalLength,
              cameraparams_PrincipalPoint, b_r);
            for (ii = 0; ii < 2; ii++) {
              dv19[ii] = b_r[ii] - z_u[ii];
            }

            if (c_norm(dv19) < 3.0) {
              // 'OnePointRANSAC_EKF:73' LI_inlier_status_i(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = true; 
              HI_inlierCandidates[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = true;
            }
          }
        }
      }

      // 'OnePointRANSAC_EKF:79' if nnz(LI_inlier_status_i) > nnz(LI_inlier_status) 
      ii = 0;
      idx = 0;
      for (k = 0; k < 40; k++) {
        if (HI_inlierCandidates[k]) {
          ii++;
        }

        if (LI_inlier_status[k]) {
          idx++;
        }
      }

      if (ii > idx) {
        // 'OnePointRANSAC_EKF:80' LI_inlier_status = LI_inlier_status_i;
        // 'OnePointRANSAC_EKF:81' epsilon = 1 - nnz(LI_inlier_status_i)/nnz(activeFeatures); 
        ii = 0;
        idx = 0;
        for (k = 0; k < 40; k++) {
          LI_inlier_status[k] = HI_inlierCandidates[k];
          if (HI_inlierCandidates[k]) {
            ii++;
          }

          if (activeFeatures[k]) {
            idx++;
          }
        }

        // 'OnePointRANSAC_EKF:82' assert(epsilon <= 1)
        // 'OnePointRANSAC_EKF:83' num_hyp = log(1-0.99)/log(epsilon);
        num_hyp = -4.60517018598809 / log(1.0 - (double)ii / (double)idx);
      }

      // 'OnePointRANSAC_EKF:86' hyp_it = hyp_it + 1;
      hyp_it++;
    }

    emxFree_real_T(&b_C);
    emxFree_real_T(&b_y);
    emxFree_real_T(&y);
    emxFree_real_T(&C);

    //  ros_info('Found %i LI inliers in %i active features', nnz(LI_inlier_status), nnz(activeFeatures)) 
    // 'OnePointRANSAC_EKF:89' if nnz(LI_inlier_status) > LI_min_support_thresh
    ii = 0;
    for (k = 0; k < 40; k++) {
      if (LI_inlier_status[k]) {
        ii++;
      }
    }

    if (ii > 3) {
      emxInit_real_T1(&c_y, 2);

      // 'OnePointRANSAC_EKF:90' [r, H, R] = getH_R_res(xt, z_u_l, LI_inlier_status, cameraparams, noiseParameters); 
      getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att,
                 b_xt->anchor_states, z_u_l, LI_inlier_status,
                 cameraparams_FocalLength, cameraparams_PrincipalPoint,
                 noiseParameters_image_noise, r, H, R);

      // 'OnePointRANSAC_EKF:91' S = (H*P*H'+R);
      H_idx_0 = (unsigned int)H->size[0];
      i53 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = (int)H_idx_0;
      c_y->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)c_y, i53, (int)sizeof(double));
      b_m = H->size[0];
      i53 = c_y->size[0] * c_y->size[1];
      c_y->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)c_y, i53, (int)sizeof(double));
      for (i53 = 0; i53 < 91; i53++) {
        loop_ub = c_y->size[0];
        for (i54 = 0; i54 < loop_ub; i54++) {
          c_y->data[i54 + c_y->size[0] * i53] = 0.0;
        }
      }

      if (H->size[0] == 0) {
      } else {
        ii = H->size[0] * 90;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i53 = idx + b_m;
          for (ic = idx; ic + 1 <= i53; ic++) {
            c_y->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (b_P[ib] != 0.0) {
              ia = ar;
              i53 = idx + b_m;
              for (ic = idx; ic + 1 <= i53; ic++) {
                ia++;
                c_y->data[ic] += b_P[ib] * H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i53 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)K, i53, (int)sizeof(double));
      loop_ub = H->size[0];
      for (i53 = 0; i53 < loop_ub; i53++) {
        for (i54 = 0; i54 < 91; i54++) {
          K->data[i54 + K->size[0] * i53] = H->data[i53 + H->size[0] * i54];
        }
      }

      a[0] = (unsigned int)c_y->size[0];
      a[1] = (unsigned int)K->size[1];
      i53 = S->size[0] * S->size[1];
      S->size[0] = (int)a[0];
      S->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)S, i53, (int)sizeof(double));
      b_m = c_y->size[0];
      i53 = S->size[0] * S->size[1];
      emxEnsureCapacity((emxArray__common *)S, i53, (int)sizeof(double));
      loop_ub = S->size[1];
      for (i53 = 0; i53 < loop_ub; i53++) {
        br = S->size[0];
        for (i54 = 0; i54 < br; i54++) {
          S->data[i54 + S->size[0] * i53] = 0.0;
        }
      }

      if ((c_y->size[0] == 0) || (K->size[1] == 0)) {
      } else {
        ii = c_y->size[0] * (K->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i53 = idx + b_m;
          for (ic = idx; ic + 1 <= i53; ic++) {
            S->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              i53 = idx + b_m;
              for (ic = idx; ic + 1 <= i53; ic++) {
                ia++;
                S->data[ic] += K->data[ib] * c_y->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      emxFree_real_T(&c_y);

      // 'OnePointRANSAC_EKF:92' K = (P*H')/S;
      i53 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)K, i53, (int)sizeof(double));
      loop_ub = H->size[0];
      for (i53 = 0; i53 < loop_ub; i53++) {
        for (i54 = 0; i54 < 91; i54++) {
          K->data[i54 + K->size[0] * i53] = H->data[i53 + H->size[0] * i54];
        }
      }

      emxInit_real_T1(&d_y, 2);
      H_idx_0 = (unsigned int)K->size[1];
      i53 = d_y->size[0] * d_y->size[1];
      d_y->size[0] = 91;
      d_y->size[1] = (int)H_idx_0;
      d_y->size[0] = 91;
      emxEnsureCapacity((emxArray__common *)d_y, i53, (int)sizeof(double));
      loop_ub = d_y->size[1];
      for (i53 = 0; i53 < loop_ub; i53++) {
        for (i54 = 0; i54 < 91; i54++) {
          d_y->data[i54 + d_y->size[0] * i53] = 0.0;
        }
      }

      if (K->size[1] == 0) {
      } else {
        ii = 91 * (K->size[1] - 1);
        for (idx = 0; idx <= ii; idx += 91) {
          for (ic = idx; ic + 1 <= idx + 91; ic++) {
            d_y->data[ic] = 0.0;
          }
        }

        br = 0;
        for (idx = 0; idx <= ii; idx += 91) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              for (ic = idx; ic + 1 <= idx + 91; ic++) {
                ia++;
                d_y->data[ic] += K->data[ib] * b_P[ia - 1];
              }
            }

            ar += 91;
          }

          br += 91;
        }
      }

      emxInit_real_T1(&b_S, 2);
      i53 = b_S->size[0] * b_S->size[1];
      b_S->size[0] = S->size[0];
      b_S->size[1] = S->size[1];
      emxEnsureCapacity((emxArray__common *)b_S, i53, (int)sizeof(double));
      loop_ub = S->size[0] * S->size[1];
      for (i53 = 0; i53 < loop_ub; i53++) {
        b_S->data[i53] = S->data[i53] + R->data[i53];
      }

      mrdivide(d_y, b_S, K);

      // 'OnePointRANSAC_EKF:94' x_apo = K*r;
      emxFree_real_T(&b_S);
      emxFree_real_T(&d_y);
      if ((K->size[1] == 1) || (r->size[0] == 1)) {
        for (i53 = 0; i53 < 91; i53++) {
          x_apo[i53] = 0.0;
          loop_ub = K->size[1];
          for (i54 = 0; i54 < loop_ub; i54++) {
            b_x_apo = x_apo[i53] + K->data[i53 + K->size[0] * i54] * r->data[i54];
            x_apo[i53] = b_x_apo;
          }
        }
      } else {
        memset(&x_apo[0], 0, 91U * sizeof(double));
        ar = 0;
        for (ib = 0; ib + 1 <= K->size[1]; ib++) {
          if (r->data[ib] != 0.0) {
            ia = ar;
            for (ic = 0; ic < 91; ic++) {
              ia++;
              b_x_apo = x_apo[ic] + r->data[ib] * K->data[ia - 1];
              x_apo[ic] = b_x_apo;
            }
          }

          ar += 91;
        }
      }

      // 'OnePointRANSAC_EKF:96' xt.robot_state.pos           = xt.robot_state.pos       + x_apo(1:3); 
      for (i53 = 0; i53 < 3; i53++) {
        b_xt->robot_state.pos[i53] += x_apo[i53];
      }

      // 'OnePointRANSAC_EKF:97' xt.robot_state.att           = quatmultJ(quatPlusThetaJ(x_apo(4:6)), xt.robot_state.att); 
      quatPlusThetaJ(*(double (*)[3])&x_apo[3], dv17);
      for (ii = 0; ii < 4; ii++) {
        c_xt[ii] = b_xt->robot_state.att[ii];
      }

      quatmultJ(dv17, c_xt, b_xt->robot_state.att);

      // 'OnePointRANSAC_EKF:98' xt.robot_state.vel           = xt.robot_state.vel       + x_apo(7:9); 
      // 'OnePointRANSAC_EKF:99' xt.robot_state.IMU.gyro_bias = xt.robot_state.IMU.gyro_bias + x_apo(10:12); 
      // 'OnePointRANSAC_EKF:100' xt.robot_state.IMU.acc_bias  = xt.robot_state.IMU.acc_bias + x_apo(13:15); 
      for (i53 = 0; i53 < 3; i53++) {
        b_xt->robot_state.vel[i53] += x_apo[6 + i53];
        b_xt->robot_state.IMU.gyro_bias[i53] += x_apo[9 + i53];
        b_xt->robot_state.IMU.acc_bias[i53] += x_apo[12 + i53];
      }

      // 'OnePointRANSAC_EKF:101' xt.origin.att                = quatmultJ(quatPlusThetaJ(x_apo(16:18)), xt.origin.att); 
      quatPlusThetaJ(*(double (*)[3])&x_apo[15], dv17);
      for (ii = 0; ii < 4; ii++) {
        c_xt[ii] = b_xt->origin.att[ii];
      }

      quatmultJ(dv17, c_xt, b_xt->origin.att);

      // 'OnePointRANSAC_EKF:102' xt.robot_state.IMU.att       = quatmultJ(quatPlusThetaJ(x_apo(19:21)), xt.robot_state.IMU.att); 
      quatPlusThetaJ(*(double (*)[3])&x_apo[18], dv17);
      for (ii = 0; ii < 4; ii++) {
        c_xt[ii] = b_xt->robot_state.IMU.att[ii];
      }

      quatmultJ(dv17, c_xt, b_xt->robot_state.IMU.att);

      // 'OnePointRANSAC_EKF:103' xt.robot_state.IMU.pos       = xt.robot_state.IMU.pos + x_apo(22:24); 
      for (i53 = 0; i53 < 3; i53++) {
        b_xt->robot_state.IMU.pos[i53] += x_apo[21 + i53];
      }

      // 'OnePointRANSAC_EKF:105' for anchorIdx = 1:numAnchors
      for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
        // 'OnePointRANSAC_EKF:106' xt.anchor_states(anchorIdx).pos = xt.anchor_states(anchorIdx).pos + x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:3)); 
        i53 = anchorIdx * 14;

        // 'OnePointRANSAC_EKF:107' xt.anchor_states(anchorIdx).att = quatmultJ(quatPlusThetaJ(x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(4:6))), xt.anchor_states(anchorIdx).att); 
        i54 = anchorIdx * 14;
        for (idx = 0; idx < 3; idx++) {
          b_xt->anchor_states[anchorIdx].pos[idx] += x_apo[(idx + i53) + 21];
          c_x_apo[idx] = x_apo[(idx + i54) + 24];
        }

        for (ii = 0; ii < 4; ii++) {
          c_xt[ii] = b_xt->anchor_states[anchorIdx].att[ii];
        }

        quatPlusThetaJ(c_x_apo, dv20);
        quatmultJ(dv20, c_xt, b_xt->anchor_states[anchorIdx].att);

        // 'OnePointRANSAC_EKF:109' for featureIdx = 1:numPointsPerAnchor
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          // 'OnePointRANSAC_EKF:110' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 1 
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              1) {
            //  only update active features
            // 'OnePointRANSAC_EKF:111' xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth + x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              inverse_depth += x_apo[(anchorIdx * 14 + featureIdx) + 27];

            // 'OnePointRANSAC_EKF:112' if xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth > 10 
            if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth > 10.0) {
              // 'OnePointRANSAC_EKF:113' ros_warn('feature %i is very close. Depth: %f', int32(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx), 1/xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth) 
              ros_warn(b_xt->anchor_states[anchorIdx].feature_states[featureIdx]
                       .status_idx, 1.0 / b_xt->anchor_states[anchorIdx].
                       feature_states[featureIdx].inverse_depth);
            }

            // 'OnePointRANSAC_EKF:115' if xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth < 0 
            if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0) {
              // 'OnePointRANSAC_EKF:116' ros_warn('Feature %i (%i on %i) is behind its anchor, rejecting', int32(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx), int32(featureIdx), int32(anchorIdx)) 
              b_ros_warn(b_xt->anchor_states[anchorIdx]
                         .feature_states[featureIdx].status_idx, featureIdx + 1,
                         anchorIdx + 1);

              // 'OnePointRANSAC_EKF:117' updateVect(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
              updateVect[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = 0;

              // 'OnePointRANSAC_EKF:118' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                0;

              // 'OnePointRANSAC_EKF:119' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                status_idx = 0;
            }
          }
        }
      }

      // 'OnePointRANSAC_EKF:126' P = (eye(numStates + numAnchors*numStatesPerAnchor)-K*H)*P; 
      if ((K->size[1] == 1) || (H->size[0] == 1)) {
        for (i53 = 0; i53 < 91; i53++) {
          for (i54 = 0; i54 < 91; i54++) {
            e_y[i53 + 91 * i54] = 0.0;
            loop_ub = K->size[1];
            for (idx = 0; idx < loop_ub; idx++) {
              e_y[i53 + 91 * i54] += K->data[i53 + K->size[0] * idx] * H->
                data[idx + H->size[0] * i54];
            }
          }
        }
      } else {
        k = K->size[1];
        memset(&e_y[0], 0, 8281U * sizeof(double));
        for (idx = 0; idx <= 8191; idx += 91) {
          for (ic = idx; ic + 1 <= idx + 91; ic++) {
            e_y[ic] = 0.0;
          }
        }

        br = 0;
        for (idx = 0; idx <= 8191; idx += 91) {
          ar = 0;
          i53 = br + k;
          for (ib = br; ib + 1 <= i53; ib++) {
            if (H->data[ib] != 0.0) {
              ia = ar;
              for (ic = idx; ic + 1 <= idx + 91; ic++) {
                ia++;
                e_y[ic] += H->data[ib] * K->data[ia - 1];
              }
            }

            ar += 91;
          }

          br += k;
        }
      }

      c_eye(dv21);
      for (i53 = 0; i53 < 91; i53++) {
        for (i54 = 0; i54 < 91; i54++) {
          dv22[i54 + 91 * i53] = dv21[i54 + 91 * i53] - e_y[i54 + 91 * i53];
        }
      }

      for (i53 = 0; i53 < 91; i53++) {
        for (i54 = 0; i54 < 91; i54++) {
          dv23[i53 + 91 * i54] = 0.0;
          for (idx = 0; idx < 91; idx++) {
            dv23[i53 + 91 * i54] += dv22[i53 + 91 * idx] * b_P[idx + 91 * i54];
          }
        }
      }

      for (i53 = 0; i53 < 91; i53++) {
        memcpy(&b_P[i53 * 91], &dv23[i53 * 91], 91U * sizeof(double));
      }
    } else {
      // 'OnePointRANSAC_EKF:127' else
      // 'OnePointRANSAC_EKF:128' LI_inlier_status(:) = false;
      for (ii = 0; ii < 40; ii++) {
        LI_inlier_status[ii] = false;
      }

      // 'OnePointRANSAC_EKF:129' ros_warn('1-Point RANSAC didnt find enough LI inliers') 
      c_ros_warn();
    }
  }

  // % D Partial EKF update using high-innovation inliers
  // 'OnePointRANSAC_EKF:133' HI_inlierCandidates = activeFeatures & ~LI_inlier_status; 
  //  high innovation inliers (ordered like updateVect)
  // 'OnePointRANSAC_EKF:134' HI_inlierStatus = HI_inlierCandidates;
  for (ii = 0; ii < 40; ii++) {
    b_HI_inlierCandidates = (activeFeatures[ii] && (!LI_inlier_status[ii]));
    HI_inlierStatus[ii] = b_HI_inlierCandidates;
    HI_inlierCandidates[ii] = b_HI_inlierCandidates;
  }

  // 'OnePointRANSAC_EKF:135' HI_ind = find(HI_inlierStatus);
  idx = 0;
  ii = 1;
  exitg1 = false;
  while ((!exitg1) && (ii < 41)) {
    guard1 = false;
    if (HI_inlierCandidates[ii - 1]) {
      idx++;
      ii_data[idx - 1] = (signed char)ii;
      if (idx >= 40) {
        exitg1 = true;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      ii++;
    }
  }

  if (1 > idx) {
    ii_size_idx_0 = 0;
  } else {
    ii_size_idx_0 = idx;
  }

  if (1 > idx) {
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }

  for (i53 = 0; i53 < loop_ub; i53++) {
    hyp_ind_data[i53] = ii_data[i53];
  }

  // 'OnePointRANSAC_EKF:137' if any(HI_inlierCandidates)
  if (any(HI_inlierCandidates)) {
    emxInit_real_T1(&b_K, 2);
    emxInit_real_T1(&b_H, 2);

    // 'OnePointRANSAC_EKF:138' K = 0;
    i53 = b_K->size[0] * b_K->size[1];
    b_K->size[0] = 1;
    b_K->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)b_K, i53, (int)sizeof(double));
    b_K->data[0] = 0.0;

    //  for coder
    // 'OnePointRANSAC_EKF:139' H = 0;
    i53 = b_H->size[0] * b_H->size[1];
    b_H->size[0] = 1;
    b_H->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)b_H, i53, (int)sizeof(double));
    b_H->data[0] = 0.0;

    //  for coder
    // 'OnePointRANSAC_EKF:140' for it = 1:VIOParameters.max_ekf_iterations
    it = 1;
    emxInit_real_T1(&c_C, 2);
    emxInit_real_T1(&d_C, 2);
    emxInit_real_T1(&f_y, 2);
    emxInit_real_T1(&g_y, 2);
    emxInit_real_T1(&h_y, 2);
    emxInit_real_T1(&e_C, 2);
    emxInit_real_T1(&c_H, 2);
    emxInit_real_T1(&d_H, 2);
    emxInit_real_T1(&e_H, 2);
    while (it <= c_VIOParameters_max_ekf_iterati) {
      // 'OnePointRANSAC_EKF:141' xt = getScaledMap(xt);
      getScaledMap(b_xt);

      //  build the map according to the current estimate
      // 'OnePointRANSAC_EKF:143' [r, H, R] = getH_R_res(xt, z_u_l, HI_inlierStatus, cameraparams, noiseParameters); 
      getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att,
                 b_xt->anchor_states, z_u_l, HI_inlierStatus,
                 cameraparams_FocalLength, cameraparams_PrincipalPoint,
                 noiseParameters_image_noise, r, H, R);
      i53 = b_H->size[0] * b_H->size[1];
      b_H->size[0] = H->size[0];
      b_H->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)b_H, i53, (int)sizeof(double));
      loop_ub = H->size[0] * H->size[1];
      for (i53 = 0; i53 < loop_ub; i53++) {
        b_H->data[i53] = H->data[i53];
      }

      //  the residual is ordered by anchors/features, not like updateVect
      // 'OnePointRANSAC_EKF:145' S = (H*P*H'+R);
      H_idx_0 = (unsigned int)H->size[0];
      i53 = f_y->size[0] * f_y->size[1];
      f_y->size[0] = (int)H_idx_0;
      f_y->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)f_y, i53, (int)sizeof(double));
      b_m = H->size[0];
      i53 = f_y->size[0] * f_y->size[1];
      f_y->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)f_y, i53, (int)sizeof(double));
      for (i53 = 0; i53 < 91; i53++) {
        loop_ub = f_y->size[0];
        for (i54 = 0; i54 < loop_ub; i54++) {
          f_y->data[i54 + f_y->size[0] * i53] = 0.0;
        }
      }

      if (H->size[0] == 0) {
      } else {
        ii = H->size[0] * 90;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i53 = idx + b_m;
          for (ic = idx; ic + 1 <= i53; ic++) {
            f_y->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (b_P[ib] != 0.0) {
              ia = ar;
              i53 = idx + b_m;
              for (ic = idx; ic + 1 <= i53; ic++) {
                ia++;
                f_y->data[ic] += b_P[ib] * H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i53 = c_H->size[0] * c_H->size[1];
      c_H->size[0] = 91;
      c_H->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)c_H, i53, (int)sizeof(double));
      loop_ub = H->size[0];
      for (i53 = 0; i53 < loop_ub; i53++) {
        for (i54 = 0; i54 < 91; i54++) {
          c_H->data[i54 + c_H->size[0] * i53] = H->data[i53 + H->size[0] * i54];
        }
      }

      ii = H->size[0];
      i53 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = ii;
      emxEnsureCapacity((emxArray__common *)K, i53, (int)sizeof(double));
      for (i53 = 0; i53 < ii; i53++) {
        for (i54 = 0; i54 < 91; i54++) {
          K->data[i54 + K->size[0] * i53] = c_H->data[i54 + 91 * i53];
        }
      }

      a[0] = (unsigned int)f_y->size[0];
      a[1] = (unsigned int)K->size[1];
      i53 = c_C->size[0] * c_C->size[1];
      c_C->size[0] = (int)a[0];
      c_C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)c_C, i53, (int)sizeof(double));
      b_m = f_y->size[0];
      i53 = c_C->size[0] * c_C->size[1];
      emxEnsureCapacity((emxArray__common *)c_C, i53, (int)sizeof(double));
      loop_ub = c_C->size[1];
      for (i53 = 0; i53 < loop_ub; i53++) {
        br = c_C->size[0];
        for (i54 = 0; i54 < br; i54++) {
          c_C->data[i54 + c_C->size[0] * i53] = 0.0;
        }
      }

      if ((f_y->size[0] == 0) || (K->size[1] == 0)) {
      } else {
        ii = f_y->size[0] * (K->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i53 = idx + b_m;
          for (ic = idx; ic + 1 <= i53; ic++) {
            c_C->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              i53 = idx + b_m;
              for (ic = idx; ic + 1 <= i53; ic++) {
                ia++;
                c_C->data[ic] += K->data[ib] * f_y->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i53 = S->size[0] * S->size[1];
      S->size[0] = c_C->size[0];
      S->size[1] = c_C->size[1];
      emxEnsureCapacity((emxArray__common *)S, i53, (int)sizeof(double));
      loop_ub = c_C->size[0] * c_C->size[1];
      for (i53 = 0; i53 < loop_ub; i53++) {
        S->data[i53] = c_C->data[i53] + R->data[i53];
      }

      // 'OnePointRANSAC_EKF:147' for k = 1:length(HI_ind)
      for (k = 0; k < ii_size_idx_0; k++) {
        // 'OnePointRANSAC_EKF:148' innov = r((k-1)*residualDim + (1:2))' / S((k-1)*residualDim + (1:2), (k-1)*residualDim + (1:2)) * r((k-1)*residualDim + (1:2)); 
        ii = k << 1;
        for (i53 = 0; i53 < 2; i53++) {
          b_r[i53] = r->data[i53 + ii];
        }

        ii = k << 1;
        idx = k << 1;
        for (i53 = 0; i53 < 2; i53++) {
          for (i54 = 0; i54 < 2; i54++) {
            c_xt[i54 + (i53 << 1)] = S->data[(i54 + ii) + S->size[0] * (i53 +
              idx)];
          }
        }

        b_mrdivide(b_r, c_xt, a);
        ii = k << 1;
        for (i53 = 0; i53 < 2; i53++) {
          z_u[i53] = r->data[i53 + ii];
        }

        // 'OnePointRANSAC_EKF:149' if innov > mahalanobis_thresh
        d10 = 0.0;
        for (i53 = 0; i53 < 2; i53++) {
          d10 += a[i53] * z_u[i53];
        }

        if (d10 > 6.0) {
          // 'OnePointRANSAC_EKF:150' r((k-1)*residualDim + (1:2)) = 0;
          ii = k << 1;
          for (i53 = 0; i53 < 2; i53++) {
            r->data[i53 + ii] = 0.0;
          }

          // 'OnePointRANSAC_EKF:151' H((k-1)*residualDim + (1:2), :) = 0;
          ii = k << 1;
          for (i53 = 0; i53 < 91; i53++) {
            for (i54 = 0; i54 < 2; i54++) {
              b_H->data[(i54 + ii) + b_H->size[0] * i53] = 0.0;
            }
          }

          // 'OnePointRANSAC_EKF:152' if it == VIOParameters.max_ekf_iterations
          if (it == c_VIOParameters_max_ekf_iterati) {
            // 'OnePointRANSAC_EKF:153' HI_inlierStatus(HI_ind(k)) = false;
            HI_inlierStatus[hyp_ind_data[k] - 1] = false;

            //  only reject the feature if its still bad in last iteration, otherwise just dont use for this update 
          }

          //          ros_info('rejecting %i', HI_ind(k))
          // 'OnePointRANSAC_EKF:156' if updateVect(HI_ind(k)) == 2
          if (updateVect[hyp_ind_data[k] - 1] == 2) {
            // 'OnePointRANSAC_EKF:157' ros_error('inconsistency')
            b_ros_error();
          }
        }
      }

      // 'OnePointRANSAC_EKF:162' S = (H*P*H'+R);
      H_idx_0 = (unsigned int)b_H->size[0];
      i53 = g_y->size[0] * g_y->size[1];
      g_y->size[0] = (int)H_idx_0;
      g_y->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)g_y, i53, (int)sizeof(double));
      b_m = b_H->size[0];
      i53 = g_y->size[0] * g_y->size[1];
      g_y->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)g_y, i53, (int)sizeof(double));
      for (i53 = 0; i53 < 91; i53++) {
        loop_ub = g_y->size[0];
        for (i54 = 0; i54 < loop_ub; i54++) {
          g_y->data[i54 + g_y->size[0] * i53] = 0.0;
        }
      }

      if (b_H->size[0] == 0) {
      } else {
        ii = b_H->size[0] * 90;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i53 = idx + b_m;
          for (ic = idx; ic + 1 <= i53; ic++) {
            g_y->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (b_P[ib] != 0.0) {
              ia = ar;
              i53 = idx + b_m;
              for (ic = idx; ic + 1 <= i53; ic++) {
                ia++;
                g_y->data[ic] += b_P[ib] * b_H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i53 = d_H->size[0] * d_H->size[1];
      d_H->size[0] = b_H->size[1];
      d_H->size[1] = b_H->size[0];
      emxEnsureCapacity((emxArray__common *)d_H, i53, (int)sizeof(double));
      loop_ub = b_H->size[0];
      for (i53 = 0; i53 < loop_ub; i53++) {
        br = b_H->size[1];
        for (i54 = 0; i54 < br; i54++) {
          d_H->data[i54 + d_H->size[0] * i53] = b_H->data[i53 + b_H->size[0] *
            i54];
        }
      }

      ii = b_H->size[0];
      i53 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = ii;
      emxEnsureCapacity((emxArray__common *)K, i53, (int)sizeof(double));
      for (i53 = 0; i53 < ii; i53++) {
        for (i54 = 0; i54 < 91; i54++) {
          K->data[i54 + K->size[0] * i53] = d_H->data[i54 + 91 * i53];
        }
      }

      a[0] = (unsigned int)g_y->size[0];
      a[1] = (unsigned int)K->size[1];
      i53 = d_C->size[0] * d_C->size[1];
      d_C->size[0] = (int)a[0];
      d_C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)d_C, i53, (int)sizeof(double));
      b_m = g_y->size[0];
      i53 = d_C->size[0] * d_C->size[1];
      emxEnsureCapacity((emxArray__common *)d_C, i53, (int)sizeof(double));
      loop_ub = d_C->size[1];
      for (i53 = 0; i53 < loop_ub; i53++) {
        br = d_C->size[0];
        for (i54 = 0; i54 < br; i54++) {
          d_C->data[i54 + d_C->size[0] * i53] = 0.0;
        }
      }

      if ((g_y->size[0] == 0) || (K->size[1] == 0)) {
      } else {
        ii = g_y->size[0] * (K->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          i53 = idx + b_m;
          for (ic = idx; ic + 1 <= i53; ic++) {
            d_C->data[ic] = 0.0;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= ii)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              i53 = idx + b_m;
              for (ic = idx; ic + 1 <= i53; ic++) {
                ia++;
                d_C->data[ic] += K->data[ib] * g_y->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      // 'OnePointRANSAC_EKF:163' K = (P*H')/S;
      i53 = e_H->size[0] * e_H->size[1];
      e_H->size[0] = b_H->size[1];
      e_H->size[1] = b_H->size[0];
      emxEnsureCapacity((emxArray__common *)e_H, i53, (int)sizeof(double));
      loop_ub = b_H->size[0];
      for (i53 = 0; i53 < loop_ub; i53++) {
        br = b_H->size[1];
        for (i54 = 0; i54 < br; i54++) {
          e_H->data[i54 + e_H->size[0] * i53] = b_H->data[i53 + b_H->size[0] *
            i54];
        }
      }

      ii = b_H->size[0];
      i53 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = ii;
      emxEnsureCapacity((emxArray__common *)K, i53, (int)sizeof(double));
      for (i53 = 0; i53 < ii; i53++) {
        for (i54 = 0; i54 < 91; i54++) {
          K->data[i54 + K->size[0] * i53] = e_H->data[i54 + 91 * i53];
        }
      }

      H_idx_0 = (unsigned int)K->size[1];
      i53 = h_y->size[0] * h_y->size[1];
      h_y->size[0] = 91;
      h_y->size[1] = (int)H_idx_0;
      h_y->size[0] = 91;
      emxEnsureCapacity((emxArray__common *)h_y, i53, (int)sizeof(double));
      loop_ub = h_y->size[1];
      for (i53 = 0; i53 < loop_ub; i53++) {
        for (i54 = 0; i54 < 91; i54++) {
          h_y->data[i54 + h_y->size[0] * i53] = 0.0;
        }
      }

      if (K->size[1] == 0) {
      } else {
        ii = 91 * (K->size[1] - 1);
        for (idx = 0; idx <= ii; idx += 91) {
          for (ic = idx; ic + 1 <= idx + 91; ic++) {
            h_y->data[ic] = 0.0;
          }
        }

        br = 0;
        for (idx = 0; idx <= ii; idx += 91) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              for (ic = idx; ic + 1 <= idx + 91; ic++) {
                ia++;
                h_y->data[ic] += K->data[ib] * b_P[ia - 1];
              }
            }

            ar += 91;
          }

          br += 91;
        }
      }

      i53 = e_C->size[0] * e_C->size[1];
      e_C->size[0] = d_C->size[0];
      e_C->size[1] = d_C->size[1];
      emxEnsureCapacity((emxArray__common *)e_C, i53, (int)sizeof(double));
      loop_ub = d_C->size[0] * d_C->size[1];
      for (i53 = 0; i53 < loop_ub; i53++) {
        e_C->data[i53] = d_C->data[i53] + R->data[i53];
      }

      mrdivide(h_y, e_C, K);
      i53 = b_K->size[0] * b_K->size[1];
      b_K->size[0] = 91;
      b_K->size[1] = K->size[1];
      emxEnsureCapacity((emxArray__common *)b_K, i53, (int)sizeof(double));
      loop_ub = K->size[0] * K->size[1];
      for (i53 = 0; i53 < loop_ub; i53++) {
        b_K->data[i53] = K->data[i53];
      }

      // 'OnePointRANSAC_EKF:165' x_apo = K*r;
      if ((b_K->size[1] == 1) || (r->size[0] == 1)) {
        loop_ub = b_K->size[0];
        for (i53 = 0; i53 < loop_ub; i53++) {
          x_apo_data[i53] = 0.0;
          br = b_K->size[1];
          for (i54 = 0; i54 < br; i54++) {
            b_x_apo_data = x_apo_data[i53] + b_K->data[i53 + b_K->size[0] * i54]
              * r->data[i54];
            x_apo_data[i53] = b_x_apo_data;
          }
        }
      } else {
        k = b_K->size[1];
        memset(&x_apo_data[0], 0, 91U * sizeof(double));
        for (ic = 1; ic < 92; ic++) {
          x_apo_data[ic - 1] = 0.0;
        }

        ar = 0;
        for (ib = 0; ib + 1 <= k; ib++) {
          if (r->data[ib] != 0.0) {
            ia = ar;
            for (ic = 0; ic + 1 < 92; ic++) {
              ia++;
              x_apo_data[ic] += r->data[ib] * b_K->data[ia - 1];
            }
          }

          ar += 91;
        }
      }

      // 'OnePointRANSAC_EKF:167' xt.robot_state.pos           = xt.robot_state.pos       + x_apo(1:3); 
      for (i53 = 0; i53 < 3; i53++) {
        b_xt->robot_state.pos[i53] += x_apo_data[i53];
      }

      // 'OnePointRANSAC_EKF:168' xt.robot_state.att           = quatmultJ(quatPlusThetaJ(x_apo(4:6)), xt.robot_state.att); 
      quatPlusThetaJ(*(double (*)[3])&x_apo_data[3], dv17);
      for (ii = 0; ii < 4; ii++) {
        c_xt[ii] = b_xt->robot_state.att[ii];
      }

      quatmultJ(dv17, c_xt, b_xt->robot_state.att);

      // 'OnePointRANSAC_EKF:169' xt.robot_state.vel           = xt.robot_state.vel       + x_apo(7:9); 
      // 'OnePointRANSAC_EKF:170' xt.robot_state.IMU.gyro_bias = xt.robot_state.IMU.gyro_bias + x_apo(10:12); 
      // 'OnePointRANSAC_EKF:171' xt.robot_state.IMU.acc_bias  = xt.robot_state.IMU.acc_bias + x_apo(13:15); 
      for (i53 = 0; i53 < 3; i53++) {
        b_xt->robot_state.vel[i53] += x_apo_data[6 + i53];
        b_xt->robot_state.IMU.gyro_bias[i53] += x_apo_data[9 + i53];
        b_xt->robot_state.IMU.acc_bias[i53] += x_apo_data[12 + i53];
      }

      // 'OnePointRANSAC_EKF:172' xt.origin.att                = quatmultJ(quatPlusThetaJ(x_apo(16:18)), xt.origin.att); 
      quatPlusThetaJ(*(double (*)[3])&x_apo_data[15], dv17);
      for (ii = 0; ii < 4; ii++) {
        c_xt[ii] = b_xt->origin.att[ii];
      }

      quatmultJ(dv17, c_xt, b_xt->origin.att);

      // 'OnePointRANSAC_EKF:173' xt.robot_state.IMU.att       = quatmultJ(quatPlusThetaJ(x_apo(19:21)), xt.robot_state.IMU.att); 
      quatPlusThetaJ(*(double (*)[3])&x_apo_data[18], dv17);
      for (ii = 0; ii < 4; ii++) {
        c_xt[ii] = b_xt->robot_state.IMU.att[ii];
      }

      quatmultJ(dv17, c_xt, b_xt->robot_state.IMU.att);

      // 'OnePointRANSAC_EKF:174' xt.robot_state.IMU.pos       = xt.robot_state.IMU.pos + x_apo(22:24); 
      for (i53 = 0; i53 < 3; i53++) {
        b_xt->robot_state.IMU.pos[i53] += x_apo_data[21 + i53];
      }

      // 'OnePointRANSAC_EKF:176' meas_idx = 1;
      // 'OnePointRANSAC_EKF:177' for anchorIdx = 1:numAnchors
      for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
        // 'OnePointRANSAC_EKF:178' xt.anchor_states(anchorIdx).pos = xt.anchor_states(anchorIdx).pos + x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:3)); 
        i53 = anchorIdx * 14;

        // 'OnePointRANSAC_EKF:179' xt.anchor_states(anchorIdx).att = quatmultJ(quatPlusThetaJ(x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(4:6))), xt.anchor_states(anchorIdx).att); 
        i54 = anchorIdx * 14;
        for (idx = 0; idx < 3; idx++) {
          b_xt->anchor_states[anchorIdx].pos[idx] += x_apo_data[(idx + i53) + 21];
          c_x_apo[idx] = x_apo_data[(idx + i54) + 24];
        }

        for (ii = 0; ii < 4; ii++) {
          c_xt[ii] = b_xt->anchor_states[anchorIdx].att[ii];
        }

        quatPlusThetaJ(c_x_apo, dv24);
        quatmultJ(dv24, c_xt, b_xt->anchor_states[anchorIdx].att);

        // 'OnePointRANSAC_EKF:181' for featureIdx = 1:numPointsPerAnchor
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          // 'OnePointRANSAC_EKF:182' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 1 
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              1) {
            //  only update active features
            // 'OnePointRANSAC_EKF:183' if HI_inlierStatus(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) 
            if (HI_inlierStatus[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1]) {
              // 'OnePointRANSAC_EKF:184' xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth + x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth += x_apo_data[(anchorIdx * 14 + featureIdx) + 27];

              // 'OnePointRANSAC_EKF:185' if it == VIOParameters.max_ekf_iterations && xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth > 10 
              if ((it == c_VIOParameters_max_ekf_iterati) &&
                  (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                   inverse_depth > 10.0)) {
                // 'OnePointRANSAC_EKF:186' ros_warn('Feature %i is very close. Depth: %f', int32(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx), 1/xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth) 
                d_ros_warn(b_xt->anchor_states[anchorIdx]
                           .feature_states[featureIdx].status_idx, 1.0 /
                           b_xt->anchor_states[anchorIdx]
                           .feature_states[featureIdx].inverse_depth);
              }

              // 'OnePointRANSAC_EKF:188' if xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth < 0 
              if ((b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                   inverse_depth < 0.0) && (it ==
                   c_VIOParameters_max_ekf_iterati)) {
                // 'OnePointRANSAC_EKF:189' if it == VIOParameters.max_ekf_iterations 
                //  only reject if we are done iterating
                // 'OnePointRANSAC_EKF:190' ros_warn('Feature %i (%i on %i) is behind its anchor, rejecting', int32(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx), int32(featureIdx), int32(anchorIdx)) 
                b_ros_warn(b_xt->anchor_states[anchorIdx]
                           .feature_states[featureIdx].status_idx, featureIdx +
                           1, anchorIdx + 1);

                // 'OnePointRANSAC_EKF:191' updateVect(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
                updateVect[b_xt->anchor_states[anchorIdx]
                  .feature_states[featureIdx].status_idx - 1] = 0;

                // 'OnePointRANSAC_EKF:192' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
                b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status
                  = 0;

                // 'OnePointRANSAC_EKF:193' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
                b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                  status_idx = 0;
              }
            } else {
              if (HI_inlierCandidates[b_xt->anchor_states[anchorIdx].
                  feature_states[featureIdx].status_idx - 1] && (it ==
                   c_VIOParameters_max_ekf_iterati)) {
                // 'OnePointRANSAC_EKF:196' elseif HI_inlierCandidates(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) 
                //  if it is not a HI inlier, but was a candidate, it was rejected by mahalanobis 
                // 'OnePointRANSAC_EKF:197' if it == VIOParameters.max_ekf_iterations 
                //  only reject if we are done iterating
                //                              ros_info('Rejecting %i (%i on %i)', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx, featureIdx, anchorIdx) 
                // 'OnePointRANSAC_EKF:199' updateVect(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
                updateVect[b_xt->anchor_states[anchorIdx]
                  .feature_states[featureIdx].status_idx - 1] = 0;

                // 'OnePointRANSAC_EKF:200' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
                b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status
                  = 0;

                // 'OnePointRANSAC_EKF:201' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
                b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                  status_idx = 0;
              }
            }
          }

          // 'OnePointRANSAC_EKF:205' meas_idx = meas_idx + 1;
        }
      }

      it++;
    }

    emxFree_real_T(&e_H);
    emxFree_real_T(&d_H);
    emxFree_real_T(&c_H);
    emxFree_real_T(&e_C);
    emxFree_real_T(&h_y);
    emxFree_real_T(&g_y);
    emxFree_real_T(&f_y);
    emxFree_real_T(&d_C);
    emxFree_real_T(&c_C);

    // 'OnePointRANSAC_EKF:211' P = (eye(numStates + numAnchors*numStatesPerAnchor)-K*H)*P; 
    emxInit_real_T1(&f_C, 2);
    if ((b_K->size[1] == 1) || (b_H->size[0] == 1)) {
      i53 = f_C->size[0] * f_C->size[1];
      f_C->size[0] = b_K->size[0];
      f_C->size[1] = b_H->size[1];
      emxEnsureCapacity((emxArray__common *)f_C, i53, (int)sizeof(double));
      loop_ub = b_K->size[0];
      for (i53 = 0; i53 < loop_ub; i53++) {
        br = b_H->size[1];
        for (i54 = 0; i54 < br; i54++) {
          f_C->data[i53 + f_C->size[0] * i54] = 0.0;
          ii = b_K->size[1];
          for (idx = 0; idx < ii; idx++) {
            f_C->data[i53 + f_C->size[0] * i54] += b_K->data[i53 + b_K->size[0] *
              idx] * b_H->data[idx + b_H->size[0] * i54];
          }
        }
      }
    } else {
      k = b_K->size[1];
      a[0] = (signed char)b_K->size[0];
      a[1] = (signed char)b_H->size[1];
      i53 = f_C->size[0] * f_C->size[1];
      f_C->size[0] = (int)a[0];
      f_C->size[1] = (int)a[1];
      emxEnsureCapacity((emxArray__common *)f_C, i53, (int)sizeof(double));
      b_m = b_K->size[0];
      i53 = f_C->size[0] * f_C->size[1];
      emxEnsureCapacity((emxArray__common *)f_C, i53, (int)sizeof(double));
      loop_ub = f_C->size[1];
      for (i53 = 0; i53 < loop_ub; i53++) {
        br = f_C->size[0];
        for (i54 = 0; i54 < br; i54++) {
          f_C->data[i54 + f_C->size[0] * i53] = 0.0;
        }
      }

      ii = b_K->size[0] * (b_H->size[1] - 1);
      for (idx = 0; idx <= ii; idx += b_m) {
        i53 = idx + b_m;
        for (ic = idx; ic + 1 <= i53; ic++) {
          f_C->data[ic] = 0.0;
        }
      }

      br = 0;
      for (idx = 0; idx <= ii; idx += b_m) {
        ar = 0;
        i53 = br + k;
        for (ib = br; ib + 1 <= i53; ib++) {
          if (b_H->data[ib] != 0.0) {
            ia = ar;
            i54 = idx + b_m;
            for (ic = idx; ic + 1 <= i54; ic++) {
              ia++;
              f_C->data[ic] += b_H->data[ib] * b_K->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += k;
      }
    }

    emxFree_real_T(&b_H);
    emxFree_real_T(&b_K);
    c_eye(dv21);
    for (i53 = 0; i53 < 91; i53++) {
      for (i54 = 0; i54 < 91; i54++) {
        dv22[i54 + 91 * i53] = dv21[i54 + 91 * i53] - f_C->data[i54 + 91 * i53];
      }
    }

    emxFree_real_T(&f_C);
    for (i53 = 0; i53 < 91; i53++) {
      for (i54 = 0; i54 < 91; i54++) {
        dv25[i53 + 91 * i54] = 0.0;
        for (idx = 0; idx < 91; idx++) {
          dv25[i53 + 91 * i54] += dv22[i53 + 91 * idx] * b_P[idx + 91 * i54];
        }
      }
    }

    for (i53 = 0; i53 < 91; i53++) {
      memcpy(&b_P[i53 * 91], &dv25[i53 * 91], 91U * sizeof(double));
    }
  }

  emxFree_real_T(&S);

  // % Update the delayed initialization features
  // 'OnePointRANSAC_EKF:215' if VIOParameters.delayed_initialization
  if (c_VIOParameters_delayed_initial) {
    emxInit_real_T1(&i_y, 2);

    // 'OnePointRANSAC_EKF:216' xt = getScaledMap(xt);
    getScaledMap(b_xt);

    // 'OnePointRANSAC_EKF:217' [r, H, R] = getH_R_res(xt, z_u_l, delayedFeatures, cameraparams, noiseParameters); 
    getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att, b_xt->anchor_states,
               z_u_l, delayedFeatures, cameraparams_FocalLength,
               cameraparams_PrincipalPoint, noiseParameters_image_noise, r, H, R);

    // 'OnePointRANSAC_EKF:219' S = (H*P*H'+R);
    H_idx_0 = (unsigned int)H->size[0];
    i53 = i_y->size[0] * i_y->size[1];
    i_y->size[0] = (int)H_idx_0;
    i_y->size[1] = 91;
    emxEnsureCapacity((emxArray__common *)i_y, i53, (int)sizeof(double));
    b_m = H->size[0];
    i53 = i_y->size[0] * i_y->size[1];
    i_y->size[1] = 91;
    emxEnsureCapacity((emxArray__common *)i_y, i53, (int)sizeof(double));
    for (i53 = 0; i53 < 91; i53++) {
      loop_ub = i_y->size[0];
      for (i54 = 0; i54 < loop_ub; i54++) {
        i_y->data[i54 + i_y->size[0] * i53] = 0.0;
      }
    }

    if (H->size[0] == 0) {
    } else {
      ii = H->size[0] * 90;
      idx = 0;
      while ((b_m > 0) && (idx <= ii)) {
        i53 = idx + b_m;
        for (ic = idx; ic + 1 <= i53; ic++) {
          i_y->data[ic] = 0.0;
        }

        idx += b_m;
      }

      br = 0;
      idx = 0;
      while ((b_m > 0) && (idx <= ii)) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 91; ib++) {
          if (b_P[ib] != 0.0) {
            ia = ar;
            i53 = idx + b_m;
            for (ic = idx; ic + 1 <= i53; ic++) {
              ia++;
              i_y->data[ic] += b_P[ib] * H->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += 91;
        idx += b_m;
      }
    }

    i53 = K->size[0] * K->size[1];
    K->size[0] = 91;
    K->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)K, i53, (int)sizeof(double));
    loop_ub = H->size[0];
    for (i53 = 0; i53 < loop_ub; i53++) {
      for (i54 = 0; i54 < 91; i54++) {
        K->data[i54 + K->size[0] * i53] = H->data[i53 + H->size[0] * i54];
      }
    }

    emxInit_real_T1(&g_C, 2);
    a[0] = (unsigned int)i_y->size[0];
    a[1] = (unsigned int)K->size[1];
    i53 = g_C->size[0] * g_C->size[1];
    g_C->size[0] = (int)a[0];
    g_C->size[1] = (int)a[1];
    emxEnsureCapacity((emxArray__common *)g_C, i53, (int)sizeof(double));
    b_m = i_y->size[0];
    i53 = g_C->size[0] * g_C->size[1];
    emxEnsureCapacity((emxArray__common *)g_C, i53, (int)sizeof(double));
    loop_ub = g_C->size[1];
    for (i53 = 0; i53 < loop_ub; i53++) {
      br = g_C->size[0];
      for (i54 = 0; i54 < br; i54++) {
        g_C->data[i54 + g_C->size[0] * i53] = 0.0;
      }
    }

    if ((i_y->size[0] == 0) || (K->size[1] == 0)) {
    } else {
      ii = i_y->size[0] * (K->size[1] - 1);
      idx = 0;
      while ((b_m > 0) && (idx <= ii)) {
        i53 = idx + b_m;
        for (ic = idx; ic + 1 <= i53; ic++) {
          g_C->data[ic] = 0.0;
        }

        idx += b_m;
      }

      br = 0;
      idx = 0;
      while ((b_m > 0) && (idx <= ii)) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 91; ib++) {
          if (K->data[ib] != 0.0) {
            ia = ar;
            i53 = idx + b_m;
            for (ic = idx; ic + 1 <= i53; ic++) {
              ia++;
              g_C->data[ic] += K->data[ib] * i_y->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += 91;
        idx += b_m;
      }
    }

    emxFree_real_T(&i_y);

    // 'OnePointRANSAC_EKF:220' K = (P*H')/S;
    i53 = K->size[0] * K->size[1];
    K->size[0] = 91;
    K->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)K, i53, (int)sizeof(double));
    loop_ub = H->size[0];
    for (i53 = 0; i53 < loop_ub; i53++) {
      for (i54 = 0; i54 < 91; i54++) {
        K->data[i54 + K->size[0] * i53] = H->data[i53 + H->size[0] * i54];
      }
    }

    emxInit_real_T1(&j_y, 2);
    H_idx_0 = (unsigned int)K->size[1];
    i53 = j_y->size[0] * j_y->size[1];
    j_y->size[0] = 91;
    j_y->size[1] = (int)H_idx_0;
    j_y->size[0] = 91;
    emxEnsureCapacity((emxArray__common *)j_y, i53, (int)sizeof(double));
    loop_ub = j_y->size[1];
    for (i53 = 0; i53 < loop_ub; i53++) {
      for (i54 = 0; i54 < 91; i54++) {
        j_y->data[i54 + j_y->size[0] * i53] = 0.0;
      }
    }

    if (K->size[1] == 0) {
    } else {
      ii = 91 * (K->size[1] - 1);
      for (idx = 0; idx <= ii; idx += 91) {
        for (ic = idx; ic + 1 <= idx + 91; ic++) {
          j_y->data[ic] = 0.0;
        }
      }

      br = 0;
      for (idx = 0; idx <= ii; idx += 91) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 91; ib++) {
          if (K->data[ib] != 0.0) {
            ia = ar;
            for (ic = idx; ic + 1 <= idx + 91; ic++) {
              ia++;
              j_y->data[ic] += K->data[ib] * b_P[ia - 1];
            }
          }

          ar += 91;
        }

        br += 91;
      }
    }

    emxInit_real_T1(&h_C, 2);
    i53 = h_C->size[0] * h_C->size[1];
    h_C->size[0] = g_C->size[0];
    h_C->size[1] = g_C->size[1];
    emxEnsureCapacity((emxArray__common *)h_C, i53, (int)sizeof(double));
    loop_ub = g_C->size[0] * g_C->size[1];
    for (i53 = 0; i53 < loop_ub; i53++) {
      h_C->data[i53] = g_C->data[i53] + R->data[i53];
    }

    emxFree_real_T(&g_C);
    mrdivide(j_y, h_C, K);

    // 'OnePointRANSAC_EKF:222' x_apo = K*r;
    emxFree_real_T(&h_C);
    emxFree_real_T(&j_y);
    if ((K->size[1] == 1) || (r->size[0] == 1)) {
      for (i53 = 0; i53 < 91; i53++) {
        x_apo[i53] = 0.0;
        loop_ub = K->size[1];
        for (i54 = 0; i54 < loop_ub; i54++) {
          b_x_apo = x_apo[i53] + K->data[i53 + K->size[0] * i54] * r->data[i54];
          x_apo[i53] = b_x_apo;
        }
      }
    } else {
      memset(&x_apo[0], 0, 91U * sizeof(double));
      ar = 0;
      for (ib = 0; ib + 1 <= K->size[1]; ib++) {
        if (r->data[ib] != 0.0) {
          ia = ar;
          for (ic = 0; ic < 91; ic++) {
            ia++;
            b_x_apo = x_apo[ic] + r->data[ib] * K->data[ia - 1];
            x_apo[ic] = b_x_apo;
          }
        }

        ar += 91;
      }
    }

    // 'OnePointRANSAC_EKF:224' for anchorIdx = 1:numAnchors
    for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
      // 'OnePointRANSAC_EKF:225' for featureIdx = 1:numPointsPerAnchor
      for (featureIdx = 0; featureIdx < 8; featureIdx++) {
        // 'OnePointRANSAC_EKF:226' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
            2) {
          // 'OnePointRANSAC_EKF:227' xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth + x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
          b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
            inverse_depth += x_apo[(anchorIdx * 14 + featureIdx) + 27];
        }
      }
    }

    // 'OnePointRANSAC_EKF:232' P = (eye(numStates + numAnchors*numStatesPerAnchor)-K*H)*P; 
    if ((K->size[1] == 1) || (H->size[0] == 1)) {
      for (i53 = 0; i53 < 91; i53++) {
        for (i54 = 0; i54 < 91; i54++) {
          e_y[i53 + 91 * i54] = 0.0;
          loop_ub = K->size[1];
          for (idx = 0; idx < loop_ub; idx++) {
            e_y[i53 + 91 * i54] += K->data[i53 + K->size[0] * idx] * H->data[idx
              + H->size[0] * i54];
          }
        }
      }
    } else {
      k = K->size[1];
      memset(&e_y[0], 0, 8281U * sizeof(double));
      for (idx = 0; idx <= 8191; idx += 91) {
        for (ic = idx; ic + 1 <= idx + 91; ic++) {
          e_y[ic] = 0.0;
        }
      }

      br = 0;
      for (idx = 0; idx <= 8191; idx += 91) {
        ar = 0;
        i53 = br + k;
        for (ib = br; ib + 1 <= i53; ib++) {
          if (H->data[ib] != 0.0) {
            ia = ar;
            for (ic = idx; ic + 1 <= idx + 91; ic++) {
              ia++;
              e_y[ic] += H->data[ib] * K->data[ia - 1];
            }
          }

          ar += 91;
        }

        br += k;
      }
    }

    c_eye(dv21);
    for (i53 = 0; i53 < 91; i53++) {
      for (i54 = 0; i54 < 91; i54++) {
        dv22[i54 + 91 * i53] = dv21[i54 + 91 * i53] - e_y[i54 + 91 * i53];
      }
    }

    for (i53 = 0; i53 < 91; i53++) {
      for (i54 = 0; i54 < 91; i54++) {
        dv26[i53 + 91 * i54] = 0.0;
        for (idx = 0; idx < 91; idx++) {
          dv26[i53 + 91 * i54] += dv22[i53 + 91 * idx] * b_P[idx + 91 * i54];
        }
      }
    }

    for (i53 = 0; i53 < 91; i53++) {
      memcpy(&b_P[i53 * 91], &dv26[i53 * 91], 91U * sizeof(double));
    }
  }

  emxFree_real_T(&R);
  emxFree_real_T(&H);
  emxFree_real_T(&r);
  emxFree_real_T(&K);

  // %
  //  if ~any(HI_inlierStatus | LI_inlier_status)
  //      ros_error('1-point RANSAC rejected all features! Resetting velocity')
  //      xt.robot_state.vel = zeros(3,1);
  //      P(7:9, :) = P(7:9, :)*10;
  //      P(:, 7:9) = P(:, 7:9)*10;
  //  end
  // 'OnePointRANSAC_EKF:241' num_active_features_before = nnz(activeFeatures);
  ii = 0;

  // 'OnePointRANSAC_EKF:242' num_active_features_after  = nnz(HI_inlierStatus | LI_inlier_status); 
  idx = 0;
  for (k = 0; k < 40; k++) {
    if (activeFeatures[k]) {
      ii++;
    }

    if (HI_inlierStatus[k] || LI_inlier_status[k]) {
      idx++;
    }
  }

  // 'OnePointRANSAC_EKF:243' rejected_ratio = num_active_features_after/num_active_features_before; 
  rejected_ratio = (double)idx / (double)ii;

  // 'OnePointRANSAC_EKF:245' if rejected_ratio < 0.1
  if (rejected_ratio < 0.1) {
    //  if more than 90% were rejected
    // 'OnePointRANSAC_EKF:246' ros_error('1-point RANSAC rejected %d%% of all features! Resetting velocity.', int32(100-rejected_ratio*100)) 
    // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
    // 'ros_error:4' if coder.target('MATLAB')
    // 'ros_error:6' elseif ~coder.target('MEX')
    // 'ros_error:7' coder.cinclude('<ros/console.h>')
    // 'ros_error:8' coder.ceval('ROS_ERROR', [str, 0], varargin{:});
    memcpy(&cv52[0], &cv53[0], 66U * sizeof(char));
    d10 = rt_roundd_snf(100.0 - rejected_ratio * 100.0);
    if (d10 < 2.147483648E+9) {
      i53 = (int)d10;
    } else {
      i53 = MAX_int32_T;
    }

    ROS_ERROR(cv52, i53);

    // 'OnePointRANSAC_EKF:248' att_cov = P(  4:6,   4:6);
    // 'OnePointRANSAC_EKF:249' gryro_bias_cov = P(10:12, 10:12);
    // 'OnePointRANSAC_EKF:250' acc_bias_cov = P(13:15, 13:15);
    // 'OnePointRANSAC_EKF:251' origin_att_cov = P(16:18, 16:18);
    // 'OnePointRANSAC_EKF:252' R_ci_cov = P(19:21, 19:21);
    for (i53 = 0; i53 < 3; i53++) {
      for (i54 = 0; i54 < 3; i54++) {
        att_cov[i54 + 3 * i53] = b_P[(i54 + 91 * (3 + i53)) + 3];
        gryro_bias_cov[i54 + 3 * i53] = b_P[(i54 + 91 * (9 + i53)) + 9];
        acc_bias_cov[i54 + 3 * i53] = b_P[(i54 + 91 * (12 + i53)) + 12];
        origin_att_cov[i54 + 3 * i53] = b_P[(i54 + 91 * (15 + i53)) + 15];
        R_ci_cov[i54 + 3 * i53] = b_P[(i54 + 91 * (18 + i53)) + 18];
      }
    }

    // 'OnePointRANSAC_EKF:254' P(:, :) = 0;
    for (i53 = 0; i53 < 8281; i53++) {
      b_P[i53] = 0.0;
    }

    // 'OnePointRANSAC_EKF:256' P(  4:6,   4:6) = att_cov;
    //  orientation of camera in origin frame
    // 'OnePointRANSAC_EKF:257' P(  7:9,   7:9) = 1*eye(3);
    //  velocity
    // 'OnePointRANSAC_EKF:258' P(10:12, 10:12) = gryro_bias_cov;
    //  gyro bias
    // 'OnePointRANSAC_EKF:259' P(13:15, 13:15) = acc_bias_cov;
    //  acc bias
    // 'OnePointRANSAC_EKF:260' P(16:18, 16:18) = origin_att_cov;
    //  origin orientation
    // 'OnePointRANSAC_EKF:261' P(19:21, 19:21) = R_ci_cov;
    for (i53 = 0; i53 < 3; i53++) {
      for (i54 = 0; i54 < 3; i54++) {
        b_P[(i54 + 91 * (3 + i53)) + 3] = att_cov[i54 + 3 * i53];
        b_P[(i54 + 91 * (6 + i53)) + 6] = k_y[i54 + 3 * i53];
        b_P[(i54 + 91 * (9 + i53)) + 9] = gryro_bias_cov[i54 + 3 * i53];
        b_P[(i54 + 91 * (12 + i53)) + 12] = acc_bias_cov[i54 + 3 * i53];
        b_P[(i54 + 91 * (15 + i53)) + 15] = origin_att_cov[i54 + 3 * i53];
        b_P[(i54 + 91 * (18 + i53)) + 18] = R_ci_cov[i54 + 3 * i53];
      }
    }

    //  R_ci
    //  set all features inactive
    // 'OnePointRANSAC_EKF:264' for anchorIdx = 1:numAnchors
    for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
      // 'OnePointRANSAC_EKF:265' for featureIdx = 1:numPointsPerAnchor
      for (featureIdx = 0; featureIdx < 8; featureIdx++) {
        // 'OnePointRANSAC_EKF:266' updateVect(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
        updateVect[b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
          status_idx - 1] = 0;

        // 'OnePointRANSAC_EKF:267' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
        b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status = 0;

        // 'OnePointRANSAC_EKF:268' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
        b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status_idx = 0;
      }
    }

    // 'OnePointRANSAC_EKF:271' for i = 1:length(updateVect)
    for (ii = 0; ii < 40; ii++) {
      // 'OnePointRANSAC_EKF:272' fprintf('%d, ', updateVect(i))
      s_fprintf(updateVect[ii]);
    }

    // 'OnePointRANSAC_EKF:274' fprintf('\n')
    b_fprintf();
  }
}

//
// THIS IS OK, It is according to the NASA memo found
//  https://claraty.jpl.nasa.gov/man/software/development/conventions/standards_docs/unadopted/JPL_Quaternions_Breckenridge.pdf
// QUATFROMROTJ Get equivalent quaternion of the rotation matrix R
//    Returns a quaternion in JPL notation
//  The implementation is copied from qGetQ(R), but we are careful about the
//  ordering of the output vector
//  Q = [0;0;0;1];
//  % [r,c] = size( R );
//  % if( r ~= 3 || c ~= 3 )
//  %     error( 'R must be a 3x3 matrix\n\r' );
//  % end
//
//  if abs(det(R) - 1) > 1e-2
//      error('The provided matrix is not a valid rotation matrix. It must have det(R) = 1, but is %i.', det(R))
//  end
//  T = R(1,1)+R(2,2)+R(3,3);
//  pivot = zeros(4,1);
//  pivot(1) = sqrt(max(1+2*R(1,1)-T, eps))/2;
//  pivot(2) = sqrt(max(1+2*R(2,2)-T, eps))/2;
//  pivot(3) = sqrt(max(1+2*R(3,3)-T, eps))/2;
//  pivot(4) = sqrt(max(1+T, eps))/2;
//  if ~all(isreal(pivot))
//      error('complex')
//  end
//  index = round(find(pivot==max(pivot)));
//  if(index(1)==1)
//      Q(1) = sqrt(1+2*R(1,1)-T)/2;
//      Q(2) = (R(1,2)+R(2,1))/(4*(Q(1)));
//      Q(3) = (R(1,3)+R(3,1))/(4*(Q(1)));
//      Q(4) = (R(2,3)-R(3,2))/(4*(Q(1)));
//  elseif(index(1)==2)
//      Q(2) = sqrt(1+2*R(2,2)-T)/2;
//      Q(1) = (R(1,2)+R(2,1))/(4*Q(2));
//      Q(3) = (R(2,3)+R(3,2))/(4*Q(2));
//      Q(4) = (R(3,1)-R(1,3))/(4*Q(2));
//  elseif(index(1)==3)
//      Q(3) = sqrt(1+2*R(3,3)-T)/2;
//      Q(1) = (R(1,3)+R(3,1))/(4*Q(3));
//      Q(2) = (R(2,3)+R(3,2))/(4*Q(3));
//      Q(4) = (R(1,2)-R(2,1))/(4*Q(3));
//  elseif(index(1)==4)
//      Q(4) = sqrt(1+T)/2;
//      Q(1) = (R(2,3)-R(3,2))/(4*Q(4));
//      Q(2) = (R(3,1)-R(1,3))/(4*Q(4));
//      Q(3) = (R(1,2)-R(2,1))/(4*Q(4));
//  end
// Arguments    : const double R[9]
//                double Q[4]
// Return Type  : void
//
static void QuatFromRotJ(const double R[9], double Q[4])
{
  double varargin_1[4];
  int ixstart;
  double mtmp;
  int itmp;
  int ix;
  boolean_T exitg1;

  // % speed optimization
  // 'QuatFromRotJ:50' [~, index] = max([1+R(1,1)-(R(2,2)+R(3,3)), 1+R(2,2)-(R(1,1)+R(3,3)), 1+R(3,3)-(R(1,1)+R(2,2)), 1+(R(1,1)+R(2,2)+R(3,3))]); 
  varargin_1[0] = (1.0 + R[0]) - (R[4] + R[8]);
  varargin_1[1] = (1.0 + R[4]) - (R[0] + R[8]);
  varargin_1[2] = (1.0 + R[8]) - (R[0] + R[4]);
  varargin_1[3] = 1.0 + ((R[0] + R[4]) + R[8]);
  ixstart = 1;
  mtmp = varargin_1[0];
  itmp = 1;
  if (rtIsNaN(varargin_1[0])) {
    ix = 2;
    exitg1 = false;
    while ((!exitg1) && (ix < 5)) {
      ixstart = ix;
      if (!rtIsNaN(varargin_1[ix - 1])) {
        mtmp = varargin_1[ix - 1];
        itmp = ix;
        exitg1 = true;
      } else {
        ix++;
      }
    }
  }

  if (ixstart < 4) {
    while (ixstart + 1 < 5) {
      if (varargin_1[ixstart] > mtmp) {
        mtmp = varargin_1[ixstart];
        itmp = ixstart + 1;
      }

      ixstart++;
    }
  }

  // 'QuatFromRotJ:50' ~
  // 'QuatFromRotJ:52' if(index==1)
  if (itmp == 1) {
    // 'QuatFromRotJ:53' Q = [sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2;
    // 'QuatFromRotJ:54'         (R(1,2)+R(2,1))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
    // 'QuatFromRotJ:55'         (R(1,3)+R(3,1))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
    // 'QuatFromRotJ:56'         (R(2,3)-R(3,2))/(4*(sqrt(1+2*R(1,1)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
    Q[0] = sqrt((1.0 + 2.0 * R[0]) - ((R[0] + R[4]) + R[8])) / 2.0;
    Q[1] = (R[3] + R[1]) / (4.0 * (sqrt((1.0 + 2.0 * R[0]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
    Q[2] = (R[6] + R[2]) / (4.0 * (sqrt((1.0 + 2.0 * R[0]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
    Q[3] = (R[7] - R[5]) / (4.0 * (sqrt((1.0 + 2.0 * R[0]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
  } else if (itmp == 2) {
    // 'QuatFromRotJ:57' elseif(index==2)
    // 'QuatFromRotJ:58' Q = [(R(1,2)+R(2,1))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
    // 'QuatFromRotJ:59'         (sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2);
    // 'QuatFromRotJ:60'         (R(2,3)+R(3,2))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
    // 'QuatFromRotJ:61'         (R(3,1)-R(1,3))/(4*(sqrt(1+2*R(2,2)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
    Q[0] = (R[3] + R[1]) / (4.0 * (sqrt((1.0 + 2.0 * R[4]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
    Q[1] = sqrt((1.0 + 2.0 * R[4]) - ((R[0] + R[4]) + R[8])) / 2.0;
    Q[2] = (R[7] + R[5]) / (4.0 * (sqrt((1.0 + 2.0 * R[4]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
    Q[3] = (R[2] - R[6]) / (4.0 * (sqrt((1.0 + 2.0 * R[4]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
  } else if (itmp == 3) {
    // 'QuatFromRotJ:62' elseif(index==3)
    // 'QuatFromRotJ:63' Q = [(R(1,3)+R(3,1))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
    // 'QuatFromRotJ:64'         (R(2,3)+R(3,2))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2)); 
    // 'QuatFromRotJ:65'         (sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2);
    // 'QuatFromRotJ:66'         (R(1,2)-R(2,1))/(4*(sqrt(1+2*R(3,3)-(R(1,1)+R(2,2)+R(3,3)))/2));]; 
    Q[0] = (R[6] + R[2]) / (4.0 * (sqrt((1.0 + 2.0 * R[8]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
    Q[1] = (R[7] + R[5]) / (4.0 * (sqrt((1.0 + 2.0 * R[8]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
    Q[2] = sqrt((1.0 + 2.0 * R[8]) - ((R[0] + R[4]) + R[8])) / 2.0;
    Q[3] = (R[3] - R[1]) / (4.0 * (sqrt((1.0 + 2.0 * R[8]) - ((R[0] + R[4]) + R
      [8])) / 2.0));
  } else {
    // 'QuatFromRotJ:67' else
    // 'QuatFromRotJ:68' Q = [(R(2,3)-R(3,2))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
    // 'QuatFromRotJ:69'         (R(3,1)-R(1,3))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
    // 'QuatFromRotJ:70'         (R(1,2)-R(2,1))/(4*(sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2)); 
    // 'QuatFromRotJ:71'         (sqrt(1+(R(1,1)+R(2,2)+R(3,3)))/2);];
    Q[0] = (R[7] - R[5]) / (4.0 * (sqrt(1.0 + ((R[0] + R[4]) + R[8])) / 2.0));
    Q[1] = (R[2] - R[6]) / (4.0 * (sqrt(1.0 + ((R[0] + R[4]) + R[8])) / 2.0));
    Q[2] = (R[3] - R[1]) / (4.0 * (sqrt(1.0 + ((R[0] + R[4]) + R[8])) / 2.0));
    Q[3] = sqrt(1.0 + ((R[0] + R[4]) + R[8])) / 2.0;
  }
}

//
// Arguments    : double P_apo[8281]
//                g_struct_T *x
//                double dt
//                double processNoise_qv
//                double processNoise_qw
//                double processNoise_qao
//                double processNoise_qwo
//                double processNoise_qR_ci
//                const double measurements_acc_duo[3]
//                const double measurements_gyr_duo[3]
// Return Type  : void
//
static void SLAM_pred_euler(double P_apo[8281], g_struct_T *x, double dt, double
  processNoise_qv, double processNoise_qw, double processNoise_qao, double
  processNoise_qwo, double processNoise_qR_ci, const double
  measurements_acc_duo[3], const double measurements_gyr_duo[3])
{
  double R_cw[9];
  double R_ci[9];
  double b_R_ci[9];
  int i48;
  int i;
  double t_ci[3];
  double w_imu[3];
  double R[9];
  double b_R[9];
  double b_measurements_acc_duo[3];
  double w_c[3];
  double d9;
  int i49;
  double b_x[9];
  double a_c[3];
  double grav_origin[3];
  double y[3];
  static const double b[3] = { 0.0, 0.0, 9.81 };

  double dv6[9];
  double b_y[3];
  double c_y[3];
  double G[315];
  static const signed char iv2[45] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0 };

  static const signed char iv3[45] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0 };

  static const signed char iv4[45] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0 };

  static const signed char iv5[45] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    1, 0, 0, 0, 1 };

  double P_xx_apr[441];
  double dv7[9];
  double dv8[9];
  double dv9[9];
  double dv10[9];
  double dv11[9];
  double dv12[9];
  double dv13[9];
  double dv14[9];
  double Phi[441];
  static const signed char iv6[63] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv7[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  double b_Phi[441];
  double b_processNoise_qw[15];
  double dv15[225];
  double b_G[315];
  double c_G[441];
  double P_xs_apr[1470];
  double c_x[4];
  double dv16[4];

  // 'SLAM_pred_euler:3' R_cw = RotFromQuatJ(x.robot_state.att);
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
  // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
  // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
  R_cw[0] = ((x->robot_state.att[0] * x->robot_state.att[0] - x->
              robot_state.att[1] * x->robot_state.att[1]) - x->robot_state.att[2]
             * x->robot_state.att[2]) + x->robot_state.att[3] *
    x->robot_state.att[3];
  R_cw[3] = 2.0 * (x->robot_state.att[0] * x->robot_state.att[1] +
                   x->robot_state.att[2] * x->robot_state.att[3]);
  R_cw[6] = 2.0 * (x->robot_state.att[0] * x->robot_state.att[2] -
                   x->robot_state.att[1] * x->robot_state.att[3]);
  R_cw[1] = 2.0 * (x->robot_state.att[0] * x->robot_state.att[1] -
                   x->robot_state.att[2] * x->robot_state.att[3]);
  R_cw[4] = ((-(x->robot_state.att[0] * x->robot_state.att[0]) +
              x->robot_state.att[1] * x->robot_state.att[1]) -
             x->robot_state.att[2] * x->robot_state.att[2]) + x->
    robot_state.att[3] * x->robot_state.att[3];
  R_cw[7] = 2.0 * (x->robot_state.att[1] * x->robot_state.att[2] +
                   x->robot_state.att[0] * x->robot_state.att[3]);
  R_cw[2] = 2.0 * (x->robot_state.att[0] * x->robot_state.att[2] +
                   x->robot_state.att[1] * x->robot_state.att[3]);
  R_cw[5] = 2.0 * (x->robot_state.att[1] * x->robot_state.att[2] -
                   x->robot_state.att[0] * x->robot_state.att[3]);
  R_cw[8] = ((-(x->robot_state.att[0] * x->robot_state.att[0]) -
              x->robot_state.att[1] * x->robot_state.att[1]) +
             x->robot_state.att[2] * x->robot_state.att[2]) + x->
    robot_state.att[3] * x->robot_state.att[3];

  //  rotation in origin frame
  // 'SLAM_pred_euler:5' R_ci = RotFromQuatJ(x.robot_state.IMU.att);
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
  // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
  // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
  R_ci[0] = ((x->robot_state.IMU.att[0] * x->robot_state.IMU.att[0] -
              x->robot_state.IMU.att[1] * x->robot_state.IMU.att[1]) -
             x->robot_state.IMU.att[2] * x->robot_state.IMU.att[2]) +
    x->robot_state.IMU.att[3] * x->robot_state.IMU.att[3];
  R_ci[3] = 2.0 * (x->robot_state.IMU.att[0] * x->robot_state.IMU.att[1] +
                   x->robot_state.IMU.att[2] * x->robot_state.IMU.att[3]);
  R_ci[6] = 2.0 * (x->robot_state.IMU.att[0] * x->robot_state.IMU.att[2] -
                   x->robot_state.IMU.att[1] * x->robot_state.IMU.att[3]);
  R_ci[1] = 2.0 * (x->robot_state.IMU.att[0] * x->robot_state.IMU.att[1] -
                   x->robot_state.IMU.att[2] * x->robot_state.IMU.att[3]);
  R_ci[4] = ((-(x->robot_state.IMU.att[0] * x->robot_state.IMU.att[0]) +
              x->robot_state.IMU.att[1] * x->robot_state.IMU.att[1]) -
             x->robot_state.IMU.att[2] * x->robot_state.IMU.att[2]) +
    x->robot_state.IMU.att[3] * x->robot_state.IMU.att[3];
  R_ci[7] = 2.0 * (x->robot_state.IMU.att[1] * x->robot_state.IMU.att[2] +
                   x->robot_state.IMU.att[0] * x->robot_state.IMU.att[3]);
  R_ci[2] = 2.0 * (x->robot_state.IMU.att[0] * x->robot_state.IMU.att[2] +
                   x->robot_state.IMU.att[1] * x->robot_state.IMU.att[3]);
  R_ci[5] = 2.0 * (x->robot_state.IMU.att[1] * x->robot_state.IMU.att[2] -
                   x->robot_state.IMU.att[0] * x->robot_state.IMU.att[3]);
  R_ci[8] = ((-(x->robot_state.IMU.att[0] * x->robot_state.IMU.att[0]) -
              x->robot_state.IMU.att[1] * x->robot_state.IMU.att[1]) +
             x->robot_state.IMU.att[2] * x->robot_state.IMU.att[2]) +
    x->robot_state.IMU.att[3] * x->robot_state.IMU.att[3];

  // 'SLAM_pred_euler:6' t_ci = x.robot_state.IMU.pos;
  //  in camera frame
  // 'SLAM_pred_euler:7' t_ci = -R_ci' * t_ci;
  for (i48 = 0; i48 < 3; i48++) {
    for (i = 0; i < 3; i++) {
      b_R_ci[i + 3 * i48] = -R_ci[i48 + 3 * i];
    }
  }

  //  in imu frame
  // 'SLAM_pred_euler:8' w_imu = measurements.gyr_duo - x.robot_state.IMU.gyro_bias; 
  for (i = 0; i < 3; i++) {
    t_ci[i] = 0.0;
    for (i48 = 0; i48 < 3; i48++) {
      t_ci[i] += b_R_ci[i + 3 * i48] * x->robot_state.IMU.pos[i48];
    }

    w_imu[i] = measurements_gyr_duo[i] - x->robot_state.IMU.gyro_bias[i];
  }

  //  gyro in IMU frame
  // 'SLAM_pred_euler:9' w_c = R_ci * w_imu;
  //  gyro in camera frame
  //  w = 0*w;
  // 'SLAM_pred_euler:11' a_imu = measurements.acc_duo - x.robot_state.IMU.acc_bias; 
  //  acceleration in IMU frame
  // 'SLAM_pred_euler:12' a_c = R_ci * (a_imu + skew(w_imu)^2 * t_ci);
  // 'skew:2' R=[0,-w(3),w(2);
  // 'skew:3'     w(3),0,-w(1);
  // 'skew:4'     -w(2),w(1),0];
  R[0] = 0.0;
  R[3] = -w_imu[2];
  R[6] = w_imu[1];
  R[1] = w_imu[2];
  R[4] = 0.0;
  R[7] = -w_imu[0];
  R[2] = -w_imu[1];
  R[5] = w_imu[0];
  R[8] = 0.0;
  for (i48 = 0; i48 < 3; i48++) {
    w_c[i48] = 0.0;
    d9 = 0.0;
    for (i = 0; i < 3; i++) {
      w_c[i48] += R_ci[i48 + 3 * i] * w_imu[i];
      b_R[i48 + 3 * i] = 0.0;
      for (i49 = 0; i49 < 3; i49++) {
        b_R[i48 + 3 * i] += R[i48 + 3 * i49] * R[i49 + 3 * i];
      }

      d9 += b_R[i48 + 3 * i] * t_ci[i];
    }

    b_measurements_acc_duo[i48] = (measurements_acc_duo[i48] -
      x->robot_state.IMU.acc_bias[i48]) + d9;
  }

  //  a = 0*a;
  // % compute the linearization F of the non linear model f
  // 'SLAM_pred_euler:17' qv    = processNoise.qv;
  // 'SLAM_pred_euler:18' qw    = processNoise.qw;
  // 'SLAM_pred_euler:19' qwo   = processNoise.qwo;
  // 'SLAM_pred_euler:20' qao   = processNoise.qao;
  // 'SLAM_pred_euler:21' qR_ci = processNoise.qR_ci;
  // 'SLAM_pred_euler:24' Q = diag([qw,qw,qw, qv,qv,qv, qwo,qwo,qwo, 0*qao,qao,0*qao, qR_ci,qR_ci,qR_ci]); 
  // 'SLAM_pred_euler:26' I = eye(3);
  // 'SLAM_pred_euler:27' O = zeros(3);
  // 'SLAM_pred_euler:29' grav_origin = RotFromQuatJ(x.origin.att) * [0; 0; 9.81]; 
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
  // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
  // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
  b_x[0] = ((x->origin.att[0] * x->origin.att[0] - x->origin.att[1] *
             x->origin.att[1]) - x->origin.att[2] * x->origin.att[2]) +
    x->origin.att[3] * x->origin.att[3];
  b_x[3] = 2.0 * (x->origin.att[0] * x->origin.att[1] + x->origin.att[2] *
                  x->origin.att[3]);
  b_x[6] = 2.0 * (x->origin.att[0] * x->origin.att[2] - x->origin.att[1] *
                  x->origin.att[3]);
  b_x[1] = 2.0 * (x->origin.att[0] * x->origin.att[1] - x->origin.att[2] *
                  x->origin.att[3]);
  b_x[4] = ((-(x->origin.att[0] * x->origin.att[0]) + x->origin.att[1] *
             x->origin.att[1]) - x->origin.att[2] * x->origin.att[2]) +
    x->origin.att[3] * x->origin.att[3];
  b_x[7] = 2.0 * (x->origin.att[1] * x->origin.att[2] + x->origin.att[0] *
                  x->origin.att[3]);
  b_x[2] = 2.0 * (x->origin.att[0] * x->origin.att[2] + x->origin.att[1] *
                  x->origin.att[3]);
  b_x[5] = 2.0 * (x->origin.att[1] * x->origin.att[2] - x->origin.att[0] *
                  x->origin.att[3]);
  b_x[8] = ((-(x->origin.att[0] * x->origin.att[0]) - x->origin.att[1] *
             x->origin.att[1]) + x->origin.att[2] * x->origin.att[2]) +
    x->origin.att[3] * x->origin.att[3];

  //  gravity transformed into the origin frame
  //    pos,            rot, vel,                                         gyr_bias,   acc_bias,          origin_att,                                    R_ci 
  // 'SLAM_pred_euler:32' F=[ O,                O,   I,                                                O,           O,                  O,                                       O; 
  // 'SLAM_pred_euler:33'     O,       -skew(w_c),   O,                                               -I,           O,                  O, -skew(R_ci*x.robot_state.IMU.gyro_bias); 
  // 'SLAM_pred_euler:34'     O, -skew(R_cw'*a_c),   O, -skew(skew(w_imu)*t_ci) - skew(w_imu)*skew(t_ci), -R_cw'*R_ci, -skew(grav_origin),                          R_cw*skew(a_c); 
  // 'SLAM_pred_euler:35'     O,                O,   O,                                                O,           O,                  O,                                       O; 
  // 'SLAM_pred_euler:36'     O,                O,   O,                                                O,           O,                  O,                                       O; 
  // 'SLAM_pred_euler:37'     O,                O,   O,                                                O,           O,                  O,                                       O; 
  // 'SLAM_pred_euler:38'     O,                O,   O,                                                O,           O,                  O,                                       O]; 
  // 'skew:2' R=[0,-w(3),w(2);
  // 'skew:3'     w(3),0,-w(1);
  // 'skew:4'     -w(2),w(1),0];
  for (i48 = 0; i48 < 3; i48++) {
    a_c[i48] = 0.0;
    grav_origin[i48] = 0.0;
    y[i48] = 0.0;
    for (i = 0; i < 3; i++) {
      a_c[i48] += R_ci[i48 + 3 * i] * b_measurements_acc_duo[i];
      grav_origin[i48] += b_x[i48 + 3 * i] * b[i];
      y[i48] += R_ci[i48 + 3 * i] * x->robot_state.IMU.gyro_bias[i];
    }
  }

  // 'skew:2' R=[0,-w(3),w(2);
  // 'skew:3'     w(3),0,-w(1);
  // 'skew:4'     -w(2),w(1),0];
  // 'skew:2' R=[0,-w(3),w(2);
  // 'skew:3'     w(3),0,-w(1);
  // 'skew:4'     -w(2),w(1),0];
  // 'skew:2' R=[0,-w(3),w(2);
  // 'skew:3'     w(3),0,-w(1);
  // 'skew:4'     -w(2),w(1),0];
  dv6[0] = 0.0;
  dv6[3] = -w_imu[2];
  dv6[6] = w_imu[1];
  dv6[1] = w_imu[2];
  dv6[4] = 0.0;
  dv6[7] = -w_imu[0];
  dv6[2] = -w_imu[1];
  dv6[5] = w_imu[0];
  dv6[8] = 0.0;
  for (i48 = 0; i48 < 3; i48++) {
    b_y[i48] = 0.0;
    c_y[i48] = 0.0;
    for (i = 0; i < 3; i++) {
      b_y[i48] += R_cw[i + 3 * i48] * a_c[i];
      c_y[i48] += dv6[i48 + 3 * i] * t_ci[i];
    }
  }

  // 'skew:2' R=[0,-w(3),w(2);
  // 'skew:3'     w(3),0,-w(1);
  // 'skew:4'     -w(2),w(1),0];
  // 'skew:2' R=[0,-w(3),w(2);
  // 'skew:3'     w(3),0,-w(1);
  // 'skew:4'     -w(2),w(1),0];
  // 'skew:2' R=[0,-w(3),w(2);
  // 'skew:3'     w(3),0,-w(1);
  // 'skew:4'     -w(2),w(1),0];
  // 'skew:2' R=[0,-w(3),w(2);
  // 'skew:3'     w(3),0,-w(1);
  // 'skew:4'     -w(2),w(1),0];
  // 'skew:2' R=[0,-w(3),w(2);
  // 'skew:3'     w(3),0,-w(1);
  // 'skew:4'     -w(2),w(1),0];
  //  pos
  //  att
  //  vel
  //  gyro bias
  //  acc bias
  //  origin att
  // 'SLAM_pred_euler:40' G=[  O,     O, O, O, O;  % pos
  // 'SLAM_pred_euler:41'     -I,     O, O, O, O;  % att
  // 'SLAM_pred_euler:42'      O,-R_cw', O, O, O;  % vel
  // 'SLAM_pred_euler:43'      O,     O, I, O, O;  % gyro bias
  // 'SLAM_pred_euler:44'      O,     O, O, I, O;  % acc bias
  // 'SLAM_pred_euler:45'      O,     O, O, O, O;  % origin att
  // 'SLAM_pred_euler:46'      O,     O, O, O, I];
  for (i48 = 0; i48 < 15; i48++) {
    for (i = 0; i < 3; i++) {
      G[i + 21 * i48] = 0.0;
      G[(i + 21 * i48) + 3] = iv2[i + 3 * i48];
    }
  }

  for (i48 = 0; i48 < 3; i48++) {
    for (i = 0; i < 3; i++) {
      G[(i + 21 * i48) + 6] = 0.0;
      G[(i + 21 * (i48 + 3)) + 6] = -R_cw[i48 + 3 * i];
      G[(i + 21 * (i48 + 6)) + 6] = 0.0;
      G[(i + 21 * (i48 + 9)) + 6] = 0.0;
      G[(i + 21 * (i48 + 12)) + 6] = 0.0;
    }
  }

  for (i48 = 0; i48 < 15; i48++) {
    for (i = 0; i < 3; i++) {
      G[(i + 21 * i48) + 9] = iv3[i + 3 * i48];
      G[(i + 21 * i48) + 12] = iv4[i + 3 * i48];
      G[(i + 21 * i48) + 15] = 0.0;
      G[(i + 21 * i48) + 18] = iv5[i + 3 * i48];
    }
  }

  //  R_ci
  // 'SLAM_pred_euler:48' Phi = eye(numStates) + F * dt;
  b_eye(P_xx_apr);
  dv7[0] = 0.0;
  dv7[3] = -w_c[2];
  dv7[6] = w_c[1];
  dv7[1] = w_c[2];
  dv7[4] = 0.0;
  dv7[7] = -w_c[0];
  dv7[2] = -w_c[1];
  dv7[5] = w_c[0];
  dv7[8] = 0.0;
  dv8[0] = 0.0;
  dv8[3] = -y[2];
  dv8[6] = y[1];
  dv8[1] = y[2];
  dv8[4] = 0.0;
  dv8[7] = -y[0];
  dv8[2] = -y[1];
  dv8[5] = y[0];
  dv8[8] = 0.0;
  dv9[0] = 0.0;
  dv9[3] = -b_y[2];
  dv9[6] = b_y[1];
  dv9[1] = b_y[2];
  dv9[4] = 0.0;
  dv9[7] = -b_y[0];
  dv9[2] = -b_y[1];
  dv9[5] = b_y[0];
  dv9[8] = 0.0;
  dv10[0] = 0.0;
  dv10[3] = -c_y[2];
  dv10[6] = c_y[1];
  dv10[1] = c_y[2];
  dv10[4] = 0.0;
  dv10[7] = -c_y[0];
  dv10[2] = -c_y[1];
  dv10[5] = c_y[0];
  dv10[8] = 0.0;
  dv11[0] = 0.0;
  dv11[3] = -w_imu[2];
  dv11[6] = w_imu[1];
  dv11[1] = w_imu[2];
  dv11[4] = 0.0;
  dv11[7] = -w_imu[0];
  dv11[2] = -w_imu[1];
  dv11[5] = w_imu[0];
  dv11[8] = 0.0;
  dv12[0] = 0.0;
  dv12[3] = -t_ci[2];
  dv12[6] = t_ci[1];
  dv12[1] = t_ci[2];
  dv12[4] = 0.0;
  dv12[7] = -t_ci[0];
  dv12[2] = -t_ci[1];
  dv12[5] = t_ci[0];
  dv12[8] = 0.0;
  dv13[0] = 0.0;
  dv13[3] = -grav_origin[2];
  dv13[6] = grav_origin[1];
  dv13[1] = grav_origin[2];
  dv13[4] = 0.0;
  dv13[7] = -grav_origin[0];
  dv13[2] = -grav_origin[1];
  dv13[5] = grav_origin[0];
  dv13[8] = 0.0;
  dv14[0] = 0.0;
  dv14[3] = -a_c[2];
  dv14[6] = a_c[1];
  dv14[1] = a_c[2];
  dv14[4] = 0.0;
  dv14[7] = -a_c[0];
  dv14[2] = -a_c[1];
  dv14[5] = a_c[0];
  dv14[8] = 0.0;
  for (i48 = 0; i48 < 3; i48++) {
    for (i = 0; i < 3; i++) {
      R[i + 3 * i48] = -R_cw[i48 + 3 * i];
      d9 = 0.0;
      for (i49 = 0; i49 < 3; i49++) {
        d9 += dv11[i48 + 3 * i49] * dv12[i49 + 3 * i];
      }

      dv6[i48 + 3 * i] = -dv10[i48 + 3 * i] - d9;
    }
  }

  for (i48 = 0; i48 < 3; i48++) {
    for (i = 0; i < 3; i++) {
      b_R_ci[i48 + 3 * i] = 0.0;
      b_R[i48 + 3 * i] = 0.0;
      for (i49 = 0; i49 < 3; i49++) {
        b_R_ci[i48 + 3 * i] += R[i48 + 3 * i49] * R_ci[i49 + 3 * i];
        b_R[i48 + 3 * i] += R_cw[i48 + 3 * i49] * dv14[i49 + 3 * i];
      }
    }
  }

  for (i48 = 0; i48 < 21; i48++) {
    for (i = 0; i < 3; i++) {
      Phi[i + 21 * i48] = iv6[i + 3 * i48];
    }
  }

  for (i48 = 0; i48 < 3; i48++) {
    for (i = 0; i < 3; i++) {
      Phi[(i + 21 * i48) + 3] = 0.0;
      Phi[(i + 21 * (i48 + 3)) + 3] = -dv7[i + 3 * i48];
      Phi[(i + 21 * (i48 + 6)) + 3] = 0.0;
      Phi[(i + 21 * (i48 + 9)) + 3] = iv7[i + 3 * i48];
      Phi[(i + 21 * (i48 + 12)) + 3] = 0.0;
      Phi[(i + 21 * (i48 + 15)) + 3] = 0.0;
      Phi[(i + 21 * (i48 + 18)) + 3] = -dv8[i + 3 * i48];
      Phi[(i + 21 * i48) + 6] = 0.0;
      Phi[(i + 21 * (i48 + 3)) + 6] = -dv9[i + 3 * i48];
      Phi[(i + 21 * (i48 + 6)) + 6] = 0.0;
      Phi[(i + 21 * (i48 + 9)) + 6] = dv6[i + 3 * i48];
      Phi[(i + 21 * (i48 + 12)) + 6] = b_R_ci[i + 3 * i48];
      Phi[(i + 21 * (i48 + 15)) + 6] = -dv13[i + 3 * i48];
      Phi[(i + 21 * (i48 + 18)) + 6] = b_R[i + 3 * i48];
    }
  }

  for (i48 = 0; i48 < 21; i48++) {
    for (i = 0; i < 3; i++) {
      Phi[(i + 21 * i48) + 9] = 0.0;
      Phi[(i + 21 * i48) + 12] = 0.0;
      Phi[(i + 21 * i48) + 15] = 0.0;
      Phi[(i + 21 * i48) + 18] = 0.0;
    }

    for (i = 0; i < 21; i++) {
      b_Phi[i + 21 * i48] = P_xx_apr[i + 21 * i48] + Phi[i + 21 * i48] * dt;
    }
  }

  // 'SLAM_pred_euler:50' states_idx = int32(1:numStates);
  // 'SLAM_pred_euler:52' P_xx_apr = Phi*P_apo(states_idx, states_idx)*Phi'  + G*Q*G'*dt; 
  b_processNoise_qw[0] = processNoise_qw;
  b_processNoise_qw[1] = processNoise_qw;
  b_processNoise_qw[2] = processNoise_qw;
  b_processNoise_qw[3] = processNoise_qv;
  b_processNoise_qw[4] = processNoise_qv;
  b_processNoise_qw[5] = processNoise_qv;
  b_processNoise_qw[6] = processNoise_qwo;
  b_processNoise_qw[7] = processNoise_qwo;
  b_processNoise_qw[8] = processNoise_qwo;
  b_processNoise_qw[9] = 0.0 * processNoise_qao;
  b_processNoise_qw[10] = processNoise_qao;
  b_processNoise_qw[11] = 0.0 * processNoise_qao;
  b_processNoise_qw[12] = processNoise_qR_ci;
  b_processNoise_qw[13] = processNoise_qR_ci;
  b_processNoise_qw[14] = processNoise_qR_ci;
  b_diag(b_processNoise_qw, dv15);

  //  covariance of the state
  // 'SLAM_pred_euler:53' P_xx_apr = (P_xx_apr + P_xx_apr')/2;
  // 'SLAM_pred_euler:54' P_xs_apr = Phi*P_apo(states_idx, int32(numStates+1:int32(end))); 
  for (i48 = 0; i48 < 21; i48++) {
    for (i = 0; i < 21; i++) {
      Phi[i48 + 21 * i] = 0.0;
      for (i49 = 0; i49 < 21; i49++) {
        Phi[i48 + 21 * i] += b_Phi[i48 + 21 * i49] * P_apo[i49 + 91 * i];
      }
    }

    for (i = 0; i < 15; i++) {
      b_G[i48 + 21 * i] = 0.0;
      for (i49 = 0; i49 < 15; i49++) {
        b_G[i48 + 21 * i] += G[i48 + 21 * i49] * dv15[i49 + 15 * i];
      }
    }

    for (i = 0; i < 21; i++) {
      c_G[i48 + 21 * i] = 0.0;
      for (i49 = 0; i49 < 15; i49++) {
        c_G[i48 + 21 * i] += b_G[i48 + 21 * i49] * G[i + 21 * i49];
      }

      d9 = 0.0;
      for (i49 = 0; i49 < 21; i49++) {
        d9 += Phi[i48 + 21 * i49] * b_Phi[i + 21 * i49];
      }

      P_xx_apr[i48 + 21 * i] = d9 + c_G[i48 + 21 * i] * dt;
    }

    for (i = 0; i < 70; i++) {
      P_xs_apr[i48 + 21 * i] = 0.0;
      for (i49 = 0; i49 < 21; i49++) {
        P_xs_apr[i48 + 21 * i] += b_Phi[i48 + 21 * i49] * P_apo[i49 + 91 * (21 +
          i)];
      }
    }
  }

  //  covariance between current state and trails
  // 'SLAM_pred_euler:56' P_apr = P_apo;
  // 'SLAM_pred_euler:57' P_apr(states_idx, states_idx) = P_xx_apr;
  for (i48 = 0; i48 < 21; i48++) {
    for (i = 0; i < 21; i++) {
      P_apo[i + 91 * i48] = (P_xx_apr[i + 21 * i48] + P_xx_apr[i48 + 21 * i]) /
        2.0;
    }
  }

  // 'SLAM_pred_euler:58' P_apr(states_idx, int32(numStates+1:int32(end))) = P_xs_apr; 
  for (i48 = 0; i48 < 70; i48++) {
    memcpy(&P_apo[i48 * 91 + 1911], &P_xs_apr[i48 * 21], 21U * sizeof(double));
  }

  // 'SLAM_pred_euler:59' P_apr(int32(numStates+1:int32(end)), states_idx) = P_xs_apr'; 
  for (i48 = 0; i48 < 21; i48++) {
    for (i = 0; i < 70; i++) {
      P_apo[(i + 91 * i48) + 21] = P_xs_apr[i48 + 21 * i];
    }
  }

  // 'SLAM_pred_euler:61' x.robot_state.pos = x.robot_state.pos + x.robot_state.vel*dt; 
  // 'SLAM_pred_euler:62' dq = quatPlusThetaJ(w_c*dt);
  // 'SLAM_pred_euler:63' x.robot_state.att = quatmultJ(dq, x.robot_state.att);
  for (i = 0; i < 3; i++) {
    x->robot_state.pos[i] += x->robot_state.vel[i] * dt;
    b_measurements_acc_duo[i] = w_c[i] * dt;
  }

  for (i = 0; i < 4; i++) {
    c_x[i] = x->robot_state.att[i];
  }

  quatPlusThetaJ(b_measurements_acc_duo, dv16);
  quatmultJ(dv16, c_x, x->robot_state.att);

  // 'SLAM_pred_euler:64' x.robot_state.vel = x.robot_state.vel + (R_cw'*a_c - grav_origin)*dt; 
  for (i48 = 0; i48 < 3; i48++) {
    d9 = 0.0;
    for (i = 0; i < 3; i++) {
      d9 += R_cw[i + 3 * i48] * a_c[i];
    }

    b_measurements_acc_duo[i48] = d9 - grav_origin[i48];
    x->robot_state.vel[i48] += b_measurements_acc_duo[i48] * dt;
  }

  //  velocity
  //  P_apr = (P_apr+P_apr')/2;
}

//
// % Iterative Camera Pose optimization (EKF)
// Arguments    : double P_apr[8281]
//                g_struct_T *b_xt
//                double c_cameraParams_CameraParameters
//                const double d_cameraParams_CameraParameters[2]
//                const double e_cameraParams_CameraParameters[2]
//                const double f_cameraParams_CameraParameters[3]
//                double g_cameraParams_CameraParameters
//                double h_cameraParams_CameraParameters
//                const double i_cameraParams_CameraParameters[2]
//                const double j_cameraParams_CameraParameters[2]
//                const double k_cameraParams_CameraParameters[3]
//                double l_cameraParams_CameraParameters
//                const double cameraParams_r_lr[3]
//                const double cameraParams_R_lr[9]
//                const double cameraParams_R_rl[9]
//                int updateVect[40]
//                double z_all_l[80]
//                double z_all_r[80]
//                double noiseParameters_image_noise
//                double c_noiseParameters_inv_depth_ini
//                const VIOParameters b_VIOParameters
//                double h_u_apo[160]
//                double b_map[120]
//                double b_delayedStatus[40]
// Return Type  : void
//
static void SLAM_upd(double P_apr[8281], g_struct_T *b_xt, double
                     c_cameraParams_CameraParameters, const double
                     d_cameraParams_CameraParameters[2], const double
                     e_cameraParams_CameraParameters[2], const double
                     f_cameraParams_CameraParameters[3], double
                     g_cameraParams_CameraParameters, double
                     h_cameraParams_CameraParameters, const double
                     i_cameraParams_CameraParameters[2], const double
                     j_cameraParams_CameraParameters[2], const double
                     k_cameraParams_CameraParameters[3], double
                     l_cameraParams_CameraParameters, const double
                     cameraParams_r_lr[3], const double cameraParams_R_lr[9],
                     const double cameraParams_R_rl[9], int updateVect[40],
                     double z_all_l[80], double z_all_r[80], double
                     noiseParameters_image_noise, double
                     c_noiseParameters_inv_depth_ini, const VIOParameters
                     b_VIOParameters, double h_u_apo[160], double b_map[120],
                     double b_delayedStatus[40])
{
  boolean_T x[40];
  int i;
  int idx;
  int ii_data[40];
  int ixstart;
  boolean_T exitg7;
  boolean_T guard3 = false;
  int loop_ub;
  double b_ii_data[40];
  int ii_size[1];
  int i50;
  int ind_l2_size[1];
  double ind_l2_data[80];
  double z_all_l_data[80];
  int z_all_l_size[1];
  double status_ind_data[80];
  boolean_T exitg6;
  boolean_T guard2 = false;
  int ii_size_idx_0;
  int ind_r_size[1];
  double ind_r_data[40];
  int z_all_r_size[1];
  int anchorIdx;
  int featureIdx;
  boolean_T fix_new_feature;
  double uncertainties[8];
  signed char active_feature[8];
  int iidx[8];
  emxArray_real_T *qualities;
  emxArray_real_T *anchorInd;
  emxArray_real_T *featureInd;
  emxArray_int32_T *b_iidx;
  double new_m_data[240];
  boolean_T triangulation_success_data[40];
  double z_curr_l[2];
  double z_curr_r[2];
  boolean_T success;
  double b_m[6];
  double new_origin_pos_rel[3];
  double m_l[3];
  boolean_T bv0[3];
  signed char i51;
  double h_u_r[2];
  double h_u_l[2];
  double b_h_u_l[2];
  boolean_T guard1 = false;
  double mtmp;
  int trueCount;
  double triangulated_depths_data[40];
  signed char triangulated_status_ind_data[40];
  signed char b_triangulated_status_ind_data[40];
  int c_triangulated_status_ind_size_;
  int ix;
  double untriangulated_depths_data[40];
  emxArray_real_T *untriangulated_idx;
  int tmp_data[40];
  signed char untriangulated_status_ind_data[40];
  double b_new_m_data[120];
  double c_new_m_data[120];
  int new_feature_idx;
  boolean_T exitg4;
  long i52;
  double b_P_apr[36];
  double P_apr_data[462];
  double b_P_apr_data[546];
  boolean_T exitg5;
  double b_uncertainties[40];
  boolean_T has_active_features;
  int uncertainties_size[1];
  double median_uncertainty;
  double totalNumActiveFeatues;
  double numDelayedFeatures;
  unsigned int delayedIdx;
  double numActivatedFeatures;
  boolean_T request_new_features;
  boolean_T exitg3;
  int request_idx;
  boolean_T exitg2;
  boolean_T b_guard1 = false;
  double c_uncertainties[5];
  double b_has_active_features[5];
  boolean_T exitg1;
  char cv50[63];
  static const char cv51[63] = { 'P', 'i', 'c', 'k', 'e', 'd', ' ', 'a', 'n',
    ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', 'w', 'i', 't', 'h', ' ', 'n', 'o',
    ' ', 'a', 'c', 't', 'i', 'v', 'e', ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e',
    's', ' ', 'a', 's', ' ', 'o', 'r', 'i', 'g', 'i', 'n', ' ', '(', 'a', 'n',
    'c', 'h', 'o', 'r', ' ', '%', 'd', ')', '\x00' };

  double new_origin_att_rel[9];
  static double J[8281];
  double c_xt[9];
  double d_xt[3];
  double e_xt[9];
  static double b_J[8281];
  double f_xt[9];
  double g_xt[9];
  double h_xt[9];

  // 'SLAM_upd:4' measDim = 4;
  // 'SLAM_upd:6' numStatesPerAnchor = 6 + numPointsPerAnchor;
  // 'SLAM_upd:7' numTrackFeatures = numAnchors*numPointsPerAnchor;
  //  undistort all valid points
  // 'SLAM_upd:10' ind_l = find(updateVect ~=0);
  for (i = 0; i < 40; i++) {
    x[i] = (updateVect[i] != 0);
  }

  idx = 0;
  ixstart = 1;
  exitg7 = false;
  while ((!exitg7) && (ixstart < 41)) {
    guard3 = false;
    if (x[ixstart - 1]) {
      idx++;
      ii_data[idx - 1] = ixstart;
      if (idx >= 40) {
        exitg7 = true;
      } else {
        guard3 = true;
      }
    } else {
      guard3 = true;
    }

    if (guard3) {
      ixstart++;
    }
  }

  if (1 > idx) {
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }

  // 'SLAM_upd:11' ind_l2 = multiplyIdx(ind_l, 2);
  ii_size[0] = loop_ub;
  for (i50 = 0; i50 < loop_ub; i50++) {
    b_ii_data[i50] = ii_data[i50];
  }

  multiplyIdx(b_ii_data, ii_size, ind_l2_data, ind_l2_size);

  // 'SLAM_upd:12' z_all_l(ind_l2) = undistortPoint(z_all_l(ind_l2), cameraParams.CameraParameters1); 
  z_all_l_size[0] = ind_l2_size[0];
  loop_ub = ind_l2_size[0];
  for (i50 = 0; i50 < loop_ub; i50++) {
    z_all_l_data[i50] = z_all_l[(int)ind_l2_data[i50] - 1];
  }

  undistortPoint(z_all_l_data, z_all_l_size, c_cameraParams_CameraParameters,
                 d_cameraParams_CameraParameters,
                 e_cameraParams_CameraParameters,
                 f_cameraParams_CameraParameters,
                 g_cameraParams_CameraParameters, status_ind_data, ii_size);
  loop_ub = ii_size[0];
  for (i50 = 0; i50 < loop_ub; i50++) {
    z_all_l[(int)ind_l2_data[i50] - 1] = status_ind_data[i50];
  }

  // 'SLAM_upd:13' ind_r = find(updateVect == 2);
  for (i = 0; i < 40; i++) {
    x[i] = (updateVect[i] == 2);
  }

  idx = 0;
  ixstart = 1;
  exitg6 = false;
  while ((!exitg6) && (ixstart < 41)) {
    guard2 = false;
    if (x[ixstart - 1]) {
      idx++;
      ii_data[idx - 1] = ixstart;
      if (idx >= 40) {
        exitg6 = true;
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }

    if (guard2) {
      ixstart++;
    }
  }

  if (1 > idx) {
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }

  if (1 > idx) {
    ii_size_idx_0 = 0;
  } else {
    ii_size_idx_0 = idx;
  }

  ind_r_size[0] = loop_ub;
  for (i50 = 0; i50 < loop_ub; i50++) {
    ind_r_data[i50] = ii_data[i50];
  }

  // 'SLAM_upd:14' ind_r2 = multiplyIdx(ind_r, 2);
  multiplyIdx(ind_r_data, ind_r_size, ind_l2_data, ind_l2_size);

  // 'SLAM_upd:15' z_all_r(ind_r2) = undistortPoint(z_all_r(ind_r2), cameraParams.CameraParameters2); 
  z_all_r_size[0] = ind_l2_size[0];
  idx = ind_l2_size[0];
  for (i50 = 0; i50 < idx; i50++) {
    z_all_l_data[i50] = z_all_r[(int)ind_l2_data[i50] - 1];
  }

  undistortPoint(z_all_l_data, z_all_r_size, h_cameraParams_CameraParameters,
                 i_cameraParams_CameraParameters,
                 j_cameraParams_CameraParameters,
                 k_cameraParams_CameraParameters,
                 l_cameraParams_CameraParameters, status_ind_data, ii_size);
  idx = ii_size[0];
  for (i50 = 0; i50 < idx; i50++) {
    z_all_r[(int)ind_l2_data[i50] - 1] = status_ind_data[i50];
  }

  //  check for lost features
  // 'SLAM_upd:18' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    // 'SLAM_upd:19' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      // 'SLAM_upd:20' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status 
      if ((b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status != 0)
          && (updateVect[b_xt->anchor_states[anchorIdx]
              .feature_states[featureIdx].status_idx - 1] != 1)) {
        // 'SLAM_upd:21' idx = xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx; 
        // 'SLAM_upd:22' if updateVect(idx) ~= 1
        // 'SLAM_upd:23' P_apr(xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx, :) = 0; 
        idx = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx;
        for (i50 = 0; i50 < 91; i50++) {
          P_apr[(idx + 91 * i50) - 1] = 0.0;
        }

        // 'SLAM_upd:24' P_apr(:, xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx) = 0; 
        idx = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx;
        for (i50 = 0; i50 < 91; i50++) {
          P_apr[i50 + 91 * (idx - 1)] = 0.0;
        }

        // 'SLAM_upd:25' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
        b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status = 0;

        // 'SLAM_upd:26' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
        b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status_idx = 0;

        //                  ros_info('Lost feature %i, which was %i on anchor %i', idx, featureIdx, anchorIdx) 
      }
    }
  }

  // 'SLAM_upd:33' if VIOParameters.fixed_feature
  if (b_VIOParameters.fixed_feature) {
    // 'SLAM_upd:34' fix_new_feature = false;
    fix_new_feature = false;

    // 'SLAM_upd:36' if xt.origin.anchor_idx
    if (b_xt->origin.anchor_idx != 0) {
      // 'SLAM_upd:37' if xt.fixed_feature
      if (b_xt->fixed_feature != 0) {
        // 'SLAM_upd:38' if xt.anchor_states(xt.origin.anchor_idx).feature_states(xt.fixed_feature).status ~= 1 
        if (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].feature_states
            [b_xt->fixed_feature - 1].status != 1) {
          // 'SLAM_upd:39' fix_new_feature = true;
          fix_new_feature = true;

          // 'SLAM_upd:40' ros_info('Fixed feature %i (%i on anchor %i) is no longer valid', xt.anchor_states(xt.origin.anchor_idx).feature_states(xt.fixed_feature).status_idx, xt.fixed_feature, xt.origin.anchor_idx) 
          ros_info(b_xt->anchor_states[b_xt->origin.anchor_idx - 1].
                   feature_states[b_xt->fixed_feature - 1].status_idx,
                   b_xt->fixed_feature, b_xt->origin.anchor_idx);
        }
      } else {
        // 'SLAM_upd:42' else
        // 'SLAM_upd:43' fix_new_feature = true;
        fix_new_feature = true;
      }
    }

    // 'SLAM_upd:47' if fix_new_feature
    if (fix_new_feature) {
      // 'SLAM_upd:48' uncertainties = zeros(numPointsPerAnchor, 1);
      // 'SLAM_upd:49' active_feature = uncertainties;
      // 'SLAM_upd:50' for featureIdx = 1:numPointsPerAnchor
      for (featureIdx = 0; featureIdx < 8; featureIdx++) {
        active_feature[featureIdx] = 0;

        // 'SLAM_upd:51' if xt.anchor_states(xt.origin.anchor_idx).feature_states(featureIdx).status == 1 
        if (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
            .feature_states[featureIdx].status == 1) {
          // 'SLAM_upd:52' uncertainties(featureIdx) = P_apr(xt.anchor_states(xt.origin.anchor_idx).feature_states(featureIdx).P_idx, xt.anchor_states(xt.origin.anchor_idx).feature_states(featureIdx).P_idx); 
          uncertainties[featureIdx] = P_apr[(b_xt->anchor_states
            [b_xt->origin.anchor_idx - 1].feature_states[featureIdx].P_idx + 91 *
            (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
             .feature_states[featureIdx].P_idx - 1)) - 1];

          // 'SLAM_upd:53' active_feature(featureIdx) = 1;
          active_feature[featureIdx] = 1;
        } else {
          // 'SLAM_upd:54' else
          // 'SLAM_upd:55' uncertainties(featureIdx) = 1000;
          uncertainties[featureIdx] = 1000.0;

          //  dont fix an inactive feature
        }
      }

      // 'SLAM_upd:58' [~, sortIdx] = sort(uncertainties, 'ascend');
      sort(uncertainties, iidx);
      for (i = 0; i < 8; i++) {
        uncertainties[i] = iidx[i];
      }

      // 'SLAM_upd:58' ~
      // 'SLAM_upd:59' if ~active_feature(sortIdx(1))
      if (!(active_feature[(int)uncertainties[0] - 1] != 0)) {
        // 'SLAM_upd:60' ros_error('picked an inactive feature')
        ros_error();
      }

      // 'SLAM_upd:62' xt.fixed_feature = int32(sortIdx(1));
      b_xt->fixed_feature = (int)uncertainties[0];

      // 'SLAM_upd:63' P_apr(xt.anchor_states(xt.origin.anchor_idx).feature_states(xt.fixed_feature).P_idx, :) = 0; 
      idx = b_xt->anchor_states[b_xt->origin.anchor_idx - 1].feature_states
        [b_xt->fixed_feature - 1].P_idx;
      for (i50 = 0; i50 < 91; i50++) {
        P_apr[(idx + 91 * i50) - 1] = 0.0;
      }

      //  fix the feature depth
      // 'SLAM_upd:64' P_apr(:, xt.anchor_states(xt.origin.anchor_idx).feature_states(xt.fixed_feature).P_idx) = 0; 
      idx = b_xt->anchor_states[b_xt->origin.anchor_idx - 1].feature_states
        [b_xt->fixed_feature - 1].P_idx;
      for (i50 = 0; i50 < 91; i50++) {
        P_apr[i50 + 91 * (idx - 1)] = 0.0;
      }

      // 'SLAM_upd:65' ros_info('Fixing feature %i (feature %i on anchor %i)', xt.anchor_states(xt.origin.anchor_idx).feature_states(xt.fixed_feature).status_idx, xt.fixed_feature, xt.origin.anchor_idx) 
      b_ros_info(b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
                 .feature_states[b_xt->fixed_feature - 1].status_idx,
                 b_xt->fixed_feature, b_xt->origin.anchor_idx);
    }
  }

  // % do the update
  // 'SLAM_upd:70' if any(updateVect == 1)
  for (i = 0; i < 40; i++) {
    x[i] = (updateVect[i] == 1);
  }

  if (any(x)) {
    // 'SLAM_upd:71' [ xt, P_apo, updateVect ] = OnePointRANSAC_EKF(xt, P_apr, z_all_l, cameraParams.CameraParameters1, noiseParameters, VIOParameters, updateVect); 
    OnePointRANSAC_EKF(b_xt, P_apr, z_all_l, d_cameraParams_CameraParameters,
                       e_cameraParams_CameraParameters,
                       noiseParameters_image_noise,
                       b_VIOParameters.max_ekf_iterations,
                       b_VIOParameters.delayed_initialization,
                       b_VIOParameters.RANSAC, updateVect);
  } else {
    // 'SLAM_upd:72' else
    // 'SLAM_upd:73' P_apo = P_apr;
  }

  // % Initialize new anchors/features
  // 'SLAM_upd:77' if length(ind_r) >= minFeatureThreshold
  emxInit_real_T(&qualities, 1);
  emxInit_real_T(&anchorInd, 1);
  emxInit_real_T(&featureInd, 1);
  emxInit_int32_T1(&b_iidx, 1);
  if (ii_size_idx_0 >= 4) {
    //  try to triangulate all new features
    // 'SLAM_upd:79' new_depths = zeros(length(ind_r), 1);
    for (i50 = 0; i50 < ii_size_idx_0; i50++) {
      ind_l2_data[i50] = 0.0;
    }

    // 'SLAM_upd:80' new_m = zeros(3, length(ind_r));
    idx = 3 * ii_size_idx_0;
    for (i50 = 0; i50 < idx; i50++) {
      new_m_data[i50] = 0.0;
    }

    // 'SLAM_upd:81' triangulation_success = false(length(ind_r), 1);
    for (i50 = 0; i50 < ii_size_idx_0; i50++) {
      triangulation_success_data[i50] = false;
    }

    // 'SLAM_upd:83' for i = 1:length(ind_r)
    for (i = 0; i < loop_ub; i++) {
      // 'SLAM_upd:84' z_curr_l = z_all_l((ind_r(i)-1)*2 + (1:2));
      idx = ((int)ind_r_data[i] - 1) * 2;

      // 'SLAM_upd:85' z_curr_r = z_all_r((ind_r(i)-1)*2 + (1:2));
      ixstart = ((int)ind_r_data[i] - 1) * 2;
      for (i50 = 0; i50 < 2; i50++) {
        z_curr_l[i50] = z_all_l[idx + i50];
        z_curr_r[i50] = z_all_r[ixstart + i50];
      }

      // 'SLAM_upd:86' if ~VIOParameters.mono
      if (!b_VIOParameters.mono) {
        // 'SLAM_upd:87' [ fp, m, success ] = initializePoint(z_curr_l, z_curr_r, cameraParams); 
        initializePoint(z_curr_l, z_curr_r, d_cameraParams_CameraParameters,
                        e_cameraParams_CameraParameters,
                        i_cameraParams_CameraParameters,
                        j_cameraParams_CameraParameters, cameraParams_r_lr,
                        cameraParams_R_lr, new_origin_pos_rel, b_m, &success);

        // 'SLAM_upd:88' m_l = m(:,1);
        for (i50 = 0; i50 < 3; i50++) {
          m_l[i50] = b_m[i50];
        }

        // 'SLAM_upd:90' if success
        if (success) {
          //  perform further checks
          // 'SLAM_upd:91' if any(isnan(fp))
          for (idx = 0; idx < 3; idx++) {
            bv0[idx] = rtIsNaN(new_origin_pos_rel[idx]);
          }

          if (b_any(bv0)) {
            // 'SLAM_upd:92' ros_warn('Bad triangulation (nan) for point %d', int8(ind_r(i))); 
            i50 = (int)rt_roundd_snf(ind_r_data[i]);
            i51 = (signed char)i50;
            e_ros_warn(i51);

            // 'SLAM_upd:93' fp = m_l;
            for (i50 = 0; i50 < 3; i50++) {
              new_origin_pos_rel[i50] = b_m[i50];
            }

            // 'SLAM_upd:94' success = false;
            success = false;
          } else {
            // 'SLAM_upd:95' else
            //  check reprojection error
            // 'SLAM_upd:97' [h_u_l, h_u_r] = predictMeasurementStereo(fp, cameraParams); 
            predictMeasurementStereo(new_origin_pos_rel,
              d_cameraParams_CameraParameters, e_cameraParams_CameraParameters,
              i_cameraParams_CameraParameters, j_cameraParams_CameraParameters,
              cameraParams_r_lr, cameraParams_R_rl, h_u_l, h_u_r);

            // 'SLAM_upd:99' reprojection_error_thresh = 5;
            // 'SLAM_upd:100' if norm(h_u_l - z_curr_l) > reprojection_error_thresh || norm(h_u_r - z_curr_r) > reprojection_error_thresh 
            for (idx = 0; idx < 2; idx++) {
              b_h_u_l[idx] = h_u_l[idx] - z_curr_l[idx];
            }

            guard1 = false;
            if (c_norm(b_h_u_l) > 5.0) {
              guard1 = true;
            } else {
              for (idx = 0; idx < 2; idx++) {
                b_h_u_l[idx] = h_u_r[idx] - z_curr_r[idx];
              }

              if (c_norm(b_h_u_l) > 5.0) {
                guard1 = true;
              } else {
                // 'SLAM_upd:104' else
                // 'SLAM_upd:105' if norm(fp) < 0.1
                if (norm(new_origin_pos_rel) < 0.1) {
                  //  feature triangulated very close
                  // 'SLAM_upd:106' fp = m_l;
                  for (i50 = 0; i50 < 3; i50++) {
                    new_origin_pos_rel[i50] = b_m[i50];
                  }

                  // 'SLAM_upd:107' success = false;
                  success = false;

                  // 'SLAM_upd:108' ros_warn('Feature %i is triangulated very close. Depth: %f', int32(ind_r(i)), norm(fp)); 
                  f_ros_warn((int)ind_r_data[i], norm(*(double (*)[3])&b_m[0]));
                }
              }
            }

            if (guard1) {
              //                          ros_info('Bad triangulation (reprojection error) for point %d', int8(ind_r(i))); 
              // 'SLAM_upd:102' fp = m_l;
              for (i50 = 0; i50 < 3; i50++) {
                new_origin_pos_rel[i50] = b_m[i50];
              }

              // 'SLAM_upd:103' success = false;
              success = false;
            }
          }
        } else {
          // 'SLAM_upd:112' else
          // 'SLAM_upd:113' fp = m_l;
          for (i50 = 0; i50 < 3; i50++) {
            new_origin_pos_rel[i50] = b_m[i50];
          }
        }
      } else {
        // 'SLAM_upd:115' else
        //  mono
        // 'SLAM_upd:116' z_n_l = [(z_curr_l(1) - cameraParams.CameraParameters1.PrincipalPoint(1))/cameraParams.CameraParameters1.FocalLength(1); 
        // 'SLAM_upd:117'                 (z_curr_l(2) - cameraParams.CameraParameters1.PrincipalPoint(2))/cameraParams.CameraParameters1.FocalLength(2)]; 
        // 'SLAM_upd:118' m_l = [z_n_l; 1];
        m_l[0] = (z_all_l[((int)ind_r_data[i] - 1) * 2] -
                  e_cameraParams_CameraParameters[0]) /
          d_cameraParams_CameraParameters[0];
        m_l[1] = (z_all_l[((int)ind_r_data[i] - 1) * 2 + 1] -
                  e_cameraParams_CameraParameters[1]) /
          d_cameraParams_CameraParameters[1];
        m_l[2] = 1.0;

        // 'SLAM_upd:119' fp = m_l/norm(m_l);
        mtmp = norm(m_l);
        for (idx = 0; idx < 3; idx++) {
          new_origin_pos_rel[idx] = m_l[idx] / mtmp;
        }

        // 'SLAM_upd:120' success = true;
        success = true;
      }

      // 'SLAM_upd:122' new_depths(i) = norm(fp);
      ind_l2_data[i] = norm(new_origin_pos_rel);

      // 'SLAM_upd:123' new_m(:, i) = m_l;
      for (i50 = 0; i50 < 3; i50++) {
        new_m_data[i50 + 3 * i] = m_l[i50];
      }

      // 'SLAM_upd:124' triangulation_success(i) = success;
      triangulation_success_data[i] = success;
    }

    // 'SLAM_upd:127' ros_info('Successfully triangulated %d of %d features', int32(nnz(triangulation_success)), int32(length(triangulation_success))) 
    idx = 0;
    for (ixstart = 0; ixstart < ii_size_idx_0; ixstart++) {
      if (triangulation_success_data[ixstart]) {
        idx++;
      }
    }

    c_ros_info(idx, loop_ub);

    // 'SLAM_upd:129' triangulated_depths = new_depths(triangulation_success);
    idx = loop_ub - 1;
    trueCount = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        trueCount++;
      }
    }

    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        triangulated_depths_data[ixstart] = ind_l2_data[i];
        ixstart++;
      }
    }

    // 'SLAM_upd:130' [triangulated_depths, triangulated_idx] = sort(triangulated_depths, 'ascend'); 
    i50 = qualities->size[0];
    qualities->size[0] = trueCount;
    emxEnsureCapacity((emxArray__common *)qualities, i50, (int)sizeof(double));
    for (i50 = 0; i50 < trueCount; i50++) {
      qualities->data[i50] = triangulated_depths_data[i50];
    }

    c_sort(qualities, b_iidx);
    i50 = anchorInd->size[0];
    anchorInd->size[0] = b_iidx->size[0];
    emxEnsureCapacity((emxArray__common *)anchorInd, i50, (int)sizeof(double));
    idx = b_iidx->size[0];
    for (i50 = 0; i50 < idx; i50++) {
      anchorInd->data[i50] = b_iidx->data[i50];
    }

    // 'SLAM_upd:131' triangulated_m = new_m(:, triangulation_success);
    idx = loop_ub - 1;
    trueCount = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        trueCount++;
      }
    }

    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        ii_data[ixstart] = i + 1;
        ixstart++;
      }
    }

    // 'SLAM_upd:132' triangulated_m = triangulated_m(:, triangulated_idx);
    // 'SLAM_upd:133' triangulated_status_ind = ind_r(triangulation_success);
    idx = loop_ub - 1;
    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        triangulated_status_ind_data[ixstart] = (signed char)ind_r_data[i];
        ixstart++;
      }
    }

    // 'SLAM_upd:134' triangulated_status_ind = triangulated_status_ind(triangulated_idx); 
    c_triangulated_status_ind_size_ = anchorInd->size[0];
    idx = anchorInd->size[0];
    for (i50 = 0; i50 < idx; i50++) {
      b_triangulated_status_ind_data[i50] = triangulated_status_ind_data[(int)
        anchorInd->data[i50] - 1];
    }

    for (i50 = 0; i50 < c_triangulated_status_ind_size_; i50++) {
      triangulated_status_ind_data[i50] = b_triangulated_status_ind_data[i50];
    }

    // 'SLAM_upd:136' untriangulated_depths = new_depths(~triangulation_success); 
    idx = loop_ub - 1;
    ix = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        ix++;
      }
    }

    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        untriangulated_depths_data[ixstart] = ind_l2_data[i];
        ixstart++;
      }
    }

    // 'SLAM_upd:137' [untriangulated_depths, untriangulated_idx] = sort(untriangulated_depths, 'ascend'); 
    i50 = featureInd->size[0];
    featureInd->size[0] = ix;
    emxEnsureCapacity((emxArray__common *)featureInd, i50, (int)sizeof(double));
    for (i50 = 0; i50 < ix; i50++) {
      featureInd->data[i50] = untriangulated_depths_data[i50];
    }

    emxInit_real_T(&untriangulated_idx, 1);
    c_sort(featureInd, b_iidx);
    i50 = untriangulated_idx->size[0];
    untriangulated_idx->size[0] = b_iidx->size[0];
    emxEnsureCapacity((emxArray__common *)untriangulated_idx, i50, (int)sizeof
                      (double));
    idx = b_iidx->size[0];
    for (i50 = 0; i50 < idx; i50++) {
      untriangulated_idx->data[i50] = b_iidx->data[i50];
    }

    // 'SLAM_upd:138' untriangulated_m = new_m(:, ~triangulation_success);
    idx = loop_ub - 1;
    ix = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        ix++;
      }
    }

    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        tmp_data[ixstart] = i + 1;
        ixstart++;
      }
    }

    // 'SLAM_upd:139' untriangulated_m = untriangulated_m(:, untriangulated_idx); 
    // 'SLAM_upd:140' untriangulated_status_ind = ind_r(~triangulation_success); 
    idx = loop_ub - 1;
    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        untriangulated_status_ind_data[ixstart] = (signed char)ind_r_data[i];
        ixstart++;
      }
    }

    // 'SLAM_upd:141' untriangulated_status_ind = untriangulated_status_ind(untriangulated_idx); 
    idx = untriangulated_idx->size[0];
    loop_ub = untriangulated_idx->size[0];
    for (i50 = 0; i50 < loop_ub; i50++) {
      b_triangulated_status_ind_data[i50] = untriangulated_status_ind_data[(int)
        untriangulated_idx->data[i50] - 1];
    }

    for (i50 = 0; i50 < idx; i50++) {
      untriangulated_status_ind_data[i50] = b_triangulated_status_ind_data[i50];
    }

    // 'SLAM_upd:143' new_depths = [triangulated_depths; untriangulated_depths]; 
    ind_l2_size[0] = qualities->size[0] + featureInd->size[0];
    loop_ub = qualities->size[0];
    for (i50 = 0; i50 < loop_ub; i50++) {
      ind_l2_data[i50] = qualities->data[i50];
    }

    loop_ub = featureInd->size[0];
    for (i50 = 0; i50 < loop_ub; i50++) {
      ind_l2_data[i50 + qualities->size[0]] = featureInd->data[i50];
    }

    // 'SLAM_upd:144' new_m = [triangulated_m, untriangulated_m];
    for (i50 = 0; i50 < trueCount; i50++) {
      for (i = 0; i < 3; i++) {
        b_new_m_data[i + 3 * i50] = new_m_data[i + 3 * (ii_data[i50] - 1)];
      }
    }

    for (i50 = 0; i50 < ix; i50++) {
      for (i = 0; i < 3; i++) {
        c_new_m_data[i + 3 * i50] = new_m_data[i + 3 * (tmp_data[i50] - 1)];
      }
    }

    loop_ub = anchorInd->size[0];
    for (i50 = 0; i50 < loop_ub; i50++) {
      for (i = 0; i < 3; i++) {
        new_m_data[i + 3 * i50] = b_new_m_data[i + 3 * ((int)anchorInd->data[i50]
          - 1)];
      }
    }

    loop_ub = untriangulated_idx->size[0];
    for (i50 = 0; i50 < loop_ub; i50++) {
      for (i = 0; i < 3; i++) {
        new_m_data[i + 3 * (i50 + anchorInd->size[0])] = c_new_m_data[i + 3 *
          ((int)untriangulated_idx->data[i50] - 1)];
      }
    }

    emxFree_real_T(&untriangulated_idx);

    // 'SLAM_upd:145' status_ind = [triangulated_status_ind; untriangulated_status_ind]; 
    for (i50 = 0; i50 < c_triangulated_status_ind_size_; i50++) {
      status_ind_data[i50] = triangulated_status_ind_data[i50];
    }

    for (i50 = 0; i50 < idx; i50++) {
      status_ind_data[i50 + c_triangulated_status_ind_size_] =
        untriangulated_status_ind_data[i50];
    }

    //  insert new features into the state. first the closest triangulated ones, 
    //  then the un-triangulated ones
    // 'SLAM_upd:149' new_feature_idx = 1;
    new_feature_idx = 0;

    // 'SLAM_upd:150' for anchorIdx = 1:numAnchors
    anchorIdx = 0;
    exitg4 = false;
    while ((!exitg4) && (anchorIdx + 1 < 6)) {
      //          if new_feature_idx > length(new_depths)
      // 'SLAM_upd:152' if new_feature_idx > nnz(triangulation_success)
      idx = 0;
      for (ixstart = 0; ixstart < ii_size_idx_0; ixstart++) {
        if (triangulation_success_data[ixstart]) {
          idx++;
        }
      }

      if ((new_feature_idx + 1 > idx) || (ind_l2_size[0] - new_feature_idx < 4))
      {
        exitg4 = true;
      } else {
        // 'SLAM_upd:155' if length(new_depths) - (new_feature_idx -1) < minFeatureThreshold 
        // 'SLAM_upd:158' if getNumValidFeatures(xt.anchor_states(anchorIdx)) < minFeatureThreshold 
        if (getNumValidFeatures(b_xt->anchor_states[anchorIdx].feature_states) <
            4.0) {
          //  anchor needs to be initialized
          //  free up updateVect
          // 'SLAM_upd:161' for featureIdx = 1:numPointsPerAnchor
          for (featureIdx = 0; featureIdx < 8; featureIdx++) {
            // 'SLAM_upd:162' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status 
            if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status
                != 0) {
              //                      ros_info('clearing up feature %i (%i on %i)', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx, featureIdx, anchorIdx) 
              // 'SLAM_upd:164' updateVect(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
              updateVect[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = 0;

              // 'SLAM_upd:165' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                status_idx = 0;

              // 'SLAM_upd:166' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                0;
            }
          }

          // 'SLAM_upd:170' if xt.origin.anchor_idx == anchorIdx
          if (b_xt->origin.anchor_idx == anchorIdx + 1) {
            // 'SLAM_upd:171' xt.origin.anchor_idx = int32(0);
            b_xt->origin.anchor_idx = 0;

            // 'SLAM_upd:172' ros_info('Initializing anchor %i, which was the origin anchor', int32(anchorIdx)) 
            d_ros_info(anchorIdx + 1);
          }

          // 'SLAM_upd:175' xt.anchor_states(anchorIdx).pos = xt.robot_state.pos; 
          for (i50 = 0; i50 < 3; i50++) {
            b_xt->anchor_states[anchorIdx].pos[i50] = b_xt->robot_state.pos[i50];
          }

          // 'SLAM_upd:176' xt.anchor_states(anchorIdx).att = xt.robot_state.att; 
          for (i50 = 0; i50 < 4; i50++) {
            b_xt->anchor_states[anchorIdx].att[i50] = b_xt->robot_state.att[i50];
          }

          // 'SLAM_upd:177' xt.anchor_states(anchorIdx).P_idx = numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6); 
          i50 = anchorIdx * 14;
          for (i = 0; i < 6; i++) {
            b_xt->anchor_states[anchorIdx].P_idx[i] = (i + i50) + 22;
          }

          // 'SLAM_upd:179' P_apo(xt.anchor_states(anchorIdx).P_idx, :) = 0;
          for (i50 = 0; i50 < 91; i50++) {
            for (i = 0; i < 6; i++) {
              P_apr[(b_xt->anchor_states[anchorIdx].P_idx[i] + 91 * i50) - 1] =
                0.0;
            }
          }

          // 'SLAM_upd:180' P_apo(:, xt.anchor_states(anchorIdx).P_idx) = 0;
          for (i50 = 0; i50 < 6; i50++) {
            for (i = 0; i < 91; i++) {
              P_apr[i + 91 * (b_xt->anchor_states[anchorIdx].P_idx[i50] - 1)] =
                0.0;
            }
          }

          // 'SLAM_upd:181' P_apo(xt.anchor_states(anchorIdx).P_idx(end) + (1:numPointsPerAnchor), :) = 0; 
          idx = b_xt->anchor_states[anchorIdx].P_idx[5];
          for (i50 = 0; i50 < 91; i50++) {
            for (i = 0; i < 8; i++) {
              i52 = (long)idx + (1 + i);
              if (i52 > 2147483647L) {
                i52 = 2147483647L;
              } else {
                if (i52 < -2147483648L) {
                  i52 = -2147483648L;
                }
              }

              P_apr[((int)i52 + 91 * i50) - 1] = 0.0;
            }
          }

          // 'SLAM_upd:182' P_apo(:, xt.anchor_states(anchorIdx).P_idx(end) + (1:numPointsPerAnchor)) = 0; 
          idx = b_xt->anchor_states[anchorIdx].P_idx[5];
          for (i50 = 0; i50 < 8; i50++) {
            for (i = 0; i < 91; i++) {
              i52 = (long)idx + (1 + i50);
              if (i52 > 2147483647L) {
                i52 = 2147483647L;
              } else {
                if (i52 < -2147483648L) {
                  i52 = -2147483648L;
                }
              }

              P_apr[i + 91 * ((int)i52 - 1)] = 0.0;
            }
          }

          //  covariance with robot states
          // 'SLAM_upd:186' P_apo(xt.anchor_states(anchorIdx).P_idx, xt.anchor_states(anchorIdx).P_idx)        = P_apo(1:6, 1:6); 
          for (i50 = 0; i50 < 6; i50++) {
            for (i = 0; i < 6; i++) {
              b_P_apr[i + 6 * i50] = P_apr[i + 91 * i50];
            }
          }

          for (i50 = 0; i50 < 6; i50++) {
            for (i = 0; i < 6; i++) {
              P_apr[(b_xt->anchor_states[anchorIdx].P_idx[i] + 91 *
                     (b_xt->anchor_states[anchorIdx].P_idx[i50] - 1)) - 1] =
                b_P_apr[i + 6 * i50];
            }
          }

          //  anchor position, attitude autocovariance
          // 'SLAM_upd:187' P_apo(xt.anchor_states(anchorIdx).P_idx, 1:6)                                      = P_apo(1:6, 1:6); 
          for (i50 = 0; i50 < 6; i50++) {
            for (i = 0; i < 6; i++) {
              b_P_apr[i + 6 * i50] = P_apr[i + 91 * i50];
            }
          }

          for (i50 = 0; i50 < 6; i50++) {
            for (i = 0; i < 6; i++) {
              P_apr[(b_xt->anchor_states[anchorIdx].P_idx[i] + 91 * i50) - 1] =
                b_P_apr[i + 6 * i50];
            }
          }

          //  anchor position, attitude covariance with robot state
          // 'SLAM_upd:188' P_apo(1:6, xt.anchor_states(anchorIdx).P_idx)                                      = P_apo(1:6, 1:6); 
          for (i50 = 0; i50 < 6; i50++) {
            for (i = 0; i < 6; i++) {
              b_P_apr[i + 6 * i50] = P_apr[i + 91 * i50];
            }
          }

          for (i50 = 0; i50 < 6; i50++) {
            for (i = 0; i < 6; i++) {
              P_apr[i + 91 * (b_xt->anchor_states[anchorIdx].P_idx[i50] - 1)] =
                b_P_apr[i + 6 * i50];
            }
          }

          //  anchor position, attitude covariance with robot state
          // 'SLAM_upd:189' P_apo(7:xt.anchor_states(anchorIdx).P_idx(1)-1, xt.anchor_states(anchorIdx).P_idx) = P_apo(7:xt.anchor_states(anchorIdx).P_idx(1)-1, 1:6); 
          i52 = b_xt->anchor_states[anchorIdx].P_idx[0] - 1L;
          if (i52 > 2147483647L) {
            i52 = 2147483647L;
          } else {
            if (i52 < -2147483648L) {
              i52 = -2147483648L;
            }
          }

          i50 = (int)i52;
          if (7 > i50) {
            i = 0;
            i50 = 0;
          } else {
            i = 6;
          }

          i52 = b_xt->anchor_states[anchorIdx].P_idx[0] - 1L;
          if (i52 > 2147483647L) {
            i52 = 2147483647L;
          } else {
            if (i52 < -2147483648L) {
              i52 = -2147483648L;
            }
          }

          if (7 > (int)i52) {
            idx = 0;
          } else {
            idx = 6;
          }

          ixstart = i50 - i;
          for (ix = 0; ix < 6; ix++) {
            loop_ub = i50 - i;
            for (trueCount = 0; trueCount < loop_ub; trueCount++) {
              P_apr_data[trueCount + ixstart * ix] = P_apr[(i + trueCount) + 91 *
                ix];
            }
          }

          for (i50 = 0; i50 < 6; i50++) {
            for (i = 0; i < ixstart; i++) {
              P_apr[(idx + i) + 91 * (b_xt->anchor_states[anchorIdx].P_idx[i50]
                - 1)] = P_apr_data[i + ixstart * i50];
            }
          }

          //  covariance with bias states etc
          // 'SLAM_upd:190' P_apo(xt.anchor_states(anchorIdx).P_idx, 7:xt.anchor_states(anchorIdx).P_idx(1)-1) = P_apo(1:6, 7:xt.anchor_states(anchorIdx).P_idx(1)-1); 
          i52 = b_xt->anchor_states[anchorIdx].P_idx[0] - 1L;
          if (i52 > 2147483647L) {
            i52 = 2147483647L;
          } else {
            if (i52 < -2147483648L) {
              i52 = -2147483648L;
            }
          }

          i50 = (int)i52;
          if (7 > i50) {
            i = 0;
            i50 = 0;
          } else {
            i = 6;
          }

          i52 = b_xt->anchor_states[anchorIdx].P_idx[0] - 1L;
          if (i52 > 2147483647L) {
            i52 = 2147483647L;
          } else {
            if (i52 < -2147483648L) {
              i52 = -2147483648L;
            }
          }

          if (7 > (int)i52) {
            idx = 0;
          } else {
            idx = 6;
          }

          loop_ub = i50 - i;
          for (ix = 0; ix < loop_ub; ix++) {
            for (trueCount = 0; trueCount < 6; trueCount++) {
              P_apr_data[trueCount + 6 * ix] = P_apr[trueCount + 91 * (i + ix)];
            }
          }

          loop_ub = i50 - i;
          for (i50 = 0; i50 < loop_ub; i50++) {
            for (i = 0; i < 6; i++) {
              P_apr[(b_xt->anchor_states[anchorIdx].P_idx[i] + 91 * (idx + i50))
                - 1] = P_apr_data[i + 6 * i50];
            }
          }

          //  covariance with bias states etc
          //  covariance with other anchor states
          // 'SLAM_upd:192' P_apo(xt.anchor_states(anchorIdx).P_idx(1)+1:end, xt.anchor_states(anchorIdx).P_idx) = P_apo(xt.anchor_states(anchorIdx).P_idx(1)+1:end, 1:6); 
          i52 = b_xt->anchor_states[anchorIdx].P_idx[0] + 1L;
          if (i52 > 2147483647L) {
            i52 = 2147483647L;
          } else {
            if (i52 < -2147483648L) {
              i52 = -2147483648L;
            }
          }

          i50 = (int)i52 - 92;
          i52 = b_xt->anchor_states[anchorIdx].P_idx[0] + 1L;
          if (i52 > 2147483647L) {
            i52 = 2147483647L;
          } else {
            if (i52 < -2147483648L) {
              i52 = -2147483648L;
            }
          }

          i = (int)i52 - 1;
          for (idx = 0; idx < 6; idx++) {
            loop_ub = -i50;
            for (ix = 0; ix < loop_ub; ix++) {
              b_P_apr_data[ix + -i50 * idx] = P_apr[((i50 + ix) + 91 * idx) + 91];
            }
          }

          for (idx = 0; idx < 6; idx++) {
            loop_ub = -i50;
            for (ix = 0; ix < loop_ub; ix++) {
              P_apr[(i + ix) + 91 * (b_xt->anchor_states[anchorIdx].P_idx[idx] -
                1)] = b_P_apr_data[ix + -i50 * idx];
            }
          }

          // 'SLAM_upd:193' P_apo(xt.anchor_states(anchorIdx).P_idx, xt.anchor_states(anchorIdx).P_idx(1)+1:end) = P_apo(1:6, xt.anchor_states(anchorIdx).P_idx(1)+1:end); 
          i52 = b_xt->anchor_states[anchorIdx].P_idx[0] + 1L;
          if (i52 > 2147483647L) {
            i52 = 2147483647L;
          } else {
            if (i52 < -2147483648L) {
              i52 = -2147483648L;
            }
          }

          i50 = (int)i52 - 92;
          i52 = b_xt->anchor_states[anchorIdx].P_idx[0] + 1L;
          if (i52 > 2147483647L) {
            i52 = 2147483647L;
          } else {
            if (i52 < -2147483648L) {
              i52 = -2147483648L;
            }
          }

          i = (int)i52 - 1;
          loop_ub = -i50;
          for (idx = 0; idx < loop_ub; idx++) {
            for (ix = 0; ix < 6; ix++) {
              b_P_apr_data[ix + 6 * idx] = P_apr[ix + 91 * ((i50 + idx) + 91)];
            }
          }

          loop_ub = -i50;
          for (i50 = 0; i50 < loop_ub; i50++) {
            for (idx = 0; idx < 6; idx++) {
              P_apr[(b_xt->anchor_states[anchorIdx].P_idx[idx] + 91 * (i + i50))
                - 1] = b_P_apr_data[idx + 6 * i50];
            }
          }

          // 'SLAM_upd:195' if coder.target('MATLAB')
          // 'SLAM_upd:212' for featureIdx = 1:numPointsPerAnchor
          featureIdx = 0;
          exitg5 = false;
          while ((!exitg5) && (featureIdx + 1 < 9)) {
            // 'SLAM_upd:213' xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth = 1/new_depths(new_feature_idx); 
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              inverse_depth = 1.0 / ind_l2_data[new_feature_idx];

            // 'SLAM_upd:214' xt.anchor_states(anchorIdx).feature_states(featureIdx).m = new_m(:,new_feature_idx); 
            for (i50 = 0; i50 < 3; i50++) {
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].m[i50] =
                new_m_data[i50 + 3 * new_feature_idx];
            }

            // 'SLAM_upd:215' if VIOParameters.delayed_initialization
            if (b_VIOParameters.delayed_initialization) {
              // 'SLAM_upd:216' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(2); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                2;
            } else {
              // 'SLAM_upd:217' else
              // 'SLAM_upd:218' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(1); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                1;
            }

            // 'SLAM_upd:220' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(status_ind(new_feature_idx)); 
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status_idx
              = (int)status_ind_data[new_feature_idx];

            // 'SLAM_upd:221' xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx = int32(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx =
              (anchorIdx * 14 + featureIdx) + 28;

            // 'SLAM_upd:223' if new_feature_idx > nnz(triangulation_success)
            idx = 0;
            for (ixstart = 0; ixstart < ii_size_idx_0; ixstart++) {
              if (triangulation_success_data[ixstart]) {
                idx++;
              }
            }

            if (new_feature_idx + 1 > idx) {
              // 'SLAM_upd:224' ros_info('Feature %d is too far away to triangulate.\n', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) 
              e_ros_info(b_xt->anchor_states[anchorIdx]
                         .feature_states[featureIdx].status_idx);

              // 'SLAM_upd:225' P_apo(xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx, xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx) = noiseParameters.inv_depth_initial_unc*10; 
              P_apr[(b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                     P_idx + 91 * (b_xt->anchor_states[anchorIdx]
                                   .feature_states[featureIdx].P_idx - 1)) - 1] =
                c_noiseParameters_inv_depth_ini * 10.0;

              //  TODO: Maybe push the mean value further away?
            } else {
              // 'SLAM_upd:227' else
              // 'SLAM_upd:228' P_apo(xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx, xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx) = noiseParameters.inv_depth_initial_unc; 
              P_apr[(b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                     P_idx + 91 * (b_xt->anchor_states[anchorIdx]
                                   .feature_states[featureIdx].P_idx - 1)) - 1] =
                c_noiseParameters_inv_depth_ini;

              // *new_depths(new_feature_idx);
            }

            //                  ros_info('Inserting feature %d as feature %i on anchor %i', int32(status_ind(new_feature_idx)), int32(featureIdx), int32(anchorIdx)) 
            // 'SLAM_upd:233' updateVect(status_ind(new_feature_idx)) = int32(1); 
            updateVect[(int)status_ind_data[new_feature_idx] - 1] = 1;

            // 'SLAM_upd:235' new_feature_idx = new_feature_idx + 1;
            new_feature_idx++;

            //                  if new_feature_idx > length(new_depths)
            // 'SLAM_upd:237' if new_feature_idx > nnz(triangulation_success)
            idx = 0;
            for (ixstart = 0; ixstart < ii_size_idx_0; ixstart++) {
              if (triangulation_success_data[ixstart]) {
                idx++;
              }
            }

            if (new_feature_idx + 1 > idx) {
              exitg5 = true;
            } else {
              featureIdx++;
            }
          }
        }

        anchorIdx++;
      }
    }
  } else {
    if (!(ii_size_idx_0 == 0)) {
      // 'SLAM_upd:243' elseif ~isempty(ind_r)
      // 'SLAM_upd:244' ros_warn('Got %d new feautures but not enough for a new anchor (min %d)', length(ind_r), int32(minFeatureThreshold)) 
      g_ros_warn((double)ii_size_idx_0);
    }
  }

  // 'SLAM_upd:246' updateVect(updateVect==int32(2)) = int32(0);
  for (i = 0; i < 40; i++) {
    if (updateVect[i] == 2) {
      updateVect[i] = 0;
    }
  }

  //  remove features that were not inserted
  // 'SLAM_upd:248' if VIOParameters.delayed_initialization
  if (b_VIOParameters.delayed_initialization) {
    //  get the median uncertainty of the active features as a benchmark on
    //  the delayed features
    // 'SLAM_upd:251' uncertainties = -1*ones(numTrackFeatures, 1);
    for (i = 0; i < 40; i++) {
      b_uncertainties[i] = -1.0;
    }

    // 'SLAM_upd:252' has_active_features = false;
    has_active_features = false;

    // 'SLAM_upd:253' for anchorIdx = 1:numAnchors
    for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
      // 'SLAM_upd:254' for featureIdx = 1:numPointsPerAnchor
      for (featureIdx = 0; featureIdx < 8; featureIdx++) {
        // 'SLAM_upd:255' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 1 
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
            1) {
          // 'SLAM_upd:256' has_active_features = true;
          has_active_features = true;

          // 'SLAM_upd:257' uncertainties(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = P_apo(xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx, xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx); 
          b_uncertainties[b_xt->anchor_states[anchorIdx]
            .feature_states[featureIdx].status_idx - 1] = P_apr
            [(b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx +
              91 * (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                    P_idx - 1)) - 1];
        }
      }
    }

    // 'SLAM_upd:262' if has_active_features
    if (has_active_features) {
      // 'SLAM_upd:263' median_uncertainty = median(uncertainties(uncertainties > 0), 1); 
      trueCount = 0;
      for (i = 0; i < 40; i++) {
        if (b_uncertainties[i] > 0.0) {
          trueCount++;
        }
      }

      ixstart = 0;
      for (i = 0; i < 40; i++) {
        if (b_uncertainties[i] > 0.0) {
          ii_data[ixstart] = i + 1;
          ixstart++;
        }
      }

      uncertainties_size[0] = trueCount;
      for (i50 = 0; i50 < trueCount; i50++) {
        b_ii_data[i50] = b_uncertainties[ii_data[i50] - 1];
      }

      median_uncertainty = median(b_ii_data, uncertainties_size);

      //  because coder does not support nanflag
      //  check if a delayed initialization feature has converged
      // 'SLAM_upd:266' for anchorIdx = 1:numAnchors
      for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
        // 'SLAM_upd:267' for featureIdx = 1:numPointsPerAnchor
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          // 'SLAM_upd:268' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 
          if ((b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
               2) && (P_apr[(b_xt->anchor_states[anchorIdx]
                             .feature_states[featureIdx].P_idx + 91 *
                             (b_xt->anchor_states[anchorIdx]
                              .feature_states[featureIdx].P_idx - 1)) - 1] <
                      median_uncertainty * 2.0)) {
            //  this feature is not active yet
            // 'SLAM_upd:269' if P_apo(xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx, xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx) < median_uncertainty*2 
            // 'SLAM_upd:270' if xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth < 0 
            if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0) {
              // 'SLAM_upd:271' ros_warn('Feature %i (%i on anchor %i) converged behind its anchor', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx, featureIdx, anchorIdx) 
              h_ros_warn(b_xt->anchor_states[anchorIdx]
                         .feature_states[featureIdx].status_idx, featureIdx + 1,
                         anchorIdx + 1);

              // 'SLAM_upd:272' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                0;

              // 'SLAM_upd:273' updateVect(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
              updateVect[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = 0;
            } else {
              // 'SLAM_upd:274' else
              //                              ros_info('Feature %i (%i on anchor %i) has converged', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx, featureIdx, anchorIdx); 
              // 'SLAM_upd:276' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(1); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                1;
            }
          }
        }
      }
    }

    //  check if delayed features need to be forced active due to too few active 
    //  features
    // 'SLAM_upd:286' minActiveFeatureRatio = 0.4;
    // 'SLAM_upd:287' totalNumActiveFeatues = getTotalNumActiveFeatures(xt);
    totalNumActiveFeatues = getTotalNumActiveFeatures(b_xt->anchor_states);

    // 'SLAM_upd:288' if totalNumActiveFeatues < minActiveFeatureRatio*single(numTrackFeatures) 
    if (totalNumActiveFeatues < 16.0) {
      //  find the best features and activate them
      // 'SLAM_upd:290' numDelayedFeatures = getTotalNumDelayedFeatures(xt);
      numDelayedFeatures = getTotalNumDelayedFeatures(b_xt->anchor_states);

      // 'SLAM_upd:291' qualities = zeros(numDelayedFeatures, 1);
      i50 = qualities->size[0];
      qualities->size[0] = (int)numDelayedFeatures;
      emxEnsureCapacity((emxArray__common *)qualities, i50, (int)sizeof(double));
      loop_ub = (int)numDelayedFeatures;
      for (i50 = 0; i50 < loop_ub; i50++) {
        qualities->data[i50] = 0.0;
      }

      //  quality measures of each delayed feature
      // 'SLAM_upd:292' anchorInd = qualities;
      i50 = anchorInd->size[0];
      anchorInd->size[0] = (int)numDelayedFeatures;
      emxEnsureCapacity((emxArray__common *)anchorInd, i50, (int)sizeof(double));
      loop_ub = (int)numDelayedFeatures;
      for (i50 = 0; i50 < loop_ub; i50++) {
        anchorInd->data[i50] = 0.0;
      }

      // 'SLAM_upd:293' featureInd = qualities;
      i50 = featureInd->size[0];
      featureInd->size[0] = (int)numDelayedFeatures;
      emxEnsureCapacity((emxArray__common *)featureInd, i50, (int)sizeof(double));
      loop_ub = (int)numDelayedFeatures;
      for (i50 = 0; i50 < loop_ub; i50++) {
        featureInd->data[i50] = 0.0;
      }

      // 'SLAM_upd:295' delayedIdx = 1;
      delayedIdx = 1U;

      // 'SLAM_upd:296' for anchorIdx = 1:numAnchors
      for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
        // 'SLAM_upd:297' for featureIdx = 1:numPointsPerAnchor
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          // 'SLAM_upd:298' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              2) {
            // 'SLAM_upd:299' rho_unc = P_apo(xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx, xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx); 
            // 'SLAM_upd:301' quality = rho_unc/noiseParameters.inv_depth_initial_unc; 
            // 'SLAM_upd:303' qualities(delayedIdx) = quality;
            qualities->data[(int)delayedIdx - 1] = P_apr[(b_xt->
              anchor_states[anchorIdx].feature_states[featureIdx].P_idx + 91 *
              (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx -
               1)) - 1] / c_noiseParameters_inv_depth_ini;

            // 'SLAM_upd:304' anchorInd(delayedIdx) = anchorIdx;
            anchorInd->data[(int)delayedIdx - 1] = (double)anchorIdx + 1.0;

            // 'SLAM_upd:305' featureInd(delayedIdx) = featureIdx;
            featureInd->data[(int)delayedIdx - 1] = (double)featureIdx + 1.0;

            // 'SLAM_upd:307' delayedIdx = delayedIdx + 1;
            delayedIdx++;
          }
        }
      }

      // 'SLAM_upd:312' [~, sortInd] = sort(qualities, 'ascend');
      c_sort(qualities, b_iidx);
      i50 = qualities->size[0];
      qualities->size[0] = b_iidx->size[0];
      emxEnsureCapacity((emxArray__common *)qualities, i50, (int)sizeof(double));
      loop_ub = b_iidx->size[0];
      for (i50 = 0; i50 < loop_ub; i50++) {
        qualities->data[i50] = b_iidx->data[i50];
      }

      // 'SLAM_upd:312' ~
      // 'SLAM_upd:314' numActivatedFeatures = 0;
      numActivatedFeatures = 0.0;

      // 'SLAM_upd:316' for i = 1:length(sortInd)
      i = 0;
      while ((i <= qualities->size[0] - 1) && (!(numActivatedFeatures > 16.0F -
               (float)totalNumActiveFeatues)) && (!(numActivatedFeatures >
               numDelayedFeatures))) {
        // 'SLAM_upd:317' if numActivatedFeatures > ceil(minActiveFeatureRatio*single(numTrackFeatures) - totalNumActiveFeatues) || numActivatedFeatures > numDelayedFeatures 
        // 'SLAM_upd:320' if xt.anchor_states(anchorInd(sortInd(i))).feature_states(featureInd(sortInd(i))).inverse_depth < 0 
        if (b_xt->anchor_states[(int)anchorInd->data[(int)qualities->data[i] - 1]
            - 1].feature_states[(int)featureInd->data[(int)qualities->data[i] -
            1] - 1].inverse_depth < 0.0) {
          // 'SLAM_upd:321' xt.anchor_states(anchorInd(sortInd(i))).feature_states(featureInd(sortInd(i))).status = int32(0); 
          b_xt->anchor_states[(int)anchorInd->data[(int)qualities->data[i] - 1]
            - 1].feature_states[(int)featureInd->data[(int)qualities->data[i] -
            1] - 1].status = 0;

          // 'SLAM_upd:322' updateVect(xt.anchor_states(anchorInd(sortInd(i))).feature_states(featureInd(sortInd(i))).status_idx) = int32(0); 
          updateVect[b_xt->anchor_states[(int)anchorInd->data[(int)
            qualities->data[i] - 1] - 1].feature_states[(int)featureInd->data
            [(int)qualities->data[i] - 1] - 1].status_idx - 1] = 0;

          // 'SLAM_upd:323' ros_warn('Trying to force insert feature %i behind its anchor', xt.anchor_states(anchorInd(sortInd(i))).feature_states(featureInd(sortInd(i))).status_idx) 
          i_ros_warn(b_xt->anchor_states[(int)anchorInd->data[(int)
                     qualities->data[i] - 1] - 1].feature_states[(int)
                     featureInd->data[(int)qualities->data[i] - 1] - 1].
                     status_idx);
        } else {
          // 'SLAM_upd:324' else
          // 'SLAM_upd:325' xt.anchor_states(anchorInd(sortInd(i))).feature_states(featureInd(sortInd(i))).status = int32(1); 
          b_xt->anchor_states[(int)anchorInd->data[(int)qualities->data[i] - 1]
            - 1].feature_states[(int)featureInd->data[(int)qualities->data[i] -
            1] - 1].status = 1;

          // 'SLAM_upd:326' ros_info('Forcing activation of feature %i (%i on anchor %i)', xt.anchor_states(anchorInd(sortInd(i))).feature_states(featureInd(sortInd(i))).status_idx, featureInd(sortInd(i)), anchorInd(sortInd(i))); 
          f_ros_info(b_xt->anchor_states[(int)anchorInd->data[(int)
                     qualities->data[i] - 1] - 1].feature_states[(int)
                     featureInd->data[(int)qualities->data[i] - 1] - 1].
                     status_idx, featureInd->data[(int)qualities->data[i] - 1],
                     anchorInd->data[(int)qualities->data[i] - 1]);

          // 'SLAM_upd:327' numActivatedFeatures = numActivatedFeatures + 1;
          numActivatedFeatures++;
        }

        i++;
      }
    }
  }

  emxFree_int32_T(&b_iidx);
  emxFree_real_T(&featureInd);
  emxFree_real_T(&anchorInd);
  emxFree_real_T(&qualities);

  //  check if new features need to be requested
  // 'SLAM_upd:334' request_new_features = false;
  request_new_features = false;

  // 'SLAM_upd:335' if nnz(updateVect == 0) > minFeatureThreshold
  idx = 0;
  for (ixstart = 0; ixstart < 40; ixstart++) {
    if (updateVect[ixstart] == 0) {
      idx++;
    }
  }

  if (idx > 4) {
    //  if a new anchor can be filled enough
    // 'SLAM_upd:336' for anchorIdx = 1:numAnchors
    anchorIdx = 1;
    exitg3 = false;
    while ((!exitg3) && (anchorIdx < 6)) {
      // 'SLAM_upd:337' if getNumValidFeatures(xt.anchor_states(anchorIdx)) < minFeatureThreshold 
      if (getNumValidFeatures(b_xt->anchor_states[anchorIdx - 1].feature_states)
          < 4.0) {
        // 'SLAM_upd:338' request_new_features = true;
        request_new_features = true;
        exitg3 = true;
      } else {
        anchorIdx++;
      }
    }
  } else {
    // 'SLAM_upd:342' else
    //  debug check
    // 'SLAM_upd:343' if coder.target('MATLAB')
  }

  // 'SLAM_upd:352' if request_new_features
  if (request_new_features) {
    // 'SLAM_upd:353' request_idx = 1;
    request_idx = 1;

    // 'SLAM_upd:354' for i = 1:length(updateVect)
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i < 40)) {
      // 'SLAM_upd:355' if updateVect(i) == 0
      b_guard1 = false;
      if (updateVect[i] == 0) {
        // 'SLAM_upd:356' updateVect(i) = 2;
        updateVect[i] = 2;

        // 'SLAM_upd:357' request_idx = request_idx +1;
        request_idx++;

        // 'SLAM_upd:358' if request_idx > max_features_to_request
        if (request_idx > 16) {
          exitg2 = true;
        } else {
          b_guard1 = true;
        }
      } else {
        b_guard1 = true;
      }

      if (b_guard1) {
        i++;
      }
    }

    // 'SLAM_upd:363' ros_info('Requesting %d new features', int32(request_idx-1)) 
    g_ros_info(request_idx - 1);

    //      updateVect(updateVect == 0) = int32(2); % get as many new features as possible 
  }

  // % robocentric update
  // 'SLAM_upd:368' if xt.origin.anchor_idx == 0
  if (b_xt->origin.anchor_idx == 0) {
    //  need to update the origin anchor and the state
    // 'SLAM_upd:369' xt.fixed_feature = int32(0);
    b_xt->fixed_feature = 0;

    //  ensure that a new feature will be fixed, if this option is enabled
    //  choose the best anchor as the new origin anchor
    // 'SLAM_upd:371' uncertainties = zeros(numAnchors, 1);
    //  uncertainties of the anchors reduced to a scalar
    // 'SLAM_upd:372' has_active_features = uncertainties;
    // 'SLAM_upd:373' for anchorIdx = 1:numAnchors
    for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
      b_has_active_features[anchorIdx] = 0.0;

      // 'SLAM_upd:374' if anyActiveAnchorFeatures(xt.anchor_states(anchorIdx))
      if (anyActiveAnchorFeatures(b_xt->anchor_states[anchorIdx].feature_states))
      {
        // 'SLAM_upd:375' uncertainties(anchorIdx) = det(P_apo(xt.anchor_states(anchorIdx).P_idx, xt.anchor_states(anchorIdx).P_idx)); 
        for (i50 = 0; i50 < 6; i50++) {
          for (i = 0; i < 6; i++) {
            b_P_apr[i + 6 * i50] = P_apr[(b_xt->anchor_states[anchorIdx].P_idx[i]
              + 91 * (b_xt->anchor_states[anchorIdx].P_idx[i50] - 1)) - 1];
          }
        }

        c_uncertainties[anchorIdx] = det(b_P_apr);

        // 'SLAM_upd:376' has_active_features(anchorIdx) = 1;
        b_has_active_features[anchorIdx] = 1.0;
      } else {
        // 'SLAM_upd:377' else
        // 'SLAM_upd:378' uncertainties(anchorIdx) = 1000;
        c_uncertainties[anchorIdx] = 1000.0;

        //  dont fix an anchor with no active features
      }
    }

    // 'SLAM_upd:381' if ~any(has_active_features)
    if (!c_any(b_has_active_features)) {
      //  can happen if outlier rejection rejected all features
      // 'SLAM_upd:382' ros_warn('Can''t fix an anchor because none have active features') 
      j_ros_warn();
    } else {
      // 'SLAM_upd:383' else
      // 'SLAM_upd:384' [~, sortIdx] = min(uncertainties);
      ixstart = 1;
      mtmp = c_uncertainties[0];
      idx = 1;
      if (rtIsNaN(c_uncertainties[0])) {
        ix = 2;
        exitg1 = false;
        while ((!exitg1) && (ix < 6)) {
          ixstart = ix;
          if (!rtIsNaN(c_uncertainties[ix - 1])) {
            mtmp = c_uncertainties[ix - 1];
            idx = ix;
            exitg1 = true;
          } else {
            ix++;
          }
        }
      }

      if (ixstart < 5) {
        while (ixstart + 1 < 6) {
          if (c_uncertainties[ixstart] < mtmp) {
            mtmp = c_uncertainties[ixstart];
            idx = ixstart + 1;
          }

          ixstart++;
        }
      }

      // 'SLAM_upd:384' ~
      // 'SLAM_upd:385' xt.origin.anchor_idx = int32(sortIdx(1));
      b_xt->origin.anchor_idx = idx;

      // 'SLAM_upd:386' if ~has_active_features(xt.origin.anchor_idx)
      if (!(b_has_active_features[b_xt->origin.anchor_idx - 1] != 0.0)) {
        //  debug check
        // 'SLAM_upd:387' ros_error('Picked an anchor with no active features as origin (anchor %d)', int32(xt.origin.anchor_idx)) 
        // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
        // 'ros_error:4' if coder.target('MATLAB')
        // 'ros_error:6' elseif ~coder.target('MEX')
        // 'ros_error:7' coder.cinclude('<ros/console.h>')
        // 'ros_error:8' coder.ceval('ROS_ERROR', [str, 0], varargin{:});
        for (i50 = 0; i50 < 63; i50++) {
          cv50[i50] = cv51[i50];
        }

        ROS_ERROR(cv50, b_xt->origin.anchor_idx);
      } else {
        // 'SLAM_upd:388' else
        // 'SLAM_upd:389' ros_info('Setting anchor %i as origin', int32(xt.origin.anchor_idx)) 
        h_ros_info(b_xt->origin.anchor_idx);

        // 'SLAM_upd:391' new_origin_pos_rel = xt.anchor_states(xt.origin.anchor_idx).pos; 
        //  in old origin frame
        // 'SLAM_upd:392' new_origin_att_rel = RotFromQuatJ(xt.anchor_states(xt.origin.anchor_idx).att); 
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
        // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
        // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
        new_origin_att_rel[0] = ((b_xt->anchor_states[b_xt->origin.anchor_idx -
          1].att[0] * b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[0] -
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[1] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[1]) -
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[2] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[2]) +
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[3] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[3];
        new_origin_att_rel[3] = 2.0 * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] + b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[6] = 2.0 * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] - b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[1] = 2.0 * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] - b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[4] = ((-(b_xt->anchor_states[b_xt->origin.anchor_idx
          - 1].att[0] * b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[0])
          + b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[1] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[1]) -
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[2] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[2]) +
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[3] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[3];
        new_origin_att_rel[7] = 2.0 * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] + b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[2] = 2.0 * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] + b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[5] = 2.0 * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] - b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[8] = ((-(b_xt->anchor_states[b_xt->origin.anchor_idx
          - 1].att[0] * b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[0])
          - b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[1] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[1]) +
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[2] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[2]) +
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[3] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[3];

        //  in old origin frame, = R_o{k+1}o{k}
        // 'SLAM_upd:394' J = eye(size(P_apo));
        d_eye(J);

        //  robot position and orientation
        // 'SLAM_upd:396' J(1:6, 1:6) = [new_origin_att_rel, zeros(3); zeros(3), eye(3)]; 
        eye(c_xt);

        // 'SLAM_upd:397' J(1:6, xt.anchor_states(xt.origin.anchor_idx).P_idx) = [-new_origin_att_rel, skew(new_origin_att_rel * (xt.robot_state.pos - new_origin_pos_rel)); zeros(3), -new_origin_att_rel']; 
        for (i = 0; i < 3; i++) {
          new_origin_pos_rel[i] = b_xt->anchor_states[b_xt->origin.anchor_idx -
            1].pos[i];
          for (i50 = 0; i50 < 3; i50++) {
            J[i50 + 91 * i] = new_origin_att_rel[i50 + 3 * i];
            J[i50 + 91 * (i + 3)] = 0.0;
            J[(i50 + 91 * i) + 3] = 0.0;
            J[(i50 + 91 * (i + 3)) + 3] = c_xt[i50 + 3 * i];
          }

          d_xt[i] = b_xt->robot_state.pos[i] - b_xt->anchor_states
            [b_xt->origin.anchor_idx - 1].pos[i];
        }

        // 'skew:2' R=[0,-w(3),w(2);
        // 'skew:3'     w(3),0,-w(1);
        // 'skew:4'     -w(2),w(1),0];
        for (i50 = 0; i50 < 3; i50++) {
          m_l[i50] = 0.0;
          for (i = 0; i < 3; i++) {
            J[i + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
                        .P_idx[i50] - 1)] = -new_origin_att_rel[i + 3 * i50];
            J[(i + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
                         .P_idx[i50] - 1)) + 3] = 0.0;
            m_l[i50] += new_origin_att_rel[i50 + 3 * i] * d_xt[i];
          }
        }

        J[91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1)] =
          0.0;
        J[91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1)] =
          -m_l[2];
        J[91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1)] =
          m_l[1];
        J[1 + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] -
                    1)] = m_l[2];
        J[1 + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] -
                    1)] = 0.0;
        J[1 + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] -
                    1)] = -m_l[0];
        J[2 + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] -
                    1)] = -m_l[1];
        J[2 + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] -
                    1)] = m_l[0];
        J[2 + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] -
                    1)] = 0.0;

        //  robot velocity
        // 'SLAM_upd:399' J(7:9, 7:9) = new_origin_att_rel;
        //  velocity
        //  origin rotation
        // 'SLAM_upd:401' J(16:18, 16:18) = new_origin_att_rel;
        for (i50 = 0; i50 < 3; i50++) {
          for (i = 0; i < 3; i++) {
            J[(i + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
                         .P_idx[i50 + 3] - 1)) + 3] = -new_origin_att_rel[i50 +
              3 * i];
            J[(i + 91 * (6 + i50)) + 6] = new_origin_att_rel[i + 3 * i50];
            J[(i + 91 * (15 + i50)) + 15] = new_origin_att_rel[i + 3 * i50];
          }
        }

        //  origin rotation
        // 'SLAM_upd:403' for anchorIdx = 1:numAnchors
        for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
          // 'SLAM_upd:404' if anchorIdx == xt.origin.anchor_idx
          if (anchorIdx + 1 == b_xt->origin.anchor_idx) {
            //  remove yaw uncertainty, but not pitch or roll
            // 'SLAM_upd:406' J(xt.anchor_states(anchorIdx).P_idx, xt.anchor_states(anchorIdx).P_idx) = [zeros(3), zeros(3); zeros(3), zeros(3)]; 
            for (i50 = 0; i50 < 6; i50++) {
              for (i = 0; i < 6; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i] + 91 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i50] - 1)) - 1] = 0.0;
              }
            }

            //  TODO: allow roll/pitch uncertainty
          } else {
            // 'SLAM_upd:407' else
            // 'SLAM_upd:408' J(xt.anchor_states(anchorIdx).P_idx, xt.anchor_states(anchorIdx).P_idx) = [new_origin_att_rel, zeros(3); zeros(3), eye(3)]; 
            eye(c_xt);
            for (i50 = 0; i50 < 3; i50++) {
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i] + 91 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i50] - 1)) - 1] =
                  new_origin_att_rel[i + 3 * i50];
              }
            }

            for (i50 = 0; i50 < 3; i50++) {
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i] + 91 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i50 + 3] - 1)) - 1] =
                  0.0;
              }
            }

            for (i50 = 0; i50 < 3; i50++) {
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i + 3] + 91 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i50] - 1)) - 1] = 0.0;
              }
            }

            // 'SLAM_upd:409' J(xt.anchor_states(anchorIdx).P_idx, xt.anchor_states(xt.origin.anchor_idx).P_idx) = [-new_origin_att_rel, skew(new_origin_att_rel * (xt.anchor_states(anchorIdx).pos - new_origin_pos_rel)); zeros(3), -new_origin_att_rel']; 
            for (i50 = 0; i50 < 3; i50++) {
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i + 3] + 91 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i50 + 3] - 1)) - 1] =
                  c_xt[i + 3 * i50];
              }

              d_xt[i50] = b_xt->anchor_states[anchorIdx].pos[i50] -
                new_origin_pos_rel[i50];
            }

            // 'skew:2' R=[0,-w(3),w(2);
            // 'skew:3'     w(3),0,-w(1);
            // 'skew:4'     -w(2),w(1),0];
            for (i50 = 0; i50 < 3; i50++) {
              m_l[i50] = 0.0;
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i] + 91 *
                   (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[i50]
                    - 1)) - 1] = -new_origin_att_rel[i + 3 * i50];
                m_l[i50] += new_origin_att_rel[i50 + 3 * i] * d_xt[i];
              }
            }

            J[(b_xt->anchor_states[anchorIdx].P_idx[0] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1))
              - 1] = 0.0;
            J[(b_xt->anchor_states[anchorIdx].P_idx[0] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1))
              - 1] = -m_l[2];
            J[(b_xt->anchor_states[anchorIdx].P_idx[0] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1))
              - 1] = m_l[1];
            J[(b_xt->anchor_states[anchorIdx].P_idx[1] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1))
              - 1] = m_l[2];
            J[(b_xt->anchor_states[anchorIdx].P_idx[1] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1))
              - 1] = 0.0;
            J[(b_xt->anchor_states[anchorIdx].P_idx[1] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1))
              - 1] = -m_l[0];
            J[(b_xt->anchor_states[anchorIdx].P_idx[2] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1))
              - 1] = -m_l[1];
            J[(b_xt->anchor_states[anchorIdx].P_idx[2] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1))
              - 1] = m_l[0];
            J[(b_xt->anchor_states[anchorIdx].P_idx[2] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1))
              - 1] = 0.0;
            for (i50 = 0; i50 < 3; i50++) {
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i + 3] + 91 *
                   (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[i50]
                    - 1)) - 1] = 0.0;
              }
            }

            for (i50 = 0; i50 < 3; i50++) {
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i + 3] + 91 *
                   (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[i50 +
                    3] - 1)) - 1] = -new_origin_att_rel[i50 + 3 * i];
              }
            }
          }

          // 'SLAM_upd:412' xt.anchor_states(anchorIdx).pos = new_origin_att_rel * (xt.anchor_states(anchorIdx).pos - new_origin_pos_rel); 
          for (i = 0; i < 3; i++) {
            m_l[i] = b_xt->anchor_states[anchorIdx].pos[i] -
              new_origin_pos_rel[i];
            b_xt->anchor_states[anchorIdx].pos[i] = 0.0;
          }

          // 'SLAM_upd:413' xt.anchor_states(anchorIdx).att = QuatFromRotJ(RotFromQuatJ(xt.anchor_states(anchorIdx).att) * new_origin_att_rel'); 
          //  if ~all(size(q) == [4, 1])
          //      error('q does not have the size of a quaternion')
          //  end
          //  if abs(norm(q) - 1) > 1e-3
          //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
          //  end
          // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
          // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
          // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
          e_xt[0] = ((b_xt->anchor_states[anchorIdx].att[0] *
                      b_xt->anchor_states[anchorIdx].att[0] -
                      b_xt->anchor_states[anchorIdx].att[1] *
                      b_xt->anchor_states[anchorIdx].att[1]) -
                     b_xt->anchor_states[anchorIdx].att[2] * b_xt->
                     anchor_states[anchorIdx].att[2]) + b_xt->
            anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx]
            .att[3];
          e_xt[3] = 2.0 * (b_xt->anchor_states[anchorIdx].att[0] *
                           b_xt->anchor_states[anchorIdx].att[1] +
                           b_xt->anchor_states[anchorIdx].att[2] *
                           b_xt->anchor_states[anchorIdx].att[3]);
          e_xt[6] = 2.0 * (b_xt->anchor_states[anchorIdx].att[0] *
                           b_xt->anchor_states[anchorIdx].att[2] -
                           b_xt->anchor_states[anchorIdx].att[1] *
                           b_xt->anchor_states[anchorIdx].att[3]);
          e_xt[1] = 2.0 * (b_xt->anchor_states[anchorIdx].att[0] *
                           b_xt->anchor_states[anchorIdx].att[1] -
                           b_xt->anchor_states[anchorIdx].att[2] *
                           b_xt->anchor_states[anchorIdx].att[3]);
          e_xt[4] = ((-(b_xt->anchor_states[anchorIdx].att[0] *
                        b_xt->anchor_states[anchorIdx].att[0]) +
                      b_xt->anchor_states[anchorIdx].att[1] *
                      b_xt->anchor_states[anchorIdx].att[1]) -
                     b_xt->anchor_states[anchorIdx].att[2] * b_xt->
                     anchor_states[anchorIdx].att[2]) + b_xt->
            anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx]
            .att[3];
          e_xt[7] = 2.0 * (b_xt->anchor_states[anchorIdx].att[1] *
                           b_xt->anchor_states[anchorIdx].att[2] +
                           b_xt->anchor_states[anchorIdx].att[0] *
                           b_xt->anchor_states[anchorIdx].att[3]);
          e_xt[2] = 2.0 * (b_xt->anchor_states[anchorIdx].att[0] *
                           b_xt->anchor_states[anchorIdx].att[2] +
                           b_xt->anchor_states[anchorIdx].att[1] *
                           b_xt->anchor_states[anchorIdx].att[3]);
          e_xt[5] = 2.0 * (b_xt->anchor_states[anchorIdx].att[1] *
                           b_xt->anchor_states[anchorIdx].att[2] -
                           b_xt->anchor_states[anchorIdx].att[0] *
                           b_xt->anchor_states[anchorIdx].att[3]);
          e_xt[8] = ((-(b_xt->anchor_states[anchorIdx].att[0] *
                        b_xt->anchor_states[anchorIdx].att[0]) -
                      b_xt->anchor_states[anchorIdx].att[1] *
                      b_xt->anchor_states[anchorIdx].att[1]) +
                     b_xt->anchor_states[anchorIdx].att[2] * b_xt->
                     anchor_states[anchorIdx].att[2]) + b_xt->
            anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx]
            .att[3];
          for (i50 = 0; i50 < 3; i50++) {
            b_xt->anchor_states[anchorIdx].pos[i50] = 0.0;
            for (i = 0; i < 3; i++) {
              b_xt->anchor_states[anchorIdx].pos[i50] += new_origin_att_rel[i50
                + 3 * i] * m_l[i];
              c_xt[i50 + 3 * i] = 0.0;
              for (idx = 0; idx < 3; idx++) {
                c_xt[i50 + 3 * i] += e_xt[i50 + 3 * idx] * new_origin_att_rel[i
                  + 3 * idx];
              }
            }
          }

          QuatFromRotJ(c_xt, b_xt->anchor_states[anchorIdx].att);
        }

        // 'SLAM_upd:416' P_apo = J * P_apo * J';
        for (i50 = 0; i50 < 91; i50++) {
          for (i = 0; i < 91; i++) {
            b_J[i50 + 91 * i] = 0.0;
            for (idx = 0; idx < 91; idx++) {
              b_J[i50 + 91 * i] += J[i50 + 91 * idx] * P_apr[idx + 91 * i];
            }
          }
        }

        for (i50 = 0; i50 < 91; i50++) {
          for (i = 0; i < 91; i++) {
            P_apr[i50 + 91 * i] = 0.0;
            for (idx = 0; idx < 91; idx++) {
              P_apr[i50 + 91 * i] += b_J[i50 + 91 * idx] * J[i + 91 * idx];
            }
          }
        }

        // 'SLAM_upd:418' xt.robot_state.pos = new_origin_att_rel * (xt.robot_state.pos - new_origin_pos_rel); 
        for (i50 = 0; i50 < 3; i50++) {
          d_xt[i50] = b_xt->robot_state.pos[i50] - new_origin_pos_rel[i50];
        }

        // 'SLAM_upd:419' xt.robot_state.att = QuatFromRotJ(RotFromQuatJ(xt.robot_state.att) * new_origin_att_rel'); 
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
        // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
        // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
        f_xt[0] = ((b_xt->robot_state.att[0] * b_xt->robot_state.att[0] -
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
          b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
        f_xt[3] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] +
                         b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
        f_xt[6] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] -
                         b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
        f_xt[1] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] -
                         b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
        f_xt[4] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) +
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
          b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
        f_xt[7] = 2.0 * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] +
                         b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
        f_xt[2] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] +
                         b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
        f_xt[5] = 2.0 * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] -
                         b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
        f_xt[8] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) -
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) +
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
          b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
        for (i50 = 0; i50 < 3; i50++) {
          b_xt->robot_state.pos[i50] = 0.0;
          for (i = 0; i < 3; i++) {
            b_xt->robot_state.pos[i50] += new_origin_att_rel[i50 + 3 * i] *
              d_xt[i];
            c_xt[i50 + 3 * i] = 0.0;
            for (idx = 0; idx < 3; idx++) {
              c_xt[i50 + 3 * i] += f_xt[i50 + 3 * idx] * new_origin_att_rel[i +
                3 * idx];
            }
          }
        }

        QuatFromRotJ(c_xt, b_xt->robot_state.att);

        // 'SLAM_upd:420' xt.robot_state.vel = new_origin_att_rel * xt.robot_state.vel; 
        for (i50 = 0; i50 < 3; i50++) {
          d_xt[i50] = 0.0;
          for (i = 0; i < 3; i++) {
            d_xt[i50] += new_origin_att_rel[i50 + 3 * i] * b_xt->
              robot_state.vel[i];
          }
        }

        // 'SLAM_upd:422' xt.origin.pos = xt.origin.pos + RotFromQuatJ(xt.origin.att)' * new_origin_pos_rel; 
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
        // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
        // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
        g_xt[0] = ((b_xt->origin.att[0] * b_xt->origin.att[0] - b_xt->
                    origin.att[1] * b_xt->origin.att[1]) - b_xt->origin.att[2] *
                   b_xt->origin.att[2]) + b_xt->origin.att[3] * b_xt->
          origin.att[3];
        g_xt[1] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[1] +
                         b_xt->origin.att[2] * b_xt->origin.att[3]);
        g_xt[2] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[2] -
                         b_xt->origin.att[1] * b_xt->origin.att[3]);
        g_xt[3] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[1] -
                         b_xt->origin.att[2] * b_xt->origin.att[3]);
        g_xt[4] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) +
                    b_xt->origin.att[1] * b_xt->origin.att[1]) -
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];
        g_xt[5] = 2.0 * (b_xt->origin.att[1] * b_xt->origin.att[2] +
                         b_xt->origin.att[0] * b_xt->origin.att[3]);
        g_xt[6] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[2] +
                         b_xt->origin.att[1] * b_xt->origin.att[3]);
        g_xt[7] = 2.0 * (b_xt->origin.att[1] * b_xt->origin.att[2] -
                         b_xt->origin.att[0] * b_xt->origin.att[3]);
        g_xt[8] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) -
                    b_xt->origin.att[1] * b_xt->origin.att[1]) +
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];

        //  in world frame
        // 'SLAM_upd:423' xt.origin.att = QuatFromRotJ(new_origin_att_rel * RotFromQuatJ(xt.origin.att)); 
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
        // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
        // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
        h_xt[0] = ((b_xt->origin.att[0] * b_xt->origin.att[0] - b_xt->
                    origin.att[1] * b_xt->origin.att[1]) - b_xt->origin.att[2] *
                   b_xt->origin.att[2]) + b_xt->origin.att[3] * b_xt->
          origin.att[3];
        h_xt[3] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[1] +
                         b_xt->origin.att[2] * b_xt->origin.att[3]);
        h_xt[6] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[2] -
                         b_xt->origin.att[1] * b_xt->origin.att[3]);
        h_xt[1] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[1] -
                         b_xt->origin.att[2] * b_xt->origin.att[3]);
        h_xt[4] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) +
                    b_xt->origin.att[1] * b_xt->origin.att[1]) -
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];
        h_xt[7] = 2.0 * (b_xt->origin.att[1] * b_xt->origin.att[2] +
                         b_xt->origin.att[0] * b_xt->origin.att[3]);
        h_xt[2] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[2] +
                         b_xt->origin.att[1] * b_xt->origin.att[3]);
        h_xt[5] = 2.0 * (b_xt->origin.att[1] * b_xt->origin.att[2] -
                         b_xt->origin.att[0] * b_xt->origin.att[3]);
        h_xt[8] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) -
                    b_xt->origin.att[1] * b_xt->origin.att[1]) +
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];
        for (i50 = 0; i50 < 3; i50++) {
          b_xt->robot_state.vel[i50] = d_xt[i50];
          mtmp = 0.0;
          for (i = 0; i < 3; i++) {
            mtmp += g_xt[i50 + 3 * i] * new_origin_pos_rel[i];
            c_xt[i50 + 3 * i] = 0.0;
            for (idx = 0; idx < 3; idx++) {
              c_xt[i50 + 3 * i] += new_origin_att_rel[i50 + 3 * idx] * h_xt[idx
                + 3 * i];
            }
          }

          b_xt->origin.pos[i50] += mtmp;
        }

        QuatFromRotJ(c_xt, b_xt->origin.att);

        //  in world frame
      }
    }
  }

  // % aposteriori measurement prediction
  // 'SLAM_upd:429' [map] = getMap(xt);
  getMap(b_xt->origin.pos, b_xt->origin.att, b_xt->anchor_states, b_map);

  //  get map for output
  // 'SLAM_upd:430' xt = getScaledMap(xt);
  getScaledMap(b_xt);

  //  update the scaled map for measurement prediction
  // 'SLAM_upd:432' h_u_apo = zeros(numTrackFeatures*4,1);
  memset(&h_u_apo[0], 0, 160U * sizeof(double));

  // 'SLAM_upd:434' delayedStatus = zeros(size(updateVect));
  memset(&b_delayedStatus[0], 0, 40U * sizeof(double));

  // 'SLAM_upd:435' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    // 'SLAM_upd:436' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      // 'SLAM_upd:437' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status 
      if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status != 0)
      {
        // 'SLAM_upd:438' [h_u_l, h_u_r] = predictMeasurementStereoDistorted(xt.anchor_states(anchorIdx).feature_states(featureIdx).scaled_map_point, cameraParams); 
        c_predictMeasurementStereoDisto(b_xt->anchor_states[anchorIdx].
          feature_states[featureIdx].scaled_map_point,
          c_cameraParams_CameraParameters, d_cameraParams_CameraParameters,
          e_cameraParams_CameraParameters, f_cameraParams_CameraParameters,
          g_cameraParams_CameraParameters, h_cameraParams_CameraParameters,
          i_cameraParams_CameraParameters, j_cameraParams_CameraParameters,
          k_cameraParams_CameraParameters, l_cameraParams_CameraParameters,
          cameraParams_r_lr, cameraParams_R_rl, h_u_l, h_u_r);

        // 'SLAM_upd:439' h_u_apo((xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx - 1)*measDim + int32(1:measDim)) = [h_u_l; h_u_r]; 
        i52 = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
          status_idx - 1L;
        if (i52 > 2147483647L) {
          i52 = 2147483647L;
        } else {
          if (i52 < -2147483648L) {
            i52 = -2147483648L;
          }
        }

        i50 = (int)i52;
        if (i50 > 536870911) {
          idx = MAX_int32_T;
        } else if (i50 <= -536870912) {
          idx = MIN_int32_T;
        } else {
          idx = i50 << 2;
        }

        for (i = 0; i < 2; i++) {
          i52 = (long)idx + (i + 1);
          if (i52 > 2147483647L) {
            i52 = 2147483647L;
          } else {
            if (i52 < -2147483648L) {
              i52 = -2147483648L;
            }
          }

          h_u_apo[(int)i52 - 1] = h_u_l[i];
        }

        for (i = 0; i < 2; i++) {
          i52 = (long)idx + (i + 3);
          if (i52 > 2147483647L) {
            i52 = 2147483647L;
          } else {
            if (i52 < -2147483648L) {
              i52 = -2147483648L;
            }
          }

          h_u_apo[(int)i52 - 1] = h_u_r[i];
        }

        // 'SLAM_upd:440' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
            2) {
          // 'SLAM_upd:441' delayedStatus(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = 1; 
          b_delayedStatus[b_xt->anchor_states[anchorIdx]
            .feature_states[featureIdx].status_idx - 1] = 1.0;
        }
      }
    }
  }
}

//
// Arguments    : const boolean_T x[40]
// Return Type  : boolean_T
//
static boolean_T any(const boolean_T x[40])
{
  boolean_T y;
  int k;
  boolean_T exitg1;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 40)) {
    if (!!x[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

//
// Arguments    : const e_struct_T anchor_state_feature_states[8]
// Return Type  : boolean_T
//
static boolean_T anyActiveAnchorFeatures(const e_struct_T
  anchor_state_feature_states[8])
{
  boolean_T ret;
  int featureIdx;
  boolean_T exitg1;

  // 'anyActiveAnchorFeatures:3' ret = false;
  ret = false;

  // 'anyActiveAnchorFeatures:4' for featureIdx = 1:numPointsPerAnchor
  featureIdx = 1;
  exitg1 = false;
  while ((!exitg1) && (featureIdx < 9)) {
    // 'anyActiveAnchorFeatures:5' if anchor_state.feature_states(featureIdx).status == 1 
    if (anchor_state_feature_states[featureIdx - 1].status == 1) {
      // 'anyActiveAnchorFeatures:6' ret = true;
      ret = true;
      exitg1 = true;
    } else {
      featureIdx++;
    }
  }

  return ret;
}

//
// Arguments    : const boolean_T x[3]
// Return Type  : boolean_T
//
static boolean_T b_any(const boolean_T x[3])
{
  boolean_T y;
  int k;
  boolean_T exitg1;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (!!x[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

//
// Arguments    : const double v[15]
//                double d[225]
// Return Type  : void
//
static void b_diag(const double v[15], double d[225])
{
  int j;
  memset(&d[0], 0, 225U * sizeof(double));
  for (j = 0; j < 15; j++) {
    d[j + 15 * j] = v[j];
  }
}

//
// Arguments    : double I[441]
// Return Type  : void
//
static void b_eye(double I[441])
{
  int k;
  memset(&I[0], 0, 441U * sizeof(double));
  for (k = 0; k < 21; k++) {
    I[k + 21 * k] = 1.0;
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double b_fprintf()
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[2] = { '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : emxArray_int32_T *idx
//                emxArray_real_T *x
//                int offset
//                int np
//                int nq
//                emxArray_int32_T *iwork
//                emxArray_real_T *xwork
// Return Type  : void
//
static void b_merge(emxArray_int32_T *idx, emxArray_real_T *x, int offset, int
                    np, int nq, emxArray_int32_T *iwork, emxArray_real_T *xwork)
{
  int n;
  int qend;
  int p;
  int iout;
  int exitg1;
  if (nq == 0) {
  } else {
    n = np + nq;
    for (qend = 0; qend + 1 <= n; qend++) {
      iwork->data[qend] = idx->data[offset + qend];
      xwork->data[qend] = x->data[offset + qend];
    }

    p = 0;
    n = np;
    qend = np + nq;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork->data[p] <= xwork->data[n]) {
        idx->data[iout] = iwork->data[p];
        x->data[iout] = xwork->data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx->data[iout] = iwork->data[n];
        x->data[iout] = xwork->data[n];
        if (n + 1 < qend) {
          n++;
        } else {
          n = (iout - p) + 1;
          while (p + 1 <= np) {
            idx->data[n + p] = iwork->data[p];
            x->data[n + p] = xwork->data[p];
            p++;
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

//
// Arguments    : const double A[2]
//                const double B[4]
//                double y[2]
// Return Type  : void
//
static void b_mrdivide(const double A[2], const double B[4], double y[2])
{
  int r1;
  int r2;
  double a21;
  if (fabs(B[1]) > fabs(B[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }

  a21 = B[r2] / B[r1];
  y[r1] = A[0] / B[r1];
  y[r2] = (A[1] - y[r1] * B[2 + r1]) / (B[2 + r2] - a21 * B[2 + r1]);
  y[r1] -= y[r2] * a21;
}

//
// Arguments    : const double x[4]
// Return Type  : double
//
static double b_norm(const double x[4])
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 4; k++) {
    absxk = fabs(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0 + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

//
// ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
// Arguments    : void
// Return Type  : void
//
static void b_ros_error()
{
  char cv22[14];
  int i22;
  static const char cv23[14] = { 'i', 'n', 'c', 'o', 'n', 's', 'i', 's', 't',
    'e', 'n', 'c', 'y', '\x00' };

  // 'ros_error:4' if coder.target('MATLAB')
  // 'ros_error:6' elseif ~coder.target('MEX')
  // 'ros_error:7' coder.cinclude('<ros/console.h>')
  // 'ros_error:8' coder.ceval('ROS_ERROR', [str, 0], varargin{:});
  for (i22 = 0; i22 < 14; i22++) {
    cv22[i22] = cv23[i22];
  }

  ROS_ERROR(cv22);
}

//
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                int varargin_2
//                int varargin_3
// Return Type  : void
//
static void b_ros_info(int varargin_1, int varargin_2, int varargin_3)
{
  char cv14[44];
  int i15;
  static const char cv15[44] = { 'F', 'i', 'x', 'i', 'n', 'g', ' ', 'f', 'e',
    'a', 't', 'u', 'r', 'e', ' ', '%', 'i', ' ', '(', 'f', 'e', 'a', 't', 'u',
    'r', 'e', ' ', '%', 'i', ' ', 'o', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r',
    ' ', '%', 'i', ')', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_info:5' if coder.target('MATLAB')
  // 'ros_info:9' elseif ~coder.target('MEX')
  // 'ros_info:10' coder.cinclude('<ros/console.h>')
  // 'ros_info:11' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_info:12' coder.ceval('ROS_INFO', [str, 0], varargin{:});
    for (i15 = 0; i15 < 44; i15++) {
      cv14[i15] = cv15[i15];
    }

    ROS_INFO(cv14, varargin_1, varargin_2, varargin_3);
  }
}

//
// ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
// Arguments    : int varargin_1
//                int varargin_2
//                int varargin_3
// Return Type  : void
//
static void b_ros_warn(int varargin_1, int varargin_2, int varargin_3)
{
  char cv18[54];
  int i20;
  static const char cv19[54] = { 'F', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%',
    'i', ' ', '(', '%', 'i', ' ', 'o', 'n', ' ', '%', 'i', ')', ' ', 'i', 's',
    ' ', 'b', 'e', 'h', 'i', 'n', 'd', ' ', 'i', 't', 's', ' ', 'a', 'n', 'c',
    'h', 'o', 'r', ',', ' ', 'r', 'e', 'j', 'e', 'c', 't', 'i', 'n', 'g', '\x00'
  };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_warn:6' if coder.target('MATLAB')
  // 'ros_warn:10' elseif ~coder.target('MEX')
  // 'ros_warn:11' coder.cinclude('<ros/console.h>')
  // 'ros_warn:12' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_warn:13' coder.ceval('ROS_WARN', [str, 0], varargin{:});
    for (i20 = 0; i20 < 54; i20++) {
      cv18[i20] = cv19[i20];
    }

    ROS_WARN(cv18, varargin_1, varargin_2, varargin_3);
  }
}

//
// Arguments    : double x[8]
//                int idx[8]
// Return Type  : void
//
static void b_sort(double x[8], int idx[8])
{
  int b_m;
  double x4[4];
  signed char idx4[4];
  double xwork[8];
  int nNaNs;
  int ib;
  int k;
  int bLen;
  int nPairs;
  int i4;
  signed char perm[4];
  int iwork[8];
  for (b_m = 0; b_m < 8; b_m++) {
    idx[b_m] = 0;
  }

  for (b_m = 0; b_m < 4; b_m++) {
    x4[b_m] = 0.0;
    idx4[b_m] = 0;
  }

  memset(&xwork[0], 0, sizeof(double) << 3);
  nNaNs = -7;
  ib = 0;
  for (k = 0; k < 8; k++) {
    if (rtIsNaN(x[k])) {
      idx[-nNaNs] = k + 1;
      xwork[-nNaNs] = x[k];
      nNaNs++;
    } else {
      ib++;
      idx4[ib - 1] = (signed char)(k + 1);
      x4[ib - 1] = x[k];
      if (ib == 4) {
        ib = (k - nNaNs) - 10;
        if (x4[0] <= x4[1]) {
          b_m = 1;
          bLen = 2;
        } else {
          b_m = 2;
          bLen = 1;
        }

        if (x4[2] <= x4[3]) {
          nPairs = 3;
          i4 = 4;
        } else {
          nPairs = 4;
          i4 = 3;
        }

        if (x4[b_m - 1] <= x4[nPairs - 1]) {
          if (x4[bLen - 1] <= x4[nPairs - 1]) {
            perm[0] = (signed char)b_m;
            perm[1] = (signed char)bLen;
            perm[2] = (signed char)nPairs;
            perm[3] = (signed char)i4;
          } else if (x4[bLen - 1] <= x4[i4 - 1]) {
            perm[0] = (signed char)b_m;
            perm[1] = (signed char)nPairs;
            perm[2] = (signed char)bLen;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)b_m;
            perm[1] = (signed char)nPairs;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)bLen;
          }
        } else if (x4[b_m - 1] <= x4[i4 - 1]) {
          if (x4[bLen - 1] <= x4[i4 - 1]) {
            perm[0] = (signed char)nPairs;
            perm[1] = (signed char)b_m;
            perm[2] = (signed char)bLen;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)nPairs;
            perm[1] = (signed char)b_m;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)bLen;
          }
        } else {
          perm[0] = (signed char)nPairs;
          perm[1] = (signed char)i4;
          perm[2] = (signed char)b_m;
          perm[3] = (signed char)bLen;
        }

        idx[ib] = idx4[perm[0] - 1];
        idx[ib + 1] = idx4[perm[1] - 1];
        idx[ib + 2] = idx4[perm[2] - 1];
        idx[ib + 3] = idx4[perm[3] - 1];
        x[ib] = x4[perm[0] - 1];
        x[ib + 1] = x4[perm[1] - 1];
        x[ib + 2] = x4[perm[2] - 1];
        x[ib + 3] = x4[perm[3] - 1];
        ib = 0;
      }
    }
  }

  if (ib > 0) {
    for (b_m = 0; b_m < 4; b_m++) {
      perm[b_m] = 0;
    }

    if (ib == 1) {
      perm[0] = 1;
    } else if (ib == 2) {
      if (x4[0] <= x4[1]) {
        perm[0] = 1;
        perm[1] = 2;
      } else {
        perm[0] = 2;
        perm[1] = 1;
      }
    } else if (x4[0] <= x4[1]) {
      if (x4[1] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 2;
        perm[2] = 3;
      } else if (x4[0] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 3;
        perm[2] = 2;
      } else {
        perm[0] = 3;
        perm[1] = 1;
        perm[2] = 2;
      }
    } else if (x4[0] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 1;
      perm[2] = 3;
    } else if (x4[1] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 3;
      perm[2] = 1;
    } else {
      perm[0] = 3;
      perm[1] = 2;
      perm[2] = 1;
    }

    for (k = 1; k <= ib; k++) {
      idx[(k - nNaNs) - ib] = idx4[perm[k - 1] - 1];
      x[(k - nNaNs) - ib] = x4[perm[k - 1] - 1];
    }
  }

  b_m = (nNaNs + 7) >> 1;
  for (k = 1; k <= b_m; k++) {
    ib = idx[k - nNaNs];
    idx[k - nNaNs] = idx[8 - k];
    idx[8 - k] = ib;
    x[k - nNaNs] = xwork[8 - k];
    x[8 - k] = xwork[k - nNaNs];
  }

  if (((nNaNs + 7) & 1) != 0) {
    x[(b_m - nNaNs) + 1] = xwork[(b_m - nNaNs) + 1];
  }

  if (1 - nNaNs > 1) {
    for (b_m = 0; b_m < 8; b_m++) {
      iwork[b_m] = 0;
    }

    nPairs = (1 - nNaNs) >> 2;
    bLen = 4;
    while (nPairs > 1) {
      if ((nPairs & 1) != 0) {
        nPairs--;
        ib = bLen * nPairs;
        b_m = 1 - (nNaNs + ib);
        if (b_m > bLen) {
          merge(idx, x, ib, bLen, b_m - bLen, iwork, xwork);
        }
      }

      ib = bLen << 1;
      nPairs >>= 1;
      for (k = 1; k <= nPairs; k++) {
        merge(idx, x, (k - 1) * ib, bLen, bLen, iwork, xwork);
      }

      bLen = ib;
    }

    if (1 - nNaNs > bLen) {
      merge(idx, x, 0, bLen, 1 - (nNaNs + bLen), iwork, xwork);
    }
  }
}

//
// Arguments    : double A[30]
//                double tau[5]
//                int jpvt[5]
// Return Type  : void
//
static void b_xgeqp3(double A[30], double tau[5], int jpvt[5])
{
  double vn1[5];
  double vn2[5];
  int k;
  int iy;
  double work[5];
  double smax;
  double temp2;
  int itemp;
  double absxk;
  double t;
  int i;
  int i_i;
  int ix;
  int pvt;
  int i62;
  int i_ip1;
  int lastv;
  int lastc;
  boolean_T exitg2;
  int exitg1;
  k = 1;
  for (iy = 0; iy < 5; iy++) {
    jpvt[iy] = 1 + iy;
    work[iy] = 0.0;
    smax = 0.0;
    temp2 = 2.2250738585072014E-308;
    for (itemp = k; itemp <= k + 5; itemp++) {
      absxk = fabs(A[itemp - 1]);
      if (absxk > temp2) {
        t = temp2 / absxk;
        smax = 1.0 + smax * t * t;
        temp2 = absxk;
      } else {
        t = absxk / temp2;
        smax += t * t;
      }
    }

    smax = temp2 * sqrt(smax);
    vn1[iy] = smax;
    vn2[iy] = vn1[iy];
    k += 6;
  }

  for (i = 0; i < 5; i++) {
    i_i = i + i * 6;
    itemp = 0;
    if (5 - i > 1) {
      ix = i;
      smax = vn1[i];
      for (k = 1; k + 1 <= 5 - i; k++) {
        ix++;
        if (vn1[ix] > smax) {
          itemp = k;
          smax = vn1[ix];
        }
      }
    }

    pvt = i + itemp;
    if (pvt + 1 != i + 1) {
      ix = 6 * pvt;
      iy = 6 * i;
      for (k = 0; k < 6; k++) {
        smax = A[ix];
        A[ix] = A[iy];
        A[iy] = smax;
        ix++;
        iy++;
      }

      itemp = jpvt[pvt];
      jpvt[pvt] = jpvt[i];
      jpvt[i] = itemp;
      vn1[pvt] = vn1[i];
      vn2[pvt] = vn2[i];
    }

    absxk = A[i_i];
    temp2 = 0.0;
    smax = d_xnrm2(5 - i, A, i_i + 2);
    if (smax != 0.0) {
      smax = hypot(A[i_i], smax);
      if (A[i_i] >= 0.0) {
        smax = -smax;
      }

      if (fabs(smax) < 1.0020841800044864E-292) {
        itemp = 0;
        do {
          itemp++;
          i62 = i_i - i;
          for (k = i_i + 1; k + 1 <= i62 + 6; k++) {
            A[k] *= 9.9792015476736E+291;
          }

          smax *= 9.9792015476736E+291;
          absxk *= 9.9792015476736E+291;
        } while (!(fabs(smax) >= 1.0020841800044864E-292));

        smax = hypot(absxk, d_xnrm2(5 - i, A, i_i + 2));
        if (absxk >= 0.0) {
          smax = -smax;
        }

        temp2 = (smax - absxk) / smax;
        absxk = 1.0 / (absxk - smax);
        i62 = i_i - i;
        for (k = i_i + 1; k + 1 <= i62 + 6; k++) {
          A[k] *= absxk;
        }

        for (k = 1; k <= itemp; k++) {
          smax *= 1.0020841800044864E-292;
        }

        absxk = smax;
      } else {
        temp2 = (smax - A[i_i]) / smax;
        absxk = 1.0 / (A[i_i] - smax);
        i62 = i_i - i;
        for (k = i_i + 1; k + 1 <= i62 + 6; k++) {
          A[k] *= absxk;
        }

        absxk = smax;
      }
    }

    tau[i] = temp2;
    A[i_i] = absxk;
    if (i + 1 < 5) {
      absxk = A[i_i];
      A[i_i] = 1.0;
      i_ip1 = (i + (i + 1) * 6) + 1;
      if (tau[i] != 0.0) {
        lastv = 6 - i;
        itemp = i_i - i;
        while ((lastv > 0) && (A[itemp + 5] == 0.0)) {
          lastv--;
          itemp--;
        }

        lastc = 4 - i;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          itemp = i_ip1 + (lastc - 1) * 6;
          k = itemp;
          do {
            exitg1 = 0;
            if (k <= (itemp + lastv) - 1) {
              if (A[k - 1] != 0.0) {
                exitg1 = 1;
              } else {
                k++;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = 0;
      }

      if (lastv > 0) {
        if (lastc == 0) {
        } else {
          for (iy = 1; iy <= lastc; iy++) {
            work[iy - 1] = 0.0;
          }

          iy = 0;
          i62 = i_ip1 + 6 * (lastc - 1);
          for (itemp = i_ip1; itemp <= i62; itemp += 6) {
            ix = i_i;
            smax = 0.0;
            pvt = (itemp + lastv) - 1;
            for (k = itemp; k <= pvt; k++) {
              smax += A[k - 1] * A[ix];
              ix++;
            }

            work[iy] += smax;
            iy++;
          }
        }

        if (-tau[i] == 0.0) {
        } else {
          itemp = i_ip1 - 1;
          pvt = 0;
          for (iy = 1; iy <= lastc; iy++) {
            if (work[pvt] != 0.0) {
              smax = work[pvt] * -tau[i];
              ix = i_i;
              i62 = lastv + itemp;
              for (k = itemp; k + 1 <= i62; k++) {
                A[k] += A[ix] * smax;
                ix++;
              }
            }

            pvt++;
            itemp += 6;
          }
        }
      }

      A[i_i] = absxk;
    }

    for (iy = i + 1; iy + 1 < 6; iy++) {
      itemp = (i + 6 * iy) + 1;
      if (vn1[iy] != 0.0) {
        smax = fabs(A[i + 6 * iy]) / vn1[iy];
        smax = 1.0 - smax * smax;
        if (smax < 0.0) {
          smax = 0.0;
        }

        temp2 = vn1[iy] / vn2[iy];
        temp2 = smax * (temp2 * temp2);
        if (temp2 <= 1.4901161193847656E-8) {
          smax = 0.0;
          if (5 - i == 1) {
            smax = fabs(A[itemp]);
          } else {
            temp2 = 2.2250738585072014E-308;
            pvt = (itemp - i) + 5;
            while (itemp + 1 <= pvt) {
              absxk = fabs(A[itemp]);
              if (absxk > temp2) {
                t = temp2 / absxk;
                smax = 1.0 + smax * t * t;
                temp2 = absxk;
              } else {
                t = absxk / temp2;
                smax += t * t;
              }

              itemp++;
            }

            smax = temp2 * sqrt(smax);
          }

          vn1[iy] = smax;
          vn2[iy] = vn1[iy];
        } else {
          vn1[iy] *= sqrt(smax);
        }
      }
    }
  }
}

//
// Arguments    : int n
//                const emxArray_real_T *x
//                int ix0
// Return Type  : double
//
static double b_xnrm2(int n, const emxArray_real_T *x, int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (n < 1) {
  } else if (n == 1) {
    y = fabs(x->data[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x->data[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0 + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrt(y);
  }

  return y;
}

//
// Arguments    : const double x[5]
// Return Type  : boolean_T
//
static boolean_T c_any(const double x[5])
{
  boolean_T y;
  int k;
  boolean_T exitg1;
  boolean_T b0;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 5)) {
    if ((x[k] == 0.0) || rtIsNaN(x[k])) {
      b0 = true;
    } else {
      b0 = false;
    }

    if (!b0) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

//
// Arguments    : double I[8281]
// Return Type  : void
//
static void c_eye(double I[8281])
{
  int k;
  memset(&I[0], 0, 8281U * sizeof(double));
  for (k = 0; k < 91; k++) {
    I[k + 91 * k] = 1.0;
  }
}

//
// Arguments    : double varargin_1
// Return Type  : double
//
static double c_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[8] = { 'q', 'v', ':', ' ', '%', 'f', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : const double x[2]
// Return Type  : double
//
static double c_norm(const double x[2])
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 2; k++) {
    absxk = fabs(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0 + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

//
// PREDICTMEASUREMENT Predict the measurement of a feature given in the left
// camera frame
//    Get the pixel coordinates where a feature given in the left camera
//    frame would be visible in both cameras. Depending on the
//    camera model used to calibrate the camera, the appropriate undistortion
//    is applied
// Arguments    : const double fp_l[3]
//                double c_stereoParams_CameraParameters
//                const double d_stereoParams_CameraParameters[2]
//                const double e_stereoParams_CameraParameters[2]
//                const double f_stereoParams_CameraParameters[3]
//                double g_stereoParams_CameraParameters
//                double h_stereoParams_CameraParameters
//                const double i_stereoParams_CameraParameters[2]
//                const double j_stereoParams_CameraParameters[2]
//                const double k_stereoParams_CameraParameters[3]
//                double l_stereoParams_CameraParameters
//                const double stereoParams_r_lr[3]
//                const double stereoParams_R_rl[9]
//                double h_d_l[2]
//                double h_d_r[2]
// Return Type  : void
//
static void c_predictMeasurementStereoDisto(const double fp_l[3], double
  c_stereoParams_CameraParameters, const double d_stereoParams_CameraParameters
  [2], const double e_stereoParams_CameraParameters[2], const double
  f_stereoParams_CameraParameters[3], double g_stereoParams_CameraParameters,
  double h_stereoParams_CameraParameters, const double
  i_stereoParams_CameraParameters[2], const double
  j_stereoParams_CameraParameters[2], const double
  k_stereoParams_CameraParameters[3], double l_stereoParams_CameraParameters,
  const double stereoParams_r_lr[3], const double stereoParams_R_rl[9], double
  h_d_l[2], double h_d_r[2])
{
  double fp_r[3];
  int i39;
  double d4;
  int i40;

  // 'predictMeasurementStereoDistorted:9' if stereoParams.CameraParameters1.DistortionModel == stereoParams.CameraParameters1.ATAN 
  if (g_stereoParams_CameraParameters == c_stereoParams_CameraParameters) {
    // 'predictMeasurementStereoDistorted:10' h_d_l = predictMeasurementDistortedAtan(fp_l, stereoParams.CameraParameters1); 
    predictMeasurementDistortedAtan(fp_l, d_stereoParams_CameraParameters,
      e_stereoParams_CameraParameters, f_stereoParams_CameraParameters, h_d_l);
  } else {
    // 'predictMeasurementStereoDistorted:11' else
    // 'predictMeasurementStereoDistorted:12' h_d_l = predictMeasurementDistortedPB(fp_l, stereoParams.CameraParameters1); 
    predictMeasurementDistortedPB(fp_l, d_stereoParams_CameraParameters,
      e_stereoParams_CameraParameters, f_stereoParams_CameraParameters, h_d_l);
  }

  // 'predictMeasurementStereoDistorted:15' fp_r = stereoParams.R_rl*fp_l - stereoParams.r_lr; 
  for (i39 = 0; i39 < 3; i39++) {
    d4 = 0.0;
    for (i40 = 0; i40 < 3; i40++) {
      d4 += stereoParams_R_rl[i39 + 3 * i40] * fp_l[i40];
    }

    fp_r[i39] = d4 - stereoParams_r_lr[i39];
  }

  // 'predictMeasurementStereoDistorted:18' if stereoParams.CameraParameters2.DistortionModel == stereoParams.CameraParameters2.ATAN 
  if (l_stereoParams_CameraParameters == h_stereoParams_CameraParameters) {
    // 'predictMeasurementStereoDistorted:19' h_d_r = predictMeasurementDistortedAtan(fp_r, stereoParams.CameraParameters2); 
    predictMeasurementDistortedAtan(fp_r, i_stereoParams_CameraParameters,
      j_stereoParams_CameraParameters, k_stereoParams_CameraParameters, h_d_r);
  } else {
    // 'predictMeasurementStereoDistorted:20' else
    // 'predictMeasurementStereoDistorted:21' h_d_r = predictMeasurementDistortedPB(fp_r, stereoParams.CameraParameters2); 
    predictMeasurementDistortedPB(fp_r, i_stereoParams_CameraParameters,
      j_stereoParams_CameraParameters, k_stereoParams_CameraParameters, h_d_r);
  }
}

//
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                int varargin_2
// Return Type  : void
//
static void c_ros_info(int varargin_1, int varargin_2)
{
  char cv30[44];
  int i28;
  static const char cv31[44] = { 'S', 'u', 'c', 'c', 'e', 's', 's', 'f', 'u',
    'l', 'l', 'y', ' ', 't', 'r', 'i', 'a', 'n', 'g', 'u', 'l', 'a', 't', 'e',
    'd', ' ', '%', 'd', ' ', 'o', 'f', ' ', '%', 'd', ' ', 'f', 'e', 'a', 't',
    'u', 'r', 'e', 's', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_info:5' if coder.target('MATLAB')
  // 'ros_info:9' elseif ~coder.target('MEX')
  // 'ros_info:10' coder.cinclude('<ros/console.h>')
  // 'ros_info:11' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_info:12' coder.ceval('ROS_INFO', [str, 0], varargin{:});
    for (i28 = 0; i28 < 44; i28++) {
      cv30[i28] = cv31[i28];
    }

    ROS_INFO(cv30, varargin_1, varargin_2);
  }
}

//
// ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
// Arguments    : void
// Return Type  : void
//
static void c_ros_warn()
{
  char cv20[44];
  int i21;
  static const char cv21[44] = { '1', '-', 'P', 'o', 'i', 'n', 't', ' ', 'R',
    'A', 'N', 'S', 'A', 'C', ' ', 'd', 'i', 'd', 'n', 't', ' ', 'f', 'i', 'n',
    'd', ' ', 'e', 'n', 'o', 'u', 'g', 'h', ' ', 'L', 'I', ' ', 'i', 'n', 'l',
    'i', 'e', 'r', 's', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_warn:6' if coder.target('MATLAB')
  // 'ros_warn:10' elseif ~coder.target('MEX')
  // 'ros_warn:11' coder.cinclude('<ros/console.h>')
  // 'ros_warn:12' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_warn:13' coder.ceval('ROS_WARN', [str, 0], varargin{:});
    for (i21 = 0; i21 < 44; i21++) {
      cv20[i21] = cv21[i21];
    }

    ROS_WARN(cv20);
  }
}

//
// Arguments    : emxArray_real_T *x
//                emxArray_int32_T *idx
// Return Type  : void
//
static void c_sort(emxArray_real_T *x, emxArray_int32_T *idx)
{
  int dim;
  dim = 2;
  if (x->size[0] != 1) {
    dim = 1;
  }

  d_sort(x, dim, idx);
}

//
// Arguments    : int n
//                const emxArray_real_T *x
//                int ix0
// Return Type  : double
//
static double c_xnrm2(int n, const emxArray_real_T *x, int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (n < 1) {
  } else if (n == 1) {
    y = fabs(x->data[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x->data[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0 + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrt(y);
  }

  return y;
}

//
// Arguments    : const struct_T x[5]
//                AnchorPose y[5]
// Return Type  : void
//
static void cast(const struct_T x[5], AnchorPose y[5])
{
  int j;
  int i;
  for (j = 0; j < 5; j++) {
    for (i = 0; i < 3; i++) {
      y[j].pos[i] = x[j].pos[i];
    }

    for (i = 0; i < 4; i++) {
      y[j].att[i] = x[j].att[i];
    }
  }
}

//
// Arguments    : const double a[3]
//                const double b[3]
//                double c[3]
// Return Type  : void
//
static void cross(const double a[3], const double b[3], double c[3])
{
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

//
// Arguments    : double I[8281]
// Return Type  : void
//
static void d_eye(double I[8281])
{
  int k;
  memset(&I[0], 0, 8281U * sizeof(double));
  for (k = 0; k < 91; k++) {
    I[k + 91 * k] = 1.0;
  }
}

//
// Arguments    : double varargin_1
// Return Type  : double
//
static double d_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[8] = { 'q', 'w', ':', ' ', '%', 'f', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1);
    fflush(filestar);
  }

  return nbytesint;
}

//
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
// Return Type  : void
//
static void d_ros_info(int varargin_1)
{
  char cv32[52];
  int i29;
  static const char cv33[52] = { 'I', 'n', 'i', 't', 'i', 'a', 'l', 'i', 'z',
    'i', 'n', 'g', ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ',', ' ',
    'w', 'h', 'i', 'c', 'h', ' ', 'w', 'a', 's', ' ', 't', 'h', 'e', ' ', 'o',
    'r', 'i', 'g', 'i', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_info:5' if coder.target('MATLAB')
  // 'ros_info:9' elseif ~coder.target('MEX')
  // 'ros_info:10' coder.cinclude('<ros/console.h>')
  // 'ros_info:11' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_info:12' coder.ceval('ROS_INFO', [str, 0], varargin{:});
    for (i29 = 0; i29 < 52; i29++) {
      cv32[i29] = cv33[i29];
    }

    ROS_INFO(cv32, varargin_1);
  }
}

//
// ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
// Arguments    : int varargin_1
//                double varargin_2
// Return Type  : void
//
static void d_ros_warn(int varargin_1, double varargin_2)
{
  char cv24[36];
  int i23;
  static const char cv25[36] = { 'F', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%',
    'i', ' ', 'i', 's', ' ', 'v', 'e', 'r', 'y', ' ', 'c', 'l', 'o', 's', 'e',
    '.', ' ', 'D', 'e', 'p', 't', 'h', ':', ' ', '%', 'f', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_warn:6' if coder.target('MATLAB')
  // 'ros_warn:10' elseif ~coder.target('MEX')
  // 'ros_warn:11' coder.cinclude('<ros/console.h>')
  // 'ros_warn:12' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_warn:13' coder.ceval('ROS_WARN', [str, 0], varargin{:});
    for (i23 = 0; i23 < 36; i23++) {
      cv24[i23] = cv25[i23];
    }

    ROS_WARN(cv24, varargin_1, varargin_2);
  }
}

//
// Arguments    : emxArray_real_T *x
//                int dim
//                emxArray_int32_T *idx
// Return Type  : void
//
static void d_sort(emxArray_real_T *x, int dim, emxArray_int32_T *idx)
{
  int i63;
  emxArray_real_T *vwork;
  int vstride;
  int x_idx_0;
  int j;
  emxArray_int32_T *iidx;
  if (dim <= 1) {
    i63 = x->size[0];
  } else {
    i63 = 1;
  }

  emxInit_real_T(&vwork, 1);
  vstride = vwork->size[0];
  vwork->size[0] = i63;
  emxEnsureCapacity((emxArray__common *)vwork, vstride, (int)sizeof(double));
  x_idx_0 = x->size[0];
  vstride = idx->size[0];
  idx->size[0] = x_idx_0;
  emxEnsureCapacity((emxArray__common *)idx, vstride, (int)sizeof(int));
  vstride = 1;
  x_idx_0 = 1;
  while (x_idx_0 <= dim - 1) {
    vstride *= x->size[0];
    x_idx_0 = 2;
  }

  j = 0;
  emxInit_int32_T1(&iidx, 1);
  while (j + 1 <= vstride) {
    for (x_idx_0 = 0; x_idx_0 + 1 <= i63; x_idx_0++) {
      vwork->data[x_idx_0] = x->data[j + x_idx_0 * vstride];
    }

    sortIdx(vwork, iidx);
    for (x_idx_0 = 0; x_idx_0 + 1 <= i63; x_idx_0++) {
      x->data[j + x_idx_0 * vstride] = vwork->data[x_idx_0];
      idx->data[j + x_idx_0 * vstride] = iidx->data[x_idx_0];
    }

    j++;
  }

  emxFree_int32_T(&iidx);
  emxFree_real_T(&vwork);
}

//
// Arguments    : int n
//                const double x[30]
//                int ix0
// Return Type  : double
//
static double d_xnrm2(int n, const double x[30], int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (n == 1) {
    y = fabs(x[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabs(x[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0 + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrt(y);
  }

  return y;
}

//
// Arguments    : const double x[36]
// Return Type  : double
//
static double det(const double x[36])
{
  double y;
  double A[36];
  signed char ipiv[6];
  int i36;
  int j;
  int c;
  int iy;
  int ix;
  double smax;
  int jy;
  double s;
  int b_j;
  int ijA;
  boolean_T isodd;
  memcpy(&A[0], &x[0], 36U * sizeof(double));
  for (i36 = 0; i36 < 6; i36++) {
    ipiv[i36] = (signed char)(1 + i36);
  }

  for (j = 0; j < 5; j++) {
    c = j * 7;
    iy = 0;
    ix = c;
    smax = fabs(A[c]);
    for (jy = 1; jy + 1 <= 6 - j; jy++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        iy = jy;
        smax = s;
      }
    }

    if (A[c + iy] != 0.0) {
      if (iy != 0) {
        ipiv[j] = (signed char)((j + iy) + 1);
        ix = j;
        iy += j;
        for (jy = 0; jy < 6; jy++) {
          smax = A[ix];
          A[ix] = A[iy];
          A[iy] = smax;
          ix += 6;
          iy += 6;
        }
      }

      i36 = (c - j) + 6;
      for (iy = c + 1; iy + 1 <= i36; iy++) {
        A[iy] /= A[c];
      }
    }

    iy = c;
    jy = c + 6;
    for (b_j = 1; b_j <= 5 - j; b_j++) {
      smax = A[jy];
      if (A[jy] != 0.0) {
        ix = c + 1;
        i36 = (iy - j) + 12;
        for (ijA = 7 + iy; ijA + 1 <= i36; ijA++) {
          A[ijA] += A[ix] * -smax;
          ix++;
        }
      }

      jy += 6;
      iy += 6;
    }
  }

  y = A[0];
  isodd = false;
  for (jy = 0; jy < 5; jy++) {
    y *= A[(jy + 6 * (jy + 1)) + 1];
    if (ipiv[jy] > 1 + jy) {
      isodd = !isodd;
    }
  }

  if (isodd) {
    y = -y;
  }

  return y;
}

//
// Arguments    : const double v[3]
//                double d[9]
// Return Type  : void
//
static void diag(const double v[3], double d[9])
{
  int j;
  memset(&d[0], 0, 9U * sizeof(double));
  for (j = 0; j < 3; j++) {
    d[j + 3 * j] = v[j];
  }
}

//
// Arguments    : double varargin_1
// Return Type  : double
//
static double e_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[9] = { 'q', 'a', 'o', ':', ' ', '%', 'f', '\x0a',
    '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1);
    fflush(filestar);
  }

  return nbytesint;
}

//
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
// Return Type  : void
//
static void e_ros_info(int varargin_1)
{
  char cv34[45];
  int i30;
  static const char cv35[45] = { 'F', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%',
    'd', ' ', 'i', 's', ' ', 't', 'o', 'o', ' ', 'f', 'a', 'r', ' ', 'a', 'w',
    'a', 'y', ' ', 't', 'o', ' ', 't', 'r', 'i', 'a', 'n', 'g', 'u', 'l', 'a',
    't', 'e', '.', '\\', 'n', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_info:5' if coder.target('MATLAB')
  // 'ros_info:9' elseif ~coder.target('MEX')
  // 'ros_info:10' coder.cinclude('<ros/console.h>')
  // 'ros_info:11' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_info:12' coder.ceval('ROS_INFO', [str, 0], varargin{:});
    for (i30 = 0; i30 < 45; i30++) {
      cv34[i30] = cv35[i30];
    }

    ROS_INFO(cv34, varargin_1);
  }
}

//
// ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
// Arguments    : signed char varargin_1
// Return Type  : void
//
static void e_ros_warn(signed char varargin_1)
{
  char cv26[37];
  int i25;
  static const char cv27[37] = { 'B', 'a', 'd', ' ', 't', 'r', 'i', 'a', 'n',
    'g', 'u', 'l', 'a', 't', 'i', 'o', 'n', ' ', '(', 'n', 'a', 'n', ')', ' ',
    'f', 'o', 'r', ' ', 'p', 'o', 'i', 'n', 't', ' ', '%', 'd', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_warn:6' if coder.target('MATLAB')
  // 'ros_warn:10' elseif ~coder.target('MEX')
  // 'ros_warn:11' coder.cinclude('<ros/console.h>')
  // 'ros_warn:12' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_warn:13' coder.ceval('ROS_WARN', [str, 0], varargin{:});
    for (i25 = 0; i25 < 37; i25++) {
      cv26[i25] = cv27[i25];
    }

    ROS_WARN(cv26, varargin_1);
  }
}

//
// Arguments    : int b
//                emxArray_int32_T *y
// Return Type  : void
//
static void eml_signed_integer_colon(int b, emxArray_int32_T *y)
{
  int n;
  int yk;
  int k;
  if (b < 1) {
    n = 0;
  } else {
    n = b;
  }

  yk = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = n;
  emxEnsureCapacity((emxArray__common *)y, yk, (int)sizeof(int));
  if (n > 0) {
    y->data[0] = 1;
    yk = 1;
    for (k = 2; k <= n; k++) {
      yk++;
      y->data[k - 1] = yk;
    }
  }
}

//
// Arguments    : emxArray__common *emxArray
//                int oldNumel
//                int elementSize
// Return Type  : void
//
static void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize)
{
  int newNumel;
  int i;
  void *newData;
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      i <<= 1;
    }

    newData = calloc((unsigned int)i, (unsigned int)elementSize);
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, (unsigned int)(elementSize * oldNumel));
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

//
// Arguments    : emxArray_int32_T **pEmxArray
// Return Type  : void
//
static void emxFree_int32_T(emxArray_int32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int32_T *)NULL) {
    if (((*pEmxArray)->data != (int *)NULL) && (*pEmxArray)->canFreeData) {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_int32_T *)NULL;
  }
}

//
// Arguments    : emxArray_real_T **pEmxArray
// Return Type  : void
//
static void emxFree_real_T(emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (double *)NULL) && (*pEmxArray)->canFreeData) {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

//
// Arguments    : emxArray_int32_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
static void emxInit_int32_T(emxArray_int32_T **pEmxArray, int b_numDimensions)
{
  emxArray_int32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int32_T *)malloc(sizeof(emxArray_int32_T));
  emxArray = *pEmxArray;
  emxArray->data = (int *)NULL;
  emxArray->numDimensions = b_numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * b_numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < b_numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : emxArray_int32_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
static void emxInit_int32_T1(emxArray_int32_T **pEmxArray, int b_numDimensions)
{
  emxArray_int32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int32_T *)malloc(sizeof(emxArray_int32_T));
  emxArray = *pEmxArray;
  emxArray->data = (int *)NULL;
  emxArray->numDimensions = b_numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * b_numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < b_numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : emxArray_real_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
static void emxInit_real_T(emxArray_real_T **pEmxArray, int b_numDimensions)
{
  emxArray_real_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
  emxArray = *pEmxArray;
  emxArray->data = (double *)NULL;
  emxArray->numDimensions = b_numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * b_numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < b_numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : emxArray_real_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
static void emxInit_real_T1(emxArray_real_T **pEmxArray, int b_numDimensions)
{
  emxArray_real_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
  emxArray = *pEmxArray;
  emxArray->data = (double *)NULL;
  emxArray->numDimensions = b_numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * b_numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < b_numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : double I[9]
// Return Type  : void
//
static void eye(double I[9])
{
  int k;
  memset(&I[0], 0, 9U * sizeof(double));
  for (k = 0; k < 3; k++) {
    I[k + 3 * k] = 1.0;
  }
}

//
// Arguments    : double varargin_1
// Return Type  : double
//
static double f_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[9] = { 'q', 'w', 'o', ':', ' ', '%', 'f', '\x0a',
    '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1);
    fflush(filestar);
  }

  return nbytesint;
}

//
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                double varargin_2
//                double varargin_3
// Return Type  : void
//
static void f_ros_info(int varargin_1, double varargin_2, double varargin_3)
{
  char cv42[51];
  int i34;
  static const char cv43[51] = { 'F', 'o', 'r', 'c', 'i', 'n', 'g', ' ', 'a',
    'c', 't', 'i', 'v', 'a', 't', 'i', 'o', 'n', ' ', 'o', 'f', ' ', 'f', 'e',
    'a', 't', 'u', 'r', 'e', ' ', '%', 'i', ' ', '(', '%', 'i', ' ', 'o', 'n',
    ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ')', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_info:5' if coder.target('MATLAB')
  // 'ros_info:9' elseif ~coder.target('MEX')
  // 'ros_info:10' coder.cinclude('<ros/console.h>')
  // 'ros_info:11' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_info:12' coder.ceval('ROS_INFO', [str, 0], varargin{:});
    for (i34 = 0; i34 < 51; i34++) {
      cv42[i34] = cv43[i34];
    }

    ROS_INFO(cv42, varargin_1, varargin_2, varargin_3);
  }
}

//
// ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
// Arguments    : int varargin_1
//                double varargin_2
// Return Type  : void
//
static void f_ros_warn(int varargin_1, double varargin_2)
{
  char cv28[49];
  int i27;
  static const char cv29[49] = { 'F', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%',
    'i', ' ', 'i', 's', ' ', 't', 'r', 'i', 'a', 'n', 'g', 'u', 'l', 'a', 't',
    'e', 'd', ' ', 'v', 'e', 'r', 'y', ' ', 'c', 'l', 'o', 's', 'e', '.', ' ',
    'D', 'e', 'p', 't', 'h', ':', ' ', '%', 'f', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_warn:6' if coder.target('MATLAB')
  // 'ros_warn:10' elseif ~coder.target('MEX')
  // 'ros_warn:11' coder.cinclude('<ros/console.h>')
  // 'ros_warn:12' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_warn:13' coder.ceval('ROS_WARN', [str, 0], varargin{:});
    for (i27 = 0; i27 < 49; i27++) {
      cv28[i27] = cv29[i27];
    }

    ROS_WARN(cv28, varargin_1, varargin_2);
  }
}

//
// Arguments    : FILE * *f
//                boolean_T *a
// Return Type  : void
//
static void fileManager(FILE * *f, boolean_T *a)
{
  *f = stdout;
  *a = true;
}

//
// Arguments    : double varargin_1
// Return Type  : double
//
static double g_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[11] = { 'q', 'R', '_', 'c', 'i', ':', ' ', '%', 'f',
    '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1);
    fflush(filestar);
  }

  return nbytesint;
}

//
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
// Return Type  : void
//
static void g_ros_info(int varargin_1)
{
  char cv44[27];
  int i35;
  static const char cv45[27] = { 'R', 'e', 'q', 'u', 'e', 's', 't', 'i', 'n',
    'g', ' ', '%', 'd', ' ', 'n', 'e', 'w', ' ', 'f', 'e', 'a', 't', 'u', 'r',
    'e', 's', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_info:5' if coder.target('MATLAB')
  // 'ros_info:9' elseif ~coder.target('MEX')
  // 'ros_info:10' coder.cinclude('<ros/console.h>')
  // 'ros_info:11' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_info:12' coder.ceval('ROS_INFO', [str, 0], varargin{:});
    for (i35 = 0; i35 < 27; i35++) {
      cv44[i35] = cv45[i35];
    }

    ROS_INFO(cv44, varargin_1);
  }
}

//
// ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
// Arguments    : double varargin_1
// Return Type  : void
//
static void g_ros_warn(double varargin_1)
{
  char cv36[62];
  int i31;
  static const char cv37[62] = { 'G', 'o', 't', ' ', '%', 'd', ' ', 'n', 'e',
    'w', ' ', 'f', 'e', 'a', 'u', 't', 'u', 'r', 'e', 's', ' ', 'b', 'u', 't',
    ' ', 'n', 'o', 't', ' ', 'e', 'n', 'o', 'u', 'g', 'h', ' ', 'f', 'o', 'r',
    ' ', 'a', ' ', 'n', 'e', 'w', ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', '(',
    'm', 'i', 'n', ' ', '%', 'd', ')', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_warn:6' if coder.target('MATLAB')
  // 'ros_warn:10' elseif ~coder.target('MEX')
  // 'ros_warn:11' coder.cinclude('<ros/console.h>')
  // 'ros_warn:12' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_warn:13' coder.ceval('ROS_WARN', [str, 0], varargin{:});
    for (i31 = 0; i31 < 62; i31++) {
      cv36[i31] = cv37[i31];
    }

    ROS_WARN(cv36, varargin_1, 4);
  }
}

//
// getAnchorPoses Get the anchor poses in the world frame
// Arguments    : const double xt_origin_pos[3]
//                const double xt_origin_att[4]
//                const f_struct_T xt_anchor_states[5]
//                struct_T anchor_poses[5]
// Return Type  : void
//
static void getAnchorPoses(const double xt_origin_pos[3], const double
  xt_origin_att[4], const f_struct_T xt_anchor_states[5], struct_T anchor_poses
  [5])
{
  double R_ow[9];
  int anchorIdx;
  double b_xt_anchor_states[9];
  double c_xt_anchor_states[9];
  int i43;
  double d6;
  int i44;
  int i45;

  // 'getAnchorPoses:4' r_ow = xt.origin.pos;
  // 'getAnchorPoses:5' R_ow = RotFromQuatJ(xt.origin.att);
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
  // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
  // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
  R_ow[0] = ((xt_origin_att[0] * xt_origin_att[0] - xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[3] = 2.0 * (xt_origin_att[0] * xt_origin_att[1] + xt_origin_att[2] *
                   xt_origin_att[3]);
  R_ow[6] = 2.0 * (xt_origin_att[0] * xt_origin_att[2] - xt_origin_att[1] *
                   xt_origin_att[3]);
  R_ow[1] = 2.0 * (xt_origin_att[0] * xt_origin_att[1] - xt_origin_att[2] *
                   xt_origin_att[3]);
  R_ow[4] = ((-(xt_origin_att[0] * xt_origin_att[0]) + xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[7] = 2.0 * (xt_origin_att[1] * xt_origin_att[2] + xt_origin_att[0] *
                   xt_origin_att[3]);
  R_ow[2] = 2.0 * (xt_origin_att[0] * xt_origin_att[2] + xt_origin_att[1] *
                   xt_origin_att[3]);
  R_ow[5] = 2.0 * (xt_origin_att[1] * xt_origin_att[2] - xt_origin_att[0] *
                   xt_origin_att[3]);
  R_ow[8] = ((-(xt_origin_att[0] * xt_origin_att[0]) - xt_origin_att[1] *
              xt_origin_att[1]) + xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];

  // 'getAnchorPoses:7' anchor_pose.pos = zeros(3,1);
  // 'getAnchorPoses:8' anchor_pose.att = zeros(4,1);
  // 'getAnchorPoses:10' anchor_poses = repmat(anchor_pose, numAnchors, 1);
  // 'getAnchorPoses:12' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    // 'getAnchorPoses:13' anchor_poses(anchorIdx).pos = r_ow + R_ow' * xt.anchor_states(anchorIdx).pos; 
    // 'getAnchorPoses:14' anchor_poses(anchorIdx).att = QuatFromRotJ(RotFromQuatJ(xt.anchor_states(anchorIdx).att) * R_ow); 
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
    // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
    // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
    c_xt_anchor_states[0] = ((xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    c_xt_anchor_states[3] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    c_xt_anchor_states[6] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    c_xt_anchor_states[1] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    c_xt_anchor_states[4] = ((-(xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0]) + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    c_xt_anchor_states[7] = 2.0 * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    c_xt_anchor_states[2] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    c_xt_anchor_states[5] = 2.0 * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    c_xt_anchor_states[8] = ((-(xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0]) - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    for (i43 = 0; i43 < 3; i43++) {
      d6 = 0.0;
      for (i44 = 0; i44 < 3; i44++) {
        d6 += R_ow[i44 + 3 * i43] * xt_anchor_states[anchorIdx].pos[i44];
        b_xt_anchor_states[i43 + 3 * i44] = 0.0;
        for (i45 = 0; i45 < 3; i45++) {
          b_xt_anchor_states[i43 + 3 * i44] += c_xt_anchor_states[i43 + 3 * i45]
            * R_ow[i45 + 3 * i44];
        }
      }

      anchor_poses[anchorIdx].pos[i43] = xt_origin_pos[i43] + d6;
    }

    QuatFromRotJ(b_xt_anchor_states, anchor_poses[anchorIdx].att);
  }
}

//
// GETJACOBIANANDRESIDUAL Get Jacobian H and residual r
//    Uses the standard camera model
//
//  INPUT ARGUMENTS:
//  - xt:                   The current state
//  - errorStateSize:       The size of the error state
//  - z_all_l:              The feature points in the left camera (2N x 1)
//  - map:                  Map of the estimated feature points (3 x N)
//  - indMeas:              The indices of the valid feature points
//  - cameraparams:         A struct with the fields
//     - focal length, center of projection, radial distortion parameters of
//     both cameras
//     - r_lr:              Translation of right camera in left camera frame
//     - R_lr:              Orientation of right camera in left camera frame
//
//  where N is number of points in the image
//
//  OUTPUT ARGUMENTS:
//  - r:    The residual (residualDim*M x 1)
//  - H_xc: The Jacobian of the measurement function with respect to the camera states (residualDim*M x length(xt))
//
//  where M =  length(indMeas) is the number of valid points in the image
// Arguments    : const double xt_robot_state_pos[3]
//                const double xt_robot_state_att[4]
//                const f_struct_T xt_anchor_states[5]
//                const double z_all_l[80]
//                const boolean_T b_status[40]
//                const double cameraparams_FocalLength[2]
//                const double cameraparams_PrincipalPoint[2]
//                double noiseParameters_image_noise
//                emxArray_real_T *r
//                emxArray_real_T *H
//                emxArray_real_T *R
// Return Type  : void
//
static void getH_R_res(const double xt_robot_state_pos[3], const double
  xt_robot_state_att[4], const f_struct_T xt_anchor_states[5], const double
  z_all_l[80], const boolean_T b_status[40], const double
  cameraparams_FocalLength[2], const double cameraparams_PrincipalPoint[2],
  double noiseParameters_image_noise, emxArray_real_T *r, emxArray_real_T *H,
  emxArray_real_T *R)
{
  double fx;
  double fy;
  int n;
  int k;
  double R_cw[9];
  int i16;
  emxArray_real_T *b_h_u;
  double res_idx;
  int anchorIdx;
  double anchorRot[9];
  int featureIdx;
  long i17;
  double z_curr_l[2];
  double h_c_n_l[2];
  double varargin_2;
  double h_u_To_h_ci_l[6];
  double h_ci_l_To_r_wc[9];
  double h_ci_l_To_R_cw[9];
  double h_ci_l_To_anchorPos[9];
  double h_ci_l_To_anchorRot[9];
  double b_xt_anchor_states[3];
  double h_ci_l_To_rho[3];
  int i18;
  double b_R_cw[9];
  double dv0[9];
  int outsize_idx_1;

  // 'getH_R_res:26' fx = cameraparams.FocalLength(1);
  fx = cameraparams_FocalLength[0];

  // 'getH_R_res:27' fy = cameraparams.FocalLength(2);
  fy = cameraparams_FocalLength[1];

  // 'getH_R_res:29' numErrorStatesPerAnchor = 6 + numPointsPerAnchor;
  // 'getH_R_res:31' numMeas = nnz(status);
  n = 0;
  for (k = 0; k < 40; k++) {
    if (b_status[k]) {
      n++;
    }
  }

  // 'getH_R_res:32' residualDim = 2;
  // 'getH_R_res:34' R_cw = RotFromQuatJ(xt.robot_state.att);
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
  // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
  // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
  R_cw[0] = ((xt_robot_state_att[0] * xt_robot_state_att[0] -
              xt_robot_state_att[1] * xt_robot_state_att[1]) -
             xt_robot_state_att[2] * xt_robot_state_att[2]) +
    xt_robot_state_att[3] * xt_robot_state_att[3];
  R_cw[3] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[1] +
                   xt_robot_state_att[2] * xt_robot_state_att[3]);
  R_cw[6] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[2] -
                   xt_robot_state_att[1] * xt_robot_state_att[3]);
  R_cw[1] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[1] -
                   xt_robot_state_att[2] * xt_robot_state_att[3]);
  R_cw[4] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) +
              xt_robot_state_att[1] * xt_robot_state_att[1]) -
             xt_robot_state_att[2] * xt_robot_state_att[2]) +
    xt_robot_state_att[3] * xt_robot_state_att[3];
  R_cw[7] = 2.0 * (xt_robot_state_att[1] * xt_robot_state_att[2] +
                   xt_robot_state_att[0] * xt_robot_state_att[3]);
  R_cw[2] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[2] +
                   xt_robot_state_att[1] * xt_robot_state_att[3]);
  R_cw[5] = 2.0 * (xt_robot_state_att[1] * xt_robot_state_att[2] -
                   xt_robot_state_att[0] * xt_robot_state_att[3]);
  R_cw[8] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) -
              xt_robot_state_att[1] * xt_robot_state_att[1]) +
             xt_robot_state_att[2] * xt_robot_state_att[2]) +
    xt_robot_state_att[3] * xt_robot_state_att[3];

  // 'getH_R_res:35' r_wc = xt.robot_state.pos;
  // 'getH_R_res:37' H = zeros(numMeas*residualDim, numStates + numAnchors*numErrorStatesPerAnchor); 
  i16 = H->size[0] * H->size[1];
  H->size[0] = (int)((double)n * 2.0);
  H->size[1] = 91;
  emxEnsureCapacity((emxArray__common *)H, i16, (int)sizeof(double));
  k = (int)((double)n * 2.0) * 91;
  for (i16 = 0; i16 < k; i16++) {
    H->data[i16] = 0.0;
  }

  // 'getH_R_res:38' z = zeros(numMeas*residualDim, 1);
  i16 = r->size[0];
  r->size[0] = (int)((double)n * 2.0);
  emxEnsureCapacity((emxArray__common *)r, i16, (int)sizeof(double));
  k = (int)((double)n * 2.0);
  for (i16 = 0; i16 < k; i16++) {
    r->data[i16] = 0.0;
  }

  emxInit_real_T(&b_h_u, 1);

  // 'getH_R_res:39' h_u = zeros(numMeas*residualDim, 1);
  i16 = b_h_u->size[0];
  b_h_u->size[0] = (int)((double)n * 2.0);
  emxEnsureCapacity((emxArray__common *)b_h_u, i16, (int)sizeof(double));
  k = (int)((double)n * 2.0);
  for (i16 = 0; i16 < k; i16++) {
    b_h_u->data[i16] = 0.0;
  }

  // 'getH_R_res:41' res_idx = 1;
  res_idx = 1.0;

  // 'getH_R_res:43' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    // 'getH_R_res:44' anchorPos = xt.anchor_states(anchorIdx).pos;
    // 'getH_R_res:45' anchorRot = RotFromQuatJ(xt.anchor_states(anchorIdx).att); 
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
    // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
    // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
    anchorRot[0] = ((xt_anchor_states[anchorIdx].att[0] *
                     xt_anchor_states[anchorIdx].att[0] -
                     xt_anchor_states[anchorIdx].att[1] *
                     xt_anchor_states[anchorIdx].att[1]) -
                    xt_anchor_states[anchorIdx].att[2] *
                    xt_anchor_states[anchorIdx].att[2]) +
      xt_anchor_states[anchorIdx].att[3] * xt_anchor_states[anchorIdx].att[3];
    anchorRot[3] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
                          xt_anchor_states[anchorIdx].att[1] +
                          xt_anchor_states[anchorIdx].att[2] *
                          xt_anchor_states[anchorIdx].att[3]);
    anchorRot[6] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
                          xt_anchor_states[anchorIdx].att[2] -
                          xt_anchor_states[anchorIdx].att[1] *
                          xt_anchor_states[anchorIdx].att[3]);
    anchorRot[1] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
                          xt_anchor_states[anchorIdx].att[1] -
                          xt_anchor_states[anchorIdx].att[2] *
                          xt_anchor_states[anchorIdx].att[3]);
    anchorRot[4] = ((-(xt_anchor_states[anchorIdx].att[0] *
                       xt_anchor_states[anchorIdx].att[0]) +
                     xt_anchor_states[anchorIdx].att[1] *
                     xt_anchor_states[anchorIdx].att[1]) -
                    xt_anchor_states[anchorIdx].att[2] *
                    xt_anchor_states[anchorIdx].att[2]) +
      xt_anchor_states[anchorIdx].att[3] * xt_anchor_states[anchorIdx].att[3];
    anchorRot[7] = 2.0 * (xt_anchor_states[anchorIdx].att[1] *
                          xt_anchor_states[anchorIdx].att[2] +
                          xt_anchor_states[anchorIdx].att[0] *
                          xt_anchor_states[anchorIdx].att[3]);
    anchorRot[2] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
                          xt_anchor_states[anchorIdx].att[2] +
                          xt_anchor_states[anchorIdx].att[1] *
                          xt_anchor_states[anchorIdx].att[3]);
    anchorRot[5] = 2.0 * (xt_anchor_states[anchorIdx].att[1] *
                          xt_anchor_states[anchorIdx].att[2] -
                          xt_anchor_states[anchorIdx].att[0] *
                          xt_anchor_states[anchorIdx].att[3]);
    anchorRot[8] = ((-(xt_anchor_states[anchorIdx].att[0] *
                       xt_anchor_states[anchorIdx].att[0]) -
                     xt_anchor_states[anchorIdx].att[1] *
                     xt_anchor_states[anchorIdx].att[1]) +
                    xt_anchor_states[anchorIdx].att[2] *
                    xt_anchor_states[anchorIdx].att[2]) +
      xt_anchor_states[anchorIdx].att[3] * xt_anchor_states[anchorIdx].att[3];

    // 'getH_R_res:47' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      // 'getH_R_res:49' idx = xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx; 
      // 'getH_R_res:51' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status && status(idx) 
      if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status != 0) &&
          b_status[xt_anchor_states[anchorIdx].feature_states[featureIdx].
          status_idx - 1]) {
        // 'getH_R_res:52' z_curr_l = z_all_l((idx-1)*2 + int32(1:2));
        i17 = xt_anchor_states[anchorIdx].feature_states[featureIdx].status_idx
          - 1L;
        if (i17 > 2147483647L) {
          i17 = 2147483647L;
        } else {
          if (i17 < -2147483648L) {
            i17 = -2147483648L;
          }
        }

        i16 = (int)i17;
        if (i16 > 1073741823) {
          k = MAX_int32_T;
        } else if (i16 <= -1073741824) {
          k = MIN_int32_T;
        } else {
          k = i16 << 1;
        }

        // 'getH_R_res:54' h_ci_l = xt.anchor_states(anchorIdx).feature_states(featureIdx).scaled_map_point; 
        // 'getH_R_res:55' rho = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth; 
        // 'getH_R_res:56' m = xt.anchor_states(anchorIdx).feature_states(featureIdx).m; 
        // 'getH_R_res:58' h_u_l = predictMeasurementMono(h_ci_l, cameraparams); 
        // predictMeasurementLeft Predict the measurement of a feature given in the left 
        // camera frame
        //    Get the normalized pixel coordinates where a feature given in the left camera 
        //    frame
        // 'predictMeasurementMono:6' fx = cameraparams.FocalLength(1);
        // 'predictMeasurementMono:7' fy = cameraparams.FocalLength(2);
        // 'predictMeasurementMono:8' Cx = cameraparams.PrincipalPoint(1);
        // 'predictMeasurementMono:9' Cy = cameraparams.PrincipalPoint(2);
        // 'predictMeasurementMono:11' h_c_n_l = fp(1:2)/fp(3);
        for (i16 = 0; i16 < 2; i16++) {
          i17 = (long)k + (1 + i16);
          if (i17 > 2147483647L) {
            i17 = 2147483647L;
          } else {
            if (i17 < -2147483648L) {
              i17 = -2147483648L;
            }
          }

          z_curr_l[i16] = z_all_l[(int)i17 - 1];
          h_c_n_l[i16] = xt_anchor_states[anchorIdx].feature_states[featureIdx].
            scaled_map_point[i16] / xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[2];
        }

        //  normalized feature in camera frame
        // 'predictMeasurementMono:13' px = [h_c_n_l(1)*fx + Cx;
        // 'predictMeasurementMono:14'       h_c_n_l(2)*fy + Cy];
        // 'getH_R_res:60' h_u((res_idx-1)*residualDim + (1:residualDim)) = h_u_l; 
        varargin_2 = (res_idx - 1.0) * 2.0;
        b_h_u->data[(int)(varargin_2 + 1.0) - 1] = h_c_n_l[0] *
          cameraparams_FocalLength[0] + cameraparams_PrincipalPoint[0];
        b_h_u->data[(int)(varargin_2 + 2.0) - 1] = h_c_n_l[1] *
          cameraparams_FocalLength[1] + cameraparams_PrincipalPoint[1];

        // 'getH_R_res:61' z((res_idx-1)*residualDim + (1:residualDim)) = z_curr_l; 
        varargin_2 = (res_idx - 1.0) * 2.0;
        for (i16 = 0; i16 < 2; i16++) {
          r->data[(int)(varargin_2 + (1.0 + (double)i16)) - 1] = z_curr_l[i16];
        }

        // 'getH_R_res:63' h_u_To_h_ci_l = [fx/h_ci_l(3), 0,             -fx*(h_ci_l(1))/h_ci_l(3)^2; 
        // 'getH_R_res:64'                 0,          fy/h_ci_l(3),    -fy*(h_ci_l(2))/h_ci_l(3)^2]; 
        h_u_To_h_ci_l[0] = fx / xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[2];
        h_u_To_h_ci_l[2] = 0.0;
        h_u_To_h_ci_l[4] = -fx * xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[0] /
          (xt_anchor_states[anchorIdx].feature_states[featureIdx].
           scaled_map_point[2] * xt_anchor_states[anchorIdx]
           .feature_states[featureIdx].scaled_map_point[2]);
        h_u_To_h_ci_l[1] = 0.0;
        h_u_To_h_ci_l[3] = fy / xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[2];
        h_u_To_h_ci_l[5] = -fy * xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[1] /
          (xt_anchor_states[anchorIdx].feature_states[featureIdx].
           scaled_map_point[2] * xt_anchor_states[anchorIdx]
           .feature_states[featureIdx].scaled_map_point[2]);

        //              h_ci_l_To_r_wc = -rho*R_cw;
        //              h_ci_l_To_R_cw = skew(h_ci_l);
        // 'getH_R_res:69' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 || rho < 0 
        if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 2)
            || (xt_anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0)) {
          //  delayed initialization or feature behind anchor
          //                  H_robot = [zeros(3), 0*h_ci_l_To_R_cw, zeros(3, numStates-6)]; 
          // 'getH_R_res:71' h_ci_l_To_r_wc = zeros(3);
          // 'getH_R_res:72' h_ci_l_To_R_cw = zeros(3);
          for (i16 = 0; i16 < 9; i16++) {
            h_ci_l_To_r_wc[i16] = 0.0;
            h_ci_l_To_R_cw[i16] = 0.0;
          }
        } else {
          // 'getH_R_res:73' else
          //                  H_robot = [h_ci_l_To_r_wc, h_ci_l_To_R_cw, zeros(3, numStates-6)]; 
          // 'getH_R_res:75' h_ci_l_To_r_wc = -rho*R_cw;
          for (i16 = 0; i16 < 9; i16++) {
            h_ci_l_To_r_wc[i16] = -xt_anchor_states[anchorIdx]
              .feature_states[featureIdx].inverse_depth * R_cw[i16];
          }

          // 'getH_R_res:76' h_ci_l_To_R_cw = skew(h_ci_l);
          // 'skew:2' R=[0,-w(3),w(2);
          // 'skew:3'     w(3),0,-w(1);
          // 'skew:4'     -w(2),w(1),0];
          h_ci_l_To_R_cw[0] = 0.0;
          h_ci_l_To_R_cw[3] = -xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[2];
          h_ci_l_To_R_cw[6] = xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[1];
          h_ci_l_To_R_cw[1] = xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[2];
          h_ci_l_To_R_cw[4] = 0.0;
          h_ci_l_To_R_cw[7] = -xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[0];
          h_ci_l_To_R_cw[2] = -xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[1];
          h_ci_l_To_R_cw[5] = xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[0];
          h_ci_l_To_R_cw[8] = 0.0;
        }

        //             %% anchor state derivatives
        //              h_ci_l_To_anchorPos = rho*R_cw;
        //              h_ci_l_To_anchorRot = -R_cw * skew(anchorRot'*m);
        //              h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
        // 'getH_R_res:85' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 || rho < 0 
        if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 2)
            || (xt_anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0)) {
          //  delayed initialization or feature behind anchor
          //                  H_map = [zeros(3, 6 + featureIdx-1), h_ci_l_To_rho, zeros(3, numPointsPerAnchor - featureIdx)]; 
          // 'getH_R_res:87' h_ci_l_To_anchorPos = zeros(3);
          // 'getH_R_res:88' h_ci_l_To_anchorRot = zeros(3);
          for (i16 = 0; i16 < 9; i16++) {
            h_ci_l_To_anchorPos[i16] = 0.0;
            h_ci_l_To_anchorRot[i16] = 0.0;
          }

          // 'getH_R_res:89' h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
          for (i16 = 0; i16 < 3; i16++) {
            b_xt_anchor_states[i16] = xt_anchor_states[anchorIdx].pos[i16] -
              xt_robot_state_pos[i16];
          }

          for (i16 = 0; i16 < 3; i16++) {
            h_ci_l_To_rho[i16] = 0.0;
            for (i18 = 0; i18 < 3; i18++) {
              h_ci_l_To_rho[i16] += R_cw[i16 + 3 * i18] * b_xt_anchor_states[i18];
            }
          }
        } else {
          // 'getH_R_res:90' else
          //                  H_map = [h_ci_l_To_anchorPos, h_ci_l_To_anchorRot, zeros(3, featureIdx-1), h_ci_l_To_rho, zeros(3, numPointsPerAnchor - featureIdx)]; 
          // 'getH_R_res:92' h_ci_l_To_anchorPos = rho*R_cw;
          for (i16 = 0; i16 < 9; i16++) {
            h_ci_l_To_anchorPos[i16] = xt_anchor_states[anchorIdx].
              feature_states[featureIdx].inverse_depth * R_cw[i16];
          }

          // 'getH_R_res:93' h_ci_l_To_anchorRot = -R_cw * skew(anchorRot'*m);
          // 'skew:2' R=[0,-w(3),w(2);
          // 'skew:3'     w(3),0,-w(1);
          // 'skew:4'     -w(2),w(1),0];
          for (i16 = 0; i16 < 3; i16++) {
            h_ci_l_To_rho[i16] = 0.0;
            for (i18 = 0; i18 < 3; i18++) {
              b_R_cw[i18 + 3 * i16] = -R_cw[i18 + 3 * i16];
              h_ci_l_To_rho[i16] += anchorRot[i18 + 3 * i16] *
                xt_anchor_states[anchorIdx].feature_states[featureIdx].m[i18];
            }
          }

          dv0[0] = 0.0;
          dv0[3] = -h_ci_l_To_rho[2];
          dv0[6] = h_ci_l_To_rho[1];
          dv0[1] = h_ci_l_To_rho[2];
          dv0[4] = 0.0;
          dv0[7] = -h_ci_l_To_rho[0];
          dv0[2] = -h_ci_l_To_rho[1];
          dv0[5] = h_ci_l_To_rho[0];
          dv0[8] = 0.0;

          // 'getH_R_res:94' h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
          for (i16 = 0; i16 < 3; i16++) {
            for (i18 = 0; i18 < 3; i18++) {
              h_ci_l_To_anchorRot[i16 + 3 * i18] = 0.0;
              for (k = 0; k < 3; k++) {
                h_ci_l_To_anchorRot[i16 + 3 * i18] += b_R_cw[i16 + 3 * k] *
                  dv0[k + 3 * i18];
              }
            }

            b_xt_anchor_states[i16] = xt_anchor_states[anchorIdx].pos[i16] -
              xt_robot_state_pos[i16];
          }

          for (i16 = 0; i16 < 3; i16++) {
            h_ci_l_To_rho[i16] = 0.0;
            for (i18 = 0; i18 < 3; i18++) {
              h_ci_l_To_rho[i16] += R_cw[i16 + 3 * i18] * b_xt_anchor_states[i18];
            }
          }
        }

        //              tmp = h_u_To_h_ci_l * [H_robot, zeros(3, (anchorIdx-1)*numErrorStatesPerAnchor), H_map, zeros(3, (numAnchors - anchorIdx)*numErrorStatesPerAnchor)]; 
        // 'getH_R_res:98' H((res_idx-1)*residualDim + (1:residualDim), 1:3) = h_u_To_h_ci_l * h_ci_l_To_r_wc; 
        varargin_2 = (res_idx - 1.0) * 2.0;
        for (i16 = 0; i16 < 2; i16++) {
          for (i18 = 0; i18 < 3; i18++) {
            H->data[((int)(varargin_2 + (1.0 + (double)i16)) + H->size[0] * i18)
              - 1] = 0.0;
            for (k = 0; k < 3; k++) {
              H->data[((int)(varargin_2 + (1.0 + (double)i16)) + H->size[0] *
                       i18) - 1] += h_u_To_h_ci_l[i16 + (k << 1)] *
                h_ci_l_To_r_wc[k + 3 * i18];
            }
          }
        }

        // 'getH_R_res:99' H((res_idx-1)*residualDim + (1:residualDim), 4:6) = h_u_To_h_ci_l * h_ci_l_To_R_cw; 
        varargin_2 = (res_idx - 1.0) * 2.0;
        for (i16 = 0; i16 < 2; i16++) {
          for (i18 = 0; i18 < 3; i18++) {
            H->data[((int)(varargin_2 + (1.0 + (double)i16)) + H->size[0] * (3 +
                      i18)) - 1] = 0.0;
            for (k = 0; k < 3; k++) {
              H->data[((int)(varargin_2 + (1.0 + (double)i16)) + H->size[0] * (3
                        + i18)) - 1] += h_u_To_h_ci_l[i16 + (k << 1)] *
                h_ci_l_To_R_cw[k + 3 * i18];
            }
          }
        }

        // 'getH_R_res:100' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + int32(1:3)) = h_u_To_h_ci_l*h_ci_l_To_anchorPos; 
        varargin_2 = (res_idx - 1.0) * 2.0;
        i16 = 21 + anchorIdx * 14;
        for (i18 = 0; i18 < 2; i18++) {
          for (k = 0; k < 3; k++) {
            H->data[((int)(varargin_2 + (1.0 + (double)i18)) + H->size[0] * (k +
                      i16)) - 1] = 0.0;
            for (outsize_idx_1 = 0; outsize_idx_1 < 3; outsize_idx_1++) {
              H->data[((int)(varargin_2 + (1.0 + (double)i18)) + H->size[0] * (k
                        + i16)) - 1] += h_u_To_h_ci_l[i18 + (outsize_idx_1 << 1)]
                * h_ci_l_To_anchorPos[outsize_idx_1 + 3 * k];
            }
          }
        }

        // 'getH_R_res:101' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + int32(4:6)) = h_u_To_h_ci_l*h_ci_l_To_anchorRot; 
        varargin_2 = (res_idx - 1.0) * 2.0;
        i16 = 24 + anchorIdx * 14;
        for (i18 = 0; i18 < 2; i18++) {
          for (k = 0; k < 3; k++) {
            H->data[((int)(varargin_2 + (1.0 + (double)i18)) + H->size[0] * (k +
                      i16)) - 1] = 0.0;
            for (outsize_idx_1 = 0; outsize_idx_1 < 3; outsize_idx_1++) {
              H->data[((int)(varargin_2 + (1.0 + (double)i18)) + H->size[0] * (k
                        + i16)) - 1] += h_u_To_h_ci_l[i18 + (outsize_idx_1 << 1)]
                * h_ci_l_To_anchorRot[outsize_idx_1 + 3 * k];
            }
          }
        }

        // 'getH_R_res:102' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + 6 + featureIdx) = h_u_To_h_ci_l*h_ci_l_To_rho; 
        varargin_2 = (res_idx - 1.0) * 2.0;
        k = (anchorIdx * 14 + featureIdx) + 27;
        for (i16 = 0; i16 < 2; i16++) {
          H->data[((int)(varargin_2 + (1.0 + (double)i16)) + H->size[0] * k) - 1]
            = 0.0;
          for (i18 = 0; i18 < 3; i18++) {
            H->data[((int)(varargin_2 + (1.0 + (double)i16)) + H->size[0] * k) -
              1] += h_u_To_h_ci_l[i16 + (i18 << 1)] * h_ci_l_To_rho[i18];
          }
        }

        // 'getH_R_res:104' if coder.target('MATLAB')
        // 'getH_R_res:136' res_idx = res_idx + 1;
        res_idx++;
      }
    }
  }

  // 'getH_R_res:142' r = z - h_u;
  i16 = r->size[0];
  emxEnsureCapacity((emxArray__common *)r, i16, (int)sizeof(double));
  k = r->size[0];
  for (i16 = 0; i16 < k; i16++) {
    r->data[i16] -= b_h_u->data[i16];
  }

  emxFree_real_T(&b_h_u);

  // 'getH_R_res:143' R = diag(repmat((noiseParameters.image_noise), 1, numMeas*2)); 
  varargin_2 = (double)n * 2.0;
  outsize_idx_1 = (int)((double)n * 2.0);
  i16 = R->size[0] * R->size[1];
  R->size[0] = (int)varargin_2;
  R->size[1] = (int)varargin_2;
  emxEnsureCapacity((emxArray__common *)R, i16, (int)sizeof(double));
  k = (int)varargin_2 * (int)varargin_2;
  for (i16 = 0; i16 < k; i16++) {
    R->data[i16] = 0.0;
  }

  for (k = 0; k + 1 <= outsize_idx_1; k++) {
    R->data[k + R->size[0] * k] = noiseParameters_image_noise;
  }
}

//
// GETMAP Get the feature points from the current state estimate in the world
// frame
//
//  INPUT ARGUMENTS:
//  - x:                    The current state estimate including anchor poses and inverse depths
//  - anchorFeatures:       A matrix describing which features belong to which
//                          anchor
//  - m_vect:               A matrix containing the rays in the left camera anchor frame of
//                          each feature
//  - numTrackFeatures:     The number of tracked features (== length of
//                          updateVect)
//  - stateSize:            The size of the robot state in xt
//  - numstatesPerAnchorxt: The size of each anchorstate in xt
//
//  OUTPUT ARGUMENTS:
//  - map:                  The map of feature points in world coordinates (3 x
//                          numTrackFeatures)
//  - anchorInd:            A vector describing which anchor each feature belongs to
//  - featureAnchorInd:     A vector describing the index of each feature in its
//                          anchor
// Arguments    : const double xt_origin_pos[3]
//                const double xt_origin_att[4]
//                const f_struct_T xt_anchor_states[5]
//                double b_map[120]
// Return Type  : void
//
static void getMap(const double xt_origin_pos[3], const double xt_origin_att[4],
                   const f_struct_T xt_anchor_states[5], double b_map[120])
{
  double R_ow[9];
  double anchorRot[9];
  int anchorIdx;
  double b_xt_anchor_states[9];
  double anchorPos[3];
  int i0;
  double d0;
  int i1;
  int y;
  int featureIdx;
  long i2;
  double b_anchorPos[3];

  // 'getMap:23' r_ow = xt.origin.pos;
  // 'getMap:24' R_ow = RotFromQuatJ(xt.origin.att);
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
  // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
  // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
  R_ow[0] = ((xt_origin_att[0] * xt_origin_att[0] - xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[3] = 2.0 * (xt_origin_att[0] * xt_origin_att[1] + xt_origin_att[2] *
                   xt_origin_att[3]);
  R_ow[6] = 2.0 * (xt_origin_att[0] * xt_origin_att[2] - xt_origin_att[1] *
                   xt_origin_att[3]);
  R_ow[1] = 2.0 * (xt_origin_att[0] * xt_origin_att[1] - xt_origin_att[2] *
                   xt_origin_att[3]);
  R_ow[4] = ((-(xt_origin_att[0] * xt_origin_att[0]) + xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[7] = 2.0 * (xt_origin_att[1] * xt_origin_att[2] + xt_origin_att[0] *
                   xt_origin_att[3]);
  R_ow[2] = 2.0 * (xt_origin_att[0] * xt_origin_att[2] + xt_origin_att[1] *
                   xt_origin_att[3]);
  R_ow[5] = 2.0 * (xt_origin_att[1] * xt_origin_att[2] - xt_origin_att[0] *
                   xt_origin_att[3]);
  R_ow[8] = ((-(xt_origin_att[0] * xt_origin_att[0]) - xt_origin_att[1] *
              xt_origin_att[1]) + xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];

  // 'getMap:26' map = zeros(numTrackFeatures*3, 1);
  memset(&b_map[0], 0, 120U * sizeof(double));

  // 'getMap:28' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    // 'getMap:29' anchorPos = r_ow + R_ow' * xt.anchor_states(anchorIdx).pos;
    // 'getMap:30' anchorRot = RotFromQuatJ(xt.anchor_states(anchorIdx).att) * R_ow; 
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
    // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
    // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
    b_xt_anchor_states[0] = ((xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    b_xt_anchor_states[3] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[6] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[1] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[4] = ((-(xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0]) + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    b_xt_anchor_states[7] = 2.0 * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[2] = 2.0 * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[5] = 2.0 * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[8] = ((-(xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0]) - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    for (i0 = 0; i0 < 3; i0++) {
      d0 = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        d0 += R_ow[i1 + 3 * i0] * xt_anchor_states[anchorIdx].pos[i1];
        anchorRot[i0 + 3 * i1] = 0.0;
        for (y = 0; y < 3; y++) {
          anchorRot[i0 + 3 * i1] += b_xt_anchor_states[i0 + 3 * y] * R_ow[y + 3 *
            i1];
        }
      }

      anchorPos[i0] = xt_origin_pos[i0] + d0;
    }

    // 'getMap:32' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      // 'getMap:33' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status 
      if (xt_anchor_states[anchorIdx].feature_states[featureIdx].status != 0) {
        // 'getMap:34' rho = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth; 
        // 'getMap:36' m = xt.anchor_states(anchorIdx).feature_states(featureIdx).m; 
        // 'getMap:38' fp = anchorPos + anchorRot'*m/rho;
        // 'getMap:40' map((xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx-1)*3 + int32(1:3))= fp; 
        i2 = xt_anchor_states[anchorIdx].feature_states[featureIdx].status_idx -
          1L;
        if (i2 > 2147483647L) {
          i2 = 2147483647L;
        } else {
          if (i2 < -2147483648L) {
            i2 = -2147483648L;
          }
        }

        i2 = (int)i2 * 3L;
        if (i2 > 2147483647L) {
          i2 = 2147483647L;
        } else {
          if (i2 < -2147483648L) {
            i2 = -2147483648L;
          }
        }

        y = (int)i2;
        for (i0 = 0; i0 < 3; i0++) {
          d0 = 0.0;
          for (i1 = 0; i1 < 3; i1++) {
            d0 += anchorRot[i1 + 3 * i0] * xt_anchor_states[anchorIdx].
              feature_states[featureIdx].m[i1];
          }

          b_anchorPos[i0] = anchorPos[i0] + d0 / xt_anchor_states[anchorIdx].
            feature_states[featureIdx].inverse_depth;
          i2 = (long)y + (1 + i0);
          if (i2 > 2147483647L) {
            i2 = 2147483647L;
          } else {
            if (i2 < -2147483648L) {
              i2 = -2147483648L;
            }
          }

          b_map[(int)i2 - 1] = b_anchorPos[i0];
        }
      }
    }
  }
}

//
// getNumValidFeatures Get the number of valid features of an anchor
// Arguments    : const e_struct_T anchor_state_feature_states[8]
// Return Type  : double
//
static double getNumValidFeatures(const e_struct_T anchor_state_feature_states[8])
{
  double n;
  int featureIdx;
  boolean_T x[2];
  int k;
  boolean_T y;
  boolean_T exitg1;

  // 'getNumValidFeatures:4' n = getNumFeaturesOfType(anchor_state, [1 2]);
  // getNumFeaturesOfType Get the number of features of type type of an anchor
  //  type can be a scalar or a row vector of types
  // 'getNumFeaturesOfType:5' n = 0;
  n = 0.0;

  // 'getNumFeaturesOfType:6' for featureIdx = 1:numPointsPerAnchor
  for (featureIdx = 0; featureIdx < 8; featureIdx++) {
    // 'getNumFeaturesOfType:7' if any(anchor_state.feature_states(featureIdx).status == type) 
    for (k = 0; k < 2; k++) {
      x[k] = (anchor_state_feature_states[featureIdx].status == 1 + k);
    }

    y = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 2)) {
      if (!!x[k]) {
        y = true;
        exitg1 = true;
      } else {
        k++;
      }
    }

    if (y) {
      // 'getNumFeaturesOfType:8' n = n + 1;
      n++;
    }
  }

  return n;
}

//
// getScaledMap Get the sacled feature points from the current state estimate
//
//  INPUT ARGUMENTS:
//  - x:                    The current state estimate including anchor poses and inverse depths
//  - anchorFeatures:       A matrix describing which features belong to which
//                          anchor
//  - m_vect:               A matrix containing the rays in the left camera anchor frame of
//                          each feature
//  - numTrackFeatures:     The number of tracked features (== length of
//                          updateVect)
//  - stateSize:            The size of the robot state in xt
//  - numstatesPerAnchorxt: The size of each anchorstate in xt
//
//  OUTPUT ARGUMENTS:
//  - map:                  The scaled map of feature points in world coordinates (3 x
//                          numTrackFeatures)
//  - anchorInd:            A vector describing which anchor each feature belongs to
//  - featureAnchorInd:     A vector describing the index of each feature in its
//                          anchor
// Arguments    : g_struct_T *b_xt
// Return Type  : void
//
static void getScaledMap(g_struct_T *b_xt)
{
  double R_cw[9];
  int anchorIdx;
  double anchorRot[9];
  int featureIdx;
  double c_xt[3];
  int i56;
  double d11;
  int i57;

  // 'getScaledMap:22' R_cw = RotFromQuatJ(xt.robot_state.att);
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
  // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
  // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
  R_cw[0] = ((b_xt->robot_state.att[0] * b_xt->robot_state.att[0] -
              b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
             b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
    b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
  R_cw[3] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] +
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
  R_cw[6] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] -
                   b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
  R_cw[1] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] -
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
  R_cw[4] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) +
              b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
             b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
    b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
  R_cw[7] = 2.0 * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] +
                   b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
  R_cw[2] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] +
                   b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
  R_cw[5] = 2.0 * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] -
                   b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
  R_cw[8] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) -
              b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) +
             b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
    b_xt->robot_state.att[3] * b_xt->robot_state.att[3];

  // 'getScaledMap:24' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    // 'getScaledMap:25' anchorPos = xt.anchor_states(anchorIdx).pos;
    // 'getScaledMap:26' anchorRot = RotFromQuatJ(xt.anchor_states(anchorIdx).att); 
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
    // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
    // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
    anchorRot[0] = ((b_xt->anchor_states[anchorIdx].att[0] * b_xt->
                     anchor_states[anchorIdx].att[0] - b_xt->
                     anchor_states[anchorIdx].att[1] * b_xt->
                     anchor_states[anchorIdx].att[1]) - b_xt->
                    anchor_states[anchorIdx].att[2] * b_xt->
                    anchor_states[anchorIdx].att[2]) + b_xt->
      anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx].att[3];
    anchorRot[3] = 2.0 * (b_xt->anchor_states[anchorIdx].att[0] *
                          b_xt->anchor_states[anchorIdx].att[1] +
                          b_xt->anchor_states[anchorIdx].att[2] *
                          b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[6] = 2.0 * (b_xt->anchor_states[anchorIdx].att[0] *
                          b_xt->anchor_states[anchorIdx].att[2] -
                          b_xt->anchor_states[anchorIdx].att[1] *
                          b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[1] = 2.0 * (b_xt->anchor_states[anchorIdx].att[0] *
                          b_xt->anchor_states[anchorIdx].att[1] -
                          b_xt->anchor_states[anchorIdx].att[2] *
                          b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[4] = ((-(b_xt->anchor_states[anchorIdx].att[0] *
                       b_xt->anchor_states[anchorIdx].att[0]) +
                     b_xt->anchor_states[anchorIdx].att[1] * b_xt->
                     anchor_states[anchorIdx].att[1]) - b_xt->
                    anchor_states[anchorIdx].att[2] * b_xt->
                    anchor_states[anchorIdx].att[2]) + b_xt->
      anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx].att[3];
    anchorRot[7] = 2.0 * (b_xt->anchor_states[anchorIdx].att[1] *
                          b_xt->anchor_states[anchorIdx].att[2] +
                          b_xt->anchor_states[anchorIdx].att[0] *
                          b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[2] = 2.0 * (b_xt->anchor_states[anchorIdx].att[0] *
                          b_xt->anchor_states[anchorIdx].att[2] +
                          b_xt->anchor_states[anchorIdx].att[1] *
                          b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[5] = 2.0 * (b_xt->anchor_states[anchorIdx].att[1] *
                          b_xt->anchor_states[anchorIdx].att[2] -
                          b_xt->anchor_states[anchorIdx].att[0] *
                          b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[8] = ((-(b_xt->anchor_states[anchorIdx].att[0] *
                       b_xt->anchor_states[anchorIdx].att[0]) -
                     b_xt->anchor_states[anchorIdx].att[1] * b_xt->
                     anchor_states[anchorIdx].att[1]) + b_xt->
                    anchor_states[anchorIdx].att[2] * b_xt->
                    anchor_states[anchorIdx].att[2]) + b_xt->
      anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx].att[3];

    // 'getScaledMap:28' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      // 'getScaledMap:29' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status 
      if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status != 0)
      {
        // 'getScaledMap:30' rho = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth; 
        // 'getScaledMap:32' m = xt.anchor_states(anchorIdx).feature_states(featureIdx).m; 
        // 'getScaledMap:34' fp_scaled = R_cw*(rho*anchorPos + anchorRot'*m - xt.robot_state.pos*rho); 
        // 'getScaledMap:36' xt.anchor_states(anchorIdx).feature_states(featureIdx).scaled_map_point = fp_scaled; 
        for (i56 = 0; i56 < 3; i56++) {
          d11 = 0.0;
          for (i57 = 0; i57 < 3; i57++) {
            d11 += anchorRot[i57 + 3 * i56] * b_xt->anchor_states[anchorIdx].
              feature_states[featureIdx].m[i57];
          }

          c_xt[i56] = (b_xt->anchor_states[anchorIdx].feature_states[featureIdx]
                       .inverse_depth * b_xt->anchor_states[anchorIdx].pos[i56]
                       + d11) - b_xt->robot_state.pos[i56] * b_xt->
            anchor_states[anchorIdx].feature_states[featureIdx].inverse_depth;
        }

        for (i56 = 0; i56 < 3; i56++) {
          b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
            scaled_map_point[i56] = 0.0;
          for (i57 = 0; i57 < 3; i57++) {
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              scaled_map_point[i56] += R_cw[i56 + 3 * i57] * c_xt[i57];
          }
        }
      }
    }
  }
}

//
// getTotalNumActiveFeatures Get the number of active features of all anchors
// Arguments    : const f_struct_T xt_anchor_states[5]
// Return Type  : double
//
static double getTotalNumActiveFeatures(const f_struct_T xt_anchor_states[5])
{
  double n;
  int anchorIdx;
  double b_n;
  int featureIdx;

  // 'getTotalNumActiveFeatures:4' n = 0;
  n = 0.0;

  // 'getTotalNumActiveFeatures:5' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    // 'getTotalNumActiveFeatures:6' n = n + getNumActiveFeatures(xt.anchor_states(anchorIdx)); 
    // getNumActiveFeatures Get the number of active features of an anchor
    // 'getNumActiveFeatures:4' n = getNumFeaturesOfType(anchor_state, 1);
    // getNumFeaturesOfType Get the number of features of type type of an anchor 
    //  type can be a scalar or a row vector of types
    // 'getNumFeaturesOfType:5' n = 0;
    b_n = 0.0;

    // 'getNumFeaturesOfType:6' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      // 'getNumFeaturesOfType:7' if any(anchor_state.feature_states(featureIdx).status == type) 
      if (!!(xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 1))
      {
        // 'getNumFeaturesOfType:8' n = n + 1;
        b_n++;
      }
    }

    n += b_n;
  }

  return n;
}

//
// getTotalNumDelayedFeatures Get the number of delayed features of all anchors
// Arguments    : const f_struct_T xt_anchor_states[5]
// Return Type  : double
//
static double getTotalNumDelayedFeatures(const f_struct_T xt_anchor_states[5])
{
  double n;
  int anchorIdx;
  double b_n;
  int featureIdx;

  // 'getTotalNumDelayedFeatures:4' n = 0;
  n = 0.0;

  // 'getTotalNumDelayedFeatures:5' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    // 'getTotalNumDelayedFeatures:6' n = n + getNumFeaturesOfType(xt.anchor_states(anchorIdx), 2); 
    // getNumFeaturesOfType Get the number of features of type type of an anchor 
    //  type can be a scalar or a row vector of types
    // 'getNumFeaturesOfType:5' n = 0;
    b_n = 0.0;

    // 'getNumFeaturesOfType:6' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      // 'getNumFeaturesOfType:7' if any(anchor_state.feature_states(featureIdx).status == type) 
      if (!!(xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 2))
      {
        // 'getNumFeaturesOfType:8' n = n + 1;
        b_n++;
      }
    }

    n += b_n;
  }

  return n;
}

//
// getWorldState Get the state of the robot in the world frame
// Arguments    : const double xt_robot_state_IMU_pos[3]
//                const double xt_robot_state_IMU_att[4]
//                const double xt_robot_state_IMU_gyro_bias[3]
//                const double xt_robot_state_IMU_acc_bias[3]
//                const double xt_robot_state_pos[3]
//                const double xt_robot_state_att[4]
//                const double xt_robot_state_vel[3]
//                const double xt_origin_pos[3]
//                const double xt_origin_att[4]
//                double world_state_pos[3]
//                double world_state_att[4]
//                double world_state_vel[3]
//                double world_state_IMU_gyro_bias[3]
//                double world_state_IMU_acc_bias[3]
//                double world_state_IMU_pos[3]
//                double world_state_IMU_att[4]
// Return Type  : void
//
static void getWorldState(const double xt_robot_state_IMU_pos[3], const double
  xt_robot_state_IMU_att[4], const double xt_robot_state_IMU_gyro_bias[3], const
  double xt_robot_state_IMU_acc_bias[3], const double xt_robot_state_pos[3],
  const double xt_robot_state_att[4], const double xt_robot_state_vel[3], const
  double xt_origin_pos[3], const double xt_origin_att[4], double
  world_state_pos[3], double world_state_att[4], double world_state_vel[3],
  double world_state_IMU_gyro_bias[3], double world_state_IMU_acc_bias[3],
  double world_state_IMU_pos[3], double world_state_IMU_att[4])
{
  double R_ow[9];
  double b_xt_robot_state_att[9];
  double c_xt_robot_state_att[9];
  int i;
  double d5;
  int i41;
  int i42;

  // 'getWorldState:4' R_co = RotFromQuatJ(xt.robot_state.att);
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
  // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
  // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
  // 'getWorldState:5' R_ow = RotFromQuatJ(xt.origin.att);
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
  // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
  // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
  R_ow[0] = ((xt_origin_att[0] * xt_origin_att[0] - xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[3] = 2.0 * (xt_origin_att[0] * xt_origin_att[1] + xt_origin_att[2] *
                   xt_origin_att[3]);
  R_ow[6] = 2.0 * (xt_origin_att[0] * xt_origin_att[2] - xt_origin_att[1] *
                   xt_origin_att[3]);
  R_ow[1] = 2.0 * (xt_origin_att[0] * xt_origin_att[1] - xt_origin_att[2] *
                   xt_origin_att[3]);
  R_ow[4] = ((-(xt_origin_att[0] * xt_origin_att[0]) + xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[7] = 2.0 * (xt_origin_att[1] * xt_origin_att[2] + xt_origin_att[0] *
                   xt_origin_att[3]);
  R_ow[2] = 2.0 * (xt_origin_att[0] * xt_origin_att[2] + xt_origin_att[1] *
                   xt_origin_att[3]);
  R_ow[5] = 2.0 * (xt_origin_att[1] * xt_origin_att[2] - xt_origin_att[0] *
                   xt_origin_att[3]);
  R_ow[8] = ((-(xt_origin_att[0] * xt_origin_att[0]) - xt_origin_att[1] *
              xt_origin_att[1]) + xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];

  // 'getWorldState:7' world_state.pos = xt.origin.pos + R_ow' * xt.robot_state.pos; 
  // 'getWorldState:8' world_state.att = QuatFromRotJ(R_co * R_ow);
  c_xt_robot_state_att[0] = ((xt_robot_state_att[0] * xt_robot_state_att[0] -
    xt_robot_state_att[1] * xt_robot_state_att[1]) - xt_robot_state_att[2] *
    xt_robot_state_att[2]) + xt_robot_state_att[3] * xt_robot_state_att[3];
  c_xt_robot_state_att[3] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[1]
    + xt_robot_state_att[2] * xt_robot_state_att[3]);
  c_xt_robot_state_att[6] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[2]
    - xt_robot_state_att[1] * xt_robot_state_att[3]);
  c_xt_robot_state_att[1] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[1]
    - xt_robot_state_att[2] * xt_robot_state_att[3]);
  c_xt_robot_state_att[4] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) +
    xt_robot_state_att[1] * xt_robot_state_att[1]) - xt_robot_state_att[2] *
    xt_robot_state_att[2]) + xt_robot_state_att[3] * xt_robot_state_att[3];
  c_xt_robot_state_att[7] = 2.0 * (xt_robot_state_att[1] * xt_robot_state_att[2]
    + xt_robot_state_att[0] * xt_robot_state_att[3]);
  c_xt_robot_state_att[2] = 2.0 * (xt_robot_state_att[0] * xt_robot_state_att[2]
    + xt_robot_state_att[1] * xt_robot_state_att[3]);
  c_xt_robot_state_att[5] = 2.0 * (xt_robot_state_att[1] * xt_robot_state_att[2]
    - xt_robot_state_att[0] * xt_robot_state_att[3]);
  c_xt_robot_state_att[8] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) -
    xt_robot_state_att[1] * xt_robot_state_att[1]) + xt_robot_state_att[2] *
    xt_robot_state_att[2]) + xt_robot_state_att[3] * xt_robot_state_att[3];
  for (i = 0; i < 3; i++) {
    d5 = 0.0;
    for (i41 = 0; i41 < 3; i41++) {
      d5 += R_ow[i41 + 3 * i] * xt_robot_state_pos[i41];
      b_xt_robot_state_att[i + 3 * i41] = 0.0;
      for (i42 = 0; i42 < 3; i42++) {
        b_xt_robot_state_att[i + 3 * i41] += c_xt_robot_state_att[i + 3 * i42] *
          R_ow[i42 + 3 * i41];
      }
    }

    world_state_pos[i] = xt_origin_pos[i] + d5;
    world_state_vel[i] = 0.0;
    for (i41 = 0; i41 < 3; i41++) {
      world_state_vel[i] += R_ow[i41 + 3 * i] * xt_robot_state_vel[i41];
    }

    world_state_IMU_gyro_bias[i] = xt_robot_state_IMU_gyro_bias[i];
    world_state_IMU_acc_bias[i] = xt_robot_state_IMU_acc_bias[i];
    world_state_IMU_pos[i] = xt_robot_state_IMU_pos[i];
  }

  QuatFromRotJ(b_xt_robot_state_att, world_state_att);

  // 'getWorldState:9' world_state.vel = R_ow' * xt.robot_state.vel;
  // 'getWorldState:10' world_state.IMU.gyro_bias = xt.robot_state.IMU.gyro_bias; 
  // 'getWorldState:11' world_state.IMU.acc_bias = xt.robot_state.IMU.acc_bias;
  // 'getWorldState:12' world_state.IMU.pos = xt.robot_state.IMU.pos;
  // 'getWorldState:13' world_state.IMU.att = xt.robot_state.IMU.att;
  for (i = 0; i < 4; i++) {
    world_state_IMU_att[i] = xt_robot_state_IMU_att[i];
  }
}

//
// Arguments    : double varargin_1
//                double varargin_2
//                double varargin_3
// Return Type  : double
//
static double h_fprintf(double varargin_1, double varargin_2, double varargin_3)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[37] = { 'g', 'y', 'r', 'o', ' ', 'b', 'i', 'a', 's',
    ' ', 'i', 'n', 'i', 't', 'i', 'a', 'l', ' ', 'u', 'n', 'c', ':', ' ', '[',
    '%', 'f', ',', ' ', '%', 'f', ',', ' ', '%', 'f', ']', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1, varargin_2, varargin_3);
    fflush(filestar);
  }

  return nbytesint;
}

//
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
// Return Type  : void
//
static void h_ros_info(int varargin_1)
{
  char cv48[28];
  int i38;
  static const char cv49[28] = { 'S', 'e', 't', 't', 'i', 'n', 'g', ' ', 'a',
    'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ' ', 'a', 's', ' ', 'o', 'r', 'i',
    'g', 'i', 'n', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_info:5' if coder.target('MATLAB')
  // 'ros_info:9' elseif ~coder.target('MEX')
  // 'ros_info:10' coder.cinclude('<ros/console.h>')
  // 'ros_info:11' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_info:12' coder.ceval('ROS_INFO', [str, 0], varargin{:});
    for (i38 = 0; i38 < 28; i38++) {
      cv48[i38] = cv49[i38];
    }

    ROS_INFO(cv48, varargin_1);
  }
}

//
// ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
// Arguments    : int varargin_1
//                int varargin_2
//                int varargin_3
// Return Type  : void
//
static void h_ros_warn(int varargin_1, int varargin_2, int varargin_3)
{
  char cv38[57];
  int i32;
  static const char cv39[57] = { 'F', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%',
    'i', ' ', '(', '%', 'i', ' ', 'o', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r',
    ' ', '%', 'i', ')', ' ', 'c', 'o', 'n', 'v', 'e', 'r', 'g', 'e', 'd', ' ',
    'b', 'e', 'h', 'i', 'n', 'd', ' ', 'i', 't', 's', ' ', 'a', 'n', 'c', 'h',
    'o', 'r', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_warn:6' if coder.target('MATLAB')
  // 'ros_warn:10' elseif ~coder.target('MEX')
  // 'ros_warn:11' coder.cinclude('<ros/console.h>')
  // 'ros_warn:12' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_warn:13' coder.ceval('ROS_WARN', [str, 0], varargin{:});
    for (i32 = 0; i32 < 57; i32++) {
      cv38[i32] = cv39[i32];
    }

    ROS_WARN(cv38, varargin_1, varargin_2, varargin_3);
  }
}

//
// Arguments    : double varargin_1
//                double varargin_2
//                double varargin_3
// Return Type  : double
//
static double i_fprintf(double varargin_1, double varargin_2, double varargin_3)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[36] = { 'a', 'c', 'c', ' ', 'b', 'i', 'a', 's', ' ',
    'i', 'n', 'i', 't', 'i', 'a', 'l', ' ', 'u', 'n', 'c', ':', ' ', '[', '%',
    'f', ',', ' ', '%', 'f', ',', ' ', '%', 'f', ']', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1, varargin_2, varargin_3);
    fflush(filestar);
  }

  return nbytesint;
}

//
// ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
// Arguments    : int varargin_1
// Return Type  : void
//
static void i_ros_warn(int varargin_1)
{
  char cv40[52];
  int i33;
  static const char cv41[52] = { 'T', 'r', 'y', 'i', 'n', 'g', ' ', 't', 'o',
    ' ', 'f', 'o', 'r', 'c', 'e', ' ', 'i', 'n', 's', 'e', 'r', 't', ' ', 'f',
    'e', 'a', 't', 'u', 'r', 'e', ' ', '%', 'i', ' ', 'b', 'e', 'h', 'i', 'n',
    'd', ' ', 'i', 't', 's', ' ', 'a', 'n', 'c', 'h', 'o', 'r', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_warn:6' if coder.target('MATLAB')
  // 'ros_warn:10' elseif ~coder.target('MEX')
  // 'ros_warn:11' coder.cinclude('<ros/console.h>')
  // 'ros_warn:12' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_warn:13' coder.ceval('ROS_WARN', [str, 0], varargin{:});
    for (i33 = 0; i33 < 52; i33++) {
      cv40[i33] = cv41[i33];
    }

    ROS_WARN(cv40, varargin_1);
  }
}

//
// INITIALIZEPOINT Initialize a feature point from undistorted measurements
//
//  INPUT ARGUMENTS:
//  - cameraparams: The camera parameter struct
//  - z_l: The undistorted measurement of the feature in the left camera
//  - z_r: The undistorted measurement of the feature in the right camera
//
//  OUTPUT ARGUMENTS:
//  - fp: The feature point in the camera frame
//  - m:  The ray in the left and right camera frame to the feature
// Arguments    : const double z_u_l[2]
//                const double z_u_r[2]
//                const double c_cameraparams_CameraParameters[2]
//                const double d_cameraparams_CameraParameters[2]
//                const double e_cameraparams_CameraParameters[2]
//                const double f_cameraparams_CameraParameters[2]
//                const double cameraparams_r_lr[3]
//                const double cameraparams_R_lr[9]
//                double fp[3]
//                double b_m[6]
//                boolean_T *success
// Return Type  : void
//
static void initializePoint(const double z_u_l[2], const double z_u_r[2], const
  double c_cameraparams_CameraParameters[2], const double
  d_cameraparams_CameraParameters[2], const double
  e_cameraparams_CameraParameters[2], const double
  f_cameraparams_CameraParameters[2], const double cameraparams_r_lr[3], const
  double cameraparams_R_lr[9], double fp[3], double b_m[6], boolean_T *success)
{
  double ml[3];
  double mr[3];
  double tol;
  double B;
  double b_pos[6];
  int k;
  int i24;
  static const signed char iv0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double rot[18];
  double A[30];
  double b[6];
  int anchorIdx;
  int rankR;
  double b_rot[9];
  signed char I[9];
  int j;
  int jpvt[5];
  double tau[5];
  double x[5];

  // 'initializePoint:13' fx_l = cameraparams.CameraParameters1.FocalLength(1);
  // 'initializePoint:14' fy_l = cameraparams.CameraParameters1.FocalLength(2);
  // 'initializePoint:15' Cx_l = cameraparams.CameraParameters1.PrincipalPoint(1); 
  // 'initializePoint:16' Cy_l = cameraparams.CameraParameters1.PrincipalPoint(2); 
  // 'initializePoint:18' fx_r = cameraparams.CameraParameters2.FocalLength(1);
  // 'initializePoint:19' fy_r = cameraparams.CameraParameters2.FocalLength(2);
  // 'initializePoint:20' Cx_r = cameraparams.CameraParameters2.PrincipalPoint(1); 
  // 'initializePoint:21' Cy_r = cameraparams.CameraParameters2.PrincipalPoint(2); 
  // 'initializePoint:23' pos = [zeros(3,1), cameraparams.r_lr];
  // 'initializePoint:24' rot = [eye(3), cameraparams.R_lr'];
  // 'initializePoint:26' z_n_l = [(z_u_l(1) - Cx_l)/fx_l; (z_u_l(2) - Cy_l)/fy_l]; 
  // 'initializePoint:27' z_n_r = [(z_u_r(1) - Cx_r)/fx_r; (z_u_r(2) - Cy_r)/fy_r]; 
  // 'initializePoint:29' ml = [z_n_l; 1];
  ml[0] = (z_u_l[0] - d_cameraparams_CameraParameters[0]) /
    c_cameraparams_CameraParameters[0];
  ml[1] = (z_u_l[1] - d_cameraparams_CameraParameters[1]) /
    c_cameraparams_CameraParameters[1];
  ml[2] = 1.0;

  // 'initializePoint:30' mr = [z_n_r; 1];
  mr[0] = (z_u_r[0] - f_cameraparams_CameraParameters[0]) /
    e_cameraparams_CameraParameters[0];
  mr[1] = (z_u_r[1] - f_cameraparams_CameraParameters[1]) /
    e_cameraparams_CameraParameters[1];
  mr[2] = 1.0;

  // 'initializePoint:31' m = [ml/norm(ml),mr/norm(mr)];
  tol = norm(ml);
  B = norm(mr);
  for (k = 0; k < 3; k++) {
    b_pos[k] = 0.0;
    b_pos[3 + k] = cameraparams_r_lr[k];
    for (i24 = 0; i24 < 3; i24++) {
      rot[i24 + 3 * k] = iv0[i24 + 3 * k];
      rot[i24 + 3 * (k + 3)] = cameraparams_R_lr[k + 3 * i24];
    }

    b_m[k] = ml[k] / tol;
    b_m[3 + k] = mr[k] / B;
  }

  //  normalized rays in left frame
  // 'initializePoint:34' ml_n_l = m(:,1);
  // 'initializePoint:35' mr_n_r = cameraparams.R_lr * m(:,2);
  for (k = 0; k < 3; k++) {
    ml[k] = 0.0;
    for (i24 = 0; i24 < 3; i24++) {
      ml[k] += cameraparams_R_lr[k + 3 * i24] * b_m[3 + i24];
    }
  }

  // 'initializePoint:36' cross_prod = cross(ml_n_l, mr_n_r);
  // 'initializePoint:37' if cross_prod(2) > 0
  if (b_m[2] * ml[0] - b_m[0] * ml[2] > 0.0) {
    // 'initializePoint:38' success = false;
    *success = false;

    // 'initializePoint:39' fp = m(:,1);
    for (k = 0; k < 3; k++) {
      fp[k] = b_m[k];
    }
  } else {
    // 'initializePoint:42' success = true;
    *success = true;

    // 'initializePoint:44' [ fp ] = triangulatePoint( pos, rot, m );
    // triangulatePoint Triangulate a point from several measurements
    //    Calculate the world position of a point given measurements of it from
    //    different positions. The solution minimizes the least squares error
    //
    //  INPUTS:
    //  pos: a 3 x n matrix containing the positions of the cameras
    //  rot: a 4 x n matrix containing the JPL quaternions of the cameras
    //  m: a 3 x n matrix containing the unit vectors pointing towards the
    //      feature in the camera frame
    //  where n is the number of poses
    //
    //  OUTPUTS:
    //  pf: the feature position in world coordinates
    //  condition: the condition number of the least squares problem. High
    //      numbers mean a badly conditiond problem and a bad result
    // 'triangulatePoint:18' numAnchors = size(pos, 2);
    // 'triangulatePoint:20' A = zeros(3*numAnchors, numAnchors + 3);
    memset(&A[0], 0, 30U * sizeof(double));

    // 'triangulatePoint:21' b = nan(3*numAnchors, 1);
    for (k = 0; k < 6; k++) {
      b[k] = rtNaN;
    }

    // 'triangulatePoint:23' for anchorIdx = 1:numAnchors
    for (anchorIdx = 0; anchorIdx < 2; anchorIdx++) {
      // 'triangulatePoint:24' currentAnchorPos = pos(:,anchorIdx);
      // 'triangulatePoint:25' currentAnchorRot = rot(:, (anchorIdx-1)*3 + (1:3)); 
      // 'triangulatePoint:26' currentM = m(:,anchorIdx);
      // 'triangulatePoint:28' A((anchorIdx-1)*3 + (1:3), anchorIdx) = currentAnchorRot'*currentM; 
      rankR = anchorIdx * 3;
      for (k = 0; k < 3; k++) {
        for (i24 = 0; i24 < 3; i24++) {
          b_rot[i24 + 3 * k] = rot[k + 3 * (i24 + rankR)];
        }
      }

      rankR = anchorIdx * 3;
      for (k = 0; k < 3; k++) {
        A[(k + rankR) + 6 * anchorIdx] = 0.0;
        for (i24 = 0; i24 < 3; i24++) {
          A[(k + rankR) + 6 * anchorIdx] += b_rot[k + 3 * i24] * b_m[i24 + 3 *
            anchorIdx];
        }
      }

      // 'triangulatePoint:29' A((anchorIdx-1)*3 + (1:3), end - 3 + (1:3)) = -eye(3); 
      for (k = 0; k < 9; k++) {
        I[k] = 0;
      }

      for (k = 0; k < 3; k++) {
        I[k + 3 * k] = 1;
      }

      rankR = anchorIdx * 3;

      // 'triangulatePoint:31' b((anchorIdx-1)*3 + (1:3)) = - currentAnchorPos;
      j = anchorIdx * 3;
      for (k = 0; k < 3; k++) {
        for (i24 = 0; i24 < 3; i24++) {
          A[(i24 + rankR) + 6 * (2 + k)] = -(double)I[i24 + 3 * k];
        }

        b[k + j] = -b_pos[k + 3 * anchorIdx];
      }
    }

    //  condition = cond(A);
    // 'triangulatePoint:35' x = A\b;
    b_xgeqp3(A, tau, jpvt);
    rankR = 0;
    tol = 6.0 * fabs(A[0]) * 2.2204460492503131E-16;
    while ((rankR < 5) && (fabs(A[rankR + 6 * rankR]) >= tol)) {
      rankR++;
    }

    for (j = 0; j < 5; j++) {
      x[j] = 0.0;
      if (tau[j] != 0.0) {
        tol = b[j];
        for (k = j + 1; k + 1 < 7; k++) {
          tol += A[k + 6 * j] * b[k];
        }

        tol *= tau[j];
        if (tol != 0.0) {
          b[j] -= tol;
          for (k = j + 1; k + 1 < 7; k++) {
            b[k] -= A[k + 6 * j] * tol;
          }
        }
      }
    }

    for (k = 0; k + 1 <= rankR; k++) {
      x[jpvt[k] - 1] = b[k];
    }

    for (j = rankR - 1; j + 1 > 0; j--) {
      x[jpvt[j] - 1] /= A[j + 6 * j];
      for (k = 0; k + 1 <= j; k++) {
        x[jpvt[k] - 1] -= x[jpvt[j] - 1] * A[k + 6 * j];
      }
    }

    // 'triangulatePoint:36' pf = x(end-2:end);
    for (k = 0; k < 3; k++) {
      fp[k] = x[k + 2];
    }

    //  is this better?
    //  A = [cameraparams.R_rl * ml, -mr];
    //  b = cameraparams.r_lr;
    //  x = A\b;
    //  fp = ml*x(1);
  }
}

//
// Arguments    : double varargin_1
// Return Type  : double
//
static double j_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[17] = { 'i', 'm', 'a', 'g', 'e', '_', 'n', 'o', 'i',
    's', 'e', ':', ' ', '%', 'f', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1);
    fflush(filestar);
  }

  return nbytesint;
}

//
// ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
// Arguments    : void
// Return Type  : void
//
static void j_ros_warn()
{
  char cv46[54];
  int i37;
  static const char cv47[54] = { 'C', 'a', 'n', '\'', 't', ' ', 'f', 'i', 'x',
    ' ', 'a', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', 'b', 'e', 'c', 'a',
    'u', 's', 'e', ' ', 'n', 'o', 'n', 'e', ' ', 'h', 'a', 'v', 'e', ' ', 'a',
    'c', 't', 'i', 'v', 'e', ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e', 's', '\x00'
  };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_warn:6' if coder.target('MATLAB')
  // 'ros_warn:10' elseif ~coder.target('MEX')
  // 'ros_warn:11' coder.cinclude('<ros/console.h>')
  // 'ros_warn:12' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_warn:13' coder.ceval('ROS_WARN', [str, 0], varargin{:});
    for (i37 = 0; i37 < 54; i37++) {
      cv46[i37] = cv47[i37];
    }

    ROS_WARN(cv46);
  }
}

//
// Arguments    : double varargin_1
// Return Type  : double
//
static double k_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[27] = { 'i', 'n', 'v', '_', 'd', 'e', 'p', 't', 'h',
    '_', 'i', 'n', 'i', 't', 'i', 'a', 'l', '_', 'u', 'n', 'c', ':', ' ', '%',
    'f', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : int varargin_1
// Return Type  : double
//
static double l_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[17] = { 'n', 'u', 'm', '_', 'a', 'n', 'c', 'h', 'o',
    'r', 's', ':', ' ', '%', 'd', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : const emxArray_real_T *A
//                emxArray_real_T *B
// Return Type  : void
//
static void lusolve(const emxArray_real_T *A, emxArray_real_T *B)
{
  emxArray_real_T *b_A;
  int n;
  int i58;
  int iy;
  emxArray_int32_T *ipiv;
  int j;
  int mmj;
  int c;
  int ix;
  double smax;
  int k;
  double s;
  int i59;
  int i;
  int jAcol;
  int jA;
  emxInit_real_T1(&b_A, 2);
  n = A->size[1];
  i58 = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  emxEnsureCapacity((emxArray__common *)b_A, i58, (int)sizeof(double));
  iy = A->size[0] * A->size[1];
  for (i58 = 0; i58 < iy; i58++) {
    b_A->data[i58] = A->data[i58];
  }

  emxInit_int32_T(&ipiv, 2);
  iy = A->size[1];
  eml_signed_integer_colon(iy, ipiv);
  if (A->size[1] < 1) {
  } else {
    if (A->size[1] - 1 <= A->size[1]) {
      i58 = A->size[1] - 1;
    } else {
      i58 = A->size[1];
    }

    for (j = 0; j + 1 <= i58; j++) {
      mmj = n - j;
      c = j * (n + 1);
      if (mmj < 1) {
        iy = -1;
      } else {
        iy = 0;
        if (mmj > 1) {
          ix = c;
          smax = fabs(b_A->data[c]);
          for (k = 1; k + 1 <= mmj; k++) {
            ix++;
            s = fabs(b_A->data[ix]);
            if (s > smax) {
              iy = k;
              smax = s;
            }
          }
        }
      }

      if (b_A->data[c + iy] != 0.0) {
        if (iy != 0) {
          ipiv->data[j] = (j + iy) + 1;
          ix = j;
          iy += j;
          for (k = 1; k <= n; k++) {
            smax = b_A->data[ix];
            b_A->data[ix] = b_A->data[iy];
            b_A->data[iy] = smax;
            ix += n;
            iy += n;
          }
        }

        i59 = c + mmj;
        for (i = c + 1; i + 1 <= i59; i++) {
          b_A->data[i] /= b_A->data[c];
        }
      }

      jAcol = (n - j) - 1;
      jA = c + n;
      iy = c + n;
      for (i = 1; i <= jAcol; i++) {
        smax = b_A->data[iy];
        if (b_A->data[iy] != 0.0) {
          ix = c + 1;
          i59 = mmj + jA;
          for (k = 1 + jA; k + 1 <= i59; k++) {
            b_A->data[k] += b_A->data[ix] * -smax;
            ix++;
          }
        }

        iy += n;
        jA += n;
      }
    }
  }

  if ((A->size[1] == 0) || (B->size[1] == 0)) {
  } else {
    for (j = 0; j + 1 <= n; j++) {
      iy = 91 * j;
      jAcol = n * j;
      for (k = 0; k + 1 <= j; k++) {
        jA = 91 * k;
        if (b_A->data[k + jAcol] != 0.0) {
          for (i = 0; i < 91; i++) {
            B->data[i + iy] -= b_A->data[k + jAcol] * B->data[i + jA];
          }
        }
      }

      smax = 1.0 / b_A->data[j + jAcol];
      for (i = 0; i < 91; i++) {
        B->data[i + iy] *= smax;
      }
    }
  }

  if ((A->size[1] == 0) || (B->size[1] == 0)) {
  } else {
    for (j = A->size[1]; j > 0; j--) {
      iy = 91 * (j - 1);
      jAcol = n * (j - 1);
      for (k = j; k + 1 <= n; k++) {
        jA = 91 * k;
        if (b_A->data[k + jAcol] != 0.0) {
          for (i = 0; i < 91; i++) {
            B->data[i + iy] -= b_A->data[k + jAcol] * B->data[i + jA];
          }
        }
      }
    }
  }

  emxFree_real_T(&b_A);
  for (iy = A->size[1] - 2; iy + 1 > 0; iy--) {
    if (ipiv->data[iy] != iy + 1) {
      jAcol = ipiv->data[iy] - 1;
      for (jA = 0; jA < 91; jA++) {
        smax = B->data[jA + B->size[0] * iy];
        B->data[jA + B->size[0] * iy] = B->data[jA + B->size[0] * jAcol];
        B->data[jA + B->size[0] * jAcol] = smax;
      }
    }
  }

  emxFree_int32_T(&ipiv);
}

//
// Arguments    : int varargin_1
// Return Type  : double
//
static double m_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[27] = { 'n', 'u', 'm', '_', 'p', 'o', 'i', 'n', 't',
    's', '_', 'p', 'e', 'r', '_', 'a', 'n', 'c', 'h', 'o', 'r', ':', ' ', '%',
    'd', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : const double x_data[]
//                const int x_size[1]
// Return Type  : double
//
static double median(const double x_data[], const int x_size[1])
{
  double y;
  int k;
  int midm1;
  int i;
  int idx_data[40];
  emxArray_int32_T *iwork;
  int n;
  int iwork_data[40];
  boolean_T p;
  int i2;
  int j;
  int pEnd;
  int b_p;
  int q;
  int qEnd;
  int kEnd;
  if (x_size[0] == 0) {
    y = rtNaN;
  } else {
    k = x_size[0];
    midm1 = k / 2;
    i = (signed char)x_size[0];
    for (k = 0; k < i; k++) {
      idx_data[k] = 0;
    }

    emxInit_int32_T1(&iwork, 1);
    n = x_size[0] + 1;
    k = iwork->size[0];
    iwork->size[0] = (signed char)x_size[0];
    emxEnsureCapacity((emxArray__common *)iwork, k, (int)sizeof(int));
    i = iwork->size[0];
    for (k = 0; k < i; k++) {
      iwork_data[k] = iwork->data[k];
    }

    emxFree_int32_T(&iwork);
    for (k = 1; k <= n - 2; k += 2) {
      if ((x_data[k - 1] <= x_data[k]) || rtIsNaN(x_data[k])) {
        p = true;
      } else {
        p = false;
      }

      if (p) {
        idx_data[k - 1] = k;
        idx_data[k] = k + 1;
      } else {
        idx_data[k - 1] = k + 1;
        idx_data[k] = k;
      }
    }

    if ((x_size[0] & 1) != 0) {
      idx_data[x_size[0] - 1] = x_size[0];
    }

    i = 2;
    while (i < n - 1) {
      i2 = i << 1;
      j = 1;
      for (pEnd = 1 + i; pEnd < n; pEnd = qEnd + i) {
        b_p = j;
        q = pEnd - 1;
        qEnd = j + i2;
        if (qEnd > n) {
          qEnd = n;
        }

        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          if ((x_data[idx_data[b_p - 1] - 1] <= x_data[idx_data[q] - 1]) ||
              rtIsNaN(x_data[idx_data[q] - 1])) {
            p = true;
          } else {
            p = false;
          }

          if (p) {
            iwork_data[k] = idx_data[b_p - 1];
            b_p++;
            if (b_p == pEnd) {
              while (q + 1 < qEnd) {
                k++;
                iwork_data[k] = idx_data[q];
                q++;
              }
            }
          } else {
            iwork_data[k] = idx_data[q];
            q++;
            if (q + 1 == qEnd) {
              while (b_p < pEnd) {
                k++;
                iwork_data[k] = idx_data[b_p - 1];
                b_p++;
              }
            }
          }

          k++;
        }

        for (k = 0; k + 1 <= kEnd; k++) {
          idx_data[(j + k) - 1] = iwork_data[k];
        }

        j = qEnd;
      }

      i = i2;
    }

    if (rtIsNaN(x_data[idx_data[(signed char)x_size[0] - 1] - 1])) {
      y = x_data[idx_data[(signed char)x_size[0] - 1] - 1];
    } else if (midm1 << 1 == x_size[0]) {
      if (((x_data[idx_data[midm1 - 1] - 1] < 0.0) && (x_data[idx_data[midm1] -
            1] >= 0.0)) || rtIsInf(x_data[idx_data[midm1 - 1] - 1]) || rtIsInf
          (x_data[idx_data[midm1] - 1])) {
        y = (x_data[idx_data[midm1 - 1] - 1] + x_data[idx_data[midm1] - 1]) /
          2.0;
      } else {
        y = x_data[idx_data[midm1 - 1] - 1] + (x_data[idx_data[midm1] - 1] -
          x_data[idx_data[midm1 - 1] - 1]) / 2.0;
      }
    } else {
      y = x_data[idx_data[midm1] - 1];
    }
  }

  return y;
}

//
// Arguments    : int idx[8]
//                double x[8]
//                int offset
//                int np
//                int nq
//                int iwork[8]
//                double xwork[8]
// Return Type  : void
//
static void merge(int idx[8], double x[8], int offset, int np, int nq, int
                  iwork[8], double xwork[8])
{
  int n;
  int qend;
  int p;
  int iout;
  int exitg1;
  if (nq == 0) {
  } else {
    n = np + nq;
    for (qend = 0; qend + 1 <= n; qend++) {
      iwork[qend] = idx[offset + qend];
      xwork[qend] = x[offset + qend];
    }

    p = 0;
    n = np;
    qend = np + nq;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork[p] <= xwork[n]) {
        idx[iout] = iwork[p];
        x[iout] = xwork[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx[iout] = iwork[n];
        x[iout] = xwork[n];
        if (n + 1 < qend) {
          n++;
        } else {
          n = (iout - p) + 1;
          while (p + 1 <= np) {
            idx[n + p] = iwork[p];
            x[n + p] = xwork[p];
            p++;
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

//
// Arguments    : emxArray_int32_T *idx
//                emxArray_real_T *x
//                int offset
//                int n
//                int preSortLevel
//                emxArray_int32_T *iwork
//                emxArray_real_T *xwork
// Return Type  : void
//
static void merge_block(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
  int n, int preSortLevel, emxArray_int32_T *iwork, emxArray_real_T *xwork)
{
  int nPairs;
  int bLen;
  int tailOffset;
  int nTail;
  nPairs = n >> preSortLevel;
  bLen = 1 << preSortLevel;
  while (nPairs > 1) {
    if ((nPairs & 1) != 0) {
      nPairs--;
      tailOffset = bLen * nPairs;
      nTail = n - tailOffset;
      if (nTail > bLen) {
        b_merge(idx, x, offset + tailOffset, bLen, nTail - bLen, iwork, xwork);
      }
    }

    tailOffset = bLen << 1;
    nPairs >>= 1;
    for (nTail = 1; nTail <= nPairs; nTail++) {
      b_merge(idx, x, offset + (nTail - 1) * tailOffset, bLen, bLen, iwork,
              xwork);
    }

    bLen = tailOffset;
  }

  if (n > bLen) {
    b_merge(idx, x, offset, bLen, n - bLen, iwork, xwork);
  }
}

//
// Arguments    : const emxArray_real_T *A
//                const emxArray_real_T *B
//                emxArray_real_T *y
// Return Type  : void
//
static void mrdivide(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *y)
{
  emxArray_real_T *Y;
  emxArray_real_T *b_A;
  emxArray_real_T *tau;
  emxArray_int32_T *jpvt;
  emxArray_real_T *b_B;
  unsigned int unnamed_idx_1;
  int b_m;
  int maxmn;
  int minmn;
  int i;
  int rankR;
  double tol;
  int mn;
  emxInit_real_T1(&Y, 2);
  emxInit_real_T1(&b_A, 2);
  emxInit_real_T(&tau, 1);
  emxInit_int32_T(&jpvt, 2);
  emxInit_real_T1(&b_B, 2);
  if ((A->size[1] == 0) || ((B->size[0] == 0) || (B->size[1] == 0))) {
    unnamed_idx_1 = (unsigned int)B->size[0];
    b_m = y->size[0] * y->size[1];
    y->size[0] = 91;
    y->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)y, b_m, (int)sizeof(double));
    maxmn = 91 * (int)unnamed_idx_1;
    for (b_m = 0; b_m < maxmn; b_m++) {
      y->data[b_m] = 0.0;
    }
  } else if (B->size[0] == B->size[1]) {
    b_m = y->size[0] * y->size[1];
    y->size[0] = 91;
    y->size[1] = A->size[1];
    emxEnsureCapacity((emxArray__common *)y, b_m, (int)sizeof(double));
    maxmn = A->size[0] * A->size[1];
    for (b_m = 0; b_m < maxmn; b_m++) {
      y->data[b_m] = A->data[b_m];
    }

    lusolve(B, y);
  } else {
    b_m = b_A->size[0] * b_A->size[1];
    b_A->size[0] = B->size[1];
    b_A->size[1] = B->size[0];
    emxEnsureCapacity((emxArray__common *)b_A, b_m, (int)sizeof(double));
    maxmn = B->size[0];
    for (b_m = 0; b_m < maxmn; b_m++) {
      minmn = B->size[1];
      for (i = 0; i < minmn; i++) {
        b_A->data[i + b_A->size[0] * b_m] = B->data[b_m + B->size[0] * i];
      }
    }

    xgeqp3(b_A, tau, jpvt);
    rankR = 0;
    if (b_A->size[0] < b_A->size[1]) {
      minmn = b_A->size[0];
      maxmn = b_A->size[1];
    } else {
      minmn = b_A->size[1];
      maxmn = b_A->size[0];
    }

    if (minmn > 0) {
      tol = (double)maxmn * fabs(b_A->data[0]) * 2.2204460492503131E-16;
      while ((rankR < minmn) && (fabs(b_A->data[rankR + b_A->size[0] * rankR]) >=
              tol)) {
        rankR++;
      }
    }

    minmn = b_A->size[1];
    b_m = Y->size[0] * Y->size[1];
    Y->size[0] = minmn;
    Y->size[1] = 91;
    emxEnsureCapacity((emxArray__common *)Y, b_m, (int)sizeof(double));
    maxmn = minmn * 91;
    for (b_m = 0; b_m < maxmn; b_m++) {
      Y->data[b_m] = 0.0;
    }

    b_m = b_B->size[0] * b_B->size[1];
    b_B->size[0] = A->size[1];
    b_B->size[1] = 91;
    emxEnsureCapacity((emxArray__common *)b_B, b_m, (int)sizeof(double));
    for (b_m = 0; b_m < 91; b_m++) {
      maxmn = A->size[1];
      for (i = 0; i < maxmn; i++) {
        b_B->data[i + b_B->size[0] * b_m] = A->data[b_m + A->size[0] * i];
      }
    }

    b_m = b_A->size[0];
    minmn = b_A->size[0];
    mn = b_A->size[1];
    if (minmn <= mn) {
      mn = minmn;
    }

    for (minmn = 0; minmn + 1 <= mn; minmn++) {
      if (tau->data[minmn] != 0.0) {
        for (maxmn = 0; maxmn < 91; maxmn++) {
          tol = b_B->data[minmn + b_B->size[0] * maxmn];
          for (i = minmn + 1; i + 1 <= b_m; i++) {
            tol += b_A->data[i + b_A->size[0] * minmn] * b_B->data[i + b_B->
              size[0] * maxmn];
          }

          tol *= tau->data[minmn];
          if (tol != 0.0) {
            b_B->data[minmn + b_B->size[0] * maxmn] -= tol;
            for (i = minmn + 1; i + 1 <= b_m; i++) {
              b_B->data[i + b_B->size[0] * maxmn] -= b_A->data[i + b_A->size[0] *
                minmn] * tol;
            }
          }
        }
      }
    }

    for (maxmn = 0; maxmn < 91; maxmn++) {
      for (i = 0; i + 1 <= rankR; i++) {
        Y->data[(jpvt->data[i] + Y->size[0] * maxmn) - 1] = b_B->data[i +
          b_B->size[0] * maxmn];
      }

      for (minmn = rankR - 1; minmn + 1 > 0; minmn--) {
        Y->data[(jpvt->data[minmn] + Y->size[0] * maxmn) - 1] /= b_A->data[minmn
          + b_A->size[0] * minmn];
        for (i = 0; i + 1 <= minmn; i++) {
          Y->data[(jpvt->data[i] + Y->size[0] * maxmn) - 1] -= Y->data
            [(jpvt->data[minmn] + Y->size[0] * maxmn) - 1] * b_A->data[i +
            b_A->size[0] * minmn];
        }
      }
    }

    b_m = y->size[0] * y->size[1];
    y->size[0] = 91;
    y->size[1] = Y->size[0];
    emxEnsureCapacity((emxArray__common *)y, b_m, (int)sizeof(double));
    maxmn = Y->size[0];
    for (b_m = 0; b_m < maxmn; b_m++) {
      for (i = 0; i < 91; i++) {
        y->data[i + y->size[0] * b_m] = Y->data[b_m + Y->size[0] * i];
      }
    }
  }

  emxFree_real_T(&b_B);
  emxFree_int32_T(&jpvt);
  emxFree_real_T(&tau);
  emxFree_real_T(&b_A);
  emxFree_real_T(&Y);
}

//
// # coder
// multiplyIdx Create a longer index array from idx
//    for example:
//    idx_mult = multiplyIdx([1 4 5], 2)
//    idx_mult = [1 2 7 8 9 10]
// Arguments    : const double idx_data[]
//                const int idx_size[1]
//                double idx_mult_data[]
//                int idx_mult_size[1]
// Return Type  : void
//
static void multiplyIdx(const double idx_data[], const int idx_size[1], double
  idx_mult_data[], int idx_mult_size[1])
{
  int loop_ub;
  int i10;
  int i;
  int j;

  // 'multiplyIdx:7' idx_mult = zeros(length(idx) * factor, 1);
  idx_mult_size[0] = idx_size[0] << 1;
  loop_ub = idx_size[0] << 1;
  for (i10 = 0; i10 < loop_ub; i10++) {
    idx_mult_data[i10] = 0.0;
  }

  // 'multiplyIdx:8' for i = 1:length(idx)
  for (i = 0; i < idx_size[0]; i++) {
    // 'multiplyIdx:9' for j = 1:factor
    for (j = 0; j < 2; j++) {
      // 'multiplyIdx:10' idx_mult((i-1)*factor + j) = (idx(i) - 1)*factor + j;
      idx_mult_data[(i << 1) + j] = (idx_data[i] - 1.0) * 2.0 + (1.0 + (double)j);
    }
  }
}

//
// Arguments    : int varargin_1
// Return Type  : double
//
static double n_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[24] = { 'm', 'a', 'x', '_', 'e', 'k', 'f', '_', 'i',
    't', 'e', 'r', 'a', 't', 'i', 'o', 'n', 's', ':', ' ', '%', 'd', '\x0a',
    '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : const double x[3]
// Return Type  : double
//
static double norm(const double x[3])
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 3; k++) {
    absxk = fabs(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0 + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

//
// Arguments    : const char varargin_1_data[]
//                const int varargin_1_size[2]
// Return Type  : double
//
static double o_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2])
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  int i4;
  char varargout_1_data[6];
  static const char cfmt[28] = { 'd', 'e', 'l', 'a', 'y', 'e', 'd', '_', 'i',
    'n', 'i', 't', 'i', 'a', 'l', 'i', 'z', 'a', 't', 'i', 'o', 'n', ':', ' ',
    '%', 's', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = varargin_1_size[1];
    for (i4 = 0; i4 < nbytesint; i4++) {
      varargout_1_data[i4] = varargin_1_data[i4];
    }

    varargout_1_data[varargin_1_size[1]] = '\x00';
    nbytesint = fprintf(filestar, cfmt, &varargout_1_data[0]);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : const char varargin_1_data[]
//                const int varargin_1_size[2]
// Return Type  : double
//
static double p_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2])
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  int i5;
  char varargout_1_data[6];
  static const char cfmt[19] = { 'f', 'i', 'x', 'e', 'd', '_', 'f', 'e', 'a',
    't', 'u', 'r', 'e', ':', ' ', '%', 's', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = varargin_1_size[1];
    for (i5 = 0; i5 < nbytesint; i5++) {
      varargout_1_data[i5] = varargin_1_data[i5];
    }

    varargout_1_data[varargin_1_size[1]] = '\x00';
    nbytesint = fprintf(filestar, cfmt, &varargout_1_data[0]);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : const double fp[3]
//                const double cameraParams_FocalLength[2]
//                const double cameraParams_PrincipalPoint[2]
//                const double cameraParams_RadialDistortion[3]
//                double h_d[2]
// Return Type  : void
//
static void predictMeasurementDistortedAtan(const double fp[3], const double
  cameraParams_FocalLength[2], const double cameraParams_PrincipalPoint[2],
  const double cameraParams_RadialDistortion[3], double h_d[2])
{
  double h_cin[2];
  int i;
  double r_u;
  double r_d;

  // 'predictMeasurementStereoDistorted:27' fx = cameraParams.FocalLength(1);
  // 'predictMeasurementStereoDistorted:28' fy = cameraParams.FocalLength(2);
  // 'predictMeasurementStereoDistorted:29' Cx = cameraParams.PrincipalPoint(1); 
  // 'predictMeasurementStereoDistorted:30' Cy = cameraParams.PrincipalPoint(2); 
  // 'predictMeasurementStereoDistorted:31' w  = cameraParams.RadialDistortion(1); 
  // 'predictMeasurementStereoDistorted:33' h_cin = fp(1:2)/fp(3);
  for (i = 0; i < 2; i++) {
    h_cin[i] = fp[i] / fp[2];
  }

  // 'predictMeasurementStereoDistorted:34' r_u = sqrt(h_cin(1)^2 + h_cin(2)^2); 
  r_u = sqrt(h_cin[0] * h_cin[0] + h_cin[1] * h_cin[1]);

  // 'predictMeasurementStereoDistorted:36' if r_u > 0.001
  if (r_u > 0.001) {
    // 'predictMeasurementStereoDistorted:37' r_d = 1/w * atan(2*r_u * tan(w/2)); 
    r_d = 1.0 / cameraParams_RadialDistortion[0] * atan(2.0 * r_u * tan
      (cameraParams_RadialDistortion[0] / 2.0));

    // 'predictMeasurementStereoDistorted:38' h_din = h_cin*r_d/r_u;
    for (i = 0; i < 2; i++) {
      h_cin[i] = h_cin[i] * r_d / r_u;
    }
  } else {
    // 'predictMeasurementStereoDistorted:39' else
    // 'predictMeasurementStereoDistorted:40' h_din = h_cin;
  }

  // 'predictMeasurementStereoDistorted:42' h_d = [fx * h_din(1) + Cx; fy * h_din(2) + Cy]; 
  h_d[0] = cameraParams_FocalLength[0] * h_cin[0] + cameraParams_PrincipalPoint
    [0];
  h_d[1] = cameraParams_FocalLength[1] * h_cin[1] + cameraParams_PrincipalPoint
    [1];
}

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

  // 'predictMeasurementStereoDistorted:46' fx = cameraParams.FocalLength(1);
  // 'predictMeasurementStereoDistorted:47' fy = cameraParams.FocalLength(2);
  // 'predictMeasurementStereoDistorted:48' Cx = cameraParams.PrincipalPoint(1); 
  // 'predictMeasurementStereoDistorted:49' Cy = cameraParams.PrincipalPoint(2); 
  // 'predictMeasurementStereoDistorted:50' k1 = cameraParams.RadialDistortion(1); 
  // 'predictMeasurementStereoDistorted:51' k2 = cameraParams.RadialDistortion(2); 
  // 'predictMeasurementStereoDistorted:52' k3 = cameraParams.RadialDistortion(3); 
  // 'predictMeasurementStereoDistorted:54' h_cin = fp(1:2)/fp(3);
  for (i = 0; i < 2; i++) {
    h_cin[i] = fp[i] / fp[2];
  }

  // 'predictMeasurementStereoDistorted:56' radsq = h_cin(1)^2 + h_cin(2)^2;
  radsq = h_cin[0] * h_cin[0] + h_cin[1] * h_cin[1];

  // 'predictMeasurementStereoDistorted:57' h_din = h_cin*(1 + k1*radsq + k2*radsq^2 + k3*radsq^4); 
  b = ((1.0 + cameraParams_RadialDistortion[0] * radsq) +
       cameraParams_RadialDistortion[1] * (radsq * radsq)) +
    cameraParams_RadialDistortion[2] * rt_powd_snf(radsq, 4.0);
  for (i = 0; i < 2; i++) {
    h_cin[i] *= b;
  }

  // 'predictMeasurementStereoDistorted:59' h_d = [Cx + fx*h_din(1);
  // 'predictMeasurementStereoDistorted:60'     Cy + fy*h_din(2)];
  h_d[0] = cameraParams_PrincipalPoint[0] + cameraParams_FocalLength[0] * h_cin
    [0];
  h_d[1] = cameraParams_PrincipalPoint[1] + cameraParams_FocalLength[1] * h_cin
    [1];
}

//
// predictMeasurementLeft Predict the measurement of a feature given in the left
// camera frame
//    Get the normalized pixel coordinates where a feature given in the left camera
//    frame
// Arguments    : const double fp[3]
//                const double cameraparams_FocalLength[2]
//                const double cameraparams_PrincipalPoint[2]
//                double px[2]
// Return Type  : void
//
static void predictMeasurementMono(const double fp[3], const double
  cameraparams_FocalLength[2], const double cameraparams_PrincipalPoint[2],
  double px[2])
{
  double h_c_n_l[2];
  int i;

  // 'predictMeasurementMono:6' fx = cameraparams.FocalLength(1);
  // 'predictMeasurementMono:7' fy = cameraparams.FocalLength(2);
  // 'predictMeasurementMono:8' Cx = cameraparams.PrincipalPoint(1);
  // 'predictMeasurementMono:9' Cy = cameraparams.PrincipalPoint(2);
  // 'predictMeasurementMono:11' h_c_n_l = fp(1:2)/fp(3);
  for (i = 0; i < 2; i++) {
    h_c_n_l[i] = fp[i] / fp[2];
  }

  //  normalized feature in camera frame
  // 'predictMeasurementMono:13' px = [h_c_n_l(1)*fx + Cx;
  // 'predictMeasurementMono:14'       h_c_n_l(2)*fy + Cy];
  px[0] = h_c_n_l[0] * cameraparams_FocalLength[0] +
    cameraparams_PrincipalPoint[0];
  px[1] = h_c_n_l[1] * cameraparams_FocalLength[1] +
    cameraparams_PrincipalPoint[1];
}

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
static void predictMeasurementStereo(const double fp_l[3], const double
  c_stereoParams_CameraParameters[2], const double
  d_stereoParams_CameraParameters[2], const double
  e_stereoParams_CameraParameters[2], const double
  f_stereoParams_CameraParameters[2], const double stereoParams_r_lr[3], const
  double stereoParams_R_rl[9], double h_cin_l[2], double h_cin_r[2])
{
  double h_c_n_l[2];
  int i;
  double fp_r[3];
  double d3;
  int i26;

  // 'predictMeasurementStereo:7' h_cin_l = predictMeasurementMono(fp_l, stereoParams.CameraParameters1); 
  // predictMeasurementLeft Predict the measurement of a feature given in the left 
  // camera frame
  //    Get the normalized pixel coordinates where a feature given in the left camera 
  //    frame
  // 'predictMeasurementMono:6' fx = cameraparams.FocalLength(1);
  // 'predictMeasurementMono:7' fy = cameraparams.FocalLength(2);
  // 'predictMeasurementMono:8' Cx = cameraparams.PrincipalPoint(1);
  // 'predictMeasurementMono:9' Cy = cameraparams.PrincipalPoint(2);
  // 'predictMeasurementMono:11' h_c_n_l = fp(1:2)/fp(3);
  for (i = 0; i < 2; i++) {
    h_c_n_l[i] = fp_l[i] / fp_l[2];
  }

  //  normalized feature in camera frame
  // 'predictMeasurementMono:13' px = [h_c_n_l(1)*fx + Cx;
  // 'predictMeasurementMono:14'       h_c_n_l(2)*fy + Cy];
  h_cin_l[0] = h_c_n_l[0] * c_stereoParams_CameraParameters[0] +
    d_stereoParams_CameraParameters[0];
  h_cin_l[1] = h_c_n_l[1] * c_stereoParams_CameraParameters[1] +
    d_stereoParams_CameraParameters[1];

  // 'predictMeasurementStereo:9' fp_r = stereoParams.R_rl*fp_l - stereoParams.r_lr; 
  for (i = 0; i < 3; i++) {
    d3 = 0.0;
    for (i26 = 0; i26 < 3; i26++) {
      d3 += stereoParams_R_rl[i + 3 * i26] * fp_l[i26];
    }

    fp_r[i] = d3 - stereoParams_r_lr[i];
  }

  // 'predictMeasurementStereo:11' h_cin_r = predictMeasurementMono(fp_r, stereoParams.CameraParameters2); 
  // predictMeasurementLeft Predict the measurement of a feature given in the left 
  // camera frame
  //    Get the normalized pixel coordinates where a feature given in the left camera 
  //    frame
  // 'predictMeasurementMono:6' fx = cameraparams.FocalLength(1);
  // 'predictMeasurementMono:7' fy = cameraparams.FocalLength(2);
  // 'predictMeasurementMono:8' Cx = cameraparams.PrincipalPoint(1);
  // 'predictMeasurementMono:9' Cy = cameraparams.PrincipalPoint(2);
  // 'predictMeasurementMono:11' h_c_n_l = fp(1:2)/fp(3);
  for (i = 0; i < 2; i++) {
    h_c_n_l[i] = fp_r[i] / fp_r[2];
  }

  //  normalized feature in camera frame
  // 'predictMeasurementMono:13' px = [h_c_n_l(1)*fx + Cx;
  // 'predictMeasurementMono:14'       h_c_n_l(2)*fy + Cy];
  h_cin_r[0] = h_c_n_l[0] * e_stereoParams_CameraParameters[0] +
    f_stereoParams_CameraParameters[0];
  h_cin_r[1] = h_c_n_l[1] * e_stereoParams_CameraParameters[1] +
    f_stereoParams_CameraParameters[1];
}

//
// print all parameters for debug check
// Arguments    : double c_noiseParameters_process_noise
//                double d_noiseParameters_process_noise
//                double e_noiseParameters_process_noise
//                double f_noiseParameters_process_noise
//                double g_noiseParameters_process_noise
//                const double c_noiseParameters_gyro_bias_ini[3]
//                const double c_noiseParameters_acc_bias_init[3]
//                double noiseParameters_image_noise
//                double c_noiseParameters_inv_depth_ini
//                int c_VIOParameters_num_points_per_
//                int VIOParameters_num_anchors
//                int c_VIOParameters_max_ekf_iterati
//                boolean_T VIOParameters_fixed_feature
//                boolean_T c_VIOParameters_delayed_initial
//                boolean_T VIOParameters_mono
//                boolean_T VIOParameters_RANSAC
// Return Type  : void
//
static void printParams(double c_noiseParameters_process_noise, double
  d_noiseParameters_process_noise, double e_noiseParameters_process_noise,
  double f_noiseParameters_process_noise, double g_noiseParameters_process_noise,
  const double c_noiseParameters_gyro_bias_ini[3], const double
  c_noiseParameters_acc_bias_init[3], double noiseParameters_image_noise, double
  c_noiseParameters_inv_depth_ini, int c_VIOParameters_num_points_per_, int
  VIOParameters_num_anchors, int c_VIOParameters_max_ekf_iterati, boolean_T
  VIOParameters_fixed_feature, boolean_T c_VIOParameters_delayed_initial,
  boolean_T VIOParameters_mono, boolean_T VIOParameters_RANSAC)
{
  char cv0[18];
  int i3;
  static const char cv1[18] = { 'N', 'o', 'i', 's', 'e', ' ', 'p', 'a', 'r', 'a',
    'm', 'e', 't', 'e', 'r', 's', ':', '\x00' };

  char cv2[15];
  static const char cv3[15] = { 'V', 'I', 'O', ' ', 'p', 'a', 'r', 'a', 'm', 'e',
    't', 'e', 'r', 's', '\x00' };

  int s_size[2];
  static const char cv4[4] = { 'T', 'r', 'u', 'e' };

  char s_data[5];
  static const char cv5[5] = { 'F', 'a', 'l', 's', 'e' };

  // 'SLAM:124' fprintf('\n');
  b_fprintf();

  // 'SLAM:125' ros_info('Noise parameters:');
  // ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_info:5' if coder.target('MATLAB')
  // 'ros_info:9' elseif ~coder.target('MEX')
  // 'ros_info:10' coder.cinclude('<ros/console.h>')
  // 'ros_info:11' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_info:12' coder.ceval('ROS_INFO', [str, 0], varargin{:});
    for (i3 = 0; i3 < 18; i3++) {
      cv0[i3] = cv1[i3];
    }

    ROS_INFO(cv0);
  }

  // 'SLAM:126' fprintf('qv: %f\n', noiseParameters.process_noise.qv);
  c_fprintf(c_noiseParameters_process_noise);

  // 'SLAM:127' fprintf('qw: %f\n', noiseParameters.process_noise.qw);
  d_fprintf(d_noiseParameters_process_noise);

  // 'SLAM:128' fprintf('qao: %f\n', noiseParameters.process_noise.qao);
  e_fprintf(e_noiseParameters_process_noise);

  // 'SLAM:129' fprintf('qwo: %f\n', noiseParameters.process_noise.qwo);
  f_fprintf(f_noiseParameters_process_noise);

  // 'SLAM:130' fprintf('qR_ci: %f\n', noiseParameters.process_noise.qR_ci);
  g_fprintf(g_noiseParameters_process_noise);

  // 'SLAM:131' fprintf('gyro bias initial unc: [%f, %f, %f]\n', noiseParameters.gyro_bias_initial_unc(1), noiseParameters.gyro_bias_initial_unc(2), noiseParameters.gyro_bias_initial_unc(3)); 
  h_fprintf(c_noiseParameters_gyro_bias_ini[0], c_noiseParameters_gyro_bias_ini
            [1], c_noiseParameters_gyro_bias_ini[2]);

  // 'SLAM:132' fprintf('acc bias initial unc: [%f, %f, %f]\n', noiseParameters.acc_bias_initial_unc(1), noiseParameters.acc_bias_initial_unc(2), noiseParameters.acc_bias_initial_unc(3)); 
  i_fprintf(c_noiseParameters_acc_bias_init[0], c_noiseParameters_acc_bias_init
            [1], c_noiseParameters_acc_bias_init[2]);

  // 'SLAM:133' fprintf('image_noise: %f\n', noiseParameters.image_noise);
  j_fprintf(noiseParameters_image_noise);

  // 'SLAM:134' fprintf('inv_depth_initial_unc: %f\n', noiseParameters.inv_depth_initial_unc); 
  k_fprintf(c_noiseParameters_inv_depth_ini);

  // 'SLAM:136' fprintf('\n');
  b_fprintf();

  // 'SLAM:137' ros_info('VIO parameters');
  // ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_info:5' if coder.target('MATLAB')
  // 'ros_info:9' elseif ~coder.target('MEX')
  // 'ros_info:10' coder.cinclude('<ros/console.h>')
  // 'ros_info:11' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_info:12' coder.ceval('ROS_INFO', [str, 0], varargin{:});
    for (i3 = 0; i3 < 15; i3++) {
      cv2[i3] = cv3[i3];
    }

    ROS_INFO(cv2);
  }

  // 'SLAM:138' fprintf('num_anchors: %d\n', int32(VIOParameters.num_anchors));
  l_fprintf(VIOParameters_num_anchors);

  // 'SLAM:139' fprintf('num_points_per_anchor: %d\n', int32(VIOParameters.num_points_per_anchor)); 
  m_fprintf(c_VIOParameters_num_points_per_);

  // 'SLAM:140' fprintf('max_ekf_iterations: %d\n', int32(VIOParameters.max_ekf_iterations)); 
  n_fprintf(c_VIOParameters_max_ekf_iterati);

  // 'SLAM:141' fprintf('delayed_initialization: %s\n', bool2str(VIOParameters.delayed_initialization)); 
  // 'SLAM:149' if bool
  if (c_VIOParameters_delayed_initial) {
    // 'SLAM:150' s = 'True';
    s_size[0] = 1;
    s_size[1] = 4;
    for (i3 = 0; i3 < 4; i3++) {
      s_data[i3] = cv4[i3];
    }
  } else {
    // 'SLAM:151' else
    // 'SLAM:152' s = 'False';
    s_size[0] = 1;
    s_size[1] = 5;
    for (i3 = 0; i3 < 5; i3++) {
      s_data[i3] = cv5[i3];
    }
  }

  o_fprintf(s_data, s_size);

  // 'SLAM:142' fprintf('fixed_feature: %s\n', bool2str(VIOParameters.fixed_feature)); 
  // 'SLAM:149' if bool
  if (VIOParameters_fixed_feature) {
    // 'SLAM:150' s = 'True';
    s_size[0] = 1;
    s_size[1] = 4;
    for (i3 = 0; i3 < 4; i3++) {
      s_data[i3] = cv4[i3];
    }
  } else {
    // 'SLAM:151' else
    // 'SLAM:152' s = 'False';
    s_size[0] = 1;
    s_size[1] = 5;
    for (i3 = 0; i3 < 5; i3++) {
      s_data[i3] = cv5[i3];
    }
  }

  p_fprintf(s_data, s_size);

  // 'SLAM:143' fprintf('mono: %s\n', bool2str(VIOParameters.mono));
  // 'SLAM:149' if bool
  if (VIOParameters_mono) {
    // 'SLAM:150' s = 'True';
    s_size[0] = 1;
    s_size[1] = 4;
    for (i3 = 0; i3 < 4; i3++) {
      s_data[i3] = cv4[i3];
    }
  } else {
    // 'SLAM:151' else
    // 'SLAM:152' s = 'False';
    s_size[0] = 1;
    s_size[1] = 5;
    for (i3 = 0; i3 < 5; i3++) {
      s_data[i3] = cv5[i3];
    }
  }

  q_fprintf(s_data, s_size);

  // 'SLAM:144' fprintf('RANSAC: %s\n', bool2str(VIOParameters.RANSAC));
  // 'SLAM:149' if bool
  if (VIOParameters_RANSAC) {
    // 'SLAM:150' s = 'True';
    s_size[0] = 1;
    s_size[1] = 4;
    for (i3 = 0; i3 < 4; i3++) {
      s_data[i3] = cv4[i3];
    }
  } else {
    // 'SLAM:151' else
    // 'SLAM:152' s = 'False';
    s_size[0] = 1;
    s_size[1] = 5;
    for (i3 = 0; i3 < 5; i3++) {
      s_data[i3] = cv5[i3];
    }
  }

  r_fprintf(s_data, s_size);

  // 'SLAM:145' fprintf('\n');
  b_fprintf();
}

//
// Arguments    : const char varargin_1_data[]
//                const int varargin_1_size[2]
// Return Type  : double
//
static double q_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2])
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  int i6;
  char varargout_1_data[6];
  static const char cfmt[10] = { 'm', 'o', 'n', 'o', ':', ' ', '%', 's', '\x0a',
    '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = varargin_1_size[1];
    for (i6 = 0; i6 < nbytesint; i6++) {
      varargout_1_data[i6] = varargin_1_data[i6];
    }

    varargout_1_data[varargin_1_size[1]] = '\x00';
    nbytesint = fprintf(filestar, cfmt, &varargout_1_data[0]);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : const double dtheta[3]
//                double dq[4]
// Return Type  : void
//
static void quatPlusThetaJ(const double dtheta[3], double dq[4])
{
  double theta;
  int i;
  double B;

  // 'quatPlusThetaJ:2' theta=norm(dtheta) * 0.5;
  theta = norm(dtheta) * 0.5;

  // 'quatPlusThetaJ:3' if theta < 0.244
  if (theta < 0.244) {
    // 'quatPlusThetaJ:4' dq = [0.5 * dtheta;1];
    for (i = 0; i < 3; i++) {
      dq[i] = 0.5 * dtheta[i];
    }

    dq[3] = 1.0;
  } else {
    // 'quatPlusThetaJ:5' else
    // 'quatPlusThetaJ:6' dq = [  0.5*dtheta(1)*sin(theta)/theta;
    // 'quatPlusThetaJ:7'             0.5*dtheta(2)*sin(theta)/theta;
    // 'quatPlusThetaJ:8'             0.5*dtheta(3)*sin(theta)/theta;
    // 'quatPlusThetaJ:9'          cos(theta)];
    dq[0] = 0.5 * dtheta[0] * sin(theta) / theta;
    dq[1] = 0.5 * dtheta[1] * sin(theta) / theta;
    dq[2] = 0.5 * dtheta[2] * sin(theta) / theta;
    dq[3] = cos(theta);
  }

  // 'quatPlusThetaJ:11' dq = dq/norm(dq);
  B = b_norm(dq);
  for (i = 0; i < 4; i++) {
    dq[i] /= B;
  }
}

//
// Arguments    : const double q[4]
//                const double p[4]
//                double qp[4]
// Return Type  : void
//
static void quatmultJ(const double q[4], const double p[4], double qp[4])
{
  double b_p[16];
  double b_q[4];
  int i8;
  int i9;
  double B;

  // 'quatmultJ:2' if coder.target('MATLAB')
  // 'quatmultJ:10' p1=p(1);
  // 'quatmultJ:11' p2=p(2);
  // 'quatmultJ:12' p3=p(3);
  // 'quatmultJ:13' p4=p(4);
  // 'quatmultJ:15' q1=q(1);
  // 'quatmultJ:16' q2=q(2);
  // 'quatmultJ:17' q3=q(3);
  // 'quatmultJ:18' q4=q(4);
  // 'quatmultJ:19' qp=[p4,-p3,p2,p1;
  // 'quatmultJ:20'     p3,p4,-p1,p2;
  // 'quatmultJ:21'     -p2,p1,p4,p3;
  // 'quatmultJ:22'     -p1,-p2,-p3,p4]*[q1;q2;q3;q4];
  b_p[0] = p[3];
  b_p[4] = -p[2];
  b_p[8] = p[1];
  b_p[12] = p[0];
  b_p[1] = p[2];
  b_p[5] = p[3];
  b_p[9] = -p[0];
  b_p[13] = p[1];
  b_p[2] = -p[1];
  b_p[6] = p[0];
  b_p[10] = p[3];
  b_p[14] = p[2];
  b_p[3] = -p[0];
  b_p[7] = -p[1];
  b_p[11] = -p[2];
  b_p[15] = p[3];
  b_q[0] = q[0];
  b_q[1] = q[1];
  b_q[2] = q[2];
  b_q[3] = q[3];
  for (i8 = 0; i8 < 4; i8++) {
    qp[i8] = 0.0;
    for (i9 = 0; i9 < 4; i9++) {
      qp[i8] += b_p[i8 + (i9 << 2)] * b_q[i9];
    }
  }

  // 'quatmultJ:23' qp = qp/norm(qp);
  B = b_norm(qp);
  for (i8 = 0; i8 < 4; i8++) {
    qp[i8] /= B;
  }
}

//
// Arguments    : const char varargin_1_data[]
//                const int varargin_1_size[2]
// Return Type  : double
//
static double r_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2])
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  int i7;
  char varargout_1_data[6];
  static const char cfmt[12] = { 'R', 'A', 'N', 'S', 'A', 'C', ':', ' ', '%',
    's', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = varargin_1_size[1];
    for (i7 = 0; i7 < nbytesint; i7++) {
      varargout_1_data[i7] = varargin_1_data[i7];
    }

    varargout_1_data[varargin_1_size[1]] = '\x00';
    nbytesint = fprintf(filestar, cfmt, &varargout_1_data[0]);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : const double x[3]
//                double y
//                double z[3]
// Return Type  : void
//
static void rdivide(const double x[3], double y, double z[3])
{
  int i;
  for (i = 0; i < 3; i++) {
    z[i] = x[i] / y;
  }
}

//
// ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
// Arguments    : void
// Return Type  : void
//
static void ros_error()
{
  char cv12[27];
  int i14;
  static const char cv13[27] = { 'p', 'i', 'c', 'k', 'e', 'd', ' ', 'a', 'n',
    ' ', 'i', 'n', 'a', 'c', 't', 'i', 'v', 'e', ' ', 'f', 'e', 'a', 't', 'u',
    'r', 'e', '\x00' };

  // 'ros_error:4' if coder.target('MATLAB')
  // 'ros_error:6' elseif ~coder.target('MEX')
  // 'ros_error:7' coder.cinclude('<ros/console.h>')
  // 'ros_error:8' coder.ceval('ROS_ERROR', [str, 0], varargin{:});
  for (i14 = 0; i14 < 27; i14++) {
    cv12[i14] = cv13[i14];
  }

  ROS_ERROR(cv12);
}

//
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                int varargin_2
//                int varargin_3
// Return Type  : void
//
static void ros_info(int varargin_1, int varargin_2, int varargin_3)
{
  char cv10[54];
  int i13;
  static const char cv11[54] = { 'F', 'i', 'x', 'e', 'd', ' ', 'f', 'e', 'a',
    't', 'u', 'r', 'e', ' ', '%', 'i', ' ', '(', '%', 'i', ' ', 'o', 'n', ' ',
    'a', 'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ')', ' ', 'i', 's', ' ', 'n',
    'o', ' ', 'l', 'o', 'n', 'g', 'e', 'r', ' ', 'v', 'a', 'l', 'i', 'd', '\x00'
  };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_info:5' if coder.target('MATLAB')
  // 'ros_info:9' elseif ~coder.target('MEX')
  // 'ros_info:10' coder.cinclude('<ros/console.h>')
  // 'ros_info:11' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_info:12' coder.ceval('ROS_INFO', [str, 0], varargin{:});
    for (i13 = 0; i13 < 54; i13++) {
      cv10[i13] = cv11[i13];
    }

    ROS_INFO(cv10, varargin_1, varargin_2, varargin_3);
  }
}

//
// ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
// Arguments    : int varargin_1
//                double varargin_2
// Return Type  : void
//
static void ros_warn(int varargin_1, double varargin_2)
{
  char cv16[36];
  int i19;
  static const char cv17[36] = { 'f', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%',
    'i', ' ', 'i', 's', ' ', 'v', 'e', 'r', 'y', ' ', 'c', 'l', 'o', 's', 'e',
    '.', ' ', 'D', 'e', 'p', 't', 'h', ':', ' ', '%', 'f', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_warn:6' if coder.target('MATLAB')
  // 'ros_warn:10' elseif ~coder.target('MEX')
  // 'ros_warn:11' coder.cinclude('<ros/console.h>')
  // 'ros_warn:12' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_warn:13' coder.ceval('ROS_WARN', [str, 0], varargin{:});
    for (i19 = 0; i19 < 36; i19++) {
      cv16[i19] = cv17[i19];
    }

    ROS_WARN(cv16, varargin_1, varargin_2);
  }
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  double d7;
  double d8;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d7 = fabs(u0);
    d8 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d7 == 1.0) {
        y = rtNaN;
      } else if (d7 > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d8 == 0.0) {
      y = 1.0;
    } else if (d8 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

//
// Arguments    : double u
// Return Type  : double
//
static double rt_roundd_snf(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

//
// Arguments    : int varargin_1
// Return Type  : double
//
static double s_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[5] = { '%', 'd', ',', ' ', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : double x[8]
//                int idx[8]
// Return Type  : void
//
static void sort(double x[8], int idx[8])
{
  b_sort(x, idx);
}

//
// Arguments    : emxArray_real_T *x
//                emxArray_int32_T *idx
// Return Type  : void
//
static void sortIdx(emxArray_real_T *x, emxArray_int32_T *idx)
{
  emxArray_real_T *b_x;
  int ib;
  int wOffset;
  int b_m;
  int n;
  double x4[4];
  int idx4[4];
  emxArray_int32_T *iwork;
  emxArray_real_T *xwork;
  int nNaNs;
  int k;
  int i3;
  int i4;
  signed char perm[4];
  int nNonNaN;
  int nBlocks;
  int b_iwork[256];
  double b_xwork[256];
  int bLen2;
  int nPairs;
  int exitg1;
  emxInit_real_T(&b_x, 1);
  ib = x->size[0];
  wOffset = b_x->size[0];
  b_x->size[0] = x->size[0];
  emxEnsureCapacity((emxArray__common *)b_x, wOffset, (int)sizeof(double));
  b_m = x->size[0];
  for (wOffset = 0; wOffset < b_m; wOffset++) {
    b_x->data[wOffset] = x->data[wOffset];
  }

  wOffset = idx->size[0];
  idx->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)idx, wOffset, (int)sizeof(int));
  for (wOffset = 0; wOffset < ib; wOffset++) {
    idx->data[wOffset] = 0;
  }

  n = x->size[0];
  for (b_m = 0; b_m < 4; b_m++) {
    x4[b_m] = 0.0;
    idx4[b_m] = 0;
  }

  emxInit_int32_T1(&iwork, 1);
  wOffset = iwork->size[0];
  iwork->size[0] = ib;
  emxEnsureCapacity((emxArray__common *)iwork, wOffset, (int)sizeof(int));
  b_m = iwork->size[0];
  wOffset = iwork->size[0];
  iwork->size[0] = b_m;
  emxEnsureCapacity((emxArray__common *)iwork, wOffset, (int)sizeof(int));
  for (wOffset = 0; wOffset < b_m; wOffset++) {
    iwork->data[wOffset] = 0;
  }

  emxInit_real_T(&xwork, 1);
  b_m = x->size[0];
  wOffset = xwork->size[0];
  xwork->size[0] = b_m;
  emxEnsureCapacity((emxArray__common *)xwork, wOffset, (int)sizeof(double));
  b_m = xwork->size[0];
  wOffset = xwork->size[0];
  xwork->size[0] = b_m;
  emxEnsureCapacity((emxArray__common *)xwork, wOffset, (int)sizeof(double));
  for (wOffset = 0; wOffset < b_m; wOffset++) {
    xwork->data[wOffset] = 0.0;
  }

  nNaNs = 1;
  ib = 0;
  for (k = 0; k + 1 <= n; k++) {
    if (rtIsNaN(b_x->data[k])) {
      idx->data[n - nNaNs] = k + 1;
      xwork->data[n - nNaNs] = b_x->data[k];
      nNaNs++;
    } else {
      ib++;
      idx4[ib - 1] = k + 1;
      x4[ib - 1] = b_x->data[k];
      if (ib == 4) {
        ib = k - nNaNs;
        if (x4[0] <= x4[1]) {
          b_m = 1;
          wOffset = 2;
        } else {
          b_m = 2;
          wOffset = 1;
        }

        if (x4[2] <= x4[3]) {
          i3 = 3;
          i4 = 4;
        } else {
          i3 = 4;
          i4 = 3;
        }

        if (x4[b_m - 1] <= x4[i3 - 1]) {
          if (x4[wOffset - 1] <= x4[i3 - 1]) {
            perm[0] = (signed char)b_m;
            perm[1] = (signed char)wOffset;
            perm[2] = (signed char)i3;
            perm[3] = (signed char)i4;
          } else if (x4[wOffset - 1] <= x4[i4 - 1]) {
            perm[0] = (signed char)b_m;
            perm[1] = (signed char)i3;
            perm[2] = (signed char)wOffset;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)b_m;
            perm[1] = (signed char)i3;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)wOffset;
          }
        } else if (x4[b_m - 1] <= x4[i4 - 1]) {
          if (x4[wOffset - 1] <= x4[i4 - 1]) {
            perm[0] = (signed char)i3;
            perm[1] = (signed char)b_m;
            perm[2] = (signed char)wOffset;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)i3;
            perm[1] = (signed char)b_m;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)wOffset;
          }
        } else {
          perm[0] = (signed char)i3;
          perm[1] = (signed char)i4;
          perm[2] = (signed char)b_m;
          perm[3] = (signed char)wOffset;
        }

        idx->data[ib - 2] = idx4[perm[0] - 1];
        idx->data[ib - 1] = idx4[perm[1] - 1];
        idx->data[ib] = idx4[perm[2] - 1];
        idx->data[ib + 1] = idx4[perm[3] - 1];
        b_x->data[ib - 2] = x4[perm[0] - 1];
        b_x->data[ib - 1] = x4[perm[1] - 1];
        b_x->data[ib] = x4[perm[2] - 1];
        b_x->data[ib + 1] = x4[perm[3] - 1];
        ib = 0;
      }
    }
  }

  wOffset = x->size[0] - nNaNs;
  if (ib > 0) {
    for (b_m = 0; b_m < 4; b_m++) {
      perm[b_m] = 0;
    }

    if (ib == 1) {
      perm[0] = 1;
    } else if (ib == 2) {
      if (x4[0] <= x4[1]) {
        perm[0] = 1;
        perm[1] = 2;
      } else {
        perm[0] = 2;
        perm[1] = 1;
      }
    } else if (x4[0] <= x4[1]) {
      if (x4[1] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 2;
        perm[2] = 3;
      } else if (x4[0] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 3;
        perm[2] = 2;
      } else {
        perm[0] = 3;
        perm[1] = 1;
        perm[2] = 2;
      }
    } else if (x4[0] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 1;
      perm[2] = 3;
    } else if (x4[1] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 3;
      perm[2] = 1;
    } else {
      perm[0] = 3;
      perm[1] = 2;
      perm[2] = 1;
    }

    for (k = 1; k <= ib; k++) {
      idx->data[(wOffset - ib) + k] = idx4[perm[k - 1] - 1];
      b_x->data[(wOffset - ib) + k] = x4[perm[k - 1] - 1];
    }
  }

  b_m = (nNaNs - 1) >> 1;
  for (k = 1; k <= b_m; k++) {
    ib = idx->data[wOffset + k];
    idx->data[wOffset + k] = idx->data[n - k];
    idx->data[n - k] = ib;
    b_x->data[wOffset + k] = xwork->data[n - k];
    b_x->data[n - k] = xwork->data[wOffset + k];
  }

  if (((nNaNs - 1) & 1) != 0) {
    b_x->data[(wOffset + b_m) + 1] = xwork->data[(wOffset + b_m) + 1];
  }

  nNonNaN = (x->size[0] - nNaNs) + 1;
  b_m = 2;
  if (nNonNaN > 1) {
    if (x->size[0] >= 256) {
      nBlocks = nNonNaN >> 8;
      if (nBlocks > 0) {
        for (i3 = 1; i3 <= nBlocks; i3++) {
          i4 = ((i3 - 1) << 8) - 1;
          for (nNaNs = 0; nNaNs < 6; nNaNs++) {
            n = 1 << (nNaNs + 2);
            bLen2 = n << 1;
            nPairs = 256 >> (nNaNs + 3);
            for (k = 1; k <= nPairs; k++) {
              b_m = i4 + (k - 1) * bLen2;
              for (ib = 1; ib <= bLen2; ib++) {
                b_iwork[ib - 1] = idx->data[b_m + ib];
                b_xwork[ib - 1] = b_x->data[b_m + ib];
              }

              wOffset = 0;
              ib = n;
              do {
                exitg1 = 0;
                b_m++;
                if (b_xwork[wOffset] <= b_xwork[ib]) {
                  idx->data[b_m] = b_iwork[wOffset];
                  b_x->data[b_m] = b_xwork[wOffset];
                  if (wOffset + 1 < n) {
                    wOffset++;
                  } else {
                    exitg1 = 1;
                  }
                } else {
                  idx->data[b_m] = b_iwork[ib];
                  b_x->data[b_m] = b_xwork[ib];
                  if (ib + 1 < bLen2) {
                    ib++;
                  } else {
                    ib = b_m - wOffset;
                    while (wOffset + 1 <= n) {
                      idx->data[(ib + wOffset) + 1] = b_iwork[wOffset];
                      b_x->data[(ib + wOffset) + 1] = b_xwork[wOffset];
                      wOffset++;
                    }

                    exitg1 = 1;
                  }
                }
              } while (exitg1 == 0);
            }
          }
        }

        b_m = nBlocks << 8;
        ib = nNonNaN - b_m;
        if (ib > 0) {
          merge_block(idx, b_x, b_m, ib, 2, iwork, xwork);
        }

        b_m = 8;
      }
    }

    merge_block(idx, b_x, 0, nNonNaN, b_m, iwork, xwork);
  }

  emxFree_real_T(&xwork);
  emxFree_int32_T(&iwork);
  wOffset = x->size[0];
  x->size[0] = b_x->size[0];
  emxEnsureCapacity((emxArray__common *)x, wOffset, (int)sizeof(double));
  b_m = b_x->size[0];
  for (wOffset = 0; wOffset < b_m; wOffset++) {
    x->data[wOffset] = b_x->data[wOffset];
  }

  emxFree_real_T(&b_x);
}

//
// UNDISTORTPOINT Undistort a point (or points) that are from a camera with
// the calibration cameraparams
//    Undistort a point or set of points from one camera. Depending on the
//    camera model used to calibrate the camera, the appropriate undistortion
//    is applied
// Arguments    : const double pt_d_data[]
//                const int pt_d_size[1]
//                double cameraparams_ATAN
//                const double cameraparams_FocalLength[2]
//                const double cameraparams_PrincipalPoint[2]
//                const double cameraparams_RadialDistortion[3]
//                double cameraparams_DistortionModel
//                double pt_u_data[]
//                int pt_u_size[1]
// Return Type  : void
//
static void undistortPoint(const double pt_d_data[], const int pt_d_size[1],
  double cameraparams_ATAN, const double cameraparams_FocalLength[2], const
  double cameraparams_PrincipalPoint[2], const double
  cameraparams_RadialDistortion[3], double cameraparams_DistortionModel, double
  pt_u_data[], int pt_u_size[1])
{
  int loop_ub;
  int i11;
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
  int b_i;
  int c_i;
  boolean_T exitg1;
  double a;
  double b_a;
  double diff;
  char cv6[24];
  static const char cv7[24] = { 'r', 'e', 'a', 'c', 'h', 'e', 'd', ' ', 'i', 't',
    'e', 'r', 'a', 't', 'i', 'o', 'n', ' ', 'l', 'i', 'm', 'i', 't', '\x00' };

  char cv8[52];
  static const char cv9[52] = { 'n', 'e', 'g', 'a', 't', 'i', 'v', 'e', ' ', 'u',
    'n', 'd', 'i', 's', 't', 'o', 'r', 't', 'e', 'd', ' ', 'r', 'a', 'd', 'i',
    'u', 's', '.', ' ', 'r', 'd', '_', 's', 'q', ' ', '=', ' ', '%', 'f', ',',
    ' ', 'r', 'u', '_', 's', 'q', ' ', '=', ' ', '%', 'f', '\x00' };

  double coeff;

  // 'undistortPoint:8' if cameraparams.DistortionModel == cameraparams.ATAN
  if (cameraparams_DistortionModel == cameraparams_ATAN) {
    // 'undistortPoint:9' pt_u = undistortPointAtan(pt_d, cameraparams);
    undistortPointAtan(pt_d_data, pt_d_size, cameraparams_FocalLength,
                       cameraparams_PrincipalPoint,
                       cameraparams_RadialDistortion, pt_u_data, pt_u_size);
  } else {
    // 'undistortPoint:10' else
    // 'undistortPoint:11' pt_u = undistortPointPB(pt_d, cameraparams);
    // % Plumb Bob
    // 'undistortPoint:58' if mod(length(pt_d), 2)
    // 'undistortPoint:62' pt_u = pt_d;
    pt_u_size[0] = pt_d_size[0];
    loop_ub = pt_d_size[0];
    for (i11 = 0; i11 < loop_ub; i11++) {
      pt_u_data[i11] = pt_d_data[i11];
    }

    // 'undistortPoint:64' fx = cameraParameters.FocalLength(1);
    fx = cameraparams_FocalLength[0];

    // 'undistortPoint:65' fy = cameraParameters.FocalLength(2);
    fy = cameraparams_FocalLength[1];

    // 'undistortPoint:66' Cx = cameraParameters.PrincipalPoint(1);
    Cx = cameraparams_PrincipalPoint[0];

    // 'undistortPoint:67' Cy = cameraParameters.PrincipalPoint(2);
    Cy = cameraparams_PrincipalPoint[1];

    // 'undistortPoint:68' k1 = cameraParameters.RadialDistortion(1);
    k1 = cameraparams_RadialDistortion[0];

    // 'undistortPoint:69' k2 = cameraParameters.RadialDistortion(2);
    k2 = cameraparams_RadialDistortion[1];

    // 'undistortPoint:70' k3 = cameraParameters.RadialDistortion(3);
    k3 = cameraparams_RadialDistortion[2];

    // 'undistortPoint:72' for i = 1:length(pt_d)/2
    d1 = (double)pt_d_size[0] / 2.0;
    for (i = 0; i < (int)d1; i++) {
      // 'undistortPoint:74' pt_d_n = [(pt_d((i-1)*2 + 1)-Cx)/fx;
      // 'undistortPoint:75'     (pt_d((i-1)*2 + 2)-Cy)/fy];
      pt_d_n[0] = (pt_d_data[i << 1] - Cx) / fx;
      pt_d_n[1] = (pt_d_data[(i << 1) + 1] - Cy) / fy;

      // 'undistortPoint:77' r_d_sq = pt_d_n(1)^2 + pt_d_n(2)^2;
      r_d_sq = pt_d_n[0] * pt_d_n[0] + pt_d_n[1] * pt_d_n[1];

      // 'undistortPoint:78' r_u_sq = get_r_u(k1, k2, k3, r_d_sq);
      // get_r_u Get undistorted radius from distorted radius
      //    Get the pixel radius of the undistorted pixels from a distorted pixel 
      //    radius and distortion parameters
      // 'get_r_u:5' x = rd_sq;
      r_u_sq = r_d_sq;

      // 'get_r_u:6' thresh = 1e-6;
      // 'get_r_u:8' for i = 1:100
      b_i = 1;
      c_i = 0;
      exitg1 = false;
      while ((!exitg1) && (c_i < 100)) {
        b_i = c_i + 1;

        // 'get_r_u:9' f_x = x * (1 + k1*x + k2*x^2 + k3*x^3)^2 - rd_sq;
        a = ((1.0 + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq)) + k3 * rt_powd_snf
          (r_u_sq, 3.0);

        // 'get_r_u:10' f_prime_x = (1 + k1*x + k2*x^2 * k3*x^3)^2 + 2*x*(1 + 2*k1*x + 2*k2*x^2 + 2*k3*x^3); 
        b_a = (1.0 + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq) * k3 * rt_powd_snf
          (r_u_sq, 3.0);

        // 'get_r_u:12' diff = f_x / f_prime_x;
        diff = (r_u_sq * (a * a) - r_d_sq) / (b_a * b_a + 2.0 * r_u_sq * (((1.0
          + 2.0 * k1 * r_u_sq) + 2.0 * k2 * (r_u_sq * r_u_sq)) + 2.0 * k3 *
          rt_powd_snf(r_u_sq, 3.0)));

        // 'get_r_u:13' x = x - diff;
        r_u_sq -= diff;

        // 'get_r_u:14' if diff < thresh && diff > -thresh
        if ((diff < 1.0E-6) && (diff > -1.0E-6)) {
          exitg1 = true;
        } else {
          c_i++;
        }
      }

      // 'get_r_u:19' if i == 100
      if ((b_i == 100) && (debug_level >= 2.0)) {
        // 'get_r_u:20' ros_warn('reached iteration limit')
        // ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
        //  debug_level == 0: print errors, == 1: print warnings, == 2: print info 
        // 'ros_warn:6' if coder.target('MATLAB')
        // 'ros_warn:10' elseif ~coder.target('MEX')
        // 'ros_warn:11' coder.cinclude('<ros/console.h>')
        // 'ros_warn:12' if debug_level >= 2
        // 'ros_warn:13' coder.ceval('ROS_WARN', [str, 0], varargin{:});
        for (i11 = 0; i11 < 24; i11++) {
          cv6[i11] = cv7[i11];
        }

        ROS_WARN(cv6);
      }

      // 'get_r_u:23' if x < 0
      if ((r_u_sq < 0.0) && (debug_level >= 2.0)) {
        // 'get_r_u:24' ros_warn('negative undistorted radius. rd_sq = %f, ru_sq = %f', rd_sq, x); 
        // ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
        //  debug_level == 0: print errors, == 1: print warnings, == 2: print info 
        // 'ros_warn:6' if coder.target('MATLAB')
        // 'ros_warn:10' elseif ~coder.target('MEX')
        // 'ros_warn:11' coder.cinclude('<ros/console.h>')
        // 'ros_warn:12' if debug_level >= 2
        // 'ros_warn:13' coder.ceval('ROS_WARN', [str, 0], varargin{:});
        for (i11 = 0; i11 < 52; i11++) {
          cv8[i11] = cv9[i11];
        }

        ROS_WARN(cv8, r_d_sq, r_u_sq);
      }

      // 'undistortPoint:80' coeff = (1 + k1*r_u_sq + k2*r_u_sq^2 + k3*r_u_sq^3); 
      coeff = ((1.0 + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq)) + k3 * rt_powd_snf
        (r_u_sq, 3.0);

      // 'undistortPoint:82' pt_u_n = pt_d_n / coeff;
      for (i11 = 0; i11 < 2; i11++) {
        pt_d_n[i11] /= coeff;
      }

      // 'undistortPoint:84' pt_u((i-1)*2 + (1:2)) = [pt_u_n(1)*fx + Cx; pt_u_n(2)*fy + Cy]; 
      b_i = i << 1;
      pt_u_data[b_i] = pt_d_n[0] * fx + Cx;
      pt_u_data[1 + b_i] = pt_d_n[1] * fy + Cy;
    }
  }
}

//
// Arguments    : const double pt_d_data[]
//                const int pt_d_size[1]
//                const double cameraparams_FocalLength[2]
//                const double cameraparams_PrincipalPoint[2]
//                const double cameraparams_RadialDistortion[3]
//                double pt_u_data[]
//                int pt_u_size[1]
// Return Type  : void
//
static void undistortPointAtan(const double pt_d_data[], const int pt_d_size[1],
  const double cameraparams_FocalLength[2], const double
  cameraparams_PrincipalPoint[2], const double cameraparams_RadialDistortion[3],
  double pt_u_data[], int pt_u_size[1])
{
  int rankR;
  int i12;
  double fx;
  double fy;
  double Cx;
  double Cy;
  double d2;
  int i;
  double pt_d_n[2];
  double r_d;
  double x[3];
  double b_x[3];
  double A[3];
  double xnorm;
  double atmp;
  double tau;
  int knt;
  double r_u;

  // % Atan
  // 'undistortPoint:20' if mod(length(pt_d), 2)
  // 'undistortPoint:24' pt_u = pt_d;
  pt_u_size[0] = pt_d_size[0];
  rankR = pt_d_size[0];
  for (i12 = 0; i12 < rankR; i12++) {
    pt_u_data[i12] = pt_d_data[i12];
  }

  // 'undistortPoint:26' fx = cameraparams.FocalLength(1);
  fx = cameraparams_FocalLength[0];

  // 'undistortPoint:27' fy = cameraparams.FocalLength(2);
  fy = cameraparams_FocalLength[1];

  // 'undistortPoint:28' Cx = cameraparams.PrincipalPoint(1);
  Cx = cameraparams_PrincipalPoint[0];

  // 'undistortPoint:29' Cy = cameraparams.PrincipalPoint(2);
  Cy = cameraparams_PrincipalPoint[1];

  // 'undistortPoint:30' w  = cameraparams.RadialDistortion;
  // 'undistortPoint:32' for i = 1:length(pt_d)/2
  d2 = (double)pt_d_size[0] / 2.0;
  for (i = 0; i < (int)d2; i++) {
    // 'undistortPoint:34' pt_d_n = [(pt_d((i-1)*2 + 1)-Cx)/fx;
    // 'undistortPoint:35'     (pt_d((i-1)*2 + 2)-Cy)/fy];
    pt_d_n[0] = (pt_d_data[i << 1] - Cx) / fx;
    pt_d_n[1] = (pt_d_data[(i << 1) + 1] - Cy) / fy;

    // 'undistortPoint:37' r_d = sqrt(pt_d_n(1)^2 + pt_d_n(2)^2);
    r_d = sqrt(pt_d_n[0] * pt_d_n[0] + pt_d_n[1] * pt_d_n[1]);

    // 'undistortPoint:39' if r_d > 0.001
    if (r_d > 0.001) {
      // 'undistortPoint:40' r_u = tan(r_d * w)/(2*tan(w/2));
      for (rankR = 0; rankR < 3; rankR++) {
        xnorm = 2.0 * tan(cameraparams_RadialDistortion[rankR] / 2.0);
        A[rankR] = xnorm;
        x[rankR] = tan(r_d * cameraparams_RadialDistortion[rankR]);
        b_x[rankR] = xnorm;
      }

      atmp = b_x[0];
      tau = 0.0;
      xnorm = xnrm2(b_x);
      if (xnorm != 0.0) {
        xnorm = hypot(b_x[0], xnorm);
        if (b_x[0] >= 0.0) {
          xnorm = -xnorm;
        }

        if (fabs(xnorm) < 1.0020841800044864E-292) {
          knt = 0;
          do {
            knt++;
            for (rankR = 0; rankR < 2; rankR++) {
              A[rankR + 1] *= 9.9792015476736E+291;
            }

            xnorm *= 9.9792015476736E+291;
            atmp *= 9.9792015476736E+291;
          } while (!(fabs(xnorm) >= 1.0020841800044864E-292));

          xnorm = hypot(atmp, xnrm2(A));
          if (atmp >= 0.0) {
            xnorm = -xnorm;
          }

          tau = (xnorm - atmp) / xnorm;
          atmp = 1.0 / (atmp - xnorm);
          for (rankR = 0; rankR < 2; rankR++) {
            A[rankR + 1] *= atmp;
          }

          for (rankR = 1; rankR <= knt; rankR++) {
            xnorm *= 1.0020841800044864E-292;
          }

          atmp = xnorm;
        } else {
          tau = (xnorm - b_x[0]) / xnorm;
          atmp = 1.0 / (b_x[0] - xnorm);
          for (rankR = 0; rankR < 2; rankR++) {
            A[rankR + 1] *= atmp;
          }

          atmp = xnorm;
        }
      }

      A[0] = atmp;
      rankR = 0;
      xnorm = 3.0 * fabs(atmp) * 2.2204460492503131E-16;
      while ((rankR < 1) && (fabs(atmp) >= xnorm)) {
        rankR = 1;
      }

      r_u = 0.0;
      for (i12 = 0; i12 < 3; i12++) {
        b_x[i12] = x[i12];
      }

      if (tau != 0.0) {
        xnorm = x[0];
        for (knt = 0; knt < 2; knt++) {
          xnorm += A[knt + 1] * x[knt + 1];
        }

        xnorm *= tau;
        if (xnorm != 0.0) {
          b_x[0] = x[0] - xnorm;
          for (knt = 0; knt < 2; knt++) {
            b_x[knt + 1] -= A[knt + 1] * xnorm;
          }
        }
      }

      knt = 1;
      while (knt <= rankR) {
        r_u = b_x[0];
        knt = 2;
      }

      while (rankR > 0) {
        r_u /= atmp;
        rankR = 0;
      }

      // 'undistortPoint:42' pt_u_n = pt_d_n*r_u/r_d;
      for (i12 = 0; i12 < 2; i12++) {
        pt_d_n[i12] = pt_d_n[i12] * r_u / r_d;
      }

      // 'undistortPoint:43' pt_u((i-1)*2 + (1:2)) = [pt_u_n(1)*fx + Cx; pt_u_n(2)*fy + Cy]; 
      knt = i << 1;
      pt_u_data[knt] = pt_d_n[0] * fx + Cx;
      pt_u_data[1 + knt] = pt_d_n[1] * fy + Cy;
    } else {
      // 'undistortPoint:45' else
      // 'undistortPoint:46' pt_u((i-1)*2 + (1:2)) = pt_d((i-1)*2 + (1:2));
      knt = i << 1;
      rankR = i << 1;
      for (i12 = 0; i12 < 2; i12++) {
        pt_u_data[i12 + rankR] = pt_d_data[i12 + knt];
      }
    }
  }
}

//
// Arguments    : emxArray_real_T *A
//                emxArray_real_T *tau
//                emxArray_int32_T *jpvt
// Return Type  : void
//
static void xgeqp3(emxArray_real_T *A, emxArray_real_T *tau, emxArray_int32_T
                   *jpvt)
{
  int b_m;
  int n;
  int mn;
  int i60;
  emxArray_real_T *work;
  int itemp;
  emxArray_real_T *vn1;
  emxArray_real_T *vn2;
  int k;
  int nmi;
  int i;
  int i_i;
  int mmi;
  int ix;
  double smax;
  double s;
  int pvt;
  int iy;
  double absxk;
  int i_ip1;
  int lastv;
  int lastc;
  boolean_T exitg2;
  int exitg1;
  double t;
  b_m = A->size[0];
  n = A->size[1];
  if (A->size[0] <= A->size[1]) {
    mn = A->size[0];
  } else {
    mn = A->size[1];
  }

  i60 = tau->size[0];
  tau->size[0] = mn;
  emxEnsureCapacity((emxArray__common *)tau, i60, (int)sizeof(double));
  eml_signed_integer_colon(A->size[1], jpvt);
  if ((A->size[0] == 0) || (A->size[1] == 0)) {
  } else {
    emxInit_real_T(&work, 1);
    itemp = A->size[1];
    i60 = work->size[0];
    work->size[0] = itemp;
    emxEnsureCapacity((emxArray__common *)work, i60, (int)sizeof(double));
    for (i60 = 0; i60 < itemp; i60++) {
      work->data[i60] = 0.0;
    }

    emxInit_real_T(&vn1, 1);
    emxInit_real_T(&vn2, 1);
    itemp = A->size[1];
    i60 = vn1->size[0];
    vn1->size[0] = itemp;
    emxEnsureCapacity((emxArray__common *)vn1, i60, (int)sizeof(double));
    i60 = vn2->size[0];
    vn2->size[0] = vn1->size[0];
    emxEnsureCapacity((emxArray__common *)vn2, i60, (int)sizeof(double));
    k = 1;
    for (nmi = 0; nmi + 1 <= n; nmi++) {
      vn1->data[nmi] = b_xnrm2(b_m, A, k);
      vn2->data[nmi] = vn1->data[nmi];
      k += b_m;
    }

    for (i = 0; i + 1 <= mn; i++) {
      i_i = i + i * b_m;
      nmi = n - i;
      mmi = (b_m - i) - 1;
      if (nmi < 1) {
        itemp = -1;
      } else {
        itemp = 0;
        if (nmi > 1) {
          ix = i;
          smax = fabs(vn1->data[i]);
          for (k = 0; k + 2 <= nmi; k++) {
            ix++;
            s = fabs(vn1->data[ix]);
            if (s > smax) {
              itemp = k + 1;
              smax = s;
            }
          }
        }
      }

      pvt = i + itemp;
      if (pvt + 1 != i + 1) {
        ix = b_m * pvt;
        iy = b_m * i;
        for (k = 1; k <= b_m; k++) {
          smax = A->data[ix];
          A->data[ix] = A->data[iy];
          A->data[iy] = smax;
          ix++;
          iy++;
        }

        itemp = jpvt->data[pvt];
        jpvt->data[pvt] = jpvt->data[i];
        jpvt->data[i] = itemp;
        vn1->data[pvt] = vn1->data[i];
        vn2->data[pvt] = vn2->data[i];
      }

      if (i + 1 < b_m) {
        absxk = A->data[i_i];
        s = 0.0;
        if (1 + mmi <= 0) {
        } else {
          smax = c_xnrm2(mmi, A, i_i + 2);
          if (smax != 0.0) {
            smax = hypot(A->data[i_i], smax);
            if (A->data[i_i] >= 0.0) {
              smax = -smax;
            }

            if (fabs(smax) < 1.0020841800044864E-292) {
              pvt = 0;
              do {
                pvt++;
                xscal(mmi, 9.9792015476736E+291, A, i_i + 2);
                smax *= 9.9792015476736E+291;
                absxk *= 9.9792015476736E+291;
              } while (!(fabs(smax) >= 1.0020841800044864E-292));

              smax = c_xnrm2(mmi, A, i_i + 2);
              smax = hypot(absxk, smax);
              if (absxk >= 0.0) {
                smax = -smax;
              }

              s = (smax - absxk) / smax;
              xscal(mmi, 1.0 / (absxk - smax), A, i_i + 2);
              for (k = 1; k <= pvt; k++) {
                smax *= 1.0020841800044864E-292;
              }

              absxk = smax;
            } else {
              s = (smax - A->data[i_i]) / smax;
              xscal(mmi, 1.0 / (A->data[i_i] - smax), A, i_i + 2);
              absxk = smax;
            }
          }
        }

        tau->data[i] = s;
        A->data[i_i] = absxk;
      } else {
        tau->data[i] = 0.0;
      }

      if (i + 1 < n) {
        absxk = A->data[i_i];
        A->data[i_i] = 1.0;
        i_ip1 = (i + (i + 1) * b_m) + 1;
        if (tau->data[i] != 0.0) {
          lastv = mmi;
          itemp = i_i + mmi;
          while ((lastv + 1 > 0) && (A->data[itemp] == 0.0)) {
            lastv--;
            itemp--;
          }

          lastc = nmi - 1;
          exitg2 = false;
          while ((!exitg2) && (lastc > 0)) {
            itemp = i_ip1 + (lastc - 1) * b_m;
            k = itemp;
            do {
              exitg1 = 0;
              if (k <= itemp + lastv) {
                if (A->data[k - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  k++;
                }
              } else {
                lastc--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = -1;
          lastc = 0;
        }

        if (lastv + 1 > 0) {
          if (lastc == 0) {
          } else {
            for (iy = 1; iy <= lastc; iy++) {
              work->data[iy - 1] = 0.0;
            }

            iy = 0;
            i60 = i_ip1 + b_m * (lastc - 1);
            itemp = i_ip1;
            while ((b_m > 0) && (itemp <= i60)) {
              ix = i_i;
              smax = 0.0;
              pvt = itemp + lastv;
              for (k = itemp; k <= pvt; k++) {
                smax += A->data[k - 1] * A->data[ix];
                ix++;
              }

              work->data[iy] += smax;
              iy++;
              itemp += b_m;
            }
          }

          if (-tau->data[i] == 0.0) {
          } else {
            itemp = 0;
            for (nmi = 1; nmi <= lastc; nmi++) {
              if (work->data[itemp] != 0.0) {
                smax = work->data[itemp] * -tau->data[i];
                ix = i_i;
                i60 = lastv + i_ip1;
                for (pvt = i_ip1; pvt <= i60; pvt++) {
                  A->data[pvt - 1] += A->data[ix] * smax;
                  ix++;
                }
              }

              itemp++;
              i_ip1 += b_m;
            }
          }
        }

        A->data[i_i] = absxk;
      }

      for (nmi = i + 1; nmi + 1 <= n; nmi++) {
        itemp = (i + b_m * nmi) + 1;
        if (vn1->data[nmi] != 0.0) {
          smax = fabs(A->data[i + A->size[0] * nmi]) / vn1->data[nmi];
          smax = 1.0 - smax * smax;
          if (smax < 0.0) {
            smax = 0.0;
          }

          s = vn1->data[nmi] / vn2->data[nmi];
          s = smax * (s * s);
          if (s <= 1.4901161193847656E-8) {
            if (i + 1 < b_m) {
              smax = 0.0;
              if (mmi < 1) {
              } else if (mmi == 1) {
                smax = fabs(A->data[itemp]);
              } else {
                s = 2.2250738585072014E-308;
                pvt = itemp + mmi;
                while (itemp + 1 <= pvt) {
                  absxk = fabs(A->data[itemp]);
                  if (absxk > s) {
                    t = s / absxk;
                    smax = 1.0 + smax * t * t;
                    s = absxk;
                  } else {
                    t = absxk / s;
                    smax += t * t;
                  }

                  itemp++;
                }

                smax = s * sqrt(smax);
              }

              vn1->data[nmi] = smax;
              vn2->data[nmi] = vn1->data[nmi];
            } else {
              vn1->data[nmi] = 0.0;
              vn2->data[nmi] = 0.0;
            }
          } else {
            vn1->data[nmi] *= sqrt(smax);
          }
        }
      }
    }

    emxFree_real_T(&vn2);
    emxFree_real_T(&vn1);
    emxFree_real_T(&work);
  }
}

//
// Arguments    : const double x[3]
// Return Type  : double
//
static double xnrm2(const double x[3])
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 2; k++) {
    absxk = fabs(x[k + 1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0 + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

//
// Arguments    : int n
//                double a
//                emxArray_real_T *x
//                int ix0
// Return Type  : void
//
static void xscal(int n, double a, emxArray_real_T *x, int ix0)
{
  int i61;
  int k;
  i61 = (ix0 + n) - 1;
  for (k = ix0; k <= i61; k++) {
    x->data[k - 1] *= a;
  }
}

//
// input
//  NOTE: Comment this out for MEXing
// Arguments    : int updateVect[40]
//                const double z_all_l[80]
//                const double z_all_r[80]
//                double dt
//                const VIOMeasurements *measurements
//                const DUOParameters *cameraParameters
//                const NoiseParameters *noiseParameters
//                const VIOParameters *b_VIOParameters
//                boolean_T vision
//                RobotState *xt_out
//                double h_u_out[160]
//                double map_out[120]
//                AnchorPose anchor_poses_out[5]
//                double delayedStatus_out[40]
// Return Type  : void
//
void SLAM(int updateVect[40], const double z_all_l[80], const double z_all_r[80],
          double dt, const VIOMeasurements *measurements, const DUOParameters
          *cameraParameters, const NoiseParameters *noiseParameters, const
          VIOParameters *b_VIOParameters, boolean_T vision, RobotState *xt_out,
          double h_u_out[160], double map_out[120], AnchorPose anchor_poses_out
          [5], double delayedStatus_out[40])
{
  int i;
  static const signed char iv1[4] = { 0, 0, 0, 1 };

  static const f_struct_T rv0[5] = { { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 },
      { 0, 0, 0, 0, 0, 0 }, { { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0,
          0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, {
            0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0,
            0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0,
            0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 },
          0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, {
          0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 } } }, { { 0.0, 0.0,
        0.0 }, { 0.0, 0.0, 0.0, 1.0 }, { 0, 0, 0, 0, 0, 0 }, { { 0.0, { 0.0, 0.0,
            0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0,
            0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 },
          0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, {
          0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0,
            0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 },
            { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0,
            0.0 }, 0, 0, 0 } } }, { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 }, {
        0, 0, 0, 0, 0, 0 }, { { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0,
          0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, {
            0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0,
            0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0,
            0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 },
          0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, {
          0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 } } }, { { 0.0, 0.0,
        0.0 }, { 0.0, 0.0, 0.0, 1.0 }, { 0, 0, 0, 0, 0, 0 }, { { 0.0, { 0.0, 0.0,
            0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0,
            0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 },
          0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, {
          0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0,
            0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 },
            { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0,
            0.0 }, 0, 0, 0 } } }, { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 }, {
        0, 0, 0, 0, 0, 0 }, { { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0,
          0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, {
            0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0,
            0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0,
            0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 },
          0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, {
          0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 } } } };

  int anchorIdx;
  int i46;
  int i47;
  double z_b[3];
  double B;
  double y_n_b[3];
  static const double dv3[3] = { 0.0, 0.0, 1.0 };

  double b_y_n_b[3];
  double x_n_b[3];
  double dv4[9];
  double b_x_n_b[9];
  double R_cw_init[9];
  static const signed char y[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double dv5[9];
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 0 };

  double b_z_all_l[80];
  double b_z_all_r[80];
  double t0_IMU_att[4];
  double t0_IMU_pos[3];
  double t0_IMU_acc_bias[3];
  double t0_IMU_gyro_bias[3];
  double t0_vel[3];
  double t0_att[4];
  struct_T rv1[5];

  // 'SLAM:4' coder.cstructname(measurements, 'VIOMeasurements', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  // 'SLAM:5' coder.cstructname(noiseParameters, 'NoiseParameters', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  // 'SLAM:6' coder.cstructname(noiseParameters.process_noise, 'ProcessNoise', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  // 'SLAM:7' coder.cstructname(cameraParameters, 'DUOParameters', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  // 'SLAM:8' coder.cstructname(cameraParameters.CameraParameters1, 'CameraParameters', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  // 'SLAM:9' coder.cstructname(cameraParameters.CameraParameters2, 'CameraParameters', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  // 'SLAM:10' coder.cstructname(VIOParameters, 'VIOParameters', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  //  TODO
  //  coder.cstructname(ControllerGains, 'ControllerGains', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  //  coder.cstructname(ref, 'ReferenceCommand', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  // 'SLAM:14' assert ( all ( size (updateVect) == [numTrackFeatures 1] ) );
  // 'SLAM:14' assert(isa(updateVect,'int32'));
  // 'SLAM:15' assert ( all ( size (z_all_l) == [numTrackFeatures*2 1] ) )
  // 'SLAM:16' assert ( all ( size (z_all_r) == [numTrackFeatures*2 1] ) )
  // 'SLAM:17' assert ( all ( size (dt) == [1] ) )
  // 'SLAM:18' assert(isa(vision,'logical'));
  // 'SLAM:22' if isempty(initialized)
  if (!initialized_not_empty) {
    // 'SLAM:23' updateVect(:) = 0;
    for (i = 0; i < 40; i++) {
      updateVect[i] = 0;
    }

    // 'SLAM:25' xt.robot_state.IMU.pos = cameraParameters.t_ci;
    // 'SLAM:26' xt.robot_state.IMU.att = QuatFromRotJ(cameraParameters.R_ci);
    QuatFromRotJ(cameraParameters->R_ci, xt.robot_state.IMU.att);

    // 'SLAM:28' xt.robot_state.IMU.gyro_bias = cameraParameters.gyro_bias;
    // 'SLAM:29' xt.robot_state.IMU.acc_bias = cameraParameters.acc_bias;
    // 'SLAM:31' xt.robot_state.pos = [0; 0; 0];
    for (i = 0; i < 3; i++) {
      xt.robot_state.IMU.pos[i] = cameraParameters->t_ci[i];
      xt.robot_state.IMU.gyro_bias[i] = cameraParameters->gyro_bias[i];
      xt.robot_state.IMU.acc_bias[i] = cameraParameters->acc_bias[i];
      xt.robot_state.pos[i] = 0.0;
    }

    //  position relative to the origin frame
    // 'SLAM:32' xt.robot_state.att = [0; 0; 0; 1];
    for (i = 0; i < 4; i++) {
      xt.robot_state.att[i] = iv1[i];
    }

    //  orientation relative to the origin frame
    // 'SLAM:33' xt.robot_state.vel = [0; 0; 0];
    //  velocity in the origin frame
    // 'SLAM:34' xt.fixed_feature = int32(0);
    xt.fixed_feature = 0;

    // 'SLAM:35' xt.origin.anchor_idx = int32(0);
    xt.origin.anchor_idx = 0;

    //  idx of the anchor that is at the origin
    // 'SLAM:36' xt.origin.pos = [0; 0; 0];
    for (i = 0; i < 3; i++) {
      xt.robot_state.vel[i] = 0.0;
      xt.origin.pos[i] = 0.0;
    }

    //  position of the origin in the world frame
    // 'SLAM:37' xt.origin.att = [0; 0; 0; 1];
    for (i = 0; i < 4; i++) {
      xt.origin.att[i] = iv1[i];
    }

    //  orientation of the origin in the world frame
    // 'SLAM:39' P = zeros(numStates + numAnchors*(6+numPointsPerAnchor));
    memset(&P[0], 0, 8281U * sizeof(double));

    //  initial error state covariance
    // 'SLAM:41' anchor_state.pos = [0; 0; 0];
    // 'SLAM:42' anchor_state.att = [0; 0; 0; 1];
    // 'SLAM:43' anchor_state.P_idx = int32(zeros(1, 6));
    // 'SLAM:45' feature_state.inverse_depth = 0;
    // 'SLAM:46' feature_state.m = zeros(3,1);
    // 'SLAM:47' feature_state.scaled_map_point = zeros(3,1);
    // 'SLAM:48' feature_state.status = int32(0);
    // 'SLAM:49' feature_state.status_idx = int32(0);
    // 'SLAM:50' feature_state.P_idx = int32(0);
    // 'SLAM:52' anchor_state.feature_states = repmat(feature_state, numPointsPerAnchor,1); 
    // 'SLAM:54' xt.anchor_states = repmat(anchor_state, numAnchors, 1);
    // 'SLAM:56' for anchorIdx = 1:numAnchors
    memcpy(&xt.anchor_states[0], &rv0[0], 5U * sizeof(f_struct_T));
    for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
      // 'SLAM:57' xt.anchor_states(anchorIdx).P_idx = numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6); 
      i46 = anchorIdx * 14;
      for (i47 = 0; i47 < 6; i47++) {
        xt.anchor_states[anchorIdx].P_idx[i47] = (i47 + i46) + 22;
      }
    }

    // 'SLAM:60' if vision
    if (vision) {
      // 'SLAM:61' h_u = zeros(numTrackFeatures*4, 1);
      memset(&h_u[0], 0, 160U * sizeof(double));

      // 'SLAM:62' map = zeros(numTrackFeatures*3, 1);
      memset(&map[0], 0, 120U * sizeof(double));

      // 'SLAM:63' delayedStatus = zeros(numTrackFeatures, 1);
      memset(&delayedStatus[0], 0, 40U * sizeof(double));
    } else {
      // 'SLAM:64' else
      // 'SLAM:65' z_b = measurements.acc_duo - xt.robot_state.IMU.acc_bias;
      for (i = 0; i < 3; i++) {
        z_b[i] = measurements->acc_duo[i] - xt.robot_state.IMU.acc_bias[i];
      }

      // 'SLAM:66' z_n_b = z_b/norm(z_b);
      B = norm(z_b);
      for (i46 = 0; i46 < 3; i46++) {
        z_b[i46] /= B;
      }

      // 'SLAM:67' m_n_b = [0;0;1];
      // 'SLAM:68' y_n_b = cross(z_n_b,m_n_b);
      cross(z_b, dv3, y_n_b);

      // 'SLAM:69' y_n_b = y_n_b./norm(y_n_b);
      for (i = 0; i < 3; i++) {
        b_y_n_b[i] = y_n_b[i];
      }

      rdivide(b_y_n_b, norm(y_n_b), y_n_b);

      // 'SLAM:70' x_n_b = (cross(y_n_b,z_n_b));
      cross(y_n_b, z_b, x_n_b);

      // 'SLAM:71' x_n_b = x_n_b./norm(x_n_b);
      for (i = 0; i < 3; i++) {
        b_y_n_b[i] = x_n_b[i];
      }

      rdivide(b_y_n_b, norm(x_n_b), x_n_b);

      // 'SLAM:73' R_iw_init = [x_n_b,y_n_b,z_n_b];
      // 'SLAM:74' R_cw_init = RotFromQuatJ(xt.robot_state.IMU.att) * R_iw_init; 
      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
      // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
      // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
      dv4[0] = ((xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0] -
                 xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) -
                xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      dv4[3] = 2.0 * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[1] +
                      xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[3]);
      dv4[6] = 2.0 * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[2] -
                      xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[3]);
      dv4[1] = 2.0 * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[1] -
                      xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[3]);
      dv4[4] = ((-(xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0]) +
                 xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) -
                xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      dv4[7] = 2.0 * (xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[2] +
                      xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[3]);
      dv4[2] = 2.0 * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[2] +
                      xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[3]);
      dv4[5] = 2.0 * (xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[2] -
                      xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[3]);
      dv4[8] = ((-(xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0]) -
                 xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) +
                xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      for (i46 = 0; i46 < 3; i46++) {
        b_x_n_b[i46] = x_n_b[i46];
        b_x_n_b[3 + i46] = y_n_b[i46];
        b_x_n_b[6 + i46] = z_b[i46];
      }

      // 'SLAM:75' xt.origin.att = QuatFromRotJ(R_cw_init);
      for (i46 = 0; i46 < 3; i46++) {
        for (i47 = 0; i47 < 3; i47++) {
          R_cw_init[i46 + 3 * i47] = 0.0;
          for (i = 0; i < 3; i++) {
            R_cw_init[i46 + 3 * i47] += dv4[i46 + 3 * i] * b_x_n_b[i + 3 * i47];
          }

          P[i47 + 91 * i46] = 0.0;
          P[(i47 + 91 * (3 + i46)) + 3] = 0.0;
          P[(i47 + 91 * (6 + i46)) + 6] = y[i47 + 3 * i46];
        }
      }

      QuatFromRotJ(R_cw_init, xt.origin.att);

      //  orientation of the origin in the world frame
      // 'SLAM:77' P(  1:3,   1:3) = zeros(3);
      //  position
      // 'SLAM:78' P(  4:6,   4:6) = zeros(3);
      //  orientation of camera in origin frame
      // 'SLAM:79' P(  7:9,   7:9) = 1*eye(3);
      //  velocity
      // 'SLAM:80' P(10:12, 10:12) = diag(noiseParameters.gyro_bias_initial_unc); 
      diag(noiseParameters->gyro_bias_initial_unc, dv4);

      //  gyro bias
      // 'SLAM:81' P(13:15, 13:15) = diag(noiseParameters.acc_bias_initial_unc); 
      diag(noiseParameters->acc_bias_initial_unc, b_x_n_b);

      //  acc bias
      // 'SLAM:82' P(16:18, 16:18) = 0.1*R_cw_init * diag([1 1 0]) * R_cw_init'; 
      for (i46 = 0; i46 < 3; i46++) {
        for (i47 = 0; i47 < 3; i47++) {
          P[(i47 + 91 * (9 + i46)) + 9] = dv4[i47 + 3 * i46];
          P[(i47 + 91 * (12 + i46)) + 12] = b_x_n_b[i47 + 3 * i46];
          dv5[i46 + 3 * i47] = 0.0;
          for (i = 0; i < 3; i++) {
            dv5[i46 + 3 * i47] += 0.1 * R_cw_init[i46 + 3 * i] * (double)b[i + 3
              * i47];
          }
        }
      }

      for (i46 = 0; i46 < 3; i46++) {
        for (i47 = 0; i47 < 3; i47++) {
          P[(i46 + 91 * (15 + i47)) + 15] = 0.0;
          for (i = 0; i < 3; i++) {
            P[(i46 + 91 * (15 + i47)) + 15] += dv5[i46 + 3 * i] * R_cw_init[i47
              + 3 * i];
          }
        }
      }

      //  origin orientation
      // 'SLAM:83' P(19:21, 19:21) = 0*0.01*eye(3);
      for (i46 = 0; i46 < 3; i46++) {
        for (i47 = 0; i47 < 3; i47++) {
          P[(i47 + 91 * (18 + i46)) + 18] = 0.0;
        }
      }

      //  R_ci
      // 'SLAM:85' h_u = zeros(numTrackFeatures*4,1);
      memset(&h_u[0], 0, 160U * sizeof(double));

      // 'SLAM:86' map = getMap(xt);
      getMap(xt.origin.pos, xt.origin.att, xt.anchor_states, map);

      // 'SLAM:87' delayedStatus = zeros(size(updateVect));
      memset(&delayedStatus[0], 0, 40U * sizeof(double));

      // 'SLAM:89' printParams(noiseParameters, VIOParameters)
      printParams(noiseParameters->process_noise.qv,
                  noiseParameters->process_noise.qw,
                  noiseParameters->process_noise.qao,
                  noiseParameters->process_noise.qwo,
                  noiseParameters->process_noise.qR_ci,
                  noiseParameters->gyro_bias_initial_unc,
                  noiseParameters->acc_bias_initial_unc,
                  noiseParameters->image_noise,
                  noiseParameters->inv_depth_initial_unc,
                  b_VIOParameters->num_points_per_anchor,
                  b_VIOParameters->num_anchors,
                  b_VIOParameters->max_ekf_iterations,
                  b_VIOParameters->fixed_feature,
                  b_VIOParameters->delayed_initialization, b_VIOParameters->mono,
                  b_VIOParameters->RANSAC);

      // 'SLAM:90' initialized = 1;
      initialized_not_empty = true;
    }
  } else {
    // 'SLAM:93' else
    // 'SLAM:95' if ~vision
    if (!vision) {
      //      [xt,P] =  SLAM_pred(P, xt, dt, noiseParameters.process_noise, measurements, numStates); 
      // 'SLAM:97' [xt,P] =  SLAM_pred_euler(P, xt, dt, noiseParameters.process_noise, measurements); 
      SLAM_pred_euler(P, &xt, dt, noiseParameters->process_noise.qv,
                      noiseParameters->process_noise.qw,
                      noiseParameters->process_noise.qao,
                      noiseParameters->process_noise.qwo,
                      noiseParameters->process_noise.qR_ci,
                      measurements->acc_duo, measurements->gyr_duo);
    } else {
      // 'SLAM:98' else
      // 'SLAM:99' [h_u, xt, P, updateVect, map, delayedStatus] = SLAM_upd(P, xt, cameraParameters, updateVect, z_all_l, z_all_r, noiseParameters, VIOParameters); 
      memcpy(&b_z_all_l[0], &z_all_l[0], 80U * sizeof(double));
      memcpy(&b_z_all_r[0], &z_all_r[0], 80U * sizeof(double));
      SLAM_upd(P, &xt, cameraParameters->CameraParameters1.ATAN,
               cameraParameters->CameraParameters1.FocalLength,
               cameraParameters->CameraParameters1.PrincipalPoint,
               cameraParameters->CameraParameters1.RadialDistortion,
               cameraParameters->CameraParameters1.DistortionModel,
               cameraParameters->CameraParameters2.ATAN,
               cameraParameters->CameraParameters2.FocalLength,
               cameraParameters->CameraParameters2.PrincipalPoint,
               cameraParameters->CameraParameters2.RadialDistortion,
               cameraParameters->CameraParameters2.DistortionModel,
               cameraParameters->r_lr, cameraParameters->R_lr,
               cameraParameters->R_rl, updateVect, b_z_all_l, b_z_all_r,
               noiseParameters->image_noise,
               noiseParameters->inv_depth_initial_unc, *b_VIOParameters, h_u,
               map, delayedStatus);
    }
  }

  // 'SLAM:102' h_u_out = h_u;
  memcpy(&h_u_out[0], &h_u[0], 160U * sizeof(double));

  // 'SLAM:103' map_out = map;
  memcpy(&map_out[0], &map[0], 120U * sizeof(double));

  // 'SLAM:104' xt_out = getWorldState(xt);
  getWorldState(xt.robot_state.IMU.pos, xt.robot_state.IMU.att,
                xt.robot_state.IMU.gyro_bias, xt.robot_state.IMU.acc_bias,
                xt.robot_state.pos, xt.robot_state.att, xt.robot_state.vel,
                xt.origin.pos, xt.origin.att, b_y_n_b, t0_att, t0_vel,
                t0_IMU_gyro_bias, t0_IMU_acc_bias, t0_IMU_pos, t0_IMU_att);
  for (i = 0; i < 3; i++) {
    xt_out->pos[i] = b_y_n_b[i];
  }

  for (i = 0; i < 4; i++) {
    xt_out->att[i] = t0_att[i];
  }

  for (i = 0; i < 3; i++) {
    xt_out->vel[i] = t0_vel[i];
    xt_out->IMU.gyro_bias[i] = t0_IMU_gyro_bias[i];
    xt_out->IMU.acc_bias[i] = t0_IMU_acc_bias[i];
    xt_out->IMU.pos[i] = t0_IMU_pos[i];
  }

  for (i = 0; i < 4; i++) {
    xt_out->IMU.att[i] = t0_IMU_att[i];
  }

  // 'SLAM:105' anchor_poses_out = getAnchorPoses(xt);
  getAnchorPoses(xt.origin.pos, xt.origin.att, xt.anchor_states, rv1);
  cast(rv1, anchor_poses_out);

  // 'SLAM:106' delayedStatus_out = delayedStatus;
  memcpy(&delayedStatus_out[0], &delayedStatus[0], 40U * sizeof(double));

  //  output
  //  NOTE: Comment this out for MEXing
  // 'SLAM:110' coder.cstructname(xt_out, 'RobotState', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  // 'SLAM:111' coder.cstructname(xt_out.IMU, 'IMUState', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  // 'SLAM:112' coder.cstructname(anchor_poses_out(1), 'AnchorPose', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  // 'SLAM:114' assert ( all ( size (h_u_out) == [numTrackFeatures*4 1] ) )
  // 'SLAM:115' assert ( all ( size (map_out) == [numTrackFeatures*3 1] ) )
  // 'SLAM:116' assert ( all ( size (anchor_poses_out) == [numAnchors 1] ) )
  // 'SLAM:117' assert ( all ( size (updateVect) == [numTrackFeatures 1] ) )
  // 'SLAM:118' assert ( all ( size (delayedStatus_out) == [numTrackFeatures 1] ) ) 
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_initialize()
{
  rt_InitInfAndNaN(8U);
  debug_level = b_debug_level;
  initialized_not_empty = false;
}

//
// Arguments    : void
// Return Type  : void
//
void SLAM_terminate()
{
  // (no terminate code required)
}

//
// File trailer for SLAM.cpp
//
// [EOF]
//
