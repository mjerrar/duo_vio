//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 25-Oct-2015 15:34:53
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
  e_struct_T feature_states[5];
} f_struct_T;

typedef struct {
  c_struct_T robot_state;
  int fixed_feature;
  d_struct_T origin;
  f_struct_T anchor_states[8];
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
static double P[11881];
static double map[120];
static double delayedStatus[40];
static double debug_level;

// Function Declarations
static void OnePointRANSAC_EKF(g_struct_T *b_xt, double b_P[11881], const double
  z_u_l[80], const double z_u_r[80], const double
  c_stereoParams_CameraParameters[2], const double
  d_stereoParams_CameraParameters[2], const double
  e_stereoParams_CameraParameters[2], const double
  f_stereoParams_CameraParameters[2], const double stereoParams_r_lr[3], const
  double stereoParams_R_rl[9], double noiseParameters_image_noise, int
  c_VIOParameters_max_ekf_iterati, boolean_T c_VIOParameters_delayed_initial,
  boolean_T VIOParameters_full_stereo, boolean_T VIOParameters_RANSAC, int
  updateVect[40]);
static void QuatFromRotJ(const double R[9], double Q[4]);
static void SLAM_pred_euler(double P_apo[11881], g_struct_T *x, double dt,
  double processNoise_qv, double processNoise_qw, double processNoise_qao,
  double processNoise_qwo, double processNoise_qR_ci, const double
  measurements_acc_duo[3], const double measurements_gyr_duo[3]);
static void SLAM_upd(double P_apr[11881], g_struct_T *b_xt, int
                     c_cameraParams_CameraParameters, const double
                     d_cameraParams_CameraParameters[2], const double
                     e_cameraParams_CameraParameters[2], const double
                     f_cameraParams_CameraParameters[3], int
                     g_cameraParams_CameraParameters, int
                     h_cameraParams_CameraParameters, const double
                     i_cameraParams_CameraParameters[2], const double
                     j_cameraParams_CameraParameters[2], const double
                     k_cameraParams_CameraParameters[3], int
                     l_cameraParams_CameraParameters, const double
                     cameraParams_r_lr[3], const double cameraParams_R_lr[9],
                     const double cameraParams_R_rl[9], int updateVect[40],
                     double z_all_l[80], double z_all_r[80], double
                     noiseParameters_image_noise, double
                     c_noiseParameters_inv_depth_ini, const VIOParameters
                     b_VIOParameters, double b_map[120], double b_delayedStatus
                     [40]);
static boolean_T any(const boolean_T x[40]);
static boolean_T anyActiveAnchorFeatures(const e_struct_T
  anchor_state_feature_states[5]);
static boolean_T b_any(const boolean_T x[3]);
static void b_diag(const double v[15], double d[225]);
static void b_eye(double I[441]);
static double b_fprintf();
static void b_getH_R_res(const double xt_robot_state_pos[3], const double
  xt_robot_state_att[4], const f_struct_T xt_anchor_states[8], const double
  z_all_l[80], const double z_all_r[80], const boolean_T b_status[40], const
  double c_stereoParams_CameraParameters[2], const double
  d_stereoParams_CameraParameters[2], const double
  e_stereoParams_CameraParameters[2], const double
  f_stereoParams_CameraParameters[2], const double stereoParams_r_lr[3], const
  double stereoParams_R_rl[9], boolean_T VIOParameters_full_stereo,
  emxArray_real_T *r, emxArray_real_T *H);
static void b_merge(emxArray_int32_T *idx, emxArray_real_T *x, int offset, int
                    np, int nq, emxArray_int32_T *iwork, emxArray_real_T *xwork);
static void b_mrdivide(double A_data[], int A_size[2], const double B_data[],
  const int B_size[2]);
static double b_norm(const double x[4]);
static void b_ros_error();
static void b_ros_info(int varargin_1, int varargin_2, int varargin_3);
static void b_ros_warn(int varargin_1, int varargin_2, int varargin_3);
static void b_sort(double x[5], int idx[5]);
static void b_xgeqp3(double A[30], double tau[5], int jpvt[5]);
static double b_xnrm2(int n, const emxArray_real_T *x, int ix0);
static boolean_T c_any(const double x[8]);
static void c_eye(double I[11881]);
static double c_fprintf(double varargin_1);
static double c_norm(const double x[2]);
static void c_ros_info(int varargin_1, int varargin_2, int varargin_3);
static void c_ros_warn();
static void c_sort(emxArray_real_T *x, emxArray_int32_T *idx);
static double c_xnrm2(int n, const emxArray_real_T *x, int ix0);
static void cast(const struct_T x[8], AnchorPose y[8]);
static void cross(const double a[3], const double b[3], double c[3]);
static void d_eye(double I[11881]);
static double d_fprintf(double varargin_1);
static void d_ros_info(int varargin_1, int varargin_2);
static void d_ros_warn(int varargin_1, double varargin_2);
static void d_sort(emxArray_real_T *x, int dim, emxArray_int32_T *idx);
static double d_xnrm2(int n, const double x[30], int ix0);
static double det(const double x[36]);
static void diag(const double v[3], double d[9]);
static int div_s32_floor(int numerator, int denominator);
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
static void f_ros_info(int varargin_1);
static void f_ros_warn(int varargin_1, double varargin_2);
static void fileManager(FILE * *f, boolean_T *a);
static double g_fprintf(double varargin_1);
static void g_ros_info(int varargin_1, double varargin_2, double varargin_3);
static void g_ros_warn(double varargin_1);
static void getAnchorPoses(const double xt_origin_pos[3], const double
  xt_origin_att[4], const f_struct_T xt_anchor_states[8], struct_T anchor_poses
  [8]);
static void getH_R_res(const double xt_robot_state_pos[3], const double
  xt_robot_state_att[4], const f_struct_T xt_anchor_states[8], const double
  z_all_l[80], const double z_all_r[80], const boolean_T b_status[40], const
  double c_stereoParams_CameraParameters[2], const double
  d_stereoParams_CameraParameters[2], const double
  e_stereoParams_CameraParameters[2], const double
  f_stereoParams_CameraParameters[2], const double stereoParams_r_lr[3], const
  double stereoParams_R_rl[9], boolean_T VIOParameters_full_stereo,
  emxArray_real_T *r, emxArray_real_T *H, emxArray_int32_T *ind);
static void getMap(const double xt_origin_pos[3], const double xt_origin_att[4],
                   const f_struct_T xt_anchor_states[8], double b_map[120]);
static double getNumValidFeatures(const e_struct_T anchor_state_feature_states[5]);
static void getScaledMap(g_struct_T *b_xt);
static double getTotalNumActiveFeatures(const f_struct_T xt_anchor_states[8]);
static double getTotalNumDelayedFeatures(const f_struct_T xt_anchor_states[8]);
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
static void i_ros_info(int varargin_1);
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
static double m_fprintf(int varargin_1);
static double median(const double x_data[], const int x_size[1]);
static void merge(int idx[5], double x[5], int offset, int np, int nq, int
                  iwork[5], double xwork[5]);
static void merge_block(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
  int n, int preSortLevel, emxArray_int32_T *iwork, emxArray_real_T *xwork);
static void mrdivide(emxArray_real_T *A, const emxArray_real_T *B);
static void multiplyIdx(const double idx_data[], const int idx_size[1], double
  idx_mult_data[], int idx_mult_size[1]);
static double n_fprintf(int varargin_1);
static double norm(const double x[3]);
static double o_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static double p_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
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
  boolean_T VIOParameters_mono, boolean_T VIOParameters_full_stereo, boolean_T
  VIOParameters_RANSAC);
static double q_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static void quatPlusThetaJ(const double dtheta[3], double dq[4]);
static void quatmultJ(const double q[4], const double p[4], double qp[4]);
static double r_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static int rankFromQR(const emxArray_real_T *A);
static void rdivide(const double x[3], double y, double z[3]);
static void ros_error();
static void ros_info(int varargin_1, int varargin_2, int varargin_3);
static void ros_warn(int varargin_1, double varargin_2);
static double rt_powd_snf(double u0, double u1);
static double rt_roundd_snf(double u);
static double s_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static void sort(double x[5], int idx[5]);
static void sortIdx(emxArray_real_T *x, emxArray_int32_T *idx);
static void undistortPoint(const double pt_d_data[], const int pt_d_size[1], int
  cameraparams_ATAN, const double cameraparams_FocalLength[2], const double
  cameraparams_PrincipalPoint[2], const double cameraparams_RadialDistortion[3],
  int cameraparams_DistortionModel, double pt_u_data[], int pt_u_size[1]);
static void undistortPointAtan(const double pt_d_data[], const int pt_d_size[1],
  const double cameraparams_FocalLength[2], const double
  cameraparams_PrincipalPoint[2], const double cameraparams_RadialDistortion[3],
  double pt_u_data[], int pt_u_size[1]);
static void xgeqp3(emxArray_real_T *A, emxArray_real_T *tau, emxArray_int32_T
                   *jpvt);
static void xgetrf(int b_m, int n, emxArray_real_T *A, int lda, emxArray_int32_T
                   *ipiv, int *info);
static double xnrm2(const double x[3]);
static void xscal(int n, double a, emxArray_real_T *x, int ix0);

// Function Definitions

//
// OnePointRANSAC_EKF Perform a 1-point RANSAC outlier rejection and update
// the state
// Arguments    : g_struct_T *b_xt
//                double b_P[11881]
//                const double z_u_l[80]
//                const double z_u_r[80]
//                const double c_stereoParams_CameraParameters[2]
//                const double d_stereoParams_CameraParameters[2]
//                const double e_stereoParams_CameraParameters[2]
//                const double f_stereoParams_CameraParameters[2]
//                const double stereoParams_r_lr[3]
//                const double stereoParams_R_rl[9]
//                double noiseParameters_image_noise
//                int c_VIOParameters_max_ekf_iterati
//                boolean_T c_VIOParameters_delayed_initial
//                boolean_T VIOParameters_full_stereo
//                boolean_T VIOParameters_RANSAC
//                int updateVect[40]
// Return Type  : void
//
static void OnePointRANSAC_EKF(g_struct_T *b_xt, double b_P[11881], const double
  z_u_l[80], const double z_u_r[80], const double
  c_stereoParams_CameraParameters[2], const double
  d_stereoParams_CameraParameters[2], const double
  e_stereoParams_CameraParameters[2], const double
  f_stereoParams_CameraParameters[2], const double stereoParams_r_lr[3], const
  double stereoParams_R_rl[9], double noiseParameters_image_noise, int
  c_VIOParameters_max_ekf_iterati, boolean_T c_VIOParameters_delayed_initial,
  boolean_T VIOParameters_full_stereo, boolean_T VIOParameters_RANSAC, int
  updateVect[40])
{
  int residualDim;
  int mahalanobis_thresh;
  double LI_residual_thresh;
  int apnd;
  boolean_T activeFeatures[40];
  boolean_T delayedFeatures[40];
  int anchorIdx;
  int featureIdx;
  boolean_T LI_inlier_status[40];
  emxArray_real_T *S;
  emxArray_real_T *K;
  emxArray_int32_T *ind;
  emxArray_real_T *r;
  emxArray_real_T *H;
  int idx;
  signed char ii_data[40];
  int b_m;
  boolean_T exitg2;
  boolean_T guard2 = false;
  int ndbl;
  int i54;
  signed char hyp_ind_data[40];
  double num_hyp;
  int hyp_it;
  emxArray_real_T *y;
  boolean_T HI_inlierCandidates[40];
  boolean_T HI_inlierStatus[40];
  long i55;
  int i56;
  double H_c_data[4];
  double P_c[6];
  double P_d[36];
  int i57;
  double P_f[6];
  double H_a_data[24];
  int ic;
  int br;
  int ar;
  int ib;
  int ia;
  double H_b_data[24];
  long i58;
  double C_data[24];
  double b_C_data[24];
  long i59;
  double a_data[24];
  double b_data[24];
  double S_feature_data[16];
  double c_C_data[16];
  double y_data[4];
  double d_C_data[4];
  double c_P;
  double c_xt[4];
  double b_S_feature_data[6];
  unsigned int unnamed_idx_1;
  double x_apo[109];
  double b_x_apo;
  double b_y_data[4];
  double R_cw[9];
  double r_wc[3];
  double anchorPos[3];
  double c_x_apo[3];
  double dv20[4];
  double gryro_bias_cov[9];
  double rho;
  double fp[3];
  double h_u_r[2];
  double h_u_l[2];
  double innov;
  double b_h_u_l[2];
  int n;
  int k;
  emxArray_real_T *b_y;
  unsigned int H_idx_0;
  double size_S;
  emxArray_real_T *b_S;
  emxArray_real_T *c_y;
  double dv21[4];
  static double d_y[11881];
  static double dv22[11881];
  static double dv23[11881];
  static double dv24[11881];
  boolean_T b_HI_inlierCandidates;
  boolean_T exitg1;
  boolean_T guard1 = false;
  int ii_size_idx_0;
  emxArray_real_T *b_K;
  emxArray_real_T *b_H;
  int it;
  emxArray_real_T *e_y;
  emxArray_real_T *f_y;
  emxArray_real_T *c_H;
  emxArray_real_T *c_S;
  emxArray_real_T *d_H;
  int b_ndbl;
  int c_ndbl;
  double e_C_data[24];
  int tmp_data[4];
  double f_C_data[24];
  double g_C_data[24];
  double h_C_data[24];
  double i_C_data[16];
  double j_C_data[16];
  double k_C_data[4];
  double l_C_data[4];
  int S_feature_size[2];
  int y_size[2];
  int b_tmp_data[4];
  double x_apo_data[109];
  double b_x_apo_data;
  double dv25[4];
  emxArray_real_T *C;
  static double dv26[11881];
  emxArray_real_T *g_y;
  emxArray_real_T *d_S;
  emxArray_real_T *h_y;
  double dv27[11881];
  double rejected_ratio;
  char cv54[66];
  static const char cv55[66] = { '1', '-', 'p', 'o', 'i', 'n', 't', ' ', 'R',
    'A', 'N', 'S', 'A', 'C', ' ', 'r', 'e', 'j', 'e', 'c', 't', 'e', 'd', ' ',
    '%', 'd', '%', '%', ' ', 'o', 'f', ' ', 'a', 'l', 'l', ' ', 'f', 'e', 'a',
    't', 'u', 'r', 'e', 's', '!', ' ', 'R', 'e', 's', 'e', 't', 't', 'i', 'n',
    'g', ' ', 'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', '.', '\x00' };

  double d_xt[9];
  double e_xt[9];
  double acc_bias_cov[9];
  double R_ci_cov[9];
  static const signed char i_y[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 0 };

  // 'OnePointRANSAC_EKF:5' numStatesPerAnchor = 6 + numPointsPerAnchor;
  // 'OnePointRANSAC_EKF:8' LI_min_support_thresh = 3;
  // 'OnePointRANSAC_EKF:10' if VIOParameters.full_stereo
  if (VIOParameters_full_stereo) {
    // 'OnePointRANSAC_EKF:11' residualDim = 4;
    residualDim = 4;

    // 'OnePointRANSAC_EKF:12' mahalanobis_thresh = 9;
    mahalanobis_thresh = 9;

    //  HI mahalanobis gate
    // 'OnePointRANSAC_EKF:13' LI_residual_thresh = 4.24;
    LI_residual_thresh = 4.24;
  } else {
    // 'OnePointRANSAC_EKF:14' else
    // 'OnePointRANSAC_EKF:15' residualDim = 2;
    residualDim = 2;

    // 'OnePointRANSAC_EKF:16' mahalanobis_thresh = 6;
    mahalanobis_thresh = 6;

    //  HI mahalanobis gate
    // 'OnePointRANSAC_EKF:17' LI_residual_thresh = 3;
    LI_residual_thresh = 3.0;
  }

  // 'OnePointRANSAC_EKF:20' activeFeatures = false(size(updateVect));
  // 'OnePointRANSAC_EKF:21' delayedFeatures = activeFeatures;
  for (apnd = 0; apnd < 40; apnd++) {
    activeFeatures[apnd] = false;
    delayedFeatures[apnd] = false;
  }

  // 'OnePointRANSAC_EKF:22' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
    // 'OnePointRANSAC_EKF:23' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 5; featureIdx++) {
      // 'OnePointRANSAC_EKF:24' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 1 
      if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status == 1)
      {
        // 'OnePointRANSAC_EKF:25' activeFeatures(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = 1; 
        activeFeatures[b_xt->anchor_states[anchorIdx].feature_states[featureIdx]
          .status_idx - 1] = true;
      } else {
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
            2) {
          // 'OnePointRANSAC_EKF:26' elseif xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 
          // 'OnePointRANSAC_EKF:27' delayedFeatures(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = 1; 
          delayedFeatures[b_xt->anchor_states[anchorIdx]
            .feature_states[featureIdx].status_idx - 1] = true;
        }
      }
    }
  }

  // 'OnePointRANSAC_EKF:32' activeFeatures = activeFeatures & (updateVect==1);
  // 'OnePointRANSAC_EKF:33' delayedFeatures = delayedFeatures & (updateVect==1); 
  // 'OnePointRANSAC_EKF:35' LI_inlier_status = false(size(updateVect));
  for (apnd = 0; apnd < 40; apnd++) {
    LI_inlier_status[apnd] = false;
    activeFeatures[apnd] = (activeFeatures[apnd] && (updateVect[apnd] == 1));
    delayedFeatures[apnd] = (delayedFeatures[apnd] && (updateVect[apnd] == 1));
  }

  // % B 1-point hypotheses generation and evaluation
  // 'OnePointRANSAC_EKF:38' if VIOParameters.RANSAC
  emxInit_real_T1(&S, 2);
  emxInit_real_T1(&K, 2);
  emxInit_int32_T1(&ind, 2);
  emxInit_real_T(&r, 1);
  emxInit_real_T1(&H, 2);
  if (VIOParameters_RANSAC) {
    //  build the map according to the current estimate
    // 'OnePointRANSAC_EKF:40' xt = getScaledMap(xt);
    getScaledMap(b_xt);

    //  randomly permute the active feature indices for 1-point RANSAC
    // 'OnePointRANSAC_EKF:42' hyp_ind = find(activeFeatures);
    idx = 0;
    b_m = 1;
    exitg2 = false;
    while ((!exitg2) && (b_m < 41)) {
      guard2 = false;
      if (activeFeatures[b_m - 1]) {
        idx++;
        ii_data[idx - 1] = (signed char)b_m;
        if (idx >= 40) {
          exitg2 = true;
        } else {
          guard2 = true;
        }
      } else {
        guard2 = true;
      }

      if (guard2) {
        b_m++;
      }
    }

    if (1 > idx) {
      ndbl = 0;
    } else {
      ndbl = idx;
    }

    for (i54 = 0; i54 < ndbl; i54++) {
      hyp_ind_data[i54] = ii_data[i54];
    }

    //  hyp_ind = hyp_ind(randperm(length(hyp_ind)));
    // 'OnePointRANSAC_EKF:45' hyp_status = LI_inlier_status;
    // 'OnePointRANSAC_EKF:46' LI_inlier_status_i = LI_inlier_status;
    // 'OnePointRANSAC_EKF:48' num_hyp = length(hyp_ind);
    if (1 > idx) {
      num_hyp = 0.0;
    } else {
      num_hyp = idx;
    }

    // 'OnePointRANSAC_EKF:49' hyp_it = 1;
    hyp_it = 1;

    // 'OnePointRANSAC_EKF:50' while hyp_it < num_hyp && hyp_it < length(hyp_ind) 
    emxInit_real_T1(&y, 2);
    while ((hyp_it < num_hyp) && (hyp_it < ndbl)) {
      // 'OnePointRANSAC_EKF:51' hyp_idx = hyp_ind(hyp_it);
      // 'OnePointRANSAC_EKF:53' LI_inlier_status_i(:) = false;
      // 'OnePointRANSAC_EKF:55' hyp_status(:) = false;
      for (apnd = 0; apnd < 40; apnd++) {
        HI_inlierCandidates[apnd] = false;
        HI_inlierStatus[apnd] = false;
      }

      // 'OnePointRANSAC_EKF:56' hyp_status(hyp_idx) = true;
      HI_inlierStatus[hyp_ind_data[hyp_it - 1] - 1] = true;

      //  used to signal which feature to compute the derivatives for
      // 'OnePointRANSAC_EKF:57' [r, H, ind] = getH_R_res(xt, z_u_l, z_u_r, hyp_status, stereoParams, VIOParameters); 
      getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att,
                 b_xt->anchor_states, z_u_l, z_u_r, HI_inlierStatus,
                 c_stereoParams_CameraParameters,
                 d_stereoParams_CameraParameters,
                 e_stereoParams_CameraParameters,
                 f_stereoParams_CameraParameters, stereoParams_r_lr,
                 stereoParams_R_rl, VIOParameters_full_stereo, r, H, ind);

      // 'OnePointRANSAC_EKF:59' P_a = P(1:6, 1:6);
      // 'OnePointRANSAC_EKF:61' anchorIdx = ind(1, 1);
      anchorIdx = ind->data[0];

      // 'OnePointRANSAC_EKF:62' featureIdx = ind(1, 2);
      // 'OnePointRANSAC_EKF:63' H_a = H(1:residualDim, 1:6);
      // 'OnePointRANSAC_EKF:64' H_b = H(1:residualDim, numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6)); 
      // 'OnePointRANSAC_EKF:65' H_c = H(1:residualDim, numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
      i55 = ind->data[0] - 1L;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = (int)i55 * 11L;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = 21L + (int)i55;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = (int)i55 + 6L;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = (long)(int)i55 + ind->data[ind->size[0]];
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i54 = (int)i55;
      for (i56 = 0; i56 < residualDim; i56++) {
        H_c_data[i56] = H->data[i56 + H->size[0] * (i54 - 1)];
      }

      // 'OnePointRANSAC_EKF:66' P_b = P(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6), 1:6); 
      // 'OnePointRANSAC_EKF:67' P_c = P(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx, 1:6); 
      i55 = ind->data[0] - 1L;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = (int)i55 * 11L;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = 21L + (int)i55;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = (int)i55 + 6L;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = (long)(int)i55 + ind->data[ind->size[0]];
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i54 = (int)i55;
      for (i56 = 0; i56 < 6; i56++) {
        P_c[i56] = b_P[(i54 + 109 * i56) - 1];
      }

      // 'OnePointRANSAC_EKF:68' P_d = P_b';
      i55 = ind->data[0] - 1L;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = (int)i55 * 11L;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = 21L + (int)i55;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i54 = (int)i55;
      for (i56 = 0; i56 < 6; i56++) {
        for (i57 = 0; i57 < 6; i57++) {
          i55 = (long)i54 + (1 + i56);
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          P_d[i57 + 6 * i56] = b_P[((int)i55 + 109 * i57) - 1];
        }
      }

      // 'OnePointRANSAC_EKF:69' P_e = P(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6), numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6)); 
      // 'OnePointRANSAC_EKF:70' P_f = P(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx, numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6)); 
      i55 = ind->data[0] - 1L;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = (int)i55 * 11L;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = 21L + (int)i55;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i54 = (int)i55;
      i55 = ind->data[0] - 1L;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = (int)i55 * 11L;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = 21L + (int)i55;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = (int)i55 + 6L;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = (long)(int)i55 + ind->data[ind->size[0]];
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i56 = (int)i55;

      // 'OnePointRANSAC_EKF:71' P_g = P_c';
      // 'OnePointRANSAC_EKF:72' P_h = P_f';
      // 'OnePointRANSAC_EKF:73' P_i = P(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx, numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
      // 'OnePointRANSAC_EKF:74' S = (H_a*P_a + H_b*P_b + H_c*P_c)*H_a' + ...
      // 'OnePointRANSAC_EKF:75'             (H_a*P_d + H_b*P_e + H_c*P_f)*H_b' + ... 
      // 'OnePointRANSAC_EKF:76'             (H_a*P_g + H_b*P_h + H_c*P_i)*H_c'; 
      for (i57 = 0; i57 < 6; i57++) {
        i55 = (long)i54 + (1 + i57);
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        P_f[i57] = b_P[(i56 + 109 * ((int)i55 - 1)) - 1];
        idx = residualDim;
        for (b_m = 0; b_m < idx; b_m++) {
          H_a_data[b_m + residualDim * i57] = 0.0;
        }
      }

      idx = residualDim * 5;
      for (apnd = 0; apnd <= idx; apnd += residualDim) {
        i54 = apnd + residualDim;
        for (ic = apnd; ic + 1 <= i54; ic++) {
          H_a_data[ic] = 0.0;
        }
      }

      br = 0;
      for (apnd = 0; apnd <= idx; apnd += residualDim) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 6; ib++) {
          if (b_P[ib % 6 + 109 * (ib / 6)] != 0.0) {
            ia = ar;
            i54 = apnd + residualDim;
            for (ic = apnd; ic + 1 <= i54; ic++) {
              ia++;
              H_a_data[ic] += b_P[ib % 6 + 109 * (ib / 6)] * H->data[(ia - 1) %
                residualDim + H->size[0] * ((ia - 1) / residualDim)];
            }
          }

          ar += residualDim;
        }

        br += 6;
      }

      for (i54 = 0; i54 < 6; i54++) {
        idx = residualDim;
        for (i56 = 0; i56 < idx; i56++) {
          H_b_data[i56 + residualDim * i54] = 0.0;
        }
      }

      idx = residualDim * 5;
      for (apnd = 0; apnd <= idx; apnd += residualDim) {
        i54 = apnd + residualDim;
        for (ic = apnd; ic + 1 <= i54; ic++) {
          H_b_data[ic] = 0.0;
        }
      }

      br = 0;
      for (apnd = 0; apnd <= idx; apnd += residualDim) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 6; ib++) {
          i55 = anchorIdx - 1L;
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i55 = (int)i55 * 11L;
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i55 = 21L + (int)i55;
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i55 = (long)(int)i55 + (1 + ib % 6);
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          if (b_P[((int)i55 + 109 * (ib / 6)) - 1] != 0.0) {
            ia = ar;
            i54 = apnd + residualDim;
            for (ic = apnd; ic + 1 <= i54; ic++) {
              ia++;
              i55 = ind->data[0] - 1L;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i55 = (int)i55 * 11L;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i55 = 21L + (int)i55;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i56 = (int)i55;
              i55 = (long)i56 + (1 + ib % 6);
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i58 = anchorIdx - 1L;
              if (i58 > 2147483647L) {
                i58 = 2147483647L;
              } else {
                if (i58 < -2147483648L) {
                  i58 = -2147483648L;
                }
              }

              i58 = (int)i58 * 11L;
              if (i58 > 2147483647L) {
                i58 = 2147483647L;
              } else {
                if (i58 < -2147483648L) {
                  i58 = -2147483648L;
                }
              }

              i58 = 21L + (int)i58;
              if (i58 > 2147483647L) {
                i58 = 2147483647L;
              } else {
                if (i58 < -2147483648L) {
                  i58 = -2147483648L;
                }
              }

              i58 = (long)(int)i58 + (1 + (ia - 1) / residualDim);
              if (i58 > 2147483647L) {
                i58 = 2147483647L;
              } else {
                if (i58 < -2147483648L) {
                  i58 = -2147483648L;
                }
              }

              H_b_data[ic] += b_P[((int)i55 + 109 * (ib / 6)) - 1] * H->data[(ia
                - 1) % residualDim + H->size[0] * ((int)i58 - 1)];
            }
          }

          ar += residualDim;
        }

        br += 6;
      }

      for (i54 = 0; i54 < 6; i54++) {
        idx = residualDim;
        for (i56 = 0; i56 < idx; i56++) {
          C_data[i56 + residualDim * i54] = 0.0;
        }
      }

      idx = residualDim * 5;
      for (apnd = 0; apnd <= idx; apnd += residualDim) {
        i54 = apnd + residualDim;
        for (ic = apnd; ic + 1 <= i54; ic++) {
          C_data[ic] = 0.0;
        }
      }

      br = 0;
      for (apnd = 0; apnd <= idx; apnd += residualDim) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 6; ib++) {
          if (P_d[ib] != 0.0) {
            ia = ar;
            i54 = apnd + residualDim;
            for (ic = apnd; ic + 1 <= i54; ic++) {
              ia++;
              C_data[ic] += P_d[ib] * H->data[(ia - 1) % residualDim + H->size[0]
                * ((ia - 1) / residualDim)];
            }
          }

          ar += residualDim;
        }

        br += 6;
      }

      for (i54 = 0; i54 < 6; i54++) {
        idx = residualDim;
        for (i56 = 0; i56 < idx; i56++) {
          b_C_data[i56 + residualDim * i54] = 0.0;
        }
      }

      idx = residualDim * 5;
      for (apnd = 0; apnd <= idx; apnd += residualDim) {
        i54 = apnd + residualDim;
        for (ic = apnd; ic + 1 <= i54; ic++) {
          b_C_data[ic] = 0.0;
        }
      }

      br = 0;
      for (apnd = 0; apnd <= idx; apnd += residualDim) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 6; ib++) {
          i55 = anchorIdx - 1L;
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i55 = (int)i55 * 11L;
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i55 = 21L + (int)i55;
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i55 = (long)(int)i55 + (1 + ib % 6);
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i58 = anchorIdx - 1L;
          if (i58 > 2147483647L) {
            i58 = 2147483647L;
          } else {
            if (i58 < -2147483648L) {
              i58 = -2147483648L;
            }
          }

          i58 = (int)i58 * 11L;
          if (i58 > 2147483647L) {
            i58 = 2147483647L;
          } else {
            if (i58 < -2147483648L) {
              i58 = -2147483648L;
            }
          }

          i58 = 21L + (int)i58;
          if (i58 > 2147483647L) {
            i58 = 2147483647L;
          } else {
            if (i58 < -2147483648L) {
              i58 = -2147483648L;
            }
          }

          i58 = (long)(int)i58 + (1 + ib / 6);
          if (i58 > 2147483647L) {
            i58 = 2147483647L;
          } else {
            if (i58 < -2147483648L) {
              i58 = -2147483648L;
            }
          }

          if (b_P[((int)i55 + 109 * ((int)i58 - 1)) - 1] != 0.0) {
            ia = ar;
            i54 = apnd + residualDim;
            for (ic = apnd; ic + 1 <= i54; ic++) {
              ia++;
              i55 = ind->data[0] - 1L;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i55 = (int)i55 * 11L;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i55 = 21L + (int)i55;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i56 = (int)i55;
              i55 = ind->data[0] - 1L;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i55 = (int)i55 * 11L;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i55 = 21L + (int)i55;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i57 = (int)i55;
              i55 = (long)i56 + (1 + ib % 6);
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i58 = (long)i57 + (1 + ib / 6);
              if (i58 > 2147483647L) {
                i58 = 2147483647L;
              } else {
                if (i58 < -2147483648L) {
                  i58 = -2147483648L;
                }
              }

              i59 = anchorIdx - 1L;
              if (i59 > 2147483647L) {
                i59 = 2147483647L;
              } else {
                if (i59 < -2147483648L) {
                  i59 = -2147483648L;
                }
              }

              i59 = (int)i59 * 11L;
              if (i59 > 2147483647L) {
                i59 = 2147483647L;
              } else {
                if (i59 < -2147483648L) {
                  i59 = -2147483648L;
                }
              }

              i59 = 21L + (int)i59;
              if (i59 > 2147483647L) {
                i59 = 2147483647L;
              } else {
                if (i59 < -2147483648L) {
                  i59 = -2147483648L;
                }
              }

              i59 = (long)(int)i59 + (1 + (ia - 1) / residualDim);
              if (i59 > 2147483647L) {
                i59 = 2147483647L;
              } else {
                if (i59 < -2147483648L) {
                  i59 = -2147483648L;
                }
              }

              b_C_data[ic] += b_P[((int)i55 + 109 * ((int)i58 - 1)) - 1] *
                H->data[(ia - 1) % residualDim + H->size[0] * ((int)i59 - 1)];
            }
          }

          ar += residualDim;
        }

        br += 6;
      }

      for (i54 = 0; i54 < residualDim; i54++) {
        for (i56 = 0; i56 < 6; i56++) {
          a_data[i54 + residualDim * i56] = (H_a_data[i54 + residualDim * i56] +
            H_b_data[i54 + residualDim * i56]) + H_c_data[i54] * P_c[i56];
        }
      }

      for (i54 = 0; i54 < residualDim; i54++) {
        for (i56 = 0; i56 < 6; i56++) {
          b_data[i56 + 6 * i54] = H->data[i54 + H->size[0] * i56];
        }
      }

      idx = residualDim;
      for (i54 = 0; i54 < idx; i54++) {
        for (i56 = 0; i56 < idx; i56++) {
          S_feature_data[i56 + residualDim * i54] = 0.0;
        }
      }

      idx = residualDim * (residualDim - 1);
      for (apnd = 0; apnd <= idx; apnd += residualDim) {
        i54 = apnd + residualDim;
        for (ic = apnd; ic + 1 <= i54; ic++) {
          S_feature_data[ic] = 0.0;
        }
      }

      br = 0;
      for (apnd = 0; apnd <= idx; apnd += residualDim) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 6; ib++) {
          if (b_data[ib] != 0.0) {
            ia = ar;
            i54 = apnd + residualDim;
            for (ic = apnd; ic + 1 <= i54; ic++) {
              ia++;
              S_feature_data[ic] += b_data[ib] * a_data[ia - 1];
            }
          }

          ar += residualDim;
        }

        br += 6;
      }

      for (i54 = 0; i54 < residualDim; i54++) {
        for (i56 = 0; i56 < 6; i56++) {
          a_data[i54 + residualDim * i56] = (C_data[i54 + residualDim * i56] +
            b_C_data[i54 + residualDim * i56]) + H_c_data[i54] * P_f[i56];
        }
      }

      i55 = ind->data[0] - 1L;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = (int)i55 * 11L;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = 21L + (int)i55;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i54 = (int)i55;
      for (i56 = 0; i56 < residualDim; i56++) {
        for (i57 = 0; i57 < 6; i57++) {
          i55 = (long)i54 + (1 + i57);
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          b_data[i57 + 6 * i56] = H->data[i56 + H->size[0] * ((int)i55 - 1)];
        }
      }

      idx = residualDim;
      for (i54 = 0; i54 < idx; i54++) {
        for (i56 = 0; i56 < idx; i56++) {
          c_C_data[i56 + residualDim * i54] = 0.0;
        }
      }

      idx = residualDim * (residualDim - 1);
      for (apnd = 0; apnd <= idx; apnd += residualDim) {
        i54 = apnd + residualDim;
        for (ic = apnd; ic + 1 <= i54; ic++) {
          c_C_data[ic] = 0.0;
        }
      }

      br = 0;
      for (apnd = 0; apnd <= idx; apnd += residualDim) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 6; ib++) {
          if (b_data[ib] != 0.0) {
            ia = ar;
            i54 = apnd + residualDim;
            for (ic = apnd; ic + 1 <= i54; ic++) {
              ia++;
              c_C_data[ic] += b_data[ib] * a_data[ia - 1];
            }
          }

          ar += residualDim;
        }

        br += 6;
      }

      b_m = residualDim;
      for (i54 = 0; i54 < b_m; i54++) {
        y_data[i54] = 0.0;
      }

      apnd = 0;
      while (apnd <= 0) {
        for (ic = 1; ic <= residualDim; ic++) {
          y_data[ic - 1] = 0.0;
        }

        apnd = residualDim;
      }

      br = 6;
      apnd = 0;
      while (apnd <= 0) {
        ar = 0;
        for (ib = br - 5; ib <= br; ib++) {
          i55 = ind->data[0] - 1L;
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i55 = (int)i55 * 11L;
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i55 = 21L + (int)i55;
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i55 = (int)i55 + 6L;
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i55 = (long)(int)i55 + ind->data[ind->size[0]];
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i54 = (int)i55;
          if (b_P[(i54 + 109 * (ib - 1)) - 1] != 0.0) {
            ia = ar;
            for (ic = 0; ic + 1 <= residualDim; ic++) {
              ia++;
              i55 = ind->data[0] - 1L;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i55 = (int)i55 * 11L;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i55 = 21L + (int)i55;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i55 = (int)i55 + 6L;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i55 = (long)(int)i55 + ind->data[ind->size[0]];
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i54 = (int)i55;
              y_data[ic] += b_P[(i54 + 109 * (ib - 1)) - 1] * H->data[(ia - 1) %
                residualDim + H->size[0] * ((ia - 1) / residualDim)];
            }
          }

          ar += residualDim;
        }

        br += 6;
        apnd = residualDim;
      }

      for (i54 = 0; i54 < b_m; i54++) {
        d_C_data[i54] = 0.0;
      }

      apnd = 0;
      while (apnd <= 0) {
        for (ic = 1; ic <= residualDim; ic++) {
          d_C_data[ic - 1] = 0.0;
        }

        apnd = residualDim;
      }

      br = 6;
      apnd = 0;
      while (apnd <= 0) {
        ar = 0;
        for (ib = br - 5; ib <= br; ib++) {
          i55 = ind->data[0] - 1L;
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i55 = (int)i55 * 11L;
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i55 = 21L + (int)i55;
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i54 = (int)i55;
          i55 = ind->data[0] - 1L;
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i55 = (int)i55 * 11L;
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i55 = 21L + (int)i55;
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i55 = (int)i55 + 6L;
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i55 = (long)(int)i55 + ind->data[ind->size[0]];
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          i56 = (int)i55;
          i55 = (long)i54 + ib;
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          if (b_P[(i56 + 109 * ((int)i55 - 1)) - 1] != 0.0) {
            ia = ar;
            for (ic = 0; ic + 1 <= residualDim; ic++) {
              ia++;
              i55 = ind->data[0] - 1L;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i55 = (int)i55 * 11L;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i55 = 21L + (int)i55;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i54 = (int)i55;
              i55 = ind->data[0] - 1L;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i55 = (int)i55 * 11L;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i55 = 21L + (int)i55;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i55 = (int)i55 + 6L;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i55 = (long)(int)i55 + ind->data[ind->size[0]];
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i56 = (int)i55;
              i55 = (long)i54 + ib;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i58 = anchorIdx - 1L;
              if (i58 > 2147483647L) {
                i58 = 2147483647L;
              } else {
                if (i58 < -2147483648L) {
                  i58 = -2147483648L;
                }
              }

              i58 = (int)i58 * 11L;
              if (i58 > 2147483647L) {
                i58 = 2147483647L;
              } else {
                if (i58 < -2147483648L) {
                  i58 = -2147483648L;
                }
              }

              i58 = 21L + (int)i58;
              if (i58 > 2147483647L) {
                i58 = 2147483647L;
              } else {
                if (i58 < -2147483648L) {
                  i58 = -2147483648L;
                }
              }

              i58 = (long)(int)i58 + (1 + (ia - 1) / residualDim);
              if (i58 > 2147483647L) {
                i58 = 2147483647L;
              } else {
                if (i58 < -2147483648L) {
                  i58 = -2147483648L;
                }
              }

              d_C_data[ic] += b_P[(i56 + 109 * ((int)i55 - 1)) - 1] * H->data
                [(ia - 1) % residualDim + H->size[0] * ((int)i58 - 1)];
            }
          }

          ar += residualDim;
        }

        br += 6;
        apnd = residualDim;
      }

      i55 = ind->data[0] - 1L;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = (int)i55 * 11L;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = 21L + (int)i55;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = (int)i55 + 6L;
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i55 = (long)(int)i55 + ind->data[ind->size[0]];
      if (i55 > 2147483647L) {
        i55 = 2147483647L;
      } else {
        if (i55 < -2147483648L) {
          i55 = -2147483648L;
        }
      }

      i58 = ind->data[0] - 1L;
      if (i58 > 2147483647L) {
        i58 = 2147483647L;
      } else {
        if (i58 < -2147483648L) {
          i58 = -2147483648L;
        }
      }

      i58 = (int)i58 * 11L;
      if (i58 > 2147483647L) {
        i58 = 2147483647L;
      } else {
        if (i58 < -2147483648L) {
          i58 = -2147483648L;
        }
      }

      i58 = 21L + (int)i58;
      if (i58 > 2147483647L) {
        i58 = 2147483647L;
      } else {
        if (i58 < -2147483648L) {
          i58 = -2147483648L;
        }
      }

      i58 = (int)i58 + 6L;
      if (i58 > 2147483647L) {
        i58 = 2147483647L;
      } else {
        if (i58 < -2147483648L) {
          i58 = -2147483648L;
        }
      }

      i58 = (long)(int)i58 + ind->data[ind->size[0]];
      if (i58 > 2147483647L) {
        i58 = 2147483647L;
      } else {
        if (i58 < -2147483648L) {
          i58 = -2147483648L;
        }
      }

      c_P = b_P[((int)i55 + 109 * ((int)i58 - 1)) - 1];
      idx = residualDim;
      for (i54 = 0; i54 < idx; i54++) {
        c_xt[i54] = (y_data[i54] + d_C_data[i54]) + H_c_data[i54] * c_P;
      }

      i54 = S->size[0] * S->size[1];
      S->size[0] = residualDim;
      S->size[1] = residualDim;
      emxEnsureCapacity((emxArray__common *)S, i54, (int)sizeof(double));
      for (i54 = 0; i54 < idx; i54++) {
        for (i56 = 0; i56 < residualDim; i56++) {
          S->data[i54 + S->size[0] * i56] = (S_feature_data[i54 + residualDim *
            i56] + c_C_data[i54 + residualDim * i56]) + c_xt[i54] * H_c_data[i56];
        }
      }

      // 'OnePointRANSAC_EKF:78' size_S = residualDim;
      // 'OnePointRANSAC_EKF:79' S(1:(size_S+1):size_S^2) = S(1:(size_S+1):size_S^2) + noiseParameters.image_noise; 
      i54 = residualDim * residualDim - 1;
      i56 = residualDim + 1;
      idx = div_s32_floor(i54, i56);
      for (i57 = 0; i57 <= idx; i57++) {
        b_S_feature_data[i57] = S->data[i56 * i57] + noiseParameters_image_noise;
      }

      idx = div_s32_floor(i54, i56) + 1;
      for (i54 = 0; i54 < idx; i54++) {
        S->data[(residualDim + 1) * i54] = b_S_feature_data[i54];
      }

      //  add R to HPH'
      // 'OnePointRANSAC_EKF:81' if coder.target('MATLAB')
      // 'OnePointRANSAC_EKF:87' K = (P*H')/S;
      i54 = K->size[0] * K->size[1];
      K->size[0] = 109;
      K->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)K, i54, (int)sizeof(double));
      idx = H->size[0];
      for (i54 = 0; i54 < idx; i54++) {
        for (i56 = 0; i56 < 109; i56++) {
          K->data[i56 + K->size[0] * i54] = H->data[i54 + H->size[0] * i56];
        }
      }

      unnamed_idx_1 = (unsigned int)K->size[1];
      i54 = y->size[0] * y->size[1];
      y->size[0] = 109;
      y->size[1] = (int)unnamed_idx_1;
      y->size[0] = 109;
      emxEnsureCapacity((emxArray__common *)y, i54, (int)sizeof(double));
      idx = y->size[1];
      for (i54 = 0; i54 < idx; i54++) {
        for (i56 = 0; i56 < 109; i56++) {
          y->data[i56 + y->size[0] * i54] = 0.0;
        }
      }

      if (K->size[1] == 0) {
      } else {
        idx = 109 * (K->size[1] - 1);
        for (apnd = 0; apnd <= idx; apnd += 109) {
          for (ic = apnd + 1; ic <= apnd + 109; ic++) {
            y->data[ic - 1] = 0.0;
          }
        }

        br = 0;
        for (apnd = 0; apnd <= idx; apnd += 109) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 109; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              for (ic = apnd; ic + 1 <= apnd + 109; ic++) {
                ia++;
                y->data[ic] += K->data[ib] * b_P[ia - 1];
              }
            }

            ar += 109;
          }

          br += 109;
        }
      }

      i54 = K->size[0] * K->size[1];
      K->size[0] = 109;
      K->size[1] = y->size[1];
      emxEnsureCapacity((emxArray__common *)K, i54, (int)sizeof(double));
      idx = y->size[0] * y->size[1];
      for (i54 = 0; i54 < idx; i54++) {
        K->data[i54] = y->data[i54];
      }

      mrdivide(K, S);

      // 'OnePointRANSAC_EKF:89' x_apo = K*r;
      if ((K->size[1] == 1) || (r->size[0] == 1)) {
        for (i54 = 0; i54 < 109; i54++) {
          x_apo[i54] = 0.0;
          idx = K->size[1];
          for (i56 = 0; i56 < idx; i56++) {
            b_x_apo = x_apo[i54] + K->data[i54 + K->size[0] * i56] * r->data[i56];
            x_apo[i54] = b_x_apo;
          }
        }
      } else {
        memset(&x_apo[0], 0, 109U * sizeof(double));
        ar = 0;
        for (ib = 0; ib + 1 <= K->size[1]; ib++) {
          if (r->data[ib] != 0.0) {
            ia = ar;
            for (ic = 0; ic < 109; ic++) {
              ia++;
              b_x_apo = x_apo[ic] + r->data[ib] * K->data[ia - 1];
              x_apo[ic] = b_x_apo;
            }
          }

          ar += 109;
        }
      }

      // 'OnePointRANSAC_EKF:91' R_cw = RotFromQuatJ(quatmultJ(quatPlusThetaJ(x_apo(4:6)), xt.robot_state.att)); 
      quatPlusThetaJ(*(double (*)[3])&x_apo[3], b_y_data);
      quatmultJ(b_y_data, b_xt->robot_state.att, c_xt);

      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
      // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
      // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
      R_cw[0] = ((c_xt[0] * c_xt[0] - c_xt[1] * c_xt[1]) - c_xt[2] * c_xt[2]) +
        c_xt[3] * c_xt[3];
      R_cw[3] = 2.0 * (c_xt[0] * c_xt[1] + c_xt[2] * c_xt[3]);
      R_cw[6] = 2.0 * (c_xt[0] * c_xt[2] - c_xt[1] * c_xt[3]);
      R_cw[1] = 2.0 * (c_xt[0] * c_xt[1] - c_xt[2] * c_xt[3]);
      R_cw[4] = ((-(c_xt[0] * c_xt[0]) + c_xt[1] * c_xt[1]) - c_xt[2] * c_xt[2])
        + c_xt[3] * c_xt[3];
      R_cw[7] = 2.0 * (c_xt[1] * c_xt[2] + c_xt[0] * c_xt[3]);
      R_cw[2] = 2.0 * (c_xt[0] * c_xt[2] + c_xt[1] * c_xt[3]);
      R_cw[5] = 2.0 * (c_xt[1] * c_xt[2] - c_xt[0] * c_xt[3]);
      R_cw[8] = ((-(c_xt[0] * c_xt[0]) - c_xt[1] * c_xt[1]) + c_xt[2] * c_xt[2])
        + c_xt[3] * c_xt[3];

      // 'OnePointRANSAC_EKF:92' r_wc = xt.robot_state.pos + x_apo(1:3);
      for (apnd = 0; apnd < 3; apnd++) {
        r_wc[apnd] = b_xt->robot_state.pos[apnd] + x_apo[apnd];
      }

      // 'OnePointRANSAC_EKF:94' for anchorIdx = 1:numAnchors
      for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
        // 'OnePointRANSAC_EKF:95' anchorPos = xt.anchor_states(anchorIdx).pos + x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:3)); 
        i54 = anchorIdx * 11;

        // 'OnePointRANSAC_EKF:96' anchorRot = RotFromQuatJ(quatmultJ(quatPlusThetaJ(x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(4:6))), xt.anchor_states(anchorIdx).att)); 
        i56 = anchorIdx * 11;
        for (i57 = 0; i57 < 3; i57++) {
          anchorPos[i57] = b_xt->anchor_states[anchorIdx].pos[i57] + x_apo[(i57
            + i54) + 21];
          c_x_apo[i57] = x_apo[(i57 + i56) + 24];
        }

        quatPlusThetaJ(c_x_apo, dv20);
        quatmultJ(dv20, b_xt->anchor_states[anchorIdx].att, c_xt);

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

        // 'OnePointRANSAC_EKF:98' for featureIdx = 1:numPointsPerAnchor
        for (featureIdx = 0; featureIdx < 5; featureIdx++) {
          // 'OnePointRANSAC_EKF:99' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 1 
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              1) {
            //  only update active features
            // 'OnePointRANSAC_EKF:100' rho = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth + x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
            rho = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              inverse_depth + x_apo[(anchorIdx * 11 + featureIdx) + 27];

            // 'OnePointRANSAC_EKF:101' m = xt.anchor_states(anchorIdx).feature_states(featureIdx).m; 
            // 'OnePointRANSAC_EKF:102' fp = R_cw*(anchorPos + anchorRot'*m/rho - r_wc); 
            for (i54 = 0; i54 < 3; i54++) {
              c_P = 0.0;
              for (i56 = 0; i56 < 3; i56++) {
                c_P += gryro_bias_cov[i56 + 3 * i54] * b_xt->
                  anchor_states[anchorIdx].feature_states[featureIdx].m[i56];
              }

              c_x_apo[i54] = (anchorPos[i54] + c_P / rho) - r_wc[i54];
            }

            for (i54 = 0; i54 < 3; i54++) {
              fp[i54] = 0.0;
              for (i56 = 0; i56 < 3; i56++) {
                fp[i54] += R_cw[i54 + 3 * i56] * c_x_apo[i56];
              }
            }

            // 'OnePointRANSAC_EKF:103' if VIOParameters.full_stereo
            if (VIOParameters_full_stereo) {
              // 'OnePointRANSAC_EKF:104' [ h_u_l, h_u_r ] = predictMeasurementStereo( fp, stereoParams ); 
              predictMeasurementStereo(fp, c_stereoParams_CameraParameters,
                d_stereoParams_CameraParameters, e_stereoParams_CameraParameters,
                f_stereoParams_CameraParameters, stereoParams_r_lr,
                stereoParams_R_rl, h_u_l, h_u_r);

              // 'OnePointRANSAC_EKF:105' z_l = z_u_l((xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx-1)*2 + int32(1:2)); 
              i55 = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                status_idx - 1L;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i54 = (int)i55;
              if (i54 > 1073741823) {
                idx = MAX_int32_T;
              } else if (i54 <= -1073741824) {
                idx = MIN_int32_T;
              } else {
                idx = i54 << 1;
              }

              // 'OnePointRANSAC_EKF:106' z_r = z_u_r((xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx-1)*2 + int32(1:2)); 
              i55 = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                status_idx - 1L;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i54 = (int)i55;
              if (i54 > 1073741823) {
                b_m = MAX_int32_T;
              } else if (i54 <= -1073741824) {
                b_m = MIN_int32_T;
              } else {
                b_m = i54 << 1;
              }

              // 'OnePointRANSAC_EKF:107' innov = norm([h_u_l - z_l; h_u_r - z_r]); 
              for (apnd = 0; apnd < 2; apnd++) {
                i55 = (long)idx + (1 + apnd);
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                c_xt[apnd] = h_u_l[apnd] - z_u_l[(int)i55 - 1];
                i55 = (long)b_m + (1 + apnd);
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                c_xt[apnd + 2] = h_u_r[apnd] - z_u_r[(int)i55 - 1];
              }

              innov = b_norm(c_xt);
            } else {
              // 'OnePointRANSAC_EKF:108' else
              // 'OnePointRANSAC_EKF:109' h_u_l = predictMeasurementMono( fp, stereoParams.CameraParameters1 ); 
              // 'OnePointRANSAC_EKF:110' z_l = z_u_l((xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx-1)*2 + int32(1:2)); 
              i55 = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                status_idx - 1L;
              if (i55 > 2147483647L) {
                i55 = 2147483647L;
              } else {
                if (i55 < -2147483648L) {
                  i55 = -2147483648L;
                }
              }

              i54 = (int)i55;
              if (i54 > 1073741823) {
                idx = MAX_int32_T;
              } else if (i54 <= -1073741824) {
                idx = MIN_int32_T;
              } else {
                idx = i54 << 1;
              }

              // 'OnePointRANSAC_EKF:111' innov = norm(h_u_l - z_l);
              predictMeasurementMono(fp, c_stereoParams_CameraParameters,
                d_stereoParams_CameraParameters, h_u_l);
              for (apnd = 0; apnd < 2; apnd++) {
                i55 = (long)idx + (1 + apnd);
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                b_h_u_l[apnd] = h_u_l[apnd] - z_u_l[(int)i55 - 1];
              }

              innov = c_norm(b_h_u_l);
            }

            // 'OnePointRANSAC_EKF:113' if innov < LI_residual_thresh
            if (innov < LI_residual_thresh) {
              // 'OnePointRANSAC_EKF:114' LI_inlier_status_i(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = true; 
              HI_inlierCandidates[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = true;
            }
          }
        }
      }

      // 'OnePointRANSAC_EKF:120' if nnz(LI_inlier_status_i) > nnz(LI_inlier_status) 
      n = 0;
      b_m = 0;
      for (k = 0; k < 40; k++) {
        if (HI_inlierCandidates[k]) {
          n++;
        }

        if (LI_inlier_status[k]) {
          b_m++;
        }
      }

      if (n > b_m) {
        // 'OnePointRANSAC_EKF:121' LI_inlier_status = LI_inlier_status_i;
        // 'OnePointRANSAC_EKF:122' epsilon = 1 - nnz(LI_inlier_status_i)/nnz(activeFeatures); 
        n = 0;
        b_m = 0;
        for (k = 0; k < 40; k++) {
          LI_inlier_status[k] = HI_inlierCandidates[k];
          if (HI_inlierCandidates[k]) {
            n++;
          }

          if (activeFeatures[k]) {
            b_m++;
          }
        }

        // 'OnePointRANSAC_EKF:123' assert(epsilon <= 1)
        // 'OnePointRANSAC_EKF:124' num_hyp = log(1-0.99)/log(epsilon);
        num_hyp = -4.60517018598809 / log(1.0 - (double)n / (double)b_m);
      }

      // 'OnePointRANSAC_EKF:127' hyp_it = hyp_it + 1;
      hyp_it++;
    }

    emxFree_real_T(&y);

    //  ros_info('Found %i LI inliers in %i active features', nnz(LI_inlier_status), nnz(activeFeatures)) 
    // 'OnePointRANSAC_EKF:130' if nnz(LI_inlier_status) > LI_min_support_thresh 
    n = 0;
    for (k = 0; k < 40; k++) {
      if (LI_inlier_status[k]) {
        n++;
      }
    }

    if (n > 3) {
      emxInit_real_T1(&b_y, 2);

      // 'OnePointRANSAC_EKF:131' [r, H] = getH_R_res(xt, z_u_l, z_u_r, LI_inlier_status, stereoParams, VIOParameters); 
      b_getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att,
                   b_xt->anchor_states, z_u_l, z_u_r, LI_inlier_status,
                   c_stereoParams_CameraParameters,
                   d_stereoParams_CameraParameters,
                   e_stereoParams_CameraParameters,
                   f_stereoParams_CameraParameters, stereoParams_r_lr,
                   stereoParams_R_rl, VIOParameters_full_stereo, r, H);

      // 'OnePointRANSAC_EKF:133' S = (H*P*H');
      H_idx_0 = (unsigned int)H->size[0];
      i54 = b_y->size[0] * b_y->size[1];
      b_y->size[0] = (int)H_idx_0;
      b_y->size[1] = 109;
      emxEnsureCapacity((emxArray__common *)b_y, i54, (int)sizeof(double));
      b_m = H->size[0];
      i54 = b_y->size[0] * b_y->size[1];
      b_y->size[1] = 109;
      emxEnsureCapacity((emxArray__common *)b_y, i54, (int)sizeof(double));
      for (i54 = 0; i54 < 109; i54++) {
        ndbl = b_y->size[0];
        for (i56 = 0; i56 < ndbl; i56++) {
          b_y->data[i56 + b_y->size[0] * i54] = 0.0;
        }
      }

      if (H->size[0] == 0) {
      } else {
        idx = H->size[0] * 108;
        apnd = 0;
        while ((b_m > 0) && (apnd <= idx)) {
          i54 = apnd + b_m;
          for (ic = apnd; ic + 1 <= i54; ic++) {
            b_y->data[ic] = 0.0;
          }

          apnd += b_m;
        }

        br = 0;
        apnd = 0;
        while ((b_m > 0) && (apnd <= idx)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 109; ib++) {
            if (b_P[ib] != 0.0) {
              ia = ar;
              i54 = apnd + b_m;
              for (ic = apnd; ic + 1 <= i54; ic++) {
                ia++;
                b_y->data[ic] += b_P[ib] * H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 109;
          apnd += b_m;
        }
      }

      i54 = K->size[0] * K->size[1];
      K->size[0] = 109;
      K->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)K, i54, (int)sizeof(double));
      ndbl = H->size[0];
      for (i54 = 0; i54 < ndbl; i54++) {
        for (i56 = 0; i56 < 109; i56++) {
          K->data[i56 + K->size[0] * i54] = H->data[i54 + H->size[0] * i56];
        }
      }

      H_idx_0 = (unsigned int)b_y->size[0];
      unnamed_idx_1 = (unsigned int)K->size[1];
      i54 = S->size[0] * S->size[1];
      S->size[0] = (int)H_idx_0;
      S->size[1] = (int)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)S, i54, (int)sizeof(double));
      b_m = b_y->size[0];
      i54 = S->size[0] * S->size[1];
      emxEnsureCapacity((emxArray__common *)S, i54, (int)sizeof(double));
      ndbl = S->size[1];
      for (i54 = 0; i54 < ndbl; i54++) {
        idx = S->size[0];
        for (i56 = 0; i56 < idx; i56++) {
          S->data[i56 + S->size[0] * i54] = 0.0;
        }
      }

      if ((b_y->size[0] == 0) || (K->size[1] == 0)) {
      } else {
        idx = b_y->size[0] * (K->size[1] - 1);
        apnd = 0;
        while ((b_m > 0) && (apnd <= idx)) {
          i54 = apnd + b_m;
          for (ic = apnd; ic + 1 <= i54; ic++) {
            S->data[ic] = 0.0;
          }

          apnd += b_m;
        }

        br = 0;
        apnd = 0;
        while ((b_m > 0) && (apnd <= idx)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 109; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              i54 = apnd + b_m;
              for (ic = apnd; ic + 1 <= i54; ic++) {
                ia++;
                S->data[ic] += K->data[ib] * b_y->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 109;
          apnd += b_m;
        }
      }

      emxFree_real_T(&b_y);

      // 'OnePointRANSAC_EKF:134' size_S = nnz(LI_inlier_status)*residualDim;
      n = 0;
      for (k = 0; k < 40; k++) {
        if (LI_inlier_status[k]) {
          n++;
        }
      }

      size_S = (double)n * (double)residualDim;

      // 'OnePointRANSAC_EKF:135' S(1:(size_S+1):size_S^2) = S(1:(size_S+1):size_S^2) + noiseParameters.image_noise; 
      c_P = size_S * size_S;
      if ((size_S + 1.0 == 0.0) || (((size_S + 1.0 > 0.0) && (1.0 > c_P)) ||
           ((0.0 > size_S + 1.0) && (c_P > 1.0)))) {
        i54 = 1;
        i56 = -1;
      } else {
        i54 = (int)(size_S + 1.0);
        i56 = (int)c_P - 1;
      }

      c_P = size_S * size_S;
      if ((size_S + 1.0 == 0.0) || (((size_S + 1.0 > 0.0) && (1.0 > c_P)) ||
           ((0.0 > size_S + 1.0) && (c_P > 1.0)))) {
        i57 = 1;
      } else {
        i57 = (int)(size_S + 1.0);
      }

      emxInit_real_T1(&b_S, 2);
      b_m = b_S->size[0] * b_S->size[1];
      b_S->size[0] = 1;
      b_S->size[1] = div_s32_floor(i56, i54) + 1;
      emxEnsureCapacity((emxArray__common *)b_S, b_m, (int)sizeof(double));
      ndbl = div_s32_floor(i56, i54);
      for (i56 = 0; i56 <= ndbl; i56++) {
        b_S->data[b_S->size[0] * i56] = S->data[i54 * i56] +
          noiseParameters_image_noise;
      }

      ndbl = b_S->size[1];
      for (i54 = 0; i54 < ndbl; i54++) {
        S->data[i57 * i54] = b_S->data[b_S->size[0] * i54];
      }

      emxFree_real_T(&b_S);

      //  add R to HPH'
      // 'OnePointRANSAC_EKF:137' K = (P*H')/S;
      i54 = K->size[0] * K->size[1];
      K->size[0] = 109;
      K->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)K, i54, (int)sizeof(double));
      ndbl = H->size[0];
      for (i54 = 0; i54 < ndbl; i54++) {
        for (i56 = 0; i56 < 109; i56++) {
          K->data[i56 + K->size[0] * i54] = H->data[i54 + H->size[0] * i56];
        }
      }

      emxInit_real_T1(&c_y, 2);
      unnamed_idx_1 = (unsigned int)K->size[1];
      i54 = c_y->size[0] * c_y->size[1];
      c_y->size[0] = 109;
      c_y->size[1] = (int)unnamed_idx_1;
      c_y->size[0] = 109;
      emxEnsureCapacity((emxArray__common *)c_y, i54, (int)sizeof(double));
      ndbl = c_y->size[1];
      for (i54 = 0; i54 < ndbl; i54++) {
        for (i56 = 0; i56 < 109; i56++) {
          c_y->data[i56 + c_y->size[0] * i54] = 0.0;
        }
      }

      if (K->size[1] == 0) {
      } else {
        idx = 109 * (K->size[1] - 1);
        for (apnd = 0; apnd <= idx; apnd += 109) {
          for (ic = apnd + 1; ic <= apnd + 109; ic++) {
            c_y->data[ic - 1] = 0.0;
          }
        }

        br = 0;
        for (apnd = 0; apnd <= idx; apnd += 109) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 109; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              for (ic = apnd; ic + 1 <= apnd + 109; ic++) {
                ia++;
                c_y->data[ic] += K->data[ib] * b_P[ia - 1];
              }
            }

            ar += 109;
          }

          br += 109;
        }
      }

      i54 = K->size[0] * K->size[1];
      K->size[0] = 109;
      K->size[1] = c_y->size[1];
      emxEnsureCapacity((emxArray__common *)K, i54, (int)sizeof(double));
      ndbl = c_y->size[0] * c_y->size[1];
      for (i54 = 0; i54 < ndbl; i54++) {
        K->data[i54] = c_y->data[i54];
      }

      emxFree_real_T(&c_y);
      mrdivide(K, S);

      // 'OnePointRANSAC_EKF:139' x_apo = K*r;
      if ((K->size[1] == 1) || (r->size[0] == 1)) {
        for (i54 = 0; i54 < 109; i54++) {
          x_apo[i54] = 0.0;
          ndbl = K->size[1];
          for (i56 = 0; i56 < ndbl; i56++) {
            b_x_apo = x_apo[i54] + K->data[i54 + K->size[0] * i56] * r->data[i56];
            x_apo[i54] = b_x_apo;
          }
        }
      } else {
        memset(&x_apo[0], 0, 109U * sizeof(double));
        ar = 0;
        for (ib = 0; ib + 1 <= K->size[1]; ib++) {
          if (r->data[ib] != 0.0) {
            ia = ar;
            for (ic = 0; ic < 109; ic++) {
              ia++;
              b_x_apo = x_apo[ic] + r->data[ib] * K->data[ia - 1];
              x_apo[ic] = b_x_apo;
            }
          }

          ar += 109;
        }
      }

      // 'OnePointRANSAC_EKF:141' xt.robot_state.pos           = xt.robot_state.pos       + x_apo(1:3); 
      for (i54 = 0; i54 < 3; i54++) {
        b_xt->robot_state.pos[i54] += x_apo[i54];
      }

      // 'OnePointRANSAC_EKF:142' xt.robot_state.att           = quatmultJ(quatPlusThetaJ(x_apo(4:6)), xt.robot_state.att); 
      quatPlusThetaJ(*(double (*)[3])&x_apo[3], b_y_data);
      for (apnd = 0; apnd < 4; apnd++) {
        c_xt[apnd] = b_xt->robot_state.att[apnd];
      }

      quatmultJ(b_y_data, c_xt, b_xt->robot_state.att);

      // 'OnePointRANSAC_EKF:143' xt.robot_state.vel           = xt.robot_state.vel       + x_apo(7:9); 
      // 'OnePointRANSAC_EKF:144' xt.robot_state.IMU.gyro_bias = xt.robot_state.IMU.gyro_bias + x_apo(10:12); 
      // 'OnePointRANSAC_EKF:145' xt.robot_state.IMU.acc_bias  = xt.robot_state.IMU.acc_bias + x_apo(13:15); 
      for (i54 = 0; i54 < 3; i54++) {
        b_xt->robot_state.vel[i54] += x_apo[6 + i54];
        b_xt->robot_state.IMU.gyro_bias[i54] += x_apo[9 + i54];
        b_xt->robot_state.IMU.acc_bias[i54] += x_apo[12 + i54];
      }

      // 'OnePointRANSAC_EKF:146' xt.origin.att                = quatmultJ(quatPlusThetaJ(x_apo(16:18)), xt.origin.att); 
      quatPlusThetaJ(*(double (*)[3])&x_apo[15], b_y_data);
      for (apnd = 0; apnd < 4; apnd++) {
        c_xt[apnd] = b_xt->origin.att[apnd];
      }

      quatmultJ(b_y_data, c_xt, b_xt->origin.att);

      // 'OnePointRANSAC_EKF:147' xt.robot_state.IMU.att       = quatmultJ(quatPlusThetaJ(x_apo(19:21)), xt.robot_state.IMU.att); 
      quatPlusThetaJ(*(double (*)[3])&x_apo[18], b_y_data);
      for (apnd = 0; apnd < 4; apnd++) {
        c_xt[apnd] = b_xt->robot_state.IMU.att[apnd];
      }

      quatmultJ(b_y_data, c_xt, b_xt->robot_state.IMU.att);

      //          xt.robot_state.IMU.pos       = xt.robot_state.IMU.pos + x_apo(22:24); 
      // 'OnePointRANSAC_EKF:150' for anchorIdx = 1:numAnchors
      for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
        // 'OnePointRANSAC_EKF:151' xt.anchor_states(anchorIdx).pos = xt.anchor_states(anchorIdx).pos + x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:3)); 
        i54 = anchorIdx * 11;

        // 'OnePointRANSAC_EKF:152' xt.anchor_states(anchorIdx).att = quatmultJ(quatPlusThetaJ(x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(4:6))), xt.anchor_states(anchorIdx).att); 
        i56 = anchorIdx * 11;
        for (i57 = 0; i57 < 3; i57++) {
          b_xt->anchor_states[anchorIdx].pos[i57] += x_apo[(i57 + i54) + 21];
          c_x_apo[i57] = x_apo[(i57 + i56) + 24];
        }

        for (apnd = 0; apnd < 4; apnd++) {
          c_xt[apnd] = b_xt->anchor_states[anchorIdx].att[apnd];
        }

        quatPlusThetaJ(c_x_apo, dv21);
        quatmultJ(dv21, c_xt, b_xt->anchor_states[anchorIdx].att);

        // 'OnePointRANSAC_EKF:154' for featureIdx = 1:numPointsPerAnchor
        for (featureIdx = 0; featureIdx < 5; featureIdx++) {
          // 'OnePointRANSAC_EKF:155' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 1 
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              1) {
            //  only update active features
            // 'OnePointRANSAC_EKF:156' xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth + x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              inverse_depth += x_apo[(anchorIdx * 11 + featureIdx) + 27];

            // 'OnePointRANSAC_EKF:157' if xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth > 10 
            if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth > 10.0) {
              // 'OnePointRANSAC_EKF:158' ros_warn('feature %i is very close. Depth: %f', int32(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx), 1/xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth) 
              ros_warn(b_xt->anchor_states[anchorIdx].feature_states[featureIdx]
                       .status_idx, 1.0 / b_xt->anchor_states[anchorIdx].
                       feature_states[featureIdx].inverse_depth);
            }

            // 'OnePointRANSAC_EKF:160' if xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth < 0 
            if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0) {
              // 'OnePointRANSAC_EKF:161' ros_warn('Feature %i (%i on %i) is behind its anchor, rejecting', int32(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx), int32(featureIdx), int32(anchorIdx)) 
              b_ros_warn(b_xt->anchor_states[anchorIdx]
                         .feature_states[featureIdx].status_idx, featureIdx + 1,
                         anchorIdx + 1);

              // 'OnePointRANSAC_EKF:162' updateVect(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
              updateVect[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = 0;

              // 'OnePointRANSAC_EKF:163' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                0;

              // 'OnePointRANSAC_EKF:164' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                status_idx = 0;
            }
          }
        }
      }

      // 'OnePointRANSAC_EKF:171' P = (eye(numStates + numAnchors*numStatesPerAnchor)-K*H)*P; 
      if ((K->size[1] == 1) || (H->size[0] == 1)) {
        for (i54 = 0; i54 < 109; i54++) {
          for (i56 = 0; i56 < 109; i56++) {
            d_y[i54 + 109 * i56] = 0.0;
            ndbl = K->size[1];
            for (i57 = 0; i57 < ndbl; i57++) {
              d_y[i54 + 109 * i56] += K->data[i54 + K->size[0] * i57] * H->
                data[i57 + H->size[0] * i56];
            }
          }
        }
      } else {
        k = K->size[1];
        memset(&d_y[0], 0, 11881U * sizeof(double));
        for (apnd = 0; apnd <= 11773; apnd += 109) {
          for (ic = apnd; ic + 1 <= apnd + 109; ic++) {
            d_y[ic] = 0.0;
          }
        }

        br = 0;
        for (apnd = 0; apnd <= 11773; apnd += 109) {
          ar = 0;
          i54 = br + k;
          for (ib = br; ib + 1 <= i54; ib++) {
            if (H->data[ib] != 0.0) {
              ia = ar;
              for (ic = apnd; ic + 1 <= apnd + 109; ic++) {
                ia++;
                d_y[ic] += H->data[ib] * K->data[ia - 1];
              }
            }

            ar += 109;
          }

          br += k;
        }
      }

      c_eye(dv22);
      for (i54 = 0; i54 < 109; i54++) {
        for (i56 = 0; i56 < 109; i56++) {
          dv23[i56 + 109 * i54] = dv22[i56 + 109 * i54] - d_y[i56 + 109 * i54];
        }
      }

      for (i54 = 0; i54 < 109; i54++) {
        for (i56 = 0; i56 < 109; i56++) {
          dv24[i54 + 109 * i56] = 0.0;
          for (i57 = 0; i57 < 109; i57++) {
            dv24[i54 + 109 * i56] += dv23[i54 + 109 * i57] * b_P[i57 + 109 * i56];
          }
        }
      }

      for (i54 = 0; i54 < 109; i54++) {
        memcpy(&b_P[i54 * 109], &dv24[i54 * 109], 109U * sizeof(double));
      }
    } else {
      // 'OnePointRANSAC_EKF:172' else
      // 'OnePointRANSAC_EKF:173' LI_inlier_status(:) = false;
      for (apnd = 0; apnd < 40; apnd++) {
        LI_inlier_status[apnd] = false;
      }

      // 'OnePointRANSAC_EKF:174' ros_warn('1-Point RANSAC didnt find enough LI inliers') 
      c_ros_warn();
    }
  }

  // % D Partial EKF update using high-innovation inliers
  // 'OnePointRANSAC_EKF:179' HI_inlierCandidates = activeFeatures & ~LI_inlier_status; 
  //  high innovation inliers (ordered like updateVect)
  // 'OnePointRANSAC_EKF:180' HI_inlierStatus = HI_inlierCandidates;
  for (apnd = 0; apnd < 40; apnd++) {
    b_HI_inlierCandidates = (activeFeatures[apnd] && (!LI_inlier_status[apnd]));
    HI_inlierStatus[apnd] = b_HI_inlierCandidates;
    HI_inlierCandidates[apnd] = b_HI_inlierCandidates;
  }

  // 'OnePointRANSAC_EKF:181' HI_ind = find(HI_inlierStatus);
  idx = 0;
  b_m = 1;
  exitg1 = false;
  while ((!exitg1) && (b_m < 41)) {
    guard1 = false;
    if (HI_inlierCandidates[b_m - 1]) {
      idx++;
      ii_data[idx - 1] = (signed char)b_m;
      if (idx >= 40) {
        exitg1 = true;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      b_m++;
    }
  }

  if (1 > idx) {
    ii_size_idx_0 = 0;
  } else {
    ii_size_idx_0 = idx;
  }

  if (1 > idx) {
    ndbl = 0;
  } else {
    ndbl = idx;
  }

  for (i54 = 0; i54 < ndbl; i54++) {
    hyp_ind_data[i54] = ii_data[i54];
  }

  // 'OnePointRANSAC_EKF:182' num_HI_inlierCandidates = nnz(HI_inlierCandidates); 
  n = 0;
  for (k = 0; k < 40; k++) {
    if (HI_inlierCandidates[k]) {
      n++;
    }
  }

  // 'OnePointRANSAC_EKF:184' if any(HI_inlierCandidates)
  if (any(HI_inlierCandidates)) {
    emxInit_real_T1(&b_K, 2);
    emxInit_real_T1(&b_H, 2);

    // 'OnePointRANSAC_EKF:185' K = 0;
    i54 = b_K->size[0] * b_K->size[1];
    b_K->size[0] = 1;
    b_K->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)b_K, i54, (int)sizeof(double));
    b_K->data[0] = 0.0;

    //  for coder
    // 'OnePointRANSAC_EKF:186' H = 0;
    i54 = b_H->size[0] * b_H->size[1];
    b_H->size[0] = 1;
    b_H->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)b_H, i54, (int)sizeof(double));
    b_H->data[0] = 0.0;

    //  for coder
    // 'OnePointRANSAC_EKF:187' for it = 1:VIOParameters.max_ekf_iterations
    it = 1;
    emxInit_real_T1(&e_y, 2);
    emxInit_real_T1(&f_y, 2);
    emxInit_real_T1(&c_H, 2);
    emxInit_real_T1(&c_S, 2);
    emxInit_real_T1(&d_H, 2);
    while (it <= c_VIOParameters_max_ekf_iterati) {
      // 'OnePointRANSAC_EKF:188' xt = getScaledMap(xt);
      getScaledMap(b_xt);

      //  build the map according to the current estimate
      // 'OnePointRANSAC_EKF:190' [r, H, ind] = getH_R_res(xt, z_u_l, z_u_r, HI_inlierStatus, stereoParams, VIOParameters); 
      getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att,
                 b_xt->anchor_states, z_u_l, z_u_r, HI_inlierStatus,
                 c_stereoParams_CameraParameters,
                 d_stereoParams_CameraParameters,
                 e_stereoParams_CameraParameters,
                 f_stereoParams_CameraParameters, stereoParams_r_lr,
                 stereoParams_R_rl, VIOParameters_full_stereo, r, H, ind);
      i54 = b_H->size[0] * b_H->size[1];
      b_H->size[0] = H->size[0];
      b_H->size[1] = 109;
      emxEnsureCapacity((emxArray__common *)b_H, i54, (int)sizeof(double));
      ndbl = H->size[0] * H->size[1];
      for (i54 = 0; i54 < ndbl; i54++) {
        b_H->data[i54] = H->data[i54];
      }

      //  the residual is ordered by anchors/features, not like updateVect
      // 'OnePointRANSAC_EKF:193' P_a = P(1:6, 1:6);
      // 'OnePointRANSAC_EKF:195' for k = 1:length(HI_ind)
      for (k = 0; k < ii_size_idx_0; k++) {
        // 'OnePointRANSAC_EKF:196' anchorIdx = ind(k, 1);
        anchorIdx = ind->data[k];

        // 'OnePointRANSAC_EKF:197' featureIdx = ind(k, 2);
        // 'OnePointRANSAC_EKF:198' H_a = H((k-1)*residualDim + (1:residualDim), 1:6); 
        ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
        apnd = ndbl + 1;
        b_m = (ndbl - residualDim) + 1;
        if (fabs((double)b_m) < 4.4408920985006262E-16 * (double)residualDim) {
          ndbl++;
          apnd = residualDim;
        } else if (b_m > 0) {
          apnd = ndbl;
        } else {
          ndbl++;
        }

        c_xt[0] = 1.0;
        if (ndbl > 1) {
          c_xt[ndbl - 1] = apnd;
          i54 = ndbl - 1;
          idx = i54 / 2;
          b_m = 1;
          while (b_m <= idx - 1) {
            c_xt[1] = 2.0;
            c_xt[ndbl - 2] = (double)apnd - 1.0;
            b_m = 2;
          }

          if (idx << 1 == ndbl - 1) {
            c_xt[idx] = (1.0 + (double)apnd) / 2.0;
          } else {
            c_xt[idx] = 1.0 + (double)idx;
            c_xt[idx + 1] = apnd - idx;
          }
        }

        b_m = (unsigned char)((unsigned int)(unsigned char)k * residualDim);
        for (i54 = 0; i54 < 6; i54++) {
          for (i56 = 0; i56 < ndbl; i56++) {
            H_a_data[i56 + ndbl * i54] = b_H->data[((int)((double)b_m + c_xt[i56])
              + b_H->size[0] * i54) - 1];
          }
        }

        // 'OnePointRANSAC_EKF:199' H_b = H((k-1)*residualDim + (1:residualDim), numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6)); 
        b_ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
        apnd = b_ndbl + 1;
        b_m = (b_ndbl - residualDim) + 1;
        if (fabs((double)b_m) < 4.4408920985006262E-16 * (double)residualDim) {
          b_ndbl++;
          apnd = residualDim;
        } else if (b_m > 0) {
          apnd = b_ndbl;
        } else {
          b_ndbl++;
        }

        b_y_data[0] = 1.0;
        if (b_ndbl > 1) {
          b_y_data[b_ndbl - 1] = apnd;
          i54 = b_ndbl - 1;
          idx = i54 / 2;
          b_m = 1;
          while (b_m <= idx - 1) {
            b_y_data[1] = 2.0;
            b_y_data[b_ndbl - 2] = (double)apnd - 1.0;
            b_m = 2;
          }

          if (idx << 1 == b_ndbl - 1) {
            b_y_data[idx] = (1.0 + (double)apnd) / 2.0;
          } else {
            b_y_data[idx] = 1.0 + (double)idx;
            b_y_data[idx + 1] = apnd - idx;
          }
        }

        b_m = (unsigned char)((unsigned int)(unsigned char)k * residualDim);
        i55 = ind->data[k] - 1L;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = (int)i55 * 11L;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = 21L + (int)i55;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i54 = (int)i55;
        for (i56 = 0; i56 < 6; i56++) {
          for (i57 = 0; i57 < b_ndbl; i57++) {
            i55 = (long)i54 + (1 + i56);
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            H_b_data[i57 + b_ndbl * i56] = b_H->data[((int)((double)b_m +
              b_y_data[i57]) + b_H->size[0] * ((int)i55 - 1)) - 1];
          }
        }

        // 'OnePointRANSAC_EKF:200' H_c = H((k-1)*residualDim + (1:residualDim), numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
        c_ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
        apnd = c_ndbl + 1;
        b_m = (c_ndbl - residualDim) + 1;
        if (fabs((double)b_m) < 4.4408920985006262E-16 * (double)residualDim) {
          c_ndbl++;
          apnd = residualDim;
        } else if (b_m > 0) {
          apnd = c_ndbl;
        } else {
          c_ndbl++;
        }

        y_data[0] = 1.0;
        if (c_ndbl > 1) {
          y_data[c_ndbl - 1] = apnd;
          i54 = c_ndbl - 1;
          idx = i54 / 2;
          b_m = 1;
          while (b_m <= idx - 1) {
            y_data[1] = 2.0;
            y_data[c_ndbl - 2] = (double)apnd - 1.0;
            b_m = 2;
          }

          if (idx << 1 == c_ndbl - 1) {
            y_data[idx] = (1.0 + (double)apnd) / 2.0;
          } else {
            y_data[idx] = 1.0 + (double)idx;
            y_data[idx + 1] = apnd - idx;
          }
        }

        b_m = (unsigned char)((unsigned int)(unsigned char)k * residualDim);
        i55 = ind->data[k] - 1L;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = (int)i55 * 11L;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = 21L + (int)i55;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = (int)i55 + 6L;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = (long)(int)i55 + ind->data[k + ind->size[0]];
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i54 = (int)i55;
        for (i56 = 0; i56 < c_ndbl; i56++) {
          H_c_data[i56] = b_H->data[((int)((double)b_m + y_data[i56]) +
            b_H->size[0] * (i54 - 1)) - 1];
        }

        // 'OnePointRANSAC_EKF:201' P_b = P(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6), 1:6); 
        // 'OnePointRANSAC_EKF:202' P_c = P(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx, 1:6); 
        i55 = ind->data[k] - 1L;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = (int)i55 * 11L;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = 21L + (int)i55;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = (int)i55 + 6L;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = (long)(int)i55 + ind->data[k + ind->size[0]];
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i54 = (int)i55;
        for (i56 = 0; i56 < 6; i56++) {
          P_c[i56] = b_P[(i54 + 109 * i56) - 1];
        }

        // 'OnePointRANSAC_EKF:203' P_d = P_b';
        i55 = ind->data[k] - 1L;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = (int)i55 * 11L;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = 21L + (int)i55;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i54 = (int)i55;
        for (i56 = 0; i56 < 6; i56++) {
          for (i57 = 0; i57 < 6; i57++) {
            i55 = (long)i54 + (1 + i56);
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            P_d[i57 + 6 * i56] = b_P[((int)i55 + 109 * i57) - 1];
          }
        }

        // 'OnePointRANSAC_EKF:204' P_e = P(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6), numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6)); 
        // 'OnePointRANSAC_EKF:205' P_f = P(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx, numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6)); 
        i55 = ind->data[k] - 1L;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = (int)i55 * 11L;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = 21L + (int)i55;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i54 = (int)i55;
        i55 = ind->data[k] - 1L;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = (int)i55 * 11L;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = 21L + (int)i55;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = (int)i55 + 6L;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = (long)(int)i55 + ind->data[k + ind->size[0]];
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i56 = (int)i55;

        // 'OnePointRANSAC_EKF:206' P_g = P_c';
        // 'OnePointRANSAC_EKF:207' P_h = P_f';
        // 'OnePointRANSAC_EKF:208' P_i = P(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx, numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
        // 'OnePointRANSAC_EKF:209' S_feature = (H_a*P_a + H_b*P_b + H_c*P_c)*H_a' + ... 
        // 'OnePointRANSAC_EKF:210'                         (H_a*P_d + H_b*P_e + H_c*P_f)*H_b' + ... 
        // 'OnePointRANSAC_EKF:211'                         (H_a*P_g + H_b*P_h + H_c*P_i)*H_c'; 
        for (i57 = 0; i57 < 6; i57++) {
          i55 = (long)i54 + (1 + i57);
          if (i55 > 2147483647L) {
            i55 = 2147483647L;
          } else {
            if (i55 < -2147483648L) {
              i55 = -2147483648L;
            }
          }

          P_f[i57] = b_P[(i56 + 109 * ((int)i55 - 1)) - 1];
          for (b_m = 0; b_m < ndbl; b_m++) {
            e_C_data[b_m + ndbl * i57] = 0.0;
          }
        }

        idx = ndbl * 5;
        for (apnd = 0; apnd <= idx; apnd += ndbl) {
          i54 = apnd + ndbl;
          for (ic = apnd; ic + 1 <= i54; ic++) {
            e_C_data[ic] = 0.0;
          }
        }

        br = 0;
        for (apnd = 0; apnd <= idx; apnd += ndbl) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 6; ib++) {
            if (b_P[ib % 6 + 109 * (ib / 6)] != 0.0) {
              ia = ar;
              i54 = apnd + ndbl;
              for (ic = apnd; ic + 1 <= i54; ic++) {
                ia++;
                b_m = (unsigned char)((unsigned int)(unsigned char)k *
                                      residualDim);
                for (i56 = 0; i56 < ndbl; i56++) {
                  tmp_data[i56] = (int)((double)b_m + c_xt[i56]);
                }

                e_C_data[ic] += b_P[ib % 6 + 109 * (ib / 6)] * b_H->data
                  [(tmp_data[(ia - 1) % ndbl] + b_H->size[0] * ((ia - 1) / ndbl))
                  - 1];
              }
            }

            ar += ndbl;
          }

          br += 6;
        }

        for (i54 = 0; i54 < 6; i54++) {
          for (i56 = 0; i56 < b_ndbl; i56++) {
            f_C_data[i56 + b_ndbl * i54] = 0.0;
          }
        }

        idx = b_ndbl * 5;
        for (apnd = 0; apnd <= idx; apnd += b_ndbl) {
          i54 = apnd + b_ndbl;
          for (ic = apnd; ic + 1 <= i54; ic++) {
            f_C_data[ic] = 0.0;
          }
        }

        br = 0;
        for (apnd = 0; apnd <= idx; apnd += b_ndbl) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 6; ib++) {
            i55 = anchorIdx - 1L;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i55 = (int)i55 * 11L;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i55 = 21L + (int)i55;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i55 = (long)(int)i55 + (1 + ib % 6);
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            if (b_P[((int)i55 + 109 * (ib / 6)) - 1] != 0.0) {
              ia = ar;
              i54 = apnd + b_ndbl;
              for (ic = apnd; ic + 1 <= i54; ic++) {
                ia++;
                b_m = (unsigned char)((unsigned int)(unsigned char)k *
                                      residualDim);
                for (i56 = 0; i56 < b_ndbl; i56++) {
                  tmp_data[i56] = (int)((double)b_m + b_y_data[i56]);
                }

                i55 = ind->data[k] - 1L;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i55 = (int)i55 * 11L;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i55 = 21L + (int)i55;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i56 = (int)i55;
                i55 = (long)i56 + (1 + ib % 6);
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i58 = anchorIdx - 1L;
                if (i58 > 2147483647L) {
                  i58 = 2147483647L;
                } else {
                  if (i58 < -2147483648L) {
                    i58 = -2147483648L;
                  }
                }

                i58 = (int)i58 * 11L;
                if (i58 > 2147483647L) {
                  i58 = 2147483647L;
                } else {
                  if (i58 < -2147483648L) {
                    i58 = -2147483648L;
                  }
                }

                i58 = 21L + (int)i58;
                if (i58 > 2147483647L) {
                  i58 = 2147483647L;
                } else {
                  if (i58 < -2147483648L) {
                    i58 = -2147483648L;
                  }
                }

                i58 = (long)(int)i58 + (1 + (ia - 1) / b_ndbl);
                if (i58 > 2147483647L) {
                  i58 = 2147483647L;
                } else {
                  if (i58 < -2147483648L) {
                    i58 = -2147483648L;
                  }
                }

                f_C_data[ic] += b_P[((int)i55 + 109 * (ib / 6)) - 1] * b_H->
                  data[(tmp_data[(ia - 1) % b_ndbl] + b_H->size[0] * ((int)i58 -
                         1)) - 1];
              }
            }

            ar += b_ndbl;
          }

          br += 6;
        }

        for (i54 = 0; i54 < 6; i54++) {
          for (i56 = 0; i56 < ndbl; i56++) {
            g_C_data[i56 + ndbl * i54] = 0.0;
          }
        }

        idx = ndbl * 5;
        for (apnd = 0; apnd <= idx; apnd += ndbl) {
          i54 = apnd + ndbl;
          for (ic = apnd; ic + 1 <= i54; ic++) {
            g_C_data[ic] = 0.0;
          }
        }

        br = 0;
        for (apnd = 0; apnd <= idx; apnd += ndbl) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 6; ib++) {
            if (P_d[ib] != 0.0) {
              ia = ar;
              i54 = apnd + ndbl;
              for (ic = apnd; ic + 1 <= i54; ic++) {
                ia++;
                b_m = (unsigned char)((unsigned int)(unsigned char)k *
                                      residualDim);
                for (i56 = 0; i56 < ndbl; i56++) {
                  tmp_data[i56] = (int)((double)b_m + c_xt[i56]);
                }

                g_C_data[ic] += P_d[ib] * b_H->data[(tmp_data[(ia - 1) % ndbl] +
                  b_H->size[0] * ((ia - 1) / ndbl)) - 1];
              }
            }

            ar += ndbl;
          }

          br += 6;
        }

        for (i54 = 0; i54 < 6; i54++) {
          for (i56 = 0; i56 < b_ndbl; i56++) {
            h_C_data[i56 + b_ndbl * i54] = 0.0;
          }
        }

        idx = b_ndbl * 5;
        for (apnd = 0; apnd <= idx; apnd += b_ndbl) {
          i54 = apnd + b_ndbl;
          for (ic = apnd; ic + 1 <= i54; ic++) {
            h_C_data[ic] = 0.0;
          }
        }

        br = 0;
        for (apnd = 0; apnd <= idx; apnd += b_ndbl) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 6; ib++) {
            i55 = anchorIdx - 1L;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i55 = (int)i55 * 11L;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i55 = 21L + (int)i55;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i55 = (long)(int)i55 + (1 + ib % 6);
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i58 = anchorIdx - 1L;
            if (i58 > 2147483647L) {
              i58 = 2147483647L;
            } else {
              if (i58 < -2147483648L) {
                i58 = -2147483648L;
              }
            }

            i58 = (int)i58 * 11L;
            if (i58 > 2147483647L) {
              i58 = 2147483647L;
            } else {
              if (i58 < -2147483648L) {
                i58 = -2147483648L;
              }
            }

            i58 = 21L + (int)i58;
            if (i58 > 2147483647L) {
              i58 = 2147483647L;
            } else {
              if (i58 < -2147483648L) {
                i58 = -2147483648L;
              }
            }

            i58 = (long)(int)i58 + (1 + ib / 6);
            if (i58 > 2147483647L) {
              i58 = 2147483647L;
            } else {
              if (i58 < -2147483648L) {
                i58 = -2147483648L;
              }
            }

            if (b_P[((int)i55 + 109 * ((int)i58 - 1)) - 1] != 0.0) {
              ia = ar;
              i54 = apnd + b_ndbl;
              for (ic = apnd; ic + 1 <= i54; ic++) {
                ia++;
                b_m = (unsigned char)((unsigned int)(unsigned char)k *
                                      residualDim);
                for (i56 = 0; i56 < b_ndbl; i56++) {
                  tmp_data[i56] = (int)((double)b_m + b_y_data[i56]);
                }

                i55 = ind->data[k] - 1L;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i55 = (int)i55 * 11L;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i55 = 21L + (int)i55;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i56 = (int)i55;
                i55 = ind->data[k] - 1L;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i55 = (int)i55 * 11L;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i55 = 21L + (int)i55;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i57 = (int)i55;
                i55 = (long)i56 + (1 + ib % 6);
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i58 = (long)i57 + (1 + ib / 6);
                if (i58 > 2147483647L) {
                  i58 = 2147483647L;
                } else {
                  if (i58 < -2147483648L) {
                    i58 = -2147483648L;
                  }
                }

                i59 = anchorIdx - 1L;
                if (i59 > 2147483647L) {
                  i59 = 2147483647L;
                } else {
                  if (i59 < -2147483648L) {
                    i59 = -2147483648L;
                  }
                }

                i59 = (int)i59 * 11L;
                if (i59 > 2147483647L) {
                  i59 = 2147483647L;
                } else {
                  if (i59 < -2147483648L) {
                    i59 = -2147483648L;
                  }
                }

                i59 = 21L + (int)i59;
                if (i59 > 2147483647L) {
                  i59 = 2147483647L;
                } else {
                  if (i59 < -2147483648L) {
                    i59 = -2147483648L;
                  }
                }

                i59 = (long)(int)i59 + (1 + (ia - 1) / b_ndbl);
                if (i59 > 2147483647L) {
                  i59 = 2147483647L;
                } else {
                  if (i59 < -2147483648L) {
                    i59 = -2147483648L;
                  }
                }

                h_C_data[ic] += b_P[((int)i55 + 109 * ((int)i58 - 1)) - 1] *
                  b_H->data[(tmp_data[(ia - 1) % b_ndbl] + b_H->size[0] * ((int)
                  i59 - 1)) - 1];
              }
            }

            ar += b_ndbl;
          }

          br += 6;
        }

        for (i54 = 0; i54 < c_ndbl; i54++) {
          for (i56 = 0; i56 < 6; i56++) {
            a_data[i54 + c_ndbl * i56] = (e_C_data[i54 + ndbl * i56] +
              f_C_data[i54 + b_ndbl * i56]) + H_c_data[i54] * P_c[i56];
          }
        }

        for (i54 = 0; i54 < ndbl; i54++) {
          for (i56 = 0; i56 < 6; i56++) {
            b_data[i56 + 6 * i54] = H_a_data[i54 + ndbl * i56];
          }
        }

        for (i54 = 0; i54 < ndbl; i54++) {
          for (i56 = 0; i56 < c_ndbl; i56++) {
            i_C_data[i56 + c_ndbl * i54] = 0.0;
          }
        }

        idx = c_ndbl * (ndbl - 1);
        for (apnd = 0; apnd <= idx; apnd += c_ndbl) {
          i54 = apnd + c_ndbl;
          for (ic = apnd; ic + 1 <= i54; ic++) {
            i_C_data[ic] = 0.0;
          }
        }

        br = 0;
        for (apnd = 0; apnd <= idx; apnd += c_ndbl) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 6; ib++) {
            if (b_data[ib] != 0.0) {
              ia = ar;
              i54 = apnd + c_ndbl;
              for (ic = apnd; ic + 1 <= i54; ic++) {
                ia++;
                i_C_data[ic] += b_data[ib] * a_data[ia - 1];
              }
            }

            ar += c_ndbl;
          }

          br += 6;
        }

        for (i54 = 0; i54 < c_ndbl; i54++) {
          for (i56 = 0; i56 < 6; i56++) {
            a_data[i54 + c_ndbl * i56] = (g_C_data[i54 + ndbl * i56] +
              h_C_data[i54 + b_ndbl * i56]) + H_c_data[i54] * P_f[i56];
          }
        }

        for (i54 = 0; i54 < b_ndbl; i54++) {
          for (i56 = 0; i56 < 6; i56++) {
            b_data[i56 + 6 * i54] = H_b_data[i54 + b_ndbl * i56];
          }
        }

        for (i54 = 0; i54 < b_ndbl; i54++) {
          for (i56 = 0; i56 < c_ndbl; i56++) {
            j_C_data[i56 + c_ndbl * i54] = 0.0;
          }
        }

        idx = c_ndbl * (b_ndbl - 1);
        for (apnd = 0; apnd <= idx; apnd += c_ndbl) {
          i54 = apnd + c_ndbl;
          for (ic = apnd; ic + 1 <= i54; ic++) {
            j_C_data[ic] = 0.0;
          }
        }

        br = 0;
        for (apnd = 0; apnd <= idx; apnd += c_ndbl) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 6; ib++) {
            if (b_data[ib] != 0.0) {
              ia = ar;
              i54 = apnd + c_ndbl;
              for (ic = apnd; ic + 1 <= i54; ic++) {
                ia++;
                j_C_data[ic] += b_data[ib] * a_data[ia - 1];
              }
            }

            ar += c_ndbl;
          }

          br += 6;
        }

        for (i54 = 0; i54 < ndbl; i54++) {
          k_C_data[i54] = 0.0;
        }

        apnd = 0;
        while (apnd <= 0) {
          for (ic = 1; ic <= ndbl; ic++) {
            k_C_data[ic - 1] = 0.0;
          }

          apnd = ndbl;
        }

        br = 6;
        apnd = 0;
        while (apnd <= 0) {
          ar = 0;
          for (ib = br - 5; ib <= br; ib++) {
            i55 = ind->data[k] - 1L;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i55 = (int)i55 * 11L;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i55 = 21L + (int)i55;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i55 = (int)i55 + 6L;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i55 = (long)(int)i55 + ind->data[k + ind->size[0]];
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i54 = (int)i55;
            if (b_P[(i54 + 109 * (ib - 1)) - 1] != 0.0) {
              ia = ar;
              for (ic = 0; ic + 1 <= ndbl; ic++) {
                ia++;
                b_m = (unsigned char)((unsigned int)(unsigned char)k *
                                      residualDim);
                for (i54 = 0; i54 < ndbl; i54++) {
                  tmp_data[i54] = (int)((double)b_m + c_xt[i54]);
                }

                i55 = ind->data[k] - 1L;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i55 = (int)i55 * 11L;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i55 = 21L + (int)i55;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i55 = (int)i55 + 6L;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i55 = (long)(int)i55 + ind->data[k + ind->size[0]];
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i54 = (int)i55;
                k_C_data[ic] += b_P[(i54 + 109 * (ib - 1)) - 1] * b_H->data
                  [(tmp_data[(ia - 1) % ndbl] + b_H->size[0] * ((ia - 1) / ndbl))
                  - 1];
              }
            }

            ar += ndbl;
          }

          br += 6;
          apnd = ndbl;
        }

        for (i54 = 0; i54 < b_ndbl; i54++) {
          l_C_data[i54] = 0.0;
        }

        apnd = 0;
        while (apnd <= 0) {
          for (ic = 1; ic <= b_ndbl; ic++) {
            l_C_data[ic - 1] = 0.0;
          }

          apnd = b_ndbl;
        }

        br = 6;
        apnd = 0;
        while (apnd <= 0) {
          ar = 0;
          for (ib = br - 5; ib <= br; ib++) {
            i55 = ind->data[k] - 1L;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i55 = (int)i55 * 11L;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i55 = 21L + (int)i55;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i54 = (int)i55;
            i55 = ind->data[k] - 1L;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i55 = (int)i55 * 11L;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i55 = 21L + (int)i55;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i55 = (int)i55 + 6L;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i55 = (long)(int)i55 + ind->data[k + ind->size[0]];
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            i56 = (int)i55;
            i55 = (long)i54 + ib;
            if (i55 > 2147483647L) {
              i55 = 2147483647L;
            } else {
              if (i55 < -2147483648L) {
                i55 = -2147483648L;
              }
            }

            if (b_P[(i56 + 109 * ((int)i55 - 1)) - 1] != 0.0) {
              ia = ar;
              for (ic = 0; ic + 1 <= b_ndbl; ic++) {
                ia++;
                b_m = (unsigned char)((unsigned int)(unsigned char)k *
                                      residualDim);
                for (i54 = 0; i54 < b_ndbl; i54++) {
                  tmp_data[i54] = (int)((double)b_m + b_y_data[i54]);
                }

                i55 = ind->data[k] - 1L;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i55 = (int)i55 * 11L;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i55 = 21L + (int)i55;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i54 = (int)i55;
                i55 = ind->data[k] - 1L;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i55 = (int)i55 * 11L;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i55 = 21L + (int)i55;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i55 = (int)i55 + 6L;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i55 = (long)(int)i55 + ind->data[k + ind->size[0]];
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i56 = (int)i55;
                i55 = (long)i54 + ib;
                if (i55 > 2147483647L) {
                  i55 = 2147483647L;
                } else {
                  if (i55 < -2147483648L) {
                    i55 = -2147483648L;
                  }
                }

                i58 = anchorIdx - 1L;
                if (i58 > 2147483647L) {
                  i58 = 2147483647L;
                } else {
                  if (i58 < -2147483648L) {
                    i58 = -2147483648L;
                  }
                }

                i58 = (int)i58 * 11L;
                if (i58 > 2147483647L) {
                  i58 = 2147483647L;
                } else {
                  if (i58 < -2147483648L) {
                    i58 = -2147483648L;
                  }
                }

                i58 = 21L + (int)i58;
                if (i58 > 2147483647L) {
                  i58 = 2147483647L;
                } else {
                  if (i58 < -2147483648L) {
                    i58 = -2147483648L;
                  }
                }

                i58 = (long)(int)i58 + (1 + (ia - 1) / b_ndbl);
                if (i58 > 2147483647L) {
                  i58 = 2147483647L;
                } else {
                  if (i58 < -2147483648L) {
                    i58 = -2147483648L;
                  }
                }

                l_C_data[ic] += b_P[(i56 + 109 * ((int)i55 - 1)) - 1] *
                  b_H->data[(tmp_data[(ia - 1) % b_ndbl] + b_H->size[0] * ((int)
                  i58 - 1)) - 1];
              }
            }

            ar += b_ndbl;
          }

          br += 6;
          apnd = b_ndbl;
        }

        i55 = ind->data[k] - 1L;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = (int)i55 * 11L;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = 21L + (int)i55;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = (int)i55 + 6L;
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i55 = (long)(int)i55 + ind->data[k + ind->size[0]];
        if (i55 > 2147483647L) {
          i55 = 2147483647L;
        } else {
          if (i55 < -2147483648L) {
            i55 = -2147483648L;
          }
        }

        i58 = ind->data[k] - 1L;
        if (i58 > 2147483647L) {
          i58 = 2147483647L;
        } else {
          if (i58 < -2147483648L) {
            i58 = -2147483648L;
          }
        }

        i58 = (int)i58 * 11L;
        if (i58 > 2147483647L) {
          i58 = 2147483647L;
        } else {
          if (i58 < -2147483648L) {
            i58 = -2147483648L;
          }
        }

        i58 = 21L + (int)i58;
        if (i58 > 2147483647L) {
          i58 = 2147483647L;
        } else {
          if (i58 < -2147483648L) {
            i58 = -2147483648L;
          }
        }

        i58 = (int)i58 + 6L;
        if (i58 > 2147483647L) {
          i58 = 2147483647L;
        } else {
          if (i58 < -2147483648L) {
            i58 = -2147483648L;
          }
        }

        i58 = (long)(int)i58 + ind->data[k + ind->size[0]];
        if (i58 > 2147483647L) {
          i58 = 2147483647L;
        } else {
          if (i58 < -2147483648L) {
            i58 = -2147483648L;
          }
        }

        c_P = b_P[((int)i55 + 109 * ((int)i58 - 1)) - 1];
        for (i54 = 0; i54 < ndbl; i54++) {
          c_xt[i54] = (k_C_data[i54] + l_C_data[i54]) + H_c_data[i54] * c_P;
        }

        S_feature_size[0] = ndbl;
        S_feature_size[1] = c_ndbl;
        for (i54 = 0; i54 < ndbl; i54++) {
          for (i56 = 0; i56 < c_ndbl; i56++) {
            S_feature_data[i54 + ndbl * i56] = (i_C_data[i54 + c_ndbl * i56] +
              j_C_data[i54 + c_ndbl * i56]) + c_xt[i54] * H_c_data[i56];
          }
        }

        // 'OnePointRANSAC_EKF:213' size_S = residualDim;
        // 'OnePointRANSAC_EKF:214' S_feature(1:(size_S+1):size_S^2) = S_feature(1:(size_S+1):size_S^2) + noiseParameters.image_noise; 
        i54 = residualDim * residualDim - 1;
        i56 = residualDim + 1;
        ndbl = div_s32_floor(i54, i56);
        for (i57 = 0; i57 <= ndbl; i57++) {
          b_S_feature_data[i57] = S_feature_data[i56 * i57] +
            noiseParameters_image_noise;
        }

        ndbl = div_s32_floor(i54, i56) + 1;
        for (i54 = 0; i54 < ndbl; i54++) {
          S_feature_data[(residualDim + 1) * i54] = b_S_feature_data[i54];
        }

        //  add R to HPH'
        // 'OnePointRANSAC_EKF:216' if coder.target('MATLAB')
        // 'OnePointRANSAC_EKF:224' innov = r((k-1)*residualDim + (1:residualDim))' / S_feature * r((k-1)*residualDim + (1:residualDim)); 
        ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
        apnd = ndbl + 1;
        b_m = (ndbl - residualDim) + 1;
        if (fabs((double)b_m) < 4.4408920985006262E-16 * (double)residualDim) {
          ndbl++;
          apnd = residualDim;
        } else if (b_m > 0) {
          apnd = ndbl;
        } else {
          ndbl++;
        }

        c_xt[0] = 1.0;
        if (ndbl > 1) {
          c_xt[ndbl - 1] = apnd;
          i54 = ndbl - 1;
          idx = i54 / 2;
          b_m = 1;
          while (b_m <= idx - 1) {
            c_xt[1] = 2.0;
            c_xt[ndbl - 2] = (double)apnd - 1.0;
            b_m = 2;
          }

          if (idx << 1 == ndbl - 1) {
            c_xt[idx] = (1.0 + (double)apnd) / 2.0;
          } else {
            c_xt[idx] = 1.0 + (double)idx;
            c_xt[idx + 1] = apnd - idx;
          }
        }

        b_ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
        apnd = b_ndbl + 1;
        b_m = (b_ndbl - residualDim) + 1;
        if (fabs((double)b_m) < 4.4408920985006262E-16 * (double)residualDim) {
          b_ndbl++;
          apnd = residualDim;
        } else if (b_m > 0) {
          apnd = b_ndbl;
        } else {
          b_ndbl++;
        }

        b_y_data[0] = 1.0;
        if (b_ndbl > 1) {
          b_y_data[b_ndbl - 1] = apnd;
          i54 = b_ndbl - 1;
          idx = i54 / 2;
          b_m = 1;
          while (b_m <= idx - 1) {
            b_y_data[1] = 2.0;
            b_y_data[b_ndbl - 2] = (double)apnd - 1.0;
            b_m = 2;
          }

          if (idx << 1 == b_ndbl - 1) {
            b_y_data[idx] = (1.0 + (double)apnd) / 2.0;
          } else {
            b_y_data[idx] = 1.0 + (double)idx;
            b_y_data[idx + 1] = apnd - idx;
          }
        }

        y_size[0] = 1;
        y_size[1] = ndbl;
        b_m = (unsigned char)((unsigned int)(unsigned char)k * residualDim);
        for (i54 = 0; i54 < ndbl; i54++) {
          y_data[i54] = r->data[(int)((double)b_m + c_xt[i54]) - 1];
        }

        b_mrdivide(y_data, y_size, S_feature_data, S_feature_size);
        b_m = (unsigned char)((unsigned int)(unsigned char)k * residualDim);
        for (i54 = 0; i54 < b_ndbl; i54++) {
          H_c_data[i54] = r->data[(int)((double)b_m + b_y_data[i54]) - 1];
        }

        if ((y_size[1] == 1) || (b_ndbl == 1)) {
          c_P = 0.0;
          for (i54 = 0; i54 < y_size[1]; i54++) {
            c_P += y_data[y_size[0] * i54] * H_c_data[i54];
          }

          innov = c_P;
        } else {
          c_P = 0.0;
          for (i54 = 0; i54 < y_size[1]; i54++) {
            c_P += y_data[y_size[0] * i54] * H_c_data[i54];
          }

          innov = c_P;
        }

        // 'OnePointRANSAC_EKF:225' if innov > mahalanobis_thresh
        if (innov > mahalanobis_thresh) {
          // 'OnePointRANSAC_EKF:226' r((k-1)*residualDim + (1:residualDim)) = 0; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          b_m = (ndbl - residualDim) + 1;
          if (fabs((double)b_m) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (b_m > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          c_xt[0] = 1.0;
          if (ndbl > 1) {
            c_xt[ndbl - 1] = apnd;
            i54 = ndbl - 1;
            idx = i54 / 2;
            b_m = 1;
            while (b_m <= idx - 1) {
              c_xt[1] = 2.0;
              c_xt[ndbl - 2] = (double)apnd - 1.0;
              b_m = 2;
            }

            if (idx << 1 == ndbl - 1) {
              c_xt[idx] = (1.0 + (double)apnd) / 2.0;
            } else {
              c_xt[idx] = 1.0 + (double)idx;
              c_xt[idx + 1] = apnd - idx;
            }
          }

          b_m = (unsigned char)((unsigned int)(unsigned char)k * residualDim);
          for (i54 = 0; i54 < ndbl; i54++) {
            b_tmp_data[i54] = (int)((double)b_m + c_xt[i54]);
          }

          for (i54 = 0; i54 < ndbl; i54++) {
            r->data[b_tmp_data[i54] - 1] = 0.0;
          }

          // 'OnePointRANSAC_EKF:227' H((k-1)*residualDim + (1:residualDim), :) = 0; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          b_m = (ndbl - residualDim) + 1;
          if (fabs((double)b_m) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (b_m > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          c_xt[0] = 1.0;
          if (ndbl > 1) {
            c_xt[ndbl - 1] = apnd;
            i54 = ndbl - 1;
            idx = i54 / 2;
            b_m = 1;
            while (b_m <= idx - 1) {
              c_xt[1] = 2.0;
              c_xt[ndbl - 2] = (double)apnd - 1.0;
              b_m = 2;
            }

            if (idx << 1 == ndbl - 1) {
              c_xt[idx] = (1.0 + (double)apnd) / 2.0;
            } else {
              c_xt[idx] = 1.0 + (double)idx;
              c_xt[idx + 1] = apnd - idx;
            }
          }

          b_m = (unsigned char)((unsigned int)(unsigned char)k * residualDim);
          for (i54 = 0; i54 < ndbl; i54++) {
            tmp_data[i54] = (int)((double)b_m + c_xt[i54]);
          }

          for (i54 = 0; i54 < 109; i54++) {
            for (i56 = 0; i56 < ndbl; i56++) {
              b_H->data[(tmp_data[i56] + b_H->size[0] * i54) - 1] = 0.0;
            }
          }

          // 'OnePointRANSAC_EKF:228' if it == VIOParameters.max_ekf_iterations
          if (it == c_VIOParameters_max_ekf_iterati) {
            // 'OnePointRANSAC_EKF:229' HI_inlierStatus(HI_ind(k)) = false;
            HI_inlierStatus[hyp_ind_data[k] - 1] = false;

            //  only reject the feature if its still bad in last iteration, otherwise just dont use for this update 
          }

          //                  ros_info('rejecting %i', HI_ind(k))
          // 'OnePointRANSAC_EKF:232' if updateVect(HI_ind(k)) == 2
          if (updateVect[hyp_ind_data[k] - 1] == 2) {
            // 'OnePointRANSAC_EKF:233' ros_error('inconsistency')
            b_ros_error();
          }
        }
      }

      // 'OnePointRANSAC_EKF:238' S = (H*P*H');
      H_idx_0 = (unsigned int)b_H->size[0];
      i54 = e_y->size[0] * e_y->size[1];
      e_y->size[0] = (int)H_idx_0;
      e_y->size[1] = 109;
      emxEnsureCapacity((emxArray__common *)e_y, i54, (int)sizeof(double));
      b_m = b_H->size[0];
      i54 = e_y->size[0] * e_y->size[1];
      e_y->size[1] = 109;
      emxEnsureCapacity((emxArray__common *)e_y, i54, (int)sizeof(double));
      for (i54 = 0; i54 < 109; i54++) {
        ndbl = e_y->size[0];
        for (i56 = 0; i56 < ndbl; i56++) {
          e_y->data[i56 + e_y->size[0] * i54] = 0.0;
        }
      }

      if (b_H->size[0] == 0) {
      } else {
        idx = b_H->size[0] * 108;
        apnd = 0;
        while ((b_m > 0) && (apnd <= idx)) {
          i54 = apnd + b_m;
          for (ic = apnd; ic + 1 <= i54; ic++) {
            e_y->data[ic] = 0.0;
          }

          apnd += b_m;
        }

        br = 0;
        apnd = 0;
        while ((b_m > 0) && (apnd <= idx)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 109; ib++) {
            if (b_P[ib] != 0.0) {
              ia = ar;
              i54 = apnd + b_m;
              for (ic = apnd; ic + 1 <= i54; ic++) {
                ia++;
                e_y->data[ic] += b_P[ib] * b_H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 109;
          apnd += b_m;
        }
      }

      i54 = c_H->size[0] * c_H->size[1];
      c_H->size[0] = b_H->size[1];
      c_H->size[1] = b_H->size[0];
      emxEnsureCapacity((emxArray__common *)c_H, i54, (int)sizeof(double));
      ndbl = b_H->size[0];
      for (i54 = 0; i54 < ndbl; i54++) {
        idx = b_H->size[1];
        for (i56 = 0; i56 < idx; i56++) {
          c_H->data[i56 + c_H->size[0] * i54] = b_H->data[i54 + b_H->size[0] *
            i56];
        }
      }

      b_m = b_H->size[0];
      i54 = K->size[0] * K->size[1];
      K->size[0] = 109;
      K->size[1] = b_m;
      emxEnsureCapacity((emxArray__common *)K, i54, (int)sizeof(double));
      for (i54 = 0; i54 < b_m; i54++) {
        for (i56 = 0; i56 < 109; i56++) {
          K->data[i56 + K->size[0] * i54] = c_H->data[i56 + 109 * i54];
        }
      }

      H_idx_0 = (unsigned int)e_y->size[0];
      unnamed_idx_1 = (unsigned int)K->size[1];
      i54 = S->size[0] * S->size[1];
      S->size[0] = (int)H_idx_0;
      S->size[1] = (int)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)S, i54, (int)sizeof(double));
      b_m = e_y->size[0];
      i54 = S->size[0] * S->size[1];
      emxEnsureCapacity((emxArray__common *)S, i54, (int)sizeof(double));
      ndbl = S->size[1];
      for (i54 = 0; i54 < ndbl; i54++) {
        idx = S->size[0];
        for (i56 = 0; i56 < idx; i56++) {
          S->data[i56 + S->size[0] * i54] = 0.0;
        }
      }

      if ((e_y->size[0] == 0) || (K->size[1] == 0)) {
      } else {
        idx = e_y->size[0] * (K->size[1] - 1);
        apnd = 0;
        while ((b_m > 0) && (apnd <= idx)) {
          i54 = apnd + b_m;
          for (ic = apnd; ic + 1 <= i54; ic++) {
            S->data[ic] = 0.0;
          }

          apnd += b_m;
        }

        br = 0;
        apnd = 0;
        while ((b_m > 0) && (apnd <= idx)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 109; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              i54 = apnd + b_m;
              for (ic = apnd; ic + 1 <= i54; ic++) {
                ia++;
                S->data[ic] += K->data[ib] * e_y->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 109;
          apnd += b_m;
        }
      }

      // 'OnePointRANSAC_EKF:239' size_S = num_HI_inlierCandidates*residualDim;
      size_S = (double)n * (double)residualDim;

      // 'OnePointRANSAC_EKF:240' S(1:(size_S+1):size_S^2) = S(1:(size_S+1):size_S^2) + noiseParameters.image_noise; 
      c_P = size_S * size_S;
      if ((size_S + 1.0 == 0.0) || (((size_S + 1.0 > 0.0) && (1.0 > c_P)) ||
           ((0.0 > size_S + 1.0) && (c_P > 1.0)))) {
        i54 = 1;
        i56 = -1;
      } else {
        i54 = (int)(size_S + 1.0);
        i56 = (int)c_P - 1;
      }

      c_P = size_S * size_S;
      if ((size_S + 1.0 == 0.0) || (((size_S + 1.0 > 0.0) && (1.0 > c_P)) ||
           ((0.0 > size_S + 1.0) && (c_P > 1.0)))) {
        i57 = 1;
      } else {
        i57 = (int)(size_S + 1.0);
      }

      b_m = c_S->size[0] * c_S->size[1];
      c_S->size[0] = 1;
      c_S->size[1] = div_s32_floor(i56, i54) + 1;
      emxEnsureCapacity((emxArray__common *)c_S, b_m, (int)sizeof(double));
      ndbl = div_s32_floor(i56, i54);
      for (i56 = 0; i56 <= ndbl; i56++) {
        c_S->data[c_S->size[0] * i56] = S->data[i54 * i56] +
          noiseParameters_image_noise;
      }

      ndbl = c_S->size[1];
      for (i54 = 0; i54 < ndbl; i54++) {
        S->data[i57 * i54] = c_S->data[c_S->size[0] * i54];
      }

      //  add R to HPH'
      // 'OnePointRANSAC_EKF:242' K = (P*H')/S;
      i54 = d_H->size[0] * d_H->size[1];
      d_H->size[0] = b_H->size[1];
      d_H->size[1] = b_H->size[0];
      emxEnsureCapacity((emxArray__common *)d_H, i54, (int)sizeof(double));
      ndbl = b_H->size[0];
      for (i54 = 0; i54 < ndbl; i54++) {
        idx = b_H->size[1];
        for (i56 = 0; i56 < idx; i56++) {
          d_H->data[i56 + d_H->size[0] * i54] = b_H->data[i54 + b_H->size[0] *
            i56];
        }
      }

      b_m = b_H->size[0];
      i54 = K->size[0] * K->size[1];
      K->size[0] = 109;
      K->size[1] = b_m;
      emxEnsureCapacity((emxArray__common *)K, i54, (int)sizeof(double));
      for (i54 = 0; i54 < b_m; i54++) {
        for (i56 = 0; i56 < 109; i56++) {
          K->data[i56 + K->size[0] * i54] = d_H->data[i56 + 109 * i54];
        }
      }

      unnamed_idx_1 = (unsigned int)K->size[1];
      i54 = f_y->size[0] * f_y->size[1];
      f_y->size[0] = 109;
      f_y->size[1] = (int)unnamed_idx_1;
      f_y->size[0] = 109;
      emxEnsureCapacity((emxArray__common *)f_y, i54, (int)sizeof(double));
      ndbl = f_y->size[1];
      for (i54 = 0; i54 < ndbl; i54++) {
        for (i56 = 0; i56 < 109; i56++) {
          f_y->data[i56 + f_y->size[0] * i54] = 0.0;
        }
      }

      if (K->size[1] == 0) {
      } else {
        idx = 109 * (K->size[1] - 1);
        for (apnd = 0; apnd <= idx; apnd += 109) {
          for (ic = apnd + 1; ic <= apnd + 109; ic++) {
            f_y->data[ic - 1] = 0.0;
          }
        }

        br = 0;
        for (apnd = 0; apnd <= idx; apnd += 109) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 109; ib++) {
            if (K->data[ib] != 0.0) {
              ia = ar;
              for (ic = apnd; ic + 1 <= apnd + 109; ic++) {
                ia++;
                f_y->data[ic] += K->data[ib] * b_P[ia - 1];
              }
            }

            ar += 109;
          }

          br += 109;
        }
      }

      i54 = K->size[0] * K->size[1];
      K->size[0] = 109;
      K->size[1] = f_y->size[1];
      emxEnsureCapacity((emxArray__common *)K, i54, (int)sizeof(double));
      ndbl = f_y->size[0] * f_y->size[1];
      for (i54 = 0; i54 < ndbl; i54++) {
        K->data[i54] = f_y->data[i54];
      }

      mrdivide(K, S);
      i54 = b_K->size[0] * b_K->size[1];
      b_K->size[0] = 109;
      b_K->size[1] = K->size[1];
      emxEnsureCapacity((emxArray__common *)b_K, i54, (int)sizeof(double));
      ndbl = K->size[0] * K->size[1];
      for (i54 = 0; i54 < ndbl; i54++) {
        b_K->data[i54] = K->data[i54];
      }

      // 'OnePointRANSAC_EKF:244' x_apo = K*r;
      if ((b_K->size[1] == 1) || (r->size[0] == 1)) {
        ndbl = b_K->size[0];
        for (i54 = 0; i54 < ndbl; i54++) {
          x_apo_data[i54] = 0.0;
          idx = b_K->size[1];
          for (i56 = 0; i56 < idx; i56++) {
            b_x_apo_data = x_apo_data[i54] + b_K->data[i54 + b_K->size[0] * i56]
              * r->data[i56];
            x_apo_data[i54] = b_x_apo_data;
          }
        }
      } else {
        k = b_K->size[1];
        memset(&x_apo_data[0], 0, 109U * sizeof(double));
        for (ic = 1; ic < 110; ic++) {
          x_apo_data[ic - 1] = 0.0;
        }

        ar = 0;
        for (ib = 0; ib + 1 <= k; ib++) {
          if (r->data[ib] != 0.0) {
            ia = ar;
            for (ic = 0; ic + 1 < 110; ic++) {
              ia++;
              x_apo_data[ic] += r->data[ib] * b_K->data[ia - 1];
            }
          }

          ar += 109;
        }
      }

      // 'OnePointRANSAC_EKF:246' xt.robot_state.pos           = xt.robot_state.pos       + x_apo(1:3); 
      for (i54 = 0; i54 < 3; i54++) {
        b_xt->robot_state.pos[i54] += x_apo_data[i54];
      }

      // 'OnePointRANSAC_EKF:247' xt.robot_state.att           = quatmultJ(quatPlusThetaJ(x_apo(4:6)), xt.robot_state.att); 
      quatPlusThetaJ(*(double (*)[3])&x_apo_data[3], b_y_data);
      for (apnd = 0; apnd < 4; apnd++) {
        c_xt[apnd] = b_xt->robot_state.att[apnd];
      }

      quatmultJ(b_y_data, c_xt, b_xt->robot_state.att);

      // 'OnePointRANSAC_EKF:248' xt.robot_state.vel           = xt.robot_state.vel       + x_apo(7:9); 
      // 'OnePointRANSAC_EKF:249' xt.robot_state.IMU.gyro_bias = xt.robot_state.IMU.gyro_bias + x_apo(10:12); 
      // 'OnePointRANSAC_EKF:250' xt.robot_state.IMU.acc_bias  = xt.robot_state.IMU.acc_bias + x_apo(13:15); 
      for (i54 = 0; i54 < 3; i54++) {
        b_xt->robot_state.vel[i54] += x_apo_data[6 + i54];
        b_xt->robot_state.IMU.gyro_bias[i54] += x_apo_data[9 + i54];
        b_xt->robot_state.IMU.acc_bias[i54] += x_apo_data[12 + i54];
      }

      // 'OnePointRANSAC_EKF:251' xt.origin.att                = quatmultJ(quatPlusThetaJ(x_apo(16:18)), xt.origin.att); 
      quatPlusThetaJ(*(double (*)[3])&x_apo_data[15], b_y_data);
      for (apnd = 0; apnd < 4; apnd++) {
        c_xt[apnd] = b_xt->origin.att[apnd];
      }

      quatmultJ(b_y_data, c_xt, b_xt->origin.att);

      // 'OnePointRANSAC_EKF:252' xt.robot_state.IMU.att       = quatmultJ(quatPlusThetaJ(x_apo(19:21)), xt.robot_state.IMU.att); 
      quatPlusThetaJ(*(double (*)[3])&x_apo_data[18], b_y_data);
      for (apnd = 0; apnd < 4; apnd++) {
        c_xt[apnd] = b_xt->robot_state.IMU.att[apnd];
      }

      quatmultJ(b_y_data, c_xt, b_xt->robot_state.IMU.att);

      //          xt.robot_state.IMU.pos       = xt.robot_state.IMU.pos + x_apo(22:24); 
      // 'OnePointRANSAC_EKF:255' meas_idx = 1;
      // 'OnePointRANSAC_EKF:256' for anchorIdx = 1:numAnchors
      for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
        // 'OnePointRANSAC_EKF:257' xt.anchor_states(anchorIdx).pos = xt.anchor_states(anchorIdx).pos + x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:3)); 
        i54 = anchorIdx * 11;

        // 'OnePointRANSAC_EKF:258' xt.anchor_states(anchorIdx).att = quatmultJ(quatPlusThetaJ(x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + int32(4:6))), xt.anchor_states(anchorIdx).att); 
        i56 = anchorIdx * 11;
        for (i57 = 0; i57 < 3; i57++) {
          b_xt->anchor_states[anchorIdx].pos[i57] += x_apo_data[(i57 + i54) + 21];
          c_x_apo[i57] = x_apo_data[(i57 + i56) + 24];
        }

        for (apnd = 0; apnd < 4; apnd++) {
          c_xt[apnd] = b_xt->anchor_states[anchorIdx].att[apnd];
        }

        quatPlusThetaJ(c_x_apo, dv25);
        quatmultJ(dv25, c_xt, b_xt->anchor_states[anchorIdx].att);

        // 'OnePointRANSAC_EKF:260' for featureIdx = 1:numPointsPerAnchor
        for (featureIdx = 0; featureIdx < 5; featureIdx++) {
          // 'OnePointRANSAC_EKF:261' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 1 
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              1) {
            //  only update active features
            // 'OnePointRANSAC_EKF:262' if HI_inlierStatus(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) 
            if (HI_inlierStatus[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1]) {
              // 'OnePointRANSAC_EKF:263' xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth + x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth += x_apo_data[(anchorIdx * 11 + featureIdx) + 27];

              // 'OnePointRANSAC_EKF:264' if it == VIOParameters.max_ekf_iterations && xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth > 10 
              if ((it == c_VIOParameters_max_ekf_iterati) &&
                  (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                   inverse_depth > 10.0)) {
                // 'OnePointRANSAC_EKF:265' ros_warn('Feature %i is very close. Depth: %f', int32(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx), 1/xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth) 
                d_ros_warn(b_xt->anchor_states[anchorIdx]
                           .feature_states[featureIdx].status_idx, 1.0 /
                           b_xt->anchor_states[anchorIdx]
                           .feature_states[featureIdx].inverse_depth);
              }

              // 'OnePointRANSAC_EKF:267' if xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth < 0 
              if ((b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                   inverse_depth < 0.0) && (it ==
                   c_VIOParameters_max_ekf_iterati)) {
                // 'OnePointRANSAC_EKF:268' if it == VIOParameters.max_ekf_iterations 
                //  only reject if we are done iterating
                // 'OnePointRANSAC_EKF:269' ros_warn('Feature %i (%i on %i) is behind its anchor, rejecting', int32(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx), int32(featureIdx), int32(anchorIdx)) 
                b_ros_warn(b_xt->anchor_states[anchorIdx]
                           .feature_states[featureIdx].status_idx, featureIdx +
                           1, anchorIdx + 1);

                // 'OnePointRANSAC_EKF:270' updateVect(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
                updateVect[b_xt->anchor_states[anchorIdx]
                  .feature_states[featureIdx].status_idx - 1] = 0;

                // 'OnePointRANSAC_EKF:271' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
                b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status
                  = 0;

                // 'OnePointRANSAC_EKF:272' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
                b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                  status_idx = 0;
              }
            } else {
              if (HI_inlierCandidates[b_xt->anchor_states[anchorIdx].
                  feature_states[featureIdx].status_idx - 1] && (it ==
                   c_VIOParameters_max_ekf_iterati)) {
                // 'OnePointRANSAC_EKF:275' elseif HI_inlierCandidates(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) 
                //  if it is not a HI inlier, but was a candidate, it was rejected by mahalanobis 
                // 'OnePointRANSAC_EKF:276' if it == VIOParameters.max_ekf_iterations 
                //  only reject if we are done iterating
                // 'OnePointRANSAC_EKF:277' ros_info('Rejecting %i (%i on %i)', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx, featureIdx, anchorIdx) 
                c_ros_info(b_xt->anchor_states[anchorIdx]
                           .feature_states[featureIdx].status_idx, featureIdx +
                           1, anchorIdx + 1);

                // 'OnePointRANSAC_EKF:278' updateVect(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
                updateVect[b_xt->anchor_states[anchorIdx]
                  .feature_states[featureIdx].status_idx - 1] = 0;

                // 'OnePointRANSAC_EKF:279' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
                b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status
                  = 0;

                // 'OnePointRANSAC_EKF:280' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
                b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                  status_idx = 0;
              }
            }
          }

          // 'OnePointRANSAC_EKF:284' meas_idx = meas_idx + 1;
        }
      }

      it++;
    }

    emxFree_real_T(&d_H);
    emxFree_real_T(&c_S);
    emxFree_real_T(&c_H);
    emxFree_real_T(&f_y);
    emxFree_real_T(&e_y);

    // 'OnePointRANSAC_EKF:290' P = (eye(numStates + numAnchors*numStatesPerAnchor)-K*H)*P; 
    emxInit_real_T1(&C, 2);
    if ((b_K->size[1] == 1) || (b_H->size[0] == 1)) {
      i54 = C->size[0] * C->size[1];
      C->size[0] = b_K->size[0];
      C->size[1] = b_H->size[1];
      emxEnsureCapacity((emxArray__common *)C, i54, (int)sizeof(double));
      ndbl = b_K->size[0];
      for (i54 = 0; i54 < ndbl; i54++) {
        idx = b_H->size[1];
        for (i56 = 0; i56 < idx; i56++) {
          C->data[i54 + C->size[0] * i56] = 0.0;
          b_m = b_K->size[1];
          for (i57 = 0; i57 < b_m; i57++) {
            C->data[i54 + C->size[0] * i56] += b_K->data[i54 + b_K->size[0] *
              i57] * b_H->data[i57 + b_H->size[0] * i56];
          }
        }
      }
    } else {
      k = b_K->size[1];
      H_idx_0 = (unsigned int)b_K->size[0];
      unnamed_idx_1 = (unsigned int)b_H->size[1];
      i54 = C->size[0] * C->size[1];
      C->size[0] = (int)H_idx_0;
      C->size[1] = (int)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)C, i54, (int)sizeof(double));
      b_m = b_K->size[0];
      i54 = C->size[0] * C->size[1];
      emxEnsureCapacity((emxArray__common *)C, i54, (int)sizeof(double));
      ndbl = C->size[1];
      for (i54 = 0; i54 < ndbl; i54++) {
        idx = C->size[0];
        for (i56 = 0; i56 < idx; i56++) {
          C->data[i56 + C->size[0] * i54] = 0.0;
        }
      }

      idx = b_K->size[0] * (b_H->size[1] - 1);
      for (apnd = 0; apnd <= idx; apnd += b_m) {
        i54 = apnd + b_m;
        for (ic = apnd; ic + 1 <= i54; ic++) {
          C->data[ic] = 0.0;
        }
      }

      br = 0;
      for (apnd = 0; apnd <= idx; apnd += b_m) {
        ar = 0;
        i54 = br + k;
        for (ib = br; ib + 1 <= i54; ib++) {
          if (b_H->data[ib] != 0.0) {
            ia = ar;
            i56 = apnd + b_m;
            for (ic = apnd; ic + 1 <= i56; ic++) {
              ia++;
              C->data[ic] += b_H->data[ib] * b_K->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += k;
      }
    }

    emxFree_real_T(&b_H);
    emxFree_real_T(&b_K);
    c_eye(dv22);
    for (i54 = 0; i54 < 109; i54++) {
      for (i56 = 0; i56 < 109; i56++) {
        dv23[i56 + 109 * i54] = dv22[i56 + 109 * i54] - C->data[i56 + 109 * i54];
      }
    }

    emxFree_real_T(&C);
    for (i54 = 0; i54 < 109; i54++) {
      for (i56 = 0; i56 < 109; i56++) {
        dv26[i54 + 109 * i56] = 0.0;
        for (i57 = 0; i57 < 109; i57++) {
          dv26[i54 + 109 * i56] += dv23[i54 + 109 * i57] * b_P[i57 + 109 * i56];
        }
      }
    }

    for (i54 = 0; i54 < 109; i54++) {
      memcpy(&b_P[i54 * 109], &dv26[i54 * 109], 109U * sizeof(double));
    }
  }

  emxFree_int32_T(&ind);

  // % Update the delayed initialization features
  // 'OnePointRANSAC_EKF:295' if VIOParameters.delayed_initialization
  if (c_VIOParameters_delayed_initial) {
    emxInit_real_T1(&g_y, 2);

    // 'OnePointRANSAC_EKF:296' xt = getScaledMap(xt);
    getScaledMap(b_xt);

    // 'OnePointRANSAC_EKF:297' [r, H] = getH_R_res(xt, z_u_l, z_u_r, delayedFeatures, stereoParams, VIOParameters); 
    b_getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att,
                 b_xt->anchor_states, z_u_l, z_u_r, delayedFeatures,
                 c_stereoParams_CameraParameters,
                 d_stereoParams_CameraParameters,
                 e_stereoParams_CameraParameters,
                 f_stereoParams_CameraParameters, stereoParams_r_lr,
                 stereoParams_R_rl, VIOParameters_full_stereo, r, H);

    // 'OnePointRANSAC_EKF:299' S = (H*P*H');
    H_idx_0 = (unsigned int)H->size[0];
    i54 = g_y->size[0] * g_y->size[1];
    g_y->size[0] = (int)H_idx_0;
    g_y->size[1] = 109;
    emxEnsureCapacity((emxArray__common *)g_y, i54, (int)sizeof(double));
    b_m = H->size[0];
    i54 = g_y->size[0] * g_y->size[1];
    g_y->size[1] = 109;
    emxEnsureCapacity((emxArray__common *)g_y, i54, (int)sizeof(double));
    for (i54 = 0; i54 < 109; i54++) {
      ndbl = g_y->size[0];
      for (i56 = 0; i56 < ndbl; i56++) {
        g_y->data[i56 + g_y->size[0] * i54] = 0.0;
      }
    }

    if (H->size[0] == 0) {
    } else {
      idx = H->size[0] * 108;
      apnd = 0;
      while ((b_m > 0) && (apnd <= idx)) {
        i54 = apnd + b_m;
        for (ic = apnd; ic + 1 <= i54; ic++) {
          g_y->data[ic] = 0.0;
        }

        apnd += b_m;
      }

      br = 0;
      apnd = 0;
      while ((b_m > 0) && (apnd <= idx)) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 109; ib++) {
          if (b_P[ib] != 0.0) {
            ia = ar;
            i54 = apnd + b_m;
            for (ic = apnd; ic + 1 <= i54; ic++) {
              ia++;
              g_y->data[ic] += b_P[ib] * H->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += 109;
        apnd += b_m;
      }
    }

    i54 = K->size[0] * K->size[1];
    K->size[0] = 109;
    K->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)K, i54, (int)sizeof(double));
    ndbl = H->size[0];
    for (i54 = 0; i54 < ndbl; i54++) {
      for (i56 = 0; i56 < 109; i56++) {
        K->data[i56 + K->size[0] * i54] = H->data[i54 + H->size[0] * i56];
      }
    }

    H_idx_0 = (unsigned int)g_y->size[0];
    unnamed_idx_1 = (unsigned int)K->size[1];
    i54 = S->size[0] * S->size[1];
    S->size[0] = (int)H_idx_0;
    S->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)S, i54, (int)sizeof(double));
    b_m = g_y->size[0];
    i54 = S->size[0] * S->size[1];
    emxEnsureCapacity((emxArray__common *)S, i54, (int)sizeof(double));
    ndbl = S->size[1];
    for (i54 = 0; i54 < ndbl; i54++) {
      idx = S->size[0];
      for (i56 = 0; i56 < idx; i56++) {
        S->data[i56 + S->size[0] * i54] = 0.0;
      }
    }

    if ((g_y->size[0] == 0) || (K->size[1] == 0)) {
    } else {
      idx = g_y->size[0] * (K->size[1] - 1);
      apnd = 0;
      while ((b_m > 0) && (apnd <= idx)) {
        i54 = apnd + b_m;
        for (ic = apnd; ic + 1 <= i54; ic++) {
          S->data[ic] = 0.0;
        }

        apnd += b_m;
      }

      br = 0;
      apnd = 0;
      while ((b_m > 0) && (apnd <= idx)) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 109; ib++) {
          if (K->data[ib] != 0.0) {
            ia = ar;
            i54 = apnd + b_m;
            for (ic = apnd; ic + 1 <= i54; ic++) {
              ia++;
              S->data[ic] += K->data[ib] * g_y->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += 109;
        apnd += b_m;
      }
    }

    emxFree_real_T(&g_y);

    // 'OnePointRANSAC_EKF:300' size_S = nnz(delayedFeatures)*residualDim;
    n = 0;
    for (k = 0; k < 40; k++) {
      if (delayedFeatures[k]) {
        n++;
      }
    }

    size_S = (double)n * (double)residualDim;

    // 'OnePointRANSAC_EKF:301' S(1:(size_S+1):size_S^2) = S(1:(size_S+1):size_S^2) + noiseParameters.image_noise; 
    c_P = size_S * size_S;
    if ((size_S + 1.0 == 0.0) || (((size_S + 1.0 > 0.0) && (1.0 > c_P)) || ((0.0
           > size_S + 1.0) && (c_P > 1.0)))) {
      i54 = 1;
      i56 = -1;
    } else {
      i54 = (int)(size_S + 1.0);
      i56 = (int)c_P - 1;
    }

    c_P = size_S * size_S;
    if ((size_S + 1.0 == 0.0) || (((size_S + 1.0 > 0.0) && (1.0 > c_P)) || ((0.0
           > size_S + 1.0) && (c_P > 1.0)))) {
      i57 = 1;
    } else {
      i57 = (int)(size_S + 1.0);
    }

    emxInit_real_T1(&d_S, 2);
    b_m = d_S->size[0] * d_S->size[1];
    d_S->size[0] = 1;
    d_S->size[1] = div_s32_floor(i56, i54) + 1;
    emxEnsureCapacity((emxArray__common *)d_S, b_m, (int)sizeof(double));
    ndbl = div_s32_floor(i56, i54);
    for (i56 = 0; i56 <= ndbl; i56++) {
      d_S->data[d_S->size[0] * i56] = S->data[i54 * i56] +
        noiseParameters_image_noise;
    }

    ndbl = d_S->size[1];
    for (i54 = 0; i54 < ndbl; i54++) {
      S->data[i57 * i54] = d_S->data[d_S->size[0] * i54];
    }

    emxFree_real_T(&d_S);

    //  add R to HPH'
    // 'OnePointRANSAC_EKF:302' K = (P*H')/S;
    i54 = K->size[0] * K->size[1];
    K->size[0] = 109;
    K->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)K, i54, (int)sizeof(double));
    ndbl = H->size[0];
    for (i54 = 0; i54 < ndbl; i54++) {
      for (i56 = 0; i56 < 109; i56++) {
        K->data[i56 + K->size[0] * i54] = H->data[i54 + H->size[0] * i56];
      }
    }

    emxInit_real_T1(&h_y, 2);
    unnamed_idx_1 = (unsigned int)K->size[1];
    i54 = h_y->size[0] * h_y->size[1];
    h_y->size[0] = 109;
    h_y->size[1] = (int)unnamed_idx_1;
    h_y->size[0] = 109;
    emxEnsureCapacity((emxArray__common *)h_y, i54, (int)sizeof(double));
    ndbl = h_y->size[1];
    for (i54 = 0; i54 < ndbl; i54++) {
      for (i56 = 0; i56 < 109; i56++) {
        h_y->data[i56 + h_y->size[0] * i54] = 0.0;
      }
    }

    if (K->size[1] == 0) {
    } else {
      idx = 109 * (K->size[1] - 1);
      for (apnd = 0; apnd <= idx; apnd += 109) {
        for (ic = apnd + 1; ic <= apnd + 109; ic++) {
          h_y->data[ic - 1] = 0.0;
        }
      }

      br = 0;
      for (apnd = 0; apnd <= idx; apnd += 109) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 109; ib++) {
          if (K->data[ib] != 0.0) {
            ia = ar;
            for (ic = apnd; ic + 1 <= apnd + 109; ic++) {
              ia++;
              h_y->data[ic] += K->data[ib] * b_P[ia - 1];
            }
          }

          ar += 109;
        }

        br += 109;
      }
    }

    i54 = K->size[0] * K->size[1];
    K->size[0] = 109;
    K->size[1] = h_y->size[1];
    emxEnsureCapacity((emxArray__common *)K, i54, (int)sizeof(double));
    ndbl = h_y->size[0] * h_y->size[1];
    for (i54 = 0; i54 < ndbl; i54++) {
      K->data[i54] = h_y->data[i54];
    }

    emxFree_real_T(&h_y);
    mrdivide(K, S);

    // 'OnePointRANSAC_EKF:304' x_apo = K*r;
    if ((K->size[1] == 1) || (r->size[0] == 1)) {
      for (i54 = 0; i54 < 109; i54++) {
        x_apo[i54] = 0.0;
        ndbl = K->size[1];
        for (i56 = 0; i56 < ndbl; i56++) {
          b_x_apo = x_apo[i54] + K->data[i54 + K->size[0] * i56] * r->data[i56];
          x_apo[i54] = b_x_apo;
        }
      }
    } else {
      memset(&x_apo[0], 0, 109U * sizeof(double));
      ar = 0;
      for (ib = 0; ib + 1 <= K->size[1]; ib++) {
        if (r->data[ib] != 0.0) {
          ia = ar;
          for (ic = 0; ic < 109; ic++) {
            ia++;
            b_x_apo = x_apo[ic] + r->data[ib] * K->data[ia - 1];
            x_apo[ic] = b_x_apo;
          }
        }

        ar += 109;
      }
    }

    // 'OnePointRANSAC_EKF:306' for anchorIdx = 1:numAnchors
    for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
      // 'OnePointRANSAC_EKF:307' for featureIdx = 1:numPointsPerAnchor
      for (featureIdx = 0; featureIdx < 5; featureIdx++) {
        // 'OnePointRANSAC_EKF:308' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
            2) {
          // 'OnePointRANSAC_EKF:309' xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth + x_apo(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
          b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
            inverse_depth += x_apo[(anchorIdx * 11 + featureIdx) + 27];
        }
      }
    }

    // 'OnePointRANSAC_EKF:314' P = (eye(numStates + numAnchors*numStatesPerAnchor)-K*H)*P; 
    if ((K->size[1] == 1) || (H->size[0] == 1)) {
      for (i54 = 0; i54 < 109; i54++) {
        for (i56 = 0; i56 < 109; i56++) {
          d_y[i54 + 109 * i56] = 0.0;
          ndbl = K->size[1];
          for (i57 = 0; i57 < ndbl; i57++) {
            d_y[i54 + 109 * i56] += K->data[i54 + K->size[0] * i57] * H->
              data[i57 + H->size[0] * i56];
          }
        }
      }
    } else {
      k = K->size[1];
      memset(&d_y[0], 0, 11881U * sizeof(double));
      for (apnd = 0; apnd <= 11773; apnd += 109) {
        for (ic = apnd; ic + 1 <= apnd + 109; ic++) {
          d_y[ic] = 0.0;
        }
      }

      br = 0;
      for (apnd = 0; apnd <= 11773; apnd += 109) {
        ar = 0;
        i54 = br + k;
        for (ib = br; ib + 1 <= i54; ib++) {
          if (H->data[ib] != 0.0) {
            ia = ar;
            for (ic = apnd; ic + 1 <= apnd + 109; ic++) {
              ia++;
              d_y[ic] += H->data[ib] * K->data[ia - 1];
            }
          }

          ar += 109;
        }

        br += k;
      }
    }

    c_eye(dv22);
    for (i54 = 0; i54 < 109; i54++) {
      for (i56 = 0; i56 < 109; i56++) {
        dv23[i56 + 109 * i54] = dv22[i56 + 109 * i54] - d_y[i56 + 109 * i54];
      }
    }

    for (i54 = 0; i54 < 109; i54++) {
      for (i56 = 0; i56 < 109; i56++) {
        dv27[i54 + 109 * i56] = 0.0;
        for (i57 = 0; i57 < 109; i57++) {
          dv27[i54 + 109 * i56] += dv23[i54 + 109 * i57] * b_P[i57 + 109 * i56];
        }
      }
    }

    for (i54 = 0; i54 < 109; i54++) {
      memcpy(&b_P[i54 * 109], &dv27[i54 * 109], 109U * sizeof(double));
    }
  }

  emxFree_real_T(&H);
  emxFree_real_T(&r);
  emxFree_real_T(&K);
  emxFree_real_T(&S);

  // %
  // 'OnePointRANSAC_EKF:318' num_active_features_before = nnz(activeFeatures);
  n = 0;

  // 'OnePointRANSAC_EKF:319' num_active_features_after  = nnz(HI_inlierStatus | LI_inlier_status); 
  b_m = 0;
  for (k = 0; k < 40; k++) {
    if (activeFeatures[k]) {
      n++;
    }

    if (HI_inlierStatus[k] || LI_inlier_status[k]) {
      b_m++;
    }
  }

  // 'OnePointRANSAC_EKF:320' rejected_ratio = num_active_features_after/num_active_features_before; 
  rejected_ratio = (double)b_m / (double)n;

  // 'OnePointRANSAC_EKF:322' if rejected_ratio < 0.1
  if (rejected_ratio < 0.1) {
    //  if more than 90% were rejected
    // 'OnePointRANSAC_EKF:323' ros_error('1-point RANSAC rejected %d%% of all features! Resetting velocity.', int32(100-rejected_ratio*100)) 
    // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
    // 'ros_error:4' if coder.target('MATLAB')
    // 'ros_error:6' elseif ~coder.target('MEX')
    // 'ros_error:7' coder.cinclude('<ros/console.h>')
    // 'ros_error:8' coder.ceval('ROS_ERROR', [str, 0], varargin{:});
    memcpy(&cv54[0], &cv55[0], 66U * sizeof(char));
    c_P = rt_roundd_snf(100.0 - rejected_ratio * 100.0);
    if (c_P < 2.147483648E+9) {
      i54 = (int)c_P;
    } else {
      i54 = MAX_int32_T;
    }

    ROS_ERROR(cv54, i54);

    // 'OnePointRANSAC_EKF:325' gryro_bias_cov = P(10:12, 10:12);
    // 'OnePointRANSAC_EKF:326' acc_bias_cov = P(13:15, 13:15);
    // 'OnePointRANSAC_EKF:327' R_ci_cov = P(19:21, 19:21);
    // 'OnePointRANSAC_EKF:329' R_cw = RotFromQuatJ(xt.robot_state.att)*RotFromQuatJ(xt.origin.att); 
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
    // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
    // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    // 'RotFromQuatJ:10' R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4)); 
    // 'RotFromQuatJ:11'     2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)); 
    // 'RotFromQuatJ:12'     2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2]; 
    d_xt[0] = ((b_xt->robot_state.att[0] * b_xt->robot_state.att[0] -
                b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
               b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
      b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
    d_xt[3] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] +
                     b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
    d_xt[6] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] -
                     b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
    d_xt[1] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] -
                     b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
    d_xt[4] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) +
                b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
               b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
      b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
    d_xt[7] = 2.0 * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] +
                     b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
    d_xt[2] = 2.0 * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] +
                     b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
    d_xt[5] = 2.0 * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] -
                     b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
    d_xt[8] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) -
                b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) +
               b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
      b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
    e_xt[0] = ((b_xt->origin.att[0] * b_xt->origin.att[0] - b_xt->origin.att[1] *
                b_xt->origin.att[1]) - b_xt->origin.att[2] * b_xt->origin.att[2])
      + b_xt->origin.att[3] * b_xt->origin.att[3];
    e_xt[3] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[1] +
                     b_xt->origin.att[2] * b_xt->origin.att[3]);
    e_xt[6] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[2] -
                     b_xt->origin.att[1] * b_xt->origin.att[3]);
    e_xt[1] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[1] -
                     b_xt->origin.att[2] * b_xt->origin.att[3]);
    e_xt[4] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) + b_xt->origin.att
                [1] * b_xt->origin.att[1]) - b_xt->origin.att[2] *
               b_xt->origin.att[2]) + b_xt->origin.att[3] * b_xt->origin.att[3];
    e_xt[7] = 2.0 * (b_xt->origin.att[1] * b_xt->origin.att[2] +
                     b_xt->origin.att[0] * b_xt->origin.att[3]);
    e_xt[2] = 2.0 * (b_xt->origin.att[0] * b_xt->origin.att[2] +
                     b_xt->origin.att[1] * b_xt->origin.att[3]);
    e_xt[5] = 2.0 * (b_xt->origin.att[1] * b_xt->origin.att[2] -
                     b_xt->origin.att[0] * b_xt->origin.att[3]);
    e_xt[8] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) - b_xt->origin.att
                [1] * b_xt->origin.att[1]) + b_xt->origin.att[2] *
               b_xt->origin.att[2]) + b_xt->origin.att[3] * b_xt->origin.att[3];
    for (i54 = 0; i54 < 3; i54++) {
      for (i56 = 0; i56 < 3; i56++) {
        gryro_bias_cov[i56 + 3 * i54] = b_P[(i56 + 109 * (9 + i54)) + 9];
        acc_bias_cov[i56 + 3 * i54] = b_P[(i56 + 109 * (12 + i54)) + 12];
        R_ci_cov[i56 + 3 * i54] = b_P[(i56 + 109 * (18 + i54)) + 18];
        R_cw[i54 + 3 * i56] = 0.0;
        for (i57 = 0; i57 < 3; i57++) {
          R_cw[i54 + 3 * i56] += d_xt[i54 + 3 * i57] * e_xt[i57 + 3 * i56];
        }
      }
    }

    // 'OnePointRANSAC_EKF:331' P(:, :) = 0;
    for (i54 = 0; i54 < 11881; i54++) {
      b_P[i54] = 0.0;
    }

    // 'OnePointRANSAC_EKF:333' P(  4:6,   4:6) = zeros(3);
    //  orientation of camera in origin frame
    // 'OnePointRANSAC_EKF:334' P(  7:9,   7:9) = 1*eye(3);
    //  velocity
    // 'OnePointRANSAC_EKF:335' P(10:12, 10:12) = gryro_bias_cov;
    //  gyro bias
    // 'OnePointRANSAC_EKF:336' P(13:15, 13:15) = acc_bias_cov;
    //  acc bias
    // 'OnePointRANSAC_EKF:337' P(16:18, 16:18) = 0.1*R_cw * diag([1 1 0]) * R_cw'; 
    for (i54 = 0; i54 < 3; i54++) {
      for (i56 = 0; i56 < 3; i56++) {
        b_P[(i56 + 109 * (3 + i54)) + 3] = 0.0;
        b_P[(i56 + 109 * (6 + i54)) + 6] = i_y[i56 + 3 * i54];
        b_P[(i56 + 109 * (9 + i54)) + 9] = gryro_bias_cov[i56 + 3 * i54];
        b_P[(i56 + 109 * (12 + i54)) + 12] = acc_bias_cov[i56 + 3 * i54];
        d_xt[i54 + 3 * i56] = 0.0;
        for (i57 = 0; i57 < 3; i57++) {
          d_xt[i54 + 3 * i56] += 0.1 * R_cw[i54 + 3 * i57] * (double)b[i57 + 3 *
            i56];
        }
      }
    }

    for (i54 = 0; i54 < 3; i54++) {
      for (i56 = 0; i56 < 3; i56++) {
        b_P[(i54 + 109 * (15 + i56)) + 15] = 0.0;
        for (i57 = 0; i57 < 3; i57++) {
          b_P[(i54 + 109 * (15 + i56)) + 15] += d_xt[i54 + 3 * i57] * R_cw[i56 +
            3 * i57];
        }
      }
    }

    //  origin orientation
    // 'OnePointRANSAC_EKF:338' P(19:21, 19:21) = R_ci_cov;
    for (i54 = 0; i54 < 3; i54++) {
      for (i56 = 0; i56 < 3; i56++) {
        b_P[(i56 + 109 * (18 + i54)) + 18] = R_ci_cov[i56 + 3 * i54];
      }
    }

    //  R_ci
    //  set all features inactive
    // 'OnePointRANSAC_EKF:341' for anchorIdx = 1:numAnchors
    for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
      // 'OnePointRANSAC_EKF:342' for featureIdx = 1:numPointsPerAnchor
      for (featureIdx = 0; featureIdx < 5; featureIdx++) {
        // 'OnePointRANSAC_EKF:343' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status 
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status !=
            0) {
          // 'OnePointRANSAC_EKF:344' updateVect(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
          updateVect[b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
            status_idx - 1] = 0;

          // 'OnePointRANSAC_EKF:345' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
          b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status = 0;

          // 'OnePointRANSAC_EKF:346' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
          b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status_idx =
            0;
        }
      }
    }
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
// Arguments    : double P_apo[11881]
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
static void SLAM_pred_euler(double P_apo[11881], g_struct_T *x, double dt,
  double processNoise_qv, double processNoise_qw, double processNoise_qao,
  double processNoise_qwo, double processNoise_qR_ci, const double
  measurements_acc_duo[3], const double measurements_gyr_duo[3])
{
  double R_cw[9];
  double R_ci[9];
  double b_R_ci[9];
  int i49;
  int i;
  double t_ci[3];
  double w_imu[3];
  double R[9];
  double b_R[9];
  double b_measurements_acc_duo[3];
  double w_c[3];
  double d10;
  int i50;
  double b_x[9];
  double a_c[3];
  double grav_origin[3];
  double y[3];
  static const double b[3] = { 0.0, 0.0, 9.81 };

  double dv9[9];
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
  double dv10[9];
  double dv11[9];
  double dv12[9];
  double dv13[9];
  double dv14[9];
  double dv15[9];
  double dv16[9];
  double dv17[9];
  double Phi[441];
  static const signed char iv6[63] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv7[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  double b_Phi[441];
  double b_processNoise_qw[15];
  double dv18[225];
  double b_G[315];
  double c_G[441];
  double P_xs_apr[1848];
  double c_x[4];
  double dv19[4];

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
  for (i49 = 0; i49 < 3; i49++) {
    for (i = 0; i < 3; i++) {
      b_R_ci[i + 3 * i49] = -R_ci[i49 + 3 * i];
    }
  }

  //  in imu frame
  // 'SLAM_pred_euler:8' w_imu = measurements.gyr_duo - x.robot_state.IMU.gyro_bias; 
  for (i = 0; i < 3; i++) {
    t_ci[i] = 0.0;
    for (i49 = 0; i49 < 3; i49++) {
      t_ci[i] += b_R_ci[i + 3 * i49] * x->robot_state.IMU.pos[i49];
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
  for (i49 = 0; i49 < 3; i49++) {
    w_c[i49] = 0.0;
    d10 = 0.0;
    for (i = 0; i < 3; i++) {
      w_c[i49] += R_ci[i49 + 3 * i] * w_imu[i];
      b_R[i49 + 3 * i] = 0.0;
      for (i50 = 0; i50 < 3; i50++) {
        b_R[i49 + 3 * i] += R[i49 + 3 * i50] * R[i50 + 3 * i];
      }

      d10 += b_R[i49 + 3 * i] * t_ci[i];
    }

    b_measurements_acc_duo[i49] = (measurements_acc_duo[i49] -
      x->robot_state.IMU.acc_bias[i49]) + d10;
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
  for (i49 = 0; i49 < 3; i49++) {
    a_c[i49] = 0.0;
    grav_origin[i49] = 0.0;
    y[i49] = 0.0;
    for (i = 0; i < 3; i++) {
      a_c[i49] += R_ci[i49 + 3 * i] * b_measurements_acc_duo[i];
      grav_origin[i49] += b_x[i49 + 3 * i] * b[i];
      y[i49] += R_ci[i49 + 3 * i] * x->robot_state.IMU.gyro_bias[i];
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
  dv9[0] = 0.0;
  dv9[3] = -w_imu[2];
  dv9[6] = w_imu[1];
  dv9[1] = w_imu[2];
  dv9[4] = 0.0;
  dv9[7] = -w_imu[0];
  dv9[2] = -w_imu[1];
  dv9[5] = w_imu[0];
  dv9[8] = 0.0;
  for (i49 = 0; i49 < 3; i49++) {
    b_y[i49] = 0.0;
    c_y[i49] = 0.0;
    for (i = 0; i < 3; i++) {
      b_y[i49] += R_cw[i + 3 * i49] * a_c[i];
      c_y[i49] += dv9[i49 + 3 * i] * t_ci[i];
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
  for (i49 = 0; i49 < 15; i49++) {
    for (i = 0; i < 3; i++) {
      G[i + 21 * i49] = 0.0;
      G[(i + 21 * i49) + 3] = iv2[i + 3 * i49];
    }
  }

  for (i49 = 0; i49 < 3; i49++) {
    for (i = 0; i < 3; i++) {
      G[(i + 21 * i49) + 6] = 0.0;
      G[(i + 21 * (i49 + 3)) + 6] = -R_cw[i49 + 3 * i];
      G[(i + 21 * (i49 + 6)) + 6] = 0.0;
      G[(i + 21 * (i49 + 9)) + 6] = 0.0;
      G[(i + 21 * (i49 + 12)) + 6] = 0.0;
    }
  }

  for (i49 = 0; i49 < 15; i49++) {
    for (i = 0; i < 3; i++) {
      G[(i + 21 * i49) + 9] = iv3[i + 3 * i49];
      G[(i + 21 * i49) + 12] = iv4[i + 3 * i49];
      G[(i + 21 * i49) + 15] = 0.0;
      G[(i + 21 * i49) + 18] = iv5[i + 3 * i49];
    }
  }

  //  R_ci
  // 'SLAM_pred_euler:48' Phi = eye(numStates) + F * dt;
  b_eye(P_xx_apr);
  dv10[0] = 0.0;
  dv10[3] = -w_c[2];
  dv10[6] = w_c[1];
  dv10[1] = w_c[2];
  dv10[4] = 0.0;
  dv10[7] = -w_c[0];
  dv10[2] = -w_c[1];
  dv10[5] = w_c[0];
  dv10[8] = 0.0;
  dv11[0] = 0.0;
  dv11[3] = -y[2];
  dv11[6] = y[1];
  dv11[1] = y[2];
  dv11[4] = 0.0;
  dv11[7] = -y[0];
  dv11[2] = -y[1];
  dv11[5] = y[0];
  dv11[8] = 0.0;
  dv12[0] = 0.0;
  dv12[3] = -b_y[2];
  dv12[6] = b_y[1];
  dv12[1] = b_y[2];
  dv12[4] = 0.0;
  dv12[7] = -b_y[0];
  dv12[2] = -b_y[1];
  dv12[5] = b_y[0];
  dv12[8] = 0.0;
  dv13[0] = 0.0;
  dv13[3] = -c_y[2];
  dv13[6] = c_y[1];
  dv13[1] = c_y[2];
  dv13[4] = 0.0;
  dv13[7] = -c_y[0];
  dv13[2] = -c_y[1];
  dv13[5] = c_y[0];
  dv13[8] = 0.0;
  dv14[0] = 0.0;
  dv14[3] = -w_imu[2];
  dv14[6] = w_imu[1];
  dv14[1] = w_imu[2];
  dv14[4] = 0.0;
  dv14[7] = -w_imu[0];
  dv14[2] = -w_imu[1];
  dv14[5] = w_imu[0];
  dv14[8] = 0.0;
  dv15[0] = 0.0;
  dv15[3] = -t_ci[2];
  dv15[6] = t_ci[1];
  dv15[1] = t_ci[2];
  dv15[4] = 0.0;
  dv15[7] = -t_ci[0];
  dv15[2] = -t_ci[1];
  dv15[5] = t_ci[0];
  dv15[8] = 0.0;
  dv16[0] = 0.0;
  dv16[3] = -grav_origin[2];
  dv16[6] = grav_origin[1];
  dv16[1] = grav_origin[2];
  dv16[4] = 0.0;
  dv16[7] = -grav_origin[0];
  dv16[2] = -grav_origin[1];
  dv16[5] = grav_origin[0];
  dv16[8] = 0.0;
  dv17[0] = 0.0;
  dv17[3] = -a_c[2];
  dv17[6] = a_c[1];
  dv17[1] = a_c[2];
  dv17[4] = 0.0;
  dv17[7] = -a_c[0];
  dv17[2] = -a_c[1];
  dv17[5] = a_c[0];
  dv17[8] = 0.0;
  for (i49 = 0; i49 < 3; i49++) {
    for (i = 0; i < 3; i++) {
      R[i + 3 * i49] = -R_cw[i49 + 3 * i];
      d10 = 0.0;
      for (i50 = 0; i50 < 3; i50++) {
        d10 += dv14[i49 + 3 * i50] * dv15[i50 + 3 * i];
      }

      dv9[i49 + 3 * i] = -dv13[i49 + 3 * i] - d10;
    }
  }

  for (i49 = 0; i49 < 3; i49++) {
    for (i = 0; i < 3; i++) {
      b_R_ci[i49 + 3 * i] = 0.0;
      b_R[i49 + 3 * i] = 0.0;
      for (i50 = 0; i50 < 3; i50++) {
        b_R_ci[i49 + 3 * i] += R[i49 + 3 * i50] * R_ci[i50 + 3 * i];
        b_R[i49 + 3 * i] += R_cw[i49 + 3 * i50] * dv17[i50 + 3 * i];
      }
    }
  }

  for (i49 = 0; i49 < 21; i49++) {
    for (i = 0; i < 3; i++) {
      Phi[i + 21 * i49] = iv6[i + 3 * i49];
    }
  }

  for (i49 = 0; i49 < 3; i49++) {
    for (i = 0; i < 3; i++) {
      Phi[(i + 21 * i49) + 3] = 0.0;
      Phi[(i + 21 * (i49 + 3)) + 3] = -dv10[i + 3 * i49];
      Phi[(i + 21 * (i49 + 6)) + 3] = 0.0;
      Phi[(i + 21 * (i49 + 9)) + 3] = iv7[i + 3 * i49];
      Phi[(i + 21 * (i49 + 12)) + 3] = 0.0;
      Phi[(i + 21 * (i49 + 15)) + 3] = 0.0;
      Phi[(i + 21 * (i49 + 18)) + 3] = -dv11[i + 3 * i49];
      Phi[(i + 21 * i49) + 6] = 0.0;
      Phi[(i + 21 * (i49 + 3)) + 6] = -dv12[i + 3 * i49];
      Phi[(i + 21 * (i49 + 6)) + 6] = 0.0;
      Phi[(i + 21 * (i49 + 9)) + 6] = dv9[i + 3 * i49];
      Phi[(i + 21 * (i49 + 12)) + 6] = b_R_ci[i + 3 * i49];
      Phi[(i + 21 * (i49 + 15)) + 6] = -dv16[i + 3 * i49];
      Phi[(i + 21 * (i49 + 18)) + 6] = b_R[i + 3 * i49];
    }
  }

  for (i49 = 0; i49 < 21; i49++) {
    for (i = 0; i < 3; i++) {
      Phi[(i + 21 * i49) + 9] = 0.0;
      Phi[(i + 21 * i49) + 12] = 0.0;
      Phi[(i + 21 * i49) + 15] = 0.0;
      Phi[(i + 21 * i49) + 18] = 0.0;
    }

    for (i = 0; i < 21; i++) {
      b_Phi[i + 21 * i49] = P_xx_apr[i + 21 * i49] + Phi[i + 21 * i49] * dt;
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
  b_diag(b_processNoise_qw, dv18);

  //  covariance of the state
  // 'SLAM_pred_euler:53' P_xx_apr = (P_xx_apr + P_xx_apr')/2;
  // 'SLAM_pred_euler:54' P_xs_apr = Phi*P_apo(states_idx, int32(numStates+1:int32(end))); 
  for (i49 = 0; i49 < 21; i49++) {
    for (i = 0; i < 21; i++) {
      Phi[i49 + 21 * i] = 0.0;
      for (i50 = 0; i50 < 21; i50++) {
        Phi[i49 + 21 * i] += b_Phi[i49 + 21 * i50] * P_apo[i50 + 109 * i];
      }
    }

    for (i = 0; i < 15; i++) {
      b_G[i49 + 21 * i] = 0.0;
      for (i50 = 0; i50 < 15; i50++) {
        b_G[i49 + 21 * i] += G[i49 + 21 * i50] * dv18[i50 + 15 * i];
      }
    }

    for (i = 0; i < 21; i++) {
      c_G[i49 + 21 * i] = 0.0;
      for (i50 = 0; i50 < 15; i50++) {
        c_G[i49 + 21 * i] += b_G[i49 + 21 * i50] * G[i + 21 * i50];
      }

      d10 = 0.0;
      for (i50 = 0; i50 < 21; i50++) {
        d10 += Phi[i49 + 21 * i50] * b_Phi[i + 21 * i50];
      }

      P_xx_apr[i49 + 21 * i] = d10 + c_G[i49 + 21 * i] * dt;
    }

    for (i = 0; i < 88; i++) {
      P_xs_apr[i49 + 21 * i] = 0.0;
      for (i50 = 0; i50 < 21; i50++) {
        P_xs_apr[i49 + 21 * i] += b_Phi[i49 + 21 * i50] * P_apo[i50 + 109 * (21
          + i)];
      }
    }
  }

  //  covariance between current state and trails
  // 'SLAM_pred_euler:56' P_apr = P_apo;
  // 'SLAM_pred_euler:57' P_apr(states_idx, states_idx) = P_xx_apr;
  for (i49 = 0; i49 < 21; i49++) {
    for (i = 0; i < 21; i++) {
      P_apo[i + 109 * i49] = (P_xx_apr[i + 21 * i49] + P_xx_apr[i49 + 21 * i]) /
        2.0;
    }
  }

  // 'SLAM_pred_euler:58' P_apr(states_idx, int32(numStates+1:int32(end))) = P_xs_apr; 
  for (i49 = 0; i49 < 88; i49++) {
    memcpy(&P_apo[i49 * 109 + 2289], &P_xs_apr[i49 * 21], 21U * sizeof(double));
  }

  // 'SLAM_pred_euler:59' P_apr(int32(numStates+1:int32(end)), states_idx) = P_xs_apr'; 
  for (i49 = 0; i49 < 21; i49++) {
    for (i = 0; i < 88; i++) {
      P_apo[(i + 109 * i49) + 21] = P_xs_apr[i49 + 21 * i];
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

  quatPlusThetaJ(b_measurements_acc_duo, dv19);
  quatmultJ(dv19, c_x, x->robot_state.att);

  // 'SLAM_pred_euler:64' x.robot_state.vel = x.robot_state.vel + (R_cw'*a_c - grav_origin)*dt; 
  for (i49 = 0; i49 < 3; i49++) {
    d10 = 0.0;
    for (i = 0; i < 3; i++) {
      d10 += R_cw[i + 3 * i49] * a_c[i];
    }

    b_measurements_acc_duo[i49] = d10 - grav_origin[i49];
    x->robot_state.vel[i49] += b_measurements_acc_duo[i49] * dt;
  }

  //  velocity
  //  P_apr = (P_apr+P_apr')/2;
}

//
// % Iterative Camera Pose optimization (EKF)
// Arguments    : double P_apr[11881]
//                g_struct_T *b_xt
//                int c_cameraParams_CameraParameters
//                const double d_cameraParams_CameraParameters[2]
//                const double e_cameraParams_CameraParameters[2]
//                const double f_cameraParams_CameraParameters[3]
//                int g_cameraParams_CameraParameters
//                int h_cameraParams_CameraParameters
//                const double i_cameraParams_CameraParameters[2]
//                const double j_cameraParams_CameraParameters[2]
//                const double k_cameraParams_CameraParameters[3]
//                int l_cameraParams_CameraParameters
//                const double cameraParams_r_lr[3]
//                const double cameraParams_R_lr[9]
//                const double cameraParams_R_rl[9]
//                int updateVect[40]
//                double z_all_l[80]
//                double z_all_r[80]
//                double noiseParameters_image_noise
//                double c_noiseParameters_inv_depth_ini
//                const VIOParameters b_VIOParameters
//                double b_map[120]
//                double b_delayedStatus[40]
// Return Type  : void
//
static void SLAM_upd(double P_apr[11881], g_struct_T *b_xt, int
                     c_cameraParams_CameraParameters, const double
                     d_cameraParams_CameraParameters[2], const double
                     e_cameraParams_CameraParameters[2], const double
                     f_cameraParams_CameraParameters[3], int
                     g_cameraParams_CameraParameters, int
                     h_cameraParams_CameraParameters, const double
                     i_cameraParams_CameraParameters[2], const double
                     j_cameraParams_CameraParameters[2], const double
                     k_cameraParams_CameraParameters[3], int
                     l_cameraParams_CameraParameters, const double
                     cameraParams_r_lr[3], const double cameraParams_R_lr[9],
                     const double cameraParams_R_rl[9], int updateVect[40],
                     double z_all_l[80], double z_all_r[80], double
                     noiseParameters_image_noise, double
                     c_noiseParameters_inv_depth_ini, const VIOParameters
                     b_VIOParameters, double b_map[120], double b_delayedStatus
                     [40])
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
  int i51;
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
  double uncertainties[5];
  signed char active_feature[5];
  int iidx[5];
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
  signed char i52;
  double h_u_r[2];
  double h_u_l[2];
  double b_h_u_l[2];
  boolean_T guard1 = false;
  double mtmp;
  int k;
  int trueCount;
  double triangulated_depths_data[40];
  signed char triangulated_status_ind_data[40];
  signed char b_triangulated_status_ind_data[40];
  int c_triangulated_status_ind_size_;
  double untriangulated_depths_data[40];
  emxArray_real_T *untriangulated_idx;
  int tmp_data[40];
  signed char untriangulated_status_ind_data[40];
  double b_new_m_data[120];
  double c_new_m_data[120];
  int new_feature_idx;
  boolean_T exitg4;
  long i53;
  double b_P_apr[36];
  int y_data[92];
  int b_tmp_data[92];
  double c_tmp_data[552];
  double d_tmp_data[552];
  int b_y_data[109];
  int e_tmp_data[109];
  double f_tmp_data[654];
  double g_tmp_data[654];
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
  double c_uncertainties[8];
  double b_has_active_features[8];
  boolean_T exitg1;
  char cv52[63];
  static const char cv53[63] = { 'P', 'i', 'c', 'k', 'e', 'd', ' ', 'a', 'n',
    ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', 'w', 'i', 't', 'h', ' ', 'n', 'o',
    ' ', 'a', 'c', 't', 'i', 'v', 'e', ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e',
    's', ' ', 'a', 's', ' ', 'o', 'r', 'i', 'g', 'i', 'n', ' ', '(', 'a', 'n',
    'c', 'h', 'o', 'r', ' ', '%', 'd', ')', '\x00' };

  double new_origin_att_rel[9];
  static double J[11881];
  double c_xt[9];
  double d_xt[3];
  double e_xt[9];
  static double b_J[11881];
  double f_xt[9];
  double g_xt[9];
  double h_xt[9];

  // 'SLAM_upd:5' numStatesPerAnchor = 6 + numPointsPerAnchor;
  // 'SLAM_upd:6' numTrackFeatures = numAnchors*numPointsPerAnchor;
  //  undistort all valid points
  // 'SLAM_upd:9' ind_l = find(updateVect ~=0);
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

  // 'SLAM_upd:10' ind_l2 = multiplyIdx(ind_l, 2);
  ii_size[0] = loop_ub;
  for (i51 = 0; i51 < loop_ub; i51++) {
    b_ii_data[i51] = ii_data[i51];
  }

  multiplyIdx(b_ii_data, ii_size, ind_l2_data, ind_l2_size);

  // 'SLAM_upd:11' z_all_l(ind_l2) = undistortPoint(z_all_l(ind_l2), cameraParams.CameraParameters1); 
  z_all_l_size[0] = ind_l2_size[0];
  loop_ub = ind_l2_size[0];
  for (i51 = 0; i51 < loop_ub; i51++) {
    z_all_l_data[i51] = z_all_l[(int)ind_l2_data[i51] - 1];
  }

  undistortPoint(z_all_l_data, z_all_l_size, c_cameraParams_CameraParameters,
                 d_cameraParams_CameraParameters,
                 e_cameraParams_CameraParameters,
                 f_cameraParams_CameraParameters,
                 g_cameraParams_CameraParameters, status_ind_data, ii_size);
  loop_ub = ii_size[0];
  for (i51 = 0; i51 < loop_ub; i51++) {
    z_all_l[(int)ind_l2_data[i51] - 1] = status_ind_data[i51];
  }

  // 'SLAM_upd:12' ind_r = find(updateVect == 2);
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
  for (i51 = 0; i51 < loop_ub; i51++) {
    ind_r_data[i51] = ii_data[i51];
  }

  // 'SLAM_upd:13' if VIOParameters.full_stereo
  if (b_VIOParameters.full_stereo) {
    // 'SLAM_upd:14' z_all_r(ind_l2) = undistortPoint(z_all_r(ind_l2), cameraParams.CameraParameters2); 
    z_all_r_size[0] = ind_l2_size[0];
    idx = ind_l2_size[0];
    for (i51 = 0; i51 < idx; i51++) {
      z_all_l_data[i51] = z_all_r[(int)ind_l2_data[i51] - 1];
    }

    undistortPoint(z_all_l_data, z_all_r_size, h_cameraParams_CameraParameters,
                   i_cameraParams_CameraParameters,
                   j_cameraParams_CameraParameters,
                   k_cameraParams_CameraParameters,
                   l_cameraParams_CameraParameters, status_ind_data, ii_size);
    idx = ii_size[0];
    for (i51 = 0; i51 < idx; i51++) {
      z_all_r[(int)ind_l2_data[i51] - 1] = status_ind_data[i51];
    }
  } else {
    // 'SLAM_upd:15' else
    // 'SLAM_upd:16' ind_r2 = multiplyIdx(ind_r, 2);
    multiplyIdx(ind_r_data, ind_r_size, ind_l2_data, ind_l2_size);

    // 'SLAM_upd:17' z_all_r(ind_r2) = undistortPoint(z_all_r(ind_r2), cameraParams.CameraParameters2); 
    z_all_r_size[0] = ind_l2_size[0];
    idx = ind_l2_size[0];
    for (i51 = 0; i51 < idx; i51++) {
      z_all_l_data[i51] = z_all_r[(int)ind_l2_data[i51] - 1];
    }

    undistortPoint(z_all_l_data, z_all_r_size, h_cameraParams_CameraParameters,
                   i_cameraParams_CameraParameters,
                   j_cameraParams_CameraParameters,
                   k_cameraParams_CameraParameters,
                   l_cameraParams_CameraParameters, status_ind_data, ii_size);
    idx = ii_size[0];
    for (i51 = 0; i51 < idx; i51++) {
      z_all_r[(int)ind_l2_data[i51] - 1] = status_ind_data[i51];
    }
  }

  //  check for lost features
  // 'SLAM_upd:21' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
    // 'SLAM_upd:22' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 5; featureIdx++) {
      // 'SLAM_upd:23' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status 
      if ((b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status != 0)
          && (updateVect[b_xt->anchor_states[anchorIdx]
              .feature_states[featureIdx].status_idx - 1] != 1)) {
        // 'SLAM_upd:24' idx = xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx; 
        // 'SLAM_upd:25' if updateVect(idx) ~= 1
        // 'SLAM_upd:26' P_apr(xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx, :) = 0; 
        idx = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx;
        for (i51 = 0; i51 < 109; i51++) {
          P_apr[(idx + 109 * i51) - 1] = 0.0;
        }

        // 'SLAM_upd:27' P_apr(:, xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx) = 0; 
        idx = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx;
        for (i51 = 0; i51 < 109; i51++) {
          P_apr[i51 + 109 * (idx - 1)] = 0.0;
        }

        // 'SLAM_upd:28' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
        b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status = 0;

        // 'SLAM_upd:29' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
        b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status_idx = 0;

        //                  ros_info('Lost feature %i, which was %i on anchor %i', idx, featureIdx, anchorIdx) 
      }
    }
  }

  // 'SLAM_upd:36' if VIOParameters.fixed_feature
  if (b_VIOParameters.fixed_feature) {
    // 'SLAM_upd:37' fix_new_feature = false;
    fix_new_feature = false;

    // 'SLAM_upd:39' if xt.origin.anchor_idx
    if (b_xt->origin.anchor_idx != 0) {
      // 'SLAM_upd:40' if xt.fixed_feature
      if (b_xt->fixed_feature != 0) {
        // 'SLAM_upd:41' if xt.anchor_states(xt.origin.anchor_idx).feature_states(xt.fixed_feature).status ~= 1 
        if (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].feature_states
            [b_xt->fixed_feature - 1].status != 1) {
          // 'SLAM_upd:42' fix_new_feature = true;
          fix_new_feature = true;

          // 'SLAM_upd:43' ros_info('Fixed feature %i (%i on anchor %i) is no longer valid', xt.anchor_states(xt.origin.anchor_idx).feature_states(xt.fixed_feature).status_idx, xt.fixed_feature, xt.origin.anchor_idx) 
          ros_info(b_xt->anchor_states[b_xt->origin.anchor_idx - 1].
                   feature_states[b_xt->fixed_feature - 1].status_idx,
                   b_xt->fixed_feature, b_xt->origin.anchor_idx);
        }
      } else {
        // 'SLAM_upd:45' else
        // 'SLAM_upd:46' fix_new_feature = true;
        fix_new_feature = true;
      }
    }

    // 'SLAM_upd:50' if fix_new_feature
    if (fix_new_feature) {
      // 'SLAM_upd:51' uncertainties = zeros(numPointsPerAnchor, 1);
      // 'SLAM_upd:52' active_feature = uncertainties;
      // 'SLAM_upd:53' for featureIdx = 1:numPointsPerAnchor
      for (featureIdx = 0; featureIdx < 5; featureIdx++) {
        active_feature[featureIdx] = 0;

        // 'SLAM_upd:54' if xt.anchor_states(xt.origin.anchor_idx).feature_states(featureIdx).status == 1 
        if (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
            .feature_states[featureIdx].status == 1) {
          // 'SLAM_upd:55' uncertainties(featureIdx) = P_apr(xt.anchor_states(xt.origin.anchor_idx).feature_states(featureIdx).P_idx, xt.anchor_states(xt.origin.anchor_idx).feature_states(featureIdx).P_idx); 
          uncertainties[featureIdx] = P_apr[(b_xt->anchor_states
            [b_xt->origin.anchor_idx - 1].feature_states[featureIdx].P_idx + 109
            * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
               .feature_states[featureIdx].P_idx - 1)) - 1];

          // 'SLAM_upd:56' active_feature(featureIdx) = 1;
          active_feature[featureIdx] = 1;
        } else {
          // 'SLAM_upd:57' else
          // 'SLAM_upd:58' uncertainties(featureIdx) = 1000;
          uncertainties[featureIdx] = 1000.0;

          //  dont fix an inactive feature
        }
      }

      // 'SLAM_upd:61' [~, sortIdx] = sort(uncertainties, 'ascend');
      sort(uncertainties, iidx);
      for (i = 0; i < 5; i++) {
        uncertainties[i] = iidx[i];
      }

      // 'SLAM_upd:61' ~
      // 'SLAM_upd:62' if ~active_feature(sortIdx(1))
      if (!(active_feature[(int)uncertainties[0] - 1] != 0)) {
        // 'SLAM_upd:63' ros_error('picked an inactive feature')
        ros_error();
      }

      // 'SLAM_upd:65' xt.fixed_feature = int32(sortIdx(1));
      b_xt->fixed_feature = (int)uncertainties[0];

      // 'SLAM_upd:66' P_apr(xt.anchor_states(xt.origin.anchor_idx).feature_states(xt.fixed_feature).P_idx, :) = 0; 
      idx = b_xt->anchor_states[b_xt->origin.anchor_idx - 1].feature_states
        [b_xt->fixed_feature - 1].P_idx;
      for (i51 = 0; i51 < 109; i51++) {
        P_apr[(idx + 109 * i51) - 1] = 0.0;
      }

      //  fix the feature depth
      // 'SLAM_upd:67' P_apr(:, xt.anchor_states(xt.origin.anchor_idx).feature_states(xt.fixed_feature).P_idx) = 0; 
      idx = b_xt->anchor_states[b_xt->origin.anchor_idx - 1].feature_states
        [b_xt->fixed_feature - 1].P_idx;
      for (i51 = 0; i51 < 109; i51++) {
        P_apr[i51 + 109 * (idx - 1)] = 0.0;
      }

      // 'SLAM_upd:68' ros_info('Fixing feature %i (feature %i on anchor %i)', xt.anchor_states(xt.origin.anchor_idx).feature_states(xt.fixed_feature).status_idx, xt.fixed_feature, xt.origin.anchor_idx) 
      b_ros_info(b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
                 .feature_states[b_xt->fixed_feature - 1].status_idx,
                 b_xt->fixed_feature, b_xt->origin.anchor_idx);
    }
  }

  // % do the update
  // 'SLAM_upd:73' if any(updateVect == 1)
  for (i = 0; i < 40; i++) {
    x[i] = (updateVect[i] == 1);
  }

  if (any(x)) {
    // 'SLAM_upd:74' [ xt, P_apo, updateVect ] = OnePointRANSAC_EKF(xt, P_apr, z_all_l, z_all_r, cameraParams, noiseParameters, VIOParameters, updateVect); 
    OnePointRANSAC_EKF(b_xt, P_apr, z_all_l, z_all_r,
                       d_cameraParams_CameraParameters,
                       e_cameraParams_CameraParameters,
                       i_cameraParams_CameraParameters,
                       j_cameraParams_CameraParameters, cameraParams_r_lr,
                       cameraParams_R_rl, noiseParameters_image_noise,
                       b_VIOParameters.max_ekf_iterations,
                       b_VIOParameters.delayed_initialization,
                       b_VIOParameters.full_stereo, b_VIOParameters.RANSAC,
                       updateVect);
  } else {
    // 'SLAM_upd:75' else
    // 'SLAM_upd:76' P_apo = P_apr;
  }

  // % Initialize new anchors/features
  // 'SLAM_upd:80' if length(ind_r) >= minFeatureThreshold
  emxInit_real_T(&qualities, 1);
  emxInit_real_T(&anchorInd, 1);
  emxInit_real_T(&featureInd, 1);
  emxInit_int32_T(&b_iidx, 1);
  if (ii_size_idx_0 >= 3) {
    //  try to triangulate all new features
    // 'SLAM_upd:82' new_depths = zeros(length(ind_r), 1);
    for (i51 = 0; i51 < ii_size_idx_0; i51++) {
      ind_l2_data[i51] = 0.0;
    }

    // 'SLAM_upd:83' new_m = zeros(3, length(ind_r));
    idx = 3 * ii_size_idx_0;
    for (i51 = 0; i51 < idx; i51++) {
      new_m_data[i51] = 0.0;
    }

    // 'SLAM_upd:84' triangulation_success = false(length(ind_r), 1);
    for (i51 = 0; i51 < ii_size_idx_0; i51++) {
      triangulation_success_data[i51] = false;
    }

    // 'SLAM_upd:86' for i = 1:length(ind_r)
    for (i = 0; i < loop_ub; i++) {
      // 'SLAM_upd:87' z_curr_l = z_all_l((ind_r(i)-1)*2 + (1:2));
      idx = ((int)ind_r_data[i] - 1) * 2;

      // 'SLAM_upd:88' z_curr_r = z_all_r((ind_r(i)-1)*2 + (1:2));
      ixstart = ((int)ind_r_data[i] - 1) * 2;
      for (i51 = 0; i51 < 2; i51++) {
        z_curr_l[i51] = z_all_l[idx + i51];
        z_curr_r[i51] = z_all_r[ixstart + i51];
      }

      // 'SLAM_upd:89' if ~VIOParameters.mono
      if (!b_VIOParameters.mono) {
        // 'SLAM_upd:90' [ fp, m, success ] = initializePoint(z_curr_l, z_curr_r, cameraParams); 
        initializePoint(z_curr_l, z_curr_r, d_cameraParams_CameraParameters,
                        e_cameraParams_CameraParameters,
                        i_cameraParams_CameraParameters,
                        j_cameraParams_CameraParameters, cameraParams_r_lr,
                        cameraParams_R_lr, new_origin_pos_rel, b_m, &success);

        // 'SLAM_upd:91' m_l = m(:,1);
        for (i51 = 0; i51 < 3; i51++) {
          m_l[i51] = b_m[i51];
        }

        // 'SLAM_upd:93' if success
        if (success) {
          //  perform further checks
          // 'SLAM_upd:94' if any(isnan(fp))
          for (idx = 0; idx < 3; idx++) {
            bv0[idx] = rtIsNaN(new_origin_pos_rel[idx]);
          }

          if (b_any(bv0)) {
            // 'SLAM_upd:95' ros_warn('Bad triangulation (nan) for point %d', int8(ind_r(i))); 
            i51 = (int)rt_roundd_snf(ind_r_data[i]);
            i52 = (signed char)i51;
            e_ros_warn(i52);

            // 'SLAM_upd:96' fp = m_l;
            for (i51 = 0; i51 < 3; i51++) {
              new_origin_pos_rel[i51] = b_m[i51];
            }

            // 'SLAM_upd:97' success = false;
            success = false;
          } else {
            // 'SLAM_upd:98' else
            //  check reprojection error
            // 'SLAM_upd:100' [h_u_l, h_u_r] = predictMeasurementStereo(fp, cameraParams); 
            predictMeasurementStereo(new_origin_pos_rel,
              d_cameraParams_CameraParameters, e_cameraParams_CameraParameters,
              i_cameraParams_CameraParameters, j_cameraParams_CameraParameters,
              cameraParams_r_lr, cameraParams_R_rl, h_u_l, h_u_r);

            // 'SLAM_upd:102' reprojection_error_thresh = 5;
            // 'SLAM_upd:103' if norm(h_u_l - z_curr_l) > reprojection_error_thresh || norm(h_u_r - z_curr_r) > reprojection_error_thresh 
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
                // 'SLAM_upd:107' else
                // 'SLAM_upd:108' if norm(fp) < 0.1
                if (norm(new_origin_pos_rel) < 0.1) {
                  //  feature triangulated very close
                  // 'SLAM_upd:109' fp = m_l;
                  for (i51 = 0; i51 < 3; i51++) {
                    new_origin_pos_rel[i51] = b_m[i51];
                  }

                  // 'SLAM_upd:110' success = false;
                  success = false;

                  // 'SLAM_upd:111' ros_warn('Feature %i is triangulated very close. Depth: %f', int32(ind_r(i)), norm(fp)); 
                  f_ros_warn((int)ind_r_data[i], norm(*(double (*)[3])&b_m[0]));
                }
              }
            }

            if (guard1) {
              //                          ros_info('Bad triangulation (reprojection error) for point %d', int8(ind_r(i))); 
              // 'SLAM_upd:105' fp = m_l;
              for (i51 = 0; i51 < 3; i51++) {
                new_origin_pos_rel[i51] = b_m[i51];
              }

              // 'SLAM_upd:106' success = false;
              success = false;
            }
          }
        } else {
          // 'SLAM_upd:115' else
          // 'SLAM_upd:116' fp = m_l;
          for (i51 = 0; i51 < 3; i51++) {
            new_origin_pos_rel[i51] = b_m[i51];
          }
        }
      } else {
        // 'SLAM_upd:118' else
        //  mono
        // 'SLAM_upd:119' z_n_l = [(z_curr_l(1) - cameraParams.CameraParameters1.PrincipalPoint(1))/cameraParams.CameraParameters1.FocalLength(1); 
        // 'SLAM_upd:120'                 (z_curr_l(2) - cameraParams.CameraParameters1.PrincipalPoint(2))/cameraParams.CameraParameters1.FocalLength(2)]; 
        // 'SLAM_upd:121' m_l = [z_n_l; 1];
        m_l[0] = (z_all_l[((int)ind_r_data[i] - 1) * 2] -
                  e_cameraParams_CameraParameters[0]) /
          d_cameraParams_CameraParameters[0];
        m_l[1] = (z_all_l[((int)ind_r_data[i] - 1) * 2 + 1] -
                  e_cameraParams_CameraParameters[1]) /
          d_cameraParams_CameraParameters[1];
        m_l[2] = 1.0;

        // 'SLAM_upd:122' fp = m_l/norm(m_l);
        mtmp = norm(m_l);
        for (idx = 0; idx < 3; idx++) {
          new_origin_pos_rel[idx] = m_l[idx] / mtmp;
        }

        // 'SLAM_upd:123' success = true;
        success = true;
      }

      // 'SLAM_upd:125' new_depths(i) = norm(fp);
      ind_l2_data[i] = norm(new_origin_pos_rel);

      // 'SLAM_upd:126' new_m(:, i) = m_l;
      for (i51 = 0; i51 < 3; i51++) {
        new_m_data[i51 + 3 * i] = m_l[i51];
      }

      // 'SLAM_upd:127' triangulation_success(i) = success;
      triangulation_success_data[i] = success;
    }

    // 'SLAM_upd:130' ros_info('Successfully triangulated %d of %d features', int32(nnz(triangulation_success)), int32(length(triangulation_success))) 
    ixstart = 0;
    for (k = 0; k < ii_size_idx_0; k++) {
      if (triangulation_success_data[k]) {
        ixstart++;
      }
    }

    d_ros_info(ixstart, loop_ub);

    // 'SLAM_upd:132' triangulated_depths = new_depths(triangulation_success);
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

    // 'SLAM_upd:133' [triangulated_depths, triangulated_idx] = sort(triangulated_depths, 'ascend'); 
    i51 = qualities->size[0];
    qualities->size[0] = trueCount;
    emxEnsureCapacity((emxArray__common *)qualities, i51, (int)sizeof(double));
    for (i51 = 0; i51 < trueCount; i51++) {
      qualities->data[i51] = triangulated_depths_data[i51];
    }

    c_sort(qualities, b_iidx);
    i51 = anchorInd->size[0];
    anchorInd->size[0] = b_iidx->size[0];
    emxEnsureCapacity((emxArray__common *)anchorInd, i51, (int)sizeof(double));
    idx = b_iidx->size[0];
    for (i51 = 0; i51 < idx; i51++) {
      anchorInd->data[i51] = b_iidx->data[i51];
    }

    // 'SLAM_upd:134' triangulated_m = new_m(:, triangulation_success);
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

    // 'SLAM_upd:135' triangulated_m = triangulated_m(:, triangulated_idx);
    // 'SLAM_upd:136' triangulated_status_ind = ind_r(triangulation_success);
    idx = loop_ub - 1;
    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        triangulated_status_ind_data[ixstart] = (signed char)ind_r_data[i];
        ixstart++;
      }
    }

    // 'SLAM_upd:137' triangulated_status_ind = triangulated_status_ind(triangulated_idx); 
    c_triangulated_status_ind_size_ = anchorInd->size[0];
    idx = anchorInd->size[0];
    for (i51 = 0; i51 < idx; i51++) {
      b_triangulated_status_ind_data[i51] = triangulated_status_ind_data[(int)
        anchorInd->data[i51] - 1];
    }

    for (i51 = 0; i51 < c_triangulated_status_ind_size_; i51++) {
      triangulated_status_ind_data[i51] = b_triangulated_status_ind_data[i51];
    }

    // 'SLAM_upd:139' untriangulated_depths = new_depths(~triangulation_success); 
    idx = loop_ub - 1;
    k = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        k++;
      }
    }

    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        untriangulated_depths_data[ixstart] = ind_l2_data[i];
        ixstart++;
      }
    }

    // 'SLAM_upd:140' [untriangulated_depths, untriangulated_idx] = sort(untriangulated_depths, 'ascend'); 
    i51 = featureInd->size[0];
    featureInd->size[0] = k;
    emxEnsureCapacity((emxArray__common *)featureInd, i51, (int)sizeof(double));
    for (i51 = 0; i51 < k; i51++) {
      featureInd->data[i51] = untriangulated_depths_data[i51];
    }

    emxInit_real_T(&untriangulated_idx, 1);
    c_sort(featureInd, b_iidx);
    i51 = untriangulated_idx->size[0];
    untriangulated_idx->size[0] = b_iidx->size[0];
    emxEnsureCapacity((emxArray__common *)untriangulated_idx, i51, (int)sizeof
                      (double));
    idx = b_iidx->size[0];
    for (i51 = 0; i51 < idx; i51++) {
      untriangulated_idx->data[i51] = b_iidx->data[i51];
    }

    // 'SLAM_upd:141' untriangulated_m = new_m(:, ~triangulation_success);
    idx = loop_ub - 1;
    k = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        k++;
      }
    }

    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        tmp_data[ixstart] = i + 1;
        ixstart++;
      }
    }

    // 'SLAM_upd:142' untriangulated_m = untriangulated_m(:, untriangulated_idx); 
    // 'SLAM_upd:143' untriangulated_status_ind = ind_r(~triangulation_success); 
    idx = loop_ub - 1;
    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        untriangulated_status_ind_data[ixstart] = (signed char)ind_r_data[i];
        ixstart++;
      }
    }

    // 'SLAM_upd:144' untriangulated_status_ind = untriangulated_status_ind(untriangulated_idx); 
    idx = untriangulated_idx->size[0];
    loop_ub = untriangulated_idx->size[0];
    for (i51 = 0; i51 < loop_ub; i51++) {
      b_triangulated_status_ind_data[i51] = untriangulated_status_ind_data[(int)
        untriangulated_idx->data[i51] - 1];
    }

    for (i51 = 0; i51 < idx; i51++) {
      untriangulated_status_ind_data[i51] = b_triangulated_status_ind_data[i51];
    }

    // 'SLAM_upd:146' new_depths = [triangulated_depths; untriangulated_depths]; 
    ind_l2_size[0] = qualities->size[0] + featureInd->size[0];
    loop_ub = qualities->size[0];
    for (i51 = 0; i51 < loop_ub; i51++) {
      ind_l2_data[i51] = qualities->data[i51];
    }

    loop_ub = featureInd->size[0];
    for (i51 = 0; i51 < loop_ub; i51++) {
      ind_l2_data[i51 + qualities->size[0]] = featureInd->data[i51];
    }

    // 'SLAM_upd:147' new_m = [triangulated_m, untriangulated_m];
    for (i51 = 0; i51 < trueCount; i51++) {
      for (i = 0; i < 3; i++) {
        b_new_m_data[i + 3 * i51] = new_m_data[i + 3 * (ii_data[i51] - 1)];
      }
    }

    for (i51 = 0; i51 < k; i51++) {
      for (i = 0; i < 3; i++) {
        c_new_m_data[i + 3 * i51] = new_m_data[i + 3 * (tmp_data[i51] - 1)];
      }
    }

    loop_ub = anchorInd->size[0];
    for (i51 = 0; i51 < loop_ub; i51++) {
      for (i = 0; i < 3; i++) {
        new_m_data[i + 3 * i51] = b_new_m_data[i + 3 * ((int)anchorInd->data[i51]
          - 1)];
      }
    }

    loop_ub = untriangulated_idx->size[0];
    for (i51 = 0; i51 < loop_ub; i51++) {
      for (i = 0; i < 3; i++) {
        new_m_data[i + 3 * (i51 + anchorInd->size[0])] = c_new_m_data[i + 3 *
          ((int)untriangulated_idx->data[i51] - 1)];
      }
    }

    emxFree_real_T(&untriangulated_idx);

    // 'SLAM_upd:148' status_ind = [triangulated_status_ind; untriangulated_status_ind]; 
    for (i51 = 0; i51 < c_triangulated_status_ind_size_; i51++) {
      status_ind_data[i51] = triangulated_status_ind_data[i51];
    }

    for (i51 = 0; i51 < idx; i51++) {
      status_ind_data[i51 + c_triangulated_status_ind_size_] =
        untriangulated_status_ind_data[i51];
    }

    //  insert new features into the state. first the closest triangulated ones, 
    //  then the un-triangulated ones
    // 'SLAM_upd:152' new_feature_idx = 1;
    new_feature_idx = 0;

    // 'SLAM_upd:153' for anchorIdx = 1:numAnchors
    anchorIdx = 0;
    exitg4 = false;
    while ((!exitg4) && (anchorIdx + 1 < 9)) {
      //          if new_feature_idx > length(new_depths)
      // 'SLAM_upd:155' if new_feature_idx > nnz(triangulation_success)
      ixstart = 0;
      for (k = 0; k < ii_size_idx_0; k++) {
        if (triangulation_success_data[k]) {
          ixstart++;
        }
      }

      if ((new_feature_idx + 1 > ixstart) || (ind_l2_size[0] - new_feature_idx <
           3)) {
        exitg4 = true;
      } else {
        // 'SLAM_upd:158' if length(new_depths) - (new_feature_idx -1) < minFeatureThreshold 
        // 'SLAM_upd:161' if getNumValidFeatures(xt.anchor_states(anchorIdx)) < minFeatureThreshold 
        if (getNumValidFeatures(b_xt->anchor_states[anchorIdx].feature_states) <
            3.0) {
          //  anchor needs to be initialized
          //  free up updateVect
          // 'SLAM_upd:164' for featureIdx = 1:numPointsPerAnchor
          for (featureIdx = 0; featureIdx < 5; featureIdx++) {
            // 'SLAM_upd:165' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status 
            if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status
                != 0) {
              //                      ros_info('clearing up feature %i (%i on %i)', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx, featureIdx, anchorIdx) 
              // 'SLAM_upd:167' updateVect(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
              updateVect[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = 0;

              // 'SLAM_upd:168' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                status_idx = 0;

              // 'SLAM_upd:169' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                0;
            }
          }

          // 'SLAM_upd:173' if xt.origin.anchor_idx == anchorIdx
          if (b_xt->origin.anchor_idx == anchorIdx + 1) {
            // 'SLAM_upd:174' xt.origin.anchor_idx = int32(0);
            b_xt->origin.anchor_idx = 0;

            // 'SLAM_upd:175' ros_info('Initializing anchor %i, which was the origin anchor', int32(anchorIdx)) 
            e_ros_info(anchorIdx + 1);
          }

          // 'SLAM_upd:178' xt.anchor_states(anchorIdx).pos = xt.robot_state.pos; 
          for (i51 = 0; i51 < 3; i51++) {
            b_xt->anchor_states[anchorIdx].pos[i51] = b_xt->robot_state.pos[i51];
          }

          // 'SLAM_upd:179' xt.anchor_states(anchorIdx).att = xt.robot_state.att; 
          for (i51 = 0; i51 < 4; i51++) {
            b_xt->anchor_states[anchorIdx].att[i51] = b_xt->robot_state.att[i51];
          }

          // 'SLAM_upd:180' xt.anchor_states(anchorIdx).P_idx = numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6); 
          i51 = anchorIdx * 11;
          for (i = 0; i < 6; i++) {
            b_xt->anchor_states[anchorIdx].P_idx[i] = (i + i51) + 22;
          }

          // 'SLAM_upd:182' P_apo(xt.anchor_states(anchorIdx).P_idx, :) = 0;
          for (i51 = 0; i51 < 109; i51++) {
            for (i = 0; i < 6; i++) {
              P_apr[(b_xt->anchor_states[anchorIdx].P_idx[i] + 109 * i51) - 1] =
                0.0;
            }
          }

          // 'SLAM_upd:183' P_apo(:, xt.anchor_states(anchorIdx).P_idx) = 0;
          for (i51 = 0; i51 < 6; i51++) {
            for (i = 0; i < 109; i++) {
              P_apr[i + 109 * (b_xt->anchor_states[anchorIdx].P_idx[i51] - 1)] =
                0.0;
            }
          }

          // 'SLAM_upd:184' P_apo(xt.anchor_states(anchorIdx).P_idx(end) + (1:numPointsPerAnchor), :) = 0; 
          idx = b_xt->anchor_states[anchorIdx].P_idx[5];
          for (i51 = 0; i51 < 109; i51++) {
            for (i = 0; i < 5; i++) {
              i53 = (long)idx + (1 + i);
              if (i53 > 2147483647L) {
                i53 = 2147483647L;
              } else {
                if (i53 < -2147483648L) {
                  i53 = -2147483648L;
                }
              }

              P_apr[((int)i53 + 109 * i51) - 1] = 0.0;
            }
          }

          // 'SLAM_upd:185' P_apo(:, xt.anchor_states(anchorIdx).P_idx(end) + (1:numPointsPerAnchor)) = 0; 
          idx = b_xt->anchor_states[anchorIdx].P_idx[5];
          for (i51 = 0; i51 < 5; i51++) {
            for (i = 0; i < 109; i++) {
              i53 = (long)idx + (1 + i51);
              if (i53 > 2147483647L) {
                i53 = 2147483647L;
              } else {
                if (i53 < -2147483648L) {
                  i53 = -2147483648L;
                }
              }

              P_apr[i + 109 * ((int)i53 - 1)] = 0.0;
            }
          }

          //  covariance with robot states
          // 'SLAM_upd:189' P_apo(xt.anchor_states(anchorIdx).P_idx, xt.anchor_states(anchorIdx).P_idx)        = P_apo(1:6, 1:6); 
          for (i51 = 0; i51 < 6; i51++) {
            for (i = 0; i < 6; i++) {
              b_P_apr[i + 6 * i51] = P_apr[i + 109 * i51];
            }
          }

          for (i51 = 0; i51 < 6; i51++) {
            for (i = 0; i < 6; i++) {
              P_apr[(b_xt->anchor_states[anchorIdx].P_idx[i] + 109 *
                     (b_xt->anchor_states[anchorIdx].P_idx[i51] - 1)) - 1] =
                b_P_apr[i + 6 * i51];
            }
          }

          //  anchor position, attitude autocovariance
          // 'SLAM_upd:190' P_apo(xt.anchor_states(anchorIdx).P_idx, 1:6)                                      = P_apo(1:6, 1:6); 
          for (i51 = 0; i51 < 6; i51++) {
            for (i = 0; i < 6; i++) {
              b_P_apr[i + 6 * i51] = P_apr[i + 109 * i51];
            }
          }

          for (i51 = 0; i51 < 6; i51++) {
            for (i = 0; i < 6; i++) {
              P_apr[(b_xt->anchor_states[anchorIdx].P_idx[i] + 109 * i51) - 1] =
                b_P_apr[i + 6 * i51];
            }
          }

          //  anchor position, attitude covariance with robot state
          // 'SLAM_upd:191' P_apo(1:6, xt.anchor_states(anchorIdx).P_idx)                                      = P_apo(1:6, 1:6); 
          for (i51 = 0; i51 < 6; i51++) {
            for (i = 0; i < 6; i++) {
              b_P_apr[i + 6 * i51] = P_apr[i + 109 * i51];
            }
          }

          for (i51 = 0; i51 < 6; i51++) {
            for (i = 0; i < 6; i++) {
              P_apr[i + 109 * (b_xt->anchor_states[anchorIdx].P_idx[i51] - 1)] =
                b_P_apr[i + 6 * i51];
            }
          }

          //  anchor position, attitude covariance with robot state
          // 'SLAM_upd:192' P_apo(int32(7:xt.anchor_states(anchorIdx).P_idx(1)-1), xt.anchor_states(anchorIdx).P_idx) = P_apo(int32(7:xt.anchor_states(anchorIdx).P_idx(1)-1), 1:6); 
          i53 = b_xt->anchor_states[anchorIdx].P_idx[0] - 1L;
          if (i53 > 2147483647L) {
            i53 = 2147483647L;
          } else {
            if (i53 < -2147483648L) {
              i53 = -2147483648L;
            }
          }

          idx = (int)i53;
          if (idx < 7) {
            ixstart = 0;
          } else {
            ixstart = idx - 6;
          }

          if (ixstart > 0) {
            y_data[0] = 7;
            idx = 7;
            for (k = 2; k <= ixstart; k++) {
              idx++;
              y_data[k - 1] = idx;
            }
          }

          for (i51 = 0; i51 < ixstart; i51++) {
            b_tmp_data[i51] = y_data[i51] - 1;
          }

          i53 = b_xt->anchor_states[anchorIdx].P_idx[0] - 1L;
          if (i53 > 2147483647L) {
            i53 = 2147483647L;
          } else {
            if (i53 < -2147483648L) {
              i53 = -2147483648L;
            }
          }

          idx = (int)i53;
          if (idx < 7) {
            ixstart = 0;
          } else {
            ixstart = idx - 6;
          }

          if (ixstart > 0) {
            y_data[0] = 7;
            idx = 7;
            for (k = 2; k <= ixstart; k++) {
              idx++;
              y_data[k - 1] = idx;
            }
          }

          for (i51 = 0; i51 < 6; i51++) {
            for (i = 0; i < ixstart; i++) {
              c_tmp_data[i + ixstart * i51] = P_apr[(y_data[i] + 109 * i51) - 1];
            }
          }

          for (i51 = 0; i51 < 6; i51++) {
            for (i = 0; i < ixstart; i++) {
              P_apr[b_tmp_data[i] + 109 * (b_xt->anchor_states[anchorIdx]
                .P_idx[i51] - 1)] = c_tmp_data[i + ixstart * i51];
            }
          }

          //  covariance with bias states etc
          // 'SLAM_upd:193' P_apo(xt.anchor_states(anchorIdx).P_idx, int32(7:xt.anchor_states(anchorIdx).P_idx(1)-1)) = P_apo(1:6, int32(7:xt.anchor_states(anchorIdx).P_idx(1)-1)); 
          i53 = b_xt->anchor_states[anchorIdx].P_idx[0] - 1L;
          if (i53 > 2147483647L) {
            i53 = 2147483647L;
          } else {
            if (i53 < -2147483648L) {
              i53 = -2147483648L;
            }
          }

          idx = (int)i53;
          if (idx < 7) {
            ixstart = 0;
          } else {
            ixstart = idx - 6;
          }

          if (ixstart > 0) {
            y_data[0] = 7;
            idx = 7;
            for (k = 2; k <= ixstart; k++) {
              idx++;
              y_data[k - 1] = idx;
            }
          }

          for (i51 = 0; i51 < ixstart; i51++) {
            b_tmp_data[i51] = y_data[i51] - 1;
          }

          i53 = b_xt->anchor_states[anchorIdx].P_idx[0] - 1L;
          if (i53 > 2147483647L) {
            i53 = 2147483647L;
          } else {
            if (i53 < -2147483648L) {
              i53 = -2147483648L;
            }
          }

          idx = (int)i53;
          if (idx < 7) {
            ixstart = 0;
          } else {
            ixstart = idx - 6;
          }

          if (ixstart > 0) {
            y_data[0] = 7;
            idx = 7;
            for (k = 2; k <= ixstart; k++) {
              idx++;
              y_data[k - 1] = idx;
            }
          }

          for (i51 = 0; i51 < ixstart; i51++) {
            for (i = 0; i < 6; i++) {
              d_tmp_data[i + 6 * i51] = P_apr[i + 109 * (y_data[i51] - 1)];
            }
          }

          for (i51 = 0; i51 < ixstart; i51++) {
            for (i = 0; i < 6; i++) {
              P_apr[(b_xt->anchor_states[anchorIdx].P_idx[i] + 109 *
                     b_tmp_data[i51]) - 1] = d_tmp_data[i + 6 * i51];
            }
          }

          //  covariance with bias states etc
          //  covariance with other anchor states
          // 'SLAM_upd:195' P_apo(int32(xt.anchor_states(anchorIdx).P_idx(1)+1:int32(end)), xt.anchor_states(anchorIdx).P_idx) = P_apo(int32(xt.anchor_states(anchorIdx).P_idx(1)+1:int32(end)), 1:6); 
          i53 = b_xt->anchor_states[anchorIdx].P_idx[0] + 1L;
          if (i53 > 2147483647L) {
            i53 = 2147483647L;
          } else {
            if (i53 < -2147483648L) {
              i53 = -2147483648L;
            }
          }

          idx = (int)i53;
          ixstart = 110 - idx;
          b_y_data[0] = idx;
          for (k = 2; k <= ixstart; k++) {
            idx++;
            b_y_data[k - 1] = idx;
          }

          for (i51 = 0; i51 < ixstart; i51++) {
            e_tmp_data[i51] = b_y_data[i51] - 1;
          }

          i53 = b_xt->anchor_states[anchorIdx].P_idx[0] + 1L;
          if (i53 > 2147483647L) {
            i53 = 2147483647L;
          } else {
            if (i53 < -2147483648L) {
              i53 = -2147483648L;
            }
          }

          idx = (int)i53;
          ixstart = 110 - idx;
          b_y_data[0] = idx;
          for (k = 2; k <= ixstart; k++) {
            idx++;
            b_y_data[k - 1] = idx;
          }

          for (i51 = 0; i51 < 6; i51++) {
            for (i = 0; i < ixstart; i++) {
              f_tmp_data[i + ixstart * i51] = P_apr[(b_y_data[i] + 109 * i51) -
                1];
            }
          }

          for (i51 = 0; i51 < 6; i51++) {
            for (i = 0; i < ixstart; i++) {
              P_apr[e_tmp_data[i] + 109 * (b_xt->anchor_states[anchorIdx]
                .P_idx[i51] - 1)] = f_tmp_data[i + ixstart * i51];
            }
          }

          // 'SLAM_upd:196' P_apo(xt.anchor_states(anchorIdx).P_idx, int32(xt.anchor_states(anchorIdx).P_idx(1)+1:int32(end))) = P_apo(1:6, int32(xt.anchor_states(anchorIdx).P_idx(1)+1:int32(end))); 
          i53 = b_xt->anchor_states[anchorIdx].P_idx[0] + 1L;
          if (i53 > 2147483647L) {
            i53 = 2147483647L;
          } else {
            if (i53 < -2147483648L) {
              i53 = -2147483648L;
            }
          }

          idx = (int)i53;
          ixstart = 110 - idx;
          b_y_data[0] = idx;
          for (k = 2; k <= ixstart; k++) {
            idx++;
            b_y_data[k - 1] = idx;
          }

          for (i51 = 0; i51 < ixstart; i51++) {
            e_tmp_data[i51] = b_y_data[i51] - 1;
          }

          i53 = b_xt->anchor_states[anchorIdx].P_idx[0] + 1L;
          if (i53 > 2147483647L) {
            i53 = 2147483647L;
          } else {
            if (i53 < -2147483648L) {
              i53 = -2147483648L;
            }
          }

          idx = (int)i53;
          ixstart = 110 - idx;
          b_y_data[0] = idx;
          for (k = 2; k <= ixstart; k++) {
            idx++;
            b_y_data[k - 1] = idx;
          }

          for (i51 = 0; i51 < ixstart; i51++) {
            for (i = 0; i < 6; i++) {
              g_tmp_data[i + 6 * i51] = P_apr[i + 109 * (b_y_data[i51] - 1)];
            }
          }

          for (i51 = 0; i51 < ixstart; i51++) {
            for (i = 0; i < 6; i++) {
              P_apr[(b_xt->anchor_states[anchorIdx].P_idx[i] + 109 *
                     e_tmp_data[i51]) - 1] = g_tmp_data[i + 6 * i51];
            }
          }

          // 'SLAM_upd:198' if coder.target('MATLAB')
          // 'SLAM_upd:215' for featureIdx = 1:numPointsPerAnchor
          featureIdx = 0;
          exitg5 = false;
          while ((!exitg5) && (featureIdx + 1 < 6)) {
            // 'SLAM_upd:216' xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth = 1/new_depths(new_feature_idx); 
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              inverse_depth = 1.0 / ind_l2_data[new_feature_idx];

            // 'SLAM_upd:217' xt.anchor_states(anchorIdx).feature_states(featureIdx).m = new_m(:,new_feature_idx); 
            for (i51 = 0; i51 < 3; i51++) {
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].m[i51] =
                new_m_data[i51 + 3 * new_feature_idx];
            }

            // 'SLAM_upd:218' if VIOParameters.delayed_initialization
            if (b_VIOParameters.delayed_initialization) {
              // 'SLAM_upd:219' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(2); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                2;
            } else {
              // 'SLAM_upd:220' else
              // 'SLAM_upd:221' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(1); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                1;
            }

            // 'SLAM_upd:223' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(status_ind(new_feature_idx)); 
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status_idx
              = (int)status_ind_data[new_feature_idx];

            // 'SLAM_upd:224' xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx = int32(numStates + (anchorIdx-1)*numStatesPerAnchor + 6 + featureIdx); 
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx =
              (anchorIdx * 11 + featureIdx) + 28;

            // 'SLAM_upd:226' if new_feature_idx > nnz(triangulation_success)
            ixstart = 0;
            for (k = 0; k < ii_size_idx_0; k++) {
              if (triangulation_success_data[k]) {
                ixstart++;
              }
            }

            if (new_feature_idx + 1 > ixstart) {
              // 'SLAM_upd:227' ros_info('Feature %d is too far away to triangulate.\n', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) 
              f_ros_info(b_xt->anchor_states[anchorIdx]
                         .feature_states[featureIdx].status_idx);

              // 'SLAM_upd:228' P_apo(xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx, xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx) = noiseParameters.inv_depth_initial_unc*10; 
              P_apr[(b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                     P_idx + 109 * (b_xt->anchor_states[anchorIdx].
                                    feature_states[featureIdx].P_idx - 1)) - 1] =
                c_noiseParameters_inv_depth_ini * 10.0;

              //  TODO: Maybe push the mean value further away?
            } else {
              // 'SLAM_upd:230' else
              // 'SLAM_upd:231' P_apo(xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx, xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx) = noiseParameters.inv_depth_initial_unc; 
              P_apr[(b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                     P_idx + 109 * (b_xt->anchor_states[anchorIdx].
                                    feature_states[featureIdx].P_idx - 1)) - 1] =
                c_noiseParameters_inv_depth_ini;

              // *new_depths(new_feature_idx);
            }

            //                  ros_info('Inserting feature %d as feature %i on anchor %i', int32(status_ind(new_feature_idx)), int32(featureIdx), int32(anchorIdx)) 
            // 'SLAM_upd:236' updateVect(status_ind(new_feature_idx)) = int32(1); 
            updateVect[(int)status_ind_data[new_feature_idx] - 1] = 1;

            // 'SLAM_upd:238' new_feature_idx = new_feature_idx + 1;
            new_feature_idx++;

            //                  if new_feature_idx > length(new_depths)
            // 'SLAM_upd:240' if new_feature_idx > nnz(triangulation_success)
            ixstart = 0;
            for (k = 0; k < ii_size_idx_0; k++) {
              if (triangulation_success_data[k]) {
                ixstart++;
              }
            }

            if (new_feature_idx + 1 > ixstart) {
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
      // 'SLAM_upd:246' elseif ~isempty(ind_r)
      // 'SLAM_upd:247' ros_warn('Got %d new feautures but not enough for a new anchor (min %d)', length(ind_r), int32(minFeatureThreshold)) 
      g_ros_warn((double)ii_size_idx_0);
    }
  }

  // 'SLAM_upd:249' updateVect(updateVect==int32(2)) = int32(0);
  for (i = 0; i < 40; i++) {
    if (updateVect[i] == 2) {
      updateVect[i] = 0;
    }
  }

  //  remove features that were not inserted
  // 'SLAM_upd:251' if VIOParameters.delayed_initialization
  if (b_VIOParameters.delayed_initialization) {
    //  get the median uncertainty of the active features as a benchmark on
    //  the delayed features
    // 'SLAM_upd:254' uncertainties = -1*ones(numTrackFeatures, 1);
    for (i = 0; i < 40; i++) {
      b_uncertainties[i] = -1.0;
    }

    // 'SLAM_upd:255' has_active_features = false;
    has_active_features = false;

    // 'SLAM_upd:256' for anchorIdx = 1:numAnchors
    for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
      // 'SLAM_upd:257' for featureIdx = 1:numPointsPerAnchor
      for (featureIdx = 0; featureIdx < 5; featureIdx++) {
        // 'SLAM_upd:258' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 1 
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
            1) {
          // 'SLAM_upd:259' has_active_features = true;
          has_active_features = true;

          // 'SLAM_upd:260' uncertainties(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = P_apo(xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx, xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx); 
          b_uncertainties[b_xt->anchor_states[anchorIdx]
            .feature_states[featureIdx].status_idx - 1] = P_apr
            [(b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx +
              109 * (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                     P_idx - 1)) - 1];
        }
      }
    }

    // 'SLAM_upd:265' if has_active_features
    if (has_active_features) {
      // 'SLAM_upd:266' median_uncertainty = median(uncertainties(uncertainties > 0), 1); 
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
      for (i51 = 0; i51 < trueCount; i51++) {
        b_ii_data[i51] = b_uncertainties[ii_data[i51] - 1];
      }

      median_uncertainty = median(b_ii_data, uncertainties_size);

      //  because coder does not support nanflag
      //  check if a delayed initialization feature has converged
      // 'SLAM_upd:269' for anchorIdx = 1:numAnchors
      for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
        // 'SLAM_upd:270' for featureIdx = 1:numPointsPerAnchor
        for (featureIdx = 0; featureIdx < 5; featureIdx++) {
          // 'SLAM_upd:271' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 
          if ((b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
               2) && (P_apr[(b_xt->anchor_states[anchorIdx]
                             .feature_states[featureIdx].P_idx + 109 *
                             (b_xt->anchor_states[anchorIdx]
                              .feature_states[featureIdx].P_idx - 1)) - 1] <
                      median_uncertainty * 2.0)) {
            //  this feature is not active yet
            // 'SLAM_upd:272' if P_apo(xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx, xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx) < median_uncertainty*2 
            // 'SLAM_upd:273' if xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth < 0 
            if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0) {
              // 'SLAM_upd:274' ros_warn('Feature %i (%i on anchor %i) converged behind its anchor', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx, featureIdx, anchorIdx) 
              h_ros_warn(b_xt->anchor_states[anchorIdx]
                         .feature_states[featureIdx].status_idx, featureIdx + 1,
                         anchorIdx + 1);

              // 'SLAM_upd:275' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                0;

              // 'SLAM_upd:276' updateVect(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
              updateVect[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = 0;
            } else {
              // 'SLAM_upd:277' else
              //                              ros_info('Feature %i (%i on anchor %i) has converged', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx, featureIdx, anchorIdx); 
              // 'SLAM_upd:279' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(1); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                1;
            }
          }
        }
      }
    }

    //  check if delayed features need to be forced active due to too few active 
    //  features
    // 'SLAM_upd:289' minActiveFeatureRatio = 0.4;
    // 'SLAM_upd:290' totalNumActiveFeatues = getTotalNumActiveFeatures(xt);
    totalNumActiveFeatues = getTotalNumActiveFeatures(b_xt->anchor_states);

    // 'SLAM_upd:291' if totalNumActiveFeatues < minActiveFeatureRatio*single(numTrackFeatures) 
    if (totalNumActiveFeatues < 16.0) {
      //  find the best features and activate them
      // 'SLAM_upd:293' numDelayedFeatures = getTotalNumDelayedFeatures(xt);
      numDelayedFeatures = getTotalNumDelayedFeatures(b_xt->anchor_states);

      // 'SLAM_upd:294' qualities = zeros(numDelayedFeatures, 1);
      i51 = qualities->size[0];
      qualities->size[0] = (int)numDelayedFeatures;
      emxEnsureCapacity((emxArray__common *)qualities, i51, (int)sizeof(double));
      loop_ub = (int)numDelayedFeatures;
      for (i51 = 0; i51 < loop_ub; i51++) {
        qualities->data[i51] = 0.0;
      }

      //  quality measures of each delayed feature
      // 'SLAM_upd:295' anchorInd = qualities;
      i51 = anchorInd->size[0];
      anchorInd->size[0] = (int)numDelayedFeatures;
      emxEnsureCapacity((emxArray__common *)anchorInd, i51, (int)sizeof(double));
      loop_ub = (int)numDelayedFeatures;
      for (i51 = 0; i51 < loop_ub; i51++) {
        anchorInd->data[i51] = 0.0;
      }

      // 'SLAM_upd:296' featureInd = qualities;
      i51 = featureInd->size[0];
      featureInd->size[0] = (int)numDelayedFeatures;
      emxEnsureCapacity((emxArray__common *)featureInd, i51, (int)sizeof(double));
      loop_ub = (int)numDelayedFeatures;
      for (i51 = 0; i51 < loop_ub; i51++) {
        featureInd->data[i51] = 0.0;
      }

      // 'SLAM_upd:298' delayedIdx = 1;
      delayedIdx = 1U;

      // 'SLAM_upd:299' for anchorIdx = 1:numAnchors
      for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
        // 'SLAM_upd:300' for featureIdx = 1:numPointsPerAnchor
        for (featureIdx = 0; featureIdx < 5; featureIdx++) {
          // 'SLAM_upd:301' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              2) {
            // 'SLAM_upd:302' rho_unc = P_apo(xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx, xt.anchor_states(anchorIdx).feature_states(featureIdx).P_idx); 
            // 'SLAM_upd:304' quality = rho_unc/noiseParameters.inv_depth_initial_unc; 
            // 'SLAM_upd:306' qualities(delayedIdx) = quality;
            qualities->data[(int)delayedIdx - 1] = P_apr[(b_xt->
              anchor_states[anchorIdx].feature_states[featureIdx].P_idx + 109 *
              (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx -
               1)) - 1] / c_noiseParameters_inv_depth_ini;

            // 'SLAM_upd:307' anchorInd(delayedIdx) = anchorIdx;
            anchorInd->data[(int)delayedIdx - 1] = (double)anchorIdx + 1.0;

            // 'SLAM_upd:308' featureInd(delayedIdx) = featureIdx;
            featureInd->data[(int)delayedIdx - 1] = (double)featureIdx + 1.0;

            // 'SLAM_upd:310' delayedIdx = delayedIdx + 1;
            delayedIdx++;
          }
        }
      }

      // 'SLAM_upd:315' [~, sortInd] = sort(qualities, 'ascend');
      c_sort(qualities, b_iidx);
      i51 = qualities->size[0];
      qualities->size[0] = b_iidx->size[0];
      emxEnsureCapacity((emxArray__common *)qualities, i51, (int)sizeof(double));
      loop_ub = b_iidx->size[0];
      for (i51 = 0; i51 < loop_ub; i51++) {
        qualities->data[i51] = b_iidx->data[i51];
      }

      // 'SLAM_upd:315' ~
      // 'SLAM_upd:317' numActivatedFeatures = 0;
      numActivatedFeatures = 0.0;

      // 'SLAM_upd:319' for i = 1:length(sortInd)
      i = 0;
      while ((i <= qualities->size[0] - 1) && (!(numActivatedFeatures > 16.0F -
               (float)totalNumActiveFeatues)) && (!(numActivatedFeatures >
               numDelayedFeatures))) {
        // 'SLAM_upd:320' if numActivatedFeatures > ceil(minActiveFeatureRatio*single(numTrackFeatures) - totalNumActiveFeatues) || numActivatedFeatures > numDelayedFeatures 
        // 'SLAM_upd:323' if xt.anchor_states(anchorInd(sortInd(i))).feature_states(featureInd(sortInd(i))).inverse_depth < 0 
        if (b_xt->anchor_states[(int)anchorInd->data[(int)qualities->data[i] - 1]
            - 1].feature_states[(int)featureInd->data[(int)qualities->data[i] -
            1] - 1].inverse_depth < 0.0) {
          // 'SLAM_upd:324' xt.anchor_states(anchorInd(sortInd(i))).feature_states(featureInd(sortInd(i))).status = int32(0); 
          b_xt->anchor_states[(int)anchorInd->data[(int)qualities->data[i] - 1]
            - 1].feature_states[(int)featureInd->data[(int)qualities->data[i] -
            1] - 1].status = 0;

          // 'SLAM_upd:325' updateVect(xt.anchor_states(anchorInd(sortInd(i))).feature_states(featureInd(sortInd(i))).status_idx) = int32(0); 
          updateVect[b_xt->anchor_states[(int)anchorInd->data[(int)
            qualities->data[i] - 1] - 1].feature_states[(int)featureInd->data
            [(int)qualities->data[i] - 1] - 1].status_idx - 1] = 0;

          // 'SLAM_upd:326' ros_warn('Trying to force insert feature %i behind its anchor', xt.anchor_states(anchorInd(sortInd(i))).feature_states(featureInd(sortInd(i))).status_idx) 
          i_ros_warn(b_xt->anchor_states[(int)anchorInd->data[(int)
                     qualities->data[i] - 1] - 1].feature_states[(int)
                     featureInd->data[(int)qualities->data[i] - 1] - 1].
                     status_idx);
        } else {
          // 'SLAM_upd:327' else
          // 'SLAM_upd:328' xt.anchor_states(anchorInd(sortInd(i))).feature_states(featureInd(sortInd(i))).status = int32(1); 
          b_xt->anchor_states[(int)anchorInd->data[(int)qualities->data[i] - 1]
            - 1].feature_states[(int)featureInd->data[(int)qualities->data[i] -
            1] - 1].status = 1;

          // 'SLAM_upd:329' ros_info('Forcing activation of feature %i (%i on anchor %i)', xt.anchor_states(anchorInd(sortInd(i))).feature_states(featureInd(sortInd(i))).status_idx, featureInd(sortInd(i)), anchorInd(sortInd(i))); 
          g_ros_info(b_xt->anchor_states[(int)anchorInd->data[(int)
                     qualities->data[i] - 1] - 1].feature_states[(int)
                     featureInd->data[(int)qualities->data[i] - 1] - 1].
                     status_idx, featureInd->data[(int)qualities->data[i] - 1],
                     anchorInd->data[(int)qualities->data[i] - 1]);

          // 'SLAM_upd:330' numActivatedFeatures = numActivatedFeatures + 1;
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
  // 'SLAM_upd:337' request_new_features = false;
  request_new_features = false;

  // 'SLAM_upd:338' if nnz(updateVect == 0) > minFeatureThreshold
  ixstart = 0;
  for (k = 0; k < 40; k++) {
    if (updateVect[k] == 0) {
      ixstart++;
    }
  }

  if (ixstart > 3) {
    //  if a new anchor can be filled enough
    // 'SLAM_upd:339' for anchorIdx = 1:numAnchors
    anchorIdx = 0;
    exitg3 = false;
    while ((!exitg3) && (anchorIdx + 1 < 9)) {
      // 'SLAM_upd:340' if getNumValidFeatures(xt.anchor_states(anchorIdx)) < minFeatureThreshold 
      if (getNumValidFeatures(b_xt->anchor_states[anchorIdx].feature_states) <
          3.0) {
        //  discard all features of this anchor
        // 'SLAM_upd:342' for featureIdx = 1:numPointsPerAnchor
        for (featureIdx = 0; featureIdx < 5; featureIdx++) {
          // 'SLAM_upd:343' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx 
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              status_idx != 0) {
            // 'SLAM_upd:344' updateVect(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = int32(0); 
            updateVect[b_xt->anchor_states[anchorIdx].feature_states[featureIdx]
              .status_idx - 1] = 0;

            // 'SLAM_upd:345' xt.anchor_states(anchorIdx).feature_states(featureIdx).status = int32(0); 
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status = 0;

            // 'SLAM_upd:346' xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx = int32(0); 
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status_idx
              = 0;
          }
        }

        // 'SLAM_upd:349' request_new_features = true;
        request_new_features = true;
        exitg3 = true;
      } else {
        anchorIdx++;
      }
    }

    //  else % debug check
    //      if coder.target('MATLAB')
    //          for anchorIdx = 1:numAnchors
    //              if getNumValidFeatures(xt.anchor_states(anchorIdx)) < minFeatureThreshold 
    //                  ros_error('anchor %i needs new features but cant ask for them', anchorIdx) 
    //              end
    //          end
    //      end
  }

  // 'SLAM_upd:363' if request_new_features
  if (request_new_features) {
    // 'SLAM_upd:364' request_idx = 1;
    request_idx = 1;

    // 'SLAM_upd:365' for i = 1:length(updateVect)
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i < 40)) {
      // 'SLAM_upd:366' if updateVect(i) == 0
      b_guard1 = false;
      if (updateVect[i] == 0) {
        // 'SLAM_upd:367' updateVect(i) = 2;
        updateVect[i] = 2;

        // 'SLAM_upd:368' request_idx = request_idx +1;
        request_idx++;

        // 'SLAM_upd:369' if request_idx > max_features_to_request
        if (request_idx > 10) {
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

    // 'SLAM_upd:374' ros_info('Requesting %d new features', int32(request_idx-1)) 
    h_ros_info(request_idx - 1);

    //      updateVect(updateVect == 0) = int32(2); % get as many new features as possible 
  }

  // % robocentric update
  // 'SLAM_upd:379' if xt.origin.anchor_idx == 0
  if (b_xt->origin.anchor_idx == 0) {
    //  need to update the origin anchor and the state
    // 'SLAM_upd:380' xt.fixed_feature = int32(0);
    b_xt->fixed_feature = 0;

    //  ensure that a new feature will be fixed, if this option is enabled
    //  choose the best anchor as the new origin anchor
    // 'SLAM_upd:382' uncertainties = zeros(numAnchors, 1);
    //  uncertainties of the anchors reduced to a scalar
    // 'SLAM_upd:383' has_active_features = uncertainties;
    // 'SLAM_upd:384' for anchorIdx = 1:numAnchors
    for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
      b_has_active_features[anchorIdx] = 0.0;

      // 'SLAM_upd:385' if anyActiveAnchorFeatures(xt.anchor_states(anchorIdx))
      if (anyActiveAnchorFeatures(b_xt->anchor_states[anchorIdx].feature_states))
      {
        // 'SLAM_upd:386' uncertainties(anchorIdx) = det(P_apo(xt.anchor_states(anchorIdx).P_idx, xt.anchor_states(anchorIdx).P_idx)); 
        for (i51 = 0; i51 < 6; i51++) {
          for (i = 0; i < 6; i++) {
            b_P_apr[i + 6 * i51] = P_apr[(b_xt->anchor_states[anchorIdx].P_idx[i]
              + 109 * (b_xt->anchor_states[anchorIdx].P_idx[i51] - 1)) - 1];
          }
        }

        c_uncertainties[anchorIdx] = det(b_P_apr);

        // 'SLAM_upd:387' has_active_features(anchorIdx) = 1;
        b_has_active_features[anchorIdx] = 1.0;
      } else {
        // 'SLAM_upd:388' else
        // 'SLAM_upd:389' uncertainties(anchorIdx) = 1000;
        c_uncertainties[anchorIdx] = 1000.0;

        //  dont fix an anchor with no active features
      }
    }

    // 'SLAM_upd:392' if ~any(has_active_features)
    if (!c_any(b_has_active_features)) {
      //  can happen if outlier rejection rejected all features
      // 'SLAM_upd:393' ros_warn('Can''t fix an anchor because none have active features') 
      j_ros_warn();
    } else {
      // 'SLAM_upd:394' else
      // 'SLAM_upd:395' [~, sortIdx] = min(uncertainties);
      ixstart = 1;
      mtmp = c_uncertainties[0];
      idx = 1;
      if (rtIsNaN(c_uncertainties[0])) {
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k < 9)) {
          ixstart = k;
          if (!rtIsNaN(c_uncertainties[k - 1])) {
            mtmp = c_uncertainties[k - 1];
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (ixstart < 8) {
        while (ixstart + 1 < 9) {
          if (c_uncertainties[ixstart] < mtmp) {
            mtmp = c_uncertainties[ixstart];
            idx = ixstart + 1;
          }

          ixstart++;
        }
      }

      // 'SLAM_upd:395' ~
      // 'SLAM_upd:396' xt.origin.anchor_idx = int32(sortIdx(1));
      b_xt->origin.anchor_idx = idx;

      // 'SLAM_upd:397' if ~has_active_features(xt.origin.anchor_idx)
      if (!(b_has_active_features[b_xt->origin.anchor_idx - 1] != 0.0)) {
        //  debug check
        // 'SLAM_upd:398' ros_error('Picked an anchor with no active features as origin (anchor %d)', int32(xt.origin.anchor_idx)) 
        // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
        // 'ros_error:4' if coder.target('MATLAB')
        // 'ros_error:6' elseif ~coder.target('MEX')
        // 'ros_error:7' coder.cinclude('<ros/console.h>')
        // 'ros_error:8' coder.ceval('ROS_ERROR', [str, 0], varargin{:});
        for (i51 = 0; i51 < 63; i51++) {
          cv52[i51] = cv53[i51];
        }

        ROS_ERROR(cv52, b_xt->origin.anchor_idx);
      } else {
        // 'SLAM_upd:399' else
        // 'SLAM_upd:400' ros_info('Setting anchor %i as origin', int32(xt.origin.anchor_idx)) 
        i_ros_info(b_xt->origin.anchor_idx);

        // 'SLAM_upd:402' new_origin_pos_rel = xt.anchor_states(xt.origin.anchor_idx).pos; 
        //  in old origin frame
        // 'SLAM_upd:403' new_origin_att_rel = RotFromQuatJ(xt.anchor_states(xt.origin.anchor_idx).att); 
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
        // 'SLAM_upd:405' J = eye(size(P_apo));
        d_eye(J);

        //  robot position and orientation
        // 'SLAM_upd:407' J(1:6, 1:6) = [new_origin_att_rel, zeros(3); zeros(3), eye(3)]; 
        eye(c_xt);

        // 'SLAM_upd:408' J(1:6, xt.anchor_states(xt.origin.anchor_idx).P_idx) = [-new_origin_att_rel, skew(new_origin_att_rel * (xt.robot_state.pos - new_origin_pos_rel)); zeros(3), -new_origin_att_rel']; 
        for (i = 0; i < 3; i++) {
          new_origin_pos_rel[i] = b_xt->anchor_states[b_xt->origin.anchor_idx -
            1].pos[i];
          for (i51 = 0; i51 < 3; i51++) {
            J[i51 + 109 * i] = new_origin_att_rel[i51 + 3 * i];
            J[i51 + 109 * (i + 3)] = 0.0;
            J[(i51 + 109 * i) + 3] = 0.0;
            J[(i51 + 109 * (i + 3)) + 3] = c_xt[i51 + 3 * i];
          }

          d_xt[i] = b_xt->robot_state.pos[i] - b_xt->anchor_states
            [b_xt->origin.anchor_idx - 1].pos[i];
        }

        // 'skew:2' R=[0,-w(3),w(2);
        // 'skew:3'     w(3),0,-w(1);
        // 'skew:4'     -w(2),w(1),0];
        for (i51 = 0; i51 < 3; i51++) {
          m_l[i51] = 0.0;
          for (i = 0; i < 3; i++) {
            J[i + 109 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
                         .P_idx[i51] - 1)] = -new_origin_att_rel[i + 3 * i51];
            J[(i + 109 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
                          .P_idx[i51] - 1)) + 3] = 0.0;
            m_l[i51] += new_origin_att_rel[i51 + 3 * i] * d_xt[i];
          }
        }

        J[109 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1)]
          = 0.0;
        J[109 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1)]
          = -m_l[2];
        J[109 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1)]
          = m_l[1];
        J[1 + 109 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] -
                     1)] = m_l[2];
        J[1 + 109 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] -
                     1)] = 0.0;
        J[1 + 109 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] -
                     1)] = -m_l[0];
        J[2 + 109 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] -
                     1)] = -m_l[1];
        J[2 + 109 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] -
                     1)] = m_l[0];
        J[2 + 109 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] -
                     1)] = 0.0;

        //  robot velocity
        // 'SLAM_upd:410' J(7:9, 7:9) = new_origin_att_rel;
        //  velocity
        //  origin rotation
        // 'SLAM_upd:412' J(16:18, 16:18) = new_origin_att_rel;
        for (i51 = 0; i51 < 3; i51++) {
          for (i = 0; i < 3; i++) {
            J[(i + 109 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
                          .P_idx[i51 + 3] - 1)) + 3] = -new_origin_att_rel[i51 +
              3 * i];
            J[(i + 109 * (6 + i51)) + 6] = new_origin_att_rel[i + 3 * i51];
            J[(i + 109 * (15 + i51)) + 15] = new_origin_att_rel[i + 3 * i51];
          }
        }

        //  origin rotation
        // 'SLAM_upd:414' for anchorIdx = 1:numAnchors
        for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
          // 'SLAM_upd:415' if anchorIdx == xt.origin.anchor_idx
          if (anchorIdx + 1 == b_xt->origin.anchor_idx) {
            //  remove yaw uncertainty, but not pitch or roll
            // 'SLAM_upd:417' J(xt.anchor_states(anchorIdx).P_idx, xt.anchor_states(anchorIdx).P_idx) = [zeros(3), zeros(3); zeros(3), zeros(3)]; 
            for (i51 = 0; i51 < 6; i51++) {
              for (i = 0; i < 6; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i] + 109 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i51] - 1)) - 1] = 0.0;
              }
            }

            //  TODO: allow roll/pitch uncertainty
          } else {
            // 'SLAM_upd:418' else
            // 'SLAM_upd:419' J(xt.anchor_states(anchorIdx).P_idx, xt.anchor_states(anchorIdx).P_idx) = [new_origin_att_rel, zeros(3); zeros(3), eye(3)]; 
            eye(c_xt);
            for (i51 = 0; i51 < 3; i51++) {
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i] + 109 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i51] - 1)) - 1] =
                  new_origin_att_rel[i + 3 * i51];
              }
            }

            for (i51 = 0; i51 < 3; i51++) {
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i] + 109 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i51 + 3] - 1)) - 1] =
                  0.0;
              }
            }

            for (i51 = 0; i51 < 3; i51++) {
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i + 3] + 109 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i51] - 1)) - 1] = 0.0;
              }
            }

            // 'SLAM_upd:420' J(xt.anchor_states(anchorIdx).P_idx, xt.anchor_states(xt.origin.anchor_idx).P_idx) = [-new_origin_att_rel, skew(new_origin_att_rel * (xt.anchor_states(anchorIdx).pos - new_origin_pos_rel)); zeros(3), -new_origin_att_rel']; 
            for (i51 = 0; i51 < 3; i51++) {
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i + 3] + 109 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i51 + 3] - 1)) - 1] =
                  c_xt[i + 3 * i51];
              }

              d_xt[i51] = b_xt->anchor_states[anchorIdx].pos[i51] -
                new_origin_pos_rel[i51];
            }

            // 'skew:2' R=[0,-w(3),w(2);
            // 'skew:3'     w(3),0,-w(1);
            // 'skew:4'     -w(2),w(1),0];
            for (i51 = 0; i51 < 3; i51++) {
              m_l[i51] = 0.0;
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i] + 109 *
                   (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[i51]
                    - 1)) - 1] = -new_origin_att_rel[i + 3 * i51];
                m_l[i51] += new_origin_att_rel[i51 + 3 * i] * d_xt[i];
              }
            }

            J[(b_xt->anchor_states[anchorIdx].P_idx[0] + 109 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1))
              - 1] = 0.0;
            J[(b_xt->anchor_states[anchorIdx].P_idx[0] + 109 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1))
              - 1] = -m_l[2];
            J[(b_xt->anchor_states[anchorIdx].P_idx[0] + 109 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1))
              - 1] = m_l[1];
            J[(b_xt->anchor_states[anchorIdx].P_idx[1] + 109 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1))
              - 1] = m_l[2];
            J[(b_xt->anchor_states[anchorIdx].P_idx[1] + 109 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1))
              - 1] = 0.0;
            J[(b_xt->anchor_states[anchorIdx].P_idx[1] + 109 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1))
              - 1] = -m_l[0];
            J[(b_xt->anchor_states[anchorIdx].P_idx[2] + 109 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1))
              - 1] = -m_l[1];
            J[(b_xt->anchor_states[anchorIdx].P_idx[2] + 109 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1))
              - 1] = m_l[0];
            J[(b_xt->anchor_states[anchorIdx].P_idx[2] + 109 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1))
              - 1] = 0.0;
            for (i51 = 0; i51 < 3; i51++) {
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i + 3] + 109 *
                   (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[i51]
                    - 1)) - 1] = 0.0;
              }
            }

            for (i51 = 0; i51 < 3; i51++) {
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i + 3] + 109 *
                   (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[i51 +
                    3] - 1)) - 1] = -new_origin_att_rel[i51 + 3 * i];
              }
            }
          }

          // 'SLAM_upd:423' xt.anchor_states(anchorIdx).pos = new_origin_att_rel * (xt.anchor_states(anchorIdx).pos - new_origin_pos_rel); 
          for (i = 0; i < 3; i++) {
            m_l[i] = b_xt->anchor_states[anchorIdx].pos[i] -
              new_origin_pos_rel[i];
            b_xt->anchor_states[anchorIdx].pos[i] = 0.0;
          }

          // 'SLAM_upd:424' xt.anchor_states(anchorIdx).att = QuatFromRotJ(RotFromQuatJ(xt.anchor_states(anchorIdx).att) * new_origin_att_rel'); 
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
          for (i51 = 0; i51 < 3; i51++) {
            b_xt->anchor_states[anchorIdx].pos[i51] = 0.0;
            for (i = 0; i < 3; i++) {
              b_xt->anchor_states[anchorIdx].pos[i51] += new_origin_att_rel[i51
                + 3 * i] * m_l[i];
              c_xt[i51 + 3 * i] = 0.0;
              for (idx = 0; idx < 3; idx++) {
                c_xt[i51 + 3 * i] += e_xt[i51 + 3 * idx] * new_origin_att_rel[i
                  + 3 * idx];
              }
            }
          }

          QuatFromRotJ(c_xt, b_xt->anchor_states[anchorIdx].att);
        }

        // 'SLAM_upd:427' P_apo = J * P_apo * J';
        for (i51 = 0; i51 < 109; i51++) {
          for (i = 0; i < 109; i++) {
            b_J[i51 + 109 * i] = 0.0;
            for (idx = 0; idx < 109; idx++) {
              b_J[i51 + 109 * i] += J[i51 + 109 * idx] * P_apr[idx + 109 * i];
            }
          }
        }

        for (i51 = 0; i51 < 109; i51++) {
          for (i = 0; i < 109; i++) {
            P_apr[i51 + 109 * i] = 0.0;
            for (idx = 0; idx < 109; idx++) {
              P_apr[i51 + 109 * i] += b_J[i51 + 109 * idx] * J[i + 109 * idx];
            }
          }
        }

        // 'SLAM_upd:429' xt.robot_state.pos = new_origin_att_rel * (xt.robot_state.pos - new_origin_pos_rel); 
        for (i51 = 0; i51 < 3; i51++) {
          d_xt[i51] = b_xt->robot_state.pos[i51] - new_origin_pos_rel[i51];
        }

        // 'SLAM_upd:430' xt.robot_state.att = QuatFromRotJ(RotFromQuatJ(xt.robot_state.att) * new_origin_att_rel'); 
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
        for (i51 = 0; i51 < 3; i51++) {
          b_xt->robot_state.pos[i51] = 0.0;
          for (i = 0; i < 3; i++) {
            b_xt->robot_state.pos[i51] += new_origin_att_rel[i51 + 3 * i] *
              d_xt[i];
            c_xt[i51 + 3 * i] = 0.0;
            for (idx = 0; idx < 3; idx++) {
              c_xt[i51 + 3 * i] += f_xt[i51 + 3 * idx] * new_origin_att_rel[i +
                3 * idx];
            }
          }
        }

        QuatFromRotJ(c_xt, b_xt->robot_state.att);

        // 'SLAM_upd:431' xt.robot_state.vel = new_origin_att_rel * xt.robot_state.vel; 
        for (i51 = 0; i51 < 3; i51++) {
          d_xt[i51] = 0.0;
          for (i = 0; i < 3; i++) {
            d_xt[i51] += new_origin_att_rel[i51 + 3 * i] * b_xt->
              robot_state.vel[i];
          }
        }

        // 'SLAM_upd:433' xt.origin.pos = xt.origin.pos + RotFromQuatJ(xt.origin.att)' * new_origin_pos_rel; 
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
        // 'SLAM_upd:434' xt.origin.att = QuatFromRotJ(new_origin_att_rel * RotFromQuatJ(xt.origin.att)); 
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
        for (i51 = 0; i51 < 3; i51++) {
          b_xt->robot_state.vel[i51] = d_xt[i51];
          mtmp = 0.0;
          for (i = 0; i < 3; i++) {
            mtmp += g_xt[i51 + 3 * i] * new_origin_pos_rel[i];
            c_xt[i51 + 3 * i] = 0.0;
            for (idx = 0; idx < 3; idx++) {
              c_xt[i51 + 3 * i] += new_origin_att_rel[i51 + 3 * idx] * h_xt[idx
                + 3 * i];
            }
          }

          b_xt->origin.pos[i51] += mtmp;
        }

        QuatFromRotJ(c_xt, b_xt->origin.att);

        //  in world frame
      }
    }
  }

  // % aposteriori measurement prediction
  // 'SLAM_upd:440' [map] = getMap(xt);
  getMap(b_xt->origin.pos, b_xt->origin.att, b_xt->anchor_states, b_map);

  //  get map for output
  //  xt = getScaledMap(xt); % update the scaled map for measurement prediction
  //  h_u_apo = zeros(numTrackFeatures*4,1);
  // 'SLAM_upd:445' delayedStatus = zeros(size(updateVect));
  memset(&b_delayedStatus[0], 0, 40U * sizeof(double));

  // 'SLAM_upd:446' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
    // 'SLAM_upd:447' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 5; featureIdx++) {
      // 'SLAM_upd:448' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status 
      if ((b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status != 0)
          && (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              2)) {
        //              [h_u_l, h_u_r] = predictMeasurementStereoDistorted(xt.anchor_states(anchorIdx).feature_states(featureIdx).scaled_map_point, cameraParams); 
        //              h_u_apo((xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx - 1)*measDim + int32(1:measDim)) = [h_u_l; h_u_r]; 
        // 'SLAM_upd:451' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 
        // 'SLAM_upd:452' delayedStatus(xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx) = 1; 
        b_delayedStatus[b_xt->anchor_states[anchorIdx].feature_states[featureIdx]
          .status_idx - 1] = 1.0;
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
// Arguments    : const e_struct_T anchor_state_feature_states[5]
// Return Type  : boolean_T
//
static boolean_T anyActiveAnchorFeatures(const e_struct_T
  anchor_state_feature_states[5])
{
  boolean_T ret;
  int featureIdx;
  boolean_T exitg1;

  // 'anyActiveAnchorFeatures:3' ret = false;
  ret = false;

  // 'anyActiveAnchorFeatures:4' for featureIdx = 1:numPointsPerAnchor
  featureIdx = 1;
  exitg1 = false;
  while ((!exitg1) && (featureIdx < 6)) {
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
//                const f_struct_T xt_anchor_states[8]
//                const double z_all_l[80]
//                const double z_all_r[80]
//                const boolean_T b_status[40]
//                const double c_stereoParams_CameraParameters[2]
//                const double d_stereoParams_CameraParameters[2]
//                const double e_stereoParams_CameraParameters[2]
//                const double f_stereoParams_CameraParameters[2]
//                const double stereoParams_r_lr[3]
//                const double stereoParams_R_rl[9]
//                boolean_T VIOParameters_full_stereo
//                emxArray_real_T *r
//                emxArray_real_T *H
// Return Type  : void
//
static void b_getH_R_res(const double xt_robot_state_pos[3], const double
  xt_robot_state_att[4], const f_struct_T xt_anchor_states[8], const double
  z_all_l[80], const double z_all_r[80], const boolean_T b_status[40], const
  double c_stereoParams_CameraParameters[2], const double
  d_stereoParams_CameraParameters[2], const double
  e_stereoParams_CameraParameters[2], const double
  f_stereoParams_CameraParameters[2], const double stereoParams_r_lr[3], const
  double stereoParams_R_rl[9], boolean_T VIOParameters_full_stereo,
  emxArray_real_T *r, emxArray_real_T *H)
{
  double fx_l;
  double fy_l;
  double fx_r;
  double fy_r;
  int nm1d2;
  int cdiff;
  int residualDim;
  double R_cw[9];
  int i20;
  unsigned int res_idx;
  int anchorIdx;
  double anchorRot[9];
  int featureIdx;
  long i21;
  double z_curr_l[2];
  double z_curr_r[2];
  double fp_l[3];
  double fp_r[3];
  double d5;
  double B;
  double h_ci_r[3];
  double h_u_r[2];
  double h_u_l[2];
  int ndbl;
  int apnd;
  double y_data[4];
  int tmp_data[4];
  double h_u_l_To_h_ci_l[6];
  double h_u_r_To_h_ci_r[6];
  double fp_r_norm_inv;
  double fp_r_32_inv;
  double h_ci_r_To_fp_r[9];
  double h_ci_l_To_r_wc[9];
  double h_ci_l_To_R_cw[9];
  double fp_l_To_r_wc[9];
  double fp_l_To_R_cw[9];
  double h_ci_l_To_anchorPos[9];
  double h_ci_l_To_anchorRot[9];
  double b_xt_anchor_states[3];
  double fp_l_To_anchorPos[9];
  double fp_l_To_anchorRot[9];
  double b_R_cw[9];
  double c_R_cw[9];
  double dv2[9];
  double dv3[9];
  int b_tmp_data[4];
  double b_h_u_r_To_h_ci_r[6];
  double c_h_u_r_To_h_ci_r[6];
  double b_h_u_l_To_h_ci_l[6];
  double d_h_u_r_To_h_ci_r[6];
  double c_h_u_l_To_h_ci_l[12];
  double d_h_u_l_To_h_ci_l[2];
  double e_h_u_r_To_h_ci_r[2];

  // 'getH_R_res:26' fx_l = stereoParams.CameraParameters1.FocalLength(1);
  fx_l = c_stereoParams_CameraParameters[0];

  // 'getH_R_res:27' fy_l = stereoParams.CameraParameters1.FocalLength(2);
  fy_l = c_stereoParams_CameraParameters[1];

  // 'getH_R_res:29' fx_r = stereoParams.CameraParameters2.FocalLength(1);
  fx_r = e_stereoParams_CameraParameters[0];

  // 'getH_R_res:30' fy_r = stereoParams.CameraParameters2.FocalLength(2);
  fy_r = e_stereoParams_CameraParameters[1];

  // 'getH_R_res:32' numErrorStatesPerAnchor = 6 + numPointsPerAnchor;
  // 'getH_R_res:34' numMeas = nnz(status);
  nm1d2 = 0;
  for (cdiff = 0; cdiff < 40; cdiff++) {
    if (b_status[cdiff]) {
      nm1d2++;
    }
  }

  // 'getH_R_res:35' if VIOParameters.full_stereo
  if (VIOParameters_full_stereo) {
    // 'getH_R_res:36' residualDim = 4;
    residualDim = 4;
  } else {
    // 'getH_R_res:37' else
    // 'getH_R_res:38' residualDim = 2;
    residualDim = 2;
  }

  // 'getH_R_res:41' R_cw = RotFromQuatJ(xt.robot_state.att);
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

  // 'getH_R_res:42' r_wc = xt.robot_state.pos;
  // 'getH_R_res:44' H = zeros(numMeas*residualDim, numStates + numAnchors*numErrorStatesPerAnchor); 
  i20 = H->size[0] * H->size[1];
  H->size[0] = (int)((double)nm1d2 * (double)residualDim);
  H->size[1] = 109;
  emxEnsureCapacity((emxArray__common *)H, i20, (int)sizeof(double));
  cdiff = (int)((double)nm1d2 * (double)residualDim) * 109;
  for (i20 = 0; i20 < cdiff; i20++) {
    H->data[i20] = 0.0;
  }

  //  z = zeros(numMeas*residualDim, 1);
  //  h_u = zeros(numMeas*residualDim, 1);
  // 'getH_R_res:47' r = zeros(numMeas*residualDim, 1);
  i20 = r->size[0];
  r->size[0] = (int)((double)nm1d2 * (double)residualDim);
  emxEnsureCapacity((emxArray__common *)r, i20, (int)sizeof(double));
  cdiff = (int)((double)nm1d2 * (double)residualDim);
  for (i20 = 0; i20 < cdiff; i20++) {
    r->data[i20] = 0.0;
  }

  // 'getH_R_res:49' ind = int32(zeros(numMeas, 2));
  // 'getH_R_res:51' res_idx = 1;
  res_idx = 1U;

  // 'getH_R_res:53' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
    // 'getH_R_res:54' anchorPos = xt.anchor_states(anchorIdx).pos;
    // 'getH_R_res:55' anchorRot = RotFromQuatJ(xt.anchor_states(anchorIdx).att); 
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

    // 'getH_R_res:57' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 5; featureIdx++) {
      // 'getH_R_res:59' idx = xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx; 
      // 'getH_R_res:61' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status && status(idx) 
      if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status != 0) &&
          b_status[xt_anchor_states[anchorIdx].feature_states[featureIdx].
          status_idx - 1]) {
        // 'getH_R_res:62' z_curr_l = z_all_l((idx-1)*2 + int32(1:2));
        i21 = xt_anchor_states[anchorIdx].feature_states[featureIdx].status_idx
          - 1L;
        if (i21 > 2147483647L) {
          i21 = 2147483647L;
        } else {
          if (i21 < -2147483648L) {
            i21 = -2147483648L;
          }
        }

        i20 = (int)i21;
        if (i20 > 1073741823) {
          cdiff = MAX_int32_T;
        } else if (i20 <= -1073741824) {
          cdiff = MIN_int32_T;
        } else {
          cdiff = i20 << 1;
        }

        // 'getH_R_res:63' z_curr_r = z_all_r((idx-1)*2 + int32(1:2));
        i21 = xt_anchor_states[anchorIdx].feature_states[featureIdx].status_idx
          - 1L;
        if (i21 > 2147483647L) {
          i21 = 2147483647L;
        } else {
          if (i21 < -2147483648L) {
            i21 = -2147483648L;
          }
        }

        i20 = (int)i21;
        if (i20 > 1073741823) {
          nm1d2 = MAX_int32_T;
        } else if (i20 <= -1073741824) {
          nm1d2 = MIN_int32_T;
        } else {
          nm1d2 = i20 << 1;
        }

        for (i20 = 0; i20 < 2; i20++) {
          i21 = (long)cdiff + (1 + i20);
          if (i21 > 2147483647L) {
            i21 = 2147483647L;
          } else {
            if (i21 < -2147483648L) {
              i21 = -2147483648L;
            }
          }

          z_curr_l[i20] = z_all_l[(int)i21 - 1];
          i21 = (long)nm1d2 + (1 + i20);
          if (i21 > 2147483647L) {
            i21 = 2147483647L;
          } else {
            if (i21 < -2147483648L) {
              i21 = -2147483648L;
            }
          }

          z_curr_r[i20] = z_all_r[(int)i21 - 1];
        }

        // 'getH_R_res:65' h_ci_l = xt.anchor_states(anchorIdx).feature_states(featureIdx).scaled_map_point; 
        // 'getH_R_res:66' rho = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth; 
        // 'getH_R_res:67' fp_l = h_ci_l/rho;
        for (i20 = 0; i20 < 3; i20++) {
          fp_l[i20] = xt_anchor_states[anchorIdx].feature_states[featureIdx].
            scaled_map_point[i20] / xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].inverse_depth;
        }

        // 'getH_R_res:68' fp_r = stereoParams.R_rl*fp_l - stereoParams.r_lr;
        for (i20 = 0; i20 < 3; i20++) {
          d5 = 0.0;
          for (cdiff = 0; cdiff < 3; cdiff++) {
            d5 += stereoParams_R_rl[i20 + 3 * cdiff] * fp_l[cdiff];
          }

          fp_r[i20] = d5 - stereoParams_r_lr[i20];
        }

        // 'getH_R_res:69' h_ci_r = fp_r / norm(fp_r);
        B = norm(fp_r);
        for (cdiff = 0; cdiff < 3; cdiff++) {
          h_ci_r[cdiff] = fp_r[cdiff] / B;
        }

        // 'getH_R_res:71' m = xt.anchor_states(anchorIdx).feature_states(featureIdx).m; 
        // 'getH_R_res:73' if VIOParameters.full_stereo
        if (VIOParameters_full_stereo) {
          // 'getH_R_res:75' [ h_u_l, h_u_r ] = predictMeasurementStereo( fp_l, stereoParams ); 
          predictMeasurementStereo(fp_l, c_stereoParams_CameraParameters,
            d_stereoParams_CameraParameters, e_stereoParams_CameraParameters,
            f_stereoParams_CameraParameters, stereoParams_r_lr,
            stereoParams_R_rl, h_u_l, h_u_r);

          // 'getH_R_res:77' r((res_idx-1)*residualDim + (1:residualDim)) = [z_curr_l - h_u_l; z_curr_r - h_u_r]; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i20 = ndbl - 1;
            nm1d2 = i20 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i20 = 0; i20 < ndbl; i20++) {
            tmp_data[i20] = (int)(B + y_data[i20]);
          }

          for (i20 = 0; i20 < 2; i20++) {
            y_data[i20] = z_curr_l[i20] - h_u_l[i20];
            y_data[i20 + 2] = z_curr_r[i20] - h_u_r[i20];
          }

          for (i20 = 0; i20 < ndbl; i20++) {
            r->data[tmp_data[i20] - 1] = y_data[i20];
          }
        } else {
          // 'getH_R_res:78' else
          // 'getH_R_res:79' h_u_l = predictMeasurementMono( fp_l, stereoParams.CameraParameters1 ); 
          // 'getH_R_res:81' r((res_idx-1)*residualDim + (1:residualDim)) = z_curr_l - h_u_l; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i20 = ndbl - 1;
            nm1d2 = i20 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i20 = 0; i20 < ndbl; i20++) {
            tmp_data[i20] = (int)(B + y_data[i20]);
          }

          predictMeasurementMono(fp_l, c_stereoParams_CameraParameters,
            d_stereoParams_CameraParameters, z_curr_r);
          for (i20 = 0; i20 < ndbl; i20++) {
            r->data[tmp_data[i20] - 1] = z_curr_l[i20] - z_curr_r[i20];
          }
        }

        // 'getH_R_res:84' h_u_l_To_h_ci_l = [fx_l/h_ci_l(3),              0, -fx_l*(h_ci_l(1))/h_ci_l(3)^2; 
        // 'getH_R_res:85'                                             0, fy_l/h_ci_l(3), -fy_l*(h_ci_l(2))/h_ci_l(3)^2]; 
        h_u_l_To_h_ci_l[0] = fx_l / xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[2];
        h_u_l_To_h_ci_l[2] = 0.0;
        h_u_l_To_h_ci_l[4] = -fx_l * xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[0] /
          (xt_anchor_states[anchorIdx].feature_states[featureIdx].
           scaled_map_point[2] * xt_anchor_states[anchorIdx]
           .feature_states[featureIdx].scaled_map_point[2]);
        h_u_l_To_h_ci_l[1] = 0.0;
        h_u_l_To_h_ci_l[3] = fy_l / xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[2];
        h_u_l_To_h_ci_l[5] = -fy_l * xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[1] /
          (xt_anchor_states[anchorIdx].feature_states[featureIdx].
           scaled_map_point[2] * xt_anchor_states[anchorIdx]
           .feature_states[featureIdx].scaled_map_point[2]);

        // 'getH_R_res:88' h_u_r_To_h_ci_r = [fx_r/h_ci_r(3), 0,              -fx_r*(h_ci_r(1))/h_ci_r(3)^2; 
        // 'getH_R_res:89'                                             0, fy_r/h_ci_r(3), -fy_r*(h_ci_r(2))/h_ci_r(3)^2]; 
        h_u_r_To_h_ci_r[0] = fx_r / h_ci_r[2];
        h_u_r_To_h_ci_r[2] = 0.0;
        h_u_r_To_h_ci_r[4] = -fx_r * h_ci_r[0] / (h_ci_r[2] * h_ci_r[2]);
        h_u_r_To_h_ci_r[1] = 0.0;
        h_u_r_To_h_ci_r[3] = fy_r / h_ci_r[2];
        h_u_r_To_h_ci_r[5] = -fy_r * h_ci_r[1] / (h_ci_r[2] * h_ci_r[2]);

        // 'getH_R_res:91' fp_r_norm_inv = 1/norm(fp_r);
        fp_r_norm_inv = 1.0 / norm(fp_r);

        // 'getH_R_res:92' fp_r_32_inv = fp_r_norm_inv^3;
        fp_r_32_inv = rt_powd_snf(fp_r_norm_inv, 3.0);

        // 'getH_R_res:94' h_ci_r_To_fp_r = [fp_r_norm_inv - fp_r(1)*fp_r(1)*fp_r_32_inv,               - fp_r(1)*fp_r(2)*fp_r_32_inv,               - fp_r(1)*fp_r(3)*fp_r_32_inv; 
        // 'getH_R_res:95'                                             - fp_r(2)*fp_r(1)*fp_r_32_inv, fp_r_norm_inv - fp_r(2)*fp_r(2)*fp_r_32_inv,               - fp_r(2)*fp_r(3)*fp_r_32_inv; 
        // 'getH_R_res:96'                                             - fp_r(3)*fp_r(1)*fp_r_32_inv,               - fp_r(3)*fp_r(2)*fp_r_32_inv, fp_r_norm_inv - fp_r(3)*fp_r(3)*fp_r_32_inv]; 
        h_ci_r_To_fp_r[0] = fp_r_norm_inv - fp_r[0] * fp_r[0] * fp_r_32_inv;
        h_ci_r_To_fp_r[3] = -fp_r[0] * fp_r[1] * fp_r_32_inv;
        h_ci_r_To_fp_r[6] = -fp_r[0] * fp_r[2] * fp_r_32_inv;
        h_ci_r_To_fp_r[1] = -fp_r[1] * fp_r[0] * fp_r_32_inv;
        h_ci_r_To_fp_r[4] = fp_r_norm_inv - fp_r[1] * fp_r[1] * fp_r_32_inv;
        h_ci_r_To_fp_r[7] = -fp_r[1] * fp_r[2] * fp_r_32_inv;
        h_ci_r_To_fp_r[2] = -fp_r[2] * fp_r[0] * fp_r_32_inv;
        h_ci_r_To_fp_r[5] = -fp_r[2] * fp_r[1] * fp_r_32_inv;
        h_ci_r_To_fp_r[8] = fp_r_norm_inv - fp_r[2] * fp_r[2] * fp_r_32_inv;

        // 'getH_R_res:98' fp_r_To_fp_l = stereoParams.R_rl;
        //              h_ci_l_To_r_wc = -rho*R_cw;
        //              h_ci_l_To_R_cw = skew(h_ci_l);
        // 'getH_R_res:103' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 || rho < 0 
        if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 2)
            || (xt_anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0)) {
          //  delayed initialization or feature behind anchor
          //                  H_robot = [zeros(3), 0*h_ci_l_To_R_cw, zeros(3, numStates-6)]; 
          // 'getH_R_res:105' h_ci_l_To_r_wc = zeros(3);
          // 'getH_R_res:106' h_ci_l_To_R_cw = zeros(3);
          // 'getH_R_res:108' fp_l_To_r_wc = zeros(3);
          // 'getH_R_res:109' fp_l_To_R_cw = zeros(3);
          for (i20 = 0; i20 < 9; i20++) {
            h_ci_l_To_r_wc[i20] = 0.0;
            h_ci_l_To_R_cw[i20] = 0.0;
            fp_l_To_r_wc[i20] = 0.0;
            fp_l_To_R_cw[i20] = 0.0;
          }
        } else {
          // 'getH_R_res:110' else
          //                  H_robot = [h_ci_l_To_r_wc, h_ci_l_To_R_cw, zeros(3, numStates-6)]; 
          // 'getH_R_res:112' h_ci_l_To_r_wc = -rho*R_cw;
          // 'getH_R_res:113' h_ci_l_To_R_cw = skew(h_ci_l);
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

          // 'getH_R_res:115' fp_l_To_r_wc = -R_cw;
          for (i20 = 0; i20 < 9; i20++) {
            h_ci_l_To_r_wc[i20] = -xt_anchor_states[anchorIdx]
              .feature_states[featureIdx].inverse_depth * R_cw[i20];
            fp_l_To_r_wc[i20] = -R_cw[i20];
          }

          // 'getH_R_res:116' fp_l_To_R_cw = skew(fp_l);
          // 'skew:2' R=[0,-w(3),w(2);
          // 'skew:3'     w(3),0,-w(1);
          // 'skew:4'     -w(2),w(1),0];
          fp_l_To_R_cw[0] = 0.0;
          fp_l_To_R_cw[3] = -fp_l[2];
          fp_l_To_R_cw[6] = fp_l[1];
          fp_l_To_R_cw[1] = fp_l[2];
          fp_l_To_R_cw[4] = 0.0;
          fp_l_To_R_cw[7] = -fp_l[0];
          fp_l_To_R_cw[2] = -fp_l[1];
          fp_l_To_R_cw[5] = fp_l[0];
          fp_l_To_R_cw[8] = 0.0;
        }

        //             %% anchor state derivatives
        //              h_ci_l_To_anchorPos = rho*R_cw;
        //              h_ci_l_To_anchorRot = -R_cw * skew(anchorRot'*m);
        //              h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
        // 'getH_R_res:125' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 || rho < 0 
        if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 2)
            || (xt_anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0)) {
          //  delayed initialization or feature behind anchor
          //                  H_map = [zeros(3, 6 + featureIdx-1), h_ci_l_To_rho, zeros(3, numPointsPerAnchor - featureIdx)]; 
          // 'getH_R_res:127' h_ci_l_To_anchorPos = zeros(3);
          // 'getH_R_res:128' h_ci_l_To_anchorRot = zeros(3);
          for (i20 = 0; i20 < 9; i20++) {
            h_ci_l_To_anchorPos[i20] = 0.0;
            h_ci_l_To_anchorRot[i20] = 0.0;
          }

          // 'getH_R_res:129' h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
          for (i20 = 0; i20 < 3; i20++) {
            b_xt_anchor_states[i20] = xt_anchor_states[anchorIdx].pos[i20] -
              xt_robot_state_pos[i20];
          }

          for (i20 = 0; i20 < 3; i20++) {
            fp_r[i20] = 0.0;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              fp_r[i20] += R_cw[i20 + 3 * cdiff] * b_xt_anchor_states[cdiff];
            }
          }

          // 'getH_R_res:131' fp_l_To_anchorPos = zeros(3);
          // 'getH_R_res:132' fp_l_To_anchorRot = zeros(3);
          for (i20 = 0; i20 < 9; i20++) {
            fp_l_To_anchorPos[i20] = 0.0;
            fp_l_To_anchorRot[i20] = 0.0;
          }

          // 'getH_R_res:133' fp_l_To_rho       = -R_cw * anchorRot'*m/rho^2;
          B = xt_anchor_states[anchorIdx].feature_states[featureIdx].
            inverse_depth * xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].inverse_depth;
          for (i20 = 0; i20 < 3; i20++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_R_cw[cdiff + 3 * i20] = -R_cw[cdiff + 3 * i20];
            }
          }

          for (i20 = 0; i20 < 3; i20++) {
            d5 = 0.0;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_R_cw[i20 + 3 * cdiff] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_R_cw[i20 + 3 * cdiff] += b_R_cw[i20 + 3 * nm1d2] *
                  anchorRot[cdiff + 3 * nm1d2];
              }

              d5 += c_R_cw[i20 + 3 * cdiff] * xt_anchor_states[anchorIdx].
                feature_states[featureIdx].m[cdiff];
            }

            fp_l[i20] = d5 / B;
          }
        } else {
          // 'getH_R_res:134' else
          //                  H_map = [h_ci_l_To_anchorPos, h_ci_l_To_anchorRot, zeros(3, featureIdx-1), h_ci_l_To_rho, zeros(3, numPointsPerAnchor - featureIdx)]; 
          // 'getH_R_res:136' h_ci_l_To_anchorPos = rho*R_cw;
          for (i20 = 0; i20 < 9; i20++) {
            h_ci_l_To_anchorPos[i20] = xt_anchor_states[anchorIdx].
              feature_states[featureIdx].inverse_depth * R_cw[i20];
          }

          // 'getH_R_res:137' h_ci_l_To_anchorRot = -R_cw * skew(anchorRot'*m);
          // 'skew:2' R=[0,-w(3),w(2);
          // 'skew:3'     w(3),0,-w(1);
          // 'skew:4'     -w(2),w(1),0];
          for (i20 = 0; i20 < 3; i20++) {
            fp_l[i20] = 0.0;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_R_cw[cdiff + 3 * i20] = -R_cw[cdiff + 3 * i20];
              fp_l[i20] += anchorRot[cdiff + 3 * i20] *
                xt_anchor_states[anchorIdx].feature_states[featureIdx].m[cdiff];
            }
          }

          dv2[0] = 0.0;
          dv2[3] = -fp_l[2];
          dv2[6] = fp_l[1];
          dv2[1] = fp_l[2];
          dv2[4] = 0.0;
          dv2[7] = -fp_l[0];
          dv2[2] = -fp_l[1];
          dv2[5] = fp_l[0];
          dv2[8] = 0.0;

          // 'getH_R_res:138' h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
          for (i20 = 0; i20 < 3; i20++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              h_ci_l_To_anchorRot[i20 + 3 * cdiff] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                h_ci_l_To_anchorRot[i20 + 3 * cdiff] += b_R_cw[i20 + 3 * nm1d2] *
                  dv2[nm1d2 + 3 * cdiff];
              }
            }

            b_xt_anchor_states[i20] = xt_anchor_states[anchorIdx].pos[i20] -
              xt_robot_state_pos[i20];
          }

          for (i20 = 0; i20 < 3; i20++) {
            fp_r[i20] = 0.0;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              fp_r[i20] += R_cw[i20 + 3 * cdiff] * b_xt_anchor_states[cdiff];
            }
          }

          // 'getH_R_res:140' fp_l_To_anchorPos = R_cw;
          memcpy(&fp_l_To_anchorPos[0], &R_cw[0], 9U * sizeof(double));

          // 'getH_R_res:141' fp_l_To_anchorRot = -R_cw * skew(anchorRot'*m/rho); 
          // 'skew:2' R=[0,-w(3),w(2);
          // 'skew:3'     w(3),0,-w(1);
          // 'skew:4'     -w(2),w(1),0];
          for (i20 = 0; i20 < 3; i20++) {
            d5 = 0.0;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              d5 += anchorRot[cdiff + 3 * i20] * xt_anchor_states[anchorIdx].
                feature_states[featureIdx].m[cdiff];
              b_R_cw[cdiff + 3 * i20] = -R_cw[cdiff + 3 * i20];
            }

            fp_l[i20] = d5 / xt_anchor_states[anchorIdx]
              .feature_states[featureIdx].inverse_depth;
          }

          dv3[0] = 0.0;
          dv3[3] = -fp_l[2];
          dv3[6] = fp_l[1];
          dv3[1] = fp_l[2];
          dv3[4] = 0.0;
          dv3[7] = -fp_l[0];
          dv3[2] = -fp_l[1];
          dv3[5] = fp_l[0];
          dv3[8] = 0.0;

          // 'getH_R_res:142' fp_l_To_rho       = -R_cw * anchorRot'*m/rho^2;
          B = xt_anchor_states[anchorIdx].feature_states[featureIdx].
            inverse_depth * xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].inverse_depth;
          for (i20 = 0; i20 < 3; i20++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              fp_l_To_anchorRot[i20 + 3 * cdiff] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                fp_l_To_anchorRot[i20 + 3 * cdiff] += b_R_cw[i20 + 3 * nm1d2] *
                  dv3[nm1d2 + 3 * cdiff];
              }

              c_R_cw[cdiff + 3 * i20] = -R_cw[cdiff + 3 * i20];
            }
          }

          for (i20 = 0; i20 < 3; i20++) {
            d5 = 0.0;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_R_cw[i20 + 3 * cdiff] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_R_cw[i20 + 3 * cdiff] += c_R_cw[i20 + 3 * nm1d2] *
                  anchorRot[cdiff + 3 * nm1d2];
              }

              d5 += b_R_cw[i20 + 3 * cdiff] * xt_anchor_states[anchorIdx].
                feature_states[featureIdx].m[cdiff];
            }

            fp_l[i20] = d5 / B;
          }
        }

        //              tmp = h_u_To_h_ci_l * [H_robot, zeros(3, (anchorIdx-1)*numErrorStatesPerAnchor), H_map, zeros(3, (numAnchors - anchorIdx)*numErrorStatesPerAnchor)]; 
        // 'getH_R_res:146' if VIOParameters.full_stereo
        if (VIOParameters_full_stereo) {
          // 'getH_R_res:147' H((res_idx-1)*residualDim + (1:residualDim), 1:3) = [h_u_l_To_h_ci_l * h_ci_l_To_r_wc; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_r_wc]; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i20 = ndbl - 1;
            nm1d2 = i20 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i20 = 0; i20 < ndbl; i20++) {
            b_tmp_data[i20] = (int)(B + y_data[i20]) - 1;
          }

          for (i20 = 0; i20 < 2; i20++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] += h_u_r_To_h_ci_r[i20 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] = 0.0;
              b_h_u_l_To_h_ci_l[i20 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i20 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
                b_h_u_l_To_h_ci_l[i20 + (cdiff << 1)] += h_u_l_To_h_ci_l[i20 +
                  (nm1d2 << 1)] * h_ci_l_To_r_wc[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                d_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] += c_h_u_r_To_h_ci_r[i20 +
                  (nm1d2 << 1)] * fp_l_To_r_wc[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i20 = 0; i20 < 3; i20++) {
            for (cdiff = 0; cdiff < 2; cdiff++) {
              c_h_u_l_To_h_ci_l[cdiff + (i20 << 2)] = b_h_u_l_To_h_ci_l[cdiff +
                (i20 << 1)];
              c_h_u_l_To_h_ci_l[(cdiff + (i20 << 2)) + 2] =
                d_h_u_r_To_h_ci_r[cdiff + (i20 << 1)];
            }
          }

          for (i20 = 0; i20 < 3; i20++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * i20] =
                c_h_u_l_To_h_ci_l[cdiff + ndbl * i20];
            }
          }

          // 'getH_R_res:148' H((res_idx-1)*residualDim + (1:residualDim), 4:6) = [h_u_l_To_h_ci_l * h_ci_l_To_R_cw; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_R_cw]; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i20 = ndbl - 1;
            nm1d2 = i20 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i20 = 0; i20 < ndbl; i20++) {
            b_tmp_data[i20] = (int)(B + y_data[i20]) - 1;
          }

          for (i20 = 0; i20 < 2; i20++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] += h_u_r_To_h_ci_r[i20 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] = 0.0;
              b_h_u_l_To_h_ci_l[i20 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i20 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
                b_h_u_l_To_h_ci_l[i20 + (cdiff << 1)] += h_u_l_To_h_ci_l[i20 +
                  (nm1d2 << 1)] * h_ci_l_To_R_cw[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                d_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] += c_h_u_r_To_h_ci_r[i20 +
                  (nm1d2 << 1)] * fp_l_To_R_cw[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i20 = 0; i20 < 3; i20++) {
            for (cdiff = 0; cdiff < 2; cdiff++) {
              c_h_u_l_To_h_ci_l[cdiff + (i20 << 2)] = b_h_u_l_To_h_ci_l[cdiff +
                (i20 << 1)];
              c_h_u_l_To_h_ci_l[(cdiff + (i20 << 2)) + 2] =
                d_h_u_r_To_h_ci_r[cdiff + (i20 << 1)];
            }
          }

          for (i20 = 0; i20 < 3; i20++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * (3 + i20)] =
                c_h_u_l_To_h_ci_l[cdiff + ndbl * i20];
            }
          }

          // 'getH_R_res:149' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + int32(1:3)) = [h_u_l_To_h_ci_l*h_ci_l_To_anchorPos; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_anchorPos]; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i20 = ndbl - 1;
            nm1d2 = i20 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i20 = 0; i20 < ndbl; i20++) {
            b_tmp_data[i20] = (int)(B + y_data[i20]) - 1;
          }

          for (i20 = 0; i20 < 2; i20++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] += h_u_r_To_h_ci_r[i20 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] = 0.0;
              b_h_u_l_To_h_ci_l[i20 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i20 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
                b_h_u_l_To_h_ci_l[i20 + (cdiff << 1)] += h_u_l_To_h_ci_l[i20 +
                  (nm1d2 << 1)] * h_ci_l_To_anchorPos[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                d_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] += c_h_u_r_To_h_ci_r[i20 +
                  (nm1d2 << 1)] * fp_l_To_anchorPos[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i20 = 0; i20 < 3; i20++) {
            for (cdiff = 0; cdiff < 2; cdiff++) {
              c_h_u_l_To_h_ci_l[cdiff + (i20 << 2)] = b_h_u_l_To_h_ci_l[cdiff +
                (i20 << 1)];
              c_h_u_l_To_h_ci_l[(cdiff + (i20 << 2)) + 2] =
                d_h_u_r_To_h_ci_r[cdiff + (i20 << 1)];
            }
          }

          i20 = anchorIdx * 11;
          for (cdiff = 0; cdiff < 3; cdiff++) {
            for (nm1d2 = 0; nm1d2 < ndbl; nm1d2++) {
              H->data[b_tmp_data[nm1d2] + H->size[0] * ((cdiff + i20) + 21)] =
                c_h_u_l_To_h_ci_l[nm1d2 + ndbl * cdiff];
            }
          }

          // 'getH_R_res:150' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + int32(4:6)) = [h_u_l_To_h_ci_l*h_ci_l_To_anchorRot; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_anchorRot]; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i20 = ndbl - 1;
            nm1d2 = i20 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i20 = 0; i20 < ndbl; i20++) {
            b_tmp_data[i20] = (int)(B + y_data[i20]) - 1;
          }

          for (i20 = 0; i20 < 2; i20++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] += h_u_r_To_h_ci_r[i20 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] = 0.0;
              b_h_u_l_To_h_ci_l[i20 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i20 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
                b_h_u_l_To_h_ci_l[i20 + (cdiff << 1)] += h_u_l_To_h_ci_l[i20 +
                  (nm1d2 << 1)] * h_ci_l_To_anchorRot[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                d_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] += c_h_u_r_To_h_ci_r[i20 +
                  (nm1d2 << 1)] * fp_l_To_anchorRot[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i20 = 0; i20 < 3; i20++) {
            for (cdiff = 0; cdiff < 2; cdiff++) {
              c_h_u_l_To_h_ci_l[cdiff + (i20 << 2)] = b_h_u_l_To_h_ci_l[cdiff +
                (i20 << 1)];
              c_h_u_l_To_h_ci_l[(cdiff + (i20 << 2)) + 2] =
                d_h_u_r_To_h_ci_r[cdiff + (i20 << 1)];
            }
          }

          i20 = anchorIdx * 11;
          for (cdiff = 0; cdiff < 3; cdiff++) {
            for (nm1d2 = 0; nm1d2 < ndbl; nm1d2++) {
              H->data[b_tmp_data[nm1d2] + H->size[0] * ((cdiff + i20) + 24)] =
                c_h_u_l_To_h_ci_l[nm1d2 + ndbl * cdiff];
            }
          }

          // 'getH_R_res:151' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + 6 + featureIdx) = [h_u_l_To_h_ci_l*h_ci_l_To_rho; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_rho]; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i20 = ndbl - 1;
            nm1d2 = i20 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i20 = 0; i20 < ndbl; i20++) {
            b_tmp_data[i20] = (int)(B + y_data[i20]) - 1;
          }

          for (i20 = 0; i20 < 2; i20++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] += h_u_r_To_h_ci_r[i20 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            d_h_u_l_To_h_ci_l[i20] = 0.0;
            e_h_u_r_To_h_ci_r[i20] = 0.0;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i20 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
              }

              d_h_u_l_To_h_ci_l[i20] += h_u_l_To_h_ci_l[i20 + (cdiff << 1)] *
                fp_r[cdiff];
              e_h_u_r_To_h_ci_r[i20] += c_h_u_r_To_h_ci_r[i20 + (cdiff << 1)] *
                fp_l[cdiff];
            }

            y_data[i20] = d_h_u_l_To_h_ci_l[i20];
            y_data[i20 + 2] = e_h_u_r_To_h_ci_r[i20];
          }

          nm1d2 = anchorIdx * 11 + featureIdx;
          for (i20 = 0; i20 < ndbl; i20++) {
            H->data[b_tmp_data[i20] + H->size[0] * (nm1d2 + 27)] = y_data[i20];
          }
        } else {
          // 'getH_R_res:152' else
          // 'getH_R_res:153' H((res_idx-1)*residualDim + (1:residualDim), 1:3) = h_u_l_To_h_ci_l * h_ci_l_To_r_wc; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i20 = ndbl - 1;
            nm1d2 = i20 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i20 = 0; i20 < ndbl; i20++) {
            b_tmp_data[i20] = (int)(B + y_data[i20]) - 1;
          }

          for (i20 = 0; i20 < 2; i20++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_h_u_l_To_h_ci_l[i20 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_h_u_l_To_h_ci_l[i20 + (cdiff << 1)] += h_u_l_To_h_ci_l[i20 +
                  (nm1d2 << 1)] * h_ci_l_To_r_wc[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i20 = 0; i20 < 3; i20++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * i20] =
                b_h_u_l_To_h_ci_l[cdiff + ndbl * i20];
            }
          }

          // 'getH_R_res:154' H((res_idx-1)*residualDim + (1:residualDim), 4:6) = h_u_l_To_h_ci_l * h_ci_l_To_R_cw; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i20 = ndbl - 1;
            nm1d2 = i20 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i20 = 0; i20 < ndbl; i20++) {
            b_tmp_data[i20] = (int)(B + y_data[i20]) - 1;
          }

          for (i20 = 0; i20 < 2; i20++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_h_u_l_To_h_ci_l[i20 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_h_u_l_To_h_ci_l[i20 + (cdiff << 1)] += h_u_l_To_h_ci_l[i20 +
                  (nm1d2 << 1)] * h_ci_l_To_R_cw[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i20 = 0; i20 < 3; i20++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * (3 + i20)] =
                b_h_u_l_To_h_ci_l[cdiff + ndbl * i20];
            }
          }

          // 'getH_R_res:155' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + int32(1:3)) = h_u_l_To_h_ci_l*h_ci_l_To_anchorPos; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i20 = ndbl - 1;
            nm1d2 = i20 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i20 = 0; i20 < ndbl; i20++) {
            b_tmp_data[i20] = (int)(B + y_data[i20]) - 1;
          }

          for (i20 = 0; i20 < 2; i20++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_h_u_l_To_h_ci_l[i20 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_h_u_l_To_h_ci_l[i20 + (cdiff << 1)] += h_u_l_To_h_ci_l[i20 +
                  (nm1d2 << 1)] * h_ci_l_To_anchorPos[nm1d2 + 3 * cdiff];
              }
            }
          }

          i20 = anchorIdx * 11;
          for (cdiff = 0; cdiff < 3; cdiff++) {
            for (nm1d2 = 0; nm1d2 < ndbl; nm1d2++) {
              H->data[b_tmp_data[nm1d2] + H->size[0] * ((cdiff + i20) + 21)] =
                b_h_u_l_To_h_ci_l[nm1d2 + ndbl * cdiff];
            }
          }

          // 'getH_R_res:156' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + int32(4:6)) = h_u_l_To_h_ci_l*h_ci_l_To_anchorRot; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i20 = ndbl - 1;
            nm1d2 = i20 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i20 = 0; i20 < ndbl; i20++) {
            b_tmp_data[i20] = (int)(B + y_data[i20]) - 1;
          }

          for (i20 = 0; i20 < 2; i20++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_h_u_l_To_h_ci_l[i20 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_h_u_l_To_h_ci_l[i20 + (cdiff << 1)] += h_u_l_To_h_ci_l[i20 +
                  (nm1d2 << 1)] * h_ci_l_To_anchorRot[nm1d2 + 3 * cdiff];
              }
            }
          }

          i20 = anchorIdx * 11;
          for (cdiff = 0; cdiff < 3; cdiff++) {
            for (nm1d2 = 0; nm1d2 < ndbl; nm1d2++) {
              H->data[b_tmp_data[nm1d2] + H->size[0] * ((cdiff + i20) + 24)] =
                b_h_u_l_To_h_ci_l[nm1d2 + ndbl * cdiff];
            }
          }

          // 'getH_R_res:157' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + 6 + featureIdx) = h_u_l_To_h_ci_l*h_ci_l_To_rho; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i20 = ndbl - 1;
            nm1d2 = i20 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i20 = 0; i20 < ndbl; i20++) {
            b_tmp_data[i20] = (int)(B + y_data[i20]) - 1;
          }

          for (i20 = 0; i20 < 2; i20++) {
            d_h_u_l_To_h_ci_l[i20] = 0.0;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_l_To_h_ci_l[i20] += h_u_l_To_h_ci_l[i20 + (cdiff << 1)] *
                fp_r[cdiff];
            }
          }

          nm1d2 = anchorIdx * 11 + featureIdx;
          for (i20 = 0; i20 < ndbl; i20++) {
            H->data[b_tmp_data[i20] + H->size[0] * (nm1d2 + 27)] =
              d_h_u_l_To_h_ci_l[i20];
          }
        }

        //              if coder.target('MATLAB')
        //
        //                  h_ci_l_To_r_wc = -rho*R_cw;
        //                  h_ci_l_To_R_cw = skew(h_ci_l);
        //
        //                  if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 || rho < 0 % delayed initialization or feature behind anchor 
        //                      H_robot = [zeros(3), 0*h_ci_l_To_R_cw, zeros(3, numStates-6)]; 
        //                  else
        //                      H_robot = [h_ci_l_To_r_wc, h_ci_l_To_R_cw, zeros(3, numStates-6)]; 
        //                  end
        //
        //                  %% anchor state derivatives
        //
        //                  h_ci_l_To_anchorPos = rho*R_cw;
        //                  h_ci_l_To_anchorRot = -R_cw * skew(anchorRot'*m);
        //                  h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
        //
        //                  if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 || rho < 0 % delayed initialization or feature behind anchor 
        //                      H_map = [zeros(3, 6 + featureIdx-1), h_ci_l_To_rho, zeros(3, numPointsPerAnchor - featureIdx)]; 
        //                  else
        //                      H_map = [h_ci_l_To_anchorPos, h_ci_l_To_anchorRot, zeros(3, featureIdx-1), h_ci_l_To_rho, zeros(3, numPointsPerAnchor - featureIdx)]; 
        //                  end
        //
        //                  H_t = h_u_To_h_ci_l * [H_robot, zeros(3, (anchorIdx-1)*numErrorStatesPerAnchor), H_map, zeros(3, (numAnchors - anchorIdx)*numErrorStatesPerAnchor)]; 
        //
        //                  if any(any(abs(H((res_idx-1)*residualDim + (1:residualDim), :) - H_t) > 1e-8)) 
        //                      figure; imagesc(H_t - H((res_idx-1)*residualDim + (1:residualDim), :)) 
        //                      error('H inconsistency')
        //                  end
        //              end
        // 'getH_R_res:191' ind(res_idx, 1) = anchorIdx;
        // 'getH_R_res:192' ind(res_idx, 2) = featureIdx;
        // 'getH_R_res:194' res_idx = res_idx + 1;
        res_idx++;
      }
    }
  }

  //  r = z - h_u;
  //  R = diag(repmat((noiseParameters.image_noise), 1, numMeas*2));
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
// Arguments    : double A_data[]
//                int A_size[2]
//                const double B_data[]
//                const int B_size[2]
// Return Type  : void
//
static void b_mrdivide(double A_data[], int A_size[2], const double B_data[],
  const int B_size[2])
{
  emxArray_real_T *A;
  emxArray_int32_T *jpvt;
  int rankR;
  int mn;
  int i;
  int pj;
  int j;
  int b_m;
  double wj;
  emxArray_real_T *tau;
  signed char unnamed_idx_0;
  double Y_data[4];
  double b_B_data[4];
  emxInit_real_T1(&A, 2);
  emxInit_int32_T1(&jpvt, 2);
  if (B_size[0] == B_size[1]) {
    rankR = B_size[1];
    mn = A->size[0] * A->size[1];
    A->size[0] = B_size[0];
    A->size[1] = B_size[1];
    emxEnsureCapacity((emxArray__common *)A, mn, (int)sizeof(double));
    i = B_size[0] * B_size[1];
    for (mn = 0; mn < i; mn++) {
      A->data[mn] = B_data[mn];
    }

    xgetrf(B_size[1], B_size[1], A, B_size[1], jpvt, &pj);
    for (j = 0; j + 1 <= rankR; j++) {
      pj = rankR * j;
      for (b_m = 0; b_m + 1 <= j; b_m++) {
        if (A->data[b_m + pj] != 0.0) {
          A_data[j] -= A->data[b_m + pj] * A_data[b_m];
        }
      }

      wj = A->data[j + pj];
      A_data[j] *= 1.0 / wj;
    }

    for (j = B_size[1]; j > 0; j--) {
      pj = rankR * (j - 1);
      for (b_m = j; b_m + 1 <= rankR; b_m++) {
        if (A->data[b_m + pj] != 0.0) {
          A_data[j - 1] -= A->data[b_m + pj] * A_data[b_m];
        }
      }
    }

    for (pj = B_size[1] - 2; pj + 1 > 0; pj--) {
      if (jpvt->data[pj] != pj + 1) {
        wj = A_data[A_size[0] * pj];
        A_data[A_size[0] * pj] = A_data[A_size[0] * (jpvt->data[pj] - 1)];
        A_data[A_size[0] * (jpvt->data[pj] - 1)] = wj;
      }
    }
  } else {
    mn = A->size[0] * A->size[1];
    A->size[0] = B_size[1];
    A->size[1] = B_size[0];
    emxEnsureCapacity((emxArray__common *)A, mn, (int)sizeof(double));
    i = B_size[0];
    for (mn = 0; mn < i; mn++) {
      pj = B_size[1];
      for (b_m = 0; b_m < pj; b_m++) {
        A->data[b_m + A->size[0] * mn] = B_data[mn + B_size[0] * b_m];
      }
    }

    emxInit_real_T(&tau, 1);
    xgeqp3(A, tau, jpvt);
    rankR = rankFromQR(A);
    unnamed_idx_0 = (signed char)A->size[1];
    i = unnamed_idx_0;
    for (mn = 0; mn < i; mn++) {
      Y_data[mn] = 0.0;
    }

    i = A_size[1];
    for (mn = 0; mn < i; mn++) {
      b_B_data[mn] = A_data[A_size[0] * mn];
    }

    b_m = A->size[0];
    pj = A->size[0];
    mn = A->size[1];
    if (pj <= mn) {
      mn = pj;
    }

    for (j = 0; j + 1 <= mn; j++) {
      if (tau->data[j] != 0.0) {
        wj = b_B_data[j];
        for (i = j + 1; i + 1 <= b_m; i++) {
          wj += A->data[i + A->size[0] * j] * b_B_data[i];
        }

        wj *= tau->data[j];
        if (wj != 0.0) {
          b_B_data[j] -= wj;
          for (i = j + 1; i + 1 <= b_m; i++) {
            b_B_data[i] -= A->data[i + A->size[0] * j] * wj;
          }
        }
      }
    }

    emxFree_real_T(&tau);
    for (i = 0; i + 1 <= rankR; i++) {
      Y_data[jpvt->data[i] - 1] = b_B_data[i];
    }

    for (j = rankR - 1; j + 1 > 0; j--) {
      pj = jpvt->data[j];
      Y_data[jpvt->data[j] - 1] /= A->data[j + A->size[0] * j];
      for (i = 0; i + 1 <= j; i++) {
        Y_data[jpvt->data[i] - 1] -= Y_data[pj - 1] * A->data[i + A->size[0] * j];
      }
    }

    A_size[0] = 1;
    A_size[1] = unnamed_idx_0;
    i = unnamed_idx_0;
    for (mn = 0; mn < i; mn++) {
      A_data[A_size[0] * mn] = Y_data[mn];
    }
  }

  emxFree_int32_T(&jpvt);
  emxFree_real_T(&A);
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
  int i25;
  static const char cv23[14] = { 'i', 'n', 'c', 'o', 'n', 's', 'i', 's', 't',
    'e', 'n', 'c', 'y', '\x00' };

  // 'ros_error:4' if coder.target('MATLAB')
  // 'ros_error:6' elseif ~coder.target('MEX')
  // 'ros_error:7' coder.cinclude('<ros/console.h>')
  // 'ros_error:8' coder.ceval('ROS_ERROR', [str, 0], varargin{:});
  for (i25 = 0; i25 < 14; i25++) {
    cv22[i25] = cv23[i25];
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
  int i16;
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
    for (i16 = 0; i16 < 44; i16++) {
      cv14[i16] = cv15[i16];
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
  int i23;
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
    for (i23 = 0; i23 < 54; i23++) {
      cv18[i23] = cv19[i23];
    }

    ROS_WARN(cv18, varargin_1, varargin_2, varargin_3);
  }
}

//
// Arguments    : double x[5]
//                int idx[5]
// Return Type  : void
//
static void b_sort(double x[5], int idx[5])
{
  int b_m;
  double x4[4];
  signed char idx4[4];
  double xwork[5];
  int nNaNs;
  int ib;
  int k;
  int bLen;
  int nPairs;
  int i4;
  signed char perm[4];
  int iwork[5];
  for (b_m = 0; b_m < 5; b_m++) {
    idx[b_m] = 0;
  }

  for (b_m = 0; b_m < 4; b_m++) {
    x4[b_m] = 0.0;
    idx4[b_m] = 0;
  }

  for (b_m = 0; b_m < 5; b_m++) {
    xwork[b_m] = 0.0;
  }

  nNaNs = -4;
  ib = 0;
  for (k = 0; k < 5; k++) {
    if (rtIsNaN(x[k])) {
      idx[-nNaNs] = k + 1;
      xwork[-nNaNs] = x[k];
      nNaNs++;
    } else {
      ib++;
      idx4[ib - 1] = (signed char)(k + 1);
      x4[ib - 1] = x[k];
      if (ib == 4) {
        ib = (k - nNaNs) - 7;
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

  b_m = (nNaNs + 4) >> 1;
  for (k = 1; k <= b_m; k++) {
    ib = idx[k - nNaNs];
    idx[k - nNaNs] = idx[5 - k];
    idx[5 - k] = ib;
    x[k - nNaNs] = xwork[5 - k];
    x[5 - k] = xwork[k - nNaNs];
  }

  if (((nNaNs + 4) & 1) != 0) {
    x[(b_m - nNaNs) + 1] = xwork[(b_m - nNaNs) + 1];
  }

  if (1 - nNaNs > 1) {
    for (b_m = 0; b_m < 5; b_m++) {
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
  int i66;
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
          i66 = i_i - i;
          for (k = i_i + 1; k + 1 <= i66 + 6; k++) {
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
        i66 = i_i - i;
        for (k = i_i + 1; k + 1 <= i66 + 6; k++) {
          A[k] *= absxk;
        }

        for (k = 1; k <= itemp; k++) {
          smax *= 1.0020841800044864E-292;
        }

        absxk = smax;
      } else {
        temp2 = (smax - A[i_i]) / smax;
        absxk = 1.0 / (A[i_i] - smax);
        i66 = i_i - i;
        for (k = i_i + 1; k + 1 <= i66 + 6; k++) {
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
          i66 = i_ip1 + 6 * (lastc - 1);
          for (itemp = i_ip1; itemp <= i66; itemp += 6) {
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
              i66 = lastv + itemp;
              for (k = itemp; k + 1 <= i66; k++) {
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
// Arguments    : const double x[8]
// Return Type  : boolean_T
//
static boolean_T c_any(const double x[8])
{
  boolean_T y;
  int k;
  boolean_T exitg1;
  boolean_T b0;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 8)) {
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
// Arguments    : double I[11881]
// Return Type  : void
//
static void c_eye(double I[11881])
{
  int k;
  memset(&I[0], 0, 11881U * sizeof(double));
  for (k = 0; k < 109; k++) {
    I[k + 109 * k] = 1.0;
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
  static const char cfmt[9] = { '\x09', 'q', 'v', ':', ' ', '%', 'f', '\x0a',
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
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                int varargin_2
//                int varargin_3
// Return Type  : void
//
static void c_ros_info(int varargin_1, int varargin_2, int varargin_3)
{
  char cv26[24];
  int i27;
  static const char cv27[24] = { 'R', 'e', 'j', 'e', 'c', 't', 'i', 'n', 'g',
    ' ', '%', 'i', ' ', '(', '%', 'i', ' ', 'o', 'n', ' ', '%', 'i', ')', '\x00'
  };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_info:5' if coder.target('MATLAB')
  // 'ros_info:9' elseif ~coder.target('MEX')
  // 'ros_info:10' coder.cinclude('<ros/console.h>')
  // 'ros_info:11' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_info:12' coder.ceval('ROS_INFO', [str, 0], varargin{:});
    for (i27 = 0; i27 < 24; i27++) {
      cv26[i27] = cv27[i27];
    }

    ROS_INFO(cv26, varargin_1, varargin_2, varargin_3);
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
  int i24;
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
    for (i24 = 0; i24 < 44; i24++) {
      cv20[i24] = cv21[i24];
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
// Arguments    : const struct_T x[8]
//                AnchorPose y[8]
// Return Type  : void
//
static void cast(const struct_T x[8], AnchorPose y[8])
{
  int j;
  int i;
  for (j = 0; j < 8; j++) {
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
// Arguments    : double I[11881]
// Return Type  : void
//
static void d_eye(double I[11881])
{
  int k;
  memset(&I[0], 0, 11881U * sizeof(double));
  for (k = 0; k < 109; k++) {
    I[k + 109 * k] = 1.0;
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
  static const char cfmt[9] = { '\x09', 'q', 'w', ':', ' ', '%', 'f', '\x0a',
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
//                int varargin_2
// Return Type  : void
//
static void d_ros_info(int varargin_1, int varargin_2)
{
  char cv32[44];
  int i31;
  static const char cv33[44] = { 'S', 'u', 'c', 'c', 'e', 's', 's', 'f', 'u',
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
    for (i31 = 0; i31 < 44; i31++) {
      cv32[i31] = cv33[i31];
    }

    ROS_INFO(cv32, varargin_1, varargin_2);
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
  int i26;
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
    for (i26 = 0; i26 < 36; i26++) {
      cv24[i26] = cv25[i26];
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
  int i67;
  emxArray_real_T *vwork;
  int vstride;
  int x_idx_0;
  int j;
  emxArray_int32_T *iidx;
  if (dim <= 1) {
    i67 = x->size[0];
  } else {
    i67 = 1;
  }

  emxInit_real_T(&vwork, 1);
  vstride = vwork->size[0];
  vwork->size[0] = i67;
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
  emxInit_int32_T(&iidx, 1);
  while (j + 1 <= vstride) {
    for (x_idx_0 = 0; x_idx_0 + 1 <= i67; x_idx_0++) {
      vwork->data[x_idx_0] = x->data[j + x_idx_0 * vstride];
    }

    sortIdx(vwork, iidx);
    for (x_idx_0 = 0; x_idx_0 + 1 <= i67; x_idx_0++) {
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
  int i39;
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
  for (i39 = 0; i39 < 6; i39++) {
    ipiv[i39] = (signed char)(1 + i39);
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

      i39 = (c - j) + 6;
      for (iy = c + 1; iy + 1 <= i39; iy++) {
        A[iy] /= A[c];
      }
    }

    iy = c;
    jy = c + 6;
    for (b_j = 1; b_j <= 5 - j; b_j++) {
      smax = A[jy];
      if (A[jy] != 0.0) {
        ix = c + 1;
        i39 = (iy - j) + 12;
        for (ijA = 7 + iy; ijA + 1 <= i39; ijA++) {
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
// Arguments    : int numerator
//                int denominator
// Return Type  : int
//
static int div_s32_floor(int numerator, int denominator)
{
  int quotient;
  unsigned int absNumerator;
  unsigned int absDenominator;
  boolean_T quotientNeedsNegation;
  unsigned int tempAbsQuotient;
  if (denominator == 0) {
    if (numerator >= 0) {
      quotient = MAX_int32_T;
    } else {
      quotient = MIN_int32_T;
    }
  } else {
    if (numerator >= 0) {
      absNumerator = (unsigned int)numerator;
    } else {
      absNumerator = (unsigned int)-numerator;
    }

    if (denominator >= 0) {
      absDenominator = (unsigned int)denominator;
    } else {
      absDenominator = (unsigned int)-denominator;
    }

    quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
    tempAbsQuotient = absNumerator / absDenominator;
    if (quotientNeedsNegation) {
      absNumerator %= absDenominator;
      if (absNumerator > 0U) {
        tempAbsQuotient++;
      }
    }

    if (quotientNeedsNegation) {
      quotient = -(int)tempAbsQuotient;
    } else {
      quotient = (int)tempAbsQuotient;
    }
  }

  return quotient;
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
  static const char cfmt[10] = { '\x09', 'q', 'a', 'o', ':', ' ', '%', 'f',
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
static void e_ros_info(int varargin_1)
{
  char cv34[52];
  int i32;
  static const char cv35[52] = { 'I', 'n', 'i', 't', 'i', 'a', 'l', 'i', 'z',
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
    for (i32 = 0; i32 < 52; i32++) {
      cv34[i32] = cv35[i32];
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
  char cv28[37];
  int i29;
  static const char cv29[37] = { 'B', 'a', 'd', ' ', 't', 'r', 'i', 'a', 'n',
    'g', 'u', 'l', 'a', 't', 'i', 'o', 'n', ' ', '(', 'n', 'a', 'n', ')', ' ',
    'f', 'o', 'r', ' ', 'p', 'o', 'i', 'n', 't', ' ', '%', 'd', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_warn:6' if coder.target('MATLAB')
  // 'ros_warn:10' elseif ~coder.target('MEX')
  // 'ros_warn:11' coder.cinclude('<ros/console.h>')
  // 'ros_warn:12' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_warn:13' coder.ceval('ROS_WARN', [str, 0], varargin{:});
    for (i29 = 0; i29 < 37; i29++) {
      cv28[i29] = cv29[i29];
    }

    ROS_WARN(cv28, varargin_1);
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
  static const char cfmt[10] = { '\x09', 'q', 'w', 'o', ':', ' ', '%', 'f',
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
static void f_ros_info(int varargin_1)
{
  char cv36[45];
  int i33;
  static const char cv37[45] = { 'F', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%',
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
    for (i33 = 0; i33 < 45; i33++) {
      cv36[i33] = cv37[i33];
    }

    ROS_INFO(cv36, varargin_1);
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
  char cv30[49];
  int i30;
  static const char cv31[49] = { 'F', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%',
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
    for (i30 = 0; i30 < 49; i30++) {
      cv30[i30] = cv31[i30];
    }

    ROS_WARN(cv30, varargin_1, varargin_2);
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
  static const char cfmt[12] = { '\x09', 'q', 'R', '_', 'c', 'i', ':', ' ', '%',
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
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                double varargin_2
//                double varargin_3
// Return Type  : void
//
static void g_ros_info(int varargin_1, double varargin_2, double varargin_3)
{
  char cv44[51];
  int i37;
  static const char cv45[51] = { 'F', 'o', 'r', 'c', 'i', 'n', 'g', ' ', 'a',
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
    for (i37 = 0; i37 < 51; i37++) {
      cv44[i37] = cv45[i37];
    }

    ROS_INFO(cv44, varargin_1, varargin_2, varargin_3);
  }
}

//
// ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
// Arguments    : double varargin_1
// Return Type  : void
//
static void g_ros_warn(double varargin_1)
{
  char cv38[62];
  int i34;
  static const char cv39[62] = { 'G', 'o', 't', ' ', '%', 'd', ' ', 'n', 'e',
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
    for (i34 = 0; i34 < 62; i34++) {
      cv38[i34] = cv39[i34];
    }

    ROS_WARN(cv38, varargin_1, 3);
  }
}

//
// getAnchorPoses Get the anchor poses in the world frame
// Arguments    : const double xt_origin_pos[3]
//                const double xt_origin_att[4]
//                const f_struct_T xt_anchor_states[8]
//                struct_T anchor_poses[8]
// Return Type  : void
//
static void getAnchorPoses(const double xt_origin_pos[3], const double
  xt_origin_att[4], const f_struct_T xt_anchor_states[8], struct_T anchor_poses
  [8])
{
  double R_ow[9];
  int anchorIdx;
  double b_xt_anchor_states[9];
  double c_xt_anchor_states[9];
  int i44;
  double d7;
  int i45;
  int i46;

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
  for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
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
    for (i44 = 0; i44 < 3; i44++) {
      d7 = 0.0;
      for (i45 = 0; i45 < 3; i45++) {
        d7 += R_ow[i45 + 3 * i44] * xt_anchor_states[anchorIdx].pos[i45];
        b_xt_anchor_states[i44 + 3 * i45] = 0.0;
        for (i46 = 0; i46 < 3; i46++) {
          b_xt_anchor_states[i44 + 3 * i45] += c_xt_anchor_states[i44 + 3 * i46]
            * R_ow[i46 + 3 * i45];
        }
      }

      anchor_poses[anchorIdx].pos[i44] = xt_origin_pos[i44] + d7;
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
//                const f_struct_T xt_anchor_states[8]
//                const double z_all_l[80]
//                const double z_all_r[80]
//                const boolean_T b_status[40]
//                const double c_stereoParams_CameraParameters[2]
//                const double d_stereoParams_CameraParameters[2]
//                const double e_stereoParams_CameraParameters[2]
//                const double f_stereoParams_CameraParameters[2]
//                const double stereoParams_r_lr[3]
//                const double stereoParams_R_rl[9]
//                boolean_T VIOParameters_full_stereo
//                emxArray_real_T *r
//                emxArray_real_T *H
//                emxArray_int32_T *ind
// Return Type  : void
//
static void getH_R_res(const double xt_robot_state_pos[3], const double
  xt_robot_state_att[4], const f_struct_T xt_anchor_states[8], const double
  z_all_l[80], const double z_all_r[80], const boolean_T b_status[40], const
  double c_stereoParams_CameraParameters[2], const double
  d_stereoParams_CameraParameters[2], const double
  e_stereoParams_CameraParameters[2], const double
  f_stereoParams_CameraParameters[2], const double stereoParams_r_lr[3], const
  double stereoParams_R_rl[9], boolean_T VIOParameters_full_stereo,
  emxArray_real_T *r, emxArray_real_T *H, emxArray_int32_T *ind)
{
  double fx_l;
  double fy_l;
  double fx_r;
  double fy_r;
  int nm1d2;
  int cdiff;
  int residualDim;
  double R_cw[9];
  int i17;
  unsigned int res_idx;
  int anchorIdx;
  double anchorRot[9];
  int featureIdx;
  long i18;
  double z_curr_l[2];
  double z_curr_r[2];
  double fp_l[3];
  double fp_r[3];
  double d3;
  double B;
  double h_ci_r[3];
  double h_u_r[2];
  double h_u_l[2];
  int ndbl;
  int apnd;
  double y_data[4];
  int tmp_data[4];
  double h_u_l_To_h_ci_l[6];
  double h_u_r_To_h_ci_r[6];
  double fp_r_norm_inv;
  double fp_r_32_inv;
  double h_ci_r_To_fp_r[9];
  double h_ci_l_To_r_wc[9];
  double h_ci_l_To_R_cw[9];
  double fp_l_To_r_wc[9];
  double fp_l_To_R_cw[9];
  double h_ci_l_To_anchorPos[9];
  double h_ci_l_To_anchorRot[9];
  double b_xt_anchor_states[3];
  double fp_l_To_anchorPos[9];
  double fp_l_To_anchorRot[9];
  double b_R_cw[9];
  double c_R_cw[9];
  double dv0[9];
  double dv1[9];
  int b_tmp_data[4];
  double b_h_u_r_To_h_ci_r[6];
  double c_h_u_r_To_h_ci_r[6];
  double b_h_u_l_To_h_ci_l[6];
  double d_h_u_r_To_h_ci_r[6];
  double c_h_u_l_To_h_ci_l[12];
  double d_h_u_l_To_h_ci_l[2];
  double e_h_u_r_To_h_ci_r[2];

  // 'getH_R_res:26' fx_l = stereoParams.CameraParameters1.FocalLength(1);
  fx_l = c_stereoParams_CameraParameters[0];

  // 'getH_R_res:27' fy_l = stereoParams.CameraParameters1.FocalLength(2);
  fy_l = c_stereoParams_CameraParameters[1];

  // 'getH_R_res:29' fx_r = stereoParams.CameraParameters2.FocalLength(1);
  fx_r = e_stereoParams_CameraParameters[0];

  // 'getH_R_res:30' fy_r = stereoParams.CameraParameters2.FocalLength(2);
  fy_r = e_stereoParams_CameraParameters[1];

  // 'getH_R_res:32' numErrorStatesPerAnchor = 6 + numPointsPerAnchor;
  // 'getH_R_res:34' numMeas = nnz(status);
  nm1d2 = 0;
  for (cdiff = 0; cdiff < 40; cdiff++) {
    if (b_status[cdiff]) {
      nm1d2++;
    }
  }

  // 'getH_R_res:35' if VIOParameters.full_stereo
  if (VIOParameters_full_stereo) {
    // 'getH_R_res:36' residualDim = 4;
    residualDim = 4;
  } else {
    // 'getH_R_res:37' else
    // 'getH_R_res:38' residualDim = 2;
    residualDim = 2;
  }

  // 'getH_R_res:41' R_cw = RotFromQuatJ(xt.robot_state.att);
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

  // 'getH_R_res:42' r_wc = xt.robot_state.pos;
  // 'getH_R_res:44' H = zeros(numMeas*residualDim, numStates + numAnchors*numErrorStatesPerAnchor); 
  i17 = H->size[0] * H->size[1];
  H->size[0] = (int)((double)nm1d2 * (double)residualDim);
  H->size[1] = 109;
  emxEnsureCapacity((emxArray__common *)H, i17, (int)sizeof(double));
  cdiff = (int)((double)nm1d2 * (double)residualDim) * 109;
  for (i17 = 0; i17 < cdiff; i17++) {
    H->data[i17] = 0.0;
  }

  //  z = zeros(numMeas*residualDim, 1);
  //  h_u = zeros(numMeas*residualDim, 1);
  // 'getH_R_res:47' r = zeros(numMeas*residualDim, 1);
  i17 = r->size[0];
  r->size[0] = (int)((double)nm1d2 * (double)residualDim);
  emxEnsureCapacity((emxArray__common *)r, i17, (int)sizeof(double));
  cdiff = (int)((double)nm1d2 * (double)residualDim);
  for (i17 = 0; i17 < cdiff; i17++) {
    r->data[i17] = 0.0;
  }

  // 'getH_R_res:49' ind = int32(zeros(numMeas, 2));
  i17 = ind->size[0] * ind->size[1];
  ind->size[0] = nm1d2;
  ind->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)ind, i17, (int)sizeof(int));
  cdiff = nm1d2 << 1;
  for (i17 = 0; i17 < cdiff; i17++) {
    ind->data[i17] = 0;
  }

  // 'getH_R_res:51' res_idx = 1;
  res_idx = 1U;

  // 'getH_R_res:53' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
    // 'getH_R_res:54' anchorPos = xt.anchor_states(anchorIdx).pos;
    // 'getH_R_res:55' anchorRot = RotFromQuatJ(xt.anchor_states(anchorIdx).att); 
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

    // 'getH_R_res:57' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 5; featureIdx++) {
      // 'getH_R_res:59' idx = xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx; 
      // 'getH_R_res:61' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status && status(idx) 
      if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status != 0) &&
          b_status[xt_anchor_states[anchorIdx].feature_states[featureIdx].
          status_idx - 1]) {
        // 'getH_R_res:62' z_curr_l = z_all_l((idx-1)*2 + int32(1:2));
        i18 = xt_anchor_states[anchorIdx].feature_states[featureIdx].status_idx
          - 1L;
        if (i18 > 2147483647L) {
          i18 = 2147483647L;
        } else {
          if (i18 < -2147483648L) {
            i18 = -2147483648L;
          }
        }

        i17 = (int)i18;
        if (i17 > 1073741823) {
          cdiff = MAX_int32_T;
        } else if (i17 <= -1073741824) {
          cdiff = MIN_int32_T;
        } else {
          cdiff = i17 << 1;
        }

        // 'getH_R_res:63' z_curr_r = z_all_r((idx-1)*2 + int32(1:2));
        i18 = xt_anchor_states[anchorIdx].feature_states[featureIdx].status_idx
          - 1L;
        if (i18 > 2147483647L) {
          i18 = 2147483647L;
        } else {
          if (i18 < -2147483648L) {
            i18 = -2147483648L;
          }
        }

        i17 = (int)i18;
        if (i17 > 1073741823) {
          nm1d2 = MAX_int32_T;
        } else if (i17 <= -1073741824) {
          nm1d2 = MIN_int32_T;
        } else {
          nm1d2 = i17 << 1;
        }

        for (i17 = 0; i17 < 2; i17++) {
          i18 = (long)cdiff + (1 + i17);
          if (i18 > 2147483647L) {
            i18 = 2147483647L;
          } else {
            if (i18 < -2147483648L) {
              i18 = -2147483648L;
            }
          }

          z_curr_l[i17] = z_all_l[(int)i18 - 1];
          i18 = (long)nm1d2 + (1 + i17);
          if (i18 > 2147483647L) {
            i18 = 2147483647L;
          } else {
            if (i18 < -2147483648L) {
              i18 = -2147483648L;
            }
          }

          z_curr_r[i17] = z_all_r[(int)i18 - 1];
        }

        // 'getH_R_res:65' h_ci_l = xt.anchor_states(anchorIdx).feature_states(featureIdx).scaled_map_point; 
        // 'getH_R_res:66' rho = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth; 
        // 'getH_R_res:67' fp_l = h_ci_l/rho;
        for (i17 = 0; i17 < 3; i17++) {
          fp_l[i17] = xt_anchor_states[anchorIdx].feature_states[featureIdx].
            scaled_map_point[i17] / xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].inverse_depth;
        }

        // 'getH_R_res:68' fp_r = stereoParams.R_rl*fp_l - stereoParams.r_lr;
        for (i17 = 0; i17 < 3; i17++) {
          d3 = 0.0;
          for (cdiff = 0; cdiff < 3; cdiff++) {
            d3 += stereoParams_R_rl[i17 + 3 * cdiff] * fp_l[cdiff];
          }

          fp_r[i17] = d3 - stereoParams_r_lr[i17];
        }

        // 'getH_R_res:69' h_ci_r = fp_r / norm(fp_r);
        B = norm(fp_r);
        for (cdiff = 0; cdiff < 3; cdiff++) {
          h_ci_r[cdiff] = fp_r[cdiff] / B;
        }

        // 'getH_R_res:71' m = xt.anchor_states(anchorIdx).feature_states(featureIdx).m; 
        // 'getH_R_res:73' if VIOParameters.full_stereo
        if (VIOParameters_full_stereo) {
          // 'getH_R_res:75' [ h_u_l, h_u_r ] = predictMeasurementStereo( fp_l, stereoParams ); 
          predictMeasurementStereo(fp_l, c_stereoParams_CameraParameters,
            d_stereoParams_CameraParameters, e_stereoParams_CameraParameters,
            f_stereoParams_CameraParameters, stereoParams_r_lr,
            stereoParams_R_rl, h_u_l, h_u_r);

          // 'getH_R_res:77' r((res_idx-1)*residualDim + (1:residualDim)) = [z_curr_l - h_u_l; z_curr_r - h_u_r]; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i17 = ndbl - 1;
            nm1d2 = i17 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i17 = 0; i17 < ndbl; i17++) {
            tmp_data[i17] = (int)(B + y_data[i17]);
          }

          for (i17 = 0; i17 < 2; i17++) {
            y_data[i17] = z_curr_l[i17] - h_u_l[i17];
            y_data[i17 + 2] = z_curr_r[i17] - h_u_r[i17];
          }

          for (i17 = 0; i17 < ndbl; i17++) {
            r->data[tmp_data[i17] - 1] = y_data[i17];
          }
        } else {
          // 'getH_R_res:78' else
          // 'getH_R_res:79' h_u_l = predictMeasurementMono( fp_l, stereoParams.CameraParameters1 ); 
          // 'getH_R_res:81' r((res_idx-1)*residualDim + (1:residualDim)) = z_curr_l - h_u_l; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i17 = ndbl - 1;
            nm1d2 = i17 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i17 = 0; i17 < ndbl; i17++) {
            tmp_data[i17] = (int)(B + y_data[i17]);
          }

          predictMeasurementMono(fp_l, c_stereoParams_CameraParameters,
            d_stereoParams_CameraParameters, z_curr_r);
          for (i17 = 0; i17 < ndbl; i17++) {
            r->data[tmp_data[i17] - 1] = z_curr_l[i17] - z_curr_r[i17];
          }
        }

        // 'getH_R_res:84' h_u_l_To_h_ci_l = [fx_l/h_ci_l(3),              0, -fx_l*(h_ci_l(1))/h_ci_l(3)^2; 
        // 'getH_R_res:85'                                             0, fy_l/h_ci_l(3), -fy_l*(h_ci_l(2))/h_ci_l(3)^2]; 
        h_u_l_To_h_ci_l[0] = fx_l / xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[2];
        h_u_l_To_h_ci_l[2] = 0.0;
        h_u_l_To_h_ci_l[4] = -fx_l * xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[0] /
          (xt_anchor_states[anchorIdx].feature_states[featureIdx].
           scaled_map_point[2] * xt_anchor_states[anchorIdx]
           .feature_states[featureIdx].scaled_map_point[2]);
        h_u_l_To_h_ci_l[1] = 0.0;
        h_u_l_To_h_ci_l[3] = fy_l / xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[2];
        h_u_l_To_h_ci_l[5] = -fy_l * xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[1] /
          (xt_anchor_states[anchorIdx].feature_states[featureIdx].
           scaled_map_point[2] * xt_anchor_states[anchorIdx]
           .feature_states[featureIdx].scaled_map_point[2]);

        // 'getH_R_res:88' h_u_r_To_h_ci_r = [fx_r/h_ci_r(3), 0,              -fx_r*(h_ci_r(1))/h_ci_r(3)^2; 
        // 'getH_R_res:89'                                             0, fy_r/h_ci_r(3), -fy_r*(h_ci_r(2))/h_ci_r(3)^2]; 
        h_u_r_To_h_ci_r[0] = fx_r / h_ci_r[2];
        h_u_r_To_h_ci_r[2] = 0.0;
        h_u_r_To_h_ci_r[4] = -fx_r * h_ci_r[0] / (h_ci_r[2] * h_ci_r[2]);
        h_u_r_To_h_ci_r[1] = 0.0;
        h_u_r_To_h_ci_r[3] = fy_r / h_ci_r[2];
        h_u_r_To_h_ci_r[5] = -fy_r * h_ci_r[1] / (h_ci_r[2] * h_ci_r[2]);

        // 'getH_R_res:91' fp_r_norm_inv = 1/norm(fp_r);
        fp_r_norm_inv = 1.0 / norm(fp_r);

        // 'getH_R_res:92' fp_r_32_inv = fp_r_norm_inv^3;
        fp_r_32_inv = rt_powd_snf(fp_r_norm_inv, 3.0);

        // 'getH_R_res:94' h_ci_r_To_fp_r = [fp_r_norm_inv - fp_r(1)*fp_r(1)*fp_r_32_inv,               - fp_r(1)*fp_r(2)*fp_r_32_inv,               - fp_r(1)*fp_r(3)*fp_r_32_inv; 
        // 'getH_R_res:95'                                             - fp_r(2)*fp_r(1)*fp_r_32_inv, fp_r_norm_inv - fp_r(2)*fp_r(2)*fp_r_32_inv,               - fp_r(2)*fp_r(3)*fp_r_32_inv; 
        // 'getH_R_res:96'                                             - fp_r(3)*fp_r(1)*fp_r_32_inv,               - fp_r(3)*fp_r(2)*fp_r_32_inv, fp_r_norm_inv - fp_r(3)*fp_r(3)*fp_r_32_inv]; 
        h_ci_r_To_fp_r[0] = fp_r_norm_inv - fp_r[0] * fp_r[0] * fp_r_32_inv;
        h_ci_r_To_fp_r[3] = -fp_r[0] * fp_r[1] * fp_r_32_inv;
        h_ci_r_To_fp_r[6] = -fp_r[0] * fp_r[2] * fp_r_32_inv;
        h_ci_r_To_fp_r[1] = -fp_r[1] * fp_r[0] * fp_r_32_inv;
        h_ci_r_To_fp_r[4] = fp_r_norm_inv - fp_r[1] * fp_r[1] * fp_r_32_inv;
        h_ci_r_To_fp_r[7] = -fp_r[1] * fp_r[2] * fp_r_32_inv;
        h_ci_r_To_fp_r[2] = -fp_r[2] * fp_r[0] * fp_r_32_inv;
        h_ci_r_To_fp_r[5] = -fp_r[2] * fp_r[1] * fp_r_32_inv;
        h_ci_r_To_fp_r[8] = fp_r_norm_inv - fp_r[2] * fp_r[2] * fp_r_32_inv;

        // 'getH_R_res:98' fp_r_To_fp_l = stereoParams.R_rl;
        //              h_ci_l_To_r_wc = -rho*R_cw;
        //              h_ci_l_To_R_cw = skew(h_ci_l);
        // 'getH_R_res:103' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 || rho < 0 
        if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 2)
            || (xt_anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0)) {
          //  delayed initialization or feature behind anchor
          //                  H_robot = [zeros(3), 0*h_ci_l_To_R_cw, zeros(3, numStates-6)]; 
          // 'getH_R_res:105' h_ci_l_To_r_wc = zeros(3);
          // 'getH_R_res:106' h_ci_l_To_R_cw = zeros(3);
          // 'getH_R_res:108' fp_l_To_r_wc = zeros(3);
          // 'getH_R_res:109' fp_l_To_R_cw = zeros(3);
          for (i17 = 0; i17 < 9; i17++) {
            h_ci_l_To_r_wc[i17] = 0.0;
            h_ci_l_To_R_cw[i17] = 0.0;
            fp_l_To_r_wc[i17] = 0.0;
            fp_l_To_R_cw[i17] = 0.0;
          }
        } else {
          // 'getH_R_res:110' else
          //                  H_robot = [h_ci_l_To_r_wc, h_ci_l_To_R_cw, zeros(3, numStates-6)]; 
          // 'getH_R_res:112' h_ci_l_To_r_wc = -rho*R_cw;
          // 'getH_R_res:113' h_ci_l_To_R_cw = skew(h_ci_l);
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

          // 'getH_R_res:115' fp_l_To_r_wc = -R_cw;
          for (i17 = 0; i17 < 9; i17++) {
            h_ci_l_To_r_wc[i17] = -xt_anchor_states[anchorIdx]
              .feature_states[featureIdx].inverse_depth * R_cw[i17];
            fp_l_To_r_wc[i17] = -R_cw[i17];
          }

          // 'getH_R_res:116' fp_l_To_R_cw = skew(fp_l);
          // 'skew:2' R=[0,-w(3),w(2);
          // 'skew:3'     w(3),0,-w(1);
          // 'skew:4'     -w(2),w(1),0];
          fp_l_To_R_cw[0] = 0.0;
          fp_l_To_R_cw[3] = -fp_l[2];
          fp_l_To_R_cw[6] = fp_l[1];
          fp_l_To_R_cw[1] = fp_l[2];
          fp_l_To_R_cw[4] = 0.0;
          fp_l_To_R_cw[7] = -fp_l[0];
          fp_l_To_R_cw[2] = -fp_l[1];
          fp_l_To_R_cw[5] = fp_l[0];
          fp_l_To_R_cw[8] = 0.0;
        }

        //             %% anchor state derivatives
        //              h_ci_l_To_anchorPos = rho*R_cw;
        //              h_ci_l_To_anchorRot = -R_cw * skew(anchorRot'*m);
        //              h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
        // 'getH_R_res:125' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 || rho < 0 
        if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 2)
            || (xt_anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0)) {
          //  delayed initialization or feature behind anchor
          //                  H_map = [zeros(3, 6 + featureIdx-1), h_ci_l_To_rho, zeros(3, numPointsPerAnchor - featureIdx)]; 
          // 'getH_R_res:127' h_ci_l_To_anchorPos = zeros(3);
          // 'getH_R_res:128' h_ci_l_To_anchorRot = zeros(3);
          for (i17 = 0; i17 < 9; i17++) {
            h_ci_l_To_anchorPos[i17] = 0.0;
            h_ci_l_To_anchorRot[i17] = 0.0;
          }

          // 'getH_R_res:129' h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
          for (i17 = 0; i17 < 3; i17++) {
            b_xt_anchor_states[i17] = xt_anchor_states[anchorIdx].pos[i17] -
              xt_robot_state_pos[i17];
          }

          for (i17 = 0; i17 < 3; i17++) {
            fp_r[i17] = 0.0;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              fp_r[i17] += R_cw[i17 + 3 * cdiff] * b_xt_anchor_states[cdiff];
            }
          }

          // 'getH_R_res:131' fp_l_To_anchorPos = zeros(3);
          // 'getH_R_res:132' fp_l_To_anchorRot = zeros(3);
          for (i17 = 0; i17 < 9; i17++) {
            fp_l_To_anchorPos[i17] = 0.0;
            fp_l_To_anchorRot[i17] = 0.0;
          }

          // 'getH_R_res:133' fp_l_To_rho       = -R_cw * anchorRot'*m/rho^2;
          B = xt_anchor_states[anchorIdx].feature_states[featureIdx].
            inverse_depth * xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].inverse_depth;
          for (i17 = 0; i17 < 3; i17++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_R_cw[cdiff + 3 * i17] = -R_cw[cdiff + 3 * i17];
            }
          }

          for (i17 = 0; i17 < 3; i17++) {
            d3 = 0.0;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_R_cw[i17 + 3 * cdiff] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_R_cw[i17 + 3 * cdiff] += b_R_cw[i17 + 3 * nm1d2] *
                  anchorRot[cdiff + 3 * nm1d2];
              }

              d3 += c_R_cw[i17 + 3 * cdiff] * xt_anchor_states[anchorIdx].
                feature_states[featureIdx].m[cdiff];
            }

            fp_l[i17] = d3 / B;
          }
        } else {
          // 'getH_R_res:134' else
          //                  H_map = [h_ci_l_To_anchorPos, h_ci_l_To_anchorRot, zeros(3, featureIdx-1), h_ci_l_To_rho, zeros(3, numPointsPerAnchor - featureIdx)]; 
          // 'getH_R_res:136' h_ci_l_To_anchorPos = rho*R_cw;
          for (i17 = 0; i17 < 9; i17++) {
            h_ci_l_To_anchorPos[i17] = xt_anchor_states[anchorIdx].
              feature_states[featureIdx].inverse_depth * R_cw[i17];
          }

          // 'getH_R_res:137' h_ci_l_To_anchorRot = -R_cw * skew(anchorRot'*m);
          // 'skew:2' R=[0,-w(3),w(2);
          // 'skew:3'     w(3),0,-w(1);
          // 'skew:4'     -w(2),w(1),0];
          for (i17 = 0; i17 < 3; i17++) {
            fp_l[i17] = 0.0;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_R_cw[cdiff + 3 * i17] = -R_cw[cdiff + 3 * i17];
              fp_l[i17] += anchorRot[cdiff + 3 * i17] *
                xt_anchor_states[anchorIdx].feature_states[featureIdx].m[cdiff];
            }
          }

          dv0[0] = 0.0;
          dv0[3] = -fp_l[2];
          dv0[6] = fp_l[1];
          dv0[1] = fp_l[2];
          dv0[4] = 0.0;
          dv0[7] = -fp_l[0];
          dv0[2] = -fp_l[1];
          dv0[5] = fp_l[0];
          dv0[8] = 0.0;

          // 'getH_R_res:138' h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
          for (i17 = 0; i17 < 3; i17++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              h_ci_l_To_anchorRot[i17 + 3 * cdiff] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                h_ci_l_To_anchorRot[i17 + 3 * cdiff] += b_R_cw[i17 + 3 * nm1d2] *
                  dv0[nm1d2 + 3 * cdiff];
              }
            }

            b_xt_anchor_states[i17] = xt_anchor_states[anchorIdx].pos[i17] -
              xt_robot_state_pos[i17];
          }

          for (i17 = 0; i17 < 3; i17++) {
            fp_r[i17] = 0.0;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              fp_r[i17] += R_cw[i17 + 3 * cdiff] * b_xt_anchor_states[cdiff];
            }
          }

          // 'getH_R_res:140' fp_l_To_anchorPos = R_cw;
          memcpy(&fp_l_To_anchorPos[0], &R_cw[0], 9U * sizeof(double));

          // 'getH_R_res:141' fp_l_To_anchorRot = -R_cw * skew(anchorRot'*m/rho); 
          // 'skew:2' R=[0,-w(3),w(2);
          // 'skew:3'     w(3),0,-w(1);
          // 'skew:4'     -w(2),w(1),0];
          for (i17 = 0; i17 < 3; i17++) {
            d3 = 0.0;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              d3 += anchorRot[cdiff + 3 * i17] * xt_anchor_states[anchorIdx].
                feature_states[featureIdx].m[cdiff];
              b_R_cw[cdiff + 3 * i17] = -R_cw[cdiff + 3 * i17];
            }

            fp_l[i17] = d3 / xt_anchor_states[anchorIdx]
              .feature_states[featureIdx].inverse_depth;
          }

          dv1[0] = 0.0;
          dv1[3] = -fp_l[2];
          dv1[6] = fp_l[1];
          dv1[1] = fp_l[2];
          dv1[4] = 0.0;
          dv1[7] = -fp_l[0];
          dv1[2] = -fp_l[1];
          dv1[5] = fp_l[0];
          dv1[8] = 0.0;

          // 'getH_R_res:142' fp_l_To_rho       = -R_cw * anchorRot'*m/rho^2;
          B = xt_anchor_states[anchorIdx].feature_states[featureIdx].
            inverse_depth * xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].inverse_depth;
          for (i17 = 0; i17 < 3; i17++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              fp_l_To_anchorRot[i17 + 3 * cdiff] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                fp_l_To_anchorRot[i17 + 3 * cdiff] += b_R_cw[i17 + 3 * nm1d2] *
                  dv1[nm1d2 + 3 * cdiff];
              }

              c_R_cw[cdiff + 3 * i17] = -R_cw[cdiff + 3 * i17];
            }
          }

          for (i17 = 0; i17 < 3; i17++) {
            d3 = 0.0;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_R_cw[i17 + 3 * cdiff] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_R_cw[i17 + 3 * cdiff] += c_R_cw[i17 + 3 * nm1d2] *
                  anchorRot[cdiff + 3 * nm1d2];
              }

              d3 += b_R_cw[i17 + 3 * cdiff] * xt_anchor_states[anchorIdx].
                feature_states[featureIdx].m[cdiff];
            }

            fp_l[i17] = d3 / B;
          }
        }

        //              tmp = h_u_To_h_ci_l * [H_robot, zeros(3, (anchorIdx-1)*numErrorStatesPerAnchor), H_map, zeros(3, (numAnchors - anchorIdx)*numErrorStatesPerAnchor)]; 
        // 'getH_R_res:146' if VIOParameters.full_stereo
        if (VIOParameters_full_stereo) {
          // 'getH_R_res:147' H((res_idx-1)*residualDim + (1:residualDim), 1:3) = [h_u_l_To_h_ci_l * h_ci_l_To_r_wc; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_r_wc]; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i17 = ndbl - 1;
            nm1d2 = i17 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i17 = 0; i17 < ndbl; i17++) {
            b_tmp_data[i17] = (int)(B + y_data[i17]) - 1;
          }

          for (i17 = 0; i17 < 2; i17++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] += h_u_r_To_h_ci_r[i17 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] = 0.0;
              b_h_u_l_To_h_ci_l[i17 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i17 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
                b_h_u_l_To_h_ci_l[i17 + (cdiff << 1)] += h_u_l_To_h_ci_l[i17 +
                  (nm1d2 << 1)] * h_ci_l_To_r_wc[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                d_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] += c_h_u_r_To_h_ci_r[i17 +
                  (nm1d2 << 1)] * fp_l_To_r_wc[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i17 = 0; i17 < 3; i17++) {
            for (cdiff = 0; cdiff < 2; cdiff++) {
              c_h_u_l_To_h_ci_l[cdiff + (i17 << 2)] = b_h_u_l_To_h_ci_l[cdiff +
                (i17 << 1)];
              c_h_u_l_To_h_ci_l[(cdiff + (i17 << 2)) + 2] =
                d_h_u_r_To_h_ci_r[cdiff + (i17 << 1)];
            }
          }

          for (i17 = 0; i17 < 3; i17++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * i17] =
                c_h_u_l_To_h_ci_l[cdiff + ndbl * i17];
            }
          }

          // 'getH_R_res:148' H((res_idx-1)*residualDim + (1:residualDim), 4:6) = [h_u_l_To_h_ci_l * h_ci_l_To_R_cw; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_R_cw]; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i17 = ndbl - 1;
            nm1d2 = i17 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i17 = 0; i17 < ndbl; i17++) {
            b_tmp_data[i17] = (int)(B + y_data[i17]) - 1;
          }

          for (i17 = 0; i17 < 2; i17++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] += h_u_r_To_h_ci_r[i17 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] = 0.0;
              b_h_u_l_To_h_ci_l[i17 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i17 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
                b_h_u_l_To_h_ci_l[i17 + (cdiff << 1)] += h_u_l_To_h_ci_l[i17 +
                  (nm1d2 << 1)] * h_ci_l_To_R_cw[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                d_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] += c_h_u_r_To_h_ci_r[i17 +
                  (nm1d2 << 1)] * fp_l_To_R_cw[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i17 = 0; i17 < 3; i17++) {
            for (cdiff = 0; cdiff < 2; cdiff++) {
              c_h_u_l_To_h_ci_l[cdiff + (i17 << 2)] = b_h_u_l_To_h_ci_l[cdiff +
                (i17 << 1)];
              c_h_u_l_To_h_ci_l[(cdiff + (i17 << 2)) + 2] =
                d_h_u_r_To_h_ci_r[cdiff + (i17 << 1)];
            }
          }

          for (i17 = 0; i17 < 3; i17++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * (3 + i17)] =
                c_h_u_l_To_h_ci_l[cdiff + ndbl * i17];
            }
          }

          // 'getH_R_res:149' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + int32(1:3)) = [h_u_l_To_h_ci_l*h_ci_l_To_anchorPos; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_anchorPos]; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i17 = ndbl - 1;
            nm1d2 = i17 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i17 = 0; i17 < ndbl; i17++) {
            b_tmp_data[i17] = (int)(B + y_data[i17]) - 1;
          }

          for (i17 = 0; i17 < 2; i17++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] += h_u_r_To_h_ci_r[i17 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] = 0.0;
              b_h_u_l_To_h_ci_l[i17 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i17 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
                b_h_u_l_To_h_ci_l[i17 + (cdiff << 1)] += h_u_l_To_h_ci_l[i17 +
                  (nm1d2 << 1)] * h_ci_l_To_anchorPos[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                d_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] += c_h_u_r_To_h_ci_r[i17 +
                  (nm1d2 << 1)] * fp_l_To_anchorPos[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i17 = 0; i17 < 3; i17++) {
            for (cdiff = 0; cdiff < 2; cdiff++) {
              c_h_u_l_To_h_ci_l[cdiff + (i17 << 2)] = b_h_u_l_To_h_ci_l[cdiff +
                (i17 << 1)];
              c_h_u_l_To_h_ci_l[(cdiff + (i17 << 2)) + 2] =
                d_h_u_r_To_h_ci_r[cdiff + (i17 << 1)];
            }
          }

          i17 = anchorIdx * 11;
          for (cdiff = 0; cdiff < 3; cdiff++) {
            for (nm1d2 = 0; nm1d2 < ndbl; nm1d2++) {
              H->data[b_tmp_data[nm1d2] + H->size[0] * ((cdiff + i17) + 21)] =
                c_h_u_l_To_h_ci_l[nm1d2 + ndbl * cdiff];
            }
          }

          // 'getH_R_res:150' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + int32(4:6)) = [h_u_l_To_h_ci_l*h_ci_l_To_anchorRot; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_anchorRot]; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i17 = ndbl - 1;
            nm1d2 = i17 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i17 = 0; i17 < ndbl; i17++) {
            b_tmp_data[i17] = (int)(B + y_data[i17]) - 1;
          }

          for (i17 = 0; i17 < 2; i17++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] += h_u_r_To_h_ci_r[i17 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] = 0.0;
              b_h_u_l_To_h_ci_l[i17 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i17 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
                b_h_u_l_To_h_ci_l[i17 + (cdiff << 1)] += h_u_l_To_h_ci_l[i17 +
                  (nm1d2 << 1)] * h_ci_l_To_anchorRot[nm1d2 + 3 * cdiff];
              }
            }

            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                d_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] += c_h_u_r_To_h_ci_r[i17 +
                  (nm1d2 << 1)] * fp_l_To_anchorRot[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i17 = 0; i17 < 3; i17++) {
            for (cdiff = 0; cdiff < 2; cdiff++) {
              c_h_u_l_To_h_ci_l[cdiff + (i17 << 2)] = b_h_u_l_To_h_ci_l[cdiff +
                (i17 << 1)];
              c_h_u_l_To_h_ci_l[(cdiff + (i17 << 2)) + 2] =
                d_h_u_r_To_h_ci_r[cdiff + (i17 << 1)];
            }
          }

          i17 = anchorIdx * 11;
          for (cdiff = 0; cdiff < 3; cdiff++) {
            for (nm1d2 = 0; nm1d2 < ndbl; nm1d2++) {
              H->data[b_tmp_data[nm1d2] + H->size[0] * ((cdiff + i17) + 24)] =
                c_h_u_l_To_h_ci_l[nm1d2 + ndbl * cdiff];
            }
          }

          // 'getH_R_res:151' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + 6 + featureIdx) = [h_u_l_To_h_ci_l*h_ci_l_To_rho; h_u_r_To_h_ci_r * h_ci_r_To_fp_r * fp_r_To_fp_l * fp_l_To_rho]; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i17 = ndbl - 1;
            nm1d2 = i17 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i17 = 0; i17 < ndbl; i17++) {
            b_tmp_data[i17] = (int)(B + y_data[i17]) - 1;
          }

          for (i17 = 0; i17 < 2; i17++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] += h_u_r_To_h_ci_r[i17 +
                  (nm1d2 << 1)] * h_ci_r_To_fp_r[nm1d2 + 3 * cdiff];
              }
            }

            d_h_u_l_To_h_ci_l[i17] = 0.0;
            e_h_u_r_To_h_ci_r[i17] = 0.0;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              c_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                c_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] += b_h_u_r_To_h_ci_r[i17 +
                  (nm1d2 << 1)] * stereoParams_R_rl[nm1d2 + 3 * cdiff];
              }

              d_h_u_l_To_h_ci_l[i17] += h_u_l_To_h_ci_l[i17 + (cdiff << 1)] *
                fp_r[cdiff];
              e_h_u_r_To_h_ci_r[i17] += c_h_u_r_To_h_ci_r[i17 + (cdiff << 1)] *
                fp_l[cdiff];
            }

            y_data[i17] = d_h_u_l_To_h_ci_l[i17];
            y_data[i17 + 2] = e_h_u_r_To_h_ci_r[i17];
          }

          nm1d2 = anchorIdx * 11 + featureIdx;
          for (i17 = 0; i17 < ndbl; i17++) {
            H->data[b_tmp_data[i17] + H->size[0] * (nm1d2 + 27)] = y_data[i17];
          }
        } else {
          // 'getH_R_res:152' else
          // 'getH_R_res:153' H((res_idx-1)*residualDim + (1:residualDim), 1:3) = h_u_l_To_h_ci_l * h_ci_l_To_r_wc; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i17 = ndbl - 1;
            nm1d2 = i17 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i17 = 0; i17 < ndbl; i17++) {
            b_tmp_data[i17] = (int)(B + y_data[i17]) - 1;
          }

          for (i17 = 0; i17 < 2; i17++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_h_u_l_To_h_ci_l[i17 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_h_u_l_To_h_ci_l[i17 + (cdiff << 1)] += h_u_l_To_h_ci_l[i17 +
                  (nm1d2 << 1)] * h_ci_l_To_r_wc[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i17 = 0; i17 < 3; i17++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * i17] =
                b_h_u_l_To_h_ci_l[cdiff + ndbl * i17];
            }
          }

          // 'getH_R_res:154' H((res_idx-1)*residualDim + (1:residualDim), 4:6) = h_u_l_To_h_ci_l * h_ci_l_To_R_cw; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i17 = ndbl - 1;
            nm1d2 = i17 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i17 = 0; i17 < ndbl; i17++) {
            b_tmp_data[i17] = (int)(B + y_data[i17]) - 1;
          }

          for (i17 = 0; i17 < 2; i17++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_h_u_l_To_h_ci_l[i17 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_h_u_l_To_h_ci_l[i17 + (cdiff << 1)] += h_u_l_To_h_ci_l[i17 +
                  (nm1d2 << 1)] * h_ci_l_To_R_cw[nm1d2 + 3 * cdiff];
              }
            }
          }

          for (i17 = 0; i17 < 3; i17++) {
            for (cdiff = 0; cdiff < ndbl; cdiff++) {
              H->data[b_tmp_data[cdiff] + H->size[0] * (3 + i17)] =
                b_h_u_l_To_h_ci_l[cdiff + ndbl * i17];
            }
          }

          // 'getH_R_res:155' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + int32(1:3)) = h_u_l_To_h_ci_l*h_ci_l_To_anchorPos; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i17 = ndbl - 1;
            nm1d2 = i17 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i17 = 0; i17 < ndbl; i17++) {
            b_tmp_data[i17] = (int)(B + y_data[i17]) - 1;
          }

          for (i17 = 0; i17 < 2; i17++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_h_u_l_To_h_ci_l[i17 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_h_u_l_To_h_ci_l[i17 + (cdiff << 1)] += h_u_l_To_h_ci_l[i17 +
                  (nm1d2 << 1)] * h_ci_l_To_anchorPos[nm1d2 + 3 * cdiff];
              }
            }
          }

          i17 = anchorIdx * 11;
          for (cdiff = 0; cdiff < 3; cdiff++) {
            for (nm1d2 = 0; nm1d2 < ndbl; nm1d2++) {
              H->data[b_tmp_data[nm1d2] + H->size[0] * ((cdiff + i17) + 21)] =
                b_h_u_l_To_h_ci_l[nm1d2 + ndbl * cdiff];
            }
          }

          // 'getH_R_res:156' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + int32(4:6)) = h_u_l_To_h_ci_l*h_ci_l_To_anchorRot; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i17 = ndbl - 1;
            nm1d2 = i17 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i17 = 0; i17 < ndbl; i17++) {
            b_tmp_data[i17] = (int)(B + y_data[i17]) - 1;
          }

          for (i17 = 0; i17 < 2; i17++) {
            for (cdiff = 0; cdiff < 3; cdiff++) {
              b_h_u_l_To_h_ci_l[i17 + (cdiff << 1)] = 0.0;
              for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
                b_h_u_l_To_h_ci_l[i17 + (cdiff << 1)] += h_u_l_To_h_ci_l[i17 +
                  (nm1d2 << 1)] * h_ci_l_To_anchorRot[nm1d2 + 3 * cdiff];
              }
            }
          }

          i17 = anchorIdx * 11;
          for (cdiff = 0; cdiff < 3; cdiff++) {
            for (nm1d2 = 0; nm1d2 < ndbl; nm1d2++) {
              H->data[b_tmp_data[nm1d2] + H->size[0] * ((cdiff + i17) + 24)] =
                b_h_u_l_To_h_ci_l[nm1d2 + ndbl * cdiff];
            }
          }

          // 'getH_R_res:157' H((res_idx-1)*residualDim + (1:residualDim), numStates+(anchorIdx-1)*numErrorStatesPerAnchor + 6 + featureIdx) = h_u_l_To_h_ci_l*h_ci_l_To_rho; 
          ndbl = (int)floor(((double)residualDim - 1.0) + 0.5);
          apnd = ndbl + 1;
          cdiff = (ndbl - residualDim) + 1;
          if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)residualDim)
          {
            ndbl++;
            apnd = residualDim;
          } else if (cdiff > 0) {
            apnd = ndbl;
          } else {
            ndbl++;
          }

          y_data[0] = 1.0;
          if (ndbl > 1) {
            y_data[ndbl - 1] = apnd;
            i17 = ndbl - 1;
            nm1d2 = i17 / 2;
            cdiff = 1;
            while (cdiff <= nm1d2 - 1) {
              y_data[1] = 2.0;
              y_data[ndbl - 2] = (double)apnd - 1.0;
              cdiff = 2;
            }

            if (nm1d2 << 1 == ndbl - 1) {
              y_data[nm1d2] = (1.0 + (double)apnd) / 2.0;
            } else {
              y_data[nm1d2] = 1.0 + (double)nm1d2;
              y_data[nm1d2 + 1] = apnd - nm1d2;
            }
          }

          B = ((double)res_idx - 1.0) * (double)residualDim;
          for (i17 = 0; i17 < ndbl; i17++) {
            b_tmp_data[i17] = (int)(B + y_data[i17]) - 1;
          }

          for (i17 = 0; i17 < 2; i17++) {
            d_h_u_l_To_h_ci_l[i17] = 0.0;
            for (cdiff = 0; cdiff < 3; cdiff++) {
              d_h_u_l_To_h_ci_l[i17] += h_u_l_To_h_ci_l[i17 + (cdiff << 1)] *
                fp_r[cdiff];
            }
          }

          nm1d2 = anchorIdx * 11 + featureIdx;
          for (i17 = 0; i17 < ndbl; i17++) {
            H->data[b_tmp_data[i17] + H->size[0] * (nm1d2 + 27)] =
              d_h_u_l_To_h_ci_l[i17];
          }
        }

        //              if coder.target('MATLAB')
        //
        //                  h_ci_l_To_r_wc = -rho*R_cw;
        //                  h_ci_l_To_R_cw = skew(h_ci_l);
        //
        //                  if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 || rho < 0 % delayed initialization or feature behind anchor 
        //                      H_robot = [zeros(3), 0*h_ci_l_To_R_cw, zeros(3, numStates-6)]; 
        //                  else
        //                      H_robot = [h_ci_l_To_r_wc, h_ci_l_To_R_cw, zeros(3, numStates-6)]; 
        //                  end
        //
        //                  %% anchor state derivatives
        //
        //                  h_ci_l_To_anchorPos = rho*R_cw;
        //                  h_ci_l_To_anchorRot = -R_cw * skew(anchorRot'*m);
        //                  h_ci_l_To_rho       = R_cw * (anchorPos - r_wc);
        //
        //                  if xt.anchor_states(anchorIdx).feature_states(featureIdx).status == 2 || rho < 0 % delayed initialization or feature behind anchor 
        //                      H_map = [zeros(3, 6 + featureIdx-1), h_ci_l_To_rho, zeros(3, numPointsPerAnchor - featureIdx)]; 
        //                  else
        //                      H_map = [h_ci_l_To_anchorPos, h_ci_l_To_anchorRot, zeros(3, featureIdx-1), h_ci_l_To_rho, zeros(3, numPointsPerAnchor - featureIdx)]; 
        //                  end
        //
        //                  H_t = h_u_To_h_ci_l * [H_robot, zeros(3, (anchorIdx-1)*numErrorStatesPerAnchor), H_map, zeros(3, (numAnchors - anchorIdx)*numErrorStatesPerAnchor)]; 
        //
        //                  if any(any(abs(H((res_idx-1)*residualDim + (1:residualDim), :) - H_t) > 1e-8)) 
        //                      figure; imagesc(H_t - H((res_idx-1)*residualDim + (1:residualDim), :)) 
        //                      error('H inconsistency')
        //                  end
        //              end
        // 'getH_R_res:191' ind(res_idx, 1) = anchorIdx;
        ind->data[(int)res_idx - 1] = anchorIdx + 1;

        // 'getH_R_res:192' ind(res_idx, 2) = featureIdx;
        ind->data[((int)res_idx + ind->size[0]) - 1] = featureIdx + 1;

        // 'getH_R_res:194' res_idx = res_idx + 1;
        res_idx++;
      }
    }
  }

  //  r = z - h_u;
  //  R = diag(repmat((noiseParameters.image_noise), 1, numMeas*2));
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
//                const f_struct_T xt_anchor_states[8]
//                double b_map[120]
// Return Type  : void
//
static void getMap(const double xt_origin_pos[3], const double xt_origin_att[4],
                   const f_struct_T xt_anchor_states[8], double b_map[120])
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
  for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
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
    for (featureIdx = 0; featureIdx < 5; featureIdx++) {
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
// Arguments    : const e_struct_T anchor_state_feature_states[5]
// Return Type  : double
//
static double getNumValidFeatures(const e_struct_T anchor_state_feature_states[5])
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
  for (featureIdx = 0; featureIdx < 5; featureIdx++) {
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
  int i60;
  double d11;
  int i61;

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
  for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
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
    for (featureIdx = 0; featureIdx < 5; featureIdx++) {
      // 'getScaledMap:29' if xt.anchor_states(anchorIdx).feature_states(featureIdx).status 
      if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status != 0)
      {
        // 'getScaledMap:30' rho = xt.anchor_states(anchorIdx).feature_states(featureIdx).inverse_depth; 
        // 'getScaledMap:32' m = xt.anchor_states(anchorIdx).feature_states(featureIdx).m; 
        // 'getScaledMap:34' fp_scaled = R_cw*(rho*anchorPos + anchorRot'*m - xt.robot_state.pos*rho); 
        // 'getScaledMap:36' xt.anchor_states(anchorIdx).feature_states(featureIdx).scaled_map_point = fp_scaled; 
        for (i60 = 0; i60 < 3; i60++) {
          d11 = 0.0;
          for (i61 = 0; i61 < 3; i61++) {
            d11 += anchorRot[i61 + 3 * i60] * b_xt->anchor_states[anchorIdx].
              feature_states[featureIdx].m[i61];
          }

          c_xt[i60] = (b_xt->anchor_states[anchorIdx].feature_states[featureIdx]
                       .inverse_depth * b_xt->anchor_states[anchorIdx].pos[i60]
                       + d11) - b_xt->robot_state.pos[i60] * b_xt->
            anchor_states[anchorIdx].feature_states[featureIdx].inverse_depth;
        }

        for (i60 = 0; i60 < 3; i60++) {
          b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
            scaled_map_point[i60] = 0.0;
          for (i61 = 0; i61 < 3; i61++) {
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              scaled_map_point[i60] += R_cw[i60 + 3 * i61] * c_xt[i61];
          }
        }
      }
    }
  }
}

//
// getTotalNumActiveFeatures Get the number of active features of all anchors
// Arguments    : const f_struct_T xt_anchor_states[8]
// Return Type  : double
//
static double getTotalNumActiveFeatures(const f_struct_T xt_anchor_states[8])
{
  double n;
  int anchorIdx;
  double b_n;
  int featureIdx;

  // 'getTotalNumActiveFeatures:4' n = 0;
  n = 0.0;

  // 'getTotalNumActiveFeatures:5' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
    // 'getTotalNumActiveFeatures:6' n = n + getNumActiveFeatures(xt.anchor_states(anchorIdx)); 
    // getNumActiveFeatures Get the number of active features of an anchor
    // 'getNumActiveFeatures:4' n = getNumFeaturesOfType(anchor_state, 1);
    // getNumFeaturesOfType Get the number of features of type type of an anchor 
    //  type can be a scalar or a row vector of types
    // 'getNumFeaturesOfType:5' n = 0;
    b_n = 0.0;

    // 'getNumFeaturesOfType:6' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 5; featureIdx++) {
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
// Arguments    : const f_struct_T xt_anchor_states[8]
// Return Type  : double
//
static double getTotalNumDelayedFeatures(const f_struct_T xt_anchor_states[8])
{
  double n;
  int anchorIdx;
  double b_n;
  int featureIdx;

  // 'getTotalNumDelayedFeatures:4' n = 0;
  n = 0.0;

  // 'getTotalNumDelayedFeatures:5' for anchorIdx = 1:numAnchors
  for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
    // 'getTotalNumDelayedFeatures:6' n = n + getNumFeaturesOfType(xt.anchor_states(anchorIdx), 2); 
    // getNumFeaturesOfType Get the number of features of type type of an anchor 
    //  type can be a scalar or a row vector of types
    // 'getNumFeaturesOfType:5' n = 0;
    b_n = 0.0;

    // 'getNumFeaturesOfType:6' for featureIdx = 1:numPointsPerAnchor
    for (featureIdx = 0; featureIdx < 5; featureIdx++) {
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
  double d6;
  int i42;
  int i43;

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
    d6 = 0.0;
    for (i42 = 0; i42 < 3; i42++) {
      d6 += R_ow[i42 + 3 * i] * xt_robot_state_pos[i42];
      b_xt_robot_state_att[i + 3 * i42] = 0.0;
      for (i43 = 0; i43 < 3; i43++) {
        b_xt_robot_state_att[i + 3 * i42] += c_xt_robot_state_att[i + 3 * i43] *
          R_ow[i43 + 3 * i42];
      }
    }

    world_state_pos[i] = xt_origin_pos[i] + d6;
    world_state_vel[i] = 0.0;
    for (i42 = 0; i42 < 3; i42++) {
      world_state_vel[i] += R_ow[i42 + 3 * i] * xt_robot_state_vel[i42];
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
  static const char cfmt[38] = { '\x09', 'g', 'y', 'r', 'o', ' ', 'b', 'i', 'a',
    's', ' ', 'i', 'n', 'i', 't', 'i', 'a', 'l', ' ', 'u', 'n', 'c', ':', ' ',
    '[', '%', 'f', ',', ' ', '%', 'f', ',', ' ', '%', 'f', ']', '\x0a', '\x00' };

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
  char cv46[27];
  int i38;
  static const char cv47[27] = { 'R', 'e', 'q', 'u', 'e', 's', 't', 'i', 'n',
    'g', ' ', '%', 'd', ' ', 'n', 'e', 'w', ' ', 'f', 'e', 'a', 't', 'u', 'r',
    'e', 's', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_info:5' if coder.target('MATLAB')
  // 'ros_info:9' elseif ~coder.target('MEX')
  // 'ros_info:10' coder.cinclude('<ros/console.h>')
  // 'ros_info:11' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_info:12' coder.ceval('ROS_INFO', [str, 0], varargin{:});
    for (i38 = 0; i38 < 27; i38++) {
      cv46[i38] = cv47[i38];
    }

    ROS_INFO(cv46, varargin_1);
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
  char cv40[57];
  int i35;
  static const char cv41[57] = { 'F', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%',
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
    for (i35 = 0; i35 < 57; i35++) {
      cv40[i35] = cv41[i35];
    }

    ROS_WARN(cv40, varargin_1, varargin_2, varargin_3);
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
  static const char cfmt[37] = { '\x09', 'a', 'c', 'c', ' ', 'b', 'i', 'a', 's',
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
static void i_ros_info(int varargin_1)
{
  char cv50[28];
  int i41;
  static const char cv51[28] = { 'S', 'e', 't', 't', 'i', 'n', 'g', ' ', 'a',
    'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ' ', 'a', 's', ' ', 'o', 'r', 'i',
    'g', 'i', 'n', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  // 'ros_info:5' if coder.target('MATLAB')
  // 'ros_info:9' elseif ~coder.target('MEX')
  // 'ros_info:10' coder.cinclude('<ros/console.h>')
  // 'ros_info:11' if debug_level >= 2
  if (debug_level >= 2.0) {
    // 'ros_info:12' coder.ceval('ROS_INFO', [str, 0], varargin{:});
    for (i41 = 0; i41 < 28; i41++) {
      cv50[i41] = cv51[i41];
    }

    ROS_INFO(cv50, varargin_1);
  }
}

//
// ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
// Arguments    : int varargin_1
// Return Type  : void
//
static void i_ros_warn(int varargin_1)
{
  char cv42[52];
  int i36;
  static const char cv43[52] = { 'T', 'r', 'y', 'i', 'n', 'g', ' ', 't', 'o',
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
    for (i36 = 0; i36 < 52; i36++) {
      cv42[i36] = cv43[i36];
    }

    ROS_WARN(cv42, varargin_1);
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
  int i28;
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
    for (i28 = 0; i28 < 3; i28++) {
      rot[i28 + 3 * k] = iv0[i28 + 3 * k];
      rot[i28 + 3 * (k + 3)] = cameraparams_R_lr[k + 3 * i28];
    }

    b_m[k] = ml[k] / tol;
    b_m[3 + k] = mr[k] / B;
  }

  //  normalized rays in left frame
  // 'initializePoint:34' ml_n_l = m(:,1);
  // 'initializePoint:35' mr_n_r = cameraparams.R_lr * m(:,2);
  for (k = 0; k < 3; k++) {
    ml[k] = 0.0;
    for (i28 = 0; i28 < 3; i28++) {
      ml[k] += cameraparams_R_lr[k + 3 * i28] * b_m[3 + i28];
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
        for (i28 = 0; i28 < 3; i28++) {
          b_rot[i28 + 3 * k] = rot[k + 3 * (i28 + rankR)];
        }
      }

      rankR = anchorIdx * 3;
      for (k = 0; k < 3; k++) {
        A[(k + rankR) + 6 * anchorIdx] = 0.0;
        for (i28 = 0; i28 < 3; i28++) {
          A[(k + rankR) + 6 * anchorIdx] += b_rot[k + 3 * i28] * b_m[i28 + 3 *
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
        for (i28 = 0; i28 < 3; i28++) {
          A[(i28 + rankR) + 6 * (2 + k)] = -(double)I[i28 + 3 * k];
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
  static const char cfmt[18] = { '\x09', 'i', 'm', 'a', 'g', 'e', '_', 'n', 'o',
    'i', 's', 'e', ':', ' ', '%', 'f', '\x0a', '\x00' };

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
  char cv48[54];
  int i40;
  static const char cv49[54] = { 'C', 'a', 'n', '\'', 't', ' ', 'f', 'i', 'x',
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
    for (i40 = 0; i40 < 54; i40++) {
      cv48[i40] = cv49[i40];
    }

    ROS_WARN(cv48);
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
  static const char cfmt[28] = { '\x09', 'i', 'n', 'v', '_', 'd', 'e', 'p', 't',
    'h', '_', 'i', 'n', 'i', 't', 'i', 'a', 'l', '_', 'u', 'n', 'c', ':', ' ',
    '%', 'f', '\x0a', '\x00' };

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
  static const char cfmt[18] = { '\x09', 'n', 'u', 'm', '_', 'a', 'n', 'c', 'h',
    'o', 'r', 's', ':', ' ', '%', 'd', '\x0a', '\x00' };

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
static double m_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[28] = { '\x09', 'n', 'u', 'm', '_', 'p', 'o', 'i', 'n',
    't', 's', '_', 'p', 'e', 'r', '_', 'a', 'n', 'c', 'h', 'o', 'r', ':', ' ',
    '%', 'd', '\x0a', '\x00' };

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

    emxInit_int32_T(&iwork, 1);
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
// Arguments    : int idx[5]
//                double x[5]
//                int offset
//                int np
//                int nq
//                int iwork[5]
//                double xwork[5]
// Return Type  : void
//
static void merge(int idx[5], double x[5], int offset, int np, int nq, int
                  iwork[5], double xwork[5])
{
  int n;
  int iout;
  int p;
  int exitg1;
  if (nq == 0) {
  } else {
    n = np + nq;
    for (iout = 0; iout + 1 <= n; iout++) {
      iwork[iout] = idx[offset + iout];
      xwork[iout] = x[offset + iout];
    }

    p = 0;
    n = np + nq;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork[p] <= xwork[4]) {
        idx[iout] = iwork[p];
        x[iout] = xwork[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx[iout] = iwork[4];
        x[iout] = xwork[4];
        if (5 < n) {
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
// Arguments    : emxArray_real_T *A
//                const emxArray_real_T *B
// Return Type  : void
//
static void mrdivide(emxArray_real_T *A, const emxArray_real_T *B)
{
  emxArray_real_T *Y;
  emxArray_real_T *b_A;
  emxArray_real_T *tau;
  emxArray_int32_T *jpvt;
  emxArray_real_T *b_B;
  unsigned int unnamed_idx_1;
  int jAcol;
  int mn;
  int b_m;
  int rankR;
  int jBcol;
  int j;
  int k;
  int i;
  double wj;
  emxInit_real_T1(&Y, 2);
  emxInit_real_T1(&b_A, 2);
  emxInit_real_T(&tau, 1);
  emxInit_int32_T1(&jpvt, 2);
  emxInit_real_T1(&b_B, 2);
  if ((A->size[1] == 0) || ((B->size[0] == 0) || (B->size[1] == 0))) {
    unnamed_idx_1 = (unsigned int)B->size[0];
    jAcol = A->size[0] * A->size[1];
    A->size[0] = 109;
    A->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)A, jAcol, (int)sizeof(double));
    mn = (int)unnamed_idx_1;
    for (jAcol = 0; jAcol < mn; jAcol++) {
      for (b_m = 0; b_m < 109; b_m++) {
        A->data[b_m + A->size[0] * jAcol] = 0.0;
      }
    }
  } else if (B->size[0] == B->size[1]) {
    rankR = B->size[1];
    jAcol = b_A->size[0] * b_A->size[1];
    b_A->size[0] = B->size[0];
    b_A->size[1] = B->size[1];
    emxEnsureCapacity((emxArray__common *)b_A, jAcol, (int)sizeof(double));
    mn = B->size[0] * B->size[1];
    for (jAcol = 0; jAcol < mn; jAcol++) {
      b_A->data[jAcol] = B->data[jAcol];
    }

    xgetrf(B->size[1], B->size[1], b_A, B->size[1], jpvt, &jBcol);
    if (A->size[1] == 0) {
    } else {
      for (j = 0; j + 1 <= rankR; j++) {
        jBcol = 109 * j;
        jAcol = rankR * j;
        for (k = 1; k <= j; k++) {
          b_m = 109 * (k - 1);
          if (b_A->data[(k + jAcol) - 1] != 0.0) {
            for (i = 0; i < 109; i++) {
              A->data[i + jBcol] -= b_A->data[(k + jAcol) - 1] * A->data[i + b_m];
            }
          }
        }

        wj = 1.0 / b_A->data[j + jAcol];
        for (i = 0; i < 109; i++) {
          A->data[i + jBcol] *= wj;
        }
      }
    }

    if (A->size[1] == 0) {
    } else {
      for (j = B->size[1]; j > 0; j--) {
        jBcol = 109 * (j - 1);
        jAcol = rankR * (j - 1) - 1;
        for (k = j + 1; k <= rankR; k++) {
          b_m = 109 * (k - 1);
          if (b_A->data[k + jAcol] != 0.0) {
            for (i = 0; i < 109; i++) {
              A->data[i + jBcol] -= b_A->data[k + jAcol] * A->data[i + b_m];
            }
          }
        }
      }
    }

    for (jBcol = B->size[1] - 2; jBcol + 1 > 0; jBcol--) {
      if (jpvt->data[jBcol] != jBcol + 1) {
        b_m = jpvt->data[jBcol] - 1;
        for (mn = 0; mn < 109; mn++) {
          wj = A->data[mn + A->size[0] * jBcol];
          A->data[mn + A->size[0] * jBcol] = A->data[mn + A->size[0] * b_m];
          A->data[mn + A->size[0] * b_m] = wj;
        }
      }
    }
  } else {
    jAcol = b_A->size[0] * b_A->size[1];
    b_A->size[0] = B->size[1];
    b_A->size[1] = B->size[0];
    emxEnsureCapacity((emxArray__common *)b_A, jAcol, (int)sizeof(double));
    mn = B->size[0];
    for (jAcol = 0; jAcol < mn; jAcol++) {
      jBcol = B->size[1];
      for (b_m = 0; b_m < jBcol; b_m++) {
        b_A->data[b_m + b_A->size[0] * jAcol] = B->data[jAcol + B->size[0] * b_m];
      }
    }

    xgeqp3(b_A, tau, jpvt);
    rankR = rankFromQR(b_A);
    jBcol = b_A->size[1];
    jAcol = Y->size[0] * Y->size[1];
    Y->size[0] = jBcol;
    Y->size[1] = 109;
    emxEnsureCapacity((emxArray__common *)Y, jAcol, (int)sizeof(double));
    mn = jBcol * 109;
    for (jAcol = 0; jAcol < mn; jAcol++) {
      Y->data[jAcol] = 0.0;
    }

    jAcol = b_B->size[0] * b_B->size[1];
    b_B->size[0] = A->size[1];
    b_B->size[1] = 109;
    emxEnsureCapacity((emxArray__common *)b_B, jAcol, (int)sizeof(double));
    for (jAcol = 0; jAcol < 109; jAcol++) {
      mn = A->size[1];
      for (b_m = 0; b_m < mn; b_m++) {
        b_B->data[b_m + b_B->size[0] * jAcol] = A->data[jAcol + A->size[0] * b_m];
      }
    }

    b_m = b_A->size[0];
    jBcol = b_A->size[0];
    mn = b_A->size[1];
    if (jBcol <= mn) {
      mn = jBcol;
    }

    for (j = 0; j + 1 <= mn; j++) {
      if (tau->data[j] != 0.0) {
        for (k = 0; k < 109; k++) {
          wj = b_B->data[j + b_B->size[0] * k];
          for (i = j + 1; i + 1 <= b_m; i++) {
            wj += b_A->data[i + b_A->size[0] * j] * b_B->data[i + b_B->size[0] *
              k];
          }

          wj *= tau->data[j];
          if (wj != 0.0) {
            b_B->data[j + b_B->size[0] * k] -= wj;
            for (i = j + 1; i + 1 <= b_m; i++) {
              b_B->data[i + b_B->size[0] * k] -= b_A->data[i + b_A->size[0] * j]
                * wj;
            }
          }
        }
      }
    }

    for (k = 0; k < 109; k++) {
      for (i = 0; i + 1 <= rankR; i++) {
        Y->data[(jpvt->data[i] + Y->size[0] * k) - 1] = b_B->data[i + b_B->size
          [0] * k];
      }

      for (j = rankR - 1; j + 1 > 0; j--) {
        Y->data[(jpvt->data[j] + Y->size[0] * k) - 1] /= b_A->data[j + b_A->
          size[0] * j];
        for (i = 0; i + 1 <= j; i++) {
          Y->data[(jpvt->data[i] + Y->size[0] * k) - 1] -= Y->data[(jpvt->data[j]
            + Y->size[0] * k) - 1] * b_A->data[i + b_A->size[0] * j];
        }
      }
    }

    jAcol = A->size[0] * A->size[1];
    A->size[0] = 109;
    A->size[1] = Y->size[0];
    emxEnsureCapacity((emxArray__common *)A, jAcol, (int)sizeof(double));
    mn = Y->size[0];
    for (jAcol = 0; jAcol < mn; jAcol++) {
      for (b_m = 0; b_m < 109; b_m++) {
        A->data[b_m + A->size[0] * jAcol] = Y->data[jAcol + Y->size[0] * b_m];
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
  int i11;
  int i;
  int j;

  // 'multiplyIdx:7' idx_mult = zeros(length(idx) * factor, 1);
  idx_mult_size[0] = idx_size[0] << 1;
  loop_ub = idx_size[0] << 1;
  for (i11 = 0; i11 < loop_ub; i11++) {
    idx_mult_data[i11] = 0.0;
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
  static const char cfmt[25] = { '\x09', 'm', 'a', 'x', '_', 'e', 'k', 'f', '_',
    'i', 't', 'e', 'r', 'a', 't', 'i', 'o', 'n', 's', ':', ' ', '%', 'd', '\x0a',
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
  static const char cfmt[29] = { '\x09', 'd', 'e', 'l', 'a', 'y', 'e', 'd', '_',
    'i', 'n', 'i', 't', 'i', 'a', 'l', 'i', 'z', 'a', 't', 'i', 'o', 'n', ':',
    ' ', '%', 's', '\x0a', '\x00' };

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
  static const char cfmt[20] = { '\x09', 'f', 'i', 'x', 'e', 'd', '_', 'f', 'e',
    'a', 't', 'u', 'r', 'e', ':', ' ', '%', 's', '\x0a', '\x00' };

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
  double d4;
  int i19;

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
    d4 = 0.0;
    for (i19 = 0; i19 < 3; i19++) {
      d4 += stereoParams_R_rl[i + 3 * i19] * fp_l[i19];
    }

    fp_r[i] = d4 - stereoParams_r_lr[i];
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
//                boolean_T VIOParameters_full_stereo
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
  boolean_T VIOParameters_mono, boolean_T VIOParameters_full_stereo, boolean_T
  VIOParameters_RANSAC)
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

  // 'SLAM:125' fprintf('\n');
  b_fprintf();

  // 'SLAM:126' ros_info('Noise parameters:');
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

  // 'SLAM:127' fprintf('\tqv: %f\n', noiseParameters.process_noise.qv);
  c_fprintf(c_noiseParameters_process_noise);

  // 'SLAM:128' fprintf('\tqw: %f\n', noiseParameters.process_noise.qw);
  d_fprintf(d_noiseParameters_process_noise);

  // 'SLAM:129' fprintf('\tqao: %f\n', noiseParameters.process_noise.qao);
  e_fprintf(e_noiseParameters_process_noise);

  // 'SLAM:130' fprintf('\tqwo: %f\n', noiseParameters.process_noise.qwo);
  f_fprintf(f_noiseParameters_process_noise);

  // 'SLAM:131' fprintf('\tqR_ci: %f\n', noiseParameters.process_noise.qR_ci);
  g_fprintf(g_noiseParameters_process_noise);

  // 'SLAM:132' fprintf('\tgyro bias initial unc: [%f, %f, %f]\n', noiseParameters.gyro_bias_initial_unc(1), noiseParameters.gyro_bias_initial_unc(2), noiseParameters.gyro_bias_initial_unc(3)); 
  h_fprintf(c_noiseParameters_gyro_bias_ini[0], c_noiseParameters_gyro_bias_ini
            [1], c_noiseParameters_gyro_bias_ini[2]);

  // 'SLAM:133' fprintf('\tacc bias initial unc: [%f, %f, %f]\n', noiseParameters.acc_bias_initial_unc(1), noiseParameters.acc_bias_initial_unc(2), noiseParameters.acc_bias_initial_unc(3)); 
  i_fprintf(c_noiseParameters_acc_bias_init[0], c_noiseParameters_acc_bias_init
            [1], c_noiseParameters_acc_bias_init[2]);

  // 'SLAM:134' fprintf('\timage_noise: %f\n', noiseParameters.image_noise);
  j_fprintf(noiseParameters_image_noise);

  // 'SLAM:135' fprintf('\tinv_depth_initial_unc: %f\n', noiseParameters.inv_depth_initial_unc); 
  k_fprintf(c_noiseParameters_inv_depth_ini);

  // 'SLAM:137' fprintf('\n');
  b_fprintf();

  // 'SLAM:138' ros_info('VIO parameters');
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

  // 'SLAM:139' fprintf('\tnum_anchors: %d\n', int32(VIOParameters.num_anchors)); 
  l_fprintf(VIOParameters_num_anchors);

  // 'SLAM:140' fprintf('\tnum_points_per_anchor: %d\n', int32(VIOParameters.num_points_per_anchor)); 
  m_fprintf(c_VIOParameters_num_points_per_);

  // 'SLAM:141' fprintf('\tmax_ekf_iterations: %d\n', int32(VIOParameters.max_ekf_iterations)); 
  n_fprintf(c_VIOParameters_max_ekf_iterati);

  // 'SLAM:142' fprintf('\tdelayed_initialization: %s\n', bool2str(VIOParameters.delayed_initialization)); 
  // 'SLAM:151' if bool
  if (c_VIOParameters_delayed_initial) {
    // 'SLAM:152' s = 'True';
    s_size[0] = 1;
    s_size[1] = 4;
    for (i3 = 0; i3 < 4; i3++) {
      s_data[i3] = cv4[i3];
    }
  } else {
    // 'SLAM:153' else
    // 'SLAM:154' s = 'False';
    s_size[0] = 1;
    s_size[1] = 5;
    for (i3 = 0; i3 < 5; i3++) {
      s_data[i3] = cv5[i3];
    }
  }

  o_fprintf(s_data, s_size);

  // 'SLAM:143' fprintf('\tfixed_feature: %s\n', bool2str(VIOParameters.fixed_feature)); 
  // 'SLAM:151' if bool
  if (VIOParameters_fixed_feature) {
    // 'SLAM:152' s = 'True';
    s_size[0] = 1;
    s_size[1] = 4;
    for (i3 = 0; i3 < 4; i3++) {
      s_data[i3] = cv4[i3];
    }
  } else {
    // 'SLAM:153' else
    // 'SLAM:154' s = 'False';
    s_size[0] = 1;
    s_size[1] = 5;
    for (i3 = 0; i3 < 5; i3++) {
      s_data[i3] = cv5[i3];
    }
  }

  p_fprintf(s_data, s_size);

  // 'SLAM:144' fprintf('\tmono: %s\n', bool2str(VIOParameters.mono));
  // 'SLAM:151' if bool
  if (VIOParameters_mono) {
    // 'SLAM:152' s = 'True';
    s_size[0] = 1;
    s_size[1] = 4;
    for (i3 = 0; i3 < 4; i3++) {
      s_data[i3] = cv4[i3];
    }
  } else {
    // 'SLAM:153' else
    // 'SLAM:154' s = 'False';
    s_size[0] = 1;
    s_size[1] = 5;
    for (i3 = 0; i3 < 5; i3++) {
      s_data[i3] = cv5[i3];
    }
  }

  q_fprintf(s_data, s_size);

  // 'SLAM:145' fprintf('\tfull_stereo: %s\n', bool2str(VIOParameters.full_stereo)); 
  // 'SLAM:151' if bool
  if (VIOParameters_full_stereo) {
    // 'SLAM:152' s = 'True';
    s_size[0] = 1;
    s_size[1] = 4;
    for (i3 = 0; i3 < 4; i3++) {
      s_data[i3] = cv4[i3];
    }
  } else {
    // 'SLAM:153' else
    // 'SLAM:154' s = 'False';
    s_size[0] = 1;
    s_size[1] = 5;
    for (i3 = 0; i3 < 5; i3++) {
      s_data[i3] = cv5[i3];
    }
  }

  r_fprintf(s_data, s_size);

  // 'SLAM:146' fprintf('\tRANSAC: %s\n', bool2str(VIOParameters.RANSAC));
  // 'SLAM:151' if bool
  if (VIOParameters_RANSAC) {
    // 'SLAM:152' s = 'True';
    s_size[0] = 1;
    s_size[1] = 4;
    for (i3 = 0; i3 < 4; i3++) {
      s_data[i3] = cv4[i3];
    }
  } else {
    // 'SLAM:153' else
    // 'SLAM:154' s = 'False';
    s_size[0] = 1;
    s_size[1] = 5;
    for (i3 = 0; i3 < 5; i3++) {
      s_data[i3] = cv5[i3];
    }
  }

  s_fprintf(s_data, s_size);

  // 'SLAM:147' fprintf('\n');
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
  static const char cfmt[11] = { '\x09', 'm', 'o', 'n', 'o', ':', ' ', '%', 's',
    '\x0a', '\x00' };

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
  int i9;
  int i10;
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
  for (i9 = 0; i9 < 4; i9++) {
    qp[i9] = 0.0;
    for (i10 = 0; i10 < 4; i10++) {
      qp[i9] += b_p[i9 + (i10 << 2)] * b_q[i10];
    }
  }

  // 'quatmultJ:23' qp = qp/norm(qp);
  B = b_norm(qp);
  for (i9 = 0; i9 < 4; i9++) {
    qp[i9] /= B;
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
  static const char cfmt[18] = { '\x09', 'f', 'u', 'l', 'l', '_', 's', 't', 'e',
    'r', 'e', 'o', ':', ' ', '%', 's', '\x0a', '\x00' };

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
// Arguments    : const emxArray_real_T *A
// Return Type  : int
//
static int rankFromQR(const emxArray_real_T *A)
{
  int r;
  int minmn;
  int maxmn;
  double tol;
  r = 0;
  if (A->size[0] < A->size[1]) {
    minmn = A->size[0];
    maxmn = A->size[1];
  } else {
    minmn = A->size[1];
    maxmn = A->size[0];
  }

  if (minmn > 0) {
    tol = (double)maxmn * fabs(A->data[0]) * 2.2204460492503131E-16;
    while ((r < minmn) && (fabs(A->data[r + A->size[0] * r]) >= tol)) {
      r++;
    }
  }

  return r;
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
  int i15;
  static const char cv13[27] = { 'p', 'i', 'c', 'k', 'e', 'd', ' ', 'a', 'n',
    ' ', 'i', 'n', 'a', 'c', 't', 'i', 'v', 'e', ' ', 'f', 'e', 'a', 't', 'u',
    'r', 'e', '\x00' };

  // 'ros_error:4' if coder.target('MATLAB')
  // 'ros_error:6' elseif ~coder.target('MEX')
  // 'ros_error:7' coder.cinclude('<ros/console.h>')
  // 'ros_error:8' coder.ceval('ROS_ERROR', [str, 0], varargin{:});
  for (i15 = 0; i15 < 27; i15++) {
    cv12[i15] = cv13[i15];
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
  int i14;
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
    for (i14 = 0; i14 < 54; i14++) {
      cv10[i14] = cv11[i14];
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
  int i22;
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
    for (i22 = 0; i22 < 36; i22++) {
      cv16[i22] = cv17[i22];
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
  double d8;
  double d9;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d8 = fabs(u0);
    d9 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d8 == 1.0) {
        y = rtNaN;
      } else if (d8 > 1.0) {
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
    } else if (d9 == 0.0) {
      y = 1.0;
    } else if (d9 == 1.0) {
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
// Arguments    : const char varargin_1_data[]
//                const int varargin_1_size[2]
// Return Type  : double
//
static double s_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2])
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  int i8;
  char varargout_1_data[6];
  static const char cfmt[13] = { '\x09', 'R', 'A', 'N', 'S', 'A', 'C', ':', ' ',
    '%', 's', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = varargin_1_size[1];
    for (i8 = 0; i8 < nbytesint; i8++) {
      varargout_1_data[i8] = varargin_1_data[i8];
    }

    varargout_1_data[varargin_1_size[1]] = '\x00';
    nbytesint = fprintf(filestar, cfmt, &varargout_1_data[0]);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : double x[5]
//                int idx[5]
// Return Type  : void
//
static void sort(double x[5], int idx[5])
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

  emxInit_int32_T(&iwork, 1);
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
//                int cameraparams_ATAN
//                const double cameraparams_FocalLength[2]
//                const double cameraparams_PrincipalPoint[2]
//                const double cameraparams_RadialDistortion[3]
//                int cameraparams_DistortionModel
//                double pt_u_data[]
//                int pt_u_size[1]
// Return Type  : void
//
static void undistortPoint(const double pt_d_data[], const int pt_d_size[1], int
  cameraparams_ATAN, const double cameraparams_FocalLength[2], const double
  cameraparams_PrincipalPoint[2], const double cameraparams_RadialDistortion[3],
  int cameraparams_DistortionModel, double pt_u_data[], int pt_u_size[1])
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
  int b_i;
  int c_i;
  boolean_T exitg1;
  double a;
  double b_a;
  double diff;
  char cv6[48];
  static const char cv7[48] = { 'P', 'i', 'x', 'e', 'l', ' ', 'r', 'a', 'd', 'i',
    'u', 's', ' ', 'i', 'n', 'v', 'e', 'r', 's', 'i', 'o', 'n', ':', ' ', 'R',
    'e', 'a', 'c', 'h', 'e', 'd', ' ', 'i', 't', 'e', 'r', 'a', 't', 'i', 'o',
    'n', ' ', 'l', 'i', 'm', 'i', 't', '\x00' };

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
    for (i12 = 0; i12 < loop_ub; i12++) {
      pt_u_data[i12] = pt_d_data[i12];
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
        // 'get_r_u:20' ros_warn('Pixel radius inversion: Reached iteration limit') 
        // ROS_WARN Print to ROS_WARN in ROS or to console in Matlab
        //  debug_level == 0: print errors, == 1: print warnings, == 2: print info 
        // 'ros_warn:6' if coder.target('MATLAB')
        // 'ros_warn:10' elseif ~coder.target('MEX')
        // 'ros_warn:11' coder.cinclude('<ros/console.h>')
        // 'ros_warn:12' if debug_level >= 2
        // 'ros_warn:13' coder.ceval('ROS_WARN', [str, 0], varargin{:});
        for (i12 = 0; i12 < 48; i12++) {
          cv6[i12] = cv7[i12];
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
        for (i12 = 0; i12 < 52; i12++) {
          cv8[i12] = cv9[i12];
        }

        ROS_WARN(cv8, r_d_sq, r_u_sq);
      }

      // 'undistortPoint:80' coeff = (1 + k1*r_u_sq + k2*r_u_sq^2 + k3*r_u_sq^3); 
      coeff = ((1.0 + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq)) + k3 * rt_powd_snf
        (r_u_sq, 3.0);

      // 'undistortPoint:82' pt_u_n = pt_d_n / coeff;
      for (i12 = 0; i12 < 2; i12++) {
        pt_d_n[i12] /= coeff;
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
  int i13;
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
  for (i13 = 0; i13 < rankR; i13++) {
    pt_u_data[i13] = pt_d_data[i13];
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
      for (i13 = 0; i13 < 3; i13++) {
        b_x[i13] = x[i13];
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
      for (i13 = 0; i13 < 2; i13++) {
        pt_d_n[i13] = pt_d_n[i13] * r_u / r_d;
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
      for (i13 = 0; i13 < 2; i13++) {
        pt_u_data[i13 + rankR] = pt_d_data[i13 + knt];
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
  int i64;
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

  i64 = tau->size[0];
  tau->size[0] = mn;
  emxEnsureCapacity((emxArray__common *)tau, i64, (int)sizeof(double));
  eml_signed_integer_colon(A->size[1], jpvt);
  if ((A->size[0] == 0) || (A->size[1] == 0)) {
  } else {
    emxInit_real_T(&work, 1);
    itemp = A->size[1];
    i64 = work->size[0];
    work->size[0] = itemp;
    emxEnsureCapacity((emxArray__common *)work, i64, (int)sizeof(double));
    for (i64 = 0; i64 < itemp; i64++) {
      work->data[i64] = 0.0;
    }

    emxInit_real_T(&vn1, 1);
    emxInit_real_T(&vn2, 1);
    itemp = A->size[1];
    i64 = vn1->size[0];
    vn1->size[0] = itemp;
    emxEnsureCapacity((emxArray__common *)vn1, i64, (int)sizeof(double));
    i64 = vn2->size[0];
    vn2->size[0] = vn1->size[0];
    emxEnsureCapacity((emxArray__common *)vn2, i64, (int)sizeof(double));
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
            i64 = i_ip1 + b_m * (lastc - 1);
            itemp = i_ip1;
            while ((b_m > 0) && (itemp <= i64)) {
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
                i64 = lastv + i_ip1;
                for (pvt = i_ip1; pvt <= i64; pvt++) {
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
// Arguments    : int b_m
//                int n
//                emxArray_real_T *A
//                int lda
//                emxArray_int32_T *ipiv
//                int *info
// Return Type  : void
//
static void xgetrf(int b_m, int n, emxArray_real_T *A, int lda, emxArray_int32_T
                   *ipiv, int *info)
{
  int c_m;
  int b_info;
  int i62;
  int j;
  int mmj;
  int c;
  int iy;
  int ix;
  double smax;
  int jA;
  double s;
  int i63;
  int jy;
  int b_j;
  int ijA;
  if (b_m <= n) {
    c_m = b_m;
  } else {
    c_m = n;
  }

  eml_signed_integer_colon(c_m, ipiv);
  b_info = 0;
  if ((b_m < 1) || (n < 1)) {
  } else {
    if (b_m - 1 <= n) {
      i62 = b_m - 1;
    } else {
      i62 = n;
    }

    for (j = 0; j + 1 <= i62; j++) {
      mmj = b_m - j;
      c = j * (lda + 1);
      if (mmj < 1) {
        iy = -1;
      } else {
        iy = 0;
        if (mmj > 1) {
          ix = c;
          smax = fabs(A->data[c]);
          for (jA = 1; jA + 1 <= mmj; jA++) {
            ix++;
            s = fabs(A->data[ix]);
            if (s > smax) {
              iy = jA;
              smax = s;
            }
          }
        }
      }

      if (A->data[c + iy] != 0.0) {
        if (iy != 0) {
          ipiv->data[j] = (j + iy) + 1;
          ix = j;
          iy += j;
          for (jA = 1; jA <= n; jA++) {
            smax = A->data[ix];
            A->data[ix] = A->data[iy];
            A->data[iy] = smax;
            ix += lda;
            iy += lda;
          }
        }

        i63 = c + mmj;
        for (iy = c + 1; iy + 1 <= i63; iy++) {
          A->data[iy] /= A->data[c];
        }
      } else {
        b_info = j + 1;
      }

      iy = (n - j) - 1;
      jA = c + lda;
      jy = c + lda;
      for (b_j = 1; b_j <= iy; b_j++) {
        smax = A->data[jy];
        if (A->data[jy] != 0.0) {
          ix = c + 1;
          i63 = mmj + jA;
          for (ijA = 1 + jA; ijA + 1 <= i63; ijA++) {
            A->data[ijA] += A->data[ix] * -smax;
            ix++;
          }
        }

        jy += lda;
        jA += lda;
      }
    }

    if ((b_info == 0) && (b_m <= n) && (!(A->data[(b_m + A->size[0] * (b_m - 1))
          - 1] != 0.0))) {
      b_info = b_m;
    }
  }

  *info = b_info;
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
  int i65;
  int k;
  i65 = (ix0 + n) - 1;
  for (k = ix0; k <= i65; k++) {
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
//                double map_out[120]
//                AnchorPose anchor_poses_out[8]
//                double delayedStatus_out[40]
// Return Type  : void
//
void SLAM(int updateVect[40], const double z_all_l[80], const double z_all_r[80],
          double dt, const VIOMeasurements *measurements, const DUOParameters
          *cameraParameters, const NoiseParameters *noiseParameters, const
          VIOParameters *b_VIOParameters, boolean_T vision, RobotState *xt_out,
          double map_out[120], AnchorPose anchor_poses_out[8], double
          delayedStatus_out[40])
{
  int i;
  static const signed char iv1[4] = { 0, 0, 0, 1 };

  static const f_struct_T rv0[8] = { { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 },
      { 0, 0, 0, 0, 0, 0 }, { { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0,
          0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, {
            0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0,
            0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0,
            0.0, 0.0 }, 0, 0, 0 } } }, { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0
      }, { 0, 0, 0, 0, 0, 0 }, { { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0,
          0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0,
          { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0,
            0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0,
            0.0, 0.0 }, 0, 0, 0 } } }, { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0
      }, { 0, 0, 0, 0, 0, 0 }, { { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0,
          0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0,
          { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0,
            0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0,
            0.0, 0.0 }, 0, 0, 0 } } }, { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0
      }, { 0, 0, 0, 0, 0, 0 }, { { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0,
          0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0,
          { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0,
            0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0,
            0.0, 0.0 }, 0, 0, 0 } } }, { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0
      }, { 0, 0, 0, 0, 0, 0 }, { { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0,
          0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0,
          { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0,
            0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0,
            0.0, 0.0 }, 0, 0, 0 } } }, { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0
      }, { 0, 0, 0, 0, 0, 0 }, { { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0,
          0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0,
          { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0,
            0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0,
            0.0, 0.0 }, 0, 0, 0 } } }, { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0
      }, { 0, 0, 0, 0, 0, 0 }, { { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0,
          0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0,
          { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0,
            0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0,
            0.0, 0.0 }, 0, 0, 0 } } }, { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0
      }, { 0, 0, 0, 0, 0, 0 }, { { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0,
          0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0,
          { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0,
            0.0 }, { 0.0, 0.0, 0.0 }, 0, 0, 0 }, { 0.0, { 0.0, 0.0, 0.0 }, { 0.0,
            0.0, 0.0 }, 0, 0, 0 } } } };

  int anchorIdx;
  int i47;
  int i48;
  double z_b[3];
  double B;
  double y_n_b[3];
  static const double dv6[3] = { 0.0, 0.0, 1.0 };

  double b_y_n_b[3];
  double x_n_b[3];
  double dv7[9];
  double b_x_n_b[9];
  double R_cw_init[9];
  static const signed char y[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double dv8[9];
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 0 };

  double b_z_all_l[80];
  double b_z_all_r[80];
  double t0_IMU_att[4];
  double t0_IMU_pos[3];
  double t0_IMU_acc_bias[3];
  double t0_IMU_gyro_bias[3];
  double t0_vel[3];
  double t0_att[4];
  struct_T rv1[8];

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
    //      xt.robot_state.pos = [-0.4; -0.8; 0.5]; % position relative to the origin frame 
    // 'SLAM:33' xt.robot_state.att = [0; 0; 0; 1];
    for (i = 0; i < 4; i++) {
      xt.robot_state.att[i] = iv1[i];
    }

    //  orientation relative to the origin frame
    // 'SLAM:34' xt.robot_state.vel = [0; 0; 0];
    //  velocity in the origin frame
    // 'SLAM:35' xt.fixed_feature = int32(0);
    xt.fixed_feature = 0;

    // 'SLAM:36' xt.origin.anchor_idx = int32(0);
    xt.origin.anchor_idx = 0;

    //  idx of the anchor that is at the origin
    // 'SLAM:37' xt.origin.pos = [0; 0; 0];
    for (i = 0; i < 3; i++) {
      xt.robot_state.vel[i] = 0.0;
      xt.origin.pos[i] = 0.0;
    }

    //  position of the origin in the world frame
    // 'SLAM:38' xt.origin.att = [0; 0; 0; 1];
    for (i = 0; i < 4; i++) {
      xt.origin.att[i] = iv1[i];
    }

    //  orientation of the origin in the world frame
    // 'SLAM:40' P = zeros(numStates + numAnchors*(6+numPointsPerAnchor));
    memset(&P[0], 0, 11881U * sizeof(double));

    //  initial error state covariance
    // 'SLAM:42' anchor_state.pos = [0; 0; 0];
    // 'SLAM:43' anchor_state.att = [0; 0; 0; 1];
    // 'SLAM:44' anchor_state.P_idx = int32(zeros(1, 6));
    // 'SLAM:46' feature_state.inverse_depth = 0;
    // 'SLAM:47' feature_state.m = zeros(3,1);
    // 'SLAM:48' feature_state.scaled_map_point = zeros(3,1);
    // 'SLAM:49' feature_state.status = int32(0);
    // 'SLAM:50' feature_state.status_idx = int32(0);
    // 'SLAM:51' feature_state.P_idx = int32(0);
    // 'SLAM:53' anchor_state.feature_states = repmat(feature_state, numPointsPerAnchor,1); 
    // 'SLAM:55' xt.anchor_states = repmat(anchor_state, numAnchors, 1);
    // 'SLAM:57' for anchorIdx = 1:numAnchors
    memcpy(&xt.anchor_states[0], &rv0[0], sizeof(f_struct_T) << 3);
    for (anchorIdx = 0; anchorIdx < 8; anchorIdx++) {
      // 'SLAM:58' xt.anchor_states(anchorIdx).P_idx = numStates + (anchorIdx-1)*numStatesPerAnchor + int32(1:6); 
      i47 = anchorIdx * 11;
      for (i48 = 0; i48 < 6; i48++) {
        xt.anchor_states[anchorIdx].P_idx[i48] = (i48 + i47) + 22;
      }
    }

    // 'SLAM:61' if vision
    if (vision) {
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
      for (i47 = 0; i47 < 3; i47++) {
        z_b[i47] /= B;
      }

      // 'SLAM:67' m_n_b = [0;0;1];
      // 'SLAM:68' y_n_b = cross(z_n_b,m_n_b);
      cross(z_b, dv6, y_n_b);

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
      dv7[0] = ((xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0] -
                 xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) -
                xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      dv7[3] = 2.0 * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[1] +
                      xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[3]);
      dv7[6] = 2.0 * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[2] -
                      xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[3]);
      dv7[1] = 2.0 * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[1] -
                      xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[3]);
      dv7[4] = ((-(xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0]) +
                 xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) -
                xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      dv7[7] = 2.0 * (xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[2] +
                      xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[3]);
      dv7[2] = 2.0 * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[2] +
                      xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[3]);
      dv7[5] = 2.0 * (xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[2] -
                      xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[3]);
      dv7[8] = ((-(xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0]) -
                 xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) +
                xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      for (i47 = 0; i47 < 3; i47++) {
        b_x_n_b[i47] = x_n_b[i47];
        b_x_n_b[3 + i47] = y_n_b[i47];
        b_x_n_b[6 + i47] = z_b[i47];
      }

      //          yaw = 87;
      //          R = rotz(yaw);
      //          R_cw_init = R_cw_init*R';
      // 'SLAM:79' xt.origin.att = QuatFromRotJ(R_cw_init);
      for (i47 = 0; i47 < 3; i47++) {
        for (i48 = 0; i48 < 3; i48++) {
          R_cw_init[i47 + 3 * i48] = 0.0;
          for (i = 0; i < 3; i++) {
            R_cw_init[i47 + 3 * i48] += dv7[i47 + 3 * i] * b_x_n_b[i + 3 * i48];
          }

          P[i48 + 109 * i47] = 0.0;
          P[(i48 + 109 * (3 + i47)) + 3] = 0.0;
          P[(i48 + 109 * (6 + i47)) + 6] = y[i48 + 3 * i47];
        }
      }

      QuatFromRotJ(R_cw_init, xt.origin.att);

      //  orientation of the origin in the world frame
      // 'SLAM:81' P(  1:3,   1:3) = zeros(3);
      //  position
      // 'SLAM:82' P(  4:6,   4:6) = zeros(3);
      //  orientation of camera in origin frame
      // 'SLAM:83' P(  7:9,   7:9) = 1*eye(3);
      //  velocity
      // 'SLAM:84' P(10:12, 10:12) = diag(noiseParameters.gyro_bias_initial_unc); 
      diag(noiseParameters->gyro_bias_initial_unc, dv7);

      //  gyro bias
      // 'SLAM:85' P(13:15, 13:15) = diag(noiseParameters.acc_bias_initial_unc); 
      diag(noiseParameters->acc_bias_initial_unc, b_x_n_b);

      //  acc bias
      // 'SLAM:86' P(16:18, 16:18) = 0.1*R_cw_init * diag([1 1 0]) * R_cw_init'; 
      for (i47 = 0; i47 < 3; i47++) {
        for (i48 = 0; i48 < 3; i48++) {
          P[(i48 + 109 * (9 + i47)) + 9] = dv7[i48 + 3 * i47];
          P[(i48 + 109 * (12 + i47)) + 12] = b_x_n_b[i48 + 3 * i47];
          dv8[i47 + 3 * i48] = 0.0;
          for (i = 0; i < 3; i++) {
            dv8[i47 + 3 * i48] += 0.1 * R_cw_init[i47 + 3 * i] * (double)b[i + 3
              * i48];
          }
        }
      }

      for (i47 = 0; i47 < 3; i47++) {
        for (i48 = 0; i48 < 3; i48++) {
          P[(i47 + 109 * (15 + i48)) + 15] = 0.0;
          for (i = 0; i < 3; i++) {
            P[(i47 + 109 * (15 + i48)) + 15] += dv8[i47 + 3 * i] * R_cw_init[i48
              + 3 * i];
          }
        }
      }

      //  origin orientation
      // 'SLAM:87' P(19:21, 19:21) = 0*0.01*eye(3);
      for (i47 = 0; i47 < 3; i47++) {
        for (i48 = 0; i48 < 3; i48++) {
          P[(i48 + 109 * (18 + i47)) + 18] = 0.0;
        }
      }

      //  R_ci
      // 'SLAM:89' map = getMap(xt);
      getMap(xt.origin.pos, xt.origin.att, xt.anchor_states, map);

      // 'SLAM:90' delayedStatus = zeros(size(updateVect));
      memset(&delayedStatus[0], 0, 40U * sizeof(double));

      // 'SLAM:92' printParams(noiseParameters, VIOParameters)
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
                  b_VIOParameters->full_stereo, b_VIOParameters->RANSAC);

      // 'SLAM:93' initialized = 1;
      initialized_not_empty = true;
    }
  } else {
    // 'SLAM:96' else
    // 'SLAM:98' if ~vision
    if (!vision) {
      //      [xt,P] =  SLAM_pred(P, xt, dt, noiseParameters.process_noise, measurements, numStates); 
      // 'SLAM:100' [xt,P] =  SLAM_pred_euler(P, xt, dt, noiseParameters.process_noise, measurements); 
      SLAM_pred_euler(P, &xt, dt, noiseParameters->process_noise.qv,
                      noiseParameters->process_noise.qw,
                      noiseParameters->process_noise.qao,
                      noiseParameters->process_noise.qwo,
                      noiseParameters->process_noise.qR_ci,
                      measurements->acc_duo, measurements->gyr_duo);
    } else {
      // 'SLAM:101' else
      // 'SLAM:102' [xt, P, updateVect, map, delayedStatus] = SLAM_upd(P, xt, cameraParameters, updateVect, z_all_l, z_all_r, noiseParameters, VIOParameters); 
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
               noiseParameters->inv_depth_initial_unc, *b_VIOParameters, map,
               delayedStatus);
    }
  }

  // 'SLAM:105' map_out = map;
  memcpy(&map_out[0], &map[0], 120U * sizeof(double));

  // 'SLAM:106' xt_out = getWorldState(xt);
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

  // 'SLAM:107' anchor_poses_out = getAnchorPoses(xt);
  getAnchorPoses(xt.origin.pos, xt.origin.att, xt.anchor_states, rv1);
  cast(rv1, anchor_poses_out);

  // 'SLAM:108' delayedStatus_out = delayedStatus;
  memcpy(&delayedStatus_out[0], &delayedStatus[0], 40U * sizeof(double));

  //  output
  //  NOTE: Comment this out for MEXing
  // 'SLAM:112' coder.cstructname(xt_out, 'RobotState', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  // 'SLAM:113' coder.cstructname(xt_out.IMU, 'IMUState', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  // 'SLAM:114' coder.cstructname(anchor_poses_out(1), 'AnchorPose', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  // 'SLAM:116' assert ( all ( size (map_out) == [numTrackFeatures*3 1] ) )
  // 'SLAM:117' assert ( all ( size (anchor_poses_out) == [numAnchors 1] ) )
  // 'SLAM:118' assert ( all ( size (updateVect) == [numTrackFeatures 1] ) )
  // 'SLAM:119' assert ( all ( size (delayedStatus_out) == [numTrackFeatures 1] ) ) 
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
