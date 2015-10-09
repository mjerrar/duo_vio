//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 09-Oct-2015 09:29:00
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

#ifndef struct_emxArray_real32_T
#define struct_emxArray_real32_T

struct emxArray_real32_T
{
  float *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_real32_T

typedef struct {
  float pos[3];
  float att[4];
  float gyro_bias[3];
  float acc_bias[3];
} b_struct_T;

typedef struct {
  b_struct_T IMU;
  float pos[3];
  float att[4];
  float vel[3];
} c_struct_T;

typedef struct {
  int anchor_idx;
  float pos[3];
  float att[4];
} d_struct_T;

typedef struct {
  float inverse_depth;
  float m[3];
  float scaled_map_point[3];
  int status;
  int status_idx;
  int P_idx;
} e_struct_T;

typedef struct {
  float pos[3];
  float att[4];
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
  float pos[3];
  float att[4];
} struct_T;

// Named Constants
#define b_debug_level                  (2.0F)

// Variable Definitions
static boolean_T initialized_not_empty;
static g_struct_T xt;
static float P[8281];
static float h_u[160];
static float map[120];
static float delayedStatus[40];
static float debug_level;

// Function Declarations
static void OnePointRANSAC_EKF(g_struct_T *b_xt, float b_P[8281], const float
  z_u_l[80], const float cameraparams_FocalLength[2], const float
  cameraparams_PrincipalPoint[2], float noiseParameters_image_noise, int
  c_VIOParameters_max_ekf_iterati, boolean_T c_VIOParameters_delayed_initial,
  boolean_T VIOParameters_RANSAC, int updateVect[40]);
static void SLAM_upd(float P_apr[8281], g_struct_T *b_xt, const float
                     c_cameraParams_CameraParameters[2], const float
                     d_cameraParams_CameraParameters[2], const float
                     e_cameraParams_CameraParameters[3], const float
                     f_cameraParams_CameraParameters[2], const float
                     g_cameraParams_CameraParameters[2], const float
                     h_cameraParams_CameraParameters[3], const float
                     cameraParams_r_lr[3], const float cameraParams_R_lr[9],
                     const float cameraParams_R_rl[9], int updateVect[40], float
                     z_all_l[80], float z_all_r[80], float
                     noiseParameters_image_noise, float
                     c_noiseParameters_inv_depth_ini, const VIOParameters
                     b_VIOParameters, float h_u_apo[160], float b_map[120],
                     float b_delayedStatus[40]);
static boolean_T any(const boolean_T x[40]);
static boolean_T anyActiveAnchorFeatures(const e_struct_T
  anchor_state_feature_states[8]);
static boolean_T b_any(const boolean_T x[3]);
static double b_fprintf();
static void b_merge(emxArray_int32_T *idx, emxArray_real32_T *x, int offset, int
                    np, int nq, emxArray_int32_T *iwork, emxArray_real32_T
                    *xwork);
static float b_norm(const float x[4]);
static void b_ros_error();
static void b_ros_info(int varargin_1, int varargin_2, int varargin_3);
static void b_sort(float x[8], int idx[8]);
static void b_xgeqp3(float A[30], float tau[5], int jpvt[5]);
static float b_xnrm2(int n, const emxArray_real32_T *x, int ix0);
static boolean_T c_any(const float x[5]);
static double c_fprintf(float varargin_1);
static float c_norm(const float x[2]);
static void c_ros_info(int varargin_1, int varargin_2, int varargin_3);
static void c_sort(emxArray_real32_T *x, emxArray_int32_T *idx);
static float c_xnrm2(int n, const float x[30], int ix0);
static void cast(const struct_T x[5], AnchorPose y[5]);
static void cross(const float a[3], const float b[3], float c[3]);
static double d_fprintf(float varargin_1);
static void d_ros_info(int varargin_1, int varargin_2);
static void d_sort(emxArray_real32_T *x, int dim, emxArray_int32_T *idx);
static float det(const float x[36]);
static void diag(const float v[3], float d[9]);
static double e_fprintf(float varargin_1);
static void e_ros_info(int varargin_1);
static void eml_signed_integer_colon(int b, emxArray_int32_T *y);
static void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
static void emxFree_int32_T(emxArray_int32_T **pEmxArray);
static void emxFree_real32_T(emxArray_real32_T **pEmxArray);
static void emxInit_int32_T(emxArray_int32_T **pEmxArray, int b_numDimensions);
static void emxInit_int32_T1(emxArray_int32_T **pEmxArray, int b_numDimensions);
static void emxInit_real32_T(emxArray_real32_T **pEmxArray, int b_numDimensions);
static void emxInit_real32_T1(emxArray_real32_T **pEmxArray, int b_numDimensions);
static double f_fprintf(float varargin_1);
static void f_ros_info(int varargin_1);
static void fileManager(FILE * *f, boolean_T *a);
static double g_fprintf(float varargin_1);
static void g_ros_info(int varargin_1, int varargin_2, int varargin_3);
static void getAnchorPoses(const float xt_origin_pos[3], const float
  xt_origin_att[4], const f_struct_T xt_anchor_states[5], struct_T anchor_poses
  [5]);
static void getH_R_res(const float xt_robot_state_pos[3], const float
  xt_robot_state_att[4], const f_struct_T xt_anchor_states[5], const float
  z_all_l[80], const boolean_T b_status[40], const float
  cameraparams_FocalLength[2], const float cameraparams_PrincipalPoint[2], float
  noiseParameters_image_noise, emxArray_real32_T *r, emxArray_real32_T *H,
  emxArray_real32_T *R);
static void getMap(const float xt_origin_pos[3], const float xt_origin_att[4],
                   const f_struct_T xt_anchor_states[5], float b_map[120]);
static float getNumValidFeatures(const e_struct_T anchor_state_feature_states[8]);
static void getScaledMap(g_struct_T *b_xt);
static float getTotalNumActiveFeatures(const f_struct_T xt_anchor_states[5]);
static float getTotalNumDelayedFeatures(const f_struct_T xt_anchor_states[5]);
static void getWorldState(const float xt_robot_state_IMU_pos[3], const float
  xt_robot_state_IMU_att[4], const float xt_robot_state_IMU_gyro_bias[3], const
  float xt_robot_state_IMU_acc_bias[3], const float xt_robot_state_pos[3], const
  float xt_robot_state_att[4], const float xt_origin_pos[3], const float
  xt_origin_att[4], float world_state_pos[3], float world_state_att[4], float
  world_state_vel[3], float world_state_IMU_gyro_bias[3], float
  world_state_IMU_acc_bias[3], float world_state_IMU_pos[3], float
  world_state_IMU_att[4]);
static double h_fprintf(float varargin_1, float varargin_2, float varargin_3);
static void h_ros_info(int varargin_1, float varargin_2, float varargin_3);
static double i_fprintf(float varargin_1, float varargin_2, float varargin_3);
static void i_ros_info(int varargin_1);
static void initializePoint(const float z_u_l[2], const float z_u_r[2], const
  float c_cameraparams_CameraParameters[2], const float
  d_cameraparams_CameraParameters[2], const float
  e_cameraparams_CameraParameters[2], const float
  f_cameraparams_CameraParameters[2], const float cameraparams_r_lr[3], const
  float cameraparams_R_lr[9], float fp[3], float b_m[6], boolean_T *success);
static double j_fprintf(float varargin_1);
static double k_fprintf(float varargin_1);
static double l_fprintf(int varargin_1);
static void lusolve(const emxArray_real32_T *A, emxArray_real32_T *B);
static double m_fprintf(int varargin_1);
static float median(const float x_data[], const int x_size[1]);
static void merge(int idx[8], float x[8], int offset, int np, int nq, int iwork
                  [8], float xwork[8]);
static void merge_block(emxArray_int32_T *idx, emxArray_real32_T *x, int offset,
  int n, int preSortLevel, emxArray_int32_T *iwork, emxArray_real32_T *xwork);
static int mul_s32_s32_s32_sat(int a, int b);
static void mul_wide_s32(int in0, int in1, unsigned int *ptrOutBitsHi, unsigned
  int *ptrOutBitsLo);
static void multiplyIdx(const float idx_data[], const int idx_size[1], float
  idx_mult_data[], int idx_mult_size[1]);
static double n_fprintf(int varargin_1);
static float norm(const float x[3]);
static double o_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static double p_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static void predictMeasurementStereo(const float fp_l[3], const float
  c_stereoParams_CameraParameters[2], const float
  d_stereoParams_CameraParameters[2], const float
  e_stereoParams_CameraParameters[2], const float
  f_stereoParams_CameraParameters[2], const float stereoParams_r_lr[3], const
  float stereoParams_R_rl[9], float h_cin_l[2], float h_cin_r[2]);
static void printParams(float c_noiseParameters_process_noise, float
  d_noiseParameters_process_noise, float e_noiseParameters_process_noise, float
  f_noiseParameters_process_noise, float g_noiseParameters_process_noise, const
  float c_noiseParameters_gyro_bias_ini[3], const float
  c_noiseParameters_acc_bias_init[3], float noiseParameters_image_noise, float
  c_noiseParameters_inv_depth_ini, int c_VIOParameters_num_points_per_, int
  VIOParameters_num_anchors, int c_VIOParameters_max_ekf_iterati, boolean_T
  VIOParameters_fixed_feature, boolean_T c_VIOParameters_delayed_initial,
  boolean_T VIOParameters_mono, boolean_T VIOParameters_RANSAC);
static double q_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static void quatmultJ(const float q[4], const float p[4], float qp[4]);
static double r_fprintf(const char varargin_1_data[], const int varargin_1_size
  [2]);
static void rdivide(const float x[3], float y, float z[3]);
static void ros_error();
static void ros_info(int varargin_1, int varargin_2, int varargin_3);
static float rt_hypotf_snf(float u0, float u1);
static float rt_powf_snf(float u0, float u1);
static float rt_roundf_snf(float u);
static void sort(float x[8], int idx[8]);
static void sortIdx(emxArray_real32_T *x, emxArray_int32_T *idx);
static void xgeqp3(emxArray_real32_T *A, emxArray_real32_T *tau,
                   emxArray_int32_T *jpvt);
static float xnrm2(int n, const emxArray_real32_T *x, int ix0);
static void xscal(int n, float a, emxArray_real32_T *x, int ix0);

// Function Definitions

//
// OnePointRANSAC_EKF Perform a 1-point RANSAC outlier rejection and update
// the state
// Arguments    : g_struct_T *b_xt
//                float b_P[8281]
//                const float z_u_l[80]
//                const float cameraparams_FocalLength[2]
//                const float cameraparams_PrincipalPoint[2]
//                float noiseParameters_image_noise
//                int c_VIOParameters_max_ekf_iterati
//                boolean_T c_VIOParameters_delayed_initial
//                boolean_T VIOParameters_RANSAC
//                int updateVect[40]
// Return Type  : void
//
static void OnePointRANSAC_EKF(g_struct_T *b_xt, float b_P[8281], const float
  z_u_l[80], const float cameraparams_FocalLength[2], const float
  cameraparams_PrincipalPoint[2], float noiseParameters_image_noise, int
  c_VIOParameters_max_ekf_iterati, boolean_T c_VIOParameters_delayed_initial,
  boolean_T VIOParameters_RANSAC, int updateVect[40])
{
  int ib;
  boolean_T activeFeatures[40];
  boolean_T delayedFeatures[40];
  int anchorIdx;
  int featureIdx;
  boolean_T LI_inlier_status[40];
  emxArray_real32_T *S;
  emxArray_real32_T *K;
  emxArray_real32_T *r;
  emxArray_real32_T *H;
  emxArray_real32_T *out;
  emxArray_real32_T *c;
  emxArray_real32_T *b_c;
  emxArray_real32_T *Y;
  emxArray_real32_T *A;
  emxArray_real32_T *tau;
  emxArray_int32_T *jpvt;
  emxArray_real32_T *B;
  int idx;
  signed char ii_data[40];
  int br;
  boolean_T exitg2;
  boolean_T guard2 = false;
  int loop_ub;
  int i24;
  signed char i_data[40];
  signed char hyp_ind_data[40];
  float num_hyp;
  int hyp_it;
  emxArray_real32_T *c_c;
  emxArray_real32_T *d_c;
  emxArray_real32_T *e_c;
  boolean_T LI_inlier_status_i[40];
  boolean_T hyp_status[40];
  unsigned int H_idx_0;
  int b_m;
  int ar;
  int i25;
  int f_c;
  int ic;
  int ia;
  unsigned int unnamed_idx_1;
  float tol;
  int k;
  float x_apo[91];
  float b_x_apo;
  float theta;
  float dq[4];
  float c_xt[4];
  float R_cw[9];
  float r_wc[3];
  float anchorPos[3];
  float fp_scaled[3];
  float anchorRot[9];
  float rho;
  float b_rho[3];
  float h_c_n_l[2];
  float z_u[2];
  float g_c[2];
  emxArray_real32_T *h_c;
  static float i_c[8281];
  float fv14[8281];
  static const signed char iv14[8281] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
  };

  float fv15[8281];
  boolean_T b_activeFeatures;
  boolean_T exitg1;
  boolean_T guard1 = false;
  emxArray_real32_T *b_K;
  emxArray_real32_T *b_H;
  int it;
  emxArray_real32_T *j_c;
  emxArray_real32_T *k_c;
  emxArray_real32_T *l_c;
  emxArray_real32_T *m_c;
  emxArray_real32_T *n_c;
  emxArray_real32_T *c_H;
  emxArray_real32_T *d_H;
  emxArray_real32_T *e_H;
  signed char o_c[2];
  signed char p_c[2];
  signed char q_c[2];
  float x_apo_data[91];
  float b_x_apo_data;
  float c_data[8281];
  float fv16[8281];
  emxArray_real32_T *r_c;
  emxArray_real32_T *s_c;
  emxArray_real32_T *t_c;
  float fv17[8281];

  //  HI mahalanobis gate
  for (ib = 0; ib < 40; ib++) {
    activeFeatures[ib] = false;
    delayedFeatures[ib] = false;
  }

  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status == 1)
      {
        activeFeatures[b_xt->anchor_states[anchorIdx].feature_states[featureIdx]
          .status_idx - 1] = true;
      } else {
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
            2) {
          delayedFeatures[b_xt->anchor_states[anchorIdx]
            .feature_states[featureIdx].status_idx - 1] = true;
        }
      }
    }
  }

  for (ib = 0; ib < 40; ib++) {
    LI_inlier_status[ib] = false;
    activeFeatures[ib] = (activeFeatures[ib] && (updateVect[ib] == 1));
    delayedFeatures[ib] = (delayedFeatures[ib] && (updateVect[ib] == 1));
  }

  // % B 1-point hypotheses generation and evaluation
  emxInit_real32_T1(&S, 2);
  emxInit_real32_T1(&K, 2);
  emxInit_real32_T(&r, 1);
  emxInit_real32_T1(&H, 2);
  emxInit_real32_T(&out, 1);
  emxInit_real32_T1(&c, 2);
  emxInit_real32_T1(&b_c, 2);
  emxInit_real32_T1(&Y, 2);
  emxInit_real32_T1(&A, 2);
  emxInit_real32_T(&tau, 1);
  emxInit_int32_T1(&jpvt, 2);
  emxInit_real32_T1(&B, 2);
  if (VIOParameters_RANSAC) {
    //  build the map according to the current estimate
    getScaledMap(b_xt);

    //  randomly permute the active feature indices for 1-point RANSAC
    idx = 0;
    br = 1;
    exitg2 = false;
    while ((!exitg2) && (br < 41)) {
      guard2 = false;
      if (activeFeatures[br - 1]) {
        idx++;
        ii_data[idx - 1] = (signed char)br;
        if (idx >= 40) {
          exitg2 = true;
        } else {
          guard2 = true;
        }
      } else {
        guard2 = true;
      }

      if (guard2) {
        br++;
      }
    }

    if (1 > idx) {
      loop_ub = 0;
    } else {
      loop_ub = idx;
    }

    if (1 > idx) {
      br = 0;
    } else {
      br = idx;
    }

    for (i24 = 0; i24 < loop_ub; i24++) {
      i_data[i24] = ii_data[i24];
    }

    i24 = out->size[0];
    out->size[0] = br;
    emxEnsureCapacity((emxArray__common *)out, i24, (int)sizeof(float));
    for (i24 = 0; i24 < br; i24++) {
      out->data[i24] = i_data[i24];
    }

    for (i24 = 0; i24 < br; i24++) {
      hyp_ind_data[i24] = i_data[i24];
    }

    //  hyp_ind = hyp_ind(randperm(length(hyp_ind)));
    num_hyp = (float)out->size[0];
    hyp_it = 1;
    emxInit_real32_T1(&c_c, 2);
    emxInit_real32_T1(&d_c, 2);
    emxInit_real32_T1(&e_c, 2);
    while ((hyp_it < num_hyp) && (hyp_it < loop_ub)) {
      for (ib = 0; ib < 40; ib++) {
        LI_inlier_status_i[ib] = false;
        hyp_status[ib] = false;
      }

      hyp_status[hyp_ind_data[hyp_it - 1] - 1] = true;

      //  used to signal which feature to compute the derivatives for
      getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att,
                 b_xt->anchor_states, z_u_l, hyp_status,
                 cameraparams_FocalLength, cameraparams_PrincipalPoint,
                 noiseParameters_image_noise, r, H, S);
      H_idx_0 = (unsigned int)H->size[0];
      i24 = c_c->size[0] * c_c->size[1];
      c_c->size[0] = (int)H_idx_0;
      c_c->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)c_c, i24, (int)sizeof(float));
      b_m = H->size[0];
      i24 = c_c->size[0] * c_c->size[1];
      c_c->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)c_c, i24, (int)sizeof(float));
      for (i24 = 0; i24 < 91; i24++) {
        ar = c_c->size[0];
        for (i25 = 0; i25 < ar; i25++) {
          c_c->data[i25 + c_c->size[0] * i24] = 0.0F;
        }
      }

      if (H->size[0] == 0) {
      } else {
        f_c = H->size[0] * 90;
        idx = 0;
        while ((b_m > 0) && (idx <= f_c)) {
          i24 = idx + b_m;
          for (ic = idx; ic + 1 <= i24; ic++) {
            c_c->data[ic] = 0.0F;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= f_c)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (b_P[ib] != 0.0F) {
              ia = ar;
              i24 = idx + b_m;
              for (ic = idx; ic + 1 <= i24; ic++) {
                ia++;
                c_c->data[ic] += b_P[ib] * H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i24 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)K, i24, (int)sizeof(float));
      ar = H->size[0];
      for (i24 = 0; i24 < ar; i24++) {
        for (i25 = 0; i25 < 91; i25++) {
          K->data[i25 + K->size[0] * i24] = H->data[i24 + H->size[0] * i25];
        }
      }

      H_idx_0 = (unsigned int)c_c->size[0];
      unnamed_idx_1 = (unsigned int)K->size[1];
      i24 = d_c->size[0] * d_c->size[1];
      d_c->size[0] = (int)H_idx_0;
      d_c->size[1] = (int)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)d_c, i24, (int)sizeof(float));
      b_m = c_c->size[0];
      i24 = d_c->size[0] * d_c->size[1];
      emxEnsureCapacity((emxArray__common *)d_c, i24, (int)sizeof(float));
      ar = d_c->size[1];
      for (i24 = 0; i24 < ar; i24++) {
        ib = d_c->size[0];
        for (i25 = 0; i25 < ib; i25++) {
          d_c->data[i25 + d_c->size[0] * i24] = 0.0F;
        }
      }

      if ((c_c->size[0] == 0) || (K->size[1] == 0)) {
      } else {
        f_c = c_c->size[0] * (K->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= f_c)) {
          i24 = idx + b_m;
          for (ic = idx; ic + 1 <= i24; ic++) {
            d_c->data[ic] = 0.0F;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= f_c)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0F) {
              ia = ar;
              i24 = idx + b_m;
              for (ic = idx; ic + 1 <= i24; ic++) {
                ia++;
                d_c->data[ic] += K->data[ib] * c_c->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i24 = S->size[0] * S->size[1];
      S->size[0] = d_c->size[0];
      S->size[1] = d_c->size[1];
      emxEnsureCapacity((emxArray__common *)S, i24, (int)sizeof(float));
      ar = d_c->size[0] * d_c->size[1];
      for (i24 = 0; i24 < ar; i24++) {
        S->data[i24] += d_c->data[i24];
      }

      i24 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)K, i24, (int)sizeof(float));
      ar = H->size[0];
      for (i24 = 0; i24 < ar; i24++) {
        for (i25 = 0; i25 < 91; i25++) {
          K->data[i25 + K->size[0] * i24] = H->data[i24 + H->size[0] * i25];
        }
      }

      unnamed_idx_1 = (unsigned int)K->size[1];
      i24 = e_c->size[0] * e_c->size[1];
      e_c->size[0] = 91;
      e_c->size[1] = (int)unnamed_idx_1;
      e_c->size[0] = 91;
      emxEnsureCapacity((emxArray__common *)e_c, i24, (int)sizeof(float));
      ar = e_c->size[1];
      for (i24 = 0; i24 < ar; i24++) {
        for (i25 = 0; i25 < 91; i25++) {
          e_c->data[i25 + e_c->size[0] * i24] = 0.0F;
        }
      }

      if (K->size[1] == 0) {
      } else {
        f_c = 91 * (K->size[1] - 1);
        for (idx = 0; idx <= f_c; idx += 91) {
          for (ic = idx + 1; ic <= idx + 91; ic++) {
            e_c->data[ic - 1] = 0.0F;
          }
        }

        br = 0;
        for (idx = 0; idx <= f_c; idx += 91) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0F) {
              ia = ar;
              for (ic = idx; ic + 1 <= idx + 91; ic++) {
                ia++;
                e_c->data[ic] += K->data[ib] * b_P[ia - 1];
              }
            }

            ar += 91;
          }

          br += 91;
        }
      }

      if ((e_c->size[1] == 0) || ((S->size[0] == 0) || (S->size[1] == 0))) {
        unnamed_idx_1 = (unsigned int)S->size[0];
        i24 = K->size[0] * K->size[1];
        K->size[0] = 91;
        K->size[1] = (int)unnamed_idx_1;
        emxEnsureCapacity((emxArray__common *)K, i24, (int)sizeof(float));
        ar = 91 * (int)unnamed_idx_1;
        for (i24 = 0; i24 < ar; i24++) {
          K->data[i24] = 0.0F;
        }
      } else if (S->size[0] == S->size[1]) {
        i24 = K->size[0] * K->size[1];
        K->size[0] = 91;
        K->size[1] = e_c->size[1];
        emxEnsureCapacity((emxArray__common *)K, i24, (int)sizeof(float));
        ar = e_c->size[0] * e_c->size[1];
        for (i24 = 0; i24 < ar; i24++) {
          K->data[i24] = e_c->data[i24];
        }

        lusolve(S, K);
      } else {
        i24 = A->size[0] * A->size[1];
        A->size[0] = S->size[1];
        A->size[1] = S->size[0];
        emxEnsureCapacity((emxArray__common *)A, i24, (int)sizeof(float));
        ar = S->size[0];
        for (i24 = 0; i24 < ar; i24++) {
          ib = S->size[1];
          for (i25 = 0; i25 < ib; i25++) {
            A->data[i25 + A->size[0] * i24] = S->data[i24 + S->size[0] * i25];
          }
        }

        xgeqp3(A, tau, jpvt);
        ia = 0;
        if (A->size[0] < A->size[1]) {
          br = A->size[0];
          idx = A->size[1];
        } else {
          br = A->size[1];
          idx = A->size[0];
        }

        if (br > 0) {
          tol = (float)idx * (float)fabs((double)A->data[0]) * 1.1920929E-7F;
          while ((ia < br) && ((float)fabs((double)A->data[ia + A->size[0] * ia])
                               >= tol)) {
            ia++;
          }
        }

        idx = A->size[1];
        i24 = Y->size[0] * Y->size[1];
        Y->size[0] = idx;
        Y->size[1] = 91;
        emxEnsureCapacity((emxArray__common *)Y, i24, (int)sizeof(float));
        ar = idx * 91;
        for (i24 = 0; i24 < ar; i24++) {
          Y->data[i24] = 0.0F;
        }

        i24 = B->size[0] * B->size[1];
        B->size[0] = e_c->size[1];
        B->size[1] = 91;
        emxEnsureCapacity((emxArray__common *)B, i24, (int)sizeof(float));
        for (i24 = 0; i24 < 91; i24++) {
          ar = e_c->size[1];
          for (i25 = 0; i25 < ar; i25++) {
            B->data[i25 + B->size[0] * i24] = e_c->data[i24 + e_c->size[0] * i25];
          }
        }

        b_m = A->size[0];
        idx = A->size[0];
        br = A->size[1];
        if (idx <= br) {
          br = idx;
        }

        for (idx = 0; idx + 1 <= br; idx++) {
          if (tau->data[idx] != 0.0F) {
            for (k = 0; k < 91; k++) {
              tol = B->data[idx + B->size[0] * k];
              for (ib = idx + 1; ib + 1 <= b_m; ib++) {
                tol += A->data[ib + A->size[0] * idx] * B->data[ib + B->size[0] *
                  k];
              }

              tol *= tau->data[idx];
              if (tol != 0.0F) {
                B->data[idx + B->size[0] * k] -= tol;
                for (ib = idx + 1; ib + 1 <= b_m; ib++) {
                  B->data[ib + B->size[0] * k] -= A->data[ib + A->size[0] * idx]
                    * tol;
                }
              }
            }
          }
        }

        for (k = 0; k < 91; k++) {
          for (ib = 0; ib + 1 <= ia; ib++) {
            Y->data[(jpvt->data[ib] + Y->size[0] * k) - 1] = B->data[ib +
              B->size[0] * k];
          }

          for (idx = ia - 1; idx + 1 > 0; idx--) {
            Y->data[(jpvt->data[idx] + Y->size[0] * k) - 1] /= A->data[idx +
              A->size[0] * idx];
            for (ib = 0; ib + 1 <= idx; ib++) {
              Y->data[(jpvt->data[ib] + Y->size[0] * k) - 1] -= Y->data
                [(jpvt->data[idx] + Y->size[0] * k) - 1] * A->data[ib + A->size
                [0] * idx];
            }
          }
        }

        i24 = K->size[0] * K->size[1];
        K->size[0] = 91;
        K->size[1] = Y->size[0];
        emxEnsureCapacity((emxArray__common *)K, i24, (int)sizeof(float));
        ar = Y->size[0];
        for (i24 = 0; i24 < ar; i24++) {
          for (i25 = 0; i25 < 91; i25++) {
            K->data[i25 + K->size[0] * i24] = Y->data[i24 + Y->size[0] * i25];
          }
        }
      }

      if ((K->size[1] == 1) || (r->size[0] == 1)) {
        for (i24 = 0; i24 < 91; i24++) {
          x_apo[i24] = 0.0F;
          ar = K->size[1];
          for (i25 = 0; i25 < ar; i25++) {
            b_x_apo = x_apo[i24] + K->data[i24 + K->size[0] * i25] * r->data[i25];
            x_apo[i24] = b_x_apo;
          }
        }
      } else {
        memset(&x_apo[0], 0, 91U * sizeof(float));
        ar = 0;
        for (ib = 0; ib + 1 <= K->size[1]; ib++) {
          if (r->data[ib] != 0.0F) {
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

      theta = norm(*(float (*)[3])&x_apo[3]) * 0.5F;
      if (theta < 0.244F) {
        for (ib = 0; ib < 3; ib++) {
          dq[ib] = 0.5F * x_apo[ib + 3];
        }

        dq[3] = 1.0F;
      } else {
        dq[0] = 0.5F * x_apo[3] * (float)sin((double)theta) / theta;
        dq[1] = 0.5F * x_apo[4] * (float)sin((double)theta) / theta;
        dq[2] = 0.5F * x_apo[5] * (float)sin((double)theta) / theta;
        dq[3] = (float)cos((double)theta);
      }

      tol = b_norm(dq);
      for (i24 = 0; i24 < 4; i24++) {
        dq[i24] /= tol;
      }

      quatmultJ(dq, b_xt->robot_state.att, c_xt);

      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      R_cw[0] = ((c_xt[0] * c_xt[0] - c_xt[1] * c_xt[1]) - c_xt[2] * c_xt[2]) +
        c_xt[3] * c_xt[3];
      R_cw[3] = 2.0F * (c_xt[0] * c_xt[1] + c_xt[2] * c_xt[3]);
      R_cw[6] = 2.0F * (c_xt[0] * c_xt[2] - c_xt[1] * c_xt[3]);
      R_cw[1] = 2.0F * (c_xt[0] * c_xt[1] - c_xt[2] * c_xt[3]);
      R_cw[4] = ((-(c_xt[0] * c_xt[0]) + c_xt[1] * c_xt[1]) - c_xt[2] * c_xt[2])
        + c_xt[3] * c_xt[3];
      R_cw[7] = 2.0F * (c_xt[1] * c_xt[2] + c_xt[0] * c_xt[3]);
      R_cw[2] = 2.0F * (c_xt[0] * c_xt[2] + c_xt[1] * c_xt[3]);
      R_cw[5] = 2.0F * (c_xt[1] * c_xt[2] - c_xt[0] * c_xt[3]);
      R_cw[8] = ((-(c_xt[0] * c_xt[0]) - c_xt[1] * c_xt[1]) + c_xt[2] * c_xt[2])
        + c_xt[3] * c_xt[3];
      for (ib = 0; ib < 3; ib++) {
        r_wc[ib] = b_xt->robot_state.pos[ib] + x_apo[ib];
      }

      for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
        f_c = anchorIdx * 14 + 21;
        ar = 21 + anchorIdx * 14;
        for (i24 = 0; i24 < 3; i24++) {
          anchorPos[i24] = b_xt->anchor_states[anchorIdx].pos[i24] + x_apo[i24 +
            f_c];
          fp_scaled[i24] = x_apo[(i24 + ar) + 3];
        }

        theta = norm(fp_scaled) * 0.5F;
        if (theta < 0.244F) {
          for (ib = 0; ib < 3; ib++) {
            dq[ib] = 0.5F * fp_scaled[ib];
          }

          dq[3] = 1.0F;
        } else {
          dq[0] = 0.5F * x_apo[ar + 3] * (float)sin((double)theta) / theta;
          dq[1] = 0.5F * x_apo[ar + 4] * (float)sin((double)theta) / theta;
          dq[2] = 0.5F * x_apo[ar + 5] * (float)sin((double)theta) / theta;
          dq[3] = (float)cos((double)theta);
        }

        tol = b_norm(dq);
        for (i24 = 0; i24 < 4; i24++) {
          dq[i24] /= tol;
        }

        quatmultJ(dq, b_xt->anchor_states[anchorIdx].att, c_xt);

        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        anchorRot[0] = ((c_xt[0] * c_xt[0] - c_xt[1] * c_xt[1]) - c_xt[2] *
                        c_xt[2]) + c_xt[3] * c_xt[3];
        anchorRot[3] = 2.0F * (c_xt[0] * c_xt[1] + c_xt[2] * c_xt[3]);
        anchorRot[6] = 2.0F * (c_xt[0] * c_xt[2] - c_xt[1] * c_xt[3]);
        anchorRot[1] = 2.0F * (c_xt[0] * c_xt[1] - c_xt[2] * c_xt[3]);
        anchorRot[4] = ((-(c_xt[0] * c_xt[0]) + c_xt[1] * c_xt[1]) - c_xt[2] *
                        c_xt[2]) + c_xt[3] * c_xt[3];
        anchorRot[7] = 2.0F * (c_xt[1] * c_xt[2] + c_xt[0] * c_xt[3]);
        anchorRot[2] = 2.0F * (c_xt[0] * c_xt[2] + c_xt[1] * c_xt[3]);
        anchorRot[5] = 2.0F * (c_xt[1] * c_xt[2] - c_xt[0] * c_xt[3]);
        anchorRot[8] = ((-(c_xt[0] * c_xt[0]) - c_xt[1] * c_xt[1]) + c_xt[2] *
                        c_xt[2]) + c_xt[3] * c_xt[3];
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              1) {
            //  only update active features
            rho = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              inverse_depth + x_apo[(anchorIdx * 14 + featureIdx) + 27];
            for (i24 = 0; i24 < 3; i24++) {
              tol = 0.0F;
              for (i25 = 0; i25 < 3; i25++) {
                tol += anchorRot[i25 + 3 * i24] * b_xt->anchor_states[anchorIdx]
                  .feature_states[featureIdx].m[i25];
              }

              b_rho[i24] = (rho * anchorPos[i24] + tol) - r_wc[i24] * rho;
            }

            for (i24 = 0; i24 < 3; i24++) {
              fp_scaled[i24] = 0.0F;
              for (i25 = 0; i25 < 3; i25++) {
                fp_scaled[i24] += R_cw[i24 + 3 * i25] * b_rho[i25];
              }
            }

            // predictMeasurementLeft Predict the measurement of a feature given in the left 
            // camera frame
            //    Get the normalized pixel coordinates where a feature given in the left camera 
            //    frame
            //  normalized feature in camera frame
            idx = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              status_idx;
            br = idx - 1;
            if ((idx < 0) && (br >= 0)) {
              br = MIN_int32_T;
            }

            if (br > 1073741823) {
              f_c = MAX_int32_T;
            } else if (br <= -1073741824) {
              f_c = MIN_int32_T;
            } else {
              f_c = br << 1;
            }

            for (ib = 0; ib < 2; ib++) {
              h_c_n_l[ib] = fp_scaled[ib] / fp_scaled[2];
              idx = 1 + ib;
              br = f_c + idx;
              if ((f_c > 0) && (br <= 0)) {
                br = MAX_int32_T;
              }

              z_u[ib] = z_u_l[br - 1];
            }

            g_c[0] = h_c_n_l[0] * cameraparams_FocalLength[0] +
              cameraparams_PrincipalPoint[0];
            g_c[1] = h_c_n_l[1] * cameraparams_FocalLength[1] +
              cameraparams_PrincipalPoint[1];
            for (i24 = 0; i24 < 2; i24++) {
              h_c_n_l[i24] = g_c[i24] - z_u[i24];
            }

            if (c_norm(h_c_n_l) < 3.0F) {
              LI_inlier_status_i[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = true;
            }
          }
        }
      }

      br = 0;
      idx = 0;
      for (k = 0; k < 40; k++) {
        if (LI_inlier_status_i[k]) {
          br++;
        }

        if (LI_inlier_status[k]) {
          idx++;
        }
      }

      if ((float)br > idx) {
        br = 0;
        idx = 0;
        for (k = 0; k < 40; k++) {
          LI_inlier_status[k] = LI_inlier_status_i[k];
          if (LI_inlier_status_i[k]) {
            br++;
          }

          if (activeFeatures[k]) {
            idx++;
          }
        }

        num_hyp = -4.6051712F / (float)log((double)(1.0F - (float)br / (float)
          idx));
      }

      hyp_it++;
    }

    emxFree_real32_T(&e_c);
    emxFree_real32_T(&d_c);
    emxFree_real32_T(&c_c);

    //  ros_info('Found %i LI inliers in %i active features', nnz(LI_inlier_status), nnz(activeFeatures)) 
    br = 0;
    for (k = 0; k < 40; k++) {
      if (LI_inlier_status[k]) {
        br++;
      }
    }

    if (br > 3.0F) {
      emxInit_real32_T1(&h_c, 2);
      getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att,
                 b_xt->anchor_states, z_u_l, LI_inlier_status,
                 cameraparams_FocalLength, cameraparams_PrincipalPoint,
                 noiseParameters_image_noise, r, H, S);
      H_idx_0 = (unsigned int)H->size[0];
      i24 = h_c->size[0] * h_c->size[1];
      h_c->size[0] = (int)H_idx_0;
      h_c->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)h_c, i24, (int)sizeof(float));
      b_m = H->size[0];
      i24 = h_c->size[0] * h_c->size[1];
      h_c->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)h_c, i24, (int)sizeof(float));
      for (i24 = 0; i24 < 91; i24++) {
        loop_ub = h_c->size[0];
        for (i25 = 0; i25 < loop_ub; i25++) {
          h_c->data[i25 + h_c->size[0] * i24] = 0.0F;
        }
      }

      if (H->size[0] == 0) {
      } else {
        f_c = H->size[0] * 90;
        idx = 0;
        while ((b_m > 0) && (idx <= f_c)) {
          i24 = idx + b_m;
          for (ic = idx; ic + 1 <= i24; ic++) {
            h_c->data[ic] = 0.0F;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= f_c)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (b_P[ib] != 0.0F) {
              ia = ar;
              i24 = idx + b_m;
              for (ic = idx; ic + 1 <= i24; ic++) {
                ia++;
                h_c->data[ic] += b_P[ib] * H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i24 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)K, i24, (int)sizeof(float));
      loop_ub = H->size[0];
      for (i24 = 0; i24 < loop_ub; i24++) {
        for (i25 = 0; i25 < 91; i25++) {
          K->data[i25 + K->size[0] * i24] = H->data[i24 + H->size[0] * i25];
        }
      }

      H_idx_0 = (unsigned int)h_c->size[0];
      unnamed_idx_1 = (unsigned int)K->size[1];
      i24 = c->size[0] * c->size[1];
      c->size[0] = (int)H_idx_0;
      c->size[1] = (int)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)c, i24, (int)sizeof(float));
      b_m = h_c->size[0];
      i24 = c->size[0] * c->size[1];
      emxEnsureCapacity((emxArray__common *)c, i24, (int)sizeof(float));
      loop_ub = c->size[1];
      for (i24 = 0; i24 < loop_ub; i24++) {
        ar = c->size[0];
        for (i25 = 0; i25 < ar; i25++) {
          c->data[i25 + c->size[0] * i24] = 0.0F;
        }
      }

      if ((h_c->size[0] == 0) || (K->size[1] == 0)) {
      } else {
        f_c = h_c->size[0] * (K->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= f_c)) {
          i24 = idx + b_m;
          for (ic = idx; ic + 1 <= i24; ic++) {
            c->data[ic] = 0.0F;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= f_c)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0F) {
              ia = ar;
              i24 = idx + b_m;
              for (ic = idx; ic + 1 <= i24; ic++) {
                ia++;
                c->data[ic] += K->data[ib] * h_c->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      emxFree_real32_T(&h_c);
      i24 = S->size[0] * S->size[1];
      S->size[0] = c->size[0];
      S->size[1] = c->size[1];
      emxEnsureCapacity((emxArray__common *)S, i24, (int)sizeof(float));
      loop_ub = c->size[0] * c->size[1];
      for (i24 = 0; i24 < loop_ub; i24++) {
        S->data[i24] += c->data[i24];
      }

      i24 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)K, i24, (int)sizeof(float));
      loop_ub = H->size[0];
      for (i24 = 0; i24 < loop_ub; i24++) {
        for (i25 = 0; i25 < 91; i25++) {
          K->data[i25 + K->size[0] * i24] = H->data[i24 + H->size[0] * i25];
        }
      }

      unnamed_idx_1 = (unsigned int)K->size[1];
      i24 = b_c->size[0] * b_c->size[1];
      b_c->size[0] = 91;
      b_c->size[1] = (int)unnamed_idx_1;
      b_c->size[0] = 91;
      emxEnsureCapacity((emxArray__common *)b_c, i24, (int)sizeof(float));
      loop_ub = b_c->size[1];
      for (i24 = 0; i24 < loop_ub; i24++) {
        for (i25 = 0; i25 < 91; i25++) {
          b_c->data[i25 + b_c->size[0] * i24] = 0.0F;
        }
      }

      if (K->size[1] == 0) {
      } else {
        f_c = 91 * (K->size[1] - 1);
        for (idx = 0; idx <= f_c; idx += 91) {
          for (ic = idx + 1; ic <= idx + 91; ic++) {
            b_c->data[ic - 1] = 0.0F;
          }
        }

        br = 0;
        for (idx = 0; idx <= f_c; idx += 91) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0F) {
              ia = ar;
              for (ic = idx; ic + 1 <= idx + 91; ic++) {
                ia++;
                b_c->data[ic] += K->data[ib] * b_P[ia - 1];
              }
            }

            ar += 91;
          }

          br += 91;
        }
      }

      if ((b_c->size[1] == 0) || ((S->size[0] == 0) || (S->size[1] == 0))) {
        unnamed_idx_1 = (unsigned int)S->size[0];
        i24 = K->size[0] * K->size[1];
        K->size[0] = 91;
        K->size[1] = (int)unnamed_idx_1;
        emxEnsureCapacity((emxArray__common *)K, i24, (int)sizeof(float));
        loop_ub = 91 * (int)unnamed_idx_1;
        for (i24 = 0; i24 < loop_ub; i24++) {
          K->data[i24] = 0.0F;
        }
      } else if (S->size[0] == S->size[1]) {
        i24 = K->size[0] * K->size[1];
        K->size[0] = 91;
        K->size[1] = b_c->size[1];
        emxEnsureCapacity((emxArray__common *)K, i24, (int)sizeof(float));
        loop_ub = b_c->size[0] * b_c->size[1];
        for (i24 = 0; i24 < loop_ub; i24++) {
          K->data[i24] = b_c->data[i24];
        }

        lusolve(S, K);
      } else {
        i24 = A->size[0] * A->size[1];
        A->size[0] = S->size[1];
        A->size[1] = S->size[0];
        emxEnsureCapacity((emxArray__common *)A, i24, (int)sizeof(float));
        loop_ub = S->size[0];
        for (i24 = 0; i24 < loop_ub; i24++) {
          ar = S->size[1];
          for (i25 = 0; i25 < ar; i25++) {
            A->data[i25 + A->size[0] * i24] = S->data[i24 + S->size[0] * i25];
          }
        }

        xgeqp3(A, tau, jpvt);
        ia = 0;
        if (A->size[0] < A->size[1]) {
          br = A->size[0];
          idx = A->size[1];
        } else {
          br = A->size[1];
          idx = A->size[0];
        }

        if (br > 0) {
          tol = (float)idx * (float)fabs((double)A->data[0]) * 1.1920929E-7F;
          while ((ia < br) && ((float)fabs((double)A->data[ia + A->size[0] * ia])
                               >= tol)) {
            ia++;
          }
        }

        idx = A->size[1];
        i24 = Y->size[0] * Y->size[1];
        Y->size[0] = idx;
        Y->size[1] = 91;
        emxEnsureCapacity((emxArray__common *)Y, i24, (int)sizeof(float));
        loop_ub = idx * 91;
        for (i24 = 0; i24 < loop_ub; i24++) {
          Y->data[i24] = 0.0F;
        }

        i24 = B->size[0] * B->size[1];
        B->size[0] = b_c->size[1];
        B->size[1] = 91;
        emxEnsureCapacity((emxArray__common *)B, i24, (int)sizeof(float));
        for (i24 = 0; i24 < 91; i24++) {
          loop_ub = b_c->size[1];
          for (i25 = 0; i25 < loop_ub; i25++) {
            B->data[i25 + B->size[0] * i24] = b_c->data[i24 + b_c->size[0] * i25];
          }
        }

        b_m = A->size[0];
        idx = A->size[0];
        br = A->size[1];
        if (idx <= br) {
          br = idx;
        }

        for (idx = 0; idx + 1 <= br; idx++) {
          if (tau->data[idx] != 0.0F) {
            for (k = 0; k < 91; k++) {
              tol = B->data[idx + B->size[0] * k];
              for (ib = idx + 1; ib + 1 <= b_m; ib++) {
                tol += A->data[ib + A->size[0] * idx] * B->data[ib + B->size[0] *
                  k];
              }

              tol *= tau->data[idx];
              if (tol != 0.0F) {
                B->data[idx + B->size[0] * k] -= tol;
                for (ib = idx + 1; ib + 1 <= b_m; ib++) {
                  B->data[ib + B->size[0] * k] -= A->data[ib + A->size[0] * idx]
                    * tol;
                }
              }
            }
          }
        }

        for (k = 0; k < 91; k++) {
          for (ib = 0; ib + 1 <= ia; ib++) {
            Y->data[(jpvt->data[ib] + Y->size[0] * k) - 1] = B->data[ib +
              B->size[0] * k];
          }

          for (idx = ia - 1; idx + 1 > 0; idx--) {
            Y->data[(jpvt->data[idx] + Y->size[0] * k) - 1] /= A->data[idx +
              A->size[0] * idx];
            for (ib = 0; ib + 1 <= idx; ib++) {
              Y->data[(jpvt->data[ib] + Y->size[0] * k) - 1] -= Y->data
                [(jpvt->data[idx] + Y->size[0] * k) - 1] * A->data[ib + A->size
                [0] * idx];
            }
          }
        }

        i24 = K->size[0] * K->size[1];
        K->size[0] = 91;
        K->size[1] = Y->size[0];
        emxEnsureCapacity((emxArray__common *)K, i24, (int)sizeof(float));
        loop_ub = Y->size[0];
        for (i24 = 0; i24 < loop_ub; i24++) {
          for (i25 = 0; i25 < 91; i25++) {
            K->data[i25 + K->size[0] * i24] = Y->data[i24 + Y->size[0] * i25];
          }
        }
      }

      if ((K->size[1] == 1) || (r->size[0] == 1)) {
        for (i24 = 0; i24 < 91; i24++) {
          x_apo[i24] = 0.0F;
          loop_ub = K->size[1];
          for (i25 = 0; i25 < loop_ub; i25++) {
            b_x_apo = x_apo[i24] + K->data[i24 + K->size[0] * i25] * r->data[i25];
            x_apo[i24] = b_x_apo;
          }
        }
      } else {
        memset(&x_apo[0], 0, 91U * sizeof(float));
        ar = 0;
        for (ib = 0; ib + 1 <= K->size[1]; ib++) {
          if (r->data[ib] != 0.0F) {
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

      for (i24 = 0; i24 < 3; i24++) {
        b_xt->robot_state.pos[i24] += x_apo[i24];
      }

      theta = norm(*(float (*)[3])&x_apo[3]) * 0.5F;
      if (theta < 0.244F) {
        for (ib = 0; ib < 3; ib++) {
          dq[ib] = 0.5F * x_apo[ib + 3];
        }

        dq[3] = 1.0F;
      } else {
        dq[0] = 0.5F * x_apo[3] * (float)sin((double)theta) / theta;
        dq[1] = 0.5F * x_apo[4] * (float)sin((double)theta) / theta;
        dq[2] = 0.5F * x_apo[5] * (float)sin((double)theta) / theta;
        dq[3] = (float)cos((double)theta);
      }

      tol = b_norm(dq);
      for (ib = 0; ib < 4; ib++) {
        c_xt[ib] = b_xt->robot_state.att[ib];
        dq[ib] /= tol;
      }

      quatmultJ(dq, c_xt, b_xt->robot_state.att);
      for (i24 = 0; i24 < 3; i24++) {
        b_xt->robot_state.vel[i24] += x_apo[6 + i24];
        b_xt->robot_state.IMU.gyro_bias[i24] += x_apo[9 + i24];
        b_xt->robot_state.IMU.acc_bias[i24] += x_apo[12 + i24];
      }

      theta = norm(*(float (*)[3])&x_apo[15]) * 0.5F;
      if (theta < 0.244F) {
        for (ib = 0; ib < 3; ib++) {
          dq[ib] = 0.5F * x_apo[ib + 15];
        }

        dq[3] = 1.0F;
      } else {
        dq[0] = 0.5F * x_apo[15] * (float)sin((double)theta) / theta;
        dq[1] = 0.5F * x_apo[16] * (float)sin((double)theta) / theta;
        dq[2] = 0.5F * x_apo[17] * (float)sin((double)theta) / theta;
        dq[3] = (float)cos((double)theta);
      }

      tol = b_norm(dq);
      for (ib = 0; ib < 4; ib++) {
        c_xt[ib] = b_xt->origin.att[ib];
        dq[ib] /= tol;
      }

      quatmultJ(dq, c_xt, b_xt->origin.att);
      theta = norm(*(float (*)[3])&x_apo[18]) * 0.5F;
      if (theta < 0.244F) {
        for (ib = 0; ib < 3; ib++) {
          dq[ib] = 0.5F * x_apo[ib + 18];
        }

        dq[3] = 1.0F;
      } else {
        dq[0] = 0.5F * x_apo[18] * (float)sin((double)theta) / theta;
        dq[1] = 0.5F * x_apo[19] * (float)sin((double)theta) / theta;
        dq[2] = 0.5F * x_apo[20] * (float)sin((double)theta) / theta;
        dq[3] = (float)cos((double)theta);
      }

      tol = b_norm(dq);
      for (ib = 0; ib < 4; ib++) {
        c_xt[ib] = b_xt->robot_state.IMU.att[ib];
        dq[ib] /= tol;
      }

      quatmultJ(dq, c_xt, b_xt->robot_state.IMU.att);
      for (i24 = 0; i24 < 3; i24++) {
        b_xt->robot_state.IMU.pos[i24] += x_apo[21 + i24];
      }

      for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
        f_c = anchorIdx * 14 + 21;
        ar = 21 + anchorIdx * 14;
        for (i24 = 0; i24 < 3; i24++) {
          b_xt->anchor_states[anchorIdx].pos[i24] += x_apo[i24 + f_c];
          fp_scaled[i24] = x_apo[(i24 + ar) + 3];
        }

        theta = norm(fp_scaled) * 0.5F;
        if (theta < 0.244F) {
          for (ib = 0; ib < 3; ib++) {
            dq[ib] = 0.5F * fp_scaled[ib];
          }

          dq[3] = 1.0F;
        } else {
          dq[0] = 0.5F * x_apo[ar + 3] * (float)sin((double)theta) / theta;
          dq[1] = 0.5F * x_apo[ar + 4] * (float)sin((double)theta) / theta;
          dq[2] = 0.5F * x_apo[ar + 5] * (float)sin((double)theta) / theta;
          dq[3] = (float)cos((double)theta);
        }

        tol = b_norm(dq);
        for (ib = 0; ib < 4; ib++) {
          c_xt[ib] = b_xt->anchor_states[anchorIdx].att[ib];
          dq[ib] /= tol;
        }

        quatmultJ(dq, c_xt, b_xt->anchor_states[anchorIdx].att);
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              1) {
            //  only update active features
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              inverse_depth += x_apo[(anchorIdx * 14 + featureIdx) + 27];
            if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0F) {
              updateVect[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = 0;
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                0;
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                status_idx = 0;
            }
          }
        }
      }

      if ((K->size[1] == 1) || (H->size[0] == 1)) {
        for (i24 = 0; i24 < 91; i24++) {
          for (i25 = 0; i25 < 91; i25++) {
            i_c[i24 + 91 * i25] = 0.0F;
            loop_ub = K->size[1];
            for (idx = 0; idx < loop_ub; idx++) {
              i_c[i24 + 91 * i25] += K->data[i24 + K->size[0] * idx] * H->
                data[idx + H->size[0] * i25];
            }
          }
        }
      } else {
        k = K->size[1];
        memset(&i_c[0], 0, 8281U * sizeof(float));
        for (idx = 0; idx <= 8191; idx += 91) {
          for (ic = idx; ic + 1 <= idx + 91; ic++) {
            i_c[ic] = 0.0F;
          }
        }

        br = 0;
        for (idx = 0; idx <= 8191; idx += 91) {
          ar = 0;
          i24 = br + k;
          for (ib = br; ib + 1 <= i24; ib++) {
            if (H->data[ib] != 0.0F) {
              ia = ar;
              for (ic = idx; ic + 1 <= idx + 91; ic++) {
                ia++;
                i_c[ic] += H->data[ib] * K->data[ia - 1];
              }
            }

            ar += 91;
          }

          br += k;
        }
      }

      for (i24 = 0; i24 < 91; i24++) {
        for (i25 = 0; i25 < 91; i25++) {
          fv14[i25 + 91 * i24] = (float)iv14[i25 + 91 * i24] - i_c[i25 + 91 *
            i24];
        }
      }

      for (i24 = 0; i24 < 91; i24++) {
        for (i25 = 0; i25 < 91; i25++) {
          fv15[i24 + 91 * i25] = 0.0F;
          for (idx = 0; idx < 91; idx++) {
            fv15[i24 + 91 * i25] += fv14[i24 + 91 * idx] * b_P[idx + 91 * i25];
          }
        }
      }

      for (i24 = 0; i24 < 91; i24++) {
        memcpy(&b_P[i24 * 91], &fv15[i24 * 91], 91U * sizeof(float));
      }
    } else {
      for (ib = 0; ib < 40; ib++) {
        LI_inlier_status[ib] = false;
      }
    }
  }

  // % D Partial EKF update using high-innovation inliers
  //  high innovation inliers (ordered like updateVect)
  for (ib = 0; ib < 40; ib++) {
    b_activeFeatures = (activeFeatures[ib] && (!LI_inlier_status[ib]));
    activeFeatures[ib] = b_activeFeatures;
    LI_inlier_status[ib] = b_activeFeatures;
  }

  idx = 0;
  br = 1;
  exitg1 = false;
  while ((!exitg1) && (br < 41)) {
    guard1 = false;
    if (activeFeatures[br - 1]) {
      idx++;
      ii_data[idx - 1] = (signed char)br;
      if (idx >= 40) {
        exitg1 = true;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      br++;
    }
  }

  if (1 > idx) {
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }

  i24 = out->size[0];
  out->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)out, i24, (int)sizeof(float));
  for (i24 = 0; i24 < loop_ub; i24++) {
    out->data[i24] = ii_data[i24];
  }

  if (any(activeFeatures)) {
    emxInit_real32_T1(&b_K, 2);
    emxInit_real32_T1(&b_H, 2);
    i24 = b_K->size[0] * b_K->size[1];
    b_K->size[0] = 1;
    b_K->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)b_K, i24, (int)sizeof(float));
    b_K->data[0] = 0.0F;

    //  for coder
    i24 = b_H->size[0] * b_H->size[1];
    b_H->size[0] = 1;
    b_H->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)b_H, i24, (int)sizeof(float));
    b_H->data[0] = 0.0F;

    //  for coder
    it = 1;
    emxInit_real32_T1(&j_c, 2);
    emxInit_real32_T1(&k_c, 2);
    emxInit_real32_T1(&l_c, 2);
    emxInit_real32_T1(&m_c, 2);
    emxInit_real32_T1(&n_c, 2);
    emxInit_real32_T1(&c_H, 2);
    emxInit_real32_T1(&d_H, 2);
    emxInit_real32_T1(&e_H, 2);
    while (it <= c_VIOParameters_max_ekf_iterati) {
      getScaledMap(b_xt);

      //  build the map according to the current estimate
      getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att,
                 b_xt->anchor_states, z_u_l, LI_inlier_status,
                 cameraparams_FocalLength, cameraparams_PrincipalPoint,
                 noiseParameters_image_noise, r, H, c);
      i24 = b_H->size[0] * b_H->size[1];
      b_H->size[0] = H->size[0];
      b_H->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)b_H, i24, (int)sizeof(float));
      loop_ub = H->size[0] * H->size[1];
      for (i24 = 0; i24 < loop_ub; i24++) {
        b_H->data[i24] = H->data[i24];
      }

      //  the residual is ordered by anchors/features, not like updateVect
      H_idx_0 = (unsigned int)H->size[0];
      i24 = j_c->size[0] * j_c->size[1];
      j_c->size[0] = (int)H_idx_0;
      j_c->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)j_c, i24, (int)sizeof(float));
      b_m = H->size[0];
      i24 = j_c->size[0] * j_c->size[1];
      j_c->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)j_c, i24, (int)sizeof(float));
      for (i24 = 0; i24 < 91; i24++) {
        loop_ub = j_c->size[0];
        for (i25 = 0; i25 < loop_ub; i25++) {
          j_c->data[i25 + j_c->size[0] * i24] = 0.0F;
        }
      }

      if (H->size[0] == 0) {
      } else {
        f_c = H->size[0] * 90;
        idx = 0;
        while ((b_m > 0) && (idx <= f_c)) {
          i24 = idx + b_m;
          for (ic = idx; ic + 1 <= i24; ic++) {
            j_c->data[ic] = 0.0F;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= f_c)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (b_P[ib] != 0.0F) {
              ia = ar;
              i24 = idx + b_m;
              for (ic = idx; ic + 1 <= i24; ic++) {
                ia++;
                j_c->data[ic] += b_P[ib] * H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i24 = c_H->size[0] * c_H->size[1];
      c_H->size[0] = 91;
      c_H->size[1] = H->size[0];
      emxEnsureCapacity((emxArray__common *)c_H, i24, (int)sizeof(float));
      loop_ub = H->size[0];
      for (i24 = 0; i24 < loop_ub; i24++) {
        for (i25 = 0; i25 < 91; i25++) {
          c_H->data[i25 + c_H->size[0] * i24] = H->data[i24 + H->size[0] * i25];
        }
      }

      br = H->size[0];
      i24 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = br;
      emxEnsureCapacity((emxArray__common *)K, i24, (int)sizeof(float));
      for (i24 = 0; i24 < br; i24++) {
        for (i25 = 0; i25 < 91; i25++) {
          K->data[i25 + K->size[0] * i24] = c_H->data[i25 + 91 * i24];
        }
      }

      H_idx_0 = (unsigned int)j_c->size[0];
      unnamed_idx_1 = (unsigned int)K->size[1];
      i24 = k_c->size[0] * k_c->size[1];
      k_c->size[0] = (int)H_idx_0;
      k_c->size[1] = (int)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)k_c, i24, (int)sizeof(float));
      b_m = j_c->size[0];
      i24 = k_c->size[0] * k_c->size[1];
      emxEnsureCapacity((emxArray__common *)k_c, i24, (int)sizeof(float));
      loop_ub = k_c->size[1];
      for (i24 = 0; i24 < loop_ub; i24++) {
        ar = k_c->size[0];
        for (i25 = 0; i25 < ar; i25++) {
          k_c->data[i25 + k_c->size[0] * i24] = 0.0F;
        }
      }

      if ((j_c->size[0] == 0) || (K->size[1] == 0)) {
      } else {
        f_c = j_c->size[0] * (K->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= f_c)) {
          i24 = idx + b_m;
          for (ic = idx; ic + 1 <= i24; ic++) {
            k_c->data[ic] = 0.0F;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= f_c)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0F) {
              ia = ar;
              i24 = idx + b_m;
              for (ic = idx; ic + 1 <= i24; ic++) {
                ia++;
                k_c->data[ic] += K->data[ib] * j_c->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i24 = S->size[0] * S->size[1];
      S->size[0] = k_c->size[0];
      S->size[1] = k_c->size[1];
      emxEnsureCapacity((emxArray__common *)S, i24, (int)sizeof(float));
      loop_ub = k_c->size[0] * k_c->size[1];
      for (i24 = 0; i24 < loop_ub; i24++) {
        S->data[i24] = k_c->data[i24] + c->data[i24];
      }

      for (k = 0; k < out->size[0]; k++) {
        f_c = k << 1;
        ar = (k << 1) - 1;
        ib = (k << 1) - 1;
        for (i24 = 0; i24 < 2; i24++) {
          o_c[i24] = (signed char)((i24 + (signed char)f_c) + 1);
          p_c[i24] = (signed char)((i24 + (signed char)(ar + 1)) + 1);
          q_c[i24] = (signed char)((i24 + (signed char)(ib + 1)) + 1);
        }

        if ((float)fabs((double)S->data[(ar + S->size[0] * (1 + ib)) + 2]) >
            (float)fabs((double)S->data[(ar + S->size[0] * (1 + ib)) + 1])) {
          idx = 1;
          br = 0;
        } else {
          idx = 0;
          br = 1;
        }

        tol = S->data[((br + ar) + S->size[0] * (1 + ib)) + 1] / S->data[((idx +
          ar) + S->size[0] * (1 + ib)) + 1];
        g_c[idx] = r->data[o_c[0] - 1] / S->data[(p_c[idx] + S->size[0] * (q_c[0]
          - 1)) - 1];
        g_c[br] = (r->data[o_c[1] - 1] - g_c[idx] * S->data[(p_c[idx] + S->size
                    [0] * (q_c[1] - 1)) - 1]) / (S->data[(p_c[br] + S->size[0] *
          (q_c[1] - 1)) - 1] - tol * S->data[(p_c[idx] + S->size[0] * (q_c[1] -
          1)) - 1]);
        g_c[idx] -= g_c[br] * tol;
        f_c = k << 1;
        for (i24 = 0; i24 < 2; i24++) {
          h_c_n_l[i24] = r->data[i24 + f_c];
        }

        tol = 0.0F;
        for (i24 = 0; i24 < 2; i24++) {
          tol += g_c[i24] * h_c_n_l[i24];
        }

        if (tol > 6.0F) {
          f_c = (k << 1) - 1;
          for (i24 = 0; i24 < 2; i24++) {
            r->data[(i24 + f_c) + 1] = 0.0F;
          }

          f_c = k << 1;
          for (i24 = 0; i24 < 91; i24++) {
            for (i25 = 0; i25 < 2; i25++) {
              b_H->data[(i25 + f_c) + b_H->size[0] * i24] = 0.0F;
            }
          }

          if (it == c_VIOParameters_max_ekf_iterati) {
            LI_inlier_status[(int)out->data[k] - 1] = false;

            //  only reject the feature if its still bad in last iteration, otherwise just dont use for this update 
          }

          //          ros_info('rejecting %i', HI_ind(k))
          if (updateVect[(int)out->data[k] - 1] == 2) {
            b_ros_error();
          }
        }
      }

      H_idx_0 = (unsigned int)b_H->size[0];
      i24 = l_c->size[0] * l_c->size[1];
      l_c->size[0] = (int)H_idx_0;
      l_c->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)l_c, i24, (int)sizeof(float));
      b_m = b_H->size[0];
      i24 = l_c->size[0] * l_c->size[1];
      l_c->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)l_c, i24, (int)sizeof(float));
      for (i24 = 0; i24 < 91; i24++) {
        loop_ub = l_c->size[0];
        for (i25 = 0; i25 < loop_ub; i25++) {
          l_c->data[i25 + l_c->size[0] * i24] = 0.0F;
        }
      }

      if (b_H->size[0] == 0) {
      } else {
        f_c = b_H->size[0] * 90;
        idx = 0;
        while ((b_m > 0) && (idx <= f_c)) {
          i24 = idx + b_m;
          for (ic = idx; ic + 1 <= i24; ic++) {
            l_c->data[ic] = 0.0F;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= f_c)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (b_P[ib] != 0.0F) {
              ia = ar;
              i24 = idx + b_m;
              for (ic = idx; ic + 1 <= i24; ic++) {
                ia++;
                l_c->data[ic] += b_P[ib] * b_H->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i24 = d_H->size[0] * d_H->size[1];
      d_H->size[0] = b_H->size[1];
      d_H->size[1] = b_H->size[0];
      emxEnsureCapacity((emxArray__common *)d_H, i24, (int)sizeof(float));
      loop_ub = b_H->size[0];
      for (i24 = 0; i24 < loop_ub; i24++) {
        ar = b_H->size[1];
        for (i25 = 0; i25 < ar; i25++) {
          d_H->data[i25 + d_H->size[0] * i24] = b_H->data[i24 + b_H->size[0] *
            i25];
        }
      }

      br = b_H->size[0];
      i24 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = br;
      emxEnsureCapacity((emxArray__common *)K, i24, (int)sizeof(float));
      for (i24 = 0; i24 < br; i24++) {
        for (i25 = 0; i25 < 91; i25++) {
          K->data[i25 + K->size[0] * i24] = d_H->data[i25 + 91 * i24];
        }
      }

      H_idx_0 = (unsigned int)l_c->size[0];
      unnamed_idx_1 = (unsigned int)K->size[1];
      i24 = m_c->size[0] * m_c->size[1];
      m_c->size[0] = (int)H_idx_0;
      m_c->size[1] = (int)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)m_c, i24, (int)sizeof(float));
      b_m = l_c->size[0];
      i24 = m_c->size[0] * m_c->size[1];
      emxEnsureCapacity((emxArray__common *)m_c, i24, (int)sizeof(float));
      loop_ub = m_c->size[1];
      for (i24 = 0; i24 < loop_ub; i24++) {
        ar = m_c->size[0];
        for (i25 = 0; i25 < ar; i25++) {
          m_c->data[i25 + m_c->size[0] * i24] = 0.0F;
        }
      }

      if ((l_c->size[0] == 0) || (K->size[1] == 0)) {
      } else {
        f_c = l_c->size[0] * (K->size[1] - 1);
        idx = 0;
        while ((b_m > 0) && (idx <= f_c)) {
          i24 = idx + b_m;
          for (ic = idx; ic + 1 <= i24; ic++) {
            m_c->data[ic] = 0.0F;
          }

          idx += b_m;
        }

        br = 0;
        idx = 0;
        while ((b_m > 0) && (idx <= f_c)) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0F) {
              ia = ar;
              i24 = idx + b_m;
              for (ic = idx; ic + 1 <= i24; ic++) {
                ia++;
                m_c->data[ic] += K->data[ib] * l_c->data[ia - 1];
              }
            }

            ar += b_m;
          }

          br += 91;
          idx += b_m;
        }
      }

      i24 = S->size[0] * S->size[1];
      S->size[0] = m_c->size[0];
      S->size[1] = m_c->size[1];
      emxEnsureCapacity((emxArray__common *)S, i24, (int)sizeof(float));
      loop_ub = m_c->size[0] * m_c->size[1];
      for (i24 = 0; i24 < loop_ub; i24++) {
        S->data[i24] = m_c->data[i24] + c->data[i24];
      }

      i24 = e_H->size[0] * e_H->size[1];
      e_H->size[0] = b_H->size[1];
      e_H->size[1] = b_H->size[0];
      emxEnsureCapacity((emxArray__common *)e_H, i24, (int)sizeof(float));
      loop_ub = b_H->size[0];
      for (i24 = 0; i24 < loop_ub; i24++) {
        ar = b_H->size[1];
        for (i25 = 0; i25 < ar; i25++) {
          e_H->data[i25 + e_H->size[0] * i24] = b_H->data[i24 + b_H->size[0] *
            i25];
        }
      }

      br = b_H->size[0];
      i24 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = br;
      emxEnsureCapacity((emxArray__common *)K, i24, (int)sizeof(float));
      for (i24 = 0; i24 < br; i24++) {
        for (i25 = 0; i25 < 91; i25++) {
          K->data[i25 + K->size[0] * i24] = e_H->data[i25 + 91 * i24];
        }
      }

      unnamed_idx_1 = (unsigned int)K->size[1];
      i24 = n_c->size[0] * n_c->size[1];
      n_c->size[0] = 91;
      n_c->size[1] = (int)unnamed_idx_1;
      n_c->size[0] = 91;
      emxEnsureCapacity((emxArray__common *)n_c, i24, (int)sizeof(float));
      loop_ub = n_c->size[1];
      for (i24 = 0; i24 < loop_ub; i24++) {
        for (i25 = 0; i25 < 91; i25++) {
          n_c->data[i25 + n_c->size[0] * i24] = 0.0F;
        }
      }

      if (K->size[1] == 0) {
      } else {
        f_c = 91 * (K->size[1] - 1);
        for (idx = 0; idx <= f_c; idx += 91) {
          for (ic = idx + 1; ic <= idx + 91; ic++) {
            n_c->data[ic - 1] = 0.0F;
          }
        }

        br = 0;
        for (idx = 0; idx <= f_c; idx += 91) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 91; ib++) {
            if (K->data[ib] != 0.0F) {
              ia = ar;
              for (ic = idx; ic + 1 <= idx + 91; ic++) {
                ia++;
                n_c->data[ic] += K->data[ib] * b_P[ia - 1];
              }
            }

            ar += 91;
          }

          br += 91;
        }
      }

      if ((n_c->size[1] == 0) || ((S->size[0] == 0) || (S->size[1] == 0))) {
        unnamed_idx_1 = (unsigned int)S->size[0];
        i24 = b_c->size[0] * b_c->size[1];
        b_c->size[0] = 91;
        b_c->size[1] = (int)unnamed_idx_1;
        emxEnsureCapacity((emxArray__common *)b_c, i24, (int)sizeof(float));
        loop_ub = 91 * (int)unnamed_idx_1;
        for (i24 = 0; i24 < loop_ub; i24++) {
          b_c->data[i24] = 0.0F;
        }
      } else if (S->size[0] == S->size[1]) {
        i24 = b_c->size[0] * b_c->size[1];
        b_c->size[0] = 91;
        b_c->size[1] = n_c->size[1];
        emxEnsureCapacity((emxArray__common *)b_c, i24, (int)sizeof(float));
        loop_ub = n_c->size[0] * n_c->size[1];
        for (i24 = 0; i24 < loop_ub; i24++) {
          b_c->data[i24] = n_c->data[i24];
        }

        lusolve(S, b_c);
      } else {
        i24 = A->size[0] * A->size[1];
        A->size[0] = S->size[1];
        A->size[1] = S->size[0];
        emxEnsureCapacity((emxArray__common *)A, i24, (int)sizeof(float));
        loop_ub = S->size[0];
        for (i24 = 0; i24 < loop_ub; i24++) {
          ar = S->size[1];
          for (i25 = 0; i25 < ar; i25++) {
            A->data[i25 + A->size[0] * i24] = S->data[i24 + S->size[0] * i25];
          }
        }

        xgeqp3(A, tau, jpvt);
        ia = 0;
        if (A->size[0] < A->size[1]) {
          br = A->size[0];
          idx = A->size[1];
        } else {
          br = A->size[1];
          idx = A->size[0];
        }

        if (br > 0) {
          tol = (float)idx * (float)fabs((double)A->data[0]) * 1.1920929E-7F;
          while ((ia < br) && ((float)fabs((double)A->data[ia + A->size[0] * ia])
                               >= tol)) {
            ia++;
          }
        }

        idx = A->size[1];
        i24 = Y->size[0] * Y->size[1];
        Y->size[0] = idx;
        Y->size[1] = 91;
        emxEnsureCapacity((emxArray__common *)Y, i24, (int)sizeof(float));
        loop_ub = idx * 91;
        for (i24 = 0; i24 < loop_ub; i24++) {
          Y->data[i24] = 0.0F;
        }

        i24 = B->size[0] * B->size[1];
        B->size[0] = n_c->size[1];
        B->size[1] = 91;
        emxEnsureCapacity((emxArray__common *)B, i24, (int)sizeof(float));
        for (i24 = 0; i24 < 91; i24++) {
          loop_ub = n_c->size[1];
          for (i25 = 0; i25 < loop_ub; i25++) {
            B->data[i25 + B->size[0] * i24] = n_c->data[i24 + n_c->size[0] * i25];
          }
        }

        b_m = A->size[0];
        idx = A->size[0];
        br = A->size[1];
        if (idx <= br) {
          br = idx;
        }

        for (idx = 0; idx + 1 <= br; idx++) {
          if (tau->data[idx] != 0.0F) {
            for (k = 0; k < 91; k++) {
              tol = B->data[idx + B->size[0] * k];
              for (ib = idx + 1; ib + 1 <= b_m; ib++) {
                tol += A->data[ib + A->size[0] * idx] * B->data[ib + B->size[0] *
                  k];
              }

              tol *= tau->data[idx];
              if (tol != 0.0F) {
                B->data[idx + B->size[0] * k] -= tol;
                for (ib = idx + 1; ib + 1 <= b_m; ib++) {
                  B->data[ib + B->size[0] * k] -= A->data[ib + A->size[0] * idx]
                    * tol;
                }
              }
            }
          }
        }

        for (k = 0; k < 91; k++) {
          for (ib = 0; ib + 1 <= ia; ib++) {
            Y->data[(jpvt->data[ib] + Y->size[0] * k) - 1] = B->data[ib +
              B->size[0] * k];
          }

          for (idx = ia - 1; idx + 1 > 0; idx--) {
            Y->data[(jpvt->data[idx] + Y->size[0] * k) - 1] /= A->data[idx +
              A->size[0] * idx];
            for (ib = 0; ib + 1 <= idx; ib++) {
              Y->data[(jpvt->data[ib] + Y->size[0] * k) - 1] -= Y->data
                [(jpvt->data[idx] + Y->size[0] * k) - 1] * A->data[ib + A->size
                [0] * idx];
            }
          }
        }

        i24 = b_c->size[0] * b_c->size[1];
        b_c->size[0] = 91;
        b_c->size[1] = Y->size[0];
        emxEnsureCapacity((emxArray__common *)b_c, i24, (int)sizeof(float));
        loop_ub = Y->size[0];
        for (i24 = 0; i24 < loop_ub; i24++) {
          for (i25 = 0; i25 < 91; i25++) {
            b_c->data[i25 + b_c->size[0] * i24] = Y->data[i24 + Y->size[0] * i25];
          }
        }
      }

      i24 = b_K->size[0] * b_K->size[1];
      b_K->size[0] = 91;
      b_K->size[1] = b_c->size[1];
      emxEnsureCapacity((emxArray__common *)b_K, i24, (int)sizeof(float));
      loop_ub = b_c->size[0] * b_c->size[1];
      for (i24 = 0; i24 < loop_ub; i24++) {
        b_K->data[i24] = b_c->data[i24];
      }

      if ((b_c->size[1] == 1) || (r->size[0] == 1)) {
        for (i24 = 0; i24 < 91; i24++) {
          x_apo_data[i24] = 0.0F;
          loop_ub = b_c->size[1];
          for (i25 = 0; i25 < loop_ub; i25++) {
            b_x_apo_data = x_apo_data[i24] + b_c->data[i24 + b_c->size[0] * i25]
              * r->data[i25];
            x_apo_data[i24] = b_x_apo_data;
          }
        }
      } else {
        k = b_c->size[1];
        memset(&x_apo_data[0], 0, 91U * sizeof(float));
        for (ic = 1; ic < 92; ic++) {
          x_apo_data[ic - 1] = 0.0F;
        }

        ar = 0;
        for (ib = 0; ib + 1 <= k; ib++) {
          if (r->data[ib] != 0.0F) {
            ia = ar;
            for (ic = 0; ic + 1 < 92; ic++) {
              ia++;
              x_apo_data[ic] += r->data[ib] * b_c->data[ia - 1];
            }
          }

          ar += 91;
        }
      }

      for (i24 = 0; i24 < 3; i24++) {
        b_xt->robot_state.pos[i24] += x_apo_data[i24];
      }

      theta = norm(*(float (*)[3])&x_apo_data[3]) * 0.5F;
      if (theta < 0.244F) {
        for (i24 = 0; i24 < 3; i24++) {
          dq[i24] = 0.5F * x_apo_data[3 + i24];
        }

        dq[3] = 1.0F;
      } else {
        dq[0] = 0.5F * x_apo_data[3] * (float)sin((double)theta) / theta;
        dq[1] = 0.5F * x_apo_data[4] * (float)sin((double)theta) / theta;
        dq[2] = 0.5F * x_apo_data[5] * (float)sin((double)theta) / theta;
        dq[3] = (float)cos((double)theta);
      }

      tol = b_norm(dq);
      for (ib = 0; ib < 4; ib++) {
        c_xt[ib] = b_xt->robot_state.att[ib];
        dq[ib] /= tol;
      }

      quatmultJ(dq, c_xt, b_xt->robot_state.att);
      for (i24 = 0; i24 < 3; i24++) {
        b_xt->robot_state.vel[i24] += x_apo_data[6 + i24];
        b_xt->robot_state.IMU.gyro_bias[i24] += x_apo_data[9 + i24];
        b_xt->robot_state.IMU.acc_bias[i24] += x_apo_data[12 + i24];
      }

      theta = norm(*(float (*)[3])&x_apo_data[15]) * 0.5F;
      if (theta < 0.244F) {
        for (i24 = 0; i24 < 3; i24++) {
          dq[i24] = 0.5F * x_apo_data[15 + i24];
        }

        dq[3] = 1.0F;
      } else {
        dq[0] = 0.5F * x_apo_data[15] * (float)sin((double)theta) / theta;
        dq[1] = 0.5F * x_apo_data[16] * (float)sin((double)theta) / theta;
        dq[2] = 0.5F * x_apo_data[17] * (float)sin((double)theta) / theta;
        dq[3] = (float)cos((double)theta);
      }

      tol = b_norm(dq);
      for (ib = 0; ib < 4; ib++) {
        c_xt[ib] = b_xt->origin.att[ib];
        dq[ib] /= tol;
      }

      quatmultJ(dq, c_xt, b_xt->origin.att);
      theta = norm(*(float (*)[3])&x_apo_data[18]) * 0.5F;
      if (theta < 0.244F) {
        for (i24 = 0; i24 < 3; i24++) {
          dq[i24] = 0.5F * x_apo_data[18 + i24];
        }

        dq[3] = 1.0F;
      } else {
        dq[0] = 0.5F * x_apo_data[18] * (float)sin((double)theta) / theta;
        dq[1] = 0.5F * x_apo_data[19] * (float)sin((double)theta) / theta;
        dq[2] = 0.5F * x_apo_data[20] * (float)sin((double)theta) / theta;
        dq[3] = (float)cos((double)theta);
      }

      tol = b_norm(dq);
      for (ib = 0; ib < 4; ib++) {
        c_xt[ib] = b_xt->robot_state.IMU.att[ib];
        dq[ib] /= tol;
      }

      quatmultJ(dq, c_xt, b_xt->robot_state.IMU.att);
      for (i24 = 0; i24 < 3; i24++) {
        b_xt->robot_state.IMU.pos[i24] += x_apo_data[21 + i24];
      }

      for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
        f_c = anchorIdx * 14 + 21;
        ar = 21 + anchorIdx * 14;
        for (i24 = 0; i24 < 3; i24++) {
          b_xt->anchor_states[anchorIdx].pos[i24] += x_apo_data[i24 + f_c];
          fp_scaled[i24] = x_apo_data[(i24 + ar) + 3];
        }

        theta = norm(fp_scaled) * 0.5F;
        if (theta < 0.244F) {
          for (ib = 0; ib < 3; ib++) {
            dq[ib] = 0.5F * fp_scaled[ib];
          }

          dq[3] = 1.0F;
        } else {
          dq[0] = 0.5F * x_apo_data[ar + 3] * (float)sin((double)theta) / theta;
          dq[1] = 0.5F * x_apo_data[ar + 4] * (float)sin((double)theta) / theta;
          dq[2] = 0.5F * x_apo_data[ar + 5] * (float)sin((double)theta) / theta;
          dq[3] = (float)cos((double)theta);
        }

        tol = b_norm(dq);
        for (ib = 0; ib < 4; ib++) {
          c_xt[ib] = b_xt->anchor_states[anchorIdx].att[ib];
          dq[ib] /= tol;
        }

        quatmultJ(dq, c_xt, b_xt->anchor_states[anchorIdx].att);
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              1) {
            //  only update active features
            if (LI_inlier_status[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1]) {
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth += x_apo_data[(anchorIdx * 14 + featureIdx) + 27];
              if ((b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                   inverse_depth < 0.0F) && (it ==
                   c_VIOParameters_max_ekf_iterati)) {
                //  only reject if we are done iterating
                updateVect[b_xt->anchor_states[anchorIdx]
                  .feature_states[featureIdx].status_idx - 1] = 0;
                b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status
                  = 0;
                b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                  status_idx = 0;
              }
            } else {
              if (activeFeatures[b_xt->anchor_states[anchorIdx]
                  .feature_states[featureIdx].status_idx - 1] && (it ==
                   c_VIOParameters_max_ekf_iterati)) {
                //  if it is not a HI inlier, but was a candidate, it was rejected by mahalanobis 
                //  only reject if we are done iterating
                c_ros_info(b_xt->anchor_states[anchorIdx]
                           .feature_states[featureIdx].status_idx, featureIdx +
                           1, anchorIdx + 1);
                updateVect[b_xt->anchor_states[anchorIdx]
                  .feature_states[featureIdx].status_idx - 1] = 0;
                b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status
                  = 0;
                b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                  status_idx = 0;
              }
            }
          }
        }
      }

      it++;
    }

    emxFree_real32_T(&e_H);
    emxFree_real32_T(&d_H);
    emxFree_real32_T(&c_H);
    emxFree_real32_T(&n_c);
    emxFree_real32_T(&m_c);
    emxFree_real32_T(&l_c);
    emxFree_real32_T(&k_c);
    emxFree_real32_T(&j_c);
    if ((b_K->size[1] == 1) || (b_H->size[0] == 1)) {
      br = b_K->size[0];
      loop_ub = b_K->size[0];
      for (i24 = 0; i24 < loop_ub; i24++) {
        ar = b_H->size[1];
        for (i25 = 0; i25 < ar; i25++) {
          c_data[i24 + br * i25] = 0.0F;
          ib = b_K->size[1];
          for (idx = 0; idx < ib; idx++) {
            c_data[i24 + br * i25] += b_K->data[i24 + b_K->size[0] * idx] *
              b_H->data[idx + b_H->size[0] * i25];
          }
        }
      }
    } else {
      k = b_K->size[1];
      H_idx_0 = (unsigned int)b_K->size[0];
      unnamed_idx_1 = (unsigned int)b_H->size[1];
      br = (int)H_idx_0;
      b_m = b_K->size[0];
      loop_ub = (int)unnamed_idx_1;
      for (i24 = 0; i24 < loop_ub; i24++) {
        for (i25 = 0; i25 < br; i25++) {
          c_data[i25 + (int)H_idx_0 * i24] = 0.0F;
        }
      }

      f_c = b_K->size[0] * (b_H->size[1] - 1);
      for (idx = 0; idx <= f_c; idx += b_m) {
        i24 = idx + b_m;
        for (ic = idx; ic + 1 <= i24; ic++) {
          c_data[ic] = 0.0F;
        }
      }

      br = 0;
      for (idx = 0; idx <= f_c; idx += b_m) {
        ar = 0;
        i24 = br + k;
        for (ib = br; ib + 1 <= i24; ib++) {
          if (b_H->data[ib] != 0.0F) {
            ia = ar;
            i25 = idx + b_m;
            for (ic = idx; ic + 1 <= i25; ic++) {
              ia++;
              c_data[ic] += b_H->data[ib] * b_K->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += k;
      }
    }

    emxFree_real32_T(&b_H);
    emxFree_real32_T(&b_K);
    for (i24 = 0; i24 < 91; i24++) {
      for (i25 = 0; i25 < 91; i25++) {
        fv14[i25 + 91 * i24] = (float)iv14[i25 + 91 * i24] - c_data[i25 + 91 *
          i24];
      }
    }

    for (i24 = 0; i24 < 91; i24++) {
      for (i25 = 0; i25 < 91; i25++) {
        fv16[i24 + 91 * i25] = 0.0F;
        for (idx = 0; idx < 91; idx++) {
          fv16[i24 + 91 * i25] += fv14[i24 + 91 * idx] * b_P[idx + 91 * i25];
        }
      }
    }

    for (i24 = 0; i24 < 91; i24++) {
      memcpy(&b_P[i24 * 91], &fv16[i24 * 91], 91U * sizeof(float));
    }
  }

  emxFree_real32_T(&b_c);
  emxFree_real32_T(&c);
  emxFree_real32_T(&out);

  // % Update the delayed initialization features
  if (c_VIOParameters_delayed_initial) {
    emxInit_real32_T1(&r_c, 2);
    getScaledMap(b_xt);
    getH_R_res(b_xt->robot_state.pos, b_xt->robot_state.att, b_xt->anchor_states,
               z_u_l, delayedFeatures, cameraparams_FocalLength,
               cameraparams_PrincipalPoint, noiseParameters_image_noise, r, H, S);
    H_idx_0 = (unsigned int)H->size[0];
    i24 = r_c->size[0] * r_c->size[1];
    r_c->size[0] = (int)H_idx_0;
    r_c->size[1] = 91;
    emxEnsureCapacity((emxArray__common *)r_c, i24, (int)sizeof(float));
    b_m = H->size[0];
    i24 = r_c->size[0] * r_c->size[1];
    r_c->size[1] = 91;
    emxEnsureCapacity((emxArray__common *)r_c, i24, (int)sizeof(float));
    for (i24 = 0; i24 < 91; i24++) {
      loop_ub = r_c->size[0];
      for (i25 = 0; i25 < loop_ub; i25++) {
        r_c->data[i25 + r_c->size[0] * i24] = 0.0F;
      }
    }

    if (H->size[0] == 0) {
    } else {
      f_c = H->size[0] * 90;
      idx = 0;
      while ((b_m > 0) && (idx <= f_c)) {
        i24 = idx + b_m;
        for (ic = idx; ic + 1 <= i24; ic++) {
          r_c->data[ic] = 0.0F;
        }

        idx += b_m;
      }

      br = 0;
      idx = 0;
      while ((b_m > 0) && (idx <= f_c)) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 91; ib++) {
          if (b_P[ib] != 0.0F) {
            ia = ar;
            i24 = idx + b_m;
            for (ic = idx; ic + 1 <= i24; ic++) {
              ia++;
              r_c->data[ic] += b_P[ib] * H->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += 91;
        idx += b_m;
      }
    }

    i24 = K->size[0] * K->size[1];
    K->size[0] = 91;
    K->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)K, i24, (int)sizeof(float));
    loop_ub = H->size[0];
    for (i24 = 0; i24 < loop_ub; i24++) {
      for (i25 = 0; i25 < 91; i25++) {
        K->data[i25 + K->size[0] * i24] = H->data[i24 + H->size[0] * i25];
      }
    }

    emxInit_real32_T1(&s_c, 2);
    H_idx_0 = (unsigned int)r_c->size[0];
    unnamed_idx_1 = (unsigned int)K->size[1];
    i24 = s_c->size[0] * s_c->size[1];
    s_c->size[0] = (int)H_idx_0;
    s_c->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)s_c, i24, (int)sizeof(float));
    b_m = r_c->size[0];
    i24 = s_c->size[0] * s_c->size[1];
    emxEnsureCapacity((emxArray__common *)s_c, i24, (int)sizeof(float));
    loop_ub = s_c->size[1];
    for (i24 = 0; i24 < loop_ub; i24++) {
      ar = s_c->size[0];
      for (i25 = 0; i25 < ar; i25++) {
        s_c->data[i25 + s_c->size[0] * i24] = 0.0F;
      }
    }

    if ((r_c->size[0] == 0) || (K->size[1] == 0)) {
    } else {
      f_c = r_c->size[0] * (K->size[1] - 1);
      idx = 0;
      while ((b_m > 0) && (idx <= f_c)) {
        i24 = idx + b_m;
        for (ic = idx; ic + 1 <= i24; ic++) {
          s_c->data[ic] = 0.0F;
        }

        idx += b_m;
      }

      br = 0;
      idx = 0;
      while ((b_m > 0) && (idx <= f_c)) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 91; ib++) {
          if (K->data[ib] != 0.0F) {
            ia = ar;
            i24 = idx + b_m;
            for (ic = idx; ic + 1 <= i24; ic++) {
              ia++;
              s_c->data[ic] += K->data[ib] * r_c->data[ia - 1];
            }
          }

          ar += b_m;
        }

        br += 91;
        idx += b_m;
      }
    }

    emxFree_real32_T(&r_c);
    i24 = S->size[0] * S->size[1];
    S->size[0] = s_c->size[0];
    S->size[1] = s_c->size[1];
    emxEnsureCapacity((emxArray__common *)S, i24, (int)sizeof(float));
    loop_ub = s_c->size[0] * s_c->size[1];
    for (i24 = 0; i24 < loop_ub; i24++) {
      S->data[i24] += s_c->data[i24];
    }

    emxFree_real32_T(&s_c);
    i24 = K->size[0] * K->size[1];
    K->size[0] = 91;
    K->size[1] = H->size[0];
    emxEnsureCapacity((emxArray__common *)K, i24, (int)sizeof(float));
    loop_ub = H->size[0];
    for (i24 = 0; i24 < loop_ub; i24++) {
      for (i25 = 0; i25 < 91; i25++) {
        K->data[i25 + K->size[0] * i24] = H->data[i24 + H->size[0] * i25];
      }
    }

    emxInit_real32_T1(&t_c, 2);
    unnamed_idx_1 = (unsigned int)K->size[1];
    i24 = t_c->size[0] * t_c->size[1];
    t_c->size[0] = 91;
    t_c->size[1] = (int)unnamed_idx_1;
    t_c->size[0] = 91;
    emxEnsureCapacity((emxArray__common *)t_c, i24, (int)sizeof(float));
    loop_ub = t_c->size[1];
    for (i24 = 0; i24 < loop_ub; i24++) {
      for (i25 = 0; i25 < 91; i25++) {
        t_c->data[i25 + t_c->size[0] * i24] = 0.0F;
      }
    }

    if (K->size[1] == 0) {
    } else {
      f_c = 91 * (K->size[1] - 1);
      for (idx = 0; idx <= f_c; idx += 91) {
        for (ic = idx + 1; ic <= idx + 91; ic++) {
          t_c->data[ic - 1] = 0.0F;
        }
      }

      br = 0;
      for (idx = 0; idx <= f_c; idx += 91) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 91; ib++) {
          if (K->data[ib] != 0.0F) {
            ia = ar;
            for (ic = idx; ic + 1 <= idx + 91; ic++) {
              ia++;
              t_c->data[ic] += K->data[ib] * b_P[ia - 1];
            }
          }

          ar += 91;
        }

        br += 91;
      }
    }

    if ((t_c->size[1] == 0) || ((S->size[0] == 0) || (S->size[1] == 0))) {
      unnamed_idx_1 = (unsigned int)S->size[0];
      i24 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = (int)unnamed_idx_1;
      emxEnsureCapacity((emxArray__common *)K, i24, (int)sizeof(float));
      loop_ub = 91 * (int)unnamed_idx_1;
      for (i24 = 0; i24 < loop_ub; i24++) {
        K->data[i24] = 0.0F;
      }
    } else if (S->size[0] == S->size[1]) {
      i24 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = t_c->size[1];
      emxEnsureCapacity((emxArray__common *)K, i24, (int)sizeof(float));
      loop_ub = t_c->size[0] * t_c->size[1];
      for (i24 = 0; i24 < loop_ub; i24++) {
        K->data[i24] = t_c->data[i24];
      }

      lusolve(S, K);
    } else {
      i24 = A->size[0] * A->size[1];
      A->size[0] = S->size[1];
      A->size[1] = S->size[0];
      emxEnsureCapacity((emxArray__common *)A, i24, (int)sizeof(float));
      loop_ub = S->size[0];
      for (i24 = 0; i24 < loop_ub; i24++) {
        ar = S->size[1];
        for (i25 = 0; i25 < ar; i25++) {
          A->data[i25 + A->size[0] * i24] = S->data[i24 + S->size[0] * i25];
        }
      }

      xgeqp3(A, tau, jpvt);
      ia = 0;
      if (A->size[0] < A->size[1]) {
        br = A->size[0];
        idx = A->size[1];
      } else {
        br = A->size[1];
        idx = A->size[0];
      }

      if (br > 0) {
        tol = (float)idx * (float)fabs((double)A->data[0]) * 1.1920929E-7F;
        while ((ia < br) && ((float)fabs((double)A->data[ia + A->size[0] * ia]) >=
                             tol)) {
          ia++;
        }
      }

      idx = A->size[1];
      i24 = Y->size[0] * Y->size[1];
      Y->size[0] = idx;
      Y->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)Y, i24, (int)sizeof(float));
      loop_ub = idx * 91;
      for (i24 = 0; i24 < loop_ub; i24++) {
        Y->data[i24] = 0.0F;
      }

      i24 = B->size[0] * B->size[1];
      B->size[0] = t_c->size[1];
      B->size[1] = 91;
      emxEnsureCapacity((emxArray__common *)B, i24, (int)sizeof(float));
      for (i24 = 0; i24 < 91; i24++) {
        loop_ub = t_c->size[1];
        for (i25 = 0; i25 < loop_ub; i25++) {
          B->data[i25 + B->size[0] * i24] = t_c->data[i24 + t_c->size[0] * i25];
        }
      }

      b_m = A->size[0];
      idx = A->size[0];
      br = A->size[1];
      if (idx <= br) {
        br = idx;
      }

      for (idx = 0; idx + 1 <= br; idx++) {
        if (tau->data[idx] != 0.0F) {
          for (k = 0; k < 91; k++) {
            tol = B->data[idx + B->size[0] * k];
            for (ib = idx + 1; ib + 1 <= b_m; ib++) {
              tol += A->data[ib + A->size[0] * idx] * B->data[ib + B->size[0] *
                k];
            }

            tol *= tau->data[idx];
            if (tol != 0.0F) {
              B->data[idx + B->size[0] * k] -= tol;
              for (ib = idx + 1; ib + 1 <= b_m; ib++) {
                B->data[ib + B->size[0] * k] -= A->data[ib + A->size[0] * idx] *
                  tol;
              }
            }
          }
        }
      }

      for (k = 0; k < 91; k++) {
        for (ib = 0; ib + 1 <= ia; ib++) {
          Y->data[(jpvt->data[ib] + Y->size[0] * k) - 1] = B->data[ib + B->size
            [0] * k];
        }

        for (idx = ia - 1; idx + 1 > 0; idx--) {
          Y->data[(jpvt->data[idx] + Y->size[0] * k) - 1] /= A->data[idx +
            A->size[0] * idx];
          for (ib = 0; ib + 1 <= idx; ib++) {
            Y->data[(jpvt->data[ib] + Y->size[0] * k) - 1] -= Y->data
              [(jpvt->data[idx] + Y->size[0] * k) - 1] * A->data[ib + A->size[0]
              * idx];
          }
        }
      }

      i24 = K->size[0] * K->size[1];
      K->size[0] = 91;
      K->size[1] = Y->size[0];
      emxEnsureCapacity((emxArray__common *)K, i24, (int)sizeof(float));
      loop_ub = Y->size[0];
      for (i24 = 0; i24 < loop_ub; i24++) {
        for (i25 = 0; i25 < 91; i25++) {
          K->data[i25 + K->size[0] * i24] = Y->data[i24 + Y->size[0] * i25];
        }
      }
    }

    emxFree_real32_T(&t_c);
    if ((K->size[1] == 1) || (r->size[0] == 1)) {
      for (i24 = 0; i24 < 91; i24++) {
        x_apo[i24] = 0.0F;
        loop_ub = K->size[1];
        for (i25 = 0; i25 < loop_ub; i25++) {
          b_x_apo = x_apo[i24] + K->data[i24 + K->size[0] * i25] * r->data[i25];
          x_apo[i24] = b_x_apo;
        }
      }
    } else {
      memset(&x_apo[0], 0, 91U * sizeof(float));
      ar = 0;
      for (ib = 0; ib + 1 <= K->size[1]; ib++) {
        if (r->data[ib] != 0.0F) {
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

    for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
      for (featureIdx = 0; featureIdx < 8; featureIdx++) {
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
            2) {
          b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
            inverse_depth += x_apo[(anchorIdx * 14 + featureIdx) + 27];
        }
      }
    }

    if ((K->size[1] == 1) || (H->size[0] == 1)) {
      for (i24 = 0; i24 < 91; i24++) {
        for (i25 = 0; i25 < 91; i25++) {
          i_c[i24 + 91 * i25] = 0.0F;
          loop_ub = K->size[1];
          for (idx = 0; idx < loop_ub; idx++) {
            i_c[i24 + 91 * i25] += K->data[i24 + K->size[0] * idx] * H->data[idx
              + H->size[0] * i25];
          }
        }
      }
    } else {
      k = K->size[1];
      memset(&i_c[0], 0, 8281U * sizeof(float));
      for (idx = 0; idx <= 8191; idx += 91) {
        for (ic = idx; ic + 1 <= idx + 91; ic++) {
          i_c[ic] = 0.0F;
        }
      }

      br = 0;
      for (idx = 0; idx <= 8191; idx += 91) {
        ar = 0;
        i24 = br + k;
        for (ib = br; ib + 1 <= i24; ib++) {
          if (H->data[ib] != 0.0F) {
            ia = ar;
            for (ic = idx; ic + 1 <= idx + 91; ic++) {
              ia++;
              i_c[ic] += H->data[ib] * K->data[ia - 1];
            }
          }

          ar += 91;
        }

        br += k;
      }
    }

    for (i24 = 0; i24 < 91; i24++) {
      for (i25 = 0; i25 < 91; i25++) {
        fv14[i25 + 91 * i24] = (float)iv14[i25 + 91 * i24] - i_c[i25 + 91 * i24];
      }
    }

    for (i24 = 0; i24 < 91; i24++) {
      for (i25 = 0; i25 < 91; i25++) {
        fv17[i24 + 91 * i25] = 0.0F;
        for (idx = 0; idx < 91; idx++) {
          fv17[i24 + 91 * i25] += fv14[i24 + 91 * idx] * b_P[idx + 91 * i25];
        }
      }
    }

    for (i24 = 0; i24 < 91; i24++) {
      memcpy(&b_P[i24 * 91], &fv17[i24 * 91], 91U * sizeof(float));
    }
  }

  emxFree_real32_T(&B);
  emxFree_int32_T(&jpvt);
  emxFree_real32_T(&tau);
  emxFree_real32_T(&A);
  emxFree_real32_T(&Y);
  emxFree_real32_T(&H);
  emxFree_real32_T(&r);
  emxFree_real32_T(&K);
  emxFree_real32_T(&S);

  // %
}

//
// % Iterative Camera Pose optimization (EKF)
// Arguments    : float P_apr[8281]
//                g_struct_T *b_xt
//                const float c_cameraParams_CameraParameters[2]
//                const float d_cameraParams_CameraParameters[2]
//                const float e_cameraParams_CameraParameters[3]
//                const float f_cameraParams_CameraParameters[2]
//                const float g_cameraParams_CameraParameters[2]
//                const float h_cameraParams_CameraParameters[3]
//                const float cameraParams_r_lr[3]
//                const float cameraParams_R_lr[9]
//                const float cameraParams_R_rl[9]
//                int updateVect[40]
//                float z_all_l[80]
//                float z_all_r[80]
//                float noiseParameters_image_noise
//                float c_noiseParameters_inv_depth_ini
//                const VIOParameters b_VIOParameters
//                float h_u_apo[160]
//                float b_map[120]
//                float b_delayedStatus[40]
// Return Type  : void
//
static void SLAM_upd(float P_apr[8281], g_struct_T *b_xt, const float
                     c_cameraParams_CameraParameters[2], const float
                     d_cameraParams_CameraParameters[2], const float
                     e_cameraParams_CameraParameters[3], const float
                     f_cameraParams_CameraParameters[2], const float
                     g_cameraParams_CameraParameters[2], const float
                     h_cameraParams_CameraParameters[3], const float
                     cameraParams_r_lr[3], const float cameraParams_R_lr[9],
                     const float cameraParams_R_rl[9], int updateVect[40], float
                     z_all_l[80], float z_all_r[80], float
                     noiseParameters_image_noise, float
                     c_noiseParameters_inv_depth_ini, const VIOParameters
                     b_VIOParameters, float h_u_apo[160], float b_map[120],
                     float b_delayedStatus[40])
{
  boolean_T x[40];
  int i;
  int idx;
  int ii_data[40];
  int ixstart;
  boolean_T exitg11;
  boolean_T guard2 = false;
  int loop_ub;
  emxArray_real32_T *ii;
  int i23;
  int ind_l2_size[1];
  float ind_l2_data[80];
  float status_ind_data[80];
  float fx;
  float fy;
  float Cx;
  float Cy;
  float k1;
  float k2;
  float k3;
  float c;
  float h_cin[2];
  float r_d_sq;
  float r_u_sq;
  boolean_T exitg10;
  float mtmp;
  float b_c;
  float diff;
  float coeff;
  boolean_T exitg9;
  boolean_T guard1 = false;
  signed char i_data[40];
  emxArray_real32_T *qualities;
  boolean_T exitg8;
  int anchorIdx;
  int featureIdx;
  boolean_T fix_new_feature;
  float uncertainties[8];
  signed char active_feature[8];
  int iidx[8];
  emxArray_real32_T *anchorInd;
  emxArray_real32_T *featureInd;
  emxArray_int32_T *b_iidx;
  float new_m_data[240];
  int c_triangulation_success_size_id;
  boolean_T triangulation_success_data[40];
  float b_h_cin[2];
  boolean_T success;
  float b_m[6];
  float fp[3];
  float new_origin_pos_rel[3];
  boolean_T bv0[3];
  float h_u_r[2];
  float a[2];
  float b_a[2];
  boolean_T b_guard1 = false;
  int ix;
  float triangulated_depths_data[40];
  float triangulated_status_ind_data[40];
  float b_triangulated_status_ind_data[40];
  int c_triangulated_status_ind_size_;
  int trueCount;
  float untriangulated_depths_data[40];
  emxArray_real32_T *auto_gen_tmp_4;
  emxArray_real32_T *out;
  int tmp_data[40];
  float untriangulated_status_ind_data[40];
  float b_new_m_data[120];
  float c_new_m_data[120];
  int new_feature_idx;
  boolean_T exitg6;
  static float J[8281];
  static const signed char iv11[8281] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
  };

  static const signed char iv12[294] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static float b_J[8281];
  boolean_T exitg7;
  float b_uncertainties[40];
  boolean_T has_active_features;
  int uncertainties_size[1];
  float totalNumActiveFeatues;
  float numDelayedFeatures;
  float delayedIdx;
  float numActivatedFeatures;
  boolean_T request_new_features;
  boolean_T exitg5;
  float c_uncertainties[5];
  float b_has_active_features[5];
  float b_P_apr[36];
  boolean_T exitg4;
  char cv28[63];
  static const char cv29[63] = { 'P', 'i', 'c', 'k', 'e', 'd', ' ', 'a', 'n',
    ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', 'w', 'i', 't', 'h', ' ', 'n', 'o',
    ' ', 'a', 'c', 't', 'i', 'v', 'e', ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e',
    's', ' ', 'a', 's', ' ', 'o', 'r', 'i', 'g', 'i', 'n', ' ', '(', 'a', 'n',
    'c', 'h', 'o', 'r', ' ', '%', 'd', ')', '\x00' };

  float new_origin_att_rel[9];
  static const signed char iv13[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1,
    0, 0, 0, 1 };

  float c_xt[3];
  float d_xt[9];
  float c_c[9];
  float varargin_1[4];
  boolean_T exitg3;
  float e_xt[9];
  boolean_T exitg2;
  float f_xt[9];
  float g_xt[9];
  boolean_T exitg1;
  float radsq;

  //  undistort all valid points
  for (i = 0; i < 40; i++) {
    x[i] = (updateVect[i] != 0);
  }

  idx = 0;
  ixstart = 1;
  exitg11 = false;
  while ((!exitg11) && (ixstart < 41)) {
    guard2 = false;
    if (x[ixstart - 1]) {
      idx++;
      ii_data[idx - 1] = ixstart;
      if (idx >= 40) {
        exitg11 = true;
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

  emxInit_real32_T(&ii, 1);
  i23 = ii->size[0];
  ii->size[0] = loop_ub;
  emxEnsureCapacity((emxArray__common *)ii, i23, (int)sizeof(float));
  for (i23 = 0; i23 < loop_ub; i23++) {
    ii->data[i23] = (float)ii_data[i23];
  }

  multiplyIdx(ii->data, ii->size, ind_l2_data, ind_l2_size);

  // UNDISTORTPOINT Undistort a point (or points) that are from a camera with
  // the calibration cameraparams
  //    Undistort a point or set of points from one camera. Depending on the
  //    camera model used to calibrate the camera, the appropriate undistortion
  //    is applied
  // % Plumb Bob
  loop_ub = ind_l2_size[0];
  emxFree_real32_T(&ii);
  for (i23 = 0; i23 < loop_ub; i23++) {
    status_ind_data[i23] = z_all_l[(int)ind_l2_data[i23] - 1];
  }

  fx = c_cameraParams_CameraParameters[0];
  fy = c_cameraParams_CameraParameters[1];
  Cx = d_cameraParams_CameraParameters[0];
  Cy = d_cameraParams_CameraParameters[1];
  k1 = e_cameraParams_CameraParameters[0];
  k2 = e_cameraParams_CameraParameters[1];
  k3 = e_cameraParams_CameraParameters[2];
  c = (float)ind_l2_size[0] / 2.0F;
  for (i = 0; i < (int)c; i++) {
    h_cin[0] = (z_all_l[(int)ind_l2_data[i << 1] - 1] - Cx) / fx;
    h_cin[1] = (z_all_l[(int)ind_l2_data[(i << 1) + 1] - 1] - Cy) / fy;
    r_d_sq = h_cin[0] * h_cin[0] + h_cin[1] * h_cin[1];

    // get_r_u Get undistorted radius from distorted radius
    //    Get the pixel radius of the undistorted pixels from a distorted pixel
    //    radius and distortion parameters
    r_u_sq = r_d_sq;
    idx = 0;
    exitg10 = false;
    while ((!exitg10) && (idx < 100)) {
      mtmp = ((1.0F + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq)) + k3 * rt_powf_snf
        (r_u_sq, 3.0F);
      b_c = (1.0F + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq) * k3 * rt_powf_snf
        (r_u_sq, 3.0F);
      diff = (r_u_sq * (mtmp * mtmp) - r_d_sq) / (b_c * b_c + 2.0F * r_u_sq *
        (((1.0F + 2.0F * k1 * r_u_sq) + 2.0F * k2 * (r_u_sq * r_u_sq)) + 2.0F *
         k3 * rt_powf_snf(r_u_sq, 3.0F)));
      r_u_sq -= diff;
      if ((diff < 1.0E-6F) && (diff > -1.0E-6F)) {
        exitg10 = true;
      } else {
        idx++;
      }
    }

    coeff = ((1.0F + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq)) + k3 * rt_powf_snf
      (r_u_sq, 3.0F);
    for (i23 = 0; i23 < 2; i23++) {
      h_cin[i23] /= coeff;
    }

    idx = (i << 1) - 1;
    status_ind_data[idx + 1] = h_cin[0] * fx + Cx;
    status_ind_data[idx + 2] = h_cin[1] * fy + Cy;
  }

  loop_ub = ind_l2_size[0];
  for (i23 = 0; i23 < loop_ub; i23++) {
    z_all_l[(int)ind_l2_data[i23] - 1] = status_ind_data[i23];
  }

  for (i = 0; i < 40; i++) {
    x[i] = (updateVect[i] == 2);
  }

  idx = 0;
  ixstart = 1;
  exitg9 = false;
  while ((!exitg9) && (ixstart < 41)) {
    guard1 = false;
    if (x[ixstart - 1]) {
      idx++;
      ii_data[idx - 1] = ixstart;
      if (idx >= 40) {
        exitg9 = true;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      ixstart++;
    }
  }

  if (1 > idx) {
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }

  if (1 > idx) {
    ixstart = 0;
  } else {
    ixstart = idx;
  }

  for (i23 = 0; i23 < loop_ub; i23++) {
    i_data[i23] = (signed char)ii_data[i23];
  }

  emxInit_real32_T(&qualities, 1);
  i23 = qualities->size[0];
  qualities->size[0] = ixstart;
  emxEnsureCapacity((emxArray__common *)qualities, i23, (int)sizeof(float));
  for (i23 = 0; i23 < ixstart; i23++) {
    qualities->data[i23] = i_data[i23];
  }

  multiplyIdx(qualities->data, qualities->size, ind_l2_data, ind_l2_size);

  // UNDISTORTPOINT Undistort a point (or points) that are from a camera with
  // the calibration cameraparams
  //    Undistort a point or set of points from one camera. Depending on the
  //    camera model used to calibrate the camera, the appropriate undistortion
  //    is applied
  // % Plumb Bob
  ixstart = ind_l2_size[0];
  for (i23 = 0; i23 < ixstart; i23++) {
    status_ind_data[i23] = z_all_r[(int)ind_l2_data[i23] - 1];
  }

  fx = f_cameraParams_CameraParameters[0];
  fy = f_cameraParams_CameraParameters[1];
  Cx = g_cameraParams_CameraParameters[0];
  Cy = g_cameraParams_CameraParameters[1];
  k1 = h_cameraParams_CameraParameters[0];
  k2 = h_cameraParams_CameraParameters[1];
  k3 = h_cameraParams_CameraParameters[2];
  c = (float)ind_l2_size[0] / 2.0F;
  for (i = 0; i < (int)c; i++) {
    h_cin[0] = (z_all_r[(int)ind_l2_data[i << 1] - 1] - Cx) / fx;
    h_cin[1] = (z_all_r[(int)ind_l2_data[(i << 1) + 1] - 1] - Cy) / fy;
    r_d_sq = h_cin[0] * h_cin[0] + h_cin[1] * h_cin[1];

    // get_r_u Get undistorted radius from distorted radius
    //    Get the pixel radius of the undistorted pixels from a distorted pixel
    //    radius and distortion parameters
    r_u_sq = r_d_sq;
    idx = 0;
    exitg8 = false;
    while ((!exitg8) && (idx < 100)) {
      mtmp = ((1.0F + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq)) + k3 * rt_powf_snf
        (r_u_sq, 3.0F);
      b_c = (1.0F + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq) * k3 * rt_powf_snf
        (r_u_sq, 3.0F);
      diff = (r_u_sq * (mtmp * mtmp) - r_d_sq) / (b_c * b_c + 2.0F * r_u_sq *
        (((1.0F + 2.0F * k1 * r_u_sq) + 2.0F * k2 * (r_u_sq * r_u_sq)) + 2.0F *
         k3 * rt_powf_snf(r_u_sq, 3.0F)));
      r_u_sq -= diff;
      if ((diff < 1.0E-6F) && (diff > -1.0E-6F)) {
        exitg8 = true;
      } else {
        idx++;
      }
    }

    coeff = ((1.0F + k1 * r_u_sq) + k2 * (r_u_sq * r_u_sq)) + k3 * rt_powf_snf
      (r_u_sq, 3.0F);
    for (i23 = 0; i23 < 2; i23++) {
      h_cin[i23] /= coeff;
    }

    idx = (i << 1) - 1;
    status_ind_data[idx + 1] = h_cin[0] * fx + Cx;
    status_ind_data[idx + 2] = h_cin[1] * fy + Cy;
  }

  ixstart = ind_l2_size[0];
  for (i23 = 0; i23 < ixstart; i23++) {
    z_all_r[(int)ind_l2_data[i23] - 1] = status_ind_data[i23];
  }

  //  check for lost features
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      if ((b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status != 0)
          && (updateVect[b_xt->anchor_states[anchorIdx]
              .feature_states[featureIdx].status_idx - 1] != 1)) {
        ixstart = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
          P_idx;
        for (i23 = 0; i23 < 91; i23++) {
          P_apr[(ixstart + 91 * i23) - 1] = 0.0F;
        }

        ixstart = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
          P_idx;
        for (i23 = 0; i23 < 91; i23++) {
          P_apr[i23 + 91 * (ixstart - 1)] = 0.0F;
        }

        b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status = 0;
        b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status_idx = 0;

        //                  ros_info('Lost feature %i, which was %i on anchor %i', idx, featureIdx, anchorIdx) 
      }
    }
  }

  if (b_VIOParameters.fixed_feature) {
    fix_new_feature = false;
    if (b_xt->origin.anchor_idx != 0) {
      if (b_xt->fixed_feature != 0) {
        if (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].feature_states
            [b_xt->fixed_feature - 1].status != 1) {
          fix_new_feature = true;
          ros_info(b_xt->anchor_states[b_xt->origin.anchor_idx - 1].
                   feature_states[b_xt->fixed_feature - 1].status_idx,
                   b_xt->fixed_feature, b_xt->origin.anchor_idx);
        }
      } else {
        fix_new_feature = true;
      }
    }

    if (fix_new_feature) {
      for (featureIdx = 0; featureIdx < 8; featureIdx++) {
        active_feature[featureIdx] = 0;
        if (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
            .feature_states[featureIdx].status == 1) {
          uncertainties[featureIdx] = P_apr[(b_xt->anchor_states
            [b_xt->origin.anchor_idx - 1].feature_states[featureIdx].P_idx + 91 *
            (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
             .feature_states[featureIdx].P_idx - 1)) - 1];
          active_feature[featureIdx] = 1;
        } else {
          uncertainties[featureIdx] = 1000.0F;

          //  dont fix an inactive feature
        }
      }

      sort(uncertainties, iidx);
      for (i = 0; i < 8; i++) {
        uncertainties[i] = (float)iidx[i];
      }

      if (!(active_feature[(int)uncertainties[0] - 1] != 0)) {
        ros_error();
      }

      mtmp = rt_roundf_snf(uncertainties[0]);
      if (mtmp < 2.14748365E+9F) {
        i23 = (int)mtmp;
      } else {
        i23 = MAX_int32_T;
      }

      b_xt->fixed_feature = i23;
      ixstart = b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
        .feature_states[b_xt->fixed_feature - 1].P_idx;
      for (i23 = 0; i23 < 91; i23++) {
        P_apr[(ixstart + 91 * i23) - 1] = 0.0F;
      }

      //  fix the feature depth
      ixstart = b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
        .feature_states[b_xt->fixed_feature - 1].P_idx;
      for (i23 = 0; i23 < 91; i23++) {
        P_apr[i23 + 91 * (ixstart - 1)] = 0.0F;
      }

      b_ros_info(b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
                 .feature_states[b_xt->fixed_feature - 1].status_idx,
                 b_xt->fixed_feature, b_xt->origin.anchor_idx);
    }
  }

  // % do the update
  for (i = 0; i < 40; i++) {
    x[i] = (updateVect[i] == 1);
  }

  if (any(x)) {
    OnePointRANSAC_EKF(b_xt, P_apr, z_all_l, c_cameraParams_CameraParameters,
                       d_cameraParams_CameraParameters,
                       noiseParameters_image_noise,
                       b_VIOParameters.max_ekf_iterations,
                       b_VIOParameters.delayed_initialization,
                       b_VIOParameters.RANSAC, updateVect);
  }

  // % Initialize new anchors/features
  emxInit_real32_T(&anchorInd, 1);
  emxInit_real32_T(&featureInd, 1);
  emxInit_int32_T(&b_iidx, 1);
  if (loop_ub >= 4) {
    //  try to triangulate all new features
    for (i23 = 0; i23 < loop_ub; i23++) {
      ind_l2_data[i23] = 0.0F;
    }

    loop_ub = 3 * qualities->size[0];
    for (i23 = 0; i23 < loop_ub; i23++) {
      new_m_data[i23] = 0.0F;
    }

    c_triangulation_success_size_id = qualities->size[0];
    loop_ub = qualities->size[0];
    for (i23 = 0; i23 < loop_ub; i23++) {
      triangulation_success_data[i23] = false;
    }

    for (i = 0; i < qualities->size[0]; i++) {
      c = (qualities->data[i] - 1.0F) * 2.0F;
      for (i23 = 0; i23 < 2; i23++) {
        h_cin[i23] = z_all_l[(int)(c + (1.0F + (float)i23)) - 1];
      }

      mtmp = (qualities->data[i] - 1.0F) * 2.0F;
      for (i23 = 0; i23 < 2; i23++) {
        b_h_cin[i23] = z_all_r[(int)(mtmp + (1.0F + (float)i23)) - 1];
      }

      if (!b_VIOParameters.mono) {
        initializePoint(h_cin, b_h_cin, c_cameraParams_CameraParameters,
                        d_cameraParams_CameraParameters,
                        f_cameraParams_CameraParameters,
                        g_cameraParams_CameraParameters, cameraParams_r_lr,
                        cameraParams_R_lr, fp, b_m, &success);
        for (i23 = 0; i23 < 3; i23++) {
          new_origin_pos_rel[i23] = b_m[i23];
        }

        if (success) {
          //  perform further checks
          for (idx = 0; idx < 3; idx++) {
            bv0[idx] = rtIsNaNF(fp[idx]);
          }

          if (b_any(bv0)) {
            for (i23 = 0; i23 < 3; i23++) {
              fp[i23] = b_m[i23];
            }

            success = false;
          } else {
            //  check reprojection error
            predictMeasurementStereo(fp, c_cameraParams_CameraParameters,
              d_cameraParams_CameraParameters, f_cameraParams_CameraParameters,
              g_cameraParams_CameraParameters, cameraParams_r_lr,
              cameraParams_R_rl, a, h_u_r);
            for (idx = 0; idx < 2; idx++) {
              b_a[idx] = a[idx] - h_cin[idx];
            }

            b_guard1 = false;
            if (c_norm(b_a) > 2.0F) {
              b_guard1 = true;
            } else {
              for (idx = 0; idx < 2; idx++) {
                a[idx] = h_u_r[idx] - b_h_cin[idx];
              }

              if (c_norm(a) > 2.0F) {
                b_guard1 = true;
              } else {
                if (norm(fp) < 0.1F) {
                  //  feature triangulated very close
                  for (i23 = 0; i23 < 3; i23++) {
                    fp[i23] = b_m[i23];
                  }

                  success = false;
                }
              }
            }

            if (b_guard1) {
              //                          ros_info('Bad triangulation (reprojection error) for point %d', int8(ind_r(i))); 
              for (i23 = 0; i23 < 3; i23++) {
                fp[i23] = b_m[i23];
              }

              success = false;
            }
          }
        } else {
          for (i23 = 0; i23 < 3; i23++) {
            fp[i23] = b_m[i23];
          }
        }
      } else {
        //  mono
        new_origin_pos_rel[0] = (z_all_l[(int)(c + 1.0F) - 1] -
          d_cameraParams_CameraParameters[0]) / c_cameraParams_CameraParameters
          [0];
        new_origin_pos_rel[1] = (z_all_l[(int)(c + 2.0F) - 1] -
          d_cameraParams_CameraParameters[1]) / c_cameraParams_CameraParameters
          [1];
        new_origin_pos_rel[2] = 1.0F;
        mtmp = norm(new_origin_pos_rel);
        for (idx = 0; idx < 3; idx++) {
          fp[idx] = new_origin_pos_rel[idx] / mtmp;
        }

        success = true;
      }

      ind_l2_data[i] = norm(fp);
      for (i23 = 0; i23 < 3; i23++) {
        new_m_data[i23 + 3 * i] = new_origin_pos_rel[i23];
      }

      triangulation_success_data[i] = success;
    }

    idx = 0;
    for (ixstart = 0; ixstart < c_triangulation_success_size_id; ixstart++) {
      if (triangulation_success_data[ixstart]) {
        idx++;
      }
    }

    d_ros_info(idx, c_triangulation_success_size_id);
    idx = c_triangulation_success_size_id - 1;
    ix = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        ix++;
      }
    }

    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        triangulated_depths_data[ixstart] = ind_l2_data[i];
        ixstart++;
      }
    }

    i23 = anchorInd->size[0];
    anchorInd->size[0] = ix;
    emxEnsureCapacity((emxArray__common *)anchorInd, i23, (int)sizeof(float));
    for (i23 = 0; i23 < ix; i23++) {
      anchorInd->data[i23] = triangulated_depths_data[i23];
    }

    c_sort(anchorInd, b_iidx);
    i23 = featureInd->size[0];
    featureInd->size[0] = b_iidx->size[0];
    emxEnsureCapacity((emxArray__common *)featureInd, i23, (int)sizeof(float));
    loop_ub = b_iidx->size[0];
    for (i23 = 0; i23 < loop_ub; i23++) {
      featureInd->data[i23] = (float)b_iidx->data[i23];
    }

    idx = c_triangulation_success_size_id - 1;
    ix = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        ix++;
      }
    }

    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        ii_data[ixstart] = i + 1;
        ixstart++;
      }
    }

    idx = c_triangulation_success_size_id - 1;
    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (triangulation_success_data[i]) {
        triangulated_status_ind_data[ixstart] = qualities->data[i];
        ixstart++;
      }
    }

    c_triangulated_status_ind_size_ = featureInd->size[0];
    loop_ub = featureInd->size[0];
    for (i23 = 0; i23 < loop_ub; i23++) {
      b_triangulated_status_ind_data[i23] = triangulated_status_ind_data[(int)
        featureInd->data[i23] - 1];
    }

    for (i23 = 0; i23 < c_triangulated_status_ind_size_; i23++) {
      triangulated_status_ind_data[i23] = b_triangulated_status_ind_data[i23];
    }

    idx = c_triangulation_success_size_id - 1;
    trueCount = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        trueCount++;
      }
    }

    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        untriangulated_depths_data[ixstart] = ind_l2_data[i];
        ixstart++;
      }
    }

    emxInit_real32_T(&auto_gen_tmp_4, 1);
    i23 = auto_gen_tmp_4->size[0];
    auto_gen_tmp_4->size[0] = trueCount;
    emxEnsureCapacity((emxArray__common *)auto_gen_tmp_4, i23, (int)sizeof(float));
    for (i23 = 0; i23 < trueCount; i23++) {
      auto_gen_tmp_4->data[i23] = untriangulated_depths_data[i23];
    }

    emxInit_real32_T(&out, 1);
    c_sort(auto_gen_tmp_4, b_iidx);
    i23 = out->size[0];
    out->size[0] = b_iidx->size[0];
    emxEnsureCapacity((emxArray__common *)out, i23, (int)sizeof(float));
    loop_ub = b_iidx->size[0];
    for (i23 = 0; i23 < loop_ub; i23++) {
      out->data[i23] = (float)b_iidx->data[i23];
    }

    idx = c_triangulation_success_size_id - 1;
    trueCount = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        trueCount++;
      }
    }

    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        tmp_data[ixstart] = i + 1;
        ixstart++;
      }
    }

    idx = c_triangulation_success_size_id - 1;
    ixstart = 0;
    for (i = 0; i <= idx; i++) {
      if (!triangulation_success_data[i]) {
        untriangulated_status_ind_data[ixstart] = qualities->data[i];
        ixstart++;
      }
    }

    idx = out->size[0];
    loop_ub = out->size[0];
    for (i23 = 0; i23 < loop_ub; i23++) {
      b_triangulated_status_ind_data[i23] = untriangulated_status_ind_data[(int)
        out->data[i23] - 1];
    }

    for (i23 = 0; i23 < idx; i23++) {
      untriangulated_status_ind_data[i23] = b_triangulated_status_ind_data[i23];
    }

    ind_l2_size[0] = anchorInd->size[0] + auto_gen_tmp_4->size[0];
    loop_ub = anchorInd->size[0];
    for (i23 = 0; i23 < loop_ub; i23++) {
      ind_l2_data[i23] = anchorInd->data[i23];
    }

    loop_ub = auto_gen_tmp_4->size[0];
    for (i23 = 0; i23 < loop_ub; i23++) {
      ind_l2_data[i23 + anchorInd->size[0]] = auto_gen_tmp_4->data[i23];
    }

    emxFree_real32_T(&auto_gen_tmp_4);
    for (i23 = 0; i23 < ix; i23++) {
      for (i = 0; i < 3; i++) {
        b_new_m_data[i + 3 * i23] = new_m_data[i + 3 * (ii_data[i23] - 1)];
      }
    }

    for (i23 = 0; i23 < trueCount; i23++) {
      for (i = 0; i < 3; i++) {
        c_new_m_data[i + 3 * i23] = new_m_data[i + 3 * (tmp_data[i23] - 1)];
      }
    }

    loop_ub = featureInd->size[0];
    for (i23 = 0; i23 < loop_ub; i23++) {
      for (i = 0; i < 3; i++) {
        new_m_data[i + 3 * i23] = b_new_m_data[i + 3 * ((int)featureInd->
          data[i23] - 1)];
      }
    }

    loop_ub = out->size[0];
    for (i23 = 0; i23 < loop_ub; i23++) {
      for (i = 0; i < 3; i++) {
        new_m_data[i + 3 * (i23 + featureInd->size[0])] = c_new_m_data[i + 3 *
          ((int)out->data[i23] - 1)];
      }
    }

    emxFree_real32_T(&out);
    for (i23 = 0; i23 < c_triangulated_status_ind_size_; i23++) {
      status_ind_data[i23] = triangulated_status_ind_data[i23];
    }

    for (i23 = 0; i23 < idx; i23++) {
      status_ind_data[i23 + c_triangulated_status_ind_size_] =
        untriangulated_status_ind_data[i23];
    }

    //  insert new features into the state. first the closest triangulated ones, 
    //  then the un-triangulated ones
    new_feature_idx = 0;
    anchorIdx = 0;
    exitg6 = false;
    while ((!exitg6) && (anchorIdx + 1 < 6)) {
      //          if new_feature_idx > length(new_depths)
      idx = 0;
      for (ixstart = 0; ixstart < c_triangulation_success_size_id; ixstart++) {
        if (triangulation_success_data[ixstart]) {
          idx++;
        }
      }

      if (((float)(new_feature_idx + 1) > idx) || ((float)ind_l2_size[0] -
           ((float)(new_feature_idx + 1) - 1.0F) < 4.0F)) {
        exitg6 = true;
      } else {
        if (getNumValidFeatures(b_xt->anchor_states[anchorIdx].feature_states) <
            4.0F) {
          //  anchor needs to be initialized
          //  free up updateVect
          for (featureIdx = 0; featureIdx < 8; featureIdx++) {
            if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status
                != 0) {
              //                      ros_info('clearing up feature %i (%i on %i)', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx, featureIdx, anchorIdx) 
              updateVect[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = 0;
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                status_idx = 0;
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                0;
            }
          }

          if (b_xt->origin.anchor_idx == anchorIdx + 1) {
            b_xt->origin.anchor_idx = 0;
            e_ros_info(anchorIdx + 1);
          }

          for (i23 = 0; i23 < 3; i23++) {
            b_xt->anchor_states[anchorIdx].pos[i23] = b_xt->robot_state.pos[i23];
          }

          for (i23 = 0; i23 < 4; i23++) {
            b_xt->anchor_states[anchorIdx].att[i23] = b_xt->robot_state.att[i23];
          }

          idx = anchorIdx * 14 + 21;
          for (i23 = 0; i23 < 6; i23++) {
            b_xt->anchor_states[anchorIdx].P_idx[i23] = (i23 + idx) + 1;
          }

          for (i23 = 0; i23 < 91; i23++) {
            for (i = 0; i < 6; i++) {
              P_apr[(b_xt->anchor_states[anchorIdx].P_idx[i] + 91 * i23) - 1] =
                0.0F;
            }
          }

          for (i23 = 0; i23 < 6; i23++) {
            for (i = 0; i < 91; i++) {
              P_apr[i + 91 * (b_xt->anchor_states[anchorIdx].P_idx[i23] - 1)] =
                0.0F;
            }
          }

          ixstart = b_xt->anchor_states[anchorIdx].P_idx[5];
          for (i23 = 0; i23 < 91; i23++) {
            for (i = 0; i < 8; i++) {
              idx = 1 + i;
              ix = ixstart + idx;
              if ((ixstart > 0) && (ix <= 0)) {
                ix = MAX_int32_T;
              }

              P_apr[(ix + 91 * i23) - 1] = 0.0F;
            }
          }

          ixstart = b_xt->anchor_states[anchorIdx].P_idx[5];
          for (i23 = 0; i23 < 8; i23++) {
            for (i = 0; i < 91; i++) {
              idx = 1 + i23;
              ix = ixstart + idx;
              if ((ixstart > 0) && (ix <= 0)) {
                ix = MAX_int32_T;
              }

              P_apr[i + 91 * (ix - 1)] = 0.0F;
            }
          }

          for (i23 = 0; i23 < 8281; i23++) {
            J[i23] = iv11[i23];
          }

          idx = anchorIdx * 14 + 21;
          for (i23 = 0; i23 < 91; i23++) {
            for (i = 0; i < 14; i++) {
              J[(i + idx) + 91 * i23] = 0.0F;
            }
          }

          idx = anchorIdx * 14 + 21;
          for (i23 = 0; i23 < 21; i23++) {
            for (i = 0; i < 14; i++) {
              J[(i + idx) + 91 * i23] = iv12[i + 14 * i23];
            }
          }

          for (i23 = 0; i23 < 91; i23++) {
            for (i = 0; i < 91; i++) {
              b_J[i23 + 91 * i] = 0.0F;
              for (idx = 0; idx < 91; idx++) {
                b_J[i23 + 91 * i] += J[i23 + 91 * idx] * P_apr[idx + 91 * i];
              }
            }
          }

          for (i23 = 0; i23 < 91; i23++) {
            for (i = 0; i < 91; i++) {
              P_apr[i23 + 91 * i] = 0.0F;
              for (idx = 0; idx < 91; idx++) {
                P_apr[i23 + 91 * i] += b_J[i23 + 91 * idx] * J[i + 91 * idx];
              }
            }
          }

          featureIdx = 0;
          exitg7 = false;
          while ((!exitg7) && (featureIdx + 1 < 9)) {
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              inverse_depth = 1.0F / ind_l2_data[new_feature_idx];
            for (i23 = 0; i23 < 3; i23++) {
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].m[i23] =
                new_m_data[i23 + 3 * new_feature_idx];
            }

            if (b_VIOParameters.delayed_initialization) {
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                2;
            } else {
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                1;
            }

            mtmp = rt_roundf_snf(status_ind_data[new_feature_idx]);
            if (mtmp < 2.14748365E+9F) {
              if (mtmp >= -2.14748365E+9F) {
                i23 = (int)mtmp;
              } else {
                i23 = MIN_int32_T;
              }
            } else if (mtmp >= 2.14748365E+9F) {
              i23 = MAX_int32_T;
            } else {
              i23 = 0;
            }

            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status_idx
              = i23;
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx =
              (anchorIdx * 14 + featureIdx) + 28;
            idx = 0;
            for (ixstart = 0; ixstart < c_triangulation_success_size_id; ixstart
                 ++) {
              if (triangulation_success_data[ixstart]) {
                idx++;
              }
            }

            if ((float)(new_feature_idx + 1) > idx) {
              f_ros_info(b_xt->anchor_states[anchorIdx]
                         .feature_states[featureIdx].status_idx);
              P_apr[(b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                     P_idx + 91 * (b_xt->anchor_states[anchorIdx]
                                   .feature_states[featureIdx].P_idx - 1)) - 1] =
                c_noiseParameters_inv_depth_ini * 10.0F;

              //  TODO: Maybe push the mean value further away?
            } else {
              P_apr[(b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                     P_idx + 91 * (b_xt->anchor_states[anchorIdx]
                                   .feature_states[featureIdx].P_idx - 1)) - 1] =
                c_noiseParameters_inv_depth_ini;

              // *new_depths(new_feature_idx);
            }

            mtmp = rt_roundf_snf(status_ind_data[new_feature_idx]);
            if (mtmp < 2.14748365E+9F) {
              if (mtmp >= -2.14748365E+9F) {
                i23 = (int)mtmp;
              } else {
                i23 = MIN_int32_T;
              }
            } else if (mtmp >= 2.14748365E+9F) {
              i23 = MAX_int32_T;
            } else {
              i23 = 0;
            }

            g_ros_info(i23, featureIdx + 1, anchorIdx + 1);
            updateVect[(int)status_ind_data[new_feature_idx] - 1] = 1;
            new_feature_idx++;

            //                  if new_feature_idx > length(new_depths)
            idx = 0;
            for (ixstart = 0; ixstart < c_triangulation_success_size_id; ixstart
                 ++) {
              if (triangulation_success_data[ixstart]) {
                idx++;
              }
            }

            if ((float)(new_feature_idx + 1) > idx) {
              exitg7 = true;
            } else {
              featureIdx++;
            }
          }
        }

        anchorIdx++;
      }
    }

    for (i = 0; i < 40; i++) {
      if (updateVect[i] == 2) {
        updateVect[i] = 0;
      }
    }

    //  remove features that were not inserted
  }

  if (b_VIOParameters.delayed_initialization) {
    //  get the median uncertainty of the active features as a benchmark on
    //  the delayed features
    for (i = 0; i < 40; i++) {
      b_uncertainties[i] = -1.0F;
    }

    has_active_features = false;
    for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
      for (featureIdx = 0; featureIdx < 8; featureIdx++) {
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
            1) {
          has_active_features = true;
          b_uncertainties[b_xt->anchor_states[anchorIdx]
            .feature_states[featureIdx].status_idx - 1] = P_apr
            [(b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx +
              91 * (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                    P_idx - 1)) - 1];
        }
      }
    }

    if (has_active_features) {
      ix = 0;
      for (i = 0; i < 40; i++) {
        if (b_uncertainties[i] > 0.0F) {
          ix++;
        }
      }

      ixstart = 0;
      for (i = 0; i < 40; i++) {
        if (b_uncertainties[i] > 0.0F) {
          ii_data[ixstart] = i + 1;
          ixstart++;
        }
      }

      uncertainties_size[0] = ix;
      for (i23 = 0; i23 < ix; i23++) {
        b_triangulated_status_ind_data[i23] = b_uncertainties[ii_data[i23] - 1];
      }

      mtmp = median(b_triangulated_status_ind_data, uncertainties_size);

      //  because coder does not support nanflag
      //  check if a delayed initialization feature has converged
      for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          if ((b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
               2) && (P_apr[(b_xt->anchor_states[anchorIdx]
                             .feature_states[featureIdx].P_idx + 91 *
                             (b_xt->anchor_states[anchorIdx]
                              .feature_states[featureIdx].P_idx - 1)) - 1] <
                      mtmp * 2.0F)) {
            //  this feature is not active yet
            if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0F) {
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                0;
              updateVect[b_xt->anchor_states[anchorIdx]
                .feature_states[featureIdx].status_idx - 1] = 0;
            } else {
              //                              ros_info('Feature %i (%i on anchor %i) has converged', xt.anchor_states(anchorIdx).feature_states(featureIdx).status_idx, featureIdx, anchorIdx); 
              b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status =
                1;
            }
          }
        }
      }
    }

    //  check if delayed features need to be forced active due to too few active 
    //  features
    totalNumActiveFeatues = getTotalNumActiveFeatures(b_xt->anchor_states);
    if (totalNumActiveFeatues < 16.0F) {
      //  find the best features and activate them
      numDelayedFeatures = getTotalNumDelayedFeatures(b_xt->anchor_states);
      i23 = qualities->size[0];
      qualities->size[0] = (int)numDelayedFeatures;
      emxEnsureCapacity((emxArray__common *)qualities, i23, (int)sizeof(float));
      loop_ub = (int)numDelayedFeatures;
      for (i23 = 0; i23 < loop_ub; i23++) {
        qualities->data[i23] = 0.0F;
      }

      //  quality measures of each delayed feature
      i23 = anchorInd->size[0];
      anchorInd->size[0] = qualities->size[0];
      emxEnsureCapacity((emxArray__common *)anchorInd, i23, (int)sizeof(float));
      loop_ub = qualities->size[0];
      for (i23 = 0; i23 < loop_ub; i23++) {
        anchorInd->data[i23] = qualities->data[i23];
      }

      i23 = featureInd->size[0];
      featureInd->size[0] = qualities->size[0];
      emxEnsureCapacity((emxArray__common *)featureInd, i23, (int)sizeof(float));
      loop_ub = qualities->size[0];
      for (i23 = 0; i23 < loop_ub; i23++) {
        featureInd->data[i23] = qualities->data[i23];
      }

      delayedIdx = 1.0F;
      for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
        for (featureIdx = 0; featureIdx < 8; featureIdx++) {
          if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
              2) {
            qualities->data[(int)delayedIdx - 1] = P_apr[(b_xt->
              anchor_states[anchorIdx].feature_states[featureIdx].P_idx + 91 *
              (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].P_idx -
               1)) - 1] / c_noiseParameters_inv_depth_ini;
            anchorInd->data[(int)delayedIdx - 1] = (float)anchorIdx + 1.0F;
            featureInd->data[(int)delayedIdx - 1] = (float)featureIdx + 1.0F;
            delayedIdx++;
          }
        }
      }

      c_sort(qualities, b_iidx);
      i23 = qualities->size[0];
      qualities->size[0] = b_iidx->size[0];
      emxEnsureCapacity((emxArray__common *)qualities, i23, (int)sizeof(float));
      loop_ub = b_iidx->size[0];
      for (i23 = 0; i23 < loop_ub; i23++) {
        qualities->data[i23] = (float)b_iidx->data[i23];
      }

      numActivatedFeatures = 0.0F;
      i = 0;
      while ((i <= (int)(float)qualities->size[0] - 1) &&
             (!(numActivatedFeatures > (float)ceil((double)(16.0F -
                 totalNumActiveFeatues)))) && (!(numActivatedFeatures >
               numDelayedFeatures))) {
        if (b_xt->anchor_states[(int)anchorInd->data[(int)qualities->data[(int)
            (1.0F + (float)i) - 1] - 1] - 1].feature_states[(int)
            featureInd->data[(int)qualities->data[(int)(1.0F + (float)i) - 1] -
            1] - 1].inverse_depth < 0.0F) {
          b_xt->anchor_states[(int)anchorInd->data[(int)qualities->data[(int)
            (1.0F + (float)i) - 1] - 1] - 1].feature_states[(int)
            featureInd->data[(int)qualities->data[(int)(1.0F + (float)i) - 1] -
            1] - 1].status = 0;
          updateVect[b_xt->anchor_states[(int)anchorInd->data[(int)
            qualities->data[(int)(1.0F + (float)i) - 1] - 1] - 1]
            .feature_states[(int)featureInd->data[(int)qualities->data[(int)
            (1.0F + (float)i) - 1] - 1] - 1].status_idx - 1] = 0;
        } else {
          b_xt->anchor_states[(int)anchorInd->data[(int)qualities->data[(int)
            (1.0F + (float)i) - 1] - 1] - 1].feature_states[(int)
            featureInd->data[(int)qualities->data[(int)(1.0F + (float)i) - 1] -
            1] - 1].status = 1;
          h_ros_info(b_xt->anchor_states[(int)anchorInd->data[(int)
                     qualities->data[(int)(1.0F + (float)i) - 1] - 1] - 1].
                     feature_states[(int)featureInd->data[(int)qualities->data
                     [(int)(1.0F + (float)i) - 1] - 1] - 1].status_idx,
                     featureInd->data[(int)qualities->data[(int)(1.0F + (float)i)
                     - 1] - 1], anchorInd->data[(int)qualities->data[(int)(1.0F
                      + (float)i) - 1] - 1]);
          numActivatedFeatures++;
        }

        i++;
      }
    }
  }

  emxFree_int32_T(&b_iidx);
  emxFree_real32_T(&featureInd);
  emxFree_real32_T(&anchorInd);
  emxFree_real32_T(&qualities);

  //  check if new features need to be requested
  request_new_features = false;
  idx = 0;
  for (ixstart = 0; ixstart < 40; ixstart++) {
    if (updateVect[ixstart] == 0) {
      idx++;
    }
  }

  if (idx > 4.0F) {
    //  if a new anchor can be filled enough
    anchorIdx = 1;
    exitg5 = false;
    while ((!exitg5) && (anchorIdx < 6)) {
      if (getNumValidFeatures(b_xt->anchor_states[anchorIdx - 1].feature_states)
          < 4.0F) {
        request_new_features = true;
        exitg5 = true;
      } else {
        anchorIdx++;
      }
    }
  } else {
    //  debug check
  }

  if (request_new_features) {
    for (i = 0; i < 40; i++) {
      if (updateVect[i] == 0) {
        updateVect[i] = 2;
      }
    }

    //  get as many new features as possible
  }

  // % robocentric update
  if (b_xt->origin.anchor_idx == 0) {
    //  need to update the origin anchor and the state
    b_xt->fixed_feature = 0;

    //  ensure that a new feature will be fixed, if this option is enabled
    //  choose the best anchor as the new origin anchor
    //  uncertainties of the anchors reduced to a scalar
    for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
      b_has_active_features[anchorIdx] = 0.0F;
      if (anyActiveAnchorFeatures(b_xt->anchor_states[anchorIdx].feature_states))
      {
        for (i23 = 0; i23 < 6; i23++) {
          for (i = 0; i < 6; i++) {
            b_P_apr[i + 6 * i23] = P_apr[(b_xt->anchor_states[anchorIdx].P_idx[i]
              + 91 * (b_xt->anchor_states[anchorIdx].P_idx[i23] - 1)) - 1];
          }
        }

        c_uncertainties[anchorIdx] = det(b_P_apr);
        b_has_active_features[anchorIdx] = 1.0F;
      } else {
        c_uncertainties[anchorIdx] = 1000.0F;

        //  dont fix an anchor with no active features
      }
    }

    if (!c_any(b_has_active_features)) {
      //  can happen if outlier rejection rejected all features
    } else {
      ixstart = 1;
      mtmp = c_uncertainties[0];
      idx = 1;
      if (rtIsNaNF(c_uncertainties[0])) {
        ix = 2;
        exitg4 = false;
        while ((!exitg4) && (ix < 6)) {
          ixstart = ix;
          if (!rtIsNaNF(c_uncertainties[ix - 1])) {
            mtmp = c_uncertainties[ix - 1];
            idx = ix;
            exitg4 = true;
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

      b_xt->origin.anchor_idx = idx;
      if (!(b_has_active_features[b_xt->origin.anchor_idx - 1] != 0.0F)) {
        //  debug check
        // #coder
        // ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
        for (i23 = 0; i23 < 63; i23++) {
          cv28[i23] = cv29[i23];
        }

        ROS_ERROR(cv28, b_xt->origin.anchor_idx);
      } else {
        i_ros_info(b_xt->origin.anchor_idx);
        for (i = 0; i < 3; i++) {
          new_origin_pos_rel[i] = b_xt->anchor_states[b_xt->origin.anchor_idx -
            1].pos[i];
        }

        //  in old origin frame
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        new_origin_att_rel[0] = ((b_xt->anchor_states[b_xt->origin.anchor_idx -
          1].att[0] * b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[0] -
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[1] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[1]) -
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[2] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[2]) +
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[3] *
          b_xt->anchor_states[b_xt->origin.anchor_idx - 1].att[3];
        new_origin_att_rel[3] = 2.0F * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] + b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[6] = 2.0F * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] - b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[1] = 2.0F * (b_xt->anchor_states
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
        new_origin_att_rel[7] = 2.0F * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] + b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[2] = 2.0F * (b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[0] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[2] + b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[1] * b_xt->anchor_states
          [b_xt->origin.anchor_idx - 1].att[3]);
        new_origin_att_rel[5] = 2.0F * (b_xt->anchor_states
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
        for (i23 = 0; i23 < 8281; i23++) {
          J[i23] = iv11[i23];
        }

        //  robot position and orientation
        for (i23 = 0; i23 < 3; i23++) {
          for (i = 0; i < 3; i++) {
            J[i + 91 * i23] = new_origin_att_rel[i + 3 * i23];
            J[i + 91 * (i23 + 3)] = 0.0F;
          }
        }

        for (i23 = 0; i23 < 6; i23++) {
          for (i = 0; i < 3; i++) {
            J[(i + 91 * i23) + 3] = iv13[i + 3 * i23];
          }
        }

        for (i23 = 0; i23 < 3; i23++) {
          c_xt[i23] = b_xt->robot_state.pos[i23] - b_xt->anchor_states
            [b_xt->origin.anchor_idx - 1].pos[i23];
        }

        for (i23 = 0; i23 < 3; i23++) {
          fp[i23] = 0.0F;
          for (i = 0; i < 3; i++) {
            J[i + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
                        .P_idx[i23] - 1)] = -new_origin_att_rel[i + 3 * i23];
            J[(i + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
                         .P_idx[i23] - 1)) + 3] = 0.0F;
            fp[i23] += new_origin_att_rel[i23 + 3 * i] * c_xt[i];
          }
        }

        J[91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1)] =
          0.0F;
        J[91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1)] =
          -fp[2];
        J[91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1)] =
          fp[1];
        J[1 + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] -
                    1)] = fp[2];
        J[1 + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] -
                    1)] = 0.0F;
        J[1 + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] -
                    1)] = -fp[0];
        J[2 + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] -
                    1)] = -fp[1];
        J[2 + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] -
                    1)] = fp[0];
        J[2 + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] -
                    1)] = 0.0F;

        //  robot velocity
        //  velocity
        //  origin rotation
        for (i23 = 0; i23 < 3; i23++) {
          for (i = 0; i < 3; i++) {
            J[(i + 91 * (b_xt->anchor_states[b_xt->origin.anchor_idx - 1]
                         .P_idx[i23 + 3] - 1)) + 3] = -new_origin_att_rel[i23 +
              3 * i];
            J[(i + 91 * (6 + i23)) + 6] = new_origin_att_rel[i + 3 * i23];
            J[(i + 91 * (15 + i23)) + 15] = new_origin_att_rel[i + 3 * i23];
          }
        }

        //  origin rotation
        for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
          if (anchorIdx + 1 == b_xt->origin.anchor_idx) {
            //  remove yaw uncertainty, but not pitch or roll
            for (i23 = 0; i23 < 6; i23++) {
              for (i = 0; i < 6; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i] + 91 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i23] - 1)) - 1] = 0.0F;
              }
            }

            //  TODO: allow roll/pitch uncertainty
          } else {
            for (i23 = 0; i23 < 3; i23++) {
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i] + 91 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i23] - 1)) - 1] =
                  new_origin_att_rel[i + 3 * i23];
              }
            }

            for (i23 = 0; i23 < 3; i23++) {
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i] + 91 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i23 + 3] - 1)) - 1] =
                  0.0F;
              }
            }

            for (i23 = 0; i23 < 6; i23++) {
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i + 3] + 91 *
                   (b_xt->anchor_states[anchorIdx].P_idx[i23] - 1)) - 1] =
                  iv13[i + 3 * i23];
              }
            }

            for (i23 = 0; i23 < 3; i23++) {
              c_xt[i23] = b_xt->anchor_states[anchorIdx].pos[i23] -
                new_origin_pos_rel[i23];
            }

            for (i23 = 0; i23 < 3; i23++) {
              fp[i23] = 0.0F;
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i] + 91 *
                   (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[i23]
                    - 1)) - 1] = -new_origin_att_rel[i + 3 * i23];
                fp[i23] += new_origin_att_rel[i23 + 3 * i] * c_xt[i];
              }
            }

            J[(b_xt->anchor_states[anchorIdx].P_idx[0] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1))
              - 1] = 0.0F;
            J[(b_xt->anchor_states[anchorIdx].P_idx[0] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1))
              - 1] = -fp[2];
            J[(b_xt->anchor_states[anchorIdx].P_idx[0] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1))
              - 1] = fp[1];
            J[(b_xt->anchor_states[anchorIdx].P_idx[1] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1))
              - 1] = fp[2];
            J[(b_xt->anchor_states[anchorIdx].P_idx[1] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1))
              - 1] = 0.0F;
            J[(b_xt->anchor_states[anchorIdx].P_idx[1] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1))
              - 1] = -fp[0];
            J[(b_xt->anchor_states[anchorIdx].P_idx[2] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[3] - 1))
              - 1] = -fp[1];
            J[(b_xt->anchor_states[anchorIdx].P_idx[2] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[4] - 1))
              - 1] = fp[0];
            J[(b_xt->anchor_states[anchorIdx].P_idx[2] + 91 *
               (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[5] - 1))
              - 1] = 0.0F;
            for (i23 = 0; i23 < 3; i23++) {
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i + 3] + 91 *
                   (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[i23]
                    - 1)) - 1] = 0.0F;
              }
            }

            for (i23 = 0; i23 < 3; i23++) {
              for (i = 0; i < 3; i++) {
                J[(b_xt->anchor_states[anchorIdx].P_idx[i + 3] + 91 *
                   (b_xt->anchor_states[b_xt->origin.anchor_idx - 1].P_idx[i23 +
                    3] - 1)) - 1] = -new_origin_att_rel[i23 + 3 * i];
              }
            }
          }

          for (i = 0; i < 3; i++) {
            fp[i] = b_xt->anchor_states[anchorIdx].pos[i] - new_origin_pos_rel[i];
            b_xt->anchor_states[anchorIdx].pos[i] = 0.0F;
          }

          //  if ~all(size(q) == [4, 1])
          //      error('q does not have the size of a quaternion')
          //  end
          //  if abs(norm(q) - 1) > 1e-3
          //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
          //  end
          d_xt[0] = ((b_xt->anchor_states[anchorIdx].att[0] *
                      b_xt->anchor_states[anchorIdx].att[0] -
                      b_xt->anchor_states[anchorIdx].att[1] *
                      b_xt->anchor_states[anchorIdx].att[1]) -
                     b_xt->anchor_states[anchorIdx].att[2] * b_xt->
                     anchor_states[anchorIdx].att[2]) + b_xt->
            anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx]
            .att[3];
          d_xt[3] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
                            b_xt->anchor_states[anchorIdx].att[1] +
                            b_xt->anchor_states[anchorIdx].att[2] *
                            b_xt->anchor_states[anchorIdx].att[3]);
          d_xt[6] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
                            b_xt->anchor_states[anchorIdx].att[2] -
                            b_xt->anchor_states[anchorIdx].att[1] *
                            b_xt->anchor_states[anchorIdx].att[3]);
          d_xt[1] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
                            b_xt->anchor_states[anchorIdx].att[1] -
                            b_xt->anchor_states[anchorIdx].att[2] *
                            b_xt->anchor_states[anchorIdx].att[3]);
          d_xt[4] = ((-(b_xt->anchor_states[anchorIdx].att[0] *
                        b_xt->anchor_states[anchorIdx].att[0]) +
                      b_xt->anchor_states[anchorIdx].att[1] *
                      b_xt->anchor_states[anchorIdx].att[1]) -
                     b_xt->anchor_states[anchorIdx].att[2] * b_xt->
                     anchor_states[anchorIdx].att[2]) + b_xt->
            anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx]
            .att[3];
          d_xt[7] = 2.0F * (b_xt->anchor_states[anchorIdx].att[1] *
                            b_xt->anchor_states[anchorIdx].att[2] +
                            b_xt->anchor_states[anchorIdx].att[0] *
                            b_xt->anchor_states[anchorIdx].att[3]);
          d_xt[2] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
                            b_xt->anchor_states[anchorIdx].att[2] +
                            b_xt->anchor_states[anchorIdx].att[1] *
                            b_xt->anchor_states[anchorIdx].att[3]);
          d_xt[5] = 2.0F * (b_xt->anchor_states[anchorIdx].att[1] *
                            b_xt->anchor_states[anchorIdx].att[2] -
                            b_xt->anchor_states[anchorIdx].att[0] *
                            b_xt->anchor_states[anchorIdx].att[3]);
          d_xt[8] = ((-(b_xt->anchor_states[anchorIdx].att[0] *
                        b_xt->anchor_states[anchorIdx].att[0]) -
                      b_xt->anchor_states[anchorIdx].att[1] *
                      b_xt->anchor_states[anchorIdx].att[1]) +
                     b_xt->anchor_states[anchorIdx].att[2] * b_xt->
                     anchor_states[anchorIdx].att[2]) + b_xt->
            anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx]
            .att[3];
          for (i23 = 0; i23 < 3; i23++) {
            b_xt->anchor_states[anchorIdx].pos[i23] = 0.0F;
            for (i = 0; i < 3; i++) {
              b_xt->anchor_states[anchorIdx].pos[i23] += new_origin_att_rel[i23
                + 3 * i] * fp[i];
              c_c[i23 + 3 * i] = 0.0F;
              for (idx = 0; idx < 3; idx++) {
                c_c[i23 + 3 * i] += d_xt[i23 + 3 * idx] * new_origin_att_rel[i +
                  3 * idx];
              }
            }
          }

          //  THIS IS OK, It is according to the NASA memo found
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
          // % speed optimization
          varargin_1[0] = (1.0F + c_c[0]) - (c_c[4] + c_c[8]);
          varargin_1[1] = (1.0F + c_c[4]) - (c_c[0] + c_c[8]);
          varargin_1[2] = (1.0F + c_c[8]) - (c_c[0] + c_c[4]);
          varargin_1[3] = 1.0F + ((c_c[0] + c_c[4]) + c_c[8]);
          ixstart = 1;
          mtmp = varargin_1[0];
          idx = 1;
          if (rtIsNaNF(varargin_1[0])) {
            ix = 2;
            exitg3 = false;
            while ((!exitg3) && (ix < 5)) {
              ixstart = ix;
              if (!rtIsNaNF(varargin_1[ix - 1])) {
                mtmp = varargin_1[ix - 1];
                idx = ix;
                exitg3 = true;
              } else {
                ix++;
              }
            }
          }

          if (ixstart < 4) {
            while (ixstart + 1 < 5) {
              if (varargin_1[ixstart] > mtmp) {
                mtmp = varargin_1[ixstart];
                idx = ixstart + 1;
              }

              ixstart++;
            }
          }

          if (idx == 1) {
            b_xt->anchor_states[anchorIdx].att[0] = (float)sqrt((double)((1.0F +
              2.0F * c_c[0]) - ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F;
            b_xt->anchor_states[anchorIdx].att[1] = (c_c[3] + c_c[1]) / (4.0F *
              ((float)sqrt((double)((1.0F + 2.0F * c_c[0]) - ((c_c[0] + c_c[4])
              + c_c[8]))) / 2.0F));
            b_xt->anchor_states[anchorIdx].att[2] = (c_c[6] + c_c[2]) / (4.0F *
              ((float)sqrt((double)((1.0F + 2.0F * c_c[0]) - ((c_c[0] + c_c[4])
              + c_c[8]))) / 2.0F));
            b_xt->anchor_states[anchorIdx].att[3] = (c_c[7] - c_c[5]) / (4.0F *
              ((float)sqrt((double)((1.0F + 2.0F * c_c[0]) - ((c_c[0] + c_c[4])
              + c_c[8]))) / 2.0F));
          } else if (idx == 2) {
            b_xt->anchor_states[anchorIdx].att[0] = (c_c[3] + c_c[1]) / (4.0F *
              ((float)sqrt((double)((1.0F + 2.0F * c_c[4]) - ((c_c[0] + c_c[4])
              + c_c[8]))) / 2.0F));
            b_xt->anchor_states[anchorIdx].att[1] = (float)sqrt((double)((1.0F +
              2.0F * c_c[4]) - ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F;
            b_xt->anchor_states[anchorIdx].att[2] = (c_c[7] + c_c[5]) / (4.0F *
              ((float)sqrt((double)((1.0F + 2.0F * c_c[4]) - ((c_c[0] + c_c[4])
              + c_c[8]))) / 2.0F));
            b_xt->anchor_states[anchorIdx].att[3] = (c_c[2] - c_c[6]) / (4.0F *
              ((float)sqrt((double)((1.0F + 2.0F * c_c[4]) - ((c_c[0] + c_c[4])
              + c_c[8]))) / 2.0F));
          } else if (idx == 3) {
            b_xt->anchor_states[anchorIdx].att[0] = (c_c[6] + c_c[2]) / (4.0F *
              ((float)sqrt((double)((1.0F + 2.0F * c_c[8]) - ((c_c[0] + c_c[4])
              + c_c[8]))) / 2.0F));
            b_xt->anchor_states[anchorIdx].att[1] = (c_c[7] + c_c[5]) / (4.0F *
              ((float)sqrt((double)((1.0F + 2.0F * c_c[8]) - ((c_c[0] + c_c[4])
              + c_c[8]))) / 2.0F));
            b_xt->anchor_states[anchorIdx].att[2] = (float)sqrt((double)((1.0F +
              2.0F * c_c[8]) - ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F;
            b_xt->anchor_states[anchorIdx].att[3] = (c_c[3] - c_c[1]) / (4.0F *
              ((float)sqrt((double)((1.0F + 2.0F * c_c[8]) - ((c_c[0] + c_c[4])
              + c_c[8]))) / 2.0F));
          } else {
            b_xt->anchor_states[anchorIdx].att[0] = (c_c[7] - c_c[5]) / (4.0F *
              ((float)sqrt((double)(1.0F + ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F));
            b_xt->anchor_states[anchorIdx].att[1] = (c_c[2] - c_c[6]) / (4.0F *
              ((float)sqrt((double)(1.0F + ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F));
            b_xt->anchor_states[anchorIdx].att[2] = (c_c[3] - c_c[1]) / (4.0F *
              ((float)sqrt((double)(1.0F + ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F));
            b_xt->anchor_states[anchorIdx].att[3] = (float)sqrt((double)(1.0F +
              ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F;
          }
        }

        for (i23 = 0; i23 < 91; i23++) {
          for (i = 0; i < 91; i++) {
            b_J[i23 + 91 * i] = 0.0F;
            for (idx = 0; idx < 91; idx++) {
              b_J[i23 + 91 * i] += J[i23 + 91 * idx] * P_apr[idx + 91 * i];
            }
          }
        }

        for (i23 = 0; i23 < 91; i23++) {
          for (i = 0; i < 91; i++) {
            P_apr[i23 + 91 * i] = 0.0F;
            for (idx = 0; idx < 91; idx++) {
              P_apr[i23 + 91 * i] += b_J[i23 + 91 * idx] * J[i + 91 * idx];
            }
          }
        }

        for (i23 = 0; i23 < 3; i23++) {
          c_xt[i23] = b_xt->robot_state.pos[i23] - new_origin_pos_rel[i23];
        }

        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        e_xt[0] = ((b_xt->robot_state.att[0] * b_xt->robot_state.att[0] -
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
          b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
        e_xt[3] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] +
                          b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
        e_xt[6] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] -
                          b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
        e_xt[1] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] -
                          b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
        e_xt[4] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) +
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
          b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
        e_xt[7] = 2.0F * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] +
                          b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
        e_xt[2] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] +
                          b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
        e_xt[5] = 2.0F * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] -
                          b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
        e_xt[8] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) -
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) +
                   b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
          b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
        for (i23 = 0; i23 < 3; i23++) {
          b_xt->robot_state.pos[i23] = 0.0F;
          for (i = 0; i < 3; i++) {
            b_xt->robot_state.pos[i23] += new_origin_att_rel[i23 + 3 * i] *
              c_xt[i];
            c_c[i23 + 3 * i] = 0.0F;
            for (idx = 0; idx < 3; idx++) {
              c_c[i23 + 3 * i] += e_xt[i23 + 3 * idx] * new_origin_att_rel[i + 3
                * idx];
            }
          }
        }

        //  THIS IS OK, It is according to the NASA memo found
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
        // % speed optimization
        varargin_1[0] = (1.0F + c_c[0]) - (c_c[4] + c_c[8]);
        varargin_1[1] = (1.0F + c_c[4]) - (c_c[0] + c_c[8]);
        varargin_1[2] = (1.0F + c_c[8]) - (c_c[0] + c_c[4]);
        varargin_1[3] = 1.0F + ((c_c[0] + c_c[4]) + c_c[8]);
        ixstart = 1;
        mtmp = varargin_1[0];
        idx = 1;
        if (rtIsNaNF(varargin_1[0])) {
          ix = 2;
          exitg2 = false;
          while ((!exitg2) && (ix < 5)) {
            ixstart = ix;
            if (!rtIsNaNF(varargin_1[ix - 1])) {
              mtmp = varargin_1[ix - 1];
              idx = ix;
              exitg2 = true;
            } else {
              ix++;
            }
          }
        }

        if (ixstart < 4) {
          while (ixstart + 1 < 5) {
            if (varargin_1[ixstart] > mtmp) {
              mtmp = varargin_1[ixstart];
              idx = ixstart + 1;
            }

            ixstart++;
          }
        }

        if (idx == 1) {
          b_xt->robot_state.att[0] = (float)sqrt((double)((1.0F + 2.0F * c_c[0])
            - ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F;
          b_xt->robot_state.att[1] = (c_c[3] + c_c[1]) / (4.0F * ((float)sqrt
            ((double)((1.0F + 2.0F * c_c[0]) - ((c_c[0] + c_c[4]) + c_c[8]))) /
            2.0F));
          b_xt->robot_state.att[2] = (c_c[6] + c_c[2]) / (4.0F * ((float)sqrt
            ((double)((1.0F + 2.0F * c_c[0]) - ((c_c[0] + c_c[4]) + c_c[8]))) /
            2.0F));
          b_xt->robot_state.att[3] = (c_c[7] - c_c[5]) / (4.0F * ((float)sqrt
            ((double)((1.0F + 2.0F * c_c[0]) - ((c_c[0] + c_c[4]) + c_c[8]))) /
            2.0F));
        } else if (idx == 2) {
          b_xt->robot_state.att[0] = (c_c[3] + c_c[1]) / (4.0F * ((float)sqrt
            ((double)((1.0F + 2.0F * c_c[4]) - ((c_c[0] + c_c[4]) + c_c[8]))) /
            2.0F));
          b_xt->robot_state.att[1] = (float)sqrt((double)((1.0F + 2.0F * c_c[4])
            - ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F;
          b_xt->robot_state.att[2] = (c_c[7] + c_c[5]) / (4.0F * ((float)sqrt
            ((double)((1.0F + 2.0F * c_c[4]) - ((c_c[0] + c_c[4]) + c_c[8]))) /
            2.0F));
          b_xt->robot_state.att[3] = (c_c[2] - c_c[6]) / (4.0F * ((float)sqrt
            ((double)((1.0F + 2.0F * c_c[4]) - ((c_c[0] + c_c[4]) + c_c[8]))) /
            2.0F));
        } else if (idx == 3) {
          b_xt->robot_state.att[0] = (c_c[6] + c_c[2]) / (4.0F * ((float)sqrt
            ((double)((1.0F + 2.0F * c_c[8]) - ((c_c[0] + c_c[4]) + c_c[8]))) /
            2.0F));
          b_xt->robot_state.att[1] = (c_c[7] + c_c[5]) / (4.0F * ((float)sqrt
            ((double)((1.0F + 2.0F * c_c[8]) - ((c_c[0] + c_c[4]) + c_c[8]))) /
            2.0F));
          b_xt->robot_state.att[2] = (float)sqrt((double)((1.0F + 2.0F * c_c[8])
            - ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F;
          b_xt->robot_state.att[3] = (c_c[3] - c_c[1]) / (4.0F * ((float)sqrt
            ((double)((1.0F + 2.0F * c_c[8]) - ((c_c[0] + c_c[4]) + c_c[8]))) /
            2.0F));
        } else {
          b_xt->robot_state.att[0] = (c_c[7] - c_c[5]) / (4.0F * ((float)sqrt
            ((double)(1.0F + ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F));
          b_xt->robot_state.att[1] = (c_c[2] - c_c[6]) / (4.0F * ((float)sqrt
            ((double)(1.0F + ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F));
          b_xt->robot_state.att[2] = (c_c[3] - c_c[1]) / (4.0F * ((float)sqrt
            ((double)(1.0F + ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F));
          b_xt->robot_state.att[3] = (float)sqrt((double)(1.0F + ((c_c[0] + c_c
            [4]) + c_c[8]))) / 2.0F;
        }

        for (i23 = 0; i23 < 3; i23++) {
          c_xt[i23] = 0.0F;
          for (i = 0; i < 3; i++) {
            c_xt[i23] += new_origin_att_rel[i23 + 3 * i] * b_xt->
              robot_state.vel[i];
          }
        }

        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        f_xt[0] = ((b_xt->origin.att[0] * b_xt->origin.att[0] - b_xt->
                    origin.att[1] * b_xt->origin.att[1]) - b_xt->origin.att[2] *
                   b_xt->origin.att[2]) + b_xt->origin.att[3] * b_xt->
          origin.att[3];
        f_xt[1] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[1] +
                          b_xt->origin.att[2] * b_xt->origin.att[3]);
        f_xt[2] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[2] -
                          b_xt->origin.att[1] * b_xt->origin.att[3]);
        f_xt[3] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[1] -
                          b_xt->origin.att[2] * b_xt->origin.att[3]);
        f_xt[4] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) +
                    b_xt->origin.att[1] * b_xt->origin.att[1]) -
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];
        f_xt[5] = 2.0F * (b_xt->origin.att[1] * b_xt->origin.att[2] +
                          b_xt->origin.att[0] * b_xt->origin.att[3]);
        f_xt[6] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[2] +
                          b_xt->origin.att[1] * b_xt->origin.att[3]);
        f_xt[7] = 2.0F * (b_xt->origin.att[1] * b_xt->origin.att[2] -
                          b_xt->origin.att[0] * b_xt->origin.att[3]);
        f_xt[8] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) -
                    b_xt->origin.att[1] * b_xt->origin.att[1]) +
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];

        //  in world frame
        //  if ~all(size(q) == [4, 1])
        //      error('q does not have the size of a quaternion')
        //  end
        //  if abs(norm(q) - 1) > 1e-3
        //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
        //  end
        g_xt[0] = ((b_xt->origin.att[0] * b_xt->origin.att[0] - b_xt->
                    origin.att[1] * b_xt->origin.att[1]) - b_xt->origin.att[2] *
                   b_xt->origin.att[2]) + b_xt->origin.att[3] * b_xt->
          origin.att[3];
        g_xt[3] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[1] +
                          b_xt->origin.att[2] * b_xt->origin.att[3]);
        g_xt[6] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[2] -
                          b_xt->origin.att[1] * b_xt->origin.att[3]);
        g_xt[1] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[1] -
                          b_xt->origin.att[2] * b_xt->origin.att[3]);
        g_xt[4] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) +
                    b_xt->origin.att[1] * b_xt->origin.att[1]) -
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];
        g_xt[7] = 2.0F * (b_xt->origin.att[1] * b_xt->origin.att[2] +
                          b_xt->origin.att[0] * b_xt->origin.att[3]);
        g_xt[2] = 2.0F * (b_xt->origin.att[0] * b_xt->origin.att[2] +
                          b_xt->origin.att[1] * b_xt->origin.att[3]);
        g_xt[5] = 2.0F * (b_xt->origin.att[1] * b_xt->origin.att[2] -
                          b_xt->origin.att[0] * b_xt->origin.att[3]);
        g_xt[8] = ((-(b_xt->origin.att[0] * b_xt->origin.att[0]) -
                    b_xt->origin.att[1] * b_xt->origin.att[1]) +
                   b_xt->origin.att[2] * b_xt->origin.att[2]) + b_xt->
          origin.att[3] * b_xt->origin.att[3];
        for (i23 = 0; i23 < 3; i23++) {
          b_xt->robot_state.vel[i23] = c_xt[i23];
          mtmp = 0.0F;
          for (i = 0; i < 3; i++) {
            mtmp += f_xt[i23 + 3 * i] * new_origin_pos_rel[i];
            c_c[i23 + 3 * i] = 0.0F;
            for (idx = 0; idx < 3; idx++) {
              c_c[i23 + 3 * i] += new_origin_att_rel[i23 + 3 * idx] * g_xt[idx +
                3 * i];
            }
          }

          b_xt->origin.pos[i23] += mtmp;
        }

        //  THIS IS OK, It is according to the NASA memo found
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
        // % speed optimization
        varargin_1[0] = (1.0F + c_c[0]) - (c_c[4] + c_c[8]);
        varargin_1[1] = (1.0F + c_c[4]) - (c_c[0] + c_c[8]);
        varargin_1[2] = (1.0F + c_c[8]) - (c_c[0] + c_c[4]);
        varargin_1[3] = 1.0F + ((c_c[0] + c_c[4]) + c_c[8]);
        ixstart = 1;
        mtmp = varargin_1[0];
        idx = 1;
        if (rtIsNaNF(varargin_1[0])) {
          ix = 2;
          exitg1 = false;
          while ((!exitg1) && (ix < 5)) {
            ixstart = ix;
            if (!rtIsNaNF(varargin_1[ix - 1])) {
              mtmp = varargin_1[ix - 1];
              idx = ix;
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
              idx = ixstart + 1;
            }

            ixstart++;
          }
        }

        if (idx == 1) {
          b_xt->origin.att[0] = (float)sqrt((double)((1.0F + 2.0F * c_c[0]) -
            ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F;
          b_xt->origin.att[1] = (c_c[3] + c_c[1]) / (4.0F * ((float)sqrt((double)
            ((1.0F + 2.0F * c_c[0]) - ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F));
          b_xt->origin.att[2] = (c_c[6] + c_c[2]) / (4.0F * ((float)sqrt((double)
            ((1.0F + 2.0F * c_c[0]) - ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F));
          b_xt->origin.att[3] = (c_c[7] - c_c[5]) / (4.0F * ((float)sqrt((double)
            ((1.0F + 2.0F * c_c[0]) - ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F));
        } else if (idx == 2) {
          b_xt->origin.att[0] = (c_c[3] + c_c[1]) / (4.0F * ((float)sqrt((double)
            ((1.0F + 2.0F * c_c[4]) - ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F));
          b_xt->origin.att[1] = (float)sqrt((double)((1.0F + 2.0F * c_c[4]) -
            ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F;
          b_xt->origin.att[2] = (c_c[7] + c_c[5]) / (4.0F * ((float)sqrt((double)
            ((1.0F + 2.0F * c_c[4]) - ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F));
          b_xt->origin.att[3] = (c_c[2] - c_c[6]) / (4.0F * ((float)sqrt((double)
            ((1.0F + 2.0F * c_c[4]) - ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F));
        } else if (idx == 3) {
          b_xt->origin.att[0] = (c_c[6] + c_c[2]) / (4.0F * ((float)sqrt((double)
            ((1.0F + 2.0F * c_c[8]) - ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F));
          b_xt->origin.att[1] = (c_c[7] + c_c[5]) / (4.0F * ((float)sqrt((double)
            ((1.0F + 2.0F * c_c[8]) - ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F));
          b_xt->origin.att[2] = (float)sqrt((double)((1.0F + 2.0F * c_c[8]) -
            ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F;
          b_xt->origin.att[3] = (c_c[3] - c_c[1]) / (4.0F * ((float)sqrt((double)
            ((1.0F + 2.0F * c_c[8]) - ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F));
        } else {
          b_xt->origin.att[0] = (c_c[7] - c_c[5]) / (4.0F * ((float)sqrt((double)
            (1.0F + ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F));
          b_xt->origin.att[1] = (c_c[2] - c_c[6]) / (4.0F * ((float)sqrt((double)
            (1.0F + ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F));
          b_xt->origin.att[2] = (c_c[3] - c_c[1]) / (4.0F * ((float)sqrt((double)
            (1.0F + ((c_c[0] + c_c[4]) + c_c[8]))) / 2.0F));
          b_xt->origin.att[3] = (float)sqrt((double)(1.0F + ((c_c[0] + c_c[4]) +
            c_c[8]))) / 2.0F;
        }

        //  in world frame
      }
    }
  }

  // % aposteriori measurement prediction
  getMap(b_xt->origin.pos, b_xt->origin.att, b_xt->anchor_states, b_map);

  //  get map for output
  getScaledMap(b_xt);

  //  update the scaled map for measurement prediction
  memset(&h_u_apo[0], 0, 160U * sizeof(float));
  memset(&b_delayedStatus[0], 0, 40U * sizeof(float));
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status != 0)
      {
        // PREDICTMEASUREMENT Predict the measurement of a feature given in the left 
        // camera frame
        //    Get the pixel coordinates where a feature given in the left camera 
        //    frame would be visible in both cameras. Depending on the
        //    camera model used to calibrate the camera, the appropriate undistortion 
        //    is applied
        mtmp = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
          scaled_map_point[2];
        for (i23 = 0; i23 < 2; i23++) {
          h_cin[i23] = b_xt->anchor_states[anchorIdx].feature_states[featureIdx]
            .scaled_map_point[i23] / mtmp;
        }

        radsq = h_cin[0] * h_cin[0] + h_cin[1] * h_cin[1];
        c = ((1.0F + e_cameraParams_CameraParameters[0] * radsq) +
             e_cameraParams_CameraParameters[1] * (radsq * radsq)) +
          e_cameraParams_CameraParameters[2] * rt_powf_snf(radsq, 4.0F);
        for (i23 = 0; i23 < 2; i23++) {
          h_cin[i23] *= c;
        }

        for (i23 = 0; i23 < 3; i23++) {
          mtmp = 0.0F;
          for (i = 0; i < 3; i++) {
            mtmp += cameraParams_R_rl[i23 + 3 * i] * b_xt->
              anchor_states[anchorIdx].feature_states[featureIdx].
              scaled_map_point[i];
          }

          new_origin_pos_rel[i23] = mtmp - cameraParams_r_lr[i23];
        }

        for (i = 0; i < 2; i++) {
          b_h_cin[i] = new_origin_pos_rel[i] / new_origin_pos_rel[2];
        }

        radsq = b_h_cin[0] * b_h_cin[0] + b_h_cin[1] * b_h_cin[1];
        c = ((1.0F + h_cameraParams_CameraParameters[0] * radsq) +
             h_cameraParams_CameraParameters[1] * (radsq * radsq)) +
          h_cameraParams_CameraParameters[2] * rt_powf_snf(radsq, 4.0F);
        for (i23 = 0; i23 < 2; i23++) {
          b_h_cin[i23] *= c;
        }

        idx = b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
          status_idx;
        ix = idx - 1;
        if ((idx < 0) && (ix >= 0)) {
          ix = MIN_int32_T;
        }

        if (ix > 536870911) {
          idx = MAX_int32_T;
        } else if (ix <= -536870912) {
          idx = MIN_int32_T;
        } else {
          idx = ix << 2;
        }

        ix = idx + 1;
        if ((idx > 0) && (ix <= 0)) {
          ix = MAX_int32_T;
        }

        h_u_apo[ix - 1] = d_cameraParams_CameraParameters[0] +
          c_cameraParams_CameraParameters[0] * h_cin[0];
        ix = idx + 2;
        if ((idx > 0) && (ix <= 0)) {
          ix = MAX_int32_T;
        }

        h_u_apo[ix - 1] = d_cameraParams_CameraParameters[1] +
          c_cameraParams_CameraParameters[1] * h_cin[1];
        ix = idx + 3;
        if ((idx > 0) && (ix <= 0)) {
          ix = MAX_int32_T;
        }

        h_u_apo[ix - 1] = g_cameraParams_CameraParameters[0] +
          f_cameraParams_CameraParameters[0] * b_h_cin[0];
        ix = idx + 4;
        if ((idx > 0) && (ix <= 0)) {
          ix = MAX_int32_T;
        }

        h_u_apo[ix - 1] = g_cameraParams_CameraParameters[1] +
          f_cameraParams_CameraParameters[1] * b_h_cin[1];
        if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status ==
            2) {
          b_delayedStatus[b_xt->anchor_states[anchorIdx]
            .feature_states[featureIdx].status_idx - 1] = 1.0F;
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
  ret = false;
  featureIdx = 1;
  exitg1 = false;
  while ((!exitg1) && (featureIdx < 9)) {
    if (anchor_state_feature_states[featureIdx - 1].status == 1) {
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
//                emxArray_real32_T *x
//                int offset
//                int np
//                int nq
//                emxArray_int32_T *iwork
//                emxArray_real32_T *xwork
// Return Type  : void
//
static void b_merge(emxArray_int32_T *idx, emxArray_real32_T *x, int offset, int
                    np, int nq, emxArray_int32_T *iwork, emxArray_real32_T
                    *xwork)
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
// Arguments    : const float x[4]
// Return Type  : float
//
static float b_norm(const float x[4])
{
  float y;
  float scale;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  scale = 1.17549435E-38F;
  for (k = 0; k < 4; k++) {
    absxk = (float)fabs((double)x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * (float)sqrt((double)y);
}

//
// #coder
// ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
// Arguments    : void
// Return Type  : void
//
static void b_ros_error()
{
  char cv12[14];
  int i12;
  static const char cv13[14] = { 'i', 'n', 'c', 'o', 'n', 's', 'i', 's', 't',
    'e', 'n', 'c', 'y', '\x00' };

  for (i12 = 0; i12 < 14; i12++) {
    cv12[i12] = cv13[i12];
  }

  ROS_ERROR(cv12);
}

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                int varargin_2
//                int varargin_3
// Return Type  : void
//
static void b_ros_info(int varargin_1, int varargin_2, int varargin_3)
{
  char cv10[44];
  int i11;
  static const char cv11[44] = { 'F', 'i', 'x', 'i', 'n', 'g', ' ', 'f', 'e',
    'a', 't', 'u', 'r', 'e', ' ', '%', 'i', ' ', '(', 'f', 'e', 'a', 't', 'u',
    'r', 'e', ' ', '%', 'i', ' ', 'o', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r',
    ' ', '%', 'i', ')', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0F) {
    for (i11 = 0; i11 < 44; i11++) {
      cv10[i11] = cv11[i11];
    }

    ROS_INFO(cv10, varargin_1, varargin_2, varargin_3);
  }
}

//
// Arguments    : float x[8]
//                int idx[8]
// Return Type  : void
//
static void b_sort(float x[8], int idx[8])
{
  int b_m;
  float x4[4];
  signed char idx4[4];
  float xwork[8];
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
    x4[b_m] = 0.0F;
    idx4[b_m] = 0;
  }

  for (b_m = 0; b_m < 8; b_m++) {
    xwork[b_m] = 0.0F;
  }

  nNaNs = -7;
  ib = 0;
  for (k = 0; k < 8; k++) {
    if (rtIsNaNF(x[k])) {
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
// Arguments    : float A[30]
//                float tau[5]
//                int jpvt[5]
// Return Type  : void
//
static void b_xgeqp3(float A[30], float tau[5], int jpvt[5])
{
  float vn1[5];
  float vn2[5];
  int k;
  int iy;
  float work[5];
  float smax;
  float temp2;
  int itemp;
  float absxk;
  float t;
  int i;
  int i_i;
  int ix;
  int pvt;
  int i32;
  int i_ip1;
  int lastv;
  int lastc;
  boolean_T exitg2;
  int exitg1;
  k = 1;
  for (iy = 0; iy < 5; iy++) {
    jpvt[iy] = 1 + iy;
    work[iy] = 0.0F;
    smax = 0.0F;
    temp2 = 1.17549435E-38F;
    for (itemp = k; itemp <= k + 5; itemp++) {
      absxk = (float)fabs((double)A[itemp - 1]);
      if (absxk > temp2) {
        t = temp2 / absxk;
        smax = 1.0F + smax * t * t;
        temp2 = absxk;
      } else {
        t = absxk / temp2;
        smax += t * t;
      }
    }

    smax = temp2 * (float)sqrt((double)smax);
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
    temp2 = 0.0F;
    smax = c_xnrm2(5 - i, A, i_i + 2);
    if (smax != 0.0F) {
      smax = rt_hypotf_snf(A[i_i], smax);
      if (A[i_i] >= 0.0F) {
        smax = -smax;
      }

      if ((float)fabs((double)smax) < 9.86076132E-32F) {
        itemp = 0;
        do {
          itemp++;
          i32 = i_i - i;
          for (k = i_i + 1; k + 1 <= i32 + 6; k++) {
            A[k] *= 1.01412048E+31F;
          }

          smax *= 1.01412048E+31F;
          absxk *= 1.01412048E+31F;
        } while (!((float)fabs((double)smax) >= 9.86076132E-32F));

        smax = rt_hypotf_snf(absxk, c_xnrm2(5 - i, A, i_i + 2));
        if (absxk >= 0.0F) {
          smax = -smax;
        }

        temp2 = (smax - absxk) / smax;
        absxk = 1.0F / (absxk - smax);
        i32 = i_i - i;
        for (k = i_i + 1; k + 1 <= i32 + 6; k++) {
          A[k] *= absxk;
        }

        for (k = 1; k <= itemp; k++) {
          smax *= 9.86076132E-32F;
        }

        absxk = smax;
      } else {
        temp2 = (smax - A[i_i]) / smax;
        absxk = 1.0F / (A[i_i] - smax);
        i32 = i_i - i;
        for (k = i_i + 1; k + 1 <= i32 + 6; k++) {
          A[k] *= absxk;
        }

        absxk = smax;
      }
    }

    tau[i] = temp2;
    A[i_i] = absxk;
    if (i + 1 < 5) {
      absxk = A[i_i];
      A[i_i] = 1.0F;
      i_ip1 = (i + (i + 1) * 6) + 1;
      if (tau[i] != 0.0F) {
        lastv = 6 - i;
        itemp = i_i - i;
        while ((lastv > 0) && (A[itemp + 5] == 0.0F)) {
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
              if (A[k - 1] != 0.0F) {
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
            work[iy - 1] = 0.0F;
          }

          iy = 0;
          i32 = i_ip1 + 6 * (lastc - 1);
          for (itemp = i_ip1; itemp <= i32; itemp += 6) {
            ix = i_i;
            smax = 0.0F;
            pvt = (itemp + lastv) - 1;
            for (k = itemp; k <= pvt; k++) {
              smax += A[k - 1] * A[ix];
              ix++;
            }

            work[iy] += smax;
            iy++;
          }
        }

        if (-tau[i] == 0.0F) {
        } else {
          itemp = i_ip1 - 1;
          pvt = 0;
          for (iy = 1; iy <= lastc; iy++) {
            if (work[pvt] != 0.0F) {
              smax = work[pvt] * -tau[i];
              ix = i_i;
              i32 = lastv + itemp;
              for (k = itemp; k + 1 <= i32; k++) {
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
      if (vn1[iy] != 0.0F) {
        smax = (float)fabs((double)A[i + 6 * iy]) / vn1[iy];
        smax = 1.0F - smax * smax;
        if (smax < 0.0F) {
          smax = 0.0F;
        }

        temp2 = vn1[iy] / vn2[iy];
        temp2 = smax * (temp2 * temp2);
        if (temp2 <= 0.000345266977F) {
          smax = 0.0F;
          if (5 - i == 1) {
            smax = (float)fabs((double)A[itemp]);
          } else {
            temp2 = 1.17549435E-38F;
            pvt = (itemp - i) + 5;
            while (itemp + 1 <= pvt) {
              absxk = (float)fabs((double)A[itemp]);
              if (absxk > temp2) {
                t = temp2 / absxk;
                smax = 1.0F + smax * t * t;
                temp2 = absxk;
              } else {
                t = absxk / temp2;
                smax += t * t;
              }

              itemp++;
            }

            smax = temp2 * (float)sqrt((double)smax);
          }

          vn1[iy] = smax;
          vn2[iy] = vn1[iy];
        } else {
          vn1[iy] *= (float)sqrt((double)smax);
        }
      }
    }
  }
}

//
// Arguments    : int n
//                const emxArray_real32_T *x
//                int ix0
// Return Type  : float
//
static float b_xnrm2(int n, const emxArray_real32_T *x, int ix0)
{
  float y;
  float scale;
  int kend;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  if (n < 1) {
  } else if (n == 1) {
    y = (float)fabs((double)x->data[ix0 - 1]);
  } else {
    scale = 1.17549435E-38F;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = (float)fabs((double)x->data[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0F + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * (float)sqrt((double)y);
  }

  return y;
}

//
// Arguments    : const float x[5]
// Return Type  : boolean_T
//
static boolean_T c_any(const float x[5])
{
  boolean_T y;
  int k;
  boolean_T exitg1;
  boolean_T b0;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 5)) {
    if ((x[k] == 0.0F) || rtIsNaNF(x[k])) {
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
// Arguments    : float varargin_1
// Return Type  : double
//
static double c_fprintf(float varargin_1)
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
// Arguments    : const float x[2]
// Return Type  : float
//
static float c_norm(const float x[2])
{
  float y;
  float scale;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  scale = 1.17549435E-38F;
  for (k = 0; k < 2; k++) {
    absxk = (float)fabs((double)x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * (float)sqrt((double)y);
}

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                int varargin_2
//                int varargin_3
// Return Type  : void
//
static void c_ros_info(int varargin_1, int varargin_2, int varargin_3)
{
  char cv14[24];
  int i13;
  static const char cv15[24] = { 'R', 'e', 'j', 'e', 'c', 't', 'i', 'n', 'g',
    ' ', '%', 'i', ' ', '(', '%', 'i', ' ', 'o', 'n', ' ', '%', 'i', ')', '\x00'
  };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0F) {
    for (i13 = 0; i13 < 24; i13++) {
      cv14[i13] = cv15[i13];
    }

    ROS_INFO(cv14, varargin_1, varargin_2, varargin_3);
  }
}

//
// Arguments    : emxArray_real32_T *x
//                emxArray_int32_T *idx
// Return Type  : void
//
static void c_sort(emxArray_real32_T *x, emxArray_int32_T *idx)
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
//                const float x[30]
//                int ix0
// Return Type  : float
//
static float c_xnrm2(int n, const float x[30], int ix0)
{
  float y;
  float scale;
  int kend;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  if (n == 1) {
    y = (float)fabs((double)x[ix0 - 1]);
  } else {
    scale = 1.17549435E-38F;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = (float)fabs((double)x[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0F + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * (float)sqrt((double)y);
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
// Arguments    : const float a[3]
//                const float b[3]
//                float c[3]
// Return Type  : void
//
static void cross(const float a[3], const float b[3], float c[3])
{
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

//
// Arguments    : float varargin_1
// Return Type  : double
//
static double d_fprintf(float varargin_1)
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
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                int varargin_2
// Return Type  : void
//
static void d_ros_info(int varargin_1, int varargin_2)
{
  char cv16[44];
  int i16;
  static const char cv17[44] = { 's', 'u', 'c', 'c', 'e', 's', 's', 'f', 'u',
    'l', 'l', 'y', ' ', 't', 'r', 'i', 'a', 'n', 'g', 'u', 'l', 'a', 't', 'e',
    'd', ' ', '%', 'd', ' ', 'o', 'f', ' ', '%', 'd', ' ', 'f', 'e', 'a', 't',
    'u', 'r', 'e', 's', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0F) {
    for (i16 = 0; i16 < 44; i16++) {
      cv16[i16] = cv17[i16];
    }

    ROS_INFO(cv16, varargin_1, varargin_2);
  }
}

//
// Arguments    : emxArray_real32_T *x
//                int dim
//                emxArray_int32_T *idx
// Return Type  : void
//
static void d_sort(emxArray_real32_T *x, int dim, emxArray_int32_T *idx)
{
  int i33;
  emxArray_real32_T *vwork;
  int vstride;
  int x_idx_0;
  int j;
  emxArray_int32_T *iidx;
  if (dim <= 1) {
    i33 = x->size[0];
  } else {
    i33 = 1;
  }

  emxInit_real32_T(&vwork, 1);
  vstride = vwork->size[0];
  vwork->size[0] = i33;
  emxEnsureCapacity((emxArray__common *)vwork, vstride, (int)sizeof(float));
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
    for (x_idx_0 = 0; x_idx_0 + 1 <= i33; x_idx_0++) {
      vwork->data[x_idx_0] = x->data[j + x_idx_0 * vstride];
    }

    sortIdx(vwork, iidx);
    for (x_idx_0 = 0; x_idx_0 + 1 <= i33; x_idx_0++) {
      x->data[j + x_idx_0 * vstride] = vwork->data[x_idx_0];
      idx->data[j + x_idx_0 * vstride] = iidx->data[x_idx_0];
    }

    j++;
  }

  emxFree_int32_T(&iidx);
  emxFree_real32_T(&vwork);
}

//
// Arguments    : const float x[36]
// Return Type  : float
//
static float det(const float x[36])
{
  float y;
  float A[36];
  signed char ipiv[6];
  int i21;
  int j;
  int c;
  int iy;
  int ix;
  float smax;
  int jy;
  float s;
  int b_j;
  int ijA;
  boolean_T isodd;
  memcpy(&A[0], &x[0], 36U * sizeof(float));
  for (i21 = 0; i21 < 6; i21++) {
    ipiv[i21] = (signed char)(1 + i21);
  }

  for (j = 0; j < 5; j++) {
    c = j * 7;
    iy = 0;
    ix = c;
    smax = (float)fabs((double)A[c]);
    for (jy = 1; jy + 1 <= 6 - j; jy++) {
      ix++;
      s = (float)fabs((double)A[ix]);
      if (s > smax) {
        iy = jy;
        smax = s;
      }
    }

    if (A[c + iy] != 0.0F) {
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

      i21 = (c - j) + 6;
      for (iy = c + 1; iy + 1 <= i21; iy++) {
        A[iy] /= A[c];
      }
    }

    iy = c;
    jy = c + 6;
    for (b_j = 1; b_j <= 5 - j; b_j++) {
      smax = A[jy];
      if (A[jy] != 0.0F) {
        ix = c + 1;
        i21 = (iy - j) + 12;
        for (ijA = 7 + iy; ijA + 1 <= i21; ijA++) {
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
// Arguments    : const float v[3]
//                float d[9]
// Return Type  : void
//
static void diag(const float v[3], float d[9])
{
  int j;
  for (j = 0; j < 9; j++) {
    d[j] = 0.0F;
  }

  for (j = 0; j < 3; j++) {
    d[j + 3 * j] = v[j];
  }
}

//
// Arguments    : float varargin_1
// Return Type  : double
//
static double e_fprintf(float varargin_1)
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
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
// Return Type  : void
//
static void e_ros_info(int varargin_1)
{
  char cv18[52];
  int i17;
  static const char cv19[52] = { 'I', 'n', 'i', 't', 'i', 'a', 'l', 'i', 'z',
    'i', 'n', 'g', ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ',', ' ',
    'w', 'h', 'i', 'c', 'h', ' ', 'w', 'a', 's', ' ', 't', 'h', 'e', ' ', 'o',
    'r', 'i', 'g', 'i', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0F) {
    for (i17 = 0; i17 < 52; i17++) {
      cv18[i17] = cv19[i17];
    }

    ROS_INFO(cv18, varargin_1);
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
// Arguments    : emxArray_real32_T **pEmxArray
// Return Type  : void
//
static void emxFree_real32_T(emxArray_real32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real32_T *)NULL) {
    if (((*pEmxArray)->data != (float *)NULL) && (*pEmxArray)->canFreeData) {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_real32_T *)NULL;
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
// Arguments    : emxArray_real32_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
static void emxInit_real32_T(emxArray_real32_T **pEmxArray, int b_numDimensions)
{
  emxArray_real32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real32_T *)malloc(sizeof(emxArray_real32_T));
  emxArray = *pEmxArray;
  emxArray->data = (float *)NULL;
  emxArray->numDimensions = b_numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * b_numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < b_numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : emxArray_real32_T **pEmxArray
//                int b_numDimensions
// Return Type  : void
//
static void emxInit_real32_T1(emxArray_real32_T **pEmxArray, int b_numDimensions)
{
  emxArray_real32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real32_T *)malloc(sizeof(emxArray_real32_T));
  emxArray = *pEmxArray;
  emxArray->data = (float *)NULL;
  emxArray->numDimensions = b_numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * b_numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < b_numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

//
// Arguments    : float varargin_1
// Return Type  : double
//
static double f_fprintf(float varargin_1)
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
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
// Return Type  : void
//
static void f_ros_info(int varargin_1)
{
  char cv20[45];
  int i18;
  static const char cv21[45] = { 'F', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%',
    'd', ' ', 'i', 's', ' ', 't', 'o', 'o', ' ', 'f', 'a', 'r', ' ', 'a', 'w',
    'a', 'y', ' ', 't', 'o', ' ', 't', 'r', 'i', 'a', 'n', 'g', 'u', 'l', 'a',
    't', 'e', '.', '\\', 'n', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0F) {
    for (i18 = 0; i18 < 45; i18++) {
      cv20[i18] = cv21[i18];
    }

    ROS_INFO(cv20, varargin_1);
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
// Arguments    : float varargin_1
// Return Type  : double
//
static double g_fprintf(float varargin_1)
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
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                int varargin_2
//                int varargin_3
// Return Type  : void
//
static void g_ros_info(int varargin_1, int varargin_2, int varargin_3)
{
  char cv22[48];
  int i19;
  static const char cv23[48] = { 'I', 'n', 's', 'e', 'r', 't', 'i', 'n', 'g',
    ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%', 'd', ' ', 'a', 's', ' ',
    'f', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%', 'i', ' ', 'o', 'n', ' ', 'a',
    'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0F) {
    for (i19 = 0; i19 < 48; i19++) {
      cv22[i19] = cv23[i19];
    }

    ROS_INFO(cv22, varargin_1, varargin_2, varargin_3);
  }
}

//
// getAnchorPoses Get the anchor poses in the world frame
// Arguments    : const float xt_origin_pos[3]
//                const float xt_origin_att[4]
//                const f_struct_T xt_anchor_states[5]
//                struct_T anchor_poses[5]
// Return Type  : void
//
static void getAnchorPoses(const float xt_origin_pos[3], const float
  xt_origin_att[4], const f_struct_T xt_anchor_states[5], struct_T anchor_poses
  [5])
{
  float R_ow[9];
  int anchorIdx;
  static const struct_T rv0[5] = { { { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F,
        0.0F } }, { { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F, 0.0F } }, { { 0.0F,
        0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F, 0.0F } }, { { 0.0F, 0.0F, 0.0F }, {
        0.0F, 0.0F, 0.0F, 0.0F } }, { { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F,
        0.0F } } };

  float b_xt_anchor_states[9];
  int ixstart;
  float mtmp;
  int itmp;
  float c[9];
  int ix;
  float varargin_1[4];
  boolean_T exitg1;

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_ow[0] = ((xt_origin_att[0] * xt_origin_att[0] - xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[3] = 2.0F * (xt_origin_att[0] * xt_origin_att[1] + xt_origin_att[2] *
                    xt_origin_att[3]);
  R_ow[6] = 2.0F * (xt_origin_att[0] * xt_origin_att[2] - xt_origin_att[1] *
                    xt_origin_att[3]);
  R_ow[1] = 2.0F * (xt_origin_att[0] * xt_origin_att[1] - xt_origin_att[2] *
                    xt_origin_att[3]);
  R_ow[4] = ((-(xt_origin_att[0] * xt_origin_att[0]) + xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[7] = 2.0F * (xt_origin_att[1] * xt_origin_att[2] + xt_origin_att[0] *
                    xt_origin_att[3]);
  R_ow[2] = 2.0F * (xt_origin_att[0] * xt_origin_att[2] + xt_origin_att[1] *
                    xt_origin_att[3]);
  R_ow[5] = 2.0F * (xt_origin_att[1] * xt_origin_att[2] - xt_origin_att[0] *
                    xt_origin_att[3]);
  R_ow[8] = ((-(xt_origin_att[0] * xt_origin_att[0]) - xt_origin_att[1] *
              xt_origin_att[1]) + xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    anchor_poses[anchorIdx] = rv0[anchorIdx];

    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    b_xt_anchor_states[0] = ((xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    b_xt_anchor_states[3] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[6] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[1] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[4] = ((-(xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0]) + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    b_xt_anchor_states[7] = 2.0F * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[2] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[5] = 2.0F * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[8] = ((-(xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0]) - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    for (ixstart = 0; ixstart < 3; ixstart++) {
      mtmp = 0.0F;
      for (itmp = 0; itmp < 3; itmp++) {
        mtmp += R_ow[itmp + 3 * ixstart] * xt_anchor_states[anchorIdx].pos[itmp];
        c[ixstart + 3 * itmp] = 0.0F;
        for (ix = 0; ix < 3; ix++) {
          c[ixstart + 3 * itmp] += b_xt_anchor_states[ixstart + 3 * ix] *
            R_ow[ix + 3 * itmp];
        }
      }

      anchor_poses[anchorIdx].pos[ixstart] = xt_origin_pos[ixstart] + mtmp;
    }

    //  THIS IS OK, It is according to the NASA memo found
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
    // % speed optimization
    varargin_1[0] = (1.0F + c[0]) - (c[4] + c[8]);
    varargin_1[1] = (1.0F + c[4]) - (c[0] + c[8]);
    varargin_1[2] = (1.0F + c[8]) - (c[0] + c[4]);
    varargin_1[3] = 1.0F + ((c[0] + c[4]) + c[8]);
    ixstart = 1;
    mtmp = varargin_1[0];
    itmp = 1;
    if (rtIsNaNF(varargin_1[0])) {
      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix < 5)) {
        ixstart = ix;
        if (!rtIsNaNF(varargin_1[ix - 1])) {
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

    if (itmp == 1) {
      anchor_poses[anchorIdx].att[0] = (float)sqrt((double)((1.0F + 2.0F * c[0])
        - ((c[0] + c[4]) + c[8]))) / 2.0F;
      anchor_poses[anchorIdx].att[1] = (c[3] + c[1]) / (4.0F * ((float)sqrt
        ((double)((1.0F + 2.0F * c[0]) - ((c[0] + c[4]) + c[8]))) / 2.0F));
      anchor_poses[anchorIdx].att[2] = (c[6] + c[2]) / (4.0F * ((float)sqrt
        ((double)((1.0F + 2.0F * c[0]) - ((c[0] + c[4]) + c[8]))) / 2.0F));
      anchor_poses[anchorIdx].att[3] = (c[7] - c[5]) / (4.0F * ((float)sqrt
        ((double)((1.0F + 2.0F * c[0]) - ((c[0] + c[4]) + c[8]))) / 2.0F));
    } else if (itmp == 2) {
      anchor_poses[anchorIdx].att[0] = (c[3] + c[1]) / (4.0F * ((float)sqrt
        ((double)((1.0F + 2.0F * c[4]) - ((c[0] + c[4]) + c[8]))) / 2.0F));
      anchor_poses[anchorIdx].att[1] = (float)sqrt((double)((1.0F + 2.0F * c[4])
        - ((c[0] + c[4]) + c[8]))) / 2.0F;
      anchor_poses[anchorIdx].att[2] = (c[7] + c[5]) / (4.0F * ((float)sqrt
        ((double)((1.0F + 2.0F * c[4]) - ((c[0] + c[4]) + c[8]))) / 2.0F));
      anchor_poses[anchorIdx].att[3] = (c[2] - c[6]) / (4.0F * ((float)sqrt
        ((double)((1.0F + 2.0F * c[4]) - ((c[0] + c[4]) + c[8]))) / 2.0F));
    } else if (itmp == 3) {
      anchor_poses[anchorIdx].att[0] = (c[6] + c[2]) / (4.0F * ((float)sqrt
        ((double)((1.0F + 2.0F * c[8]) - ((c[0] + c[4]) + c[8]))) / 2.0F));
      anchor_poses[anchorIdx].att[1] = (c[7] + c[5]) / (4.0F * ((float)sqrt
        ((double)((1.0F + 2.0F * c[8]) - ((c[0] + c[4]) + c[8]))) / 2.0F));
      anchor_poses[anchorIdx].att[2] = (float)sqrt((double)((1.0F + 2.0F * c[8])
        - ((c[0] + c[4]) + c[8]))) / 2.0F;
      anchor_poses[anchorIdx].att[3] = (c[3] - c[1]) / (4.0F * ((float)sqrt
        ((double)((1.0F + 2.0F * c[8]) - ((c[0] + c[4]) + c[8]))) / 2.0F));
    } else {
      anchor_poses[anchorIdx].att[0] = (c[7] - c[5]) / (4.0F * ((float)sqrt
        ((double)(1.0F + ((c[0] + c[4]) + c[8]))) / 2.0F));
      anchor_poses[anchorIdx].att[1] = (c[2] - c[6]) / (4.0F * ((float)sqrt
        ((double)(1.0F + ((c[0] + c[4]) + c[8]))) / 2.0F));
      anchor_poses[anchorIdx].att[2] = (c[3] - c[1]) / (4.0F * ((float)sqrt
        ((double)(1.0F + ((c[0] + c[4]) + c[8]))) / 2.0F));
      anchor_poses[anchorIdx].att[3] = (float)sqrt((double)(1.0F + ((c[0] + c[4])
        + c[8]))) / 2.0F;
    }
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
// Arguments    : const float xt_robot_state_pos[3]
//                const float xt_robot_state_att[4]
//                const f_struct_T xt_anchor_states[5]
//                const float z_all_l[80]
//                const boolean_T b_status[40]
//                const float cameraparams_FocalLength[2]
//                const float cameraparams_PrincipalPoint[2]
//                float noiseParameters_image_noise
//                emxArray_real32_T *r
//                emxArray_real32_T *H
//                emxArray_real32_T *R
// Return Type  : void
//
static void getH_R_res(const float xt_robot_state_pos[3], const float
  xt_robot_state_att[4], const f_struct_T xt_anchor_states[5], const float
  z_all_l[80], const boolean_T b_status[40], const float
  cameraparams_FocalLength[2], const float cameraparams_PrincipalPoint[2], float
  noiseParameters_image_noise, emxArray_real32_T *r, emxArray_real32_T *H,
  emxArray_real32_T *R)
{
  float fx;
  float fy;
  int n;
  int k;
  float R_cw[9];
  float c;
  int ia;
  emxArray_real32_T *b_h_u;
  float res_idx;
  int anchorIdx;
  float anchorRot[9];
  int featureIdx;
  int outsize_idx_1;
  int b_c;
  float h_c_n_l[2];
  float h_u_To_h_ci_l[6];
  float h_ci_l_To_R_cw[9];
  int ar;
  float H_robot[63];
  float b_xt_anchor_states[3];
  float c_c[3];
  float h_ci_l_To_rho[3];
  int ib;
  float H_map_data[63];
  float b_R_cw[9];
  float fv0[9];
  float c_R_cw[9];
  float b_data[462];
  float C_data[308];
  int ic;
  fx = cameraparams_FocalLength[0];
  fy = cameraparams_FocalLength[1];
  n = 0;
  for (k = 0; k < 40; k++) {
    if (b_status[k]) {
      n++;
    }
  }

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_cw[0] = ((xt_robot_state_att[0] * xt_robot_state_att[0] -
              xt_robot_state_att[1] * xt_robot_state_att[1]) -
             xt_robot_state_att[2] * xt_robot_state_att[2]) +
    xt_robot_state_att[3] * xt_robot_state_att[3];
  R_cw[3] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[1] +
                    xt_robot_state_att[2] * xt_robot_state_att[3]);
  R_cw[6] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[2] -
                    xt_robot_state_att[1] * xt_robot_state_att[3]);
  R_cw[1] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[1] -
                    xt_robot_state_att[2] * xt_robot_state_att[3]);
  R_cw[4] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) +
              xt_robot_state_att[1] * xt_robot_state_att[1]) -
             xt_robot_state_att[2] * xt_robot_state_att[2]) +
    xt_robot_state_att[3] * xt_robot_state_att[3];
  R_cw[7] = 2.0F * (xt_robot_state_att[1] * xt_robot_state_att[2] +
                    xt_robot_state_att[0] * xt_robot_state_att[3]);
  R_cw[2] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[2] +
                    xt_robot_state_att[1] * xt_robot_state_att[3]);
  R_cw[5] = 2.0F * (xt_robot_state_att[1] * xt_robot_state_att[2] -
                    xt_robot_state_att[0] * xt_robot_state_att[3]);
  R_cw[8] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) -
              xt_robot_state_att[1] * xt_robot_state_att[1]) +
             xt_robot_state_att[2] * xt_robot_state_att[2]) +
    xt_robot_state_att[3] * xt_robot_state_att[3];
  c = (float)n * 2.0F;
  ia = H->size[0] * H->size[1];
  H->size[0] = (int)c;
  H->size[1] = 91;
  emxEnsureCapacity((emxArray__common *)H, ia, (int)sizeof(float));
  k = (int)c * 91;
  for (ia = 0; ia < k; ia++) {
    H->data[ia] = 0.0F;
  }

  c = (float)n * 2.0F;
  ia = r->size[0];
  r->size[0] = (int)c;
  emxEnsureCapacity((emxArray__common *)r, ia, (int)sizeof(float));
  k = (int)c;
  for (ia = 0; ia < k; ia++) {
    r->data[ia] = 0.0F;
  }

  emxInit_real32_T(&b_h_u, 1);
  c = (float)n * 2.0F;
  ia = b_h_u->size[0];
  b_h_u->size[0] = (int)c;
  emxEnsureCapacity((emxArray__common *)b_h_u, ia, (int)sizeof(float));
  k = (int)c;
  for (ia = 0; ia < k; ia++) {
    b_h_u->data[ia] = 0.0F;
  }

  res_idx = 1.0F;
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    anchorRot[0] = ((xt_anchor_states[anchorIdx].att[0] *
                     xt_anchor_states[anchorIdx].att[0] -
                     xt_anchor_states[anchorIdx].att[1] *
                     xt_anchor_states[anchorIdx].att[1]) -
                    xt_anchor_states[anchorIdx].att[2] *
                    xt_anchor_states[anchorIdx].att[2]) +
      xt_anchor_states[anchorIdx].att[3] * xt_anchor_states[anchorIdx].att[3];
    anchorRot[3] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    anchorRot[6] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    anchorRot[1] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    anchorRot[4] = ((-(xt_anchor_states[anchorIdx].att[0] *
                       xt_anchor_states[anchorIdx].att[0]) +
                     xt_anchor_states[anchorIdx].att[1] *
                     xt_anchor_states[anchorIdx].att[1]) -
                    xt_anchor_states[anchorIdx].att[2] *
                    xt_anchor_states[anchorIdx].att[2]) +
      xt_anchor_states[anchorIdx].att[3] * xt_anchor_states[anchorIdx].att[3];
    anchorRot[7] = 2.0F * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    anchorRot[2] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    anchorRot[5] = 2.0F * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    anchorRot[8] = ((-(xt_anchor_states[anchorIdx].att[0] *
                       xt_anchor_states[anchorIdx].att[0]) -
                     xt_anchor_states[anchorIdx].att[1] *
                     xt_anchor_states[anchorIdx].att[1]) +
                    xt_anchor_states[anchorIdx].att[2] *
                    xt_anchor_states[anchorIdx].att[2]) +
      xt_anchor_states[anchorIdx].att[3] * xt_anchor_states[anchorIdx].att[3];
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status != 0) &&
          b_status[xt_anchor_states[anchorIdx].feature_states[featureIdx].
          status_idx - 1]) {
        k = xt_anchor_states[anchorIdx].feature_states[featureIdx].status_idx;
        outsize_idx_1 = k - 1;
        if ((k < 0) && (outsize_idx_1 >= 0)) {
          outsize_idx_1 = MIN_int32_T;
        }

        if (outsize_idx_1 > 1073741823) {
          b_c = MAX_int32_T;
        } else if (outsize_idx_1 <= -1073741824) {
          b_c = MIN_int32_T;
        } else {
          b_c = outsize_idx_1 << 1;
        }

        // predictMeasurementLeft Predict the measurement of a feature given in the left 
        // camera frame
        //    Get the normalized pixel coordinates where a feature given in the left camera 
        //    frame
        for (ia = 0; ia < 2; ia++) {
          h_c_n_l[ia] = xt_anchor_states[anchorIdx].feature_states[featureIdx].
            scaled_map_point[ia] / xt_anchor_states[anchorIdx]
            .feature_states[featureIdx].scaled_map_point[2];
        }

        //  normalized feature in camera frame
        c = (res_idx - 1.0F) * 2.0F;
        b_h_u->data[(int)(c + 1.0F) - 1] = h_c_n_l[0] *
          cameraparams_FocalLength[0] + cameraparams_PrincipalPoint[0];
        b_h_u->data[(int)(c + 2.0F) - 1] = h_c_n_l[1] *
          cameraparams_FocalLength[1] + cameraparams_PrincipalPoint[1];
        c = (res_idx - 1.0F) * 2.0F;
        for (ia = 0; ia < 2; ia++) {
          k = 1 + ia;
          outsize_idx_1 = b_c + k;
          if ((b_c > 0) && (outsize_idx_1 <= 0)) {
            outsize_idx_1 = MAX_int32_T;
          }

          r->data[(int)(c + (1.0F + (float)ia)) - 1] = z_all_l[outsize_idx_1 - 1];
        }

        h_u_To_h_ci_l[0] = fx / xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[2];
        h_u_To_h_ci_l[2] = 0.0F;
        h_u_To_h_ci_l[4] = -fx * xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[0] /
          (xt_anchor_states[anchorIdx].feature_states[featureIdx].
           scaled_map_point[2] * xt_anchor_states[anchorIdx]
           .feature_states[featureIdx].scaled_map_point[2]);
        h_u_To_h_ci_l[1] = 0.0F;
        h_u_To_h_ci_l[3] = fy / xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[2];
        h_u_To_h_ci_l[5] = -fy * xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[1] /
          (xt_anchor_states[anchorIdx].feature_states[featureIdx].
           scaled_map_point[2] * xt_anchor_states[anchorIdx]
           .feature_states[featureIdx].scaled_map_point[2]);
        h_ci_l_To_R_cw[0] = 0.0F;
        h_ci_l_To_R_cw[3] = -xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[2];
        h_ci_l_To_R_cw[6] = xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[1];
        h_ci_l_To_R_cw[1] = xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[2];
        h_ci_l_To_R_cw[4] = 0.0F;
        h_ci_l_To_R_cw[7] = -xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[0];
        h_ci_l_To_R_cw[2] = -xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[1];
        h_ci_l_To_R_cw[5] = xt_anchor_states[anchorIdx]
          .feature_states[featureIdx].scaled_map_point[0];
        h_ci_l_To_R_cw[8] = 0.0F;
        if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 2)
            || (xt_anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0F)) {
          //  delayed initialization or feature behind anchor
          for (ia = 0; ia < 3; ia++) {
            for (ar = 0; ar < 3; ar++) {
              H_robot[ar + 3 * ia] = 0.0F;
              H_robot[ar + 3 * (ia + 3)] = 0.0F * h_ci_l_To_R_cw[ar + 3 * ia];
            }
          }

          for (ia = 0; ia < 15; ia++) {
            for (ar = 0; ar < 3; ar++) {
              H_robot[ar + 3 * (ia + 6)] = 0.0F;
            }
          }
        } else {
          for (ia = 0; ia < 3; ia++) {
            for (ar = 0; ar < 3; ar++) {
              H_robot[ar + 3 * ia] = -xt_anchor_states[anchorIdx]
                .feature_states[featureIdx].inverse_depth * R_cw[ar + 3 * ia];
              H_robot[ar + 3 * (ia + 3)] = h_ci_l_To_R_cw[ar + 3 * ia];
            }
          }

          for (ia = 0; ia < 15; ia++) {
            for (ar = 0; ar < 3; ar++) {
              H_robot[ar + 3 * (ia + 6)] = 0.0F;
            }
          }
        }

        //             %% anchor state derivatives
        for (ia = 0; ia < 3; ia++) {
          c_c[ia] = 0.0F;
          for (ar = 0; ar < 3; ar++) {
            c_c[ia] += anchorRot[ar + 3 * ia] * xt_anchor_states[anchorIdx].
              feature_states[featureIdx].m[ar];
          }

          b_xt_anchor_states[ia] = xt_anchor_states[anchorIdx].pos[ia] -
            xt_robot_state_pos[ia];
        }

        for (ia = 0; ia < 3; ia++) {
          h_ci_l_To_rho[ia] = 0.0F;
          for (ar = 0; ar < 3; ar++) {
            h_ci_l_To_rho[ia] += R_cw[ia + 3 * ar] * b_xt_anchor_states[ar];
          }
        }

        if ((xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 2)
            || (xt_anchor_states[anchorIdx].feature_states[featureIdx].
                inverse_depth < 0.0F)) {
          //  delayed initialization or feature behind anchor
          b_c = -featureIdx;
          ib = (featureIdx + b_c) + 14;
          k = featureIdx + 6;
          for (ia = 0; ia < k; ia++) {
            for (ar = 0; ar < 3; ar++) {
              H_map_data[ar + 3 * ia] = 0.0F;
            }
          }

          for (ia = 0; ia < 3; ia++) {
            H_map_data[ia + 3 * (featureIdx + 6)] = h_ci_l_To_rho[ia];
          }

          k = b_c + 7;
          for (ia = 0; ia < k; ia++) {
            for (ar = 0; ar < 3; ar++) {
              H_map_data[ar + 3 * ((ia + featureIdx) + 7)] = 0.0F;
            }
          }
        } else {
          b_c = -featureIdx;
          for (ia = 0; ia < 3; ia++) {
            for (ar = 0; ar < 3; ar++) {
              b_R_cw[ar + 3 * ia] = -R_cw[ar + 3 * ia];
            }
          }

          fv0[0] = 0.0F;
          fv0[3] = -c_c[2];
          fv0[6] = c_c[1];
          fv0[1] = c_c[2];
          fv0[4] = 0.0F;
          fv0[7] = -c_c[0];
          fv0[2] = -c_c[1];
          fv0[5] = c_c[0];
          fv0[8] = 0.0F;
          ib = (featureIdx + b_c) + 14;
          for (ia = 0; ia < 3; ia++) {
            for (ar = 0; ar < 3; ar++) {
              c_R_cw[ia + 3 * ar] = 0.0F;
              for (k = 0; k < 3; k++) {
                c_R_cw[ia + 3 * ar] += b_R_cw[ia + 3 * k] * fv0[k + 3 * ar];
              }

              H_map_data[ar + 3 * ia] = xt_anchor_states[anchorIdx].
                feature_states[featureIdx].inverse_depth * R_cw[ar + 3 * ia];
            }
          }

          for (ia = 0; ia < 3; ia++) {
            for (ar = 0; ar < 3; ar++) {
              H_map_data[ar + 3 * (ia + 3)] = c_R_cw[ar + 3 * ia];
            }
          }

          for (ia = 0; ia < featureIdx; ia++) {
            for (ar = 0; ar < 3; ar++) {
              H_map_data[ar + 3 * (ia + 6)] = 0.0F;
            }
          }

          for (ia = 0; ia < 3; ia++) {
            H_map_data[ia + 3 * (6 + featureIdx)] = h_ci_l_To_rho[ia];
          }

          k = b_c + 7;
          for (ia = 0; ia < k; ia++) {
            for (ar = 0; ar < 3; ar++) {
              H_map_data[ar + 3 * ((ia + featureIdx) + 7)] = 0.0F;
            }
          }
        }

        c = (res_idx - 1.0F) * 2.0F;
        b_c = anchorIdx * 14;
        k = (4 - anchorIdx) * 14;
        outsize_idx_1 = ((b_c + ib) + k) + 21;
        for (ia = 0; ia < 21; ia++) {
          for (ar = 0; ar < 3; ar++) {
            b_data[ar + 3 * ia] = H_robot[ar + 3 * ia];
          }
        }

        for (ia = 0; ia < b_c; ia++) {
          for (ar = 0; ar < 3; ar++) {
            b_data[ar + 3 * (ia + 21)] = 0.0F;
          }
        }

        for (ia = 0; ia < ib; ia++) {
          for (ar = 0; ar < 3; ar++) {
            b_data[ar + 3 * ((ia + b_c) + 21)] = H_map_data[ar + 3 * ia];
          }
        }

        for (ia = 0; ia < k; ia++) {
          for (ar = 0; ar < 3; ar++) {
            b_data[ar + 3 * (((ia + b_c) + ib) + 21)] = 0.0F;
          }
        }

        for (ia = 0; ia < outsize_idx_1; ia++) {
          for (ar = 0; ar < 2; ar++) {
            C_data[ar + (ia << 1)] = 0.0F;
          }
        }

        b_c = (outsize_idx_1 - 1) << 1;
        for (k = 0; k <= b_c; k += 2) {
          for (ic = k; ic + 1 <= k + 2; ic++) {
            C_data[ic] = 0.0F;
          }
        }

        outsize_idx_1 = 0;
        for (k = 0; k <= b_c; k += 2) {
          ar = 0;
          for (ib = outsize_idx_1; ib + 1 <= outsize_idx_1 + 3; ib++) {
            if (b_data[ib] != 0.0F) {
              ia = ar;
              for (ic = k; ic + 1 <= k + 2; ic++) {
                ia++;
                C_data[ic] += b_data[ib] * h_u_To_h_ci_l[ia - 1];
              }
            }

            ar += 2;
          }

          outsize_idx_1 += 3;
        }

        for (ia = 0; ia < 91; ia++) {
          for (ar = 0; ar < 2; ar++) {
            H->data[((int)(c + (1.0F + (float)ar)) + H->size[0] * ia) - 1] =
              C_data[ar + 2 * ia];
          }
        }

        res_idx++;
      }
    }
  }

  ia = r->size[0];
  emxEnsureCapacity((emxArray__common *)r, ia, (int)sizeof(float));
  k = r->size[0];
  for (ia = 0; ia < k; ia++) {
    r->data[ia] -= b_h_u->data[ia];
  }

  emxFree_real32_T(&b_h_u);
  c = (float)n * 2.0F;
  outsize_idx_1 = (int)((float)n * 2.0F);
  ia = R->size[0] * R->size[1];
  R->size[0] = (int)c;
  R->size[1] = (int)c;
  emxEnsureCapacity((emxArray__common *)R, ia, (int)sizeof(float));
  k = (int)c * (int)c;
  for (ia = 0; ia < k; ia++) {
    R->data[ia] = 0.0F;
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
// Arguments    : const float xt_origin_pos[3]
//                const float xt_origin_att[4]
//                const f_struct_T xt_anchor_states[5]
//                float b_map[120]
// Return Type  : void
//
static void getMap(const float xt_origin_pos[3], const float xt_origin_att[4],
                   const f_struct_T xt_anchor_states[5], float b_map[120])
{
  float R_ow[9];
  float anchorRot[9];
  int anchorIdx;
  float b_xt_anchor_states[9];
  float anchorPos[3];
  int i0;
  float f0;
  int qY;
  int q0;
  int featureIdx;
  int c;
  float b_anchorPos[3];

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_ow[0] = ((xt_origin_att[0] * xt_origin_att[0] - xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[3] = 2.0F * (xt_origin_att[0] * xt_origin_att[1] + xt_origin_att[2] *
                    xt_origin_att[3]);
  R_ow[6] = 2.0F * (xt_origin_att[0] * xt_origin_att[2] - xt_origin_att[1] *
                    xt_origin_att[3]);
  R_ow[1] = 2.0F * (xt_origin_att[0] * xt_origin_att[1] - xt_origin_att[2] *
                    xt_origin_att[3]);
  R_ow[4] = ((-(xt_origin_att[0] * xt_origin_att[0]) + xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[7] = 2.0F * (xt_origin_att[1] * xt_origin_att[2] + xt_origin_att[0] *
                    xt_origin_att[3]);
  R_ow[2] = 2.0F * (xt_origin_att[0] * xt_origin_att[2] + xt_origin_att[1] *
                    xt_origin_att[3]);
  R_ow[5] = 2.0F * (xt_origin_att[1] * xt_origin_att[2] - xt_origin_att[0] *
                    xt_origin_att[3]);
  R_ow[8] = ((-(xt_origin_att[0] * xt_origin_att[0]) - xt_origin_att[1] *
              xt_origin_att[1]) + xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  memset(&b_map[0], 0, 120U * sizeof(float));
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    b_xt_anchor_states[0] = ((xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    b_xt_anchor_states[3] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[6] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[1] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[1] - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[4] = ((-(xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0]) + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) - xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    b_xt_anchor_states[7] = 2.0F * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[2] = 2.0F * (xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[2] + xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[5] = 2.0F * (xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[2] - xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[3]);
    b_xt_anchor_states[8] = ((-(xt_anchor_states[anchorIdx].att[0] *
      xt_anchor_states[anchorIdx].att[0]) - xt_anchor_states[anchorIdx].att[1] *
      xt_anchor_states[anchorIdx].att[1]) + xt_anchor_states[anchorIdx].att[2] *
      xt_anchor_states[anchorIdx].att[2]) + xt_anchor_states[anchorIdx].att[3] *
      xt_anchor_states[anchorIdx].att[3];
    for (i0 = 0; i0 < 3; i0++) {
      f0 = 0.0F;
      for (qY = 0; qY < 3; qY++) {
        f0 += R_ow[qY + 3 * i0] * xt_anchor_states[anchorIdx].pos[qY];
        anchorRot[i0 + 3 * qY] = 0.0F;
        for (q0 = 0; q0 < 3; q0++) {
          anchorRot[i0 + 3 * qY] += b_xt_anchor_states[i0 + 3 * q0] * R_ow[q0 +
            3 * qY];
        }
      }

      anchorPos[i0] = xt_origin_pos[i0] + f0;
    }

    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      if (xt_anchor_states[anchorIdx].feature_states[featureIdx].status != 0) {
        q0 = xt_anchor_states[anchorIdx].feature_states[featureIdx].status_idx;
        qY = q0 - 1;
        if ((q0 < 0) && (qY >= 0)) {
          qY = MIN_int32_T;
        }

        c = mul_s32_s32_s32_sat(qY, 3);
        for (i0 = 0; i0 < 3; i0++) {
          f0 = 0.0F;
          for (qY = 0; qY < 3; qY++) {
            f0 += anchorRot[qY + 3 * i0] * xt_anchor_states[anchorIdx].
              feature_states[featureIdx].m[qY];
          }

          b_anchorPos[i0] = anchorPos[i0] + f0 / xt_anchor_states[anchorIdx].
            feature_states[featureIdx].inverse_depth;
          q0 = 1 + i0;
          qY = c + q0;
          if ((c > 0) && (qY <= 0)) {
            qY = MAX_int32_T;
          }

          b_map[qY - 1] = b_anchorPos[i0];
        }
      }
    }
  }
}

//
// getNumValidFeatures Get the number of valid features of an anchor
// Arguments    : const e_struct_T anchor_state_feature_states[8]
// Return Type  : float
//
static float getNumValidFeatures(const e_struct_T anchor_state_feature_states[8])
{
  float n;
  int featureIdx;
  boolean_T x[2];
  int k;
  boolean_T y;
  boolean_T exitg1;

  // getNumFeaturesOfType Get the number of features of type type of an anchor
  //  type can be a scalar or a row vector of types
  n = 0.0F;
  for (featureIdx = 0; featureIdx < 8; featureIdx++) {
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
  float R_cw[9];
  int anchorIdx;
  float anchorRot[9];
  int featureIdx;
  float c_xt[3];
  int i26;
  float f4;
  int i27;

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_cw[0] = ((b_xt->robot_state.att[0] * b_xt->robot_state.att[0] -
              b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
             b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
    b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
  R_cw[3] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] +
                    b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
  R_cw[6] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] -
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
  R_cw[1] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[1] -
                    b_xt->robot_state.att[2] * b_xt->robot_state.att[3]);
  R_cw[4] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) +
              b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) -
             b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
    b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
  R_cw[7] = 2.0F * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] +
                    b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
  R_cw[2] = 2.0F * (b_xt->robot_state.att[0] * b_xt->robot_state.att[2] +
                    b_xt->robot_state.att[1] * b_xt->robot_state.att[3]);
  R_cw[5] = 2.0F * (b_xt->robot_state.att[1] * b_xt->robot_state.att[2] -
                    b_xt->robot_state.att[0] * b_xt->robot_state.att[3]);
  R_cw[8] = ((-(b_xt->robot_state.att[0] * b_xt->robot_state.att[0]) -
              b_xt->robot_state.att[1] * b_xt->robot_state.att[1]) +
             b_xt->robot_state.att[2] * b_xt->robot_state.att[2]) +
    b_xt->robot_state.att[3] * b_xt->robot_state.att[3];
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    anchorRot[0] = ((b_xt->anchor_states[anchorIdx].att[0] * b_xt->
                     anchor_states[anchorIdx].att[0] - b_xt->
                     anchor_states[anchorIdx].att[1] * b_xt->
                     anchor_states[anchorIdx].att[1]) - b_xt->
                    anchor_states[anchorIdx].att[2] * b_xt->
                    anchor_states[anchorIdx].att[2]) + b_xt->
      anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx].att[3];
    anchorRot[3] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
      b_xt->anchor_states[anchorIdx].att[1] + b_xt->anchor_states[anchorIdx]
      .att[2] * b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[6] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
      b_xt->anchor_states[anchorIdx].att[2] - b_xt->anchor_states[anchorIdx]
      .att[1] * b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[1] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
      b_xt->anchor_states[anchorIdx].att[1] - b_xt->anchor_states[anchorIdx]
      .att[2] * b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[4] = ((-(b_xt->anchor_states[anchorIdx].att[0] *
                       b_xt->anchor_states[anchorIdx].att[0]) +
                     b_xt->anchor_states[anchorIdx].att[1] * b_xt->
                     anchor_states[anchorIdx].att[1]) - b_xt->
                    anchor_states[anchorIdx].att[2] * b_xt->
                    anchor_states[anchorIdx].att[2]) + b_xt->
      anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx].att[3];
    anchorRot[7] = 2.0F * (b_xt->anchor_states[anchorIdx].att[1] *
      b_xt->anchor_states[anchorIdx].att[2] + b_xt->anchor_states[anchorIdx]
      .att[0] * b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[2] = 2.0F * (b_xt->anchor_states[anchorIdx].att[0] *
      b_xt->anchor_states[anchorIdx].att[2] + b_xt->anchor_states[anchorIdx]
      .att[1] * b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[5] = 2.0F * (b_xt->anchor_states[anchorIdx].att[1] *
      b_xt->anchor_states[anchorIdx].att[2] - b_xt->anchor_states[anchorIdx]
      .att[0] * b_xt->anchor_states[anchorIdx].att[3]);
    anchorRot[8] = ((-(b_xt->anchor_states[anchorIdx].att[0] *
                       b_xt->anchor_states[anchorIdx].att[0]) -
                     b_xt->anchor_states[anchorIdx].att[1] * b_xt->
                     anchor_states[anchorIdx].att[1]) + b_xt->
                    anchor_states[anchorIdx].att[2] * b_xt->
                    anchor_states[anchorIdx].att[2]) + b_xt->
      anchor_states[anchorIdx].att[3] * b_xt->anchor_states[anchorIdx].att[3];
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      if (b_xt->anchor_states[anchorIdx].feature_states[featureIdx].status != 0)
      {
        for (i26 = 0; i26 < 3; i26++) {
          f4 = 0.0F;
          for (i27 = 0; i27 < 3; i27++) {
            f4 += anchorRot[i27 + 3 * i26] * b_xt->anchor_states[anchorIdx].
              feature_states[featureIdx].m[i27];
          }

          c_xt[i26] = (b_xt->anchor_states[anchorIdx].feature_states[featureIdx]
                       .inverse_depth * b_xt->anchor_states[anchorIdx].pos[i26]
                       + f4) - b_xt->robot_state.pos[i26] * b_xt->
            anchor_states[anchorIdx].feature_states[featureIdx].inverse_depth;
        }

        for (i26 = 0; i26 < 3; i26++) {
          b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
            scaled_map_point[i26] = 0.0F;
          for (i27 = 0; i27 < 3; i27++) {
            b_xt->anchor_states[anchorIdx].feature_states[featureIdx].
              scaled_map_point[i26] += R_cw[i26 + 3 * i27] * c_xt[i27];
          }
        }
      }
    }
  }
}

//
// getTotalNumActiveFeatures Get the number of active features of all anchors
// Arguments    : const f_struct_T xt_anchor_states[5]
// Return Type  : float
//
static float getTotalNumActiveFeatures(const f_struct_T xt_anchor_states[5])
{
  float n;
  int anchorIdx;
  float b_n;
  int featureIdx;
  n = 0.0F;
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    // getNumActiveFeatures Get the number of active features of an anchor
    // getNumFeaturesOfType Get the number of features of type type of an anchor 
    //  type can be a scalar or a row vector of types
    b_n = 0.0F;
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      if (!!(xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 1))
      {
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
// Return Type  : float
//
static float getTotalNumDelayedFeatures(const f_struct_T xt_anchor_states[5])
{
  float n;
  int anchorIdx;
  float b_n;
  int featureIdx;
  n = 0.0F;
  for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
    // getNumFeaturesOfType Get the number of features of type type of an anchor 
    //  type can be a scalar or a row vector of types
    b_n = 0.0F;
    for (featureIdx = 0; featureIdx < 8; featureIdx++) {
      if (!!(xt_anchor_states[anchorIdx].feature_states[featureIdx].status == 2))
      {
        b_n++;
      }
    }

    n += b_n;
  }

  return n;
}

//
// getWorldState Get the state of the robot in the world frame
// Arguments    : const float xt_robot_state_IMU_pos[3]
//                const float xt_robot_state_IMU_att[4]
//                const float xt_robot_state_IMU_gyro_bias[3]
//                const float xt_robot_state_IMU_acc_bias[3]
//                const float xt_robot_state_pos[3]
//                const float xt_robot_state_att[4]
//                const float xt_origin_pos[3]
//                const float xt_origin_att[4]
//                float world_state_pos[3]
//                float world_state_att[4]
//                float world_state_vel[3]
//                float world_state_IMU_gyro_bias[3]
//                float world_state_IMU_acc_bias[3]
//                float world_state_IMU_pos[3]
//                float world_state_IMU_att[4]
// Return Type  : void
//
static void getWorldState(const float xt_robot_state_IMU_pos[3], const float
  xt_robot_state_IMU_att[4], const float xt_robot_state_IMU_gyro_bias[3], const
  float xt_robot_state_IMU_acc_bias[3], const float xt_robot_state_pos[3], const
  float xt_robot_state_att[4], const float xt_origin_pos[3], const float
  xt_origin_att[4], float world_state_pos[3], float world_state_att[4], float
  world_state_vel[3], float world_state_IMU_gyro_bias[3], float
  world_state_IMU_acc_bias[3], float world_state_IMU_pos[3], float
  world_state_IMU_att[4])
{
  float R_ow[9];
  float b_xt_robot_state_att[9];
  int ix;
  float mtmp;
  int ixstart;
  float c[9];
  int itmp;
  float varargin_1[4];
  boolean_T exitg1;

  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  //  if ~all(size(q) == [4, 1])
  //      error('q does not have the size of a quaternion')
  //  end
  //  if abs(norm(q) - 1) > 1e-3
  //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
  //  end
  R_ow[0] = ((xt_origin_att[0] * xt_origin_att[0] - xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[3] = 2.0F * (xt_origin_att[0] * xt_origin_att[1] + xt_origin_att[2] *
                    xt_origin_att[3]);
  R_ow[6] = 2.0F * (xt_origin_att[0] * xt_origin_att[2] - xt_origin_att[1] *
                    xt_origin_att[3]);
  R_ow[1] = 2.0F * (xt_origin_att[0] * xt_origin_att[1] - xt_origin_att[2] *
                    xt_origin_att[3]);
  R_ow[4] = ((-(xt_origin_att[0] * xt_origin_att[0]) + xt_origin_att[1] *
              xt_origin_att[1]) - xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  R_ow[7] = 2.0F * (xt_origin_att[1] * xt_origin_att[2] + xt_origin_att[0] *
                    xt_origin_att[3]);
  R_ow[2] = 2.0F * (xt_origin_att[0] * xt_origin_att[2] + xt_origin_att[1] *
                    xt_origin_att[3]);
  R_ow[5] = 2.0F * (xt_origin_att[1] * xt_origin_att[2] - xt_origin_att[0] *
                    xt_origin_att[3]);
  R_ow[8] = ((-(xt_origin_att[0] * xt_origin_att[0]) - xt_origin_att[1] *
              xt_origin_att[1]) + xt_origin_att[2] * xt_origin_att[2]) +
    xt_origin_att[3] * xt_origin_att[3];
  b_xt_robot_state_att[0] = ((xt_robot_state_att[0] * xt_robot_state_att[0] -
    xt_robot_state_att[1] * xt_robot_state_att[1]) - xt_robot_state_att[2] *
    xt_robot_state_att[2]) + xt_robot_state_att[3] * xt_robot_state_att[3];
  b_xt_robot_state_att[3] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[1]
    + xt_robot_state_att[2] * xt_robot_state_att[3]);
  b_xt_robot_state_att[6] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[2]
    - xt_robot_state_att[1] * xt_robot_state_att[3]);
  b_xt_robot_state_att[1] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[1]
    - xt_robot_state_att[2] * xt_robot_state_att[3]);
  b_xt_robot_state_att[4] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) +
    xt_robot_state_att[1] * xt_robot_state_att[1]) - xt_robot_state_att[2] *
    xt_robot_state_att[2]) + xt_robot_state_att[3] * xt_robot_state_att[3];
  b_xt_robot_state_att[7] = 2.0F * (xt_robot_state_att[1] * xt_robot_state_att[2]
    + xt_robot_state_att[0] * xt_robot_state_att[3]);
  b_xt_robot_state_att[2] = 2.0F * (xt_robot_state_att[0] * xt_robot_state_att[2]
    + xt_robot_state_att[1] * xt_robot_state_att[3]);
  b_xt_robot_state_att[5] = 2.0F * (xt_robot_state_att[1] * xt_robot_state_att[2]
    - xt_robot_state_att[0] * xt_robot_state_att[3]);
  b_xt_robot_state_att[8] = ((-(xt_robot_state_att[0] * xt_robot_state_att[0]) -
    xt_robot_state_att[1] * xt_robot_state_att[1]) + xt_robot_state_att[2] *
    xt_robot_state_att[2]) + xt_robot_state_att[3] * xt_robot_state_att[3];
  for (ix = 0; ix < 3; ix++) {
    mtmp = 0.0F;
    for (ixstart = 0; ixstart < 3; ixstart++) {
      mtmp += R_ow[ixstart + 3 * ix] * xt_robot_state_pos[ixstart];
      c[ix + 3 * ixstart] = 0.0F;
      for (itmp = 0; itmp < 3; itmp++) {
        c[ix + 3 * ixstart] += b_xt_robot_state_att[ix + 3 * itmp] * R_ow[itmp +
          3 * ixstart];
      }
    }

    world_state_pos[ix] = xt_origin_pos[ix] + mtmp;
  }

  //  THIS IS OK, It is according to the NASA memo found
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
  // % speed optimization
  varargin_1[0] = (1.0F + c[0]) - (c[4] + c[8]);
  varargin_1[1] = (1.0F + c[4]) - (c[0] + c[8]);
  varargin_1[2] = (1.0F + c[8]) - (c[0] + c[4]);
  varargin_1[3] = 1.0F + ((c[0] + c[4]) + c[8]);
  ixstart = 1;
  mtmp = varargin_1[0];
  itmp = 1;
  if (rtIsNaNF(varargin_1[0])) {
    ix = 2;
    exitg1 = false;
    while ((!exitg1) && (ix < 5)) {
      ixstart = ix;
      if (!rtIsNaNF(varargin_1[ix - 1])) {
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

  if (itmp == 1) {
    world_state_att[0] = (float)sqrt((double)((1.0F + 2.0F * c[0]) - ((c[0] + c
      [4]) + c[8]))) / 2.0F;
    world_state_att[1] = (c[3] + c[1]) / (4.0F * ((float)sqrt((double)((1.0F +
      2.0F * c[0]) - ((c[0] + c[4]) + c[8]))) / 2.0F));
    world_state_att[2] = (c[6] + c[2]) / (4.0F * ((float)sqrt((double)((1.0F +
      2.0F * c[0]) - ((c[0] + c[4]) + c[8]))) / 2.0F));
    world_state_att[3] = (c[7] - c[5]) / (4.0F * ((float)sqrt((double)((1.0F +
      2.0F * c[0]) - ((c[0] + c[4]) + c[8]))) / 2.0F));
  } else if (itmp == 2) {
    world_state_att[0] = (c[3] + c[1]) / (4.0F * ((float)sqrt((double)((1.0F +
      2.0F * c[4]) - ((c[0] + c[4]) + c[8]))) / 2.0F));
    world_state_att[1] = (float)sqrt((double)((1.0F + 2.0F * c[4]) - ((c[0] + c
      [4]) + c[8]))) / 2.0F;
    world_state_att[2] = (c[7] + c[5]) / (4.0F * ((float)sqrt((double)((1.0F +
      2.0F * c[4]) - ((c[0] + c[4]) + c[8]))) / 2.0F));
    world_state_att[3] = (c[2] - c[6]) / (4.0F * ((float)sqrt((double)((1.0F +
      2.0F * c[4]) - ((c[0] + c[4]) + c[8]))) / 2.0F));
  } else if (itmp == 3) {
    world_state_att[0] = (c[6] + c[2]) / (4.0F * ((float)sqrt((double)((1.0F +
      2.0F * c[8]) - ((c[0] + c[4]) + c[8]))) / 2.0F));
    world_state_att[1] = (c[7] + c[5]) / (4.0F * ((float)sqrt((double)((1.0F +
      2.0F * c[8]) - ((c[0] + c[4]) + c[8]))) / 2.0F));
    world_state_att[2] = (float)sqrt((double)((1.0F + 2.0F * c[8]) - ((c[0] + c
      [4]) + c[8]))) / 2.0F;
    world_state_att[3] = (c[3] - c[1]) / (4.0F * ((float)sqrt((double)((1.0F +
      2.0F * c[8]) - ((c[0] + c[4]) + c[8]))) / 2.0F));
  } else {
    world_state_att[0] = (c[7] - c[5]) / (4.0F * ((float)sqrt((double)(1.0F +
      ((c[0] + c[4]) + c[8]))) / 2.0F));
    world_state_att[1] = (c[2] - c[6]) / (4.0F * ((float)sqrt((double)(1.0F +
      ((c[0] + c[4]) + c[8]))) / 2.0F));
    world_state_att[2] = (c[3] - c[1]) / (4.0F * ((float)sqrt((double)(1.0F +
      ((c[0] + c[4]) + c[8]))) / 2.0F));
    world_state_att[3] = (float)sqrt((double)(1.0F + ((c[0] + c[4]) + c[8]))) /
      2.0F;
  }

  for (ixstart = 0; ixstart < 3; ixstart++) {
    world_state_vel[ixstart] = 0.0F;
    for (ix = 0; ix < 3; ix++) {
      world_state_vel[ixstart] += R_ow[ix + 3 * ixstart] * xt_robot_state_pos[ix];
    }

    world_state_IMU_gyro_bias[ixstart] = xt_robot_state_IMU_gyro_bias[ixstart];
    world_state_IMU_acc_bias[ixstart] = xt_robot_state_IMU_acc_bias[ixstart];
    world_state_IMU_pos[ixstart] = xt_robot_state_IMU_pos[ixstart];
  }

  for (ixstart = 0; ixstart < 4; ixstart++) {
    world_state_IMU_att[ixstart] = xt_robot_state_IMU_att[ixstart];
  }
}

//
// Arguments    : float varargin_1
//                float varargin_2
//                float varargin_3
// Return Type  : double
//
static double h_fprintf(float varargin_1, float varargin_2, float varargin_3)
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
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                float varargin_2
//                float varargin_3
// Return Type  : void
//
static void h_ros_info(int varargin_1, float varargin_2, float varargin_3)
{
  char cv24[51];
  int i20;
  static const char cv25[51] = { 'F', 'o', 'r', 'c', 'i', 'n', 'g', ' ', 'a',
    'c', 't', 'i', 'v', 'a', 't', 'i', 'o', 'n', ' ', 'o', 'f', ' ', 'f', 'e',
    'a', 't', 'u', 'r', 'e', ' ', '%', 'i', ' ', '(', '%', 'i', ' ', 'o', 'n',
    ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ')', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0F) {
    for (i20 = 0; i20 < 51; i20++) {
      cv24[i20] = cv25[i20];
    }

    ROS_INFO(cv24, varargin_1, varargin_2, varargin_3);
  }
}

//
// Arguments    : float varargin_1
//                float varargin_2
//                float varargin_3
// Return Type  : double
//
static double i_fprintf(float varargin_1, float varargin_2, float varargin_3)
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
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
// Return Type  : void
//
static void i_ros_info(int varargin_1)
{
  char cv26[28];
  int i22;
  static const char cv27[28] = { 'S', 'e', 't', 't', 'i', 'n', 'g', ' ', 'a',
    'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ' ', 'a', 's', ' ', 'o', 'r', 'i',
    'g', 'i', 'n', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0F) {
    for (i22 = 0; i22 < 28; i22++) {
      cv26[i22] = cv27[i22];
    }

    ROS_INFO(cv26, varargin_1);
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
// Arguments    : const float z_u_l[2]
//                const float z_u_r[2]
//                const float c_cameraparams_CameraParameters[2]
//                const float d_cameraparams_CameraParameters[2]
//                const float e_cameraparams_CameraParameters[2]
//                const float f_cameraparams_CameraParameters[2]
//                const float cameraparams_r_lr[3]
//                const float cameraparams_R_lr[9]
//                float fp[3]
//                float b_m[6]
//                boolean_T *success
// Return Type  : void
//
static void initializePoint(const float z_u_l[2], const float z_u_r[2], const
  float c_cameraparams_CameraParameters[2], const float
  d_cameraparams_CameraParameters[2], const float
  e_cameraparams_CameraParameters[2], const float
  f_cameraparams_CameraParameters[2], const float cameraparams_r_lr[3], const
  float cameraparams_R_lr[9], float fp[3], float b_m[6], boolean_T *success)
{
  float ml[3];
  float mr[3];
  float tol;
  float fcnOutput;
  float b_pos[6];
  int i;
  int i14;
  static const signed char iv0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  float rot[18];
  float A[30];
  float b[6];
  int anchorIdx;
  int rankR;
  int j;
  float b_rot[9];
  static const signed char iv1[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  int jpvt[5];
  float tau[5];
  float x[5];
  ml[0] = (z_u_l[0] - d_cameraparams_CameraParameters[0]) /
    c_cameraparams_CameraParameters[0];
  ml[1] = (z_u_l[1] - d_cameraparams_CameraParameters[1]) /
    c_cameraparams_CameraParameters[1];
  ml[2] = 1.0F;
  mr[0] = (z_u_r[0] - f_cameraparams_CameraParameters[0]) /
    e_cameraparams_CameraParameters[0];
  mr[1] = (z_u_r[1] - f_cameraparams_CameraParameters[1]) /
    e_cameraparams_CameraParameters[1];
  mr[2] = 1.0F;
  tol = norm(ml);
  fcnOutput = norm(mr);
  for (i = 0; i < 3; i++) {
    b_pos[i] = 0.0F;
    b_pos[3 + i] = cameraparams_r_lr[i];
    for (i14 = 0; i14 < 3; i14++) {
      rot[i14 + 3 * i] = iv0[i14 + 3 * i];
      rot[i14 + 3 * (i + 3)] = cameraparams_R_lr[i + 3 * i14];
    }

    b_m[i] = ml[i] / tol;
    b_m[3 + i] = mr[i] / fcnOutput;
  }

  //  normalized rays in left frame
  for (i = 0; i < 3; i++) {
    ml[i] = 0.0F;
    for (i14 = 0; i14 < 3; i14++) {
      ml[i] += cameraparams_R_lr[i + 3 * i14] * b_m[3 + i14];
    }
  }

  if (b_m[2] * ml[0] - b_m[0] * ml[2] > 0.0F) {
    *success = false;
    for (i = 0; i < 3; i++) {
      fp[i] = b_m[i];
    }
  } else {
    *success = true;

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
    memset(&A[0], 0, 30U * sizeof(float));
    for (i = 0; i < 6; i++) {
      b[i] = ((real32_T)rtNaN);
    }

    for (anchorIdx = 0; anchorIdx < 2; anchorIdx++) {
      rankR = anchorIdx * 3;
      j = anchorIdx * 3;
      for (i = 0; i < 3; i++) {
        for (i14 = 0; i14 < 3; i14++) {
          b_rot[i14 + 3 * i] = rot[i + 3 * (i14 + rankR)];
        }
      }

      for (i = 0; i < 3; i++) {
        A[(i + j) + 6 * anchorIdx] = 0.0F;
        for (i14 = 0; i14 < 3; i14++) {
          A[(i + j) + 6 * anchorIdx] += b_rot[i + 3 * i14] * b_m[i14 + 3 *
            anchorIdx];
        }
      }

      rankR = anchorIdx * 3;
      j = anchorIdx * 3;
      for (i = 0; i < 3; i++) {
        for (i14 = 0; i14 < 3; i14++) {
          A[(i14 + rankR) + 6 * (2 + i)] = iv1[i14 + 3 * i];
        }

        b[i + j] = -b_pos[i + 3 * anchorIdx];
      }
    }

    //  condition = cond(A);
    b_xgeqp3(A, tau, jpvt);
    rankR = 0;
    tol = 6.0F * (float)fabs((double)A[0]) * 1.1920929E-7F;
    while ((rankR < 5) && ((float)fabs((double)A[rankR + 6 * rankR]) >= tol)) {
      rankR++;
    }

    for (j = 0; j < 5; j++) {
      x[j] = 0.0F;
      if (tau[j] != 0.0F) {
        tol = b[j];
        for (i = j + 1; i + 1 < 7; i++) {
          tol += A[i + 6 * j] * b[i];
        }

        tol *= tau[j];
        if (tol != 0.0F) {
          b[j] -= tol;
          for (i = j + 1; i + 1 < 7; i++) {
            b[i] -= A[i + 6 * j] * tol;
          }
        }
      }
    }

    for (i = 0; i + 1 <= rankR; i++) {
      x[jpvt[i] - 1] = b[i];
    }

    for (j = rankR - 1; j + 1 > 0; j--) {
      x[jpvt[j] - 1] /= A[j + 6 * j];
      for (i = 0; i + 1 <= j; i++) {
        x[jpvt[i] - 1] -= x[jpvt[j] - 1] * A[i + 6 * j];
      }
    }

    for (i = 0; i < 3; i++) {
      fp[i] = x[i + 2];
    }

    //  is this better?
    //  A = [cameraparams.R_rl * ml, -mr];
    //  b = cameraparams.r_lr;
    //  x = A\b;
    //  fp = ml*x(1);
  }
}

//
// Arguments    : float varargin_1
// Return Type  : double
//
static double j_fprintf(float varargin_1)
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
// Arguments    : float varargin_1
// Return Type  : double
//
static double k_fprintf(float varargin_1)
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
// Arguments    : const emxArray_real32_T *A
//                emxArray_real32_T *B
// Return Type  : void
//
static void lusolve(const emxArray_real32_T *A, emxArray_real32_T *B)
{
  emxArray_real32_T *b_A;
  int n;
  int i28;
  int iy;
  emxArray_int32_T *ipiv;
  int j;
  int mmj;
  int c;
  int ix;
  float smax;
  int k;
  float s;
  int i29;
  int i;
  int jAcol;
  int jA;
  emxInit_real32_T1(&b_A, 2);
  n = A->size[1];
  i28 = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  emxEnsureCapacity((emxArray__common *)b_A, i28, (int)sizeof(float));
  iy = A->size[0] * A->size[1];
  for (i28 = 0; i28 < iy; i28++) {
    b_A->data[i28] = A->data[i28];
  }

  emxInit_int32_T1(&ipiv, 2);
  iy = A->size[1];
  eml_signed_integer_colon(iy, ipiv);
  if (A->size[1] < 1) {
  } else {
    if (A->size[1] - 1 <= A->size[1]) {
      i28 = A->size[1] - 1;
    } else {
      i28 = A->size[1];
    }

    for (j = 0; j + 1 <= i28; j++) {
      mmj = n - j;
      c = j * (n + 1);
      if (mmj < 1) {
        iy = -1;
      } else {
        iy = 0;
        if (mmj > 1) {
          ix = c;
          smax = (float)fabs((double)b_A->data[c]);
          for (k = 1; k + 1 <= mmj; k++) {
            ix++;
            s = (float)fabs((double)b_A->data[ix]);
            if (s > smax) {
              iy = k;
              smax = s;
            }
          }
        }
      }

      if (b_A->data[c + iy] != 0.0F) {
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

        i29 = c + mmj;
        for (i = c + 1; i + 1 <= i29; i++) {
          b_A->data[i] /= b_A->data[c];
        }
      }

      jAcol = (n - j) - 1;
      jA = c + n;
      iy = c + n;
      for (i = 1; i <= jAcol; i++) {
        smax = b_A->data[iy];
        if (b_A->data[iy] != 0.0F) {
          ix = c + 1;
          i29 = mmj + jA;
          for (k = 1 + jA; k + 1 <= i29; k++) {
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
        if (b_A->data[k + jAcol] != 0.0F) {
          for (i = 0; i < 91; i++) {
            B->data[i + iy] -= b_A->data[k + jAcol] * B->data[i + jA];
          }
        }
      }

      smax = 1.0F / b_A->data[j + jAcol];
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
        if (b_A->data[k + jAcol] != 0.0F) {
          for (i = 0; i < 91; i++) {
            B->data[i + iy] -= b_A->data[k + jAcol] * B->data[i + jA];
          }
        }
      }
    }
  }

  emxFree_real32_T(&b_A);
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
// Arguments    : const float x_data[]
//                const int x_size[1]
// Return Type  : float
//
static float median(const float x_data[], const int x_size[1])
{
  float y;
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
    y = ((real32_T)rtNaN);
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
      if ((x_data[k - 1] <= x_data[k]) || rtIsNaNF(x_data[k])) {
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
              rtIsNaNF(x_data[idx_data[q] - 1])) {
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

    if (rtIsNaNF(x_data[idx_data[(signed char)x_size[0] - 1] - 1])) {
      y = x_data[idx_data[(signed char)x_size[0] - 1] - 1];
    } else if (midm1 << 1 == x_size[0]) {
      if (((x_data[idx_data[midm1 - 1] - 1] < 0.0F) && (x_data[idx_data[midm1] -
            1] >= 0.0F)) || rtIsInfF(x_data[idx_data[midm1 - 1] - 1]) ||
          rtIsInfF(x_data[idx_data[midm1] - 1])) {
        y = (x_data[idx_data[midm1 - 1] - 1] + x_data[idx_data[midm1] - 1]) /
          2.0F;
      } else {
        y = x_data[idx_data[midm1 - 1] - 1] + (x_data[idx_data[midm1] - 1] -
          x_data[idx_data[midm1 - 1] - 1]) / 2.0F;
      }
    } else {
      y = x_data[idx_data[midm1] - 1];
    }
  }

  return y;
}

//
// Arguments    : int idx[8]
//                float x[8]
//                int offset
//                int np
//                int nq
//                int iwork[8]
//                float xwork[8]
// Return Type  : void
//
static void merge(int idx[8], float x[8], int offset, int np, int nq, int iwork
                  [8], float xwork[8])
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
//                emxArray_real32_T *x
//                int offset
//                int n
//                int preSortLevel
//                emxArray_int32_T *iwork
//                emxArray_real32_T *xwork
// Return Type  : void
//
static void merge_block(emxArray_int32_T *idx, emxArray_real32_T *x, int offset,
  int n, int preSortLevel, emxArray_int32_T *iwork, emxArray_real32_T *xwork)
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
// Arguments    : int a
//                int b
// Return Type  : int
//
static int mul_s32_s32_s32_sat(int a, int b)
{
  int result;
  unsigned int u32_clo;
  unsigned int u32_chi;
  mul_wide_s32(a, b, &u32_chi, &u32_clo);
  if (((int)u32_chi > 0) || ((u32_chi == 0U) && (u32_clo >= 2147483648U))) {
    result = MAX_int32_T;
  } else if (((int)u32_chi < -1) || (((int)u32_chi == -1) && (u32_clo <
               2147483648U))) {
    result = MIN_int32_T;
  } else {
    result = (int)u32_clo;
  }

  return result;
}

//
// Arguments    : int in0
//                int in1
//                unsigned int *ptrOutBitsHi
//                unsigned int *ptrOutBitsLo
// Return Type  : void
//
static void mul_wide_s32(int in0, int in1, unsigned int *ptrOutBitsHi, unsigned
  int *ptrOutBitsLo)
{
  unsigned int absIn0;
  unsigned int absIn1;
  int negativeProduct;
  int in0Hi;
  int in0Lo;
  int in1Hi;
  int in1Lo;
  unsigned int productLoHi;
  unsigned int productLoLo;
  unsigned int outBitsLo;
  if (in0 < 0) {
    absIn0 = (unsigned int)-in0;
  } else {
    absIn0 = (unsigned int)in0;
  }

  if (in1 < 0) {
    absIn1 = (unsigned int)-in1;
  } else {
    absIn1 = (unsigned int)in1;
  }

  negativeProduct = !((in0 == 0) || ((in1 == 0) || ((in0 > 0) == (in1 > 0))));
  in0Hi = (int)(absIn0 >> 16U);
  in0Lo = (int)(absIn0 & 65535U);
  in1Hi = (int)(absIn1 >> 16U);
  in1Lo = (int)(absIn1 & 65535U);
  absIn0 = (unsigned int)in0Hi * in1Hi;
  absIn1 = (unsigned int)in0Hi * in1Lo;
  productLoHi = (unsigned int)in0Lo * in1Hi;
  productLoLo = (unsigned int)in0Lo * in1Lo;
  in0Hi = 0;
  outBitsLo = productLoLo + (productLoHi << 16U);
  if (outBitsLo < productLoLo) {
    in0Hi = 1;
  }

  productLoLo = outBitsLo;
  outBitsLo += absIn1 << 16U;
  if (outBitsLo < productLoLo) {
    in0Hi++;
  }

  absIn0 = ((in0Hi + absIn0) + (productLoHi >> 16U)) + (absIn1 >> 16U);
  if (negativeProduct) {
    absIn0 = ~absIn0;
    outBitsLo = ~outBitsLo;
    outBitsLo++;
    if (outBitsLo == 0U) {
      absIn0++;
    }
  }

  *ptrOutBitsHi = absIn0;
  *ptrOutBitsLo = outBitsLo;
}

//
// # coder
// multiplyIdx Create a longer index array from idx
//    for example:
//    idx_mult = multiplyIdx([1 4 5], 2)
//    idx_mult = [1 2 7 8 9 10]
// Arguments    : const float idx_data[]
//                const int idx_size[1]
//                float idx_mult_data[]
//                int idx_mult_size[1]
// Return Type  : void
//
static void multiplyIdx(const float idx_data[], const int idx_size[1], float
  idx_mult_data[], int idx_mult_size[1])
{
  int c;
  int i8;
  int i;
  int j;
  c = idx_size[0] << 1;
  idx_mult_size[0] = c;
  for (i8 = 0; i8 < c; i8++) {
    idx_mult_data[i8] = 0.0F;
  }

  for (i = 0; i < idx_size[0]; i++) {
    for (j = 0; j < 2; j++) {
      idx_mult_data[(i << 1) + j] = (idx_data[i] - 1.0F) * 2.0F + (1.0F + (float)
        j);
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
// Arguments    : const float x[3]
// Return Type  : float
//
static float norm(const float x[3])
{
  float y;
  float scale;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  scale = 1.17549435E-38F;
  for (k = 0; k < 3; k++) {
    absxk = (float)fabs((double)x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * (float)sqrt((double)y);
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
  int i2;
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
    for (i2 = 0; i2 < nbytesint; i2++) {
      varargout_1_data[i2] = varargin_1_data[i2];
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
  int i3;
  char varargout_1_data[6];
  static const char cfmt[19] = { 'f', 'i', 'x', 'e', 'd', '_', 'f', 'e', 'a',
    't', 'u', 'r', 'e', ':', ' ', '%', 's', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = varargin_1_size[1];
    for (i3 = 0; i3 < nbytesint; i3++) {
      varargout_1_data[i3] = varargin_1_data[i3];
    }

    varargout_1_data[varargin_1_size[1]] = '\x00';
    nbytesint = fprintf(filestar, cfmt, &varargout_1_data[0]);
    fflush(filestar);
  }

  return nbytesint;
}

//
// predictMeasurementStereo Predict the measurement of a feature given in the left
// camera frame
//    Get the pixel undistorted coordinates where a feature given in the left camera
//    frame would be visible in both cameras
// Arguments    : const float fp_l[3]
//                const float c_stereoParams_CameraParameters[2]
//                const float d_stereoParams_CameraParameters[2]
//                const float e_stereoParams_CameraParameters[2]
//                const float f_stereoParams_CameraParameters[2]
//                const float stereoParams_r_lr[3]
//                const float stereoParams_R_rl[9]
//                float h_cin_l[2]
//                float h_cin_r[2]
// Return Type  : void
//
static void predictMeasurementStereo(const float fp_l[3], const float
  c_stereoParams_CameraParameters[2], const float
  d_stereoParams_CameraParameters[2], const float
  e_stereoParams_CameraParameters[2], const float
  f_stereoParams_CameraParameters[2], const float stereoParams_r_lr[3], const
  float stereoParams_R_rl[9], float h_cin_l[2], float h_cin_r[2])
{
  float h_c_n_l[2];
  int i;
  float fp_r[3];
  float f1;
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
    f1 = 0.0F;
    for (i15 = 0; i15 < 3; i15++) {
      f1 += stereoParams_R_rl[i + 3 * i15] * fp_l[i15];
    }

    fp_r[i] = f1 - stereoParams_r_lr[i];
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
// print all parameters for debug check
// Arguments    : float c_noiseParameters_process_noise
//                float d_noiseParameters_process_noise
//                float e_noiseParameters_process_noise
//                float f_noiseParameters_process_noise
//                float g_noiseParameters_process_noise
//                const float c_noiseParameters_gyro_bias_ini[3]
//                const float c_noiseParameters_acc_bias_init[3]
//                float noiseParameters_image_noise
//                float c_noiseParameters_inv_depth_ini
//                int c_VIOParameters_num_points_per_
//                int VIOParameters_num_anchors
//                int c_VIOParameters_max_ekf_iterati
//                boolean_T VIOParameters_fixed_feature
//                boolean_T c_VIOParameters_delayed_initial
//                boolean_T VIOParameters_mono
//                boolean_T VIOParameters_RANSAC
// Return Type  : void
//
static void printParams(float c_noiseParameters_process_noise, float
  d_noiseParameters_process_noise, float e_noiseParameters_process_noise, float
  f_noiseParameters_process_noise, float g_noiseParameters_process_noise, const
  float c_noiseParameters_gyro_bias_ini[3], const float
  c_noiseParameters_acc_bias_init[3], float noiseParameters_image_noise, float
  c_noiseParameters_inv_depth_ini, int c_VIOParameters_num_points_per_, int
  VIOParameters_num_anchors, int c_VIOParameters_max_ekf_iterati, boolean_T
  VIOParameters_fixed_feature, boolean_T c_VIOParameters_delayed_initial,
  boolean_T VIOParameters_mono, boolean_T VIOParameters_RANSAC)
{
  char cv0[18];
  int i1;
  static const char cv1[18] = { 'N', 'o', 'i', 's', 'e', ' ', 'p', 'a', 'r', 'a',
    'm', 'e', 't', 'e', 'r', 's', ':', '\x00' };

  char cv2[15];
  static const char cv3[15] = { 'V', 'I', 'O', ' ', 'p', 'a', 'r', 'a', 'm', 'e',
    't', 'e', 'r', 's', '\x00' };

  int s_size[2];
  static const char cv4[4] = { 'T', 'r', 'u', 'e' };

  char s_data[5];
  static const char cv5[5] = { 'F', 'a', 'l', 's', 'e' };

  b_fprintf();

  // #coder
  // ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0F) {
    for (i1 = 0; i1 < 18; i1++) {
      cv0[i1] = cv1[i1];
    }

    ROS_INFO(cv0);
  }

  c_fprintf(c_noiseParameters_process_noise);
  d_fprintf(d_noiseParameters_process_noise);
  e_fprintf(e_noiseParameters_process_noise);
  f_fprintf(f_noiseParameters_process_noise);
  g_fprintf(g_noiseParameters_process_noise);
  h_fprintf(c_noiseParameters_gyro_bias_ini[0], c_noiseParameters_gyro_bias_ini
            [1], c_noiseParameters_gyro_bias_ini[2]);
  i_fprintf(c_noiseParameters_acc_bias_init[0], c_noiseParameters_acc_bias_init
            [1], c_noiseParameters_acc_bias_init[2]);
  j_fprintf(noiseParameters_image_noise);
  k_fprintf(c_noiseParameters_inv_depth_ini);
  b_fprintf();

  // #coder
  // ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0F) {
    for (i1 = 0; i1 < 15; i1++) {
      cv2[i1] = cv3[i1];
    }

    ROS_INFO(cv2);
  }

  l_fprintf(VIOParameters_num_anchors);
  m_fprintf(c_VIOParameters_num_points_per_);
  n_fprintf(c_VIOParameters_max_ekf_iterati);
  if (c_VIOParameters_delayed_initial) {
    s_size[0] = 1;
    s_size[1] = 4;
    for (i1 = 0; i1 < 4; i1++) {
      s_data[i1] = cv4[i1];
    }
  } else {
    s_size[0] = 1;
    s_size[1] = 5;
    for (i1 = 0; i1 < 5; i1++) {
      s_data[i1] = cv5[i1];
    }
  }

  o_fprintf(s_data, s_size);
  if (VIOParameters_fixed_feature) {
    s_size[0] = 1;
    s_size[1] = 4;
    for (i1 = 0; i1 < 4; i1++) {
      s_data[i1] = cv4[i1];
    }
  } else {
    s_size[0] = 1;
    s_size[1] = 5;
    for (i1 = 0; i1 < 5; i1++) {
      s_data[i1] = cv5[i1];
    }
  }

  p_fprintf(s_data, s_size);
  if (VIOParameters_mono) {
    s_size[0] = 1;
    s_size[1] = 4;
    for (i1 = 0; i1 < 4; i1++) {
      s_data[i1] = cv4[i1];
    }
  } else {
    s_size[0] = 1;
    s_size[1] = 5;
    for (i1 = 0; i1 < 5; i1++) {
      s_data[i1] = cv5[i1];
    }
  }

  q_fprintf(s_data, s_size);
  if (VIOParameters_RANSAC) {
    s_size[0] = 1;
    s_size[1] = 4;
    for (i1 = 0; i1 < 4; i1++) {
      s_data[i1] = cv4[i1];
    }
  } else {
    s_size[0] = 1;
    s_size[1] = 5;
    for (i1 = 0; i1 < 5; i1++) {
      s_data[i1] = cv5[i1];
    }
  }

  r_fprintf(s_data, s_size);
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
  int i4;
  char varargout_1_data[6];
  static const char cfmt[10] = { 'm', 'o', 'n', 'o', ':', ' ', '%', 's', '\x0a',
    '\x00' };

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
// Arguments    : const float q[4]
//                const float p[4]
//                float qp[4]
// Return Type  : void
//
static void quatmultJ(const float q[4], const float p[4], float qp[4])
{
  float b_p[16];
  float b_q[4];
  int i6;
  int i7;
  float fcnOutput;
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
  for (i6 = 0; i6 < 4; i6++) {
    qp[i6] = 0.0F;
    for (i7 = 0; i7 < 4; i7++) {
      qp[i6] += b_p[i6 + (i7 << 2)] * b_q[i7];
    }
  }

  fcnOutput = b_norm(qp);
  for (i6 = 0; i6 < 4; i6++) {
    qp[i6] /= fcnOutput;
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
  int i5;
  char varargout_1_data[6];
  static const char cfmt[12] = { 'R', 'A', 'N', 'S', 'A', 'C', ':', ' ', '%',
    's', '\x0a', '\x00' };

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
// Arguments    : const float x[3]
//                float y
//                float z[3]
// Return Type  : void
//
static void rdivide(const float x[3], float y, float z[3])
{
  int i;
  for (i = 0; i < 3; i++) {
    z[i] = x[i] / y;
  }
}

//
// #coder
// ROS_ERROR Print to ROS_ERROR in ROS or to console in Matlab
// Arguments    : void
// Return Type  : void
//
static void ros_error()
{
  char cv8[27];
  int i10;
  static const char cv9[27] = { 'p', 'i', 'c', 'k', 'e', 'd', ' ', 'a', 'n', ' ',
    'i', 'n', 'a', 'c', 't', 'i', 'v', 'e', ' ', 'f', 'e', 'a', 't', 'u', 'r',
    'e', '\x00' };

  for (i10 = 0; i10 < 27; i10++) {
    cv8[i10] = cv9[i10];
  }

  ROS_ERROR(cv8);
}

//
// #coder
// ROS_INFO Print to ROS_INFO in ROS or to console in Matlab
// Arguments    : int varargin_1
//                int varargin_2
//                int varargin_3
// Return Type  : void
//
static void ros_info(int varargin_1, int varargin_2, int varargin_3)
{
  char cv6[54];
  int i9;
  static const char cv7[54] = { 'F', 'i', 'x', 'e', 'd', ' ', 'f', 'e', 'a', 't',
    'u', 'r', 'e', ' ', '%', 'i', ' ', '(', '%', 'i', ' ', 'o', 'n', ' ', 'a',
    'n', 'c', 'h', 'o', 'r', ' ', '%', 'i', ')', ' ', 'i', 's', ' ', 'n', 'o',
    ' ', 'l', 'o', 'n', 'g', 'e', 'r', ' ', 'v', 'a', 'l', 'i', 'd', '\x00' };

  //  debug_level == 0: print errors, == 1: print warnings, == 2: print info
  if (debug_level >= 2.0F) {
    for (i9 = 0; i9 < 54; i9++) {
      cv6[i9] = cv7[i9];
    }

    ROS_INFO(cv6, varargin_1, varargin_2, varargin_3);
  }
}

//
// Arguments    : float u0
//                float u1
// Return Type  : float
//
static float rt_hypotf_snf(float u0, float u1)
{
  float y;
  float a;
  float b;
  a = (float)fabs((double)u0);
  b = (float)fabs((double)u1);
  if (a < b) {
    a /= b;
    y = b * (float)sqrt((double)(a * a + 1.0F));
  } else if (a > b) {
    b /= a;
    y = a * (float)sqrt((double)(b * b + 1.0F));
  } else if (rtIsNaNF(b)) {
    y = b;
  } else {
    y = a * 1.41421354F;
  }

  return y;
}

//
// Arguments    : float u0
//                float u1
// Return Type  : float
//
static float rt_powf_snf(float u0, float u1)
{
  float y;
  float f2;
  float f3;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = ((real32_T)rtNaN);
  } else {
    f2 = (float)fabs((double)u0);
    f3 = (float)fabs((double)u1);
    if (rtIsInfF(u1)) {
      if (f2 == 1.0F) {
        y = ((real32_T)rtNaN);
      } else if (f2 > 1.0F) {
        if (u1 > 0.0F) {
          y = ((real32_T)rtInf);
        } else {
          y = 0.0F;
        }
      } else if (u1 > 0.0F) {
        y = 0.0F;
      } else {
        y = ((real32_T)rtInf);
      }
    } else if (f3 == 0.0F) {
      y = 1.0F;
    } else if (f3 == 1.0F) {
      if (u1 > 0.0F) {
        y = u0;
      } else {
        y = 1.0F / u0;
      }
    } else if (u1 == 2.0F) {
      y = u0 * u0;
    } else if ((u1 == 0.5F) && (u0 >= 0.0F)) {
      y = (float)sqrt((double)u0);
    } else if ((u0 < 0.0F) && (u1 > (float)floor((double)u1))) {
      y = ((real32_T)rtNaN);
    } else {
      y = (float)pow((double)u0, (double)u1);
    }
  }

  return y;
}

//
// Arguments    : float u
// Return Type  : float
//
static float rt_roundf_snf(float u)
{
  float y;
  if ((float)fabs((double)u) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = (float)floor((double)(u + 0.5F));
    } else if (u > -0.5F) {
      y = u * 0.0F;
    } else {
      y = (float)ceil((double)(u - 0.5F));
    }
  } else {
    y = u;
  }

  return y;
}

//
// Arguments    : float x[8]
//                int idx[8]
// Return Type  : void
//
static void sort(float x[8], int idx[8])
{
  b_sort(x, idx);
}

//
// Arguments    : emxArray_real32_T *x
//                emxArray_int32_T *idx
// Return Type  : void
//
static void sortIdx(emxArray_real32_T *x, emxArray_int32_T *idx)
{
  emxArray_real32_T *b_x;
  int ib;
  int wOffset;
  int b_m;
  int n;
  float x4[4];
  int idx4[4];
  emxArray_int32_T *iwork;
  emxArray_real32_T *xwork;
  int nNaNs;
  int k;
  int i3;
  int i4;
  signed char perm[4];
  int nNonNaN;
  int nBlocks;
  int b_iwork[256];
  float b_xwork[256];
  int bLen2;
  int nPairs;
  int exitg1;
  emxInit_real32_T(&b_x, 1);
  ib = x->size[0];
  wOffset = b_x->size[0];
  b_x->size[0] = x->size[0];
  emxEnsureCapacity((emxArray__common *)b_x, wOffset, (int)sizeof(float));
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
    x4[b_m] = 0.0F;
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

  emxInit_real32_T(&xwork, 1);
  b_m = x->size[0];
  wOffset = xwork->size[0];
  xwork->size[0] = b_m;
  emxEnsureCapacity((emxArray__common *)xwork, wOffset, (int)sizeof(float));
  b_m = xwork->size[0];
  wOffset = xwork->size[0];
  xwork->size[0] = b_m;
  emxEnsureCapacity((emxArray__common *)xwork, wOffset, (int)sizeof(float));
  for (wOffset = 0; wOffset < b_m; wOffset++) {
    xwork->data[wOffset] = 0.0F;
  }

  nNaNs = 1;
  ib = 0;
  for (k = 0; k + 1 <= n; k++) {
    if (rtIsNaNF(b_x->data[k])) {
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

  emxFree_real32_T(&xwork);
  emxFree_int32_T(&iwork);
  wOffset = x->size[0];
  x->size[0] = b_x->size[0];
  emxEnsureCapacity((emxArray__common *)x, wOffset, (int)sizeof(float));
  b_m = b_x->size[0];
  for (wOffset = 0; wOffset < b_m; wOffset++) {
    x->data[wOffset] = b_x->data[wOffset];
  }

  emxFree_real32_T(&b_x);
}

//
// Arguments    : emxArray_real32_T *A
//                emxArray_real32_T *tau
//                emxArray_int32_T *jpvt
// Return Type  : void
//
static void xgeqp3(emxArray_real32_T *A, emxArray_real32_T *tau,
                   emxArray_int32_T *jpvt)
{
  int b_m;
  int n;
  int mn;
  int i30;
  emxArray_real32_T *work;
  int itemp;
  emxArray_real32_T *vn1;
  emxArray_real32_T *vn2;
  int k;
  int nmi;
  int i;
  int i_i;
  int mmi;
  int ix;
  float smax;
  float s;
  int pvt;
  int iy;
  float absxk;
  int i_ip1;
  int lastv;
  int lastc;
  boolean_T exitg2;
  int exitg1;
  float t;
  b_m = A->size[0];
  n = A->size[1];
  if (A->size[0] <= A->size[1]) {
    mn = A->size[0];
  } else {
    mn = A->size[1];
  }

  i30 = tau->size[0];
  tau->size[0] = mn;
  emxEnsureCapacity((emxArray__common *)tau, i30, (int)sizeof(float));
  eml_signed_integer_colon(A->size[1], jpvt);
  if ((A->size[0] == 0) || (A->size[1] == 0)) {
  } else {
    emxInit_real32_T(&work, 1);
    itemp = A->size[1];
    i30 = work->size[0];
    work->size[0] = itemp;
    emxEnsureCapacity((emxArray__common *)work, i30, (int)sizeof(float));
    for (i30 = 0; i30 < itemp; i30++) {
      work->data[i30] = 0.0F;
    }

    emxInit_real32_T(&vn1, 1);
    emxInit_real32_T(&vn2, 1);
    itemp = A->size[1];
    i30 = vn1->size[0];
    vn1->size[0] = itemp;
    emxEnsureCapacity((emxArray__common *)vn1, i30, (int)sizeof(float));
    i30 = vn2->size[0];
    vn2->size[0] = vn1->size[0];
    emxEnsureCapacity((emxArray__common *)vn2, i30, (int)sizeof(float));
    k = 1;
    for (nmi = 0; nmi + 1 <= n; nmi++) {
      vn1->data[nmi] = xnrm2(b_m, A, k);
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
          smax = (float)fabs((double)vn1->data[i]);
          for (k = 0; k + 2 <= nmi; k++) {
            ix++;
            s = (float)fabs((double)vn1->data[ix]);
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
        s = 0.0F;
        if (1 + mmi <= 0) {
        } else {
          smax = b_xnrm2(mmi, A, i_i + 2);
          if (smax != 0.0F) {
            smax = rt_hypotf_snf(A->data[i_i], smax);
            if (A->data[i_i] >= 0.0F) {
              smax = -smax;
            }

            if ((float)fabs((double)smax) < 9.86076132E-32F) {
              pvt = 0;
              do {
                pvt++;
                xscal(mmi, 1.01412048E+31F, A, i_i + 2);
                smax *= 1.01412048E+31F;
                absxk *= 1.01412048E+31F;
              } while (!((float)fabs((double)smax) >= 9.86076132E-32F));

              smax = b_xnrm2(mmi, A, i_i + 2);
              smax = rt_hypotf_snf(absxk, smax);
              if (absxk >= 0.0F) {
                smax = -smax;
              }

              s = (smax - absxk) / smax;
              xscal(mmi, 1.0F / (absxk - smax), A, i_i + 2);
              for (k = 1; k <= pvt; k++) {
                smax *= 9.86076132E-32F;
              }

              absxk = smax;
            } else {
              s = (smax - A->data[i_i]) / smax;
              xscal(mmi, 1.0F / (A->data[i_i] - smax), A, i_i + 2);
              absxk = smax;
            }
          }
        }

        tau->data[i] = s;
        A->data[i_i] = absxk;
      } else {
        tau->data[i] = 0.0F;
      }

      if (i + 1 < n) {
        absxk = A->data[i_i];
        A->data[i_i] = 1.0F;
        i_ip1 = (i + (i + 1) * b_m) + 1;
        if (tau->data[i] != 0.0F) {
          lastv = mmi;
          itemp = i_i + mmi;
          while ((lastv + 1 > 0) && (A->data[itemp] == 0.0F)) {
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
                if (A->data[k - 1] != 0.0F) {
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
              work->data[iy - 1] = 0.0F;
            }

            iy = 0;
            i30 = i_ip1 + b_m * (lastc - 1);
            itemp = i_ip1;
            while ((b_m > 0) && (itemp <= i30)) {
              ix = i_i;
              smax = 0.0F;
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

          if (-tau->data[i] == 0.0F) {
          } else {
            itemp = 0;
            for (nmi = 1; nmi <= lastc; nmi++) {
              if (work->data[itemp] != 0.0F) {
                smax = work->data[itemp] * -tau->data[i];
                ix = i_i;
                i30 = lastv + i_ip1;
                for (pvt = i_ip1; pvt <= i30; pvt++) {
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
        if (vn1->data[nmi] != 0.0F) {
          smax = (float)fabs((double)A->data[i + A->size[0] * nmi]) / vn1->
            data[nmi];
          smax = 1.0F - smax * smax;
          if (smax < 0.0F) {
            smax = 0.0F;
          }

          s = vn1->data[nmi] / vn2->data[nmi];
          s = smax * (s * s);
          if (s <= 0.000345266977F) {
            if (i + 1 < b_m) {
              smax = 0.0F;
              if (mmi < 1) {
              } else if (mmi == 1) {
                smax = (float)fabs((double)A->data[itemp]);
              } else {
                s = 1.17549435E-38F;
                pvt = itemp + mmi;
                while (itemp + 1 <= pvt) {
                  absxk = (float)fabs((double)A->data[itemp]);
                  if (absxk > s) {
                    t = s / absxk;
                    smax = 1.0F + smax * t * t;
                    s = absxk;
                  } else {
                    t = absxk / s;
                    smax += t * t;
                  }

                  itemp++;
                }

                smax = s * (float)sqrt((double)smax);
              }

              vn1->data[nmi] = smax;
              vn2->data[nmi] = vn1->data[nmi];
            } else {
              vn1->data[nmi] = 0.0F;
              vn2->data[nmi] = 0.0F;
            }
          } else {
            vn1->data[nmi] *= (float)sqrt((double)smax);
          }
        }
      }
    }

    emxFree_real32_T(&vn2);
    emxFree_real32_T(&vn1);
    emxFree_real32_T(&work);
  }
}

//
// Arguments    : int n
//                const emxArray_real32_T *x
//                int ix0
// Return Type  : float
//
static float xnrm2(int n, const emxArray_real32_T *x, int ix0)
{
  float y;
  float scale;
  int kend;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  if (n < 1) {
  } else if (n == 1) {
    y = (float)fabs((double)x->data[ix0 - 1]);
  } else {
    scale = 1.17549435E-38F;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = (float)fabs((double)x->data[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0F + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * (float)sqrt((double)y);
  }

  return y;
}

//
// Arguments    : int n
//                float a
//                emxArray_real32_T *x
//                int ix0
// Return Type  : void
//
static void xscal(int n, float a, emxArray_real32_T *x, int ix0)
{
  int i31;
  int k;
  i31 = (ix0 + n) - 1;
  for (k = ix0; k <= i31; k++) {
    x->data[k - 1] *= a;
  }
}

//
// NOTE: Comment this out for MEXing
//  input
// Arguments    : int updateVect[40]
//                const float z_all_l[80]
//                const float z_all_r[80]
//                float dt
//                const VIOMeasurements *measurements
//                const DUOParameters *cameraParameters
//                const NoiseParameters *noiseParameters
//                const VIOParameters *b_VIOParameters
//                boolean_T vision
//                RobotState *xt_out
//                float h_u_out[160]
//                float map_out[120]
//                AnchorPose anchor_poses_out[5]
//                float delayedStatus_out[40]
// Return Type  : void
//
void SLAM(int updateVect[40], const float z_all_l[80], const float z_all_r[80],
          float dt, const VIOMeasurements *measurements, const DUOParameters
          *cameraParameters, const NoiseParameters *noiseParameters, const
          VIOParameters *b_VIOParameters, boolean_T vision, RobotState *xt_out,
          float h_u_out[160], float map_out[120], AnchorPose anchor_poses_out[5],
          float delayedStatus_out[40])
{
  int ixstart;
  float varargin_1[4];
  float mtmp;
  int itmp;
  int ix;
  boolean_T exitg2;
  static const signed char iv2[4] = { 0, 0, 0, 1 };

  static const f_struct_T rv1[5] = { { { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F,
        1.0F }, { 0, 0, 0, 0, 0, 0 }, { { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F,
            0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F,
            0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F
          }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0,
          0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, {
          0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, {
            0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F,
            0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 } } }, { { 0.0F, 0.0F,
        0.0F }, { 0.0F, 0.0F, 0.0F, 1.0F }, { 0, 0, 0, 0, 0, 0 }, { { 0.0F, {
            0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F,
            0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F,
            0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F
          }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, {
            0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F,
            0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F,
            0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F
          }, 0, 0, 0 } } }, { { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F, 1.0F },
        { 0, 0, 0, 0, 0, 0 }, { { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F
          }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0,
          0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, {
          0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, {
            0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F,
            0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F,
            0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F
          }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 } } }, { { 0.0F, 0.0F, 0.0F }, { 0.0F,
        0.0F, 0.0F, 1.0F }, { 0, 0, 0, 0, 0, 0 }, { { 0.0F, { 0.0F, 0.0F, 0.0F },
          { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F,
            0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F,
            0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F
          }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0,
          0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, {
          0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, {
            0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 } } }, { { 0.0F,
        0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F, 1.0F }, { 0, 0, 0, 0, 0, 0 }, { { 0.0F,
          { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F,
            0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F,
            0.0F }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F
          }, { 0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, {
            0.0F, 0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F,
            0.0F, 0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F,
            0.0F }, 0, 0, 0 }, { 0.0F, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 0.0F
          }, 0, 0, 0 } } } };

  int anchorIdx;
  float t_ci[3];
  float w_imu[3];
  static const float fv3[3] = { 0.0F, 0.0F, 1.0F };

  float w_c[3];
  float a_c[3];
  float fv4[9];
  float R_ci[9];
  float R_cw[9];
  boolean_T exitg1;
  float fv5[9];
  float fv6[9];
  static const signed char iv3[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 0 };

  float b_R_ci[9];
  float R[9];
  float b_R[9];
  float b_measurements[3];
  float v[15];
  float d[225];
  float grav_origin[3];
  float c[3];
  float b_c[3];
  float c_c[3];
  static const float b_b[3] = { 0.0F, 0.0F, 9.81F };

  float G[315];
  static const signed char iv4[45] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0 };

  static const signed char iv5[45] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0 };

  static const signed char iv6[45] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0 };

  static const signed char iv7[45] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    1, 0, 0, 0, 1 };

  float fv7[9];
  float fv8[9];
  float fv9[9];
  float fv10[9];
  float fv11[9];
  float fv12[9];
  float fv13[9];
  float Phi[441];
  static const signed char iv8[63] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv9[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  float b_Phi[441];
  static const signed char iv10[441] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  float b_G[315];
  float c_G[441];
  float P_xx_apr[441];
  float P_xs_apr[1470];
  float theta;
  float dq[4];
  float b_z_all_l[80];
  float b_z_all_r[80];
  float t0_IMU_att[4];
  float t0_IMU_pos[3];
  struct_T rv2[5];

  //  TODO
  //  coder.cstructname(ControllerGains, 'ControllerGains', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  //  coder.cstructname(ref, 'ReferenceCommand', 'extern', 'HeaderFile', '../InterfaceStructs.h'); 
  if (!initialized_not_empty) {
    for (ixstart = 0; ixstart < 40; ixstart++) {
      updateVect[ixstart] = 0;
    }

    for (ixstart = 0; ixstart < 3; ixstart++) {
      xt.robot_state.IMU.pos[ixstart] = cameraParameters->t_ci[ixstart];
    }

    //  THIS IS OK, It is according to the NASA memo found
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
    // % speed optimization
    varargin_1[0] = (1.0F + cameraParameters->R_ci[0]) - (cameraParameters->
      R_ci[4] + cameraParameters->R_ci[8]);
    varargin_1[1] = (1.0F + cameraParameters->R_ci[4]) - (cameraParameters->
      R_ci[0] + cameraParameters->R_ci[8]);
    varargin_1[2] = (1.0F + cameraParameters->R_ci[8]) - (cameraParameters->
      R_ci[0] + cameraParameters->R_ci[4]);
    varargin_1[3] = 1.0F + ((cameraParameters->R_ci[0] + cameraParameters->R_ci
      [4]) + cameraParameters->R_ci[8]);
    ixstart = 1;
    mtmp = varargin_1[0];
    itmp = 1;
    if (rtIsNaNF(varargin_1[0])) {
      ix = 2;
      exitg2 = false;
      while ((!exitg2) && (ix < 5)) {
        ixstart = ix;
        if (!rtIsNaNF(varargin_1[ix - 1])) {
          mtmp = varargin_1[ix - 1];
          itmp = ix;
          exitg2 = true;
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

    if (itmp == 1) {
      xt.robot_state.IMU.att[0] = (float)sqrt((double)((1.0F + 2.0F *
        cameraParameters->R_ci[0]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8]))) / 2.0F;
      xt.robot_state.IMU.att[1] = (cameraParameters->R_ci[3] +
        cameraParameters->R_ci[1]) / (4.0F * ((float)sqrt((double)((1.0F + 2.0F *
        cameraParameters->R_ci[0]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8]))) / 2.0F));
      xt.robot_state.IMU.att[2] = (cameraParameters->R_ci[6] +
        cameraParameters->R_ci[2]) / (4.0F * ((float)sqrt((double)((1.0F + 2.0F *
        cameraParameters->R_ci[0]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8]))) / 2.0F));
      xt.robot_state.IMU.att[3] = (cameraParameters->R_ci[7] -
        cameraParameters->R_ci[5]) / (4.0F * ((float)sqrt((double)((1.0F + 2.0F *
        cameraParameters->R_ci[0]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8]))) / 2.0F));
    } else if (itmp == 2) {
      xt.robot_state.IMU.att[0] = (cameraParameters->R_ci[3] +
        cameraParameters->R_ci[1]) / (4.0F * ((float)sqrt((double)((1.0F + 2.0F *
        cameraParameters->R_ci[4]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8]))) / 2.0F));
      xt.robot_state.IMU.att[1] = (float)sqrt((double)((1.0F + 2.0F *
        cameraParameters->R_ci[4]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8]))) / 2.0F;
      xt.robot_state.IMU.att[2] = (cameraParameters->R_ci[7] +
        cameraParameters->R_ci[5]) / (4.0F * ((float)sqrt((double)((1.0F + 2.0F *
        cameraParameters->R_ci[4]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8]))) / 2.0F));
      xt.robot_state.IMU.att[3] = (cameraParameters->R_ci[2] -
        cameraParameters->R_ci[6]) / (4.0F * ((float)sqrt((double)((1.0F + 2.0F *
        cameraParameters->R_ci[4]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8]))) / 2.0F));
    } else if (itmp == 3) {
      xt.robot_state.IMU.att[0] = (cameraParameters->R_ci[6] +
        cameraParameters->R_ci[2]) / (4.0F * ((float)sqrt((double)((1.0F + 2.0F *
        cameraParameters->R_ci[8]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8]))) / 2.0F));
      xt.robot_state.IMU.att[1] = (cameraParameters->R_ci[7] +
        cameraParameters->R_ci[5]) / (4.0F * ((float)sqrt((double)((1.0F + 2.0F *
        cameraParameters->R_ci[8]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8]))) / 2.0F));
      xt.robot_state.IMU.att[2] = (float)sqrt((double)((1.0F + 2.0F *
        cameraParameters->R_ci[8]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8]))) / 2.0F;
      xt.robot_state.IMU.att[3] = (cameraParameters->R_ci[3] -
        cameraParameters->R_ci[1]) / (4.0F * ((float)sqrt((double)((1.0F + 2.0F *
        cameraParameters->R_ci[8]) - ((cameraParameters->R_ci[0] +
        cameraParameters->R_ci[4]) + cameraParameters->R_ci[8]))) / 2.0F));
    } else {
      xt.robot_state.IMU.att[0] = (cameraParameters->R_ci[7] -
        cameraParameters->R_ci[5]) / (4.0F * ((float)sqrt((double)(1.0F +
        ((cameraParameters->R_ci[0] + cameraParameters->R_ci[4]) +
         cameraParameters->R_ci[8]))) / 2.0F));
      xt.robot_state.IMU.att[1] = (cameraParameters->R_ci[2] -
        cameraParameters->R_ci[6]) / (4.0F * ((float)sqrt((double)(1.0F +
        ((cameraParameters->R_ci[0] + cameraParameters->R_ci[4]) +
         cameraParameters->R_ci[8]))) / 2.0F));
      xt.robot_state.IMU.att[2] = (cameraParameters->R_ci[3] -
        cameraParameters->R_ci[1]) / (4.0F * ((float)sqrt((double)(1.0F +
        ((cameraParameters->R_ci[0] + cameraParameters->R_ci[4]) +
         cameraParameters->R_ci[8]))) / 2.0F));
      xt.robot_state.IMU.att[3] = (float)sqrt((double)(1.0F +
        ((cameraParameters->R_ci[0] + cameraParameters->R_ci[4]) +
         cameraParameters->R_ci[8]))) / 2.0F;
    }

    for (ixstart = 0; ixstart < 3; ixstart++) {
      xt.robot_state.IMU.gyro_bias[ixstart] = cameraParameters->
        gyro_bias[ixstart];
      xt.robot_state.IMU.acc_bias[ixstart] = cameraParameters->acc_bias[ixstart];
      xt.robot_state.pos[ixstart] = 0.0F;
    }

    //  position relative to the origin frame
    for (ixstart = 0; ixstart < 4; ixstart++) {
      xt.robot_state.att[ixstart] = iv2[ixstart];
    }

    //  orientation relative to the origin frame
    //  velocity in the origin frame
    xt.fixed_feature = 0;
    xt.origin.anchor_idx = 0;

    //  idx of the anchor that is at the origin
    for (ixstart = 0; ixstart < 3; ixstart++) {
      xt.robot_state.vel[ixstart] = 0.0F;
      xt.origin.pos[ixstart] = 0.0F;
    }

    //  position of the origin in the world frame
    for (ixstart = 0; ixstart < 4; ixstart++) {
      xt.origin.att[ixstart] = iv2[ixstart];
    }

    //  orientation of the origin in the world frame
    memset(&P[0], 0, 8281U * sizeof(float));

    //  initial error state covariance
    memcpy(&xt.anchor_states[0], &rv1[0], 5U * sizeof(f_struct_T));
    for (anchorIdx = 0; anchorIdx < 5; anchorIdx++) {
      ixstart = anchorIdx * 14 + 21;
      for (ix = 0; ix < 6; ix++) {
        xt.anchor_states[anchorIdx].P_idx[ix] = (ix + ixstart) + 1;
      }
    }

    if (vision) {
      memset(&h_u[0], 0, 160U * sizeof(float));
      memset(&map[0], 0, 120U * sizeof(float));
      memset(&delayedStatus[0], 0, 40U * sizeof(float));
    } else {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        t_ci[ixstart] = measurements->acc_duo[ixstart] -
          xt.robot_state.IMU.acc_bias[ixstart];
      }

      mtmp = norm(t_ci);
      for (ix = 0; ix < 3; ix++) {
        t_ci[ix] /= mtmp;
      }

      cross(t_ci, fv3, w_imu);
      rdivide(w_imu, norm(w_imu), w_c);
      cross(w_c, t_ci, w_imu);

      //  if ~all(size(q) == [4, 1])
      //      error('q does not have the size of a quaternion')
      //  end
      //  if abs(norm(q) - 1) > 1e-3
      //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
      //  end
      rdivide(w_imu, norm(w_imu), a_c);
      fv4[0] = ((xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0] -
                 xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) -
                xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      fv4[3] = 2.0F * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[1] +
                       xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[3]);
      fv4[6] = 2.0F * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[2] -
                       xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[3]);
      fv4[1] = 2.0F * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[1] -
                       xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[3]);
      fv4[4] = ((-(xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0]) +
                 xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) -
                xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      fv4[7] = 2.0F * (xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[2] +
                       xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[3]);
      fv4[2] = 2.0F * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[2] +
                       xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[3]);
      fv4[5] = 2.0F * (xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[2] -
                       xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[3]);
      fv4[8] = ((-(xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0]) -
                 xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) +
                xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
        xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
      for (ix = 0; ix < 3; ix++) {
        R_ci[ix] = a_c[ix];
        R_ci[3 + ix] = w_c[ix];
        R_ci[6 + ix] = t_ci[ix];
      }

      for (ix = 0; ix < 3; ix++) {
        for (ixstart = 0; ixstart < 3; ixstart++) {
          R_cw[ix + 3 * ixstart] = 0.0F;
          for (itmp = 0; itmp < 3; itmp++) {
            R_cw[ix + 3 * ixstart] += fv4[ix + 3 * itmp] * R_ci[itmp + 3 *
              ixstart];
          }
        }
      }

      //  THIS IS OK, It is according to the NASA memo found
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
      // % speed optimization
      varargin_1[0] = (1.0F + R_cw[0]) - (R_cw[4] + R_cw[8]);
      varargin_1[1] = (1.0F + R_cw[4]) - (R_cw[0] + R_cw[8]);
      varargin_1[2] = (1.0F + R_cw[8]) - (R_cw[0] + R_cw[4]);
      varargin_1[3] = 1.0F + ((R_cw[0] + R_cw[4]) + R_cw[8]);
      ixstart = 1;
      mtmp = varargin_1[0];
      itmp = 1;
      if (rtIsNaNF(varargin_1[0])) {
        ix = 2;
        exitg1 = false;
        while ((!exitg1) && (ix < 5)) {
          ixstart = ix;
          if (!rtIsNaNF(varargin_1[ix - 1])) {
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

      if (itmp == 1) {
        xt.origin.att[0] = (float)sqrt((double)((1.0F + 2.0F * R_cw[0]) -
          ((R_cw[0] + R_cw[4]) + R_cw[8]))) / 2.0F;
        xt.origin.att[1] = (R_cw[3] + R_cw[1]) / (4.0F * ((float)sqrt((double)
          ((1.0F + 2.0F * R_cw[0]) - ((R_cw[0] + R_cw[4]) + R_cw[8]))) / 2.0F));
        xt.origin.att[2] = (R_cw[6] + R_cw[2]) / (4.0F * ((float)sqrt((double)
          ((1.0F + 2.0F * R_cw[0]) - ((R_cw[0] + R_cw[4]) + R_cw[8]))) / 2.0F));
        xt.origin.att[3] = (R_cw[7] - R_cw[5]) / (4.0F * ((float)sqrt((double)
          ((1.0F + 2.0F * R_cw[0]) - ((R_cw[0] + R_cw[4]) + R_cw[8]))) / 2.0F));
      } else if (itmp == 2) {
        xt.origin.att[0] = (R_cw[3] + R_cw[1]) / (4.0F * ((float)sqrt((double)
          ((1.0F + 2.0F * R_cw[4]) - ((R_cw[0] + R_cw[4]) + R_cw[8]))) / 2.0F));
        xt.origin.att[1] = (float)sqrt((double)((1.0F + 2.0F * R_cw[4]) -
          ((R_cw[0] + R_cw[4]) + R_cw[8]))) / 2.0F;
        xt.origin.att[2] = (R_cw[7] + R_cw[5]) / (4.0F * ((float)sqrt((double)
          ((1.0F + 2.0F * R_cw[4]) - ((R_cw[0] + R_cw[4]) + R_cw[8]))) / 2.0F));
        xt.origin.att[3] = (R_cw[2] - R_cw[6]) / (4.0F * ((float)sqrt((double)
          ((1.0F + 2.0F * R_cw[4]) - ((R_cw[0] + R_cw[4]) + R_cw[8]))) / 2.0F));
      } else if (itmp == 3) {
        xt.origin.att[0] = (R_cw[6] + R_cw[2]) / (4.0F * ((float)sqrt((double)
          ((1.0F + 2.0F * R_cw[8]) - ((R_cw[0] + R_cw[4]) + R_cw[8]))) / 2.0F));
        xt.origin.att[1] = (R_cw[7] + R_cw[5]) / (4.0F * ((float)sqrt((double)
          ((1.0F + 2.0F * R_cw[8]) - ((R_cw[0] + R_cw[4]) + R_cw[8]))) / 2.0F));
        xt.origin.att[2] = (float)sqrt((double)((1.0F + 2.0F * R_cw[8]) -
          ((R_cw[0] + R_cw[4]) + R_cw[8]))) / 2.0F;
        xt.origin.att[3] = (R_cw[3] - R_cw[1]) / (4.0F * ((float)sqrt((double)
          ((1.0F + 2.0F * R_cw[8]) - ((R_cw[0] + R_cw[4]) + R_cw[8]))) / 2.0F));
      } else {
        xt.origin.att[0] = (R_cw[7] - R_cw[5]) / (4.0F * ((float)sqrt((double)
          (1.0F + ((R_cw[0] + R_cw[4]) + R_cw[8]))) / 2.0F));
        xt.origin.att[1] = (R_cw[2] - R_cw[6]) / (4.0F * ((float)sqrt((double)
          (1.0F + ((R_cw[0] + R_cw[4]) + R_cw[8]))) / 2.0F));
        xt.origin.att[2] = (R_cw[3] - R_cw[1]) / (4.0F * ((float)sqrt((double)
          (1.0F + ((R_cw[0] + R_cw[4]) + R_cw[8]))) / 2.0F));
        xt.origin.att[3] = (float)sqrt((double)(1.0F + ((R_cw[0] + R_cw[4]) +
          R_cw[8]))) / 2.0F;
      }

      //  orientation of the origin in the world frame
      //  position
      //  orientation of camera in origin frame
      //  velocity
      diag(noiseParameters->gyro_bias_initial_unc, fv4);

      //  gyro bias
      diag(noiseParameters->acc_bias_initial_unc, fv5);

      //  acc bias
      for (ix = 0; ix < 3; ix++) {
        for (ixstart = 0; ixstart < 3; ixstart++) {
          P[ixstart + 91 * ix] = 0.0F;
          P[(ixstart + 91 * (3 + ix)) + 3] = 0.0F;
          P[(ixstart + 91 * (6 + ix)) + 6] = iv3[ixstart + 3 * ix];
          P[(ixstart + 91 * (9 + ix)) + 9] = fv4[ixstart + 3 * ix];
          P[(ixstart + 91 * (12 + ix)) + 12] = fv5[ixstart + 3 * ix];
          fv6[ix + 3 * ixstart] = 0.0F;
          for (itmp = 0; itmp < 3; itmp++) {
            fv6[ix + 3 * ixstart] += 0.1F * R_cw[ix + 3 * itmp] * (float)b[itmp
              + 3 * ixstart];
          }
        }
      }

      for (ix = 0; ix < 3; ix++) {
        for (ixstart = 0; ixstart < 3; ixstart++) {
          P[(ix + 91 * (15 + ixstart)) + 15] = 0.0F;
          for (itmp = 0; itmp < 3; itmp++) {
            P[(ix + 91 * (15 + ixstart)) + 15] += fv6[ix + 3 * itmp] *
              R_cw[ixstart + 3 * itmp];
          }
        }
      }

      //  origin orientation
      for (ix = 0; ix < 3; ix++) {
        for (ixstart = 0; ixstart < 3; ixstart++) {
          P[(ixstart + 91 * (18 + ix)) + 18] = 0.0F;
        }
      }

      //  R_ci
      memset(&h_u[0], 0, 160U * sizeof(float));
      getMap(xt.origin.pos, xt.origin.att, xt.anchor_states, map);
      memset(&delayedStatus[0], 0, 40U * sizeof(float));
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
      initialized_not_empty = true;
    }
  } else if (!vision) {
    //      [xt,P] =  SLAM_pred(P, xt, dt, noiseParameters.process_noise, measurements, numStates); 
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    R_cw[0] = ((xt.robot_state.att[0] * xt.robot_state.att[0] -
                xt.robot_state.att[1] * xt.robot_state.att[1]) -
               xt.robot_state.att[2] * xt.robot_state.att[2]) +
      xt.robot_state.att[3] * xt.robot_state.att[3];
    R_cw[3] = 2.0F * (xt.robot_state.att[0] * xt.robot_state.att[1] +
                      xt.robot_state.att[2] * xt.robot_state.att[3]);
    R_cw[6] = 2.0F * (xt.robot_state.att[0] * xt.robot_state.att[2] -
                      xt.robot_state.att[1] * xt.robot_state.att[3]);
    R_cw[1] = 2.0F * (xt.robot_state.att[0] * xt.robot_state.att[1] -
                      xt.robot_state.att[2] * xt.robot_state.att[3]);
    R_cw[4] = ((-(xt.robot_state.att[0] * xt.robot_state.att[0]) +
                xt.robot_state.att[1] * xt.robot_state.att[1]) -
               xt.robot_state.att[2] * xt.robot_state.att[2]) +
      xt.robot_state.att[3] * xt.robot_state.att[3];
    R_cw[7] = 2.0F * (xt.robot_state.att[1] * xt.robot_state.att[2] +
                      xt.robot_state.att[0] * xt.robot_state.att[3]);
    R_cw[2] = 2.0F * (xt.robot_state.att[0] * xt.robot_state.att[2] +
                      xt.robot_state.att[1] * xt.robot_state.att[3]);
    R_cw[5] = 2.0F * (xt.robot_state.att[1] * xt.robot_state.att[2] -
                      xt.robot_state.att[0] * xt.robot_state.att[3]);
    R_cw[8] = ((-(xt.robot_state.att[0] * xt.robot_state.att[0]) -
                xt.robot_state.att[1] * xt.robot_state.att[1]) +
               xt.robot_state.att[2] * xt.robot_state.att[2]) +
      xt.robot_state.att[3] * xt.robot_state.att[3];

    //  rotation in origin frame
    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    b_R_ci[0] = ((xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0] -
                  xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) -
                 xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
      xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
    b_R_ci[3] = 2.0F * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[1] +
                        xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[3]);
    b_R_ci[6] = 2.0F * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[2] -
                        xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[3]);
    b_R_ci[1] = 2.0F * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[1] -
                        xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[3]);
    b_R_ci[4] = ((-(xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0]) +
                  xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) -
                 xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
      xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];
    b_R_ci[7] = 2.0F * (xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[2] +
                        xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[3]);
    b_R_ci[2] = 2.0F * (xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[2] +
                        xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[3]);
    b_R_ci[5] = 2.0F * (xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[2] -
                        xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[3]);
    b_R_ci[8] = ((-(xt.robot_state.IMU.att[0] * xt.robot_state.IMU.att[0]) -
                  xt.robot_state.IMU.att[1] * xt.robot_state.IMU.att[1]) +
                 xt.robot_state.IMU.att[2] * xt.robot_state.IMU.att[2]) +
      xt.robot_state.IMU.att[3] * xt.robot_state.IMU.att[3];

    //  in camera frame
    for (ix = 0; ix < 3; ix++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        R_ci[ixstart + 3 * ix] = -b_R_ci[ix + 3 * ixstart];
      }
    }

    //  in imu frame
    for (ixstart = 0; ixstart < 3; ixstart++) {
      t_ci[ixstart] = 0.0F;
      for (ix = 0; ix < 3; ix++) {
        t_ci[ixstart] += R_ci[ixstart + 3 * ix] * xt.robot_state.IMU.pos[ix];
      }

      w_imu[ixstart] = measurements->gyr_duo[ixstart] -
        xt.robot_state.IMU.gyro_bias[ixstart];
    }

    //  gyro in IMU frame
    //  gyro in camera frame
    //  w = 0*w;
    //  acceleration in IMU frame
    R[0] = 0.0F;
    R[3] = -w_imu[2];
    R[6] = w_imu[1];
    R[1] = w_imu[2];
    R[4] = 0.0F;
    R[7] = -w_imu[0];
    R[2] = -w_imu[1];
    R[5] = w_imu[0];
    R[8] = 0.0F;
    for (ix = 0; ix < 3; ix++) {
      w_c[ix] = 0.0F;
      mtmp = 0.0F;
      for (ixstart = 0; ixstart < 3; ixstart++) {
        w_c[ix] += b_R_ci[ix + 3 * ixstart] * w_imu[ixstart];
        b_R[ix + 3 * ixstart] = 0.0F;
        for (itmp = 0; itmp < 3; itmp++) {
          b_R[ix + 3 * ixstart] += R[ix + 3 * itmp] * R[itmp + 3 * ixstart];
        }

        mtmp += b_R[ix + 3 * ixstart] * t_ci[ixstart];
      }

      b_measurements[ix] = (measurements->acc_duo[ix] -
                            xt.robot_state.IMU.acc_bias[ix]) + mtmp;
    }

    for (ix = 0; ix < 3; ix++) {
      a_c[ix] = 0.0F;
      for (ixstart = 0; ixstart < 3; ixstart++) {
        a_c[ix] += b_R_ci[ix + 3 * ixstart] * b_measurements[ixstart];
      }
    }

    //  a = 0*a;
    // % compute the linearization F of the non linear model f
    v[0] = noiseParameters->process_noise.qw;
    v[1] = noiseParameters->process_noise.qw;
    v[2] = noiseParameters->process_noise.qw;
    v[3] = noiseParameters->process_noise.qv;
    v[4] = noiseParameters->process_noise.qv;
    v[5] = noiseParameters->process_noise.qv;
    v[6] = noiseParameters->process_noise.qwo;
    v[7] = noiseParameters->process_noise.qwo;
    v[8] = noiseParameters->process_noise.qwo;
    v[9] = 0.0F * noiseParameters->process_noise.qao;
    v[10] = noiseParameters->process_noise.qao;
    v[11] = 0.0F * noiseParameters->process_noise.qao;
    v[12] = noiseParameters->process_noise.qR_ci;
    v[13] = noiseParameters->process_noise.qR_ci;
    v[14] = noiseParameters->process_noise.qR_ci;
    memset(&d[0], 0, 225U * sizeof(float));
    for (ixstart = 0; ixstart < 15; ixstart++) {
      d[ixstart + 15 * ixstart] = v[ixstart];
    }

    //  if ~all(size(q) == [4, 1])
    //      error('q does not have the size of a quaternion')
    //  end
    //  if abs(norm(q) - 1) > 1e-3
    //      error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1') 
    //  end
    fv4[0] = ((xt.origin.att[0] * xt.origin.att[0] - xt.origin.att[1] *
               xt.origin.att[1]) - xt.origin.att[2] * xt.origin.att[2]) +
      xt.origin.att[3] * xt.origin.att[3];
    fv4[3] = 2.0F * (xt.origin.att[0] * xt.origin.att[1] + xt.origin.att[2] *
                     xt.origin.att[3]);
    fv4[6] = 2.0F * (xt.origin.att[0] * xt.origin.att[2] - xt.origin.att[1] *
                     xt.origin.att[3]);
    fv4[1] = 2.0F * (xt.origin.att[0] * xt.origin.att[1] - xt.origin.att[2] *
                     xt.origin.att[3]);
    fv4[4] = ((-(xt.origin.att[0] * xt.origin.att[0]) + xt.origin.att[1] *
               xt.origin.att[1]) - xt.origin.att[2] * xt.origin.att[2]) +
      xt.origin.att[3] * xt.origin.att[3];
    fv4[7] = 2.0F * (xt.origin.att[1] * xt.origin.att[2] + xt.origin.att[0] *
                     xt.origin.att[3]);
    fv4[2] = 2.0F * (xt.origin.att[0] * xt.origin.att[2] + xt.origin.att[1] *
                     xt.origin.att[3]);
    fv4[5] = 2.0F * (xt.origin.att[1] * xt.origin.att[2] - xt.origin.att[0] *
                     xt.origin.att[3]);
    fv4[8] = ((-(xt.origin.att[0] * xt.origin.att[0]) - xt.origin.att[1] *
               xt.origin.att[1]) + xt.origin.att[2] * xt.origin.att[2]) +
      xt.origin.att[3] * xt.origin.att[3];

    //  gravity transformed into the origin frame
    //    pos,            rot, vel,                                         gyr_bias,   acc_bias,          origin_att,                                    R_ci 
    fv5[0] = 0.0F;
    fv5[3] = -w_imu[2];
    fv5[6] = w_imu[1];
    fv5[1] = w_imu[2];
    fv5[4] = 0.0F;
    fv5[7] = -w_imu[0];
    fv5[2] = -w_imu[1];
    fv5[5] = w_imu[0];
    fv5[8] = 0.0F;
    for (ix = 0; ix < 3; ix++) {
      grav_origin[ix] = 0.0F;
      c[ix] = 0.0F;
      b_c[ix] = 0.0F;
      c_c[ix] = 0.0F;
      for (ixstart = 0; ixstart < 3; ixstart++) {
        grav_origin[ix] += fv4[ix + 3 * ixstart] * b_b[ixstart];
        c[ix] += b_R_ci[ix + 3 * ixstart] * xt.robot_state.IMU.gyro_bias[ixstart];
        b_c[ix] += R_cw[ixstart + 3 * ix] * a_c[ixstart];
        c_c[ix] += fv5[ix + 3 * ixstart] * t_ci[ixstart];
      }
    }

    //  pos
    //  att
    //  vel
    //  gyro bias
    //  acc bias
    //  origin att
    for (ix = 0; ix < 15; ix++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        G[ixstart + 21 * ix] = 0.0F;
        G[(ixstart + 21 * ix) + 3] = iv4[ixstart + 3 * ix];
      }
    }

    for (ix = 0; ix < 3; ix++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        G[(ixstart + 21 * ix) + 6] = 0.0F;
        G[(ixstart + 21 * (ix + 3)) + 6] = -R_cw[ix + 3 * ixstart];
        G[(ixstart + 21 * (ix + 6)) + 6] = 0.0F;
        G[(ixstart + 21 * (ix + 9)) + 6] = 0.0F;
        G[(ixstart + 21 * (ix + 12)) + 6] = 0.0F;
      }
    }

    for (ix = 0; ix < 15; ix++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        G[(ixstart + 21 * ix) + 9] = iv5[ixstart + 3 * ix];
        G[(ixstart + 21 * ix) + 12] = iv6[ixstart + 3 * ix];
        G[(ixstart + 21 * ix) + 15] = 0.0F;
        G[(ixstart + 21 * ix) + 18] = iv7[ixstart + 3 * ix];
      }
    }

    //  R_ci
    fv6[0] = 0.0F;
    fv6[3] = -w_c[2];
    fv6[6] = w_c[1];
    fv6[1] = w_c[2];
    fv6[4] = 0.0F;
    fv6[7] = -w_c[0];
    fv6[2] = -w_c[1];
    fv6[5] = w_c[0];
    fv6[8] = 0.0F;
    fv7[0] = 0.0F;
    fv7[3] = -c[2];
    fv7[6] = c[1];
    fv7[1] = c[2];
    fv7[4] = 0.0F;
    fv7[7] = -c[0];
    fv7[2] = -c[1];
    fv7[5] = c[0];
    fv7[8] = 0.0F;
    fv8[0] = 0.0F;
    fv8[3] = -b_c[2];
    fv8[6] = b_c[1];
    fv8[1] = b_c[2];
    fv8[4] = 0.0F;
    fv8[7] = -b_c[0];
    fv8[2] = -b_c[1];
    fv8[5] = b_c[0];
    fv8[8] = 0.0F;
    fv9[0] = 0.0F;
    fv9[3] = -c_c[2];
    fv9[6] = c_c[1];
    fv9[1] = c_c[2];
    fv9[4] = 0.0F;
    fv9[7] = -c_c[0];
    fv9[2] = -c_c[1];
    fv9[5] = c_c[0];
    fv9[8] = 0.0F;
    fv10[0] = 0.0F;
    fv10[3] = -w_imu[2];
    fv10[6] = w_imu[1];
    fv10[1] = w_imu[2];
    fv10[4] = 0.0F;
    fv10[7] = -w_imu[0];
    fv10[2] = -w_imu[1];
    fv10[5] = w_imu[0];
    fv10[8] = 0.0F;
    fv11[0] = 0.0F;
    fv11[3] = -t_ci[2];
    fv11[6] = t_ci[1];
    fv11[1] = t_ci[2];
    fv11[4] = 0.0F;
    fv11[7] = -t_ci[0];
    fv11[2] = -t_ci[1];
    fv11[5] = t_ci[0];
    fv11[8] = 0.0F;
    fv12[0] = 0.0F;
    fv12[3] = -grav_origin[2];
    fv12[6] = grav_origin[1];
    fv12[1] = grav_origin[2];
    fv12[4] = 0.0F;
    fv12[7] = -grav_origin[0];
    fv12[2] = -grav_origin[1];
    fv12[5] = grav_origin[0];
    fv12[8] = 0.0F;
    fv13[0] = 0.0F;
    fv13[3] = -a_c[2];
    fv13[6] = a_c[1];
    fv13[1] = a_c[2];
    fv13[4] = 0.0F;
    fv13[7] = -a_c[0];
    fv13[2] = -a_c[1];
    fv13[5] = a_c[0];
    fv13[8] = 0.0F;
    for (ix = 0; ix < 3; ix++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        R[ixstart + 3 * ix] = -R_cw[ix + 3 * ixstart];
        mtmp = 0.0F;
        for (itmp = 0; itmp < 3; itmp++) {
          mtmp += fv10[ix + 3 * itmp] * fv11[itmp + 3 * ixstart];
        }

        fv4[ix + 3 * ixstart] = -fv9[ix + 3 * ixstart] - mtmp;
      }
    }

    for (ix = 0; ix < 3; ix++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        R_ci[ix + 3 * ixstart] = 0.0F;
        b_R[ix + 3 * ixstart] = 0.0F;
        for (itmp = 0; itmp < 3; itmp++) {
          R_ci[ix + 3 * ixstart] += R[ix + 3 * itmp] * b_R_ci[itmp + 3 * ixstart];
          b_R[ix + 3 * ixstart] += R_cw[ix + 3 * itmp] * fv13[itmp + 3 * ixstart];
        }
      }
    }

    for (ix = 0; ix < 21; ix++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        Phi[ixstart + 21 * ix] = iv8[ixstart + 3 * ix];
      }
    }

    for (ix = 0; ix < 3; ix++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        Phi[(ixstart + 21 * ix) + 3] = 0.0F;
        Phi[(ixstart + 21 * (ix + 3)) + 3] = -fv6[ixstart + 3 * ix];
        Phi[(ixstart + 21 * (ix + 6)) + 3] = 0.0F;
        Phi[(ixstart + 21 * (ix + 9)) + 3] = iv9[ixstart + 3 * ix];
        Phi[(ixstart + 21 * (ix + 12)) + 3] = 0.0F;
        Phi[(ixstart + 21 * (ix + 15)) + 3] = 0.0F;
        Phi[(ixstart + 21 * (ix + 18)) + 3] = -fv7[ixstart + 3 * ix];
        Phi[(ixstart + 21 * ix) + 6] = 0.0F;
        Phi[(ixstart + 21 * (ix + 3)) + 6] = -fv8[ixstart + 3 * ix];
        Phi[(ixstart + 21 * (ix + 6)) + 6] = 0.0F;
        Phi[(ixstart + 21 * (ix + 9)) + 6] = fv4[ixstart + 3 * ix];
        Phi[(ixstart + 21 * (ix + 12)) + 6] = R_ci[ixstart + 3 * ix];
        Phi[(ixstart + 21 * (ix + 15)) + 6] = -fv12[ixstart + 3 * ix];
        Phi[(ixstart + 21 * (ix + 18)) + 6] = b_R[ixstart + 3 * ix];
      }
    }

    for (ix = 0; ix < 21; ix++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        Phi[(ixstart + 21 * ix) + 9] = 0.0F;
        Phi[(ixstart + 21 * ix) + 12] = 0.0F;
        Phi[(ixstart + 21 * ix) + 15] = 0.0F;
        Phi[(ixstart + 21 * ix) + 18] = 0.0F;
      }

      for (ixstart = 0; ixstart < 21; ixstart++) {
        b_Phi[ixstart + 21 * ix] = (float)iv10[ixstart + 21 * ix] + Phi[ixstart
          + 21 * ix] * dt;
      }
    }

    //  covariance of the state
    for (ix = 0; ix < 21; ix++) {
      for (ixstart = 0; ixstart < 21; ixstart++) {
        Phi[ix + 21 * ixstart] = 0.0F;
        for (itmp = 0; itmp < 21; itmp++) {
          Phi[ix + 21 * ixstart] += b_Phi[ix + 21 * itmp] * P[itmp + 91 *
            ixstart];
        }
      }

      for (ixstart = 0; ixstart < 15; ixstart++) {
        b_G[ix + 21 * ixstart] = 0.0F;
        for (itmp = 0; itmp < 15; itmp++) {
          b_G[ix + 21 * ixstart] += G[ix + 21 * itmp] * d[itmp + 15 * ixstart];
        }
      }

      for (ixstart = 0; ixstart < 21; ixstart++) {
        c_G[ix + 21 * ixstart] = 0.0F;
        for (itmp = 0; itmp < 15; itmp++) {
          c_G[ix + 21 * ixstart] += b_G[ix + 21 * itmp] * G[ixstart + 21 * itmp];
        }

        mtmp = 0.0F;
        for (itmp = 0; itmp < 21; itmp++) {
          mtmp += Phi[ix + 21 * itmp] * b_Phi[ixstart + 21 * itmp];
        }

        P_xx_apr[ix + 21 * ixstart] = mtmp + c_G[ix + 21 * ixstart] * dt;
      }

      for (ixstart = 0; ixstart < 70; ixstart++) {
        P_xs_apr[ix + 21 * ixstart] = 0.0F;
        for (itmp = 0; itmp < 21; itmp++) {
          P_xs_apr[ix + 21 * ixstart] += b_Phi[ix + 21 * itmp] * P[itmp + 91 *
            (21 + ixstart)];
        }
      }
    }

    //  covariance between current state and trails
    for (ix = 0; ix < 21; ix++) {
      for (ixstart = 0; ixstart < 21; ixstart++) {
        P[ixstart + 91 * ix] = (P_xx_apr[ixstart + 21 * ix] + P_xx_apr[ix + 21 *
          ixstart]) / 2.0F;
      }
    }

    for (ix = 0; ix < 70; ix++) {
      memcpy(&P[ix * 91 + 1911], &P_xs_apr[ix * 21], 21U * sizeof(float));
    }

    for (ix = 0; ix < 21; ix++) {
      for (ixstart = 0; ixstart < 70; ixstart++) {
        P[(ixstart + 91 * ix) + 21] = P_xs_apr[ix + 21 * ixstart];
      }
    }

    for (ix = 0; ix < 3; ix++) {
      xt.robot_state.pos[ix] += xt.robot_state.vel[ix] * dt;
      w_c[ix] *= dt;
    }

    theta = norm(w_c) * 0.5F;
    if (theta < 0.244F) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        dq[ixstart] = 0.5F * w_c[ixstart];
      }

      dq[3] = 1.0F;
    } else {
      dq[0] = 0.5F * w_c[0] * (float)sin((double)theta) / theta;
      dq[1] = 0.5F * w_c[1] * (float)sin((double)theta) / theta;
      dq[2] = 0.5F * w_c[2] * (float)sin((double)theta) / theta;
      dq[3] = (float)cos((double)theta);
    }

    mtmp = b_norm(dq);
    for (ixstart = 0; ixstart < 4; ixstart++) {
      varargin_1[ixstart] = xt.robot_state.att[ixstart];
      dq[ixstart] /= mtmp;
    }

    quatmultJ(dq, varargin_1, xt.robot_state.att);
    for (ix = 0; ix < 3; ix++) {
      mtmp = 0.0F;
      for (ixstart = 0; ixstart < 3; ixstart++) {
        mtmp += R_cw[ixstart + 3 * ix] * a_c[ixstart];
      }

      b_measurements[ix] = mtmp - grav_origin[ix];
      xt.robot_state.vel[ix] += b_measurements[ix] * dt;
    }

    //  velocity
    //  P_apr = (P_apr+P_apr')/2;
  } else {
    memcpy(&b_z_all_l[0], &z_all_l[0], 80U * sizeof(float));
    memcpy(&b_z_all_r[0], &z_all_r[0], 80U * sizeof(float));
    SLAM_upd(P, &xt, cameraParameters->CameraParameters1.FocalLength,
             cameraParameters->CameraParameters1.PrincipalPoint,
             cameraParameters->CameraParameters1.RadialDistortion,
             cameraParameters->CameraParameters2.FocalLength,
             cameraParameters->CameraParameters2.PrincipalPoint,
             cameraParameters->CameraParameters2.RadialDistortion,
             cameraParameters->r_lr, cameraParameters->R_lr,
             cameraParameters->R_rl, updateVect, b_z_all_l, b_z_all_r,
             noiseParameters->image_noise,
             noiseParameters->inv_depth_initial_unc, *b_VIOParameters, h_u, map,
             delayedStatus);
  }

  memcpy(&h_u_out[0], &h_u[0], 160U * sizeof(float));
  memcpy(&map_out[0], &map[0], 120U * sizeof(float));
  getWorldState(xt.robot_state.IMU.pos, xt.robot_state.IMU.att,
                xt.robot_state.IMU.gyro_bias, xt.robot_state.IMU.acc_bias,
                xt.robot_state.pos, xt.robot_state.att, xt.origin.pos,
                xt.origin.att, b_measurements, varargin_1, c, b_c, c_c,
                t0_IMU_pos, t0_IMU_att);
  for (ixstart = 0; ixstart < 3; ixstart++) {
    xt_out->pos[ixstart] = b_measurements[ixstart];
  }

  for (ixstart = 0; ixstart < 4; ixstart++) {
    xt_out->att[ixstart] = varargin_1[ixstart];
  }

  for (ixstart = 0; ixstart < 3; ixstart++) {
    xt_out->vel[ixstart] = c[ixstart];
    xt_out->IMU.gyro_bias[ixstart] = b_c[ixstart];
    xt_out->IMU.acc_bias[ixstart] = c_c[ixstart];
    xt_out->IMU.pos[ixstart] = t0_IMU_pos[ixstart];
  }

  for (ixstart = 0; ixstart < 4; ixstart++) {
    xt_out->IMU.att[ixstart] = t0_IMU_att[ixstart];
  }

  getAnchorPoses(xt.origin.pos, xt.origin.att, xt.anchor_states, rv2);
  cast(rv2, anchor_poses_out);
  memcpy(&delayedStatus_out[0], &delayedStatus[0], 40U * sizeof(float));

  //  NOTE: Comment this out for MEXing
  //  output
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
