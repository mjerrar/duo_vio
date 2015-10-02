//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_types.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 02-Oct-2015 15:34:55
//
#ifndef __SLAM_TYPES_H__
#define __SLAM_TYPES_H__

// Include Files
#include "rtwtypes.h"

// Type Definitions
#include "../InterfaceStructs.h"

typedef struct {
  double pos[3];
  double att[4];
  double gyro_bias[3];
  double acc_bias[3];
} struct_T;

typedef struct {
  struct_T IMU;
  double pos[3];
  double att[4];
  double vel[3];
} b_struct_T;

typedef struct {
  double anchor_idx;
  double pos[3];
  double att[4];
} c_struct_T;

typedef struct {
  double inverse_depth;
  double m[3];
  double scaled_map_point[3];
  double status;
  double status_idx;
  double P_idx;
} d_struct_T;

#ifndef struct_emxArray_struct_T
#define struct_emxArray_struct_T

struct emxArray_struct_T
{
  d_struct_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_struct_T

typedef struct {
  double pos[3];
  double att[4];
  double P_idx[6];
  emxArray_struct_T *feature_states;
} e_struct_T;

#ifndef struct_emxArray_AnchorPose
#define struct_emxArray_AnchorPose

struct emxArray_AnchorPose
{
  AnchorPose *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_AnchorPose

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

#ifndef struct_emxArray_b_struct_T
#define struct_emxArray_b_struct_T

struct emxArray_b_struct_T
{
  e_struct_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_b_struct_T

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T

struct emxArray_boolean_T
{
  boolean_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_boolean_T

typedef struct {
  double pos[3];
  double att[4];
} g_struct_T;

#ifndef struct_emxArray_c_struct_T
#define struct_emxArray_c_struct_T

struct emxArray_c_struct_T
{
  g_struct_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_c_struct_T

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
  b_struct_T robot_state;
  double fixed_feature;
  c_struct_T origin;
  emxArray_b_struct_T *anchor_states;
} f_struct_T;

#endif

//
// File trailer for SLAM_types.h
//
// [EOF]
//
