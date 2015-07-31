//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SLAM_types.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 31-Jul-2015 14:58:50
//
#ifndef __SLAM_TYPES_H__
#define __SLAM_TYPES_H__

// Include Files
#include "rtwtypes.h"

// Type Definitions
#include <stdio.h>

typedef struct {
  double Xmap[4];
  double Ymap[4];
  double XmapSingle[4];
  double YmapSingle[4];
  double NewOrigin[2];
} b_struct_T;

typedef struct {
  double T[9];
  double Dimensionality;
} c_struct_T;

typedef struct {
  c_struct_T H1;
  c_struct_T H2;
  double Q[16];
  double XBounds[2];
  double YBounds[2];
  boolean_T Initialized;
  double RectifiedImageSize[2];
} d_struct_T;

typedef struct {
  char Name[30];
  char Version[3];
  char Release[8];
  char Date[11];
} e_struct_T;

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
  double RadialDistortion[3];
  double TangentialDistortion[2];
  double WorldPoints[80];
  char WorldUnits[2];
  boolean_T EstimateSkew;
  double NumRadialDistortionCoefficients;
  boolean_T EstimateTangentialDistortion;
  double TranslationVectors[141];
  double ReprojectionErrors[3760];
  double RotationVectors[141];
  double NumPatterns;
  double IntrinsicMatrix[9];
  double FocalLength[2];
  double PrincipalPoint[2];
  double Skew;
  double MeanReprojectionError;
  double ReprojectedPoints[3760];
  double RotationMatrices[423];
} struct_T;

typedef struct {
  struct_T CameraParameters1;
  struct_T CameraParameters2;
  double RotationOfCamera2[9];
  double TranslationOfCamera2[3];
  double FundamentalMatrix[9];
  double EssentialMatrix[9];
  double MeanReprojectionError;
  double NumPatterns;
  double WorldPoints[80];
  char WorldUnits[2];
  b_struct_T RectifyMap1;
  b_struct_T RectifyMap2;
  d_struct_T RectificationParams;
  e_struct_T Version;
  double r_lr[3];
  double R_lr[9];
  double R_rl[9];
} f_struct_T;

#endif

//
// File trailer for SLAM_types.h
//
// [EOF]
//
