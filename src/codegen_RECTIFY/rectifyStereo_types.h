//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: rectifyStereo_types.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Jun-2015 16:07:13
//
#ifndef __RECTIFYSTEREO_TYPES_H__
#define __RECTIFYSTEREO_TYPES_H__

// Include Files
#include "rtwtypes.h"

// Type Definitions
#ifndef struct_emxArray_uint8_T
#define struct_emxArray_uint8_T

struct emxArray_uint8_T
{
  unsigned char *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_uint8_T

typedef struct {
  char Name[30];
  char Version[3];
  char Release[8];
  char Date[11];
} struct2_T;

typedef struct {
  double RadialDistortion[3];
  double TangentialDistortion[2];
  double WorldPoints[216];
  char WorldUnits[2];
  boolean_T EstimateSkew;
  double NumRadialDistortionCoefficients;
  boolean_T EstimateTangentialDistortion;
  double RotationVectors[93];
  double TranslationVectors[93];
  double ReprojectionErrors[6696];
  double IntrinsicMatrix[9];
  struct2_T Version;
} struct1_T;

typedef struct {
  boolean_T Initialized;
  double H1[9];
  double H2[9];
  double Q[16];
  double XBounds[2];
  double YBounds[2];
  double OriginalImageSize[2];
  char OutputView[5];
} struct3_T;

typedef struct {
  struct1_T CameraParameters1;
  struct1_T CameraParameters2;
  double RotationOfCamera2[9];
  double TranslationOfCamera2[3];
  struct2_T Version;
  struct3_T RectificationParams;
} struct0_T;

typedef struct {
  double RadialDistortion[3];
  double TangentialDistortion[2];
  double WorldPoints[216];
  char WorldUnits[2];
  boolean_T EstimateSkew;
  double NumRadialDistortionCoefficients;
  boolean_T EstimateTangentialDistortion;
  double RotationVectors[42];
  double TranslationVectors[42];
  double ReprojectionErrors[3024];
  double IntrinsicMatrix[9];
  struct2_T Version;
} struct5_T;

typedef struct {
  struct5_T CameraParameters1;
  struct5_T CameraParameters2;
  double RotationOfCamera2[9];
  double TranslationOfCamera2[3];
  struct2_T Version;
  struct3_T RectificationParams;
} struct4_T;

#endif

//
// File trailer for rectifyStereo_types.h
//
// [EOF]
//
