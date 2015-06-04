//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Jun-2015 16:07:13
//

//***********************************************************************
// This automatically generated example C main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************
// Include Files
#include "rt_nonfinite.h"
#include "rectifyStereo.h"
#include "main.h"
#include "libmwremaptbb.h"

// Function Declarations
static void argInit_108x2_real_T(double result[216]);
static void argInit_108x2x14_real_T(double result[3024]);
static void argInit_108x2x31_real_T(double result[6696]);
static void argInit_14x3_real_T(double result[42]);
static void argInit_1x11_char_T(char result[11]);
static void argInit_1x2_char_T(char result[2]);
static void argInit_1x2_real_T(double result[2]);
static void argInit_1x30_char_T(char result[30]);
static void argInit_1x3_char_T(char result[3]);
static void argInit_1x3_real_T(double result[3]);
static void argInit_1x5_char_T(char result[5]);
static void argInit_1x8_char_T(char result[8]);
static void argInit_31x3_real_T(double result[93]);
static void argInit_3x3_real_T(double result[9]);
static void argInit_480x752_uint8_T(unsigned char result[360960]);
static void argInit_4x4_real_T(double result[16]);
static boolean_T argInit_boolean_T();
static char argInit_char_T();
static double argInit_real_T();
static void argInit_struct0_T(struct0_T *result);
static void argInit_struct1_T(struct1_T *result);
static struct2_T argInit_struct2_T();
static void argInit_struct3_T(struct3_T *result);
static void argInit_struct4_T(struct4_T *result);
static void argInit_struct5_T(struct5_T *result);
static unsigned char argInit_uint8_T();
static void main_rectifyStereo();

// Function Definitions

//
// Arguments    : double result[216]
// Return Type  : void
//
static void argInit_108x2_real_T(double result[216])
{
  int b_j0;
  int b_j1;

  // Loop over the array to initialize each element.
  for (b_j0 = 0; b_j0 < 108; b_j0++) {
    for (b_j1 = 0; b_j1 < 2; b_j1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[b_j0 + 108 * b_j1] = argInit_real_T();
    }
  }
}

//
// Arguments    : double result[3024]
// Return Type  : void
//
static void argInit_108x2x14_real_T(double result[3024])
{
  int b_j0;
  int b_j1;
  int j2;

  // Loop over the array to initialize each element.
  for (b_j0 = 0; b_j0 < 108; b_j0++) {
    for (b_j1 = 0; b_j1 < 2; b_j1++) {
      for (j2 = 0; j2 < 14; j2++) {
        // Set the value of the array element.
        // Change this value to the value that the application requires.
        result[(b_j0 + 108 * b_j1) + 216 * j2] = argInit_real_T();
      }
    }
  }
}

//
// Arguments    : double result[6696]
// Return Type  : void
//
static void argInit_108x2x31_real_T(double result[6696])
{
  int b_j0;
  int b_j1;
  int j2;

  // Loop over the array to initialize each element.
  for (b_j0 = 0; b_j0 < 108; b_j0++) {
    for (b_j1 = 0; b_j1 < 2; b_j1++) {
      for (j2 = 0; j2 < 31; j2++) {
        // Set the value of the array element.
        // Change this value to the value that the application requires.
        result[(b_j0 + 108 * b_j1) + 216 * j2] = argInit_real_T();
      }
    }
  }
}

//
// Arguments    : double result[42]
// Return Type  : void
//
static void argInit_14x3_real_T(double result[42])
{
  int b_j0;
  int b_j1;

  // Loop over the array to initialize each element.
  for (b_j0 = 0; b_j0 < 14; b_j0++) {
    for (b_j1 = 0; b_j1 < 3; b_j1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[b_j0 + 14 * b_j1] = argInit_real_T();
    }
  }
}

//
// Arguments    : char result[11]
// Return Type  : void
//
static void argInit_1x11_char_T(char result[11])
{
  int b_j1;

  // Loop over the array to initialize each element.
  for (b_j1 = 0; b_j1 < 11; b_j1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[b_j1] = argInit_char_T();
  }
}

//
// Arguments    : char result[2]
// Return Type  : void
//
static void argInit_1x2_char_T(char result[2])
{
  int b_j1;

  // Loop over the array to initialize each element.
  for (b_j1 = 0; b_j1 < 2; b_j1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[b_j1] = argInit_char_T();
  }
}

//
// Arguments    : double result[2]
// Return Type  : void
//
static void argInit_1x2_real_T(double result[2])
{
  int b_j1;

  // Loop over the array to initialize each element.
  for (b_j1 = 0; b_j1 < 2; b_j1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[b_j1] = argInit_real_T();
  }
}

//
// Arguments    : char result[30]
// Return Type  : void
//
static void argInit_1x30_char_T(char result[30])
{
  int b_j1;

  // Loop over the array to initialize each element.
  for (b_j1 = 0; b_j1 < 30; b_j1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[b_j1] = argInit_char_T();
  }
}

//
// Arguments    : char result[3]
// Return Type  : void
//
static void argInit_1x3_char_T(char result[3])
{
  int b_j1;

  // Loop over the array to initialize each element.
  for (b_j1 = 0; b_j1 < 3; b_j1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[b_j1] = argInit_char_T();
  }
}

//
// Arguments    : double result[3]
// Return Type  : void
//
static void argInit_1x3_real_T(double result[3])
{
  int b_j1;

  // Loop over the array to initialize each element.
  for (b_j1 = 0; b_j1 < 3; b_j1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[b_j1] = argInit_real_T();
  }
}

//
// Arguments    : char result[5]
// Return Type  : void
//
static void argInit_1x5_char_T(char result[5])
{
  int b_j1;

  // Loop over the array to initialize each element.
  for (b_j1 = 0; b_j1 < 5; b_j1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[b_j1] = argInit_char_T();
  }
}

//
// Arguments    : char result[8]
// Return Type  : void
//
static void argInit_1x8_char_T(char result[8])
{
  int b_j1;

  // Loop over the array to initialize each element.
  for (b_j1 = 0; b_j1 < 8; b_j1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[b_j1] = argInit_char_T();
  }
}

//
// Arguments    : double result[93]
// Return Type  : void
//
static void argInit_31x3_real_T(double result[93])
{
  int b_j0;
  int b_j1;

  // Loop over the array to initialize each element.
  for (b_j0 = 0; b_j0 < 31; b_j0++) {
    for (b_j1 = 0; b_j1 < 3; b_j1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[b_j0 + 31 * b_j1] = argInit_real_T();
    }
  }
}

//
// Arguments    : double result[9]
// Return Type  : void
//
static void argInit_3x3_real_T(double result[9])
{
  int b_j0;
  int b_j1;

  // Loop over the array to initialize each element.
  for (b_j0 = 0; b_j0 < 3; b_j0++) {
    for (b_j1 = 0; b_j1 < 3; b_j1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[b_j0 + 3 * b_j1] = argInit_real_T();
    }
  }
}

//
// Arguments    : unsigned char result[360960]
// Return Type  : void
//
static void argInit_480x752_uint8_T(unsigned char result[360960])
{
  int b_j0;
  int b_j1;

  // Loop over the array to initialize each element.
  for (b_j0 = 0; b_j0 < 480; b_j0++) {
    for (b_j1 = 0; b_j1 < 752; b_j1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[b_j0 + 480 * b_j1] = argInit_uint8_T();
    }
  }
}

//
// Arguments    : double result[16]
// Return Type  : void
//
static void argInit_4x4_real_T(double result[16])
{
  int b_j0;
  int b_j1;

  // Loop over the array to initialize each element.
  for (b_j0 = 0; b_j0 < 4; b_j0++) {
    for (b_j1 = 0; b_j1 < 4; b_j1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[b_j0 + (b_j1 << 2)] = argInit_real_T();
    }
  }
}

//
// Arguments    : void
// Return Type  : boolean_T
//
static boolean_T argInit_boolean_T()
{
  return false;
}

//
// Arguments    : void
// Return Type  : char
//
static char argInit_char_T()
{
  return '?';
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : struct0_T *result
// Return Type  : void
//
static void argInit_struct0_T(struct0_T *result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_struct1_T(&result->CameraParameters1);
  argInit_struct1_T(&result->CameraParameters2);
  argInit_3x3_real_T(result->RotationOfCamera2);
  argInit_1x3_real_T(result->TranslationOfCamera2);
  result->Version = argInit_struct2_T();
  argInit_struct3_T(&result->RectificationParams);
}

//
// Arguments    : struct1_T *result
// Return Type  : void
//
static void argInit_struct1_T(struct1_T *result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_1x3_real_T(result->RadialDistortion);
  argInit_1x2_real_T(result->TangentialDistortion);
  argInit_108x2_real_T(result->WorldPoints);
  argInit_1x2_char_T(result->WorldUnits);
  result->EstimateSkew = argInit_boolean_T();
  result->NumRadialDistortionCoefficients = argInit_real_T();
  result->EstimateTangentialDistortion = argInit_boolean_T();
  argInit_31x3_real_T(result->RotationVectors);
  argInit_31x3_real_T(result->TranslationVectors);
  argInit_108x2x31_real_T(result->ReprojectionErrors);
  argInit_3x3_real_T(result->IntrinsicMatrix);
  result->Version = argInit_struct2_T();
}

//
// Arguments    : void
// Return Type  : struct2_T
//
static struct2_T argInit_struct2_T()
{
  struct2_T result;

  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_1x30_char_T(result.Name);
  argInit_1x3_char_T(result.Version);
  argInit_1x8_char_T(result.Release);
  argInit_1x11_char_T(result.Date);
  return result;
}

//
// Arguments    : struct3_T *result
// Return Type  : void
//
static void argInit_struct3_T(struct3_T *result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result->Initialized = argInit_boolean_T();
  argInit_3x3_real_T(result->H1);
  argInit_3x3_real_T(result->H2);
  argInit_4x4_real_T(result->Q);
  argInit_1x2_real_T(result->XBounds);
  argInit_1x2_real_T(result->YBounds);
  argInit_1x2_real_T(result->OriginalImageSize);
  argInit_1x5_char_T(result->OutputView);
}

//
// Arguments    : struct4_T *result
// Return Type  : void
//
static void argInit_struct4_T(struct4_T *result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_struct5_T(&result->CameraParameters1);
  argInit_struct5_T(&result->CameraParameters2);
  argInit_3x3_real_T(result->RotationOfCamera2);
  argInit_1x3_real_T(result->TranslationOfCamera2);
  result->Version = argInit_struct2_T();
  argInit_struct3_T(&result->RectificationParams);
}

//
// Arguments    : struct5_T *result
// Return Type  : void
//
static void argInit_struct5_T(struct5_T *result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_1x3_real_T(result->RadialDistortion);
  argInit_1x2_real_T(result->TangentialDistortion);
  argInit_108x2_real_T(result->WorldPoints);
  argInit_1x2_char_T(result->WorldUnits);
  result->EstimateSkew = argInit_boolean_T();
  result->NumRadialDistortionCoefficients = argInit_real_T();
  result->EstimateTangentialDistortion = argInit_boolean_T();
  argInit_14x3_real_T(result->RotationVectors);
  argInit_14x3_real_T(result->TranslationVectors);
  argInit_108x2x14_real_T(result->ReprojectionErrors);
  argInit_3x3_real_T(result->IntrinsicMatrix);
  result->Version = argInit_struct2_T();
}

//
// Arguments    : void
// Return Type  : unsigned char
//
static unsigned char argInit_uint8_T()
{
  return 0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_rectifyStereo()
{
  emxArray_uint8_T *ImGrayR_l;
  emxArray_uint8_T *ImGrayR_r;
  static unsigned char uv0[360960];
  static unsigned char uv1[360960];
  struct0_T r2;
  struct4_T r3;
  double cameraparams_out[4];
  emxInitArray_uint8_T(&ImGrayR_l, 2);
  emxInitArray_uint8_T(&ImGrayR_r, 2);

  // Initialize function 'rectifyStereo' input arguments.
  // Initialize function input argument 'ImGrayR_lr'.
  // Initialize function input argument 'ImGrayR_rr'.
  // Initialize function input argument 'stereoParamStruct'.
  // Initialize function input argument 'stereoParamsSecondStruct'.
  // Call the entry-point 'rectifyStereo'.
  argInit_480x752_uint8_T(uv0);
  argInit_480x752_uint8_T(uv1);
  argInit_struct0_T(&r2);
  argInit_struct4_T(&r3);
  rectifyStereo(uv0, uv1, &r2, &r3, ImGrayR_l, ImGrayR_r, cameraparams_out);
  emxDestroyArray_uint8_T(ImGrayR_r);
  emxDestroyArray_uint8_T(ImGrayR_l);
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // Initialize the application.
  // You do not need to do this more than one time.
  rectifyStereo_initialize();

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_rectifyStereo();

  // Terminate the application.
  // You do not need to do this more than one time.
  rectifyStereo_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
