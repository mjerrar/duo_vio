//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fprintf.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 07-Oct-2015 10:22:33
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "fprintf.h"
#include "fileManager.h"
#include <ros/console.h>
#include <stdio.h>

// Function Declarations
static double bb_fprintf(int varargin_1);
static double c_fprintf();
static double db_fprintf(const char varargin_1_data[], const int
  varargin_1_size[2]);
static double e_fprintf(double varargin_1);
static double fb_fprintf(const char varargin_1_data[], const int
  varargin_1_size[2]);
static double g_fprintf(double varargin_1);
static double hb_fprintf(const char varargin_1_data[], const int
  varargin_1_size[2]);
static double i_fprintf(double varargin_1);
static double jb_fprintf(const char varargin_1_data[], const int
  varargin_1_size[2]);
static double k_fprintf(double varargin_1);
static double m_fprintf(double varargin_1);
static double o_fprintf(double varargin_1, double varargin_2, double varargin_3);
static double q_fprintf(double varargin_1, double varargin_2, double varargin_3);
static double s_fprintf(double varargin_1);
static double u_fprintf(double varargin_1);
static double w_fprintf(int varargin_1);
static double y_fprintf(int varargin_1);

// Function Definitions

//
// Arguments    : int varargin_1
// Return Type  : double
//
static double bb_fprintf(int varargin_1)
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
// Arguments    : void
// Return Type  : double
//
static double c_fprintf()
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
// Arguments    : const char varargin_1_data[]
//                const int varargin_1_size[2]
// Return Type  : double
//
static double db_fprintf(const char varargin_1_data[], const int
  varargin_1_size[2])
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  int i6;
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
// Arguments    : double varargin_1
// Return Type  : double
//
static double e_fprintf(double varargin_1)
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
// Arguments    : const char varargin_1_data[]
//                const int varargin_1_size[2]
// Return Type  : double
//
static double fb_fprintf(const char varargin_1_data[], const int
  varargin_1_size[2])
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  int i7;
  char varargout_1_data[6];
  static const char cfmt[19] = { 'f', 'i', 'x', 'e', 'd', '_', 'f', 'e', 'a',
    't', 'u', 'r', 'e', ':', ' ', '%', 's', '\x0a', '\x00' };

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
// Arguments    : double varargin_1
// Return Type  : double
//
static double g_fprintf(double varargin_1)
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
// Arguments    : const char varargin_1_data[]
//                const int varargin_1_size[2]
// Return Type  : double
//
static double hb_fprintf(const char varargin_1_data[], const int
  varargin_1_size[2])
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  int i8;
  char varargout_1_data[6];
  static const char cfmt[10] = { 'm', 'o', 'n', 'o', ':', ' ', '%', 's', '\x0a',
    '\x00' };

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
// Arguments    : double varargin_1
// Return Type  : double
//
static double i_fprintf(double varargin_1)
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
// Arguments    : const char varargin_1_data[]
//                const int varargin_1_size[2]
// Return Type  : double
//
static double jb_fprintf(const char varargin_1_data[], const int
  varargin_1_size[2])
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  int i9;
  char varargout_1_data[6];
  static const char cfmt[12] = { 'R', 'A', 'N', 'S', 'A', 'C', ':', ' ', '%',
    's', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = varargin_1_size[1];
    for (i9 = 0; i9 < nbytesint; i9++) {
      varargout_1_data[i9] = varargin_1_data[i9];
    }

    varargout_1_data[varargin_1_size[1]] = '\x00';
    nbytesint = fprintf(filestar, cfmt, &varargout_1_data[0]);
    fflush(filestar);
  }

  return nbytesint;
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
// Arguments    : double varargin_1
// Return Type  : double
//
static double m_fprintf(double varargin_1)
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
// Arguments    : double varargin_1
//                double varargin_2
//                double varargin_3
// Return Type  : double
//
static double o_fprintf(double varargin_1, double varargin_2, double varargin_3)
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
// Arguments    : double varargin_1
//                double varargin_2
//                double varargin_3
// Return Type  : double
//
static double q_fprintf(double varargin_1, double varargin_2, double varargin_3)
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
// Arguments    : double varargin_1
// Return Type  : double
//
static double s_fprintf(double varargin_1)
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
// Arguments    : double varargin_1
// Return Type  : double
//
static double u_fprintf(double varargin_1)
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
static double w_fprintf(int varargin_1)
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
// Arguments    : int varargin_1
// Return Type  : double
//
static double y_fprintf(int varargin_1)
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
// Arguments    : int formatSpec
// Return Type  : void
//
void ab_fprintf(int formatSpec)
{
  bb_fprintf(formatSpec);
}

//
// Arguments    : void
// Return Type  : void
//
void b_fprintf()
{
  c_fprintf();
}

//
// Arguments    : const char formatSpec_data[]
//                const int formatSpec_size[2]
// Return Type  : void
//
void cb_fprintf(const char formatSpec_data[], const int formatSpec_size[2])
{
  db_fprintf(formatSpec_data, formatSpec_size);
}

//
// Arguments    : double formatSpec
// Return Type  : void
//
void d_fprintf(double formatSpec)
{
  e_fprintf(formatSpec);
}

//
// Arguments    : const char formatSpec_data[]
//                const int formatSpec_size[2]
// Return Type  : void
//
void eb_fprintf(const char formatSpec_data[], const int formatSpec_size[2])
{
  fb_fprintf(formatSpec_data, formatSpec_size);
}

//
// Arguments    : double formatSpec
// Return Type  : void
//
void f_fprintf(double formatSpec)
{
  g_fprintf(formatSpec);
}

//
// Arguments    : const char formatSpec_data[]
//                const int formatSpec_size[2]
// Return Type  : void
//
void gb_fprintf(const char formatSpec_data[], const int formatSpec_size[2])
{
  hb_fprintf(formatSpec_data, formatSpec_size);
}

//
// Arguments    : double formatSpec
// Return Type  : void
//
void h_fprintf(double formatSpec)
{
  i_fprintf(formatSpec);
}

//
// Arguments    : const char formatSpec_data[]
//                const int formatSpec_size[2]
// Return Type  : void
//
void ib_fprintf(const char formatSpec_data[], const int formatSpec_size[2])
{
  jb_fprintf(formatSpec_data, formatSpec_size);
}

//
// Arguments    : double formatSpec
// Return Type  : void
//
void j_fprintf(double formatSpec)
{
  k_fprintf(formatSpec);
}

//
// Arguments    : double formatSpec
// Return Type  : void
//
void l_fprintf(double formatSpec)
{
  m_fprintf(formatSpec);
}

//
// Arguments    : double formatSpec
//                double varargin_1
//                double varargin_2
// Return Type  : void
//
void n_fprintf(double formatSpec, double varargin_1, double varargin_2)
{
  o_fprintf(formatSpec, varargin_1, varargin_2);
}

//
// Arguments    : double formatSpec
//                double varargin_1
//                double varargin_2
// Return Type  : void
//
void p_fprintf(double formatSpec, double varargin_1, double varargin_2)
{
  q_fprintf(formatSpec, varargin_1, varargin_2);
}

//
// Arguments    : double formatSpec
// Return Type  : void
//
void r_fprintf(double formatSpec)
{
  s_fprintf(formatSpec);
}

//
// Arguments    : double formatSpec
// Return Type  : void
//
void t_fprintf(double formatSpec)
{
  u_fprintf(formatSpec);
}

//
// Arguments    : int formatSpec
// Return Type  : void
//
void v_fprintf(int formatSpec)
{
  w_fprintf(formatSpec);
}

//
// Arguments    : int formatSpec
// Return Type  : void
//
void x_fprintf(int formatSpec)
{
  y_fprintf(formatSpec);
}

//
// File trailer for fprintf.cpp
//
// [EOF]
//
