//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fprintf.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 11:35:06
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "fprintf.h"
#include "fileManager.h"
#include <stdio.h>

// Function Declarations
static double c_fprintf();
static double e_fprintf(double varargin_1);
static double g_fprintf(signed char varargin_1);
static double i_fprintf(int varargin_1, int varargin_2);
static double k_fprintf();
static double m_fprintf();
static double o_fprintf(signed char varargin_1);
static double q_fprintf(signed char varargin_1);

// Function Definitions

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
  static const char cfmt[47] = { 'F', 'i', 'n', 'i', 's', 'h', 'e', 'd', ' ',
    'i', 'n', 'i', 't', 'i', 'a', 'l', 'i', 'z', 'i', 'n', 'g', ' ', 'a', 't',
    't', 'i', 't', 'u', 'd', 'e', '.', ' ', 'S', 't', 'a', 'r', 't', 'i', 'n',
    'g', ' ', 'S', 'L', 'A', 'M', '\x0a', '\x00' };

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
// Arguments    : double varargin_1
// Return Type  : double
//
static double e_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[29] = { 'W', 'A', 'R', 'N', 'I', 'N', 'G', ':', ' ',
    'm', 'o', 'd', 'e', 'l', '_', 'p', 'r', 'o', 'b', ' ', 'i', 's', ' ', '%',
    '.', '3', 'f', '\x0a', '\x00' };

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
// Arguments    : signed char varargin_1
// Return Type  : double
//
static double g_fprintf(signed char varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[40] = { 'B', 'a', 'd', ' ', 't', 'r', 'i', 'a', 'n',
    'g', 'u', 'l', 'a', 't', 'i', 'o', 'n', '.', ' ', 'D', 'i', 's', 'c', 'a',
    'r', 'd', 'i', 'n', 'g', ' ', 'p', 'o', 'i', 'n', 't', ' ', '%', 'd', '\x0a',
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
// Arguments    : int varargin_1
//                int varargin_2
// Return Type  : double
//
static double i_fprintf(int varargin_1, int varargin_2)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[42] = { 'I', 'n', 'i', 't', 'i', 'a', 'l', 'i', 'z',
    'e', 'd', ' ', '%', 'i', ' ', 'n', 'e', 'w', ' ', 'f', 'e', 'a', 't', 'u',
    'r', 'e', 's', ' ', 'o', 'n', ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ', '%',
    'i', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1, varargin_2);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : void
// Return Type  : double
//
static double k_fprintf()
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[79] = { 'T', 'h', 'e', 'r', 'e', ' ', 'a', 'r', 'e',
    ' ', 's', 't', 'e', 'r', 'e', 'o', ' ', 'm', 'e', 'a', 's', 'u', 'r', 'e',
    'm', 'e', 'n', 't', 's', ' ', 'a', 'v', 'a', 'i', 'l', 'a', 'b', 'l', 'e',
    ' ', 'b', 'u', 't', ' ', 'n', 'o', ' ', 'a', 'n', 'c', 'h', 'o', 'r', ' ',
    'n', 'e', 'e', 'd', 's', ' ', 't', 'o', ' ', 'b', 'e', ' ', 'i', 'n', 'i',
    't', 'i', 'a', 'l', 'i', 'z', 'e', 'd', '\x0a', '\x00' };

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
// Arguments    : void
// Return Type  : double
//
static double m_fprintf()
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[36] = { 'W', 'A', 'R', 'N', 'I', 'N', 'G', ':', ' ',
    'm', 'o', 'd', 'e', 'l', ' ', 'p', 'r', 'o', 'b', 'a', 'b', 'i', 'l', 'i',
    't', 'i', 'e', 's', ' ', 'a', 'r', 'e', ' ', '0', '\x0a', '\x00' };

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
// Arguments    : signed char varargin_1
// Return Type  : double
//
static double o_fprintf(signed char varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[22] = { 'R', 'e', 'j', 'e', 'c', 't', 'i', 'n', 'g',
    ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e', ' ', '%', 'i', '\x0a', '\x00' };

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
// Arguments    : signed char varargin_1
// Return Type  : double
//
static double q_fprintf(signed char varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[28] = { 'R', 'e', 'q', 'u', 'e', 's', 't', 'i', 'n',
    'g', ' ', '%', 'i', ' ', 'n', 'e', 'w', ' ', 'f', 'e', 'a', 't', 'u', 'r',
    'e', 's', '\x0a', '\x00' };

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
// Return Type  : void
//
void b_fprintf()
{
  c_fprintf();
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
// Arguments    : signed char formatSpec
// Return Type  : void
//
void f_fprintf(signed char formatSpec)
{
  g_fprintf(formatSpec);
}

//
// Arguments    : int formatSpec
//                int varargin_1
// Return Type  : void
//
void h_fprintf(int formatSpec, int varargin_1)
{
  i_fprintf(formatSpec, varargin_1);
}

//
// Arguments    : void
// Return Type  : void
//
void j_fprintf()
{
  k_fprintf();
}

//
// Arguments    : void
// Return Type  : void
//
void l_fprintf()
{
  m_fprintf();
}

//
// Arguments    : signed char formatSpec
// Return Type  : void
//
void n_fprintf(signed char formatSpec)
{
  o_fprintf(formatSpec);
}

//
// Arguments    : signed char formatSpec
// Return Type  : void
//
void p_fprintf(signed char formatSpec)
{
  q_fprintf(formatSpec);
}

//
// File trailer for fprintf.cpp
//
// [EOF]
//
