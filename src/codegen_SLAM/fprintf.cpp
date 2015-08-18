//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fprintf.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 18-Aug-2015 11:22:44
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "fprintf.h"
#include "fileManager.h"
#include <stdio.h>

// Function Declarations
static double c_fprintf(double varargin_1, double varargin_2, double varargin_3);
static double e_fprintf(double varargin_1, double varargin_2, double varargin_3);
static double g_fprintf();
static double i_fprintf(int varargin_1);
static double k_fprintf();
static double m_fprintf(signed char varargin_1);
static double o_fprintf(signed char varargin_1);
static double q_fprintf(int varargin_1, int varargin_2);
static double s_fprintf();

// Function Definitions

//
// Arguments    : double varargin_1
//                double varargin_2
//                double varargin_3
// Return Type  : double
//
static double c_fprintf(double varargin_1, double varargin_2, double varargin_3)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[43] = { 'S', 'L', 'A', 'M', ' ', 'i', 'n', 'i', 't',
    'i', 'a', 'l', 'i', 'z', 'i', 'n', 'g', ',', ' ', 'z', 'a', ':', ' ', '(',
    '%', '.', '3', 'f', ',', ' ', '%', '.', '3', 'f', ',', ' ', '%', '.', '3',
    'f', ')', '\x0a', '\x00' };

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
static double e_fprintf(double varargin_1, double varargin_2, double varargin_3)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[42] = { 'A', 't', 't', ' ', 'i', 'n', 'i', 't', 'i',
    'a', 'l', 'i', 'z', 'i', 'n', 'g', ',', ' ', 'z', 'a', ':', ' ', '(', '%',
    '.', '3', 'f', ',', ' ', '%', '.', '3', 'f', ',', ' ', '%', '.', '3', 'f',
    ')', '\x0a', '\x00' };

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
// Arguments    : void
// Return Type  : double
//
static double g_fprintf()
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
// Arguments    : int varargin_1
// Return Type  : double
//
static double i_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[104] = { 'E', 'n', 'd', 'e', 'd', ' ', 'h', 'y', 'p',
    'o', 't', 'h', 'e', 's', 'i', 's', ' ', 't', 'e', 's', 't', '.', ' ', 'F',
    'o', 'u', 'n', 'd', ' ', '%', 'i', ' ', 'L', 'I', ' ', 'i', 'n', 'l', 'i',
    'e', 'r', 's', ',', ' ', 'w', 'h', 'i', 'c', 'h', ' ', 'i', 's', ' ', 'b',
    'e', 'l', 'o', 'w', ' ', 't', 'h', 'e', ' ', 't', 'h', 'r', 'e', 's', 'h',
    'o', 'l', 'd', ' ', '%', 'i', '.', ' ', 'N', 'o', 't', ' ', 'd', 'o', 'i',
    'n', 'g', ' ', 'L', 'I', ' ', 'E', 'K', 'F', ' ', 'u', 'p', 'd', 'a', 't',
    'e', '.', '\x0a', '\x00', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1, 3);
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
  static const char cfmt[39] = { '1', '-', 'p', 'o', 'i', 'n', 't', ' ', 'R',
    'A', 'N', 'S', 'A', 'C', ' ', 'r', 'e', 'j', 'e', 'c', 't', 'e', 'd', ' ',
    'a', 'l', 'l', ' ', 'f', 'e', 'a', 't', 'u', 'r', 'e', 's', '!', '\x00',
    '\x00' };

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
static double m_fprintf(signed char varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[46] = { 'B', 'a', 'd', ' ', 't', 'r', 'i', 'a', 'n',
    'g', 'u', 'l', 'a', 't', 'i', 'o', 'n', ' ', '(', 'n', 'a', 'n', ')', '.',
    ' ', 'D', 'i', 's', 'c', 'a', 'r', 'd', 'i', 'n', 'g', ' ', 'p', 'o', 'i',
    'n', 't', ' ', '%', 'd', '\x0a', '\x00' };

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
static double o_fprintf(signed char varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[61] = { 'B', 'a', 'd', ' ', 't', 'r', 'i', 'a', 'n',
    'g', 'u', 'l', 'a', 't', 'i', 'o', 'n', ' ', '(', 'r', 'e', 'p', 'r', 'o',
    'j', 'e', 'c', 't', 'i', 'o', 'n', ' ', 'e', 'r', 'r', 'o', 'r', ')', '.',
    ' ', 'D', 'i', 's', 'c', 'a', 'r', 'd', 'i', 'n', 'g', ' ', 'p', 'o', 'i',
    'n', 't', ' ', '%', 'd', '\x0a', '\x00' };

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
static double q_fprintf(int varargin_1, int varargin_2)
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
static double s_fprintf()
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
// Arguments    : double formatSpec
//                double varargin_1
//                double varargin_2
// Return Type  : void
//
void b_fprintf(double formatSpec, double varargin_1, double varargin_2)
{
  c_fprintf(formatSpec, varargin_1, varargin_2);
}

//
// Arguments    : double formatSpec
//                double varargin_1
//                double varargin_2
// Return Type  : void
//
void d_fprintf(double formatSpec, double varargin_1, double varargin_2)
{
  e_fprintf(formatSpec, varargin_1, varargin_2);
}

//
// Arguments    : void
// Return Type  : void
//
void f_fprintf()
{
  g_fprintf();
}

//
// Arguments    : int formatSpec
// Return Type  : void
//
void h_fprintf(int formatSpec)
{
  i_fprintf(formatSpec);
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
// Arguments    : signed char formatSpec
// Return Type  : void
//
void l_fprintf(signed char formatSpec)
{
  m_fprintf(formatSpec);
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
// Arguments    : int formatSpec
//                int varargin_1
// Return Type  : void
//
void p_fprintf(int formatSpec, int varargin_1)
{
  q_fprintf(formatSpec, varargin_1);
}

//
// Arguments    : void
// Return Type  : void
//
void r_fprintf()
{
  s_fprintf();
}

//
// File trailer for fprintf.cpp
//
// [EOF]
//
