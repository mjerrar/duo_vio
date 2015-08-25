//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fprintf.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 25-Aug-2015 17:43:12
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "fprintf.h"
#include "fileManager.h"
#include <stdio.h>

// Function Declarations
static double bb_fprintf(double varargin_1);
static double c_fprintf(double varargin_1, double varargin_2);
static double db_fprintf(double varargin_1);
static double e_fprintf(double varargin_1);
static double fb_fprintf(double varargin_1);
static double g_fprintf(double varargin_1);
static double hb_fprintf(double varargin_1);
static double i_fprintf(double varargin_1);
static double jb_fprintf(int varargin_1);
static double k_fprintf(double varargin_1);
static double lb_fprintf();
static double m_fprintf(double varargin_1);
static double nb_fprintf(signed char varargin_1);
static double o_fprintf(signed char varargin_1);
static double pb_fprintf(signed char varargin_1);
static double q_fprintf(signed char varargin_1);
static double rb_fprintf(int varargin_1, int varargin_2);
static double s_fprintf(signed char varargin_1);
static double tb_fprintf();
static double u_fprintf(signed char varargin_1);
static double w_fprintf(signed char varargin_1);
static double y_fprintf(double varargin_1);

// Function Definitions

//
// Arguments    : double varargin_1
// Return Type  : double
//
static double bb_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[27] = { 'C', 'o', 'n', 't', 'r', 'o', 'l', 'l', 'e',
    'r', 'G', 'a', 'i', 'n', 's', '.', 'K', 'd', '_', 'z', ' ', '=', ' ', '%',
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
// Arguments    : double varargin_1
//                double varargin_2
// Return Type  : double
//
static double c_fprintf(double varargin_1, double varargin_2)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[37] = { 'n', 'o', 'i', 's', 'e', 'P', 'a', 'r', 'a',
    'm', 'e', 't', 'e', 'r', 's', '.', 'i', 'm', 'a', 'g', 'e', '_', 'n', 'o',
    'i', 's', 'e', ' ', '=', ' ', '%', 'f', ' ', '%', 'f', '\x0a', '\x00' };

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
// Arguments    : double varargin_1
// Return Type  : double
//
static double db_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[28] = { 'C', 'o', 'n', 't', 'r', 'o', 'l', 'l', 'e',
    'r', 'G', 'a', 'i', 'n', 's', '.', 'K', 'p', '_', 'x', 'y', ' ', '=', ' ',
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
// Arguments    : double varargin_1
// Return Type  : double
//
static double e_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[40] = { 'n', 'o', 'i', 's', 'e', 'P', 'a', 'r', 'a',
    'm', 'e', 't', 'e', 'r', 's', '.', 'o', 'r', 'i', 'e', 'n', 't', 'a', 't',
    'i', 'o', 'n', '_', 'n', 'o', 'i', 's', 'e', ' ', '=', ' ', '%', 'f', '\x0a',
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
static double fb_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[29] = { 'C', 'o', 'n', 't', 'r', 'o', 'l', 'l', 'e',
    'r', 'G', 'a', 'i', 'n', 's', '.', 'K', 'p', '_', 'y', 'a', 'w', ' ', '=',
    ' ', '%', 'f', '\x0a', '\x00' };

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
static double g_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[37] = { 'n', 'o', 'i', 's', 'e', 'P', 'a', 'r', 'a',
    'm', 'e', 't', 'e', 'r', 's', '.', 'p', 'r', 'e', 's', 's', 'u', 'r', 'e',
    '_', 'n', 'o', 'i', 's', 'e', ' ', '=', ' ', '%', 'f', '\x0a', '\x00' };

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
static double hb_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[27] = { 'C', 'o', 'n', 't', 'r', 'o', 'l', 'l', 'e',
    'r', 'G', 'a', 'i', 'n', 's', '.', 'K', 'p', '_', 'z', ' ', '=', ' ', '%',
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
// Arguments    : double varargin_1
// Return Type  : double
//
static double i_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[34] = { 'n', 'o', 'i', 's', 'e', 'P', 'a', 'r', 'a',
    'm', 'e', 't', 'e', 'r', 's', '.', 'i', 'm', 'a', 'g', 'e', '_', 'n', 'o',
    'i', 's', 'e', ' ', '=', ' ', '%', 'f', '\x0a', '\x00' };

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
static double jb_fprintf(int varargin_1)
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
// Arguments    : double varargin_1
// Return Type  : double
//
static double k_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[39] = { 'V', 'I', 'O', 'P', 'a', 'r', 'a', 'm', 'e',
    't', 'e', 'r', 's', '.', 'm', 'a', 'x', '_', 'e', 'k', 'f', '_', 'i', 't',
    'e', 'r', 'a', 't', 'i', 'o', 'n', 's', ' ', '=', ' ', '%', 'f', '\x0a',
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
static double lb_fprintf()
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
// Arguments    : double varargin_1
// Return Type  : double
//
static double m_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[32] = { 'V', 'I', 'O', 'P', 'a', 'r', 'a', 'm', 'e',
    't', 'e', 'r', 's', '.', 'n', 'u', 'm', '_', 'a', 'n', 'c', 'h', 'o', 'r',
    's', ' ', '=', ' ', '%', 'f', '\x0a', '\x00' };

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
static double nb_fprintf(signed char varargin_1)
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
  static const char cfmt[42] = { 'V', 'I', 'O', 'P', 'a', 'r', 'a', 'm', 'e',
    't', 'e', 'r', 's', '.', 'n', 'u', 'm', '_', 'p', 'o', 'i', 'n', 't', 's',
    '_', 'p', 'e', 'r', '_', 'a', 'n', 'c', 'h', 'o', 'r', ' ', '=', ' ', '%',
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
// Arguments    : signed char varargin_1
// Return Type  : double
//
static double pb_fprintf(signed char varargin_1)
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
// Arguments    : signed char varargin_1
// Return Type  : double
//
static double q_fprintf(signed char varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[46] = { 'V', 'I', 'O', 'P', 'a', 'r', 'a', 'm', 'e',
    't', 'e', 'r', 's', '.', 'u', 's', 'e', '_', 'c', 'o', 'n', 't', 'r', 'o',
    'l', 'l', 'e', 'r', '_', 't', 'o', '_', 'p', 'r', 'e', 'd', 'i', 'c', 't',
    ' ', '=', ' ', '%', 'd', '\x0a', '\x00' };

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
static double rb_fprintf(int varargin_1, int varargin_2)
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
// Arguments    : signed char varargin_1
// Return Type  : double
//
static double s_fprintf(signed char varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[37] = { 'V', 'I', 'O', 'P', 'a', 'r', 'a', 'm', 'e',
    't', 'e', 'r', 's', '.', 'u', 's', 'e', '_', 'm', 'a', 'g', 'n', 'e', 't',
    'o', 'm', 'e', 't', 'e', 'r', ' ', '=', ' ', '%', 'd', '\x0a', '\x00' };

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
static double tb_fprintf()
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
// Arguments    : signed char varargin_1
// Return Type  : double
//
static double u_fprintf(signed char varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[36] = { 'V', 'I', 'O', 'P', 'a', 'r', 'a', 'm', 'e',
    't', 'e', 'r', 's', '.', 'u', 's', 'e', '_', 'o', 'r', 'i', 'e', 'n', 't',
    'a', 't', 'i', 'o', 'n', ' ', '=', ' ', '%', 'd', '\x0a', '\x00' };

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
static double w_fprintf(signed char varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[33] = { 'V', 'I', 'O', 'P', 'a', 'r', 'a', 'm', 'e',
    't', 'e', 'r', 's', '.', 'u', 's', 'e', '_', 'p', 'r', 'e', 's', 's', 'u',
    'r', 'e', ' ', '=', ' ', '%', 'd', '\x0a', '\x00' };

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
static double y_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[28] = { 'C', 'o', 'n', 't', 'r', 'o', 'l', 'l', 'e',
    'r', 'G', 'a', 'i', 'n', 's', '.', 'K', 'd', '_', 'x', 'y', ' ', '=', ' ',
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
// Arguments    : double formatSpec
// Return Type  : void
//
void ab_fprintf(double formatSpec)
{
  bb_fprintf(formatSpec);
}

//
// Arguments    : double formatSpec
//                double varargin_1
// Return Type  : void
//
void b_fprintf(double formatSpec, double varargin_1)
{
  c_fprintf(formatSpec, varargin_1);
}

//
// Arguments    : double formatSpec
// Return Type  : void
//
void cb_fprintf(double formatSpec)
{
  db_fprintf(formatSpec);
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
// Arguments    : double formatSpec
// Return Type  : void
//
void eb_fprintf(double formatSpec)
{
  fb_fprintf(formatSpec);
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
// Arguments    : double formatSpec
// Return Type  : void
//
void gb_fprintf(double formatSpec)
{
  hb_fprintf(formatSpec);
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
// Arguments    : int formatSpec
// Return Type  : void
//
void ib_fprintf(int formatSpec)
{
  jb_fprintf(formatSpec);
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
// Arguments    : void
// Return Type  : void
//
void kb_fprintf()
{
  lb_fprintf();
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
// Arguments    : signed char formatSpec
// Return Type  : void
//
void mb_fprintf(signed char formatSpec)
{
  nb_fprintf(formatSpec);
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
void ob_fprintf(signed char formatSpec)
{
  pb_fprintf(formatSpec);
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
// Arguments    : int formatSpec
//                int varargin_1
// Return Type  : void
//
void qb_fprintf(int formatSpec, int varargin_1)
{
  rb_fprintf(formatSpec, varargin_1);
}

//
// Arguments    : signed char formatSpec
// Return Type  : void
//
void r_fprintf(signed char formatSpec)
{
  s_fprintf(formatSpec);
}

//
// Arguments    : void
// Return Type  : void
//
void sb_fprintf()
{
  tb_fprintf();
}

//
// Arguments    : signed char formatSpec
// Return Type  : void
//
void t_fprintf(signed char formatSpec)
{
  u_fprintf(formatSpec);
}

//
// Arguments    : signed char formatSpec
// Return Type  : void
//
void v_fprintf(signed char formatSpec)
{
  w_fprintf(formatSpec);
}

//
// Arguments    : double formatSpec
// Return Type  : void
//
void x_fprintf(double formatSpec)
{
  y_fprintf(formatSpec);
}

//
// File trailer for fprintf.cpp
//
// [EOF]
//
