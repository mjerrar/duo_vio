//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fprintf.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 15-Jul-2015 17:00:42
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "fprintf.h"
#include "fileManager.h"
#include <stdio.h>

// Function Declarations
static double ac_fprintf();
static double bb_fprintf(int varargin_1);
static double c_fprintf();
static double db_fprintf(int varargin_1);
static double e_fprintf();
static double fb_fprintf(int varargin_1);
static double g_fprintf();
static double hb_fprintf(int varargin_1);
static double i_fprintf();
static double jb_fprintf(int varargin_1);
static double k_fprintf(int varargin_1);
static double lb_fprintf(int varargin_1);
static double m_fprintf(int varargin_1);
static double nb_fprintf();
static double o_fprintf(int varargin_1);
static double pb_fprintf();
static double q_fprintf(double varargin_1);
static double rb_fprintf();
static double s_fprintf(int varargin_1);
static double tb_fprintf();
static double u_fprintf(double varargin_1);
static double vb_fprintf();
static double w_fprintf(int varargin_1);
static double xb_fprintf();
static double y_fprintf(int varargin_1);

// Function Definitions

//
// Arguments    : void
// Return Type  : double
//
static double ac_fprintf()
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[35] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'S', ' ', 'n', 'o', 't', ' ', 'i',
    'n', 'v', 'e', 'r', 't', 'i', 'b', 'l', 'e', '\x0a', '\x00' };

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
static double bb_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[27] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'r', 'a', 'd', '_', 'l', ' ', '%',
    'i', '\x0a', '\x00' };

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
  static const char cfmt[23] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'R', '_', 'c', 'w', '\x0a', '\x00' };

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
static double db_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[27] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'r', 'a', 'd', '_', 'r', ' ', '%',
    'i', '\x0a', '\x00' };

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
static double e_fprintf()
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[28] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'r', '_', 'w', 'c', '_', 'p', 'r',
    'e', 'd', '\x0a', '\x00' };

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
static double fb_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[29] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'h', '_', 'd', 'i', 'n', '_', 'l',
    ' ', '%', 'i', '\x0a', '\x00' };

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
static double g_fprintf()
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[23] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'r', '_', 'l', 'r', '\x0a', '\x00' };

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
static double hb_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[29] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'h', '_', 'd', 'i', 'n', '_', 'r',
    ' ', '%', 'i', '\x0a', '\x00' };

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
static double i_fprintf()
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[23] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'R', '_', 'l', 'r', '\x0a', '\x00' };

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
static double jb_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[28] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'h', '_', 'd', 'i', '_', 'l', ' ',
    '%', 'i', '\x0a', '\x00' };

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
static double k_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[34] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'h', '_', 'c', 'i', '_', 'l', '(',
    '3', ')', '=', '=', '0', ' ', '%', 'i', '\x0a', '\x00' };

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
static double lb_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[28] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'h', '_', 'd', 'i', '_', 'r', ' ',
    '%', 'i', '\x0a', '\x00' };

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
  static const char cfmt[34] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'h', '_', 'c', 'i', '_', 'r', '(',
    '3', ')', '=', '=', '0', ' ', '%', 'i', '\x0a', '\x00' };

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
static double nb_fprintf()
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[13] = { 'N', 'A', 'N', ' ', 'E', 'R', 'R', 'O', 'R',
    ' ', 'H', '\x0a', '\x00' };

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
static double o_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[29] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'h', '_', 'c', 'i', 'n', '_', 'l',
    ' ', '%', 'i', '\x0a', '\x00' };

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
static double pb_fprintf()
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[15] = { 'N', 'A', 'N', ' ', 'E', 'R', 'R', 'O', 'R',
    ' ', 'h', '_', 'u', '\x0a', '\x00' };

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
static double q_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[33] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'h', '_', 'c', 'i', '_', 'l', '(',
    '3', ')', ' ', '=', ' ', '%', 'f', '\x0a', '\x00' };

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
static double rb_fprintf()
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[13] = { 'N', 'A', 'N', ' ', 'E', 'R', 'R', 'O', 'R',
    ' ', 'r', '\x0a', '\x00' };

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
static double s_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[29] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'h', '_', 'c', 'i', 'n', '_', 'r',
    ' ', '%', 'i', '\x0a', '\x00' };

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
  static const char cfmt[24] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'x', '_', 'a', 'p', 'o', '\x0a',
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
static double u_fprintf(double varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[33] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'h', '_', 'c', 'i', '_', 'r', '(',
    '3', ')', ' ', '=', ' ', '%', 'f', '\x0a', '\x00' };

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
static double vb_fprintf()
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[20] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'K', '\x0a', '\x00' };

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
static double w_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[29] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'h', '_', 'u', 'i', 'n', '_', 'l',
    ' ', '%', 'i', '\x0a', '\x00' };

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
static double xb_fprintf()
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[20] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'S', '\x0a', '\x00' };

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
static double y_fprintf(int varargin_1)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[29] = { '#', '#', '#', '#', '#', '#', '#', '#', '#',
    '#', '#', '#', '#', '#', '#', '#', ' ', 'h', '_', 'u', 'i', 'n', '_', 'r',
    ' ', '%', 'i', '\x0a', '\x00' };

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
// Arguments    : int formatSpec
// Return Type  : void
//
void cb_fprintf(int formatSpec)
{
  db_fprintf(formatSpec);
}

//
// Arguments    : void
// Return Type  : void
//
void d_fprintf()
{
  e_fprintf();
}

//
// Arguments    : int formatSpec
// Return Type  : void
//
void eb_fprintf(int formatSpec)
{
  fb_fprintf(formatSpec);
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
void gb_fprintf(int formatSpec)
{
  hb_fprintf(formatSpec);
}

//
// Arguments    : void
// Return Type  : void
//
void h_fprintf()
{
  i_fprintf();
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
// Arguments    : int formatSpec
// Return Type  : void
//
void j_fprintf(int formatSpec)
{
  k_fprintf(formatSpec);
}

//
// Arguments    : int formatSpec
// Return Type  : void
//
void kb_fprintf(int formatSpec)
{
  lb_fprintf(formatSpec);
}

//
// Arguments    : int formatSpec
// Return Type  : void
//
void l_fprintf(int formatSpec)
{
  m_fprintf(formatSpec);
}

//
// Arguments    : void
// Return Type  : void
//
void mb_fprintf()
{
  nb_fprintf();
}

//
// Arguments    : int formatSpec
// Return Type  : void
//
void n_fprintf(int formatSpec)
{
  o_fprintf(formatSpec);
}

//
// Arguments    : void
// Return Type  : void
//
void ob_fprintf()
{
  pb_fprintf();
}

//
// Arguments    : double formatSpec
// Return Type  : void
//
void p_fprintf(double formatSpec)
{
  q_fprintf(formatSpec);
}

//
// Arguments    : void
// Return Type  : void
//
void qb_fprintf()
{
  rb_fprintf();
}

//
// Arguments    : int formatSpec
// Return Type  : void
//
void r_fprintf(int formatSpec)
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
// Arguments    : double formatSpec
// Return Type  : void
//
void t_fprintf(double formatSpec)
{
  u_fprintf(formatSpec);
}

//
// Arguments    : void
// Return Type  : void
//
void ub_fprintf()
{
  vb_fprintf();
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
// Arguments    : void
// Return Type  : void
//
void wb_fprintf()
{
  xb_fprintf();
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
// Arguments    : void
// Return Type  : void
//
void yb_fprintf()
{
  ac_fprintf();
}

//
// File trailer for fprintf.cpp
//
// [EOF]
//
