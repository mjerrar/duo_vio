//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fprintf.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 04-Sep-2015 22:40:16
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "fprintf.h"
#include "fileManager.h"
#include <ros/console.h>
#include <stdio.h>

// Function Declarations
static double c_fprintf(signed char varargin_1);
static double e_fprintf(signed char varargin_1);
static double g_fprintf(int varargin_1, int varargin_2);
static double i_fprintf();
static double k_fprintf(double varargin_1, double varargin_2, double varargin_3);
static double m_fprintf(double varargin_1, double varargin_2, double varargin_3,
  double varargin_4, double varargin_5, double varargin_6, double varargin_7,
  double varargin_8);

// Function Definitions

//
// Arguments    : signed char varargin_1
// Return Type  : double
//
static double c_fprintf(signed char varargin_1)
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
static double e_fprintf(signed char varargin_1)
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
static double g_fprintf(int varargin_1, int varargin_2)
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
static double i_fprintf()
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
// Arguments    : double varargin_1
//                double varargin_2
//                double varargin_3
// Return Type  : double
//
static double k_fprintf(double varargin_1, double varargin_2, double varargin_3)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[51] = { 'p', 'r', 'e', 'd', 'i', 'c', 't', 'i', 'n',
    'g', ' ', 'w', 'i', 't', 'h', ' ', 'c', 'o', 'n', 't', 'r', 'o', 'l', 'l',
    'e', 'r', '.', ' ', 'm', 'o', 'd', 'e', 'l', ':', ' ', '%', '.', '3', 'f',
    ' ', '%', '.', '3', 'f', ' ', '%', '.', '3', 'f', '\x0a', '\x00' };

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
//                double varargin_4
//                double varargin_5
//                double varargin_6
//                double varargin_7
//                double varargin_8
// Return Type  : double
//
static double m_fprintf(double varargin_1, double varargin_2, double varargin_3,
  double varargin_4, double varargin_5, double varargin_6, double varargin_7,
  double varargin_8)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[76] = { 'p', 'o', 's', 'i', 't', 'i', 'o', 'n', ' ',
    'e', 'r', 'r', 'o', 'r', ' ', '(', '%', '.', '3', 'f', ',', ' ', '%', '.',
    '3', 'f', ',', ' ', '%', '.', '3', 'f', ',', ' ', '%', '.', '3', 'f', ')',
    ',', ' ', 'c', 'o', 'n', 't', 'r', 'o', 'l', ':', ' ', '(', '%', '.', '3',
    'f', ',', ' ', '%', '.', '3', 'f', ',', ' ', '%', '.', '3', 'f', ',', ' ',
    '%', '.', '3', 'f', ')', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, cfmt, varargin_1, varargin_2, varargin_3,
                        varargin_4, varargin_5, varargin_6, varargin_7,
                        varargin_8);
    fflush(filestar);
  }

  return nbytesint;
}

//
// Arguments    : signed char formatSpec
// Return Type  : void
//
void b_fprintf(signed char formatSpec)
{
  c_fprintf(formatSpec);
}

//
// Arguments    : signed char formatSpec
// Return Type  : void
//
void d_fprintf(signed char formatSpec)
{
  e_fprintf(formatSpec);
}

//
// Arguments    : int formatSpec
//                int varargin_1
// Return Type  : void
//
void f_fprintf(int formatSpec, int varargin_1)
{
  g_fprintf(formatSpec, varargin_1);
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
// Arguments    : double formatSpec
//                double varargin_1
//                double varargin_2
// Return Type  : void
//
void j_fprintf(double formatSpec, double varargin_1, double varargin_2)
{
  k_fprintf(formatSpec, varargin_1, varargin_2);
}

//
// Arguments    : double formatSpec
//                double varargin_1
//                double varargin_2
//                double varargin_3
//                double varargin_4
//                double varargin_5
//                double varargin_6
//                double varargin_7
// Return Type  : void
//
void l_fprintf(double formatSpec, double varargin_1, double varargin_2, double
               varargin_3, double varargin_4, double varargin_5, double
               varargin_6, double varargin_7)
{
  m_fprintf(formatSpec, varargin_1, varargin_2, varargin_3, varargin_4,
            varargin_5, varargin_6, varargin_7);
}

//
// File trailer for fprintf.cpp
//
// [EOF]
//
