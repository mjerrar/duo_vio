//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: det.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 06-Oct-2015 15:29:53
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "det.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double x[36]
// Return Type  : double
//
double det(const double x[36])
{
  double y;
  double A[36];
  signed char ipiv[6];
  int i23;
  int j;
  int c;
  int iy;
  int ix;
  double smax;
  int jy;
  double s;
  int b_j;
  int ijA;
  boolean_T isodd;
  memcpy(&A[0], &x[0], 36U * sizeof(double));
  for (i23 = 0; i23 < 6; i23++) {
    ipiv[i23] = (signed char)(1 + i23);
  }

  for (j = 0; j < 5; j++) {
    c = j * 7;
    iy = 0;
    ix = c;
    smax = fabs(A[c]);
    for (jy = 1; jy + 1 <= 6 - j; jy++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        iy = jy;
        smax = s;
      }
    }

    if (A[c + iy] != 0.0) {
      if (iy != 0) {
        ipiv[j] = (signed char)((j + iy) + 1);
        ix = j;
        iy += j;
        for (jy = 0; jy < 6; jy++) {
          smax = A[ix];
          A[ix] = A[iy];
          A[iy] = smax;
          ix += 6;
          iy += 6;
        }
      }

      i23 = (c - j) + 6;
      for (iy = c + 1; iy + 1 <= i23; iy++) {
        A[iy] /= A[c];
      }
    }

    iy = c;
    jy = c + 6;
    for (b_j = 1; b_j <= 5 - j; b_j++) {
      smax = A[jy];
      if (A[jy] != 0.0) {
        ix = c + 1;
        i23 = (iy - j) + 12;
        for (ijA = 7 + iy; ijA + 1 <= i23; ijA++) {
          A[ijA] += A[ix] * -smax;
          ix++;
        }
      }

      jy += 6;
      iy += 6;
    }
  }

  y = A[0];
  isodd = false;
  for (jy = 0; jy < 5; jy++) {
    y *= A[(jy + 6 * (jy + 1)) + 1];
    if (ipiv[jy] > 1 + jy) {
      isodd = !isodd;
    }
  }

  if (isodd) {
    y = -y;
  }

  return y;
}

//
// File trailer for det.cpp
//
// [EOF]
//
