//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: kron.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 24-Aug-2015 21:00:43
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "kron.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double A_data[]
//                const int A_size[2]
//                const double B[4]
//                double K_data[]
//                int K_size[2]
// Return Type  : void
//
void kron(const double A_data[], const int A_size[2], const double B[4], double
          K_data[], int K_size[2])
{
  int kidx;
  int b_j1;
  int j2;
  int i1;
  int i2;
  K_size[0] = (signed char)(A_size[0] << 1);
  K_size[1] = (signed char)(A_size[1] << 1);
  kidx = -1;
  for (b_j1 = 1; b_j1 <= A_size[1]; b_j1++) {
    for (j2 = 0; j2 < 2; j2++) {
      for (i1 = 1; i1 <= A_size[0]; i1++) {
        for (i2 = 0; i2 < 2; i2++) {
          kidx++;
          K_data[kidx] = A_data[(i1 + A_size[0] * (b_j1 - 1)) - 1] * B[i2 + (j2 <<
            1)];
        }
      }
    }
  }
}

//
// File trailer for kron.cpp
//
// [EOF]
//
