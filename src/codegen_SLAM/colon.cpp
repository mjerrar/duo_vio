//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: colon.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 30-Aug-2015 16:19:29
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "colon.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// Arguments    : int b
//                int y_data[]
//                int y_size[2]
// Return Type  : void
//
void eml_signed_integer_colon(int b, int y_data[], int y_size[2])
{
  int n;
  int yk;
  int k;
  if (b < 1) {
    n = 0;
  } else {
    n = b;
  }

  y_size[0] = 1;
  y_size[1] = n;
  if (n > 0) {
    y_data[0] = 1;
    yk = 1;
    for (k = 2; k <= n; k++) {
      yk++;
      y_data[k - 1] = yk;
    }
  }
}

//
// File trailer for colon.cpp
//
// [EOF]
//
