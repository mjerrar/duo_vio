//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eig.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 19-Aug-2015 11:35:06
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "eig.h"
#include "initializePoint.h"
#include "sqrt.h"
#include "SLAM_rtwutil.h"
#include <stdio.h>

// Function Declarations
static void b_eml_matlab_zlartg(const creal_T f, const creal_T g, double *cs,
  creal_T *sn);
static int div_s32(int numerator, int denominator);
static void eml_matlab_zggev(creal_T A[16], int *info, creal_T alpha1[4],
  creal_T beta1[4], creal_T V[16]);
static void eml_matlab_zhgeqz(creal_T A[16], int ilo, int ihi, creal_T Z[16],
  int *info, creal_T alpha1[4], creal_T beta1[4]);
static void eml_matlab_zlartg(const creal_T f, const creal_T g, double *cs,
  creal_T *sn, creal_T *r);
static void eml_matlab_ztgevc(const creal_T A[16], creal_T V[16]);

// Function Definitions

//
// Arguments    : const creal_T f
//                const creal_T g
//                double *cs
//                creal_T *sn
// Return Type  : void
//
static void b_eml_matlab_zlartg(const creal_T f, const creal_T g, double *cs,
  creal_T *sn)
{
  double scale;
  double f2s;
  double g2;
  double fs_re;
  double fs_im;
  double gs_re;
  double gs_im;
  boolean_T guard1 = false;
  double g2s;
  scale = fabs(f.re);
  f2s = fabs(f.im);
  if (f2s > scale) {
    scale = f2s;
  }

  f2s = fabs(g.re);
  g2 = fabs(g.im);
  if (g2 > f2s) {
    f2s = g2;
  }

  if (f2s > scale) {
    scale = f2s;
  }

  fs_re = f.re;
  fs_im = f.im;
  gs_re = g.re;
  gs_im = g.im;
  guard1 = false;
  if (scale >= 7.4428285367870146E+137) {
    do {
      fs_re *= 1.3435752215134178E-138;
      fs_im *= 1.3435752215134178E-138;
      gs_re *= 1.3435752215134178E-138;
      gs_im *= 1.3435752215134178E-138;
      scale *= 1.3435752215134178E-138;
    } while (!(scale < 7.4428285367870146E+137));

    guard1 = true;
  } else if (scale <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
    } else {
      do {
        fs_re *= 7.4428285367870146E+137;
        fs_im *= 7.4428285367870146E+137;
        gs_re *= 7.4428285367870146E+137;
        gs_im *= 7.4428285367870146E+137;
        scale *= 7.4428285367870146E+137;
      } while (!(scale > 1.3435752215134178E-138));

      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    scale = fs_re * fs_re + fs_im * fs_im;
    g2 = gs_re * gs_re + gs_im * gs_im;
    f2s = g2;
    if (1.0 > g2) {
      f2s = 1.0;
    }

    if (scale <= f2s * 2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        scale = rt_hypotd_snf(gs_re, gs_im);
        sn->re = gs_re / scale;
        sn->im = -gs_im / scale;
      } else {
        g2s = sqrt(g2);
        *cs = rt_hypotd_snf(fs_re, fs_im) / g2s;
        f2s = fabs(f.re);
        g2 = fabs(f.im);
        if (g2 > f2s) {
          f2s = g2;
        }

        if (f2s > 1.0) {
          scale = rt_hypotd_snf(f.re, f.im);
          fs_re = f.re / scale;
          fs_im = f.im / scale;
        } else {
          f2s = 7.4428285367870146E+137 * f.re;
          g2 = 7.4428285367870146E+137 * f.im;
          scale = rt_hypotd_snf(f2s, g2);
          fs_re = f2s / scale;
          fs_im = g2 / scale;
        }

        gs_re /= g2s;
        gs_im = -gs_im / g2s;
        sn->re = fs_re * gs_re - fs_im * gs_im;
        sn->im = fs_re * gs_im + fs_im * gs_re;
      }
    } else {
      f2s = sqrt(1.0 + g2 / scale);
      *cs = 1.0 / f2s;
      scale += g2;
      fs_re = f2s * fs_re / scale;
      fs_im = f2s * fs_im / scale;
      sn->re = fs_re * gs_re - fs_im * -gs_im;
      sn->im = fs_re * -gs_im + fs_im * gs_re;
    }
  }
}

//
// Arguments    : int numerator
//                int denominator
// Return Type  : int
//
static int div_s32(int numerator, int denominator)
{
  int quotient;
  unsigned int absNumerator;
  unsigned int absDenominator;
  boolean_T quotientNeedsNegation;
  if (denominator == 0) {
    if (numerator >= 0) {
      quotient = MAX_int32_T;
    } else {
      quotient = MIN_int32_T;
    }
  } else {
    if (numerator >= 0) {
      absNumerator = (unsigned int)numerator;
    } else {
      absNumerator = (unsigned int)-numerator;
    }

    if (denominator >= 0) {
      absDenominator = (unsigned int)denominator;
    } else {
      absDenominator = (unsigned int)-denominator;
    }

    quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
    absNumerator /= absDenominator;
    if (quotientNeedsNegation) {
      quotient = -(int)absNumerator;
    } else {
      quotient = (int)absNumerator;
    }
  }

  return quotient;
}

//
// Arguments    : creal_T A[16]
//                int *info
//                creal_T alpha1[4]
//                creal_T beta1[4]
//                creal_T V[16]
// Return Type  : void
//
static void eml_matlab_zggev(creal_T A[16], int *info, creal_T alpha1[4],
  creal_T beta1[4], creal_T V[16])
{
  double anrm;
  int nzcount;
  boolean_T exitg7;
  double absxk;
  int i;
  boolean_T ilascl;
  double anrmto;
  double ctoc;
  boolean_T notdone;
  double cfrom1;
  double stemp_re;
  double stemp_im;
  creal_T b_A[16];
  int rscale[4];
  int ilo;
  int ihi;
  int32_T exitg2;
  int j;
  int ii;
  boolean_T exitg5;
  int jj;
  boolean_T exitg6;
  boolean_T guard2 = false;
  creal_T atmp;
  int32_T exitg1;
  boolean_T exitg3;
  boolean_T exitg4;
  boolean_T guard1 = false;
  static const signed char iv2[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 1 };

  *info = 0;
  anrm = 0.0;
  nzcount = 0;
  exitg7 = false;
  while ((!exitg7) && (nzcount < 16)) {
    absxk = rt_hypotd_snf(A[nzcount].re, A[nzcount].im);
    if (rtIsNaN(absxk)) {
      anrm = rtNaN;
      exitg7 = true;
    } else {
      if (absxk > anrm) {
        anrm = absxk;
      }

      nzcount++;
    }
  }

  if (!((!rtIsInf(anrm)) && (!rtIsNaN(anrm)))) {
    for (i = 0; i < 4; i++) {
      alpha1[i].re = rtNaN;
      alpha1[i].im = 0.0;
      beta1[i].re = rtNaN;
      beta1[i].im = 0.0;
    }

    for (nzcount = 0; nzcount < 16; nzcount++) {
      V[nzcount].re = rtNaN;
      V[nzcount].im = 0.0;
    }
  } else {
    ilascl = false;
    anrmto = anrm;
    if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
      anrmto = 6.7178761075670888E-139;
      ilascl = true;
    } else {
      if (anrm > 1.4885657073574029E+138) {
        anrmto = 1.4885657073574029E+138;
        ilascl = true;
      }
    }

    if (ilascl) {
      absxk = anrm;
      ctoc = anrmto;
      notdone = true;
      while (notdone) {
        cfrom1 = absxk * 2.0041683600089728E-292;
        stemp_re = ctoc / 4.9896007738368E+291;
        if ((cfrom1 > ctoc) && (ctoc != 0.0)) {
          stemp_im = 2.0041683600089728E-292;
          absxk = cfrom1;
        } else if (stemp_re > absxk) {
          stemp_im = 4.9896007738368E+291;
          ctoc = stemp_re;
        } else {
          stemp_im = ctoc / absxk;
          notdone = false;
        }

        for (nzcount = 0; nzcount < 16; nzcount++) {
          A[nzcount].re *= stemp_im;
          A[nzcount].im *= stemp_im;
        }
      }
    }

    memcpy(&b_A[0], &A[0], sizeof(creal_T) << 4);
    for (i = 0; i < 4; i++) {
      rscale[i] = 0;
    }

    ilo = 0;
    ihi = 4;
    do {
      exitg2 = 0;
      i = 0;
      j = 0;
      notdone = false;
      ii = ihi;
      exitg5 = false;
      while ((!exitg5) && (ii > 0)) {
        nzcount = 0;
        i = ii;
        j = ihi;
        jj = 1;
        exitg6 = false;
        while ((!exitg6) && (jj <= ihi)) {
          guard2 = false;
          if ((b_A[(ii + ((jj - 1) << 2)) - 1].re != 0.0) || (b_A[(ii + ((jj - 1)
                 << 2)) - 1].im != 0.0) || (ii == jj)) {
            if (nzcount == 0) {
              j = jj;
              nzcount = 1;
              guard2 = true;
            } else {
              nzcount = 2;
              exitg6 = true;
            }
          } else {
            guard2 = true;
          }

          if (guard2) {
            jj++;
          }
        }

        if (nzcount < 2) {
          notdone = true;
          exitg5 = true;
        } else {
          ii--;
        }
      }

      if (!notdone) {
        exitg2 = 2;
      } else {
        if (i != ihi) {
          for (nzcount = 0; nzcount < 4; nzcount++) {
            atmp = b_A[(i + (nzcount << 2)) - 1];
            b_A[(i + (nzcount << 2)) - 1] = b_A[(ihi + (nzcount << 2)) - 1];
            b_A[(ihi + (nzcount << 2)) - 1] = atmp;
          }
        }

        if (j != ihi) {
          for (nzcount = 0; nzcount + 1 <= ihi; nzcount++) {
            atmp = b_A[nzcount + ((j - 1) << 2)];
            b_A[nzcount + ((j - 1) << 2)] = b_A[nzcount + ((ihi - 1) << 2)];
            b_A[nzcount + ((ihi - 1) << 2)] = atmp;
          }
        }

        rscale[ihi - 1] = j;
        ihi--;
        if (ihi == 1) {
          rscale[0] = 1;
          exitg2 = 1;
        }
      }
    } while (exitg2 == 0);

    if (exitg2 == 1) {
    } else {
      do {
        exitg1 = 0;
        i = 0;
        j = 0;
        notdone = false;
        jj = ilo + 1;
        exitg3 = false;
        while ((!exitg3) && (jj <= ihi)) {
          nzcount = 0;
          i = ihi;
          j = jj;
          ii = ilo + 1;
          exitg4 = false;
          while ((!exitg4) && (ii <= ihi)) {
            guard1 = false;
            if ((b_A[(ii + ((jj - 1) << 2)) - 1].re != 0.0) || (b_A[(ii + ((jj -
                    1) << 2)) - 1].im != 0.0) || (ii == jj)) {
              if (nzcount == 0) {
                i = ii;
                nzcount = 1;
                guard1 = true;
              } else {
                nzcount = 2;
                exitg4 = true;
              }
            } else {
              guard1 = true;
            }

            if (guard1) {
              ii++;
            }
          }

          if (nzcount < 2) {
            notdone = true;
            exitg3 = true;
          } else {
            jj++;
          }
        }

        if (!notdone) {
          exitg1 = 1;
        } else {
          if (i != ilo + 1) {
            for (nzcount = ilo; nzcount + 1 < 5; nzcount++) {
              atmp = b_A[(i + (nzcount << 2)) - 1];
              b_A[(i + (nzcount << 2)) - 1] = b_A[ilo + (nzcount << 2)];
              b_A[ilo + (nzcount << 2)] = atmp;
            }
          }

          if (j != ilo + 1) {
            for (nzcount = 0; nzcount + 1 <= ihi; nzcount++) {
              atmp = b_A[nzcount + ((j - 1) << 2)];
              b_A[nzcount + ((j - 1) << 2)] = b_A[nzcount + (ilo << 2)];
              b_A[nzcount + (ilo << 2)] = atmp;
            }
          }

          rscale[ilo] = j;
          ilo++;
          if (ilo + 1 == ihi) {
            rscale[ilo] = ilo + 1;
            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
    }

    for (nzcount = 0; nzcount < 16; nzcount++) {
      V[nzcount].re = iv2[nzcount];
      V[nzcount].im = 0.0;
    }

    if (ihi < ilo + 3) {
    } else {
      for (ii = ilo; ii + 1 < ihi - 1; ii++) {
        for (nzcount = ihi - 1; nzcount + 1 > ii + 2; nzcount--) {
          eml_matlab_zlartg(b_A[(nzcount + (ii << 2)) - 1], b_A[nzcount + (ii <<
            2)], &cfrom1, &atmp, &b_A[(nzcount + (ii << 2)) - 1]);
          b_A[nzcount + (ii << 2)].re = 0.0;
          b_A[nzcount + (ii << 2)].im = 0.0;
          for (j = ii + 1; j + 1 <= ihi; j++) {
            stemp_re = cfrom1 * b_A[(nzcount + (j << 2)) - 1].re + (atmp.re *
              b_A[nzcount + (j << 2)].re - atmp.im * b_A[nzcount + (j << 2)].im);
            stemp_im = cfrom1 * b_A[(nzcount + (j << 2)) - 1].im + (atmp.re *
              b_A[nzcount + (j << 2)].im + atmp.im * b_A[nzcount + (j << 2)].re);
            absxk = b_A[(nzcount + (j << 2)) - 1].im;
            ctoc = b_A[(nzcount + (j << 2)) - 1].re;
            b_A[nzcount + (j << 2)].re = cfrom1 * b_A[nzcount + (j << 2)].re -
              (atmp.re * b_A[(nzcount + (j << 2)) - 1].re + atmp.im * b_A
               [(nzcount + (j << 2)) - 1].im);
            b_A[nzcount + (j << 2)].im = cfrom1 * b_A[nzcount + (j << 2)].im -
              (atmp.re * absxk - atmp.im * ctoc);
            b_A[(nzcount + (j << 2)) - 1].re = stemp_re;
            b_A[(nzcount + (j << 2)) - 1].im = stemp_im;
          }

          atmp.re = -atmp.re;
          atmp.im = -atmp.im;
          for (i = ilo; i + 1 <= ihi; i++) {
            stemp_re = cfrom1 * b_A[i + (nzcount << 2)].re + (atmp.re * b_A[i +
              ((nzcount - 1) << 2)].re - atmp.im * b_A[i + ((nzcount - 1) << 2)]
              .im);
            stemp_im = cfrom1 * b_A[i + (nzcount << 2)].im + (atmp.re * b_A[i +
              ((nzcount - 1) << 2)].im + atmp.im * b_A[i + ((nzcount - 1) << 2)]
              .re);
            absxk = b_A[i + (nzcount << 2)].im;
            ctoc = b_A[i + (nzcount << 2)].re;
            b_A[i + ((nzcount - 1) << 2)].re = cfrom1 * b_A[i + ((nzcount - 1) <<
              2)].re - (atmp.re * b_A[i + (nzcount << 2)].re + atmp.im * b_A[i +
                        (nzcount << 2)].im);
            b_A[i + ((nzcount - 1) << 2)].im = cfrom1 * b_A[i + ((nzcount - 1) <<
              2)].im - (atmp.re * absxk - atmp.im * ctoc);
            b_A[i + (nzcount << 2)].re = stemp_re;
            b_A[i + (nzcount << 2)].im = stemp_im;
          }

          for (i = 0; i < 4; i++) {
            stemp_re = cfrom1 * V[i + (nzcount << 2)].re + (atmp.re * V[i +
              ((nzcount - 1) << 2)].re - atmp.im * V[i + ((nzcount - 1) << 2)].
              im);
            stemp_im = cfrom1 * V[i + (nzcount << 2)].im + (atmp.re * V[i +
              ((nzcount - 1) << 2)].im + atmp.im * V[i + ((nzcount - 1) << 2)].
              re);
            absxk = V[i + (nzcount << 2)].im;
            ctoc = V[i + (nzcount << 2)].re;
            V[i + ((nzcount - 1) << 2)].re = cfrom1 * V[i + ((nzcount - 1) << 2)]
              .re - (atmp.re * V[i + (nzcount << 2)].re + atmp.im * V[i +
                     (nzcount << 2)].im);
            V[i + ((nzcount - 1) << 2)].im = cfrom1 * V[i + ((nzcount - 1) << 2)]
              .im - (atmp.re * absxk - atmp.im * ctoc);
            V[i + (nzcount << 2)].re = stemp_re;
            V[i + (nzcount << 2)].im = stemp_im;
          }
        }
      }
    }

    eml_matlab_zhgeqz(b_A, ilo + 1, ihi, V, info, alpha1, beta1);
    if (*info != 0) {
    } else {
      eml_matlab_ztgevc(b_A, V);
      if (ilo + 1 > 1) {
        for (i = ilo - 1; i + 1 >= 1; i--) {
          nzcount = rscale[i] - 1;
          if (rscale[i] != i + 1) {
            for (j = 0; j < 4; j++) {
              atmp = V[i + (j << 2)];
              V[i + (j << 2)] = V[nzcount + (j << 2)];
              V[nzcount + (j << 2)] = atmp;
            }
          }
        }
      }

      if (ihi < 4) {
        while (ihi + 1 < 5) {
          nzcount = rscale[ihi] - 1;
          if (rscale[ihi] != ihi + 1) {
            for (j = 0; j < 4; j++) {
              atmp = V[ihi + (j << 2)];
              V[ihi + (j << 2)] = V[nzcount + (j << 2)];
              V[nzcount + (j << 2)] = atmp;
            }
          }

          ihi++;
        }
      }

      for (nzcount = 0; nzcount < 4; nzcount++) {
        absxk = fabs(V[nzcount << 2].re) + fabs(V[nzcount << 2].im);
        for (ii = 0; ii < 3; ii++) {
          ctoc = fabs(V[(ii + (nzcount << 2)) + 1].re) + fabs(V[(ii + (nzcount <<
            2)) + 1].im);
          if (ctoc > absxk) {
            absxk = ctoc;
          }
        }

        if (absxk >= 6.7178761075670888E-139) {
          absxk = 1.0 / absxk;
          for (ii = 0; ii < 4; ii++) {
            V[ii + (nzcount << 2)].re *= absxk;
            V[ii + (nzcount << 2)].im *= absxk;
          }
        }
      }

      if (ilascl) {
        notdone = true;
        while (notdone) {
          cfrom1 = anrmto * 2.0041683600089728E-292;
          stemp_re = anrm / 4.9896007738368E+291;
          if ((cfrom1 > anrm) && (anrm != 0.0)) {
            stemp_im = 2.0041683600089728E-292;
            anrmto = cfrom1;
          } else if (stemp_re > anrmto) {
            stemp_im = 4.9896007738368E+291;
            anrm = stemp_re;
          } else {
            stemp_im = anrm / anrmto;
            notdone = false;
          }

          for (nzcount = 0; nzcount < 4; nzcount++) {
            alpha1[nzcount].re *= stemp_im;
            alpha1[nzcount].im *= stemp_im;
          }
        }
      }
    }
  }
}

//
// Arguments    : creal_T A[16]
//                int ilo
//                int ihi
//                creal_T Z[16]
//                int *info
//                creal_T alpha1[4]
//                creal_T beta1[4]
// Return Type  : void
//
static void eml_matlab_zhgeqz(creal_T A[16], int ilo, int ihi, creal_T Z[16],
  int *info, creal_T alpha1[4], creal_T beta1[4])
{
  int i;
  double eshift_re;
  double eshift_im;
  creal_T ctemp;
  double rho_re;
  double rho_im;
  double anorm;
  double sumsq;
  boolean_T firstNonZero;
  int j;
  int jp1;
  double reAij;
  double imAij;
  double temp1;
  double temp2;
  double b_atol;
  boolean_T failed;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  int ifirst;
  int istart;
  int ilast;
  int ilastm1;
  int iiter;
  boolean_T goto60;
  boolean_T goto70;
  boolean_T goto90;
  int jiter;
  int32_T exitg1;
  boolean_T exitg3;
  boolean_T b_guard1 = false;
  creal_T t1;
  creal_T d;
  boolean_T exitg2;
  *info = 0;
  for (i = 0; i < 4; i++) {
    alpha1[i].re = 0.0;
    alpha1[i].im = 0.0;
    beta1[i].re = 1.0;
    beta1[i].im = 0.0;
  }

  eshift_re = 0.0;
  eshift_im = 0.0;
  ctemp.re = 0.0;
  ctemp.im = 0.0;
  rho_re = 0.0;
  rho_im = 0.0;
  anorm = 0.0;
  if (ilo > ihi) {
  } else {
    anorm = 0.0;
    sumsq = 0.0;
    firstNonZero = true;
    for (j = ilo; j <= ihi; j++) {
      jp1 = j + 1;
      if (ihi < j + 1) {
        jp1 = ihi;
      }

      for (i = ilo; i <= jp1; i++) {
        reAij = A[(i + ((j - 1) << 2)) - 1].re;
        imAij = A[(i + ((j - 1) << 2)) - 1].im;
        if (reAij != 0.0) {
          temp1 = fabs(reAij);
          if (firstNonZero) {
            sumsq = 1.0;
            anorm = temp1;
            firstNonZero = false;
          } else if (anorm < temp1) {
            temp2 = anorm / temp1;
            sumsq = 1.0 + sumsq * temp2 * temp2;
            anorm = temp1;
          } else {
            temp2 = temp1 / anorm;
            sumsq += temp2 * temp2;
          }
        }

        if (imAij != 0.0) {
          temp1 = fabs(imAij);
          if (firstNonZero) {
            sumsq = 1.0;
            anorm = temp1;
            firstNonZero = false;
          } else if (anorm < temp1) {
            temp2 = anorm / temp1;
            sumsq = 1.0 + sumsq * temp2 * temp2;
            anorm = temp1;
          } else {
            temp2 = temp1 / anorm;
            sumsq += temp2 * temp2;
          }
        }
      }
    }

    anorm *= sqrt(sumsq);
  }

  reAij = 2.2204460492503131E-16 * anorm;
  b_atol = 2.2250738585072014E-308;
  if (reAij > 2.2250738585072014E-308) {
    b_atol = reAij;
  }

  temp1 = 2.2250738585072014E-308;
  if (anorm > 2.2250738585072014E-308) {
    temp1 = anorm;
  }

  imAij = 1.0 / temp1;
  failed = true;
  for (j = ihi; j + 1 < 5; j++) {
    alpha1[j] = A[j + (j << 2)];
  }

  guard1 = false;
  guard2 = false;
  if (ihi >= ilo) {
    ifirst = ilo;
    istart = ilo;
    ilast = ihi - 1;
    ilastm1 = ihi - 2;
    iiter = 0;
    goto60 = false;
    goto70 = false;
    goto90 = false;
    jiter = 1;
    do {
      exitg1 = 0;
      if (jiter <= 30 * ((ihi - ilo) + 1)) {
        if (ilast + 1 == ilo) {
          goto60 = true;
        } else if (fabs(A[ilast + (ilastm1 << 2)].re) + fabs(A[ilast + (ilastm1 <<
          2)].im) <= b_atol) {
          A[ilast + (ilastm1 << 2)].re = 0.0;
          A[ilast + (ilastm1 << 2)].im = 0.0;
          goto60 = true;
        } else {
          j = ilastm1;
          exitg3 = false;
          while ((!exitg3) && (j + 1 >= ilo)) {
            if (j + 1 == ilo) {
              firstNonZero = true;
            } else if (fabs(A[j + ((j - 1) << 2)].re) + fabs(A[j + ((j - 1) << 2)]
                        .im) <= b_atol) {
              A[j + ((j - 1) << 2)].re = 0.0;
              A[j + ((j - 1) << 2)].im = 0.0;
              firstNonZero = true;
            } else {
              firstNonZero = false;
            }

            if (firstNonZero) {
              ifirst = j + 1;
              goto70 = true;
              exitg3 = true;
            } else {
              j--;
            }
          }
        }

        if (goto60 || goto70) {
          firstNonZero = true;
        } else {
          firstNonZero = false;
        }

        if (!firstNonZero) {
          for (i = 0; i < 4; i++) {
            alpha1[i].re = rtNaN;
            alpha1[i].im = 0.0;
            beta1[i].re = rtNaN;
            beta1[i].im = 0.0;
          }

          for (jp1 = 0; jp1 < 16; jp1++) {
            Z[jp1].re = rtNaN;
            Z[jp1].im = 0.0;
          }

          *info = 1;
          exitg1 = 1;
        } else {
          b_guard1 = false;
          if (goto60) {
            goto60 = false;
            alpha1[ilast] = A[ilast + (ilast << 2)];
            ilast = ilastm1;
            ilastm1--;
            if (ilast + 1 < ilo) {
              failed = false;
              guard2 = true;
              exitg1 = 1;
            } else {
              iiter = 0;
              eshift_re = 0.0;
              eshift_im = 0.0;
              b_guard1 = true;
            }
          } else {
            if (goto70) {
              goto70 = false;
              iiter++;
              if (iiter - div_s32(iiter, 10) * 10 != 0) {
                reAij = -(A[ilast + (ilast << 2)].re - A[ilastm1 + (ilastm1 << 2)]
                          .re);
                temp1 = -(A[ilast + (ilast << 2)].im - A[ilastm1 + (ilastm1 << 2)]
                          .im);
                if (temp1 == 0.0) {
                  t1.re = reAij / 2.0;
                  t1.im = 0.0;
                } else if (reAij == 0.0) {
                  t1.re = 0.0;
                  t1.im = temp1 / 2.0;
                } else {
                  t1.re = reAij / 2.0;
                  t1.im = temp1 / 2.0;
                }

                d.re = (t1.re * t1.re - t1.im * t1.im) + (A[ilastm1 + (ilast <<
                  2)].re * A[ilast + (ilastm1 << 2)].re - A[ilastm1 + (ilast <<
                  2)].im * A[ilast + (ilastm1 << 2)].im);
                d.im = (t1.re * t1.im + t1.im * t1.re) + (A[ilastm1 + (ilast <<
                  2)].re * A[ilast + (ilastm1 << 2)].im + A[ilastm1 + (ilast <<
                  2)].im * A[ilast + (ilastm1 << 2)].re);
                b_sqrt(&d);
                rho_re = A[ilastm1 + (ilastm1 << 2)].re - (t1.re - d.re);
                rho_im = A[ilastm1 + (ilastm1 << 2)].im - (t1.im - d.im);
                anorm = A[ilastm1 + (ilastm1 << 2)].re - (t1.re + d.re);
                reAij = A[ilastm1 + (ilastm1 << 2)].im - (t1.im + d.im);
                if (rt_hypotd_snf(rho_re - A[ilast + (ilast << 2)].re, rho_im -
                                  A[ilast + (ilast << 2)].im) <= rt_hypotd_snf
                    (anorm - A[ilast + (ilast << 2)].re, reAij - A[ilast +
                     (ilast << 2)].im)) {
                  anorm = rho_re;
                  reAij = rho_im;
                  rho_re = t1.re - d.re;
                  rho_im = t1.im - d.im;
                } else {
                  rho_re = t1.re + d.re;
                  rho_im = t1.im + d.im;
                }
              } else {
                eshift_re += A[ilast + (ilastm1 << 2)].re;
                eshift_im += A[ilast + (ilastm1 << 2)].im;
                anorm = eshift_re;
                reAij = eshift_im;
              }

              j = ilastm1;
              jp1 = ilastm1 + 1;
              exitg2 = false;
              while ((!exitg2) && (j + 1 > ifirst)) {
                istart = j + 1;
                ctemp.re = A[j + (j << 2)].re - anorm;
                ctemp.im = A[j + (j << 2)].im - reAij;
                temp1 = imAij * (fabs(ctemp.re) + fabs(ctemp.im));
                temp2 = imAij * (fabs(A[jp1 + (j << 2)].re) + fabs(A[jp1 + (j <<
                  2)].im));
                sumsq = temp1;
                if (temp2 > temp1) {
                  sumsq = temp2;
                }

                if ((sumsq < 1.0) && (sumsq != 0.0)) {
                  temp1 /= sumsq;
                  temp2 /= sumsq;
                }

                if ((fabs(A[j + ((j - 1) << 2)].re) + fabs(A[j + ((j - 1) << 2)]
                      .im)) * temp2 <= temp1 * b_atol) {
                  goto90 = true;
                  exitg2 = true;
                } else {
                  jp1 = j;
                  j--;
                }
              }

              if (!goto90) {
                istart = ifirst;
                if (ifirst == ilastm1 + 1) {
                  ctemp.re = rho_re;
                  ctemp.im = rho_im;
                } else {
                  ctemp.re = A[(ifirst + ((ifirst - 1) << 2)) - 1].re - anorm;
                  ctemp.im = A[(ifirst + ((ifirst - 1) << 2)) - 1].im - reAij;
                }

                goto90 = true;
              }
            }

            if (goto90) {
              goto90 = false;
              b_eml_matlab_zlartg(ctemp, A[istart + ((istart - 1) << 2)], &temp1,
                                  &t1);
              j = istart;
              jp1 = istart - 2;
              while (j < ilast + 1) {
                if (j > istart) {
                  eml_matlab_zlartg(A[(j + (jp1 << 2)) - 1], A[j + (jp1 << 2)],
                                    &temp1, &t1, &A[(j + (jp1 << 2)) - 1]);
                  A[j + (jp1 << 2)].re = 0.0;
                  A[j + (jp1 << 2)].im = 0.0;
                }

                for (jp1 = j - 1; jp1 + 1 < 5; jp1++) {
                  d.re = temp1 * A[(j + (jp1 << 2)) - 1].re + (t1.re * A[j +
                    (jp1 << 2)].re - t1.im * A[j + (jp1 << 2)].im);
                  d.im = temp1 * A[(j + (jp1 << 2)) - 1].im + (t1.re * A[j +
                    (jp1 << 2)].im + t1.im * A[j + (jp1 << 2)].re);
                  anorm = A[(j + (jp1 << 2)) - 1].im;
                  reAij = A[(j + (jp1 << 2)) - 1].re;
                  A[j + (jp1 << 2)].re = temp1 * A[j + (jp1 << 2)].re - (t1.re *
                    A[(j + (jp1 << 2)) - 1].re + t1.im * A[(j + (jp1 << 2)) - 1]
                    .im);
                  A[j + (jp1 << 2)].im = temp1 * A[j + (jp1 << 2)].im - (t1.re *
                    anorm - t1.im * reAij);
                  A[(j + (jp1 << 2)) - 1] = d;
                }

                t1.re = -t1.re;
                t1.im = -t1.im;
                jp1 = j;
                if (ilast + 1 < j + 2) {
                  jp1 = ilast - 1;
                }

                for (i = 0; i + 1 <= jp1 + 2; i++) {
                  d.re = temp1 * A[i + (j << 2)].re + (t1.re * A[i + ((j - 1) <<
                    2)].re - t1.im * A[i + ((j - 1) << 2)].im);
                  d.im = temp1 * A[i + (j << 2)].im + (t1.re * A[i + ((j - 1) <<
                    2)].im + t1.im * A[i + ((j - 1) << 2)].re);
                  anorm = A[i + (j << 2)].im;
                  reAij = A[i + (j << 2)].re;
                  A[i + ((j - 1) << 2)].re = temp1 * A[i + ((j - 1) << 2)].re -
                    (t1.re * A[i + (j << 2)].re + t1.im * A[i + (j << 2)].im);
                  A[i + ((j - 1) << 2)].im = temp1 * A[i + ((j - 1) << 2)].im -
                    (t1.re * anorm - t1.im * reAij);
                  A[i + (j << 2)] = d;
                }

                for (i = 0; i < 4; i++) {
                  d.re = temp1 * Z[i + (j << 2)].re + (t1.re * Z[i + ((j - 1) <<
                    2)].re - t1.im * Z[i + ((j - 1) << 2)].im);
                  d.im = temp1 * Z[i + (j << 2)].im + (t1.re * Z[i + ((j - 1) <<
                    2)].im + t1.im * Z[i + ((j - 1) << 2)].re);
                  anorm = Z[i + (j << 2)].im;
                  reAij = Z[i + (j << 2)].re;
                  Z[i + ((j - 1) << 2)].re = temp1 * Z[i + ((j - 1) << 2)].re -
                    (t1.re * Z[i + (j << 2)].re + t1.im * Z[i + (j << 2)].im);
                  Z[i + ((j - 1) << 2)].im = temp1 * Z[i + ((j - 1) << 2)].im -
                    (t1.re * anorm - t1.im * reAij);
                  Z[i + (j << 2)] = d;
                }

                jp1 = j - 1;
                j++;
              }
            }

            b_guard1 = true;
          }

          if (b_guard1) {
            jiter++;
          }
        }
      } else {
        guard2 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  } else {
    guard1 = true;
  }

  if (guard2) {
    if (failed) {
      *info = ilast + 1;
      for (jp1 = 0; jp1 + 1 <= ilast + 1; jp1++) {
        alpha1[jp1].re = rtNaN;
        alpha1[jp1].im = 0.0;
        beta1[jp1].re = rtNaN;
        beta1[jp1].im = 0.0;
      }

      for (jp1 = 0; jp1 < 16; jp1++) {
        Z[jp1].re = rtNaN;
        Z[jp1].im = 0.0;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    for (j = 0; j + 1 < ilo; j++) {
      alpha1[j] = A[j + (j << 2)];
    }
  }
}

//
// Arguments    : const creal_T f
//                const creal_T g
//                double *cs
//                creal_T *sn
//                creal_T *r
// Return Type  : void
//
static void eml_matlab_zlartg(const creal_T f, const creal_T g, double *cs,
  creal_T *sn, creal_T *r)
{
  double scale;
  double f2s;
  double g2;
  double fs_re;
  double fs_im;
  double gs_re;
  double gs_im;
  int count;
  int rescaledir;
  boolean_T guard1 = false;
  double g2s;
  scale = fabs(f.re);
  f2s = fabs(f.im);
  if (f2s > scale) {
    scale = f2s;
  }

  f2s = fabs(g.re);
  g2 = fabs(g.im);
  if (g2 > f2s) {
    f2s = g2;
  }

  if (f2s > scale) {
    scale = f2s;
  }

  fs_re = f.re;
  fs_im = f.im;
  gs_re = g.re;
  gs_im = g.im;
  count = 0;
  rescaledir = 0;
  guard1 = false;
  if (scale >= 7.4428285367870146E+137) {
    do {
      count++;
      fs_re *= 1.3435752215134178E-138;
      fs_im *= 1.3435752215134178E-138;
      gs_re *= 1.3435752215134178E-138;
      gs_im *= 1.3435752215134178E-138;
      scale *= 1.3435752215134178E-138;
    } while (!(scale < 7.4428285367870146E+137));

    rescaledir = 1;
    guard1 = true;
  } else if (scale <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
      *r = f;
    } else {
      do {
        count++;
        fs_re *= 7.4428285367870146E+137;
        fs_im *= 7.4428285367870146E+137;
        gs_re *= 7.4428285367870146E+137;
        gs_im *= 7.4428285367870146E+137;
        scale *= 7.4428285367870146E+137;
      } while (!(scale > 1.3435752215134178E-138));

      rescaledir = -1;
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    scale = fs_re * fs_re + fs_im * fs_im;
    g2 = gs_re * gs_re + gs_im * gs_im;
    f2s = g2;
    if (1.0 > g2) {
      f2s = 1.0;
    }

    if (scale <= f2s * 2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        r->re = rt_hypotd_snf(g.re, g.im);
        r->im = 0.0;
        f2s = rt_hypotd_snf(gs_re, gs_im);
        sn->re = gs_re / f2s;
        sn->im = -gs_im / f2s;
      } else {
        g2s = sqrt(g2);
        *cs = rt_hypotd_snf(fs_re, fs_im) / g2s;
        f2s = fabs(f.re);
        g2 = fabs(f.im);
        if (g2 > f2s) {
          f2s = g2;
        }

        if (f2s > 1.0) {
          f2s = rt_hypotd_snf(f.re, f.im);
          fs_re = f.re / f2s;
          fs_im = f.im / f2s;
        } else {
          g2 = 7.4428285367870146E+137 * f.re;
          scale = 7.4428285367870146E+137 * f.im;
          f2s = rt_hypotd_snf(g2, scale);
          fs_re = g2 / f2s;
          fs_im = scale / f2s;
        }

        gs_re /= g2s;
        gs_im = -gs_im / g2s;
        sn->re = fs_re * gs_re - fs_im * gs_im;
        sn->im = fs_re * gs_im + fs_im * gs_re;
        r->re = *cs * f.re + (sn->re * g.re - sn->im * g.im);
        r->im = *cs * f.im + (sn->re * g.im + sn->im * g.re);
      }
    } else {
      f2s = sqrt(1.0 + g2 / scale);
      r->re = f2s * fs_re;
      r->im = f2s * fs_im;
      *cs = 1.0 / f2s;
      f2s = scale + g2;
      g2 = r->re / f2s;
      f2s = r->im / f2s;
      sn->re = g2 * gs_re - f2s * -gs_im;
      sn->im = g2 * -gs_im + f2s * gs_re;
      if (rescaledir > 0) {
        for (rescaledir = 1; rescaledir <= count; rescaledir++) {
          r->re *= 7.4428285367870146E+137;
          r->im *= 7.4428285367870146E+137;
        }
      } else {
        if (rescaledir < 0) {
          for (rescaledir = 1; rescaledir <= count; rescaledir++) {
            r->re *= 1.3435752215134178E-138;
            r->im *= 1.3435752215134178E-138;
          }
        }
      }
    }
  }
}

//
// Arguments    : const creal_T A[16]
//                creal_T V[16]
// Return Type  : void
//
static void eml_matlab_ztgevc(const creal_T A[16], creal_T V[16])
{
  double rworka[4];
  int j;
  double anorm;
  int b_j;
  double d_re;
  double scale;
  double ascale;
  int je;
  double temp;
  double salpha_re;
  double salpha_im;
  double acoeff;
  boolean_T b3;
  boolean_T b4;
  double y;
  double acoefa;
  creal_T work1[4];
  int jr;
  double dmin;
  double d_im;
  double work1_re;
  double work1_im;
  creal_T work2[4];
  for (j = 0; j < 4; j++) {
    rworka[j] = 0.0;
  }

  anorm = fabs(A[0].re) + fabs(A[0].im);
  for (b_j = 0; b_j < 3; b_j++) {
    for (j = 0; j <= b_j; j++) {
      rworka[b_j + 1] += fabs(A[j + ((b_j + 1) << 2)].re) + fabs(A[j + ((b_j + 1)
        << 2)].im);
    }

    d_re = rworka[b_j + 1] + (fabs(A[(b_j + ((b_j + 1) << 2)) + 1].re) + fabs(A
      [(b_j + ((b_j + 1) << 2)) + 1].im));
    if (d_re > anorm) {
      anorm = d_re;
    }
  }

  scale = anorm;
  if (2.2250738585072014E-308 > anorm) {
    scale = 2.2250738585072014E-308;
  }

  ascale = 1.0 / scale;
  for (je = 0; je < 4; je++) {
    scale = (fabs(A[(((3 - je) << 2) - je) + 3].re) + fabs(A[(((3 - je) << 2) -
               je) + 3].im)) * ascale;
    if (1.0 > scale) {
      scale = 1.0;
    }

    temp = 1.0 / scale;
    salpha_re = ascale * (temp * A[(((3 - je) << 2) - je) + 3].re);
    salpha_im = ascale * (temp * A[(((3 - je) << 2) - je) + 3].im);
    acoeff = temp * ascale;
    if ((fabs(temp) >= 2.2250738585072014E-308) && (fabs(acoeff) <
         4.0083367200179456E-292)) {
      b3 = true;
    } else {
      b3 = false;
    }

    if ((fabs(salpha_re) + fabs(salpha_im) >= 2.2250738585072014E-308) && (fabs
         (salpha_re) + fabs(salpha_im) < 4.0083367200179456E-292)) {
      b4 = true;
    } else {
      b4 = false;
    }

    scale = 1.0;
    if (b3) {
      scale = anorm;
      if (2.4948003869184E+291 < anorm) {
        scale = 2.4948003869184E+291;
      }

      scale *= 4.0083367200179456E-292 / fabs(temp);
    }

    if (b4) {
      d_re = 4.0083367200179456E-292 / (fabs(salpha_re) + fabs(salpha_im));
      if (d_re > scale) {
        scale = d_re;
      }
    }

    if (b3 || b4) {
      d_re = fabs(acoeff);
      y = fabs(salpha_re) + fabs(salpha_im);
      if (1.0 > d_re) {
        d_re = 1.0;
      }

      if (y > d_re) {
        d_re = y;
      }

      d_re = 1.0 / (2.2250738585072014E-308 * d_re);
      if (d_re < scale) {
        scale = d_re;
      }

      if (b3) {
        acoeff = ascale * (scale * temp);
      } else {
        acoeff *= scale;
      }

      if (b4) {
        salpha_re *= scale;
        salpha_im *= scale;
      } else {
        salpha_re *= scale;
        salpha_im *= scale;
      }
    }

    acoefa = fabs(acoeff);
    for (jr = 0; jr < 4; jr++) {
      work1[jr].re = 0.0;
      work1[jr].im = 0.0;
    }

    work1[3 - je].re = 1.0;
    work1[3 - je].im = 0.0;
    dmin = 2.2204460492503131E-16 * acoefa * anorm;
    d_re = 2.2204460492503131E-16 * (fabs(salpha_re) + fabs(salpha_im));
    if (d_re > dmin) {
      dmin = d_re;
    }

    if (2.2250738585072014E-308 > dmin) {
      dmin = 2.2250738585072014E-308;
    }

    for (jr = 0; jr <= 2 - je; jr++) {
      work1[jr].re = acoeff * A[jr + ((3 - je) << 2)].re;
      work1[jr].im = acoeff * A[jr + ((3 - je) << 2)].im;
    }

    work1[3 - je].re = 1.0;
    work1[3 - je].im = 0.0;
    for (b_j = -2; b_j + 2 <= 2 - je; b_j++) {
      j = -(je + b_j);
      d_re = acoeff * A[j + (j << 2)].re - salpha_re;
      d_im = acoeff * A[j + (j << 2)].im - salpha_im;
      if (fabs(d_re) + fabs(d_im) <= dmin) {
        d_re = dmin;
        d_im = 0.0;
      }

      if ((fabs(d_re) + fabs(d_im) < 1.0) && (fabs(work1[j].re) + fabs(work1[j].
            im) >= 1.1235582092889474E+307 * (fabs(d_re) + fabs(d_im)))) {
        temp = 1.0 / (fabs(work1[j].re) + fabs(work1[j].im));
        for (jr = 0; jr <= 3 - je; jr++) {
          work1[jr].re *= temp;
          work1[jr].im *= temp;
        }
      }

      work1_re = -work1[j].re;
      work1_im = -work1[j].im;
      if (d_im == 0.0) {
        if (-work1[j].im == 0.0) {
          work1[j].re = -work1[j].re / d_re;
          work1[j].im = 0.0;
        } else if (-work1[j].re == 0.0) {
          work1[j].re = 0.0;
          work1[j].im = work1_im / d_re;
        } else {
          work1[j].re = -work1[j].re / d_re;
          work1[j].im = work1_im / d_re;
        }
      } else if (d_re == 0.0) {
        if (-work1[j].re == 0.0) {
          work1[j].re = -work1[j].im / d_im;
          work1[j].im = 0.0;
        } else if (-work1[j].im == 0.0) {
          work1[j].re = 0.0;
          work1[j].im = -(work1_re / d_im);
        } else {
          work1[j].re = -work1[j].im / d_im;
          work1[j].im = -(work1_re / d_im);
        }
      } else {
        temp = fabs(d_re);
        scale = fabs(d_im);
        if (temp > scale) {
          y = d_im / d_re;
          scale = d_re + y * d_im;
          work1[j].re = (-work1[j].re + y * -work1[j].im) / scale;
          work1[j].im = (work1_im - y * work1_re) / scale;
        } else if (scale == temp) {
          if (d_re > 0.0) {
            y = 0.5;
          } else {
            y = -0.5;
          }

          if (d_im > 0.0) {
            scale = 0.5;
          } else {
            scale = -0.5;
          }

          work1[j].re = (-work1[j].re * y + -work1[j].im * scale) / temp;
          work1[j].im = (work1_im * y - work1_re * scale) / temp;
        } else {
          y = d_re / d_im;
          scale = d_im + y * d_re;
          work1[j].re = (y * -work1[j].re + -work1[j].im) / scale;
          work1[j].im = (y * work1_im - work1_re) / scale;
        }
      }

      if (j + 1 > 1) {
        if (fabs(work1[j].re) + fabs(work1[j].im) > 1.0) {
          temp = 1.0 / (fabs(work1[j].re) + fabs(work1[j].im));
          if (acoefa * rworka[j] >= 1.1235582092889474E+307 * temp) {
            for (jr = 0; jr <= 3 - je; jr++) {
              work1[jr].re *= temp;
              work1[jr].im *= temp;
            }
          }
        }

        d_re = acoeff * work1[j].re;
        d_im = acoeff * work1[j].im;
        for (jr = 0; jr < j; jr++) {
          work1[jr].re += d_re * A[jr + (j << 2)].re - d_im * A[jr + (j << 2)].
            im;
          work1[jr].im += d_re * A[jr + (j << 2)].im + d_im * A[jr + (j << 2)].
            re;
        }
      }
    }

    for (jr = 0; jr < 4; jr++) {
      work2[jr].re = 0.0;
      work2[jr].im = 0.0;
    }

    for (j = 0; j <= 3 - je; j++) {
      for (jr = 0; jr < 4; jr++) {
        work2[jr].re += V[jr + (j << 2)].re * work1[j].re - V[jr + (j << 2)].im *
          work1[j].im;
        work2[jr].im += V[jr + (j << 2)].re * work1[j].im + V[jr + (j << 2)].im *
          work1[j].re;
      }
    }

    scale = fabs(work2[0].re) + fabs(work2[0].im);
    for (jr = 0; jr < 3; jr++) {
      d_re = fabs(work2[jr + 1].re) + fabs(work2[jr + 1].im);
      if (d_re > scale) {
        scale = d_re;
      }
    }

    if (scale > 2.2250738585072014E-308) {
      temp = 1.0 / scale;
      for (jr = 0; jr < 4; jr++) {
        V[jr + ((3 - je) << 2)].re = temp * work2[jr].re;
        V[jr + ((3 - je) << 2)].im = temp * work2[jr].im;
      }
    } else {
      for (jr = 0; jr < 4; jr++) {
        V[jr + ((3 - je) << 2)].re = 0.0;
        V[jr + ((3 - je) << 2)].im = 0.0;
      }
    }
  }
}

//
// Arguments    : const double A[16]
//                creal_T V[16]
//                creal_T D[16]
// Return Type  : void
//
void eig(const double A[16], creal_T V[16], creal_T D[16])
{
  creal_T b_A[16];
  int j;
  creal_T beta1[4];
  creal_T alpha1[4];
  int coltop;
  double colnorm;
  double scale;
  double absxk;
  double t;
  for (j = 0; j < 16; j++) {
    b_A[j].re = A[j];
    b_A[j].im = 0.0;
  }

  eml_matlab_zggev(b_A, &j, alpha1, beta1, V);
  for (coltop = 0; coltop < 14; coltop += 4) {
    colnorm = 0.0;
    scale = 2.2250738585072014E-308;
    for (j = coltop; j + 1 <= coltop + 4; j++) {
      absxk = fabs(V[j].re);
      if (absxk > scale) {
        t = scale / absxk;
        colnorm = 1.0 + colnorm * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        colnorm += t * t;
      }

      absxk = fabs(V[j].im);
      if (absxk > scale) {
        t = scale / absxk;
        colnorm = 1.0 + colnorm * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        colnorm += t * t;
      }
    }

    colnorm = scale * sqrt(colnorm);
    for (j = coltop; j + 1 <= coltop + 4; j++) {
      scale = V[j].im;
      if (V[j].im == 0.0) {
        V[j].re /= colnorm;
        V[j].im = 0.0;
      } else if (V[j].re == 0.0) {
        V[j].re = 0.0;
        V[j].im = scale / colnorm;
      } else {
        V[j].re /= colnorm;
        V[j].im = scale / colnorm;
      }
    }
  }

  for (j = 0; j < 16; j++) {
    D[j].re = 0.0;
    D[j].im = 0.0;
  }

  for (j = 0; j < 4; j++) {
    if (beta1[j].im == 0.0) {
      if (alpha1[j].im == 0.0) {
        D[j + (j << 2)].re = alpha1[j].re / beta1[j].re;
        D[j + (j << 2)].im = 0.0;
      } else if (alpha1[j].re == 0.0) {
        D[j + (j << 2)].re = 0.0;
        D[j + (j << 2)].im = alpha1[j].im / beta1[j].re;
      } else {
        D[j + (j << 2)].re = alpha1[j].re / beta1[j].re;
        D[j + (j << 2)].im = alpha1[j].im / beta1[j].re;
      }
    } else if (beta1[j].re == 0.0) {
      if (alpha1[j].re == 0.0) {
        D[j + (j << 2)].re = alpha1[j].im / beta1[j].im;
        D[j + (j << 2)].im = 0.0;
      } else if (alpha1[j].im == 0.0) {
        D[j + (j << 2)].re = 0.0;
        D[j + (j << 2)].im = -(alpha1[j].re / beta1[j].im);
      } else {
        D[j + (j << 2)].re = alpha1[j].im / beta1[j].im;
        D[j + (j << 2)].im = -(alpha1[j].re / beta1[j].im);
      }
    } else {
      t = fabs(beta1[j].re);
      scale = fabs(beta1[j].im);
      if (t > scale) {
        scale = beta1[j].im / beta1[j].re;
        absxk = beta1[j].re + scale * beta1[j].im;
        D[j + (j << 2)].re = (alpha1[j].re + scale * alpha1[j].im) / absxk;
        D[j + (j << 2)].im = (alpha1[j].im - scale * alpha1[j].re) / absxk;
      } else if (scale == t) {
        if (beta1[j].re > 0.0) {
          scale = 0.5;
        } else {
          scale = -0.5;
        }

        if (beta1[j].im > 0.0) {
          absxk = 0.5;
        } else {
          absxk = -0.5;
        }

        D[j + (j << 2)].re = (alpha1[j].re * scale + alpha1[j].im * absxk) / t;
        D[j + (j << 2)].im = (alpha1[j].im * scale - alpha1[j].re * absxk) / t;
      } else {
        scale = beta1[j].re / beta1[j].im;
        absxk = beta1[j].im + scale * beta1[j].re;
        D[j + (j << 2)].re = (scale * alpha1[j].re + alpha1[j].im) / absxk;
        D[j + (j << 2)].im = (scale * alpha1[j].im - alpha1[j].re) / absxk;
      }
    }
  }
}

//
// File trailer for eig.cpp
//
// [EOF]
//
