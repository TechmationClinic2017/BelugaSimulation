/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mldivide.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 15-Jan-2017 12:59:41
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "beluga_dynamic_model.h"
#include "mldivide.h"

/* Function Definitions */

/*
 * Arguments    : const double A[36]
 *                double B[6]
 * Return Type  : void
 */
void mldivide(const double A[36], double B[6])
{
  double b_A[36];
  signed char ipiv[6];
  int i0;
  int j;
  int k;
  int c;
  int kAcol;
  int ix;
  double smax;
  int jy;
  double s;
  int ijA;
  memcpy(&b_A[0], &A[0], 36U * sizeof(double));
  for (i0 = 0; i0 < 6; i0++) {
    ipiv[i0] = (signed char)(1 + i0);
  }

  for (j = 0; j < 5; j++) {
    c = j * 7;
    kAcol = 0;
    ix = c;
    smax = fabs(b_A[c]);
    for (k = 2; k <= 6 - j; k++) {
      ix++;
      s = fabs(b_A[ix]);
      if (s > smax) {
        kAcol = k - 1;
        smax = s;
      }
    }

    if (b_A[c + kAcol] != 0.0) {
      if (kAcol != 0) {
        ipiv[j] = (signed char)((j + kAcol) + 1);
        ix = j;
        kAcol += j;
        for (k = 0; k < 6; k++) {
          smax = b_A[ix];
          b_A[ix] = b_A[kAcol];
          b_A[kAcol] = smax;
          ix += 6;
          kAcol += 6;
        }
      }

      i0 = (c - j) + 6;
      for (jy = c + 1; jy + 1 <= i0; jy++) {
        b_A[jy] /= b_A[c];
      }
    }

    kAcol = c;
    jy = c + 6;
    for (k = 1; k <= 5 - j; k++) {
      smax = b_A[jy];
      if (b_A[jy] != 0.0) {
        ix = c + 1;
        i0 = (kAcol - j) + 12;
        for (ijA = 7 + kAcol; ijA + 1 <= i0; ijA++) {
          b_A[ijA] += b_A[ix] * -smax;
          ix++;
        }
      }

      jy += 6;
      kAcol += 6;
    }

    if (ipiv[j] != j + 1) {
      smax = B[j];
      B[j] = B[ipiv[j] - 1];
      B[ipiv[j] - 1] = smax;
    }
  }

  for (k = 0; k < 6; k++) {
    kAcol = 6 * k;
    if (B[k] != 0.0) {
      for (jy = k + 1; jy + 1 < 7; jy++) {
        B[jy] -= B[k] * b_A[jy + kAcol];
      }
    }
  }

  for (k = 5; k >= 0; k += -1) {
    kAcol = 6 * k;
    if (B[k] != 0.0) {
      B[k] /= b_A[k + kAcol];
      for (jy = 0; jy + 1 <= k; jy++) {
        B[jy] -= B[k] * b_A[jy + kAcol];
      }
    }
  }
}

/*
 * File trailer for mldivide.c
 *
 * [EOF]
 */
