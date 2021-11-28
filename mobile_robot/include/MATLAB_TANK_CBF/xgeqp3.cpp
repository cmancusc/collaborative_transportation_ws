//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xgeqp3.cpp
//
//  Code generation for function 'xgeqp3'
//


// Include files
#include "xgeqp3.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "xzgeqp3.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder
{
  namespace internal
  {
    namespace lapack
    {
      void xgeqp3(double A[1521], int m, int n, int jpvt[39], double tau[39])
      {
        double vn1[39];
        double vn2[39];
        double work[39];
        double temp;
        int minmn_tmp;
        if (m < n) {
          minmn_tmp = m;
        } else {
          minmn_tmp = n;
        }

        std::memset(&tau[0], 0, 39U * sizeof(double));
        if (minmn_tmp < 1) {
          for (int pvt = 0; pvt < n; pvt++) {
            jpvt[pvt] = pvt + 1;
          }
        } else {
          int ix;
          int iy;
          int k;
          int nfxd;
          int pvt;
          nfxd = 0;
          for (pvt = 0; pvt < n; pvt++) {
            if (jpvt[pvt] != 0) {
              nfxd++;
              if (pvt + 1 != nfxd) {
                ix = pvt * 39;
                iy = (nfxd - 1) * 39;
                for (k = 0; k < m; k++) {
                  temp = A[ix];
                  A[ix] = A[iy];
                  A[iy] = temp;
                  ix++;
                  iy++;
                }

                jpvt[pvt] = jpvt[nfxd - 1];
                jpvt[nfxd - 1] = pvt + 1;
              } else {
                jpvt[pvt] = pvt + 1;
              }
            } else {
              jpvt[pvt] = pvt + 1;
            }
          }

          if (nfxd >= minmn_tmp) {
            nfxd = minmn_tmp;
          }

          reflapack::qrf(A, m, n, nfxd, tau);
          if (nfxd < minmn_tmp) {
            double d;
            int i;
            std::memset(&work[0], 0, 39U * sizeof(double));
            std::memset(&vn1[0], 0, 39U * sizeof(double));
            std::memset(&vn2[0], 0, 39U * sizeof(double));
            i = nfxd + 1;
            for (pvt = i; pvt <= n; pvt++) {
              d = blas::xnrm2(m - nfxd, A, (nfxd + (pvt - 1) * 39) + 1);
              vn1[pvt - 1] = d;
              vn2[pvt - 1] = d;
            }

            i = nfxd + 1;
            for (int b_i = i; b_i <= minmn_tmp; b_i++) {
              double s;
              int ii;
              int ip1;
              int mmi;
              int nmi;
              ip1 = b_i + 1;
              iy = (b_i - 1) * 39;
              ii = (iy + b_i) - 1;
              nmi = (n - b_i) + 1;
              mmi = m - b_i;
              if (nmi < 1) {
                nfxd = -2;
              } else {
                nfxd = -1;
                if (nmi > 1) {
                  ix = b_i;
                  temp = std::abs(vn1[b_i - 1]);
                  for (k = 2; k <= nmi; k++) {
                    ix++;
                    s = std::abs(vn1[ix - 1]);
                    if (s > temp) {
                      nfxd = k - 2;
                      temp = s;
                    }
                  }
                }
              }

              pvt = b_i + nfxd;
              if (pvt + 1 != b_i) {
                ix = pvt * 39;
                for (k = 0; k < m; k++) {
                  temp = A[ix];
                  A[ix] = A[iy];
                  A[iy] = temp;
                  ix++;
                  iy++;
                }

                nfxd = jpvt[pvt];
                jpvt[pvt] = jpvt[b_i - 1];
                jpvt[b_i - 1] = nfxd;
                vn1[pvt] = vn1[b_i - 1];
                vn2[pvt] = vn2[b_i - 1];
              }

              if (b_i < m) {
                temp = A[ii];
                d = reflapack::xzlarfg(mmi + 1, &temp, A, ii + 2);
                tau[b_i - 1] = d;
                A[ii] = temp;
              } else {
                d = 0.0;
                tau[b_i - 1] = 0.0;
              }

              if (b_i < n) {
                temp = A[ii];
                A[ii] = 1.0;
                reflapack::xzlarf(mmi + 1, nmi - 1, ii + 1, d, A, ii + 40, work);
                A[ii] = temp;
              }

              for (pvt = ip1; pvt <= n; pvt++) {
                nfxd = b_i + (pvt - 1) * 39;
                d = vn1[pvt - 1];
                if (d != 0.0) {
                  temp = std::abs(A[nfxd - 1]) / d;
                  temp = 1.0 - temp * temp;
                  if (temp < 0.0) {
                    temp = 0.0;
                  }

                  s = d / vn2[pvt - 1];
                  s = temp * (s * s);
                  if (s <= 1.4901161193847656E-8) {
                    if (b_i < m) {
                      d = blas::xnrm2(mmi, A, nfxd + 1);
                      vn1[pvt - 1] = d;
                      vn2[pvt - 1] = d;
                    } else {
                      vn1[pvt - 1] = 0.0;
                      vn2[pvt - 1] = 0.0;
                    }
                  } else {
                    vn1[pvt - 1] = d * std::sqrt(temp);
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}

// End of code generation (xgeqp3.cpp)
