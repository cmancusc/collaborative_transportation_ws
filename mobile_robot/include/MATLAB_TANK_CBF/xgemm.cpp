//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xgemm.cpp
//
//  Code generation for function 'xgemm'
//


// Include files
#include "xgemm.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace coder
{
  namespace internal
  {
    namespace blas
    {
      void xgemm(int m, int n, int k, const double A[64], int lda, const double
                 B[1521], int ib0, double C[1092])
      {
        int ar;
        int br;
        int cr;
        if ((m != 0) && (n != 0)) {
          int i;
          int i1;
          int lastColC;
          br = ib0;
          lastColC = 39 * (n - 1);
          for (cr = 0; cr <= lastColC; cr += 39) {
            i = cr + 1;
            i1 = cr + m;
            if (i <= i1) {
              std::memset(&C[i + -1], 0, ((i1 - i) + 1) * sizeof(double));
            }
          }

          for (cr = 0; cr <= lastColC; cr += 39) {
            ar = -1;
            i = br + k;
            for (int ib = br; ib < i; ib++) {
              int i2;
              int ia;
              ia = ar;
              i1 = cr + 1;
              i2 = cr + m;
              for (int ic = i1; ic <= i2; ic++) {
                ia++;
                C[ic - 1] += B[ib - 1] * A[ia];
              }

              ar += lda;
            }

            br += 39;
          }
        }
      }

      void xgemm(int m, int n, int k, const double A[1521], int ia0, const
                 double B[1092], double C[1521])
      {
        double temp;
        int ar;
        int br;
        int cr;
        if ((m != 0) && (n != 0)) {
          int i;
          int i1;
          int lastColC;
          lastColC = 39 * (n - 1);
          for (cr = 0; cr <= lastColC; cr += 39) {
            i = cr + 1;
            i1 = cr + m;
            if (i <= i1) {
              std::memset(&C[i + -1], 0, ((i1 - i) + 1) * sizeof(double));
            }
          }

          br = -1;
          for (cr = 0; cr <= lastColC; cr += 39) {
            ar = ia0;
            i = cr + 1;
            i1 = cr + m;
            for (int ic = i; ic <= i1; ic++) {
              temp = 0.0;
              for (int w = 0; w < k; w++) {
                temp += A[(w + ar) - 1] * B[(w + br) + 1];
              }

              C[ic - 1] += temp;
              ar += 39;
            }

            br += 39;
          }
        }
      }
    }
  }
}

// End of code generation (xgemm.cpp)
