//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xpotrf.cpp
//
//  Code generation for function 'xpotrf'
//


// Include files
#include "xpotrf.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder
{
  namespace internal
  {
    namespace lapack
    {
      int xpotrf(int n, double A[1849])
      {
        double c;
        double ssq;
        int iac;
        int info;
        int iy;
        int j;
        int nmj;
        boolean_T exitg1;
        info = 0;
        j = 0;
        exitg1 = false;
        while ((!exitg1) && (j <= n - 1)) {
          int idxA1j;
          int idxAjj;
          int ix;
          idxA1j = j * 43;
          idxAjj = idxA1j + j;
          ssq = 0.0;
          if (j >= 1) {
            ix = idxA1j;
            iy = idxA1j;
            for (nmj = 0; nmj < j; nmj++) {
              ssq += A[ix] * A[iy];
              ix++;
              iy++;
            }
          }

          ssq = A[idxAjj] - ssq;
          if (ssq > 0.0) {
            ssq = std::sqrt(ssq);
            A[idxAjj] = ssq;
            if (j + 1 < n) {
              int i;
              int ia0;
              int idxAjjp1;
              nmj = (n - j) - 2;
              ia0 = idxA1j + 44;
              idxAjjp1 = idxAjj + 44;
              if ((j != 0) && (nmj + 1 != 0)) {
                iy = idxAjj + 43;
                i = (idxA1j + 43 * nmj) + 44;
                for (iac = ia0; iac <= i; iac += 43) {
                  int i1;
                  ix = idxA1j;
                  c = 0.0;
                  i1 = (iac + j) - 1;
                  for (int ia = iac; ia <= i1; ia++) {
                    c += A[ia - 1] * A[ix];
                    ix++;
                  }

                  A[iy] += -c;
                  iy += 43;
                }
              }

              ssq = 1.0 / ssq;
              i = (idxAjj + 43 * nmj) + 44;
              for (nmj = idxAjjp1; nmj <= i; nmj += 43) {
                A[nmj - 1] *= ssq;
              }
            }

            j++;
          } else {
            A[idxAjj] = ssq;
            info = j + 1;
            exitg1 = true;
          }
        }

        return info;
      }
    }
  }
}

// End of code generation (xpotrf.cpp)
