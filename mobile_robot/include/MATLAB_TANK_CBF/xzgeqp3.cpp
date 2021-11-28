//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xzgeqp3.cpp
//
//  Code generation for function 'xzgeqp3'
//


// Include files
#include "xzgeqp3.h"
#include "rt_nonfinite.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include <cstring>

// Function Definitions
namespace coder
{
  namespace internal
  {
    namespace reflapack
    {
      void qrf(double A[1521], int m, int n, int nfxd, double tau[39])
      {
        double work[39];
        double atmp;
        std::memset(&work[0], 0, 39U * sizeof(double));
        for (int i = 0; i < nfxd; i++) {
          int ii;
          int mmi;
          ii = i * 39 + i;
          mmi = m - i;
          if (i + 1 < m) {
            atmp = A[ii];
            tau[i] = xzlarfg(mmi, &atmp, A, ii + 2);
            A[ii] = atmp;
          } else {
            tau[i] = 0.0;
          }

          if (i + 1 < n) {
            atmp = A[ii];
            A[ii] = 1.0;
            xzlarf(mmi, (n - i) - 1, ii + 1, tau[i], A, ii + 40, work);
            A[ii] = atmp;
          }
        }
      }
    }
  }
}

// End of code generation (xzgeqp3.cpp)
