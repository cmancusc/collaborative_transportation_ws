//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xnrm2.cpp
//
//  Code generation for function 'xnrm2'
//


// Include files
#include "xnrm2.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder
{
  namespace internal
  {
    namespace blas
    {
      double xnrm2(int n, const double x[28])
      {
        double y;
        y = 0.0;
        if (n >= 1) {
          if (n == 1) {
            y = std::abs(x[0]);
          } else {
            double scale;
            scale = 3.3121686421112381E-170;
            for (int k = 0; k < n; k++) {
              double absxk;
              absxk = std::abs(x[k]);
              if (absxk > scale) {
                double t;
                t = scale / absxk;
                y = y * t * t + 1.0;
                scale = absxk;
              } else {
                double t;
                t = absxk / scale;
                y += t * t;
              }
            }

            y = scale * std::sqrt(y);
          }
        }

        return y;
      }

      double xnrm2(int n, const double x[1521], int ix0)
      {
        double y;
        y = 0.0;
        if (n >= 1) {
          if (n == 1) {
            y = std::abs(x[ix0 - 1]);
          } else {
            double scale;
            int kend;
            scale = 3.3121686421112381E-170;
            kend = (ix0 + n) - 1;
            for (int k = ix0; k <= kend; k++) {
              double absxk;
              absxk = std::abs(x[k - 1]);
              if (absxk > scale) {
                double t;
                t = scale / absxk;
                y = y * t * t + 1.0;
                scale = absxk;
              } else {
                double t;
                t = absxk / scale;
                y += t * t;
              }
            }

            y = scale * std::sqrt(y);
          }
        }

        return y;
      }
    }
  }
}

// End of code generation (xnrm2.cpp)
