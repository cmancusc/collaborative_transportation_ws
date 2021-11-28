//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeComplError.cpp
//
//  Code generation for function 'computeComplError'
//


// Include files
#include "computeComplError.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace fminconsqp
      {
        namespace stopping
        {
          double computeComplError(const double cIneq[21], const double lambda
            [43])
          {
            double nlpComplError;
            nlpComplError = 0.0;
            for (int idx = 0; idx < 21; idx++) {
              double b_u0;
              double d;
              double u0;
              u0 = cIneq[idx];
              b_u0 = std::abs(u0);
              d = lambda[idx];
              if ((!(b_u0 < d)) && (!rtIsNaN(d))) {
                b_u0 = d;
              }

              u0 = std::abs(u0 * d);
              if ((!(u0 < b_u0)) && (!rtIsNaN(b_u0))) {
                u0 = b_u0;
              }

              if ((!(nlpComplError > u0)) && (!rtIsNaN(u0))) {
                nlpComplError = u0;
              }
            }

            return nlpComplError;
          }
        }
      }
    }
  }
}

// End of code generation (computeComplError.cpp)
