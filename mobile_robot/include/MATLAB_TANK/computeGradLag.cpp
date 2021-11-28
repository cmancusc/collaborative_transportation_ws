//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeGradLag.cpp
//
//  Code generation for function 'computeGradLag'
//


// Include files
#include "computeGradLag.h"
#include "rt_nonfinite.h"
#include <cstring>

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
          void b_computeGradLag(double workspace[910], int nVar, const double
                                grad[26], const double AineqTrans[442], const
                                int finiteLB[26], int mLB, const double lambda
                                [35])
          {
            int i;
            int iac;
            int ix;
            int iy;
            if (0 <= nVar - 1) {
              std::memcpy(&workspace[0], &grad[0], nVar * sizeof(double));
            }

            ix = 0;
            for (iac = 0; iac <= 416; iac += 26) {
              iy = 0;
              i = iac + nVar;
              for (int ia = iac + 1; ia <= i; ia++) {
                workspace[iy] += AineqTrans[ia - 1] * lambda[ix];
                iy++;
              }

              ix++;
            }

            ix = 17;
            for (iy = 0; iy < mLB; iy++) {
              i = finiteLB[iy];
              workspace[i - 1] -= lambda[ix];
              ix++;
            }
          }

          void computeGradLag(double workspace[26], int nVar, const double grad
                              [26], const double AineqTrans[442], const int
                              finiteLB[26], int mLB, const double lambda[35])
          {
            int i;
            int iac;
            int ix;
            int iy;
            if (0 <= nVar - 1) {
              std::memcpy(&workspace[0], &grad[0], nVar * sizeof(double));
            }

            ix = 0;
            for (iac = 0; iac <= 416; iac += 26) {
              iy = 0;
              i = iac + nVar;
              for (int ia = iac + 1; ia <= i; ia++) {
                workspace[iy] += AineqTrans[ia - 1] * lambda[ix];
                iy++;
              }

              ix++;
            }

            ix = 17;
            for (iy = 0; iy < mLB; iy++) {
              i = finiteLB[iy];
              workspace[i - 1] -= lambda[ix];
              ix++;
            }
          }
        }
      }
    }
  }
}

// End of code generation (computeGradLag.cpp)
