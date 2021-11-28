//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeForwardDifferences.cpp
//
//  Code generation for function 'computeForwardDifferences'
//


// Include files
#include "computeForwardDifferences.h"
#include "MATLAB_OPT_TANK.h"
#include "MATLAB_OPT_TANK_internal_types.h"
#include "anonymous_function.h"
#include "finDiffEvalAndChkErr.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace utils
      {
        namespace FiniteDifferences
        {
          namespace internal
          {
            boolean_T computeForwardDifferences(MATLAB_OPT_TANK *aInstancePtr,
              k_struct_T *obj, double fCurrent, const double cIneqCurrent[17],
              double xk[8], double gradf[26], double JacCineqTrans[442])
            {
              double deltaX;
              int idx;
              boolean_T evalOK;
              boolean_T exitg1;
              evalOK = true;
              obj->numEvals = 0;
              idx = 0;
              exitg1 = false;
              while ((!exitg1) && (idx < 8)) {
                boolean_T guard1 = false;
                deltaX = std::abs(xk[idx]);
                if (!(deltaX > 1.0)) {
                  deltaX = 1.0;
                }

                deltaX *= 1.4901161193847656E-8 * (1.0 - 2.0 * static_cast<
                  double>(xk[idx] < 0.0));
                evalOK = finDiffEvalAndChkErr(aInstancePtr, obj->objfun,
                  &obj->nonlin, &obj->f_1, obj->cIneq_1, idx + 1, deltaX, xk);
                obj->numEvals++;
                guard1 = false;
                if (!evalOK) {
                  deltaX = -deltaX;
                  evalOK = finDiffEvalAndChkErr(aInstancePtr, obj->objfun,
                    &obj->nonlin, &obj->f_1, obj->cIneq_1, idx + 1, deltaX, xk);
                  obj->numEvals++;
                  if (!evalOK) {
                    exitg1 = true;
                  } else {
                    guard1 = true;
                  }
                } else {
                  guard1 = true;
                }

                if (guard1) {
                  gradf[idx] = (obj->f_1 - fCurrent) / deltaX;
                  for (int idx_row = 0; idx_row < 17; idx_row++) {
                    JacCineqTrans[idx + 26 * idx_row] = (obj->cIneq_1[idx_row] -
                      cIneqCurrent[idx_row]) / deltaX;
                  }

                  idx++;
                }
              }

              return evalOK;
            }
          }
        }
      }
    }
  }
}

// End of code generation (computeForwardDifferences.cpp)
