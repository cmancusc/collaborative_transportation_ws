//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  finDiffEvalAndChkErr.cpp
//
//  Code generation for function 'finDiffEvalAndChkErr'
//


// Include files
#include "finDiffEvalAndChkErr.h"
#include "MATLAB_OPT_TANK.h"
#include "anonymous_function.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"

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
            boolean_T finDiffEvalAndChkErr(MATLAB_OPT_TANK *aInstancePtr, const
              anonymous_function obj_objfun, const b_anonymous_function
              *obj_nonlin, double *fplus, double cIneqPlus[17], int dim, double
              delta, double xk[8])
            {
              double temp_tmp;
              boolean_T evalOK;
              temp_tmp = xk[dim - 1];
              xk[dim - 1] = temp_tmp + delta;
              *fplus = aInstancePtr->cost_function(xk,
                obj_objfun.tunableEnvironment[0].f1);
              evalOK = ((!rtIsInf(*fplus)) && (!rtIsNaN(*fplus)));
              if (evalOK) {
                int idx;
                aInstancePtr->constraints(xk, obj_nonlin->tunableEnvironment.f1,
                  obj_nonlin->tunableEnvironment.f2,
                  obj_nonlin->tunableEnvironment.f3,
                  obj_nonlin->tunableEnvironment.f4,
                  obj_nonlin->tunableEnvironment.f5, cIneqPlus);
                idx = 0;
                while (evalOK && (idx + 1 <= 17)) {
                  evalOK = ((!rtIsInf(cIneqPlus[idx])) && (!rtIsNaN
                             (cIneqPlus[idx])));
                  idx++;
                }

                if (evalOK) {
                  xk[dim - 1] = temp_tmp;
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

// End of code generation (finDiffEvalAndChkErr.cpp)
