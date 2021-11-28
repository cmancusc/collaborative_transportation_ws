//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeObjective_.cpp
//
//  Code generation for function 'computeObjective_'
//


// Include files
#include "computeObjective_.h"
#include "MATLAB_4_CBF.h"
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
        namespace ObjNonlinEvaluator
        {
          void computeObjective_(MATLAB_4_CBF *aInstancePtr, const
            anonymous_function obj_objfun, const double x[8], double *fval, int *
            status)
          {
            *fval = aInstancePtr->cost_function_4_cbf(x,
              obj_objfun.tunableEnvironment[0].f1);
            *status = 1;
            if (rtIsInf(*fval) || rtIsNaN(*fval)) {
              if (rtIsNaN(*fval)) {
                *status = -6;
              } else if (*fval < 0.0) {
                *status = -4;
              } else {
                *status = -5;
              }
            }
          }
        }
      }
    }
  }
}

// End of code generation (computeObjective_.cpp)
