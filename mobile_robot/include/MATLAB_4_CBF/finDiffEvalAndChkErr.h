//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  finDiffEvalAndChkErr.h
//
//  Code generation for function 'finDiffEvalAndChkErr'
//


#ifndef FINDIFFEVALANDCHKERR_H
#define FINDIFFEVALANDCHKERR_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
class MATLAB_4_CBF;
namespace coder
{
  class anonymous_function;
  class b_anonymous_function;
}

// Function Declarations
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
            boolean_T finDiffEvalAndChkErr(MATLAB_4_CBF *aInstancePtr, const
              anonymous_function obj_objfun, const b_anonymous_function
              *obj_nonlin, double *fplus, double cIneqPlus[21], int dim, double
              delta, double xk[8]);
          }
        }
      }
    }
  }
}

#endif

// End of code generation (finDiffEvalAndChkErr.h)
