//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeObjective_.h
//
//  Code generation for function 'computeObjective_'
//


#ifndef COMPUTEOBJECTIVE__H
#define COMPUTEOBJECTIVE__H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
class MATLAB_OPT_TANK;
namespace coder
{
  class anonymous_function;
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
        namespace ObjNonlinEvaluator
        {
          void computeObjective_(MATLAB_OPT_TANK *aInstancePtr, const
            anonymous_function obj_objfun, const double x[8], double *fval, int *
            status);
        }
      }
    }
  }
}

#endif

// End of code generation (computeObjective_.h)
