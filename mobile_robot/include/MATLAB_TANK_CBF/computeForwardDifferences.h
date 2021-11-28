//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeForwardDifferences.h
//
//  Code generation for function 'computeForwardDifferences'
//


#ifndef COMPUTEFORWARDDIFFERENCES_H
#define COMPUTEFORWARDDIFFERENCES_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
class MATLAB_OPT_TANK_CBF;
struct k_struct_T;

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
            boolean_T computeForwardDifferences(MATLAB_OPT_TANK_CBF
              *aInstancePtr, k_struct_T *obj, double fCurrent, const double
              cIneqCurrent[19], double xk[8], double gradf[28], double
              JacCineqTrans[532]);
          }
        }
      }
    }
  }
}

#endif

// End of code generation (computeForwardDifferences.h)
