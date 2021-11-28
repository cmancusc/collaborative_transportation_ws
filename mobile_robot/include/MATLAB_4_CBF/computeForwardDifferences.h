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
class MATLAB_4_CBF;
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
            boolean_T computeForwardDifferences(MATLAB_4_CBF *aInstancePtr,
              k_struct_T *obj, double fCurrent, const double cIneqCurrent[21],
              double xk[8], double gradf[30], double JacCineqTrans[630]);
          }
        }
      }
    }
  }
}

#endif

// End of code generation (computeForwardDifferences.h)
