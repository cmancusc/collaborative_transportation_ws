//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeGrad_StoreHx.h
//
//  Code generation for function 'computeGrad_StoreHx'
//


#ifndef COMPUTEGRAD_STOREHX_H
#define COMPUTEGRAD_STOREHX_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct h_struct_T;

// Function Declarations
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace qpactiveset
      {
        namespace Objective
        {
          void computeGrad_StoreHx(h_struct_T *obj, const double H[64], const
            double f[30], const double x[30]);
        }
      }
    }
  }
}

#endif

// End of code generation (computeGrad_StoreHx.h)
