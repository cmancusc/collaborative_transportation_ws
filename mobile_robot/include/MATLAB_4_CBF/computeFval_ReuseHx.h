//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeFval_ReuseHx.h
//
//  Code generation for function 'computeFval_ReuseHx'
//


#ifndef COMPUTEFVAL_REUSEHX_H
#define COMPUTEFVAL_REUSEHX_H

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
          double computeFval_ReuseHx(const h_struct_T *obj, double workspace
            [1290], const double f[30], const double x[30]);
        }
      }
    }
  }
}

#endif

// End of code generation (computeFval_ReuseHx.h)
