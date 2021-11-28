//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  factorQR.h
//
//  Code generation for function 'factorQR'
//


#ifndef FACTORQR_H
#define FACTORQR_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct g_struct_T;

// Function Declarations
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace QRManager
      {
        void factorQR(g_struct_T *obj, const double A[910], int mrows, int ncols);
      }
    }
  }
}

#endif

// End of code generation (factorQR.h)
