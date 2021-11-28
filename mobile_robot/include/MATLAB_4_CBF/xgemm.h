//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xgemm.h
//
//  Code generation for function 'xgemm'
//


#ifndef XGEMM_H
#define XGEMM_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder
{
  namespace internal
  {
    namespace blas
    {
      void xgemm(int m, int n, int k, const double A[64], int lda, const double
                 B[1849], int ib0, double C[1290]);
      void xgemm(int m, int n, int k, const double A[1849], int ia0, const
                 double B[1290], double C[1849]);
    }
  }
}

#endif

// End of code generation (xgemm.h)
