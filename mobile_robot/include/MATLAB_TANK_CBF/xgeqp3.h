//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xgeqp3.h
//
//  Code generation for function 'xgeqp3'
//


#ifndef XGEQP3_H
#define XGEQP3_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder
{
  namespace internal
  {
    namespace lapack
    {
      void xgeqp3(double A[1521], int m, int n, int jpvt[39], double tau[39]);
    }
  }
}

#endif

// End of code generation (xgeqp3.h)
