//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeGradLag.h
//
//  Code generation for function 'computeGradLag'
//


#ifndef COMPUTEGRADLAG_H
#define COMPUTEGRADLAG_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace fminconsqp
      {
        namespace stopping
        {
          void b_computeGradLag(double workspace[1092], int nVar, const double
                                grad[28], const double AineqTrans[532], const
                                int finiteLB[28], int mLB, const double lambda
                                [39]);
          void computeGradLag(double workspace[28], int nVar, const double grad
                              [28], const double AineqTrans[532], const int
                              finiteLB[28], int mLB, const double lambda[39]);
        }
      }
    }
  }
}

#endif

// End of code generation (computeGradLag.h)
