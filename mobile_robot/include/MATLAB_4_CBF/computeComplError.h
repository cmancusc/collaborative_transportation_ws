//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeComplError.h
//
//  Code generation for function 'computeComplError'
//


#ifndef COMPUTECOMPLERROR_H
#define COMPUTECOMPLERROR_H

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
          double computeComplError(const double cIneq[21], const double lambda
            [43]);
        }
      }
    }
  }
}

#endif

// End of code generation (computeComplError.h)
