//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  BFGSUpdate.h
//
//  Code generation for function 'BFGSUpdate'
//


#ifndef BFGSUPDATE_H
#define BFGSUPDATE_H

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
        boolean_T BFGSUpdate(int nvar, double Bk[64], const double sk[28],
                             double yk[28], double workspace[1092]);
      }
    }
  }
}

#endif

// End of code generation (BFGSUpdate.h)
