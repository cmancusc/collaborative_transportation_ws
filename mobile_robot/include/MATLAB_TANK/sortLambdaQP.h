//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  sortLambdaQP.h
//
//  Code generation for function 'sortLambdaQP'
//


#ifndef SORTLAMBDAQP_H
#define SORTLAMBDAQP_H

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
      namespace qpactiveset
      {
        namespace parseoutput
        {
          void sortLambdaQP(double lambda[35], int WorkingSet_nActiveConstr,
                            const int WorkingSet_sizes[5], const int
                            WorkingSet_isActiveIdx[6], const int WorkingSet_Wid
                            [35], const int WorkingSet_Wlocalidx[35], double
                            workspace[910]);
        }
      }
    }
  }
}

#endif

// End of code generation (sortLambdaQP.h)