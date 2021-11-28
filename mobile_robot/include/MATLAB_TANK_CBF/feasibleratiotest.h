//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  feasibleratiotest.h
//
//  Code generation for function 'feasibleratiotest'
//


#ifndef FEASIBLERATIOTEST_H
#define FEASIBLERATIOTEST_H

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
        void feasibleratiotest(const double solution_xstar[28], const double
          solution_searchDir[28], double workspace[1092], int workingset_nVar,
          const double workingset_Aineq[532], const double workingset_bineq[19],
          const double workingset_lb[28], const int workingset_indexLB[28],
          const int workingset_sizes[5], const int workingset_isActiveIdx[6],
          const boolean_T workingset_isActiveConstr[39], const int
          workingset_nWConstr[5], boolean_T isPhaseOne, double *alpha, boolean_T
          *newBlocking, int *constrType, int *constrIdx);
      }
    }
  }
}

#endif

// End of code generation (feasibleratiotest.h)
