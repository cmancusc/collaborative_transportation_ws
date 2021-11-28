//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  sortLambdaQP.cpp
//
//  Code generation for function 'sortLambdaQP'
//


// Include files
#include "sortLambdaQP.h"
#include "rt_nonfinite.h"

// Function Definitions
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
                            workspace[910])
          {
            if (WorkingSet_nActiveConstr != 0) {
              int idx;
              int mAll;
              mAll = WorkingSet_sizes[3] + 16;
              for (idx = 0; idx <= mAll; idx++) {
                workspace[idx] = lambda[idx];
                lambda[idx] = 0.0;
              }

              mAll = 0;
              idx = 0;
              while ((idx + 1 <= WorkingSet_nActiveConstr) &&
                     (WorkingSet_Wid[idx] <= 2)) {
                lambda[WorkingSet_Wlocalidx[idx] - 1] = workspace[mAll];
                mAll++;
                idx++;
              }

              while (idx + 1 <= WorkingSet_nActiveConstr) {
                int idxOffset;
                switch (WorkingSet_Wid[idx]) {
                 case 3:
                  idxOffset = 1;
                  break;

                 case 4:
                  idxOffset = 18;
                  break;

                 default:
                  idxOffset = WorkingSet_isActiveIdx[4];
                  break;
                }

                lambda[(idxOffset + WorkingSet_Wlocalidx[idx]) - 2] =
                  workspace[mAll];
                mAll++;
                idx++;
              }
            }
          }
        }
      }
    }
  }
}

// End of code generation (sortLambdaQP.cpp)
