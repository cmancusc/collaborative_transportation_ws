//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  updateWorkingSetForNewQP.cpp
//
//  Code generation for function 'updateWorkingSetForNewQP'
//


// Include files
#include "updateWorkingSetForNewQP.h"
#include "MATLAB_4_CBF_internal_types.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace fminconsqp
      {
        namespace internal
        {
          void updateWorkingSetForNewQP(e_struct_T *WorkingSet, const double
            cIneq[21])
          {
            int idx;
            int nVar;
            nVar = WorkingSet->nVar;
            for (idx = 0; idx < 21; idx++) {
              WorkingSet->bineq[idx] = -cIneq[idx];
            }

            if (WorkingSet->nActiveConstr > 0) {
              int i;
              i = WorkingSet->nActiveConstr;
              for (idx = 1; idx <= i; idx++) {
                switch (WorkingSet->Wid[idx - 1]) {
                 case 4:
                  WorkingSet->bwset[idx - 1] = WorkingSet->lb
                    [WorkingSet->indexLB[WorkingSet->Wlocalidx[idx - 1] - 1] - 1];
                  break;

                 case 5:
                  // A check that is always false is detected at compile-time. Eliminating code that follows. 
                  break;

                 default:
                  {
                    WorkingSet->bwset[idx - 1] = WorkingSet->bineq
                      [WorkingSet->Wlocalidx[idx - 1] - 1];
                    if (WorkingSet->Wlocalidx[idx - 1] >= 21) {
                      int iy0;
                      iy0 = 30 * (idx - 1);
                      for (int k = 0; k < nVar; k++) {
                        WorkingSet->ATwset[iy0 + k] = WorkingSet->Aineq[k + 600];
                      }
                    }
                  }
                  break;
                }
              }
            }
          }
        }
      }
    }
  }
}

// End of code generation (updateWorkingSetForNewQP.cpp)
