//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  setProblemType.cpp
//
//  Code generation for function 'setProblemType'
//


// Include files
#include "setProblemType.h"
#include "MATLAB_OPT_TANK_CBF_internal_types.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace qpactiveset
      {
        namespace WorkingSet
        {
          void setProblemType(e_struct_T *obj, int PROBLEM_TYPE)
          {
            switch (PROBLEM_TYPE) {
             case 3:
              {
                int i;
                obj->nVar = 8;
                obj->mConstr = 19;
                for (i = 0; i < 5; i++) {
                  obj->sizes[i] = obj->sizesNormal[i];
                }

                for (i = 0; i < 6; i++) {
                  obj->isActiveIdx[i] = obj->isActiveIdxNormal[i];
                }
              }
              break;

             case 1:
              {
                int i;
                int idx_lb;
                obj->nVar = 9;
                obj->mConstr = 20;
                for (i = 0; i < 5; i++) {
                  obj->sizes[i] = obj->sizesPhaseOne[i];
                }

                for (i = 0; i < 6; i++) {
                  obj->isActiveIdx[i] = obj->isActiveIdxPhaseOne[i];
                }

                for (idx_lb = 0; idx_lb < 19; idx_lb++) {
                  obj->Aineq[28 * idx_lb + 8] = -1.0;
                }

                obj->indexLB[obj->sizes[3] - 1] = 9;
                obj->lb[8] = 1.0E-5;
                i = obj->nActiveConstr;
                for (idx_lb = 1; idx_lb <= i; idx_lb++) {
                  obj->ATwset[28 * (idx_lb - 1) + 8] = -1.0;
                }

                obj->isActiveConstr[obj->isActiveIdx[4] - 2] = false;
              }
              break;

             case 2:
              {
                int i;
                obj->nVar = 27;
                obj->mConstr = 38;
                for (i = 0; i < 5; i++) {
                  obj->sizes[i] = obj->sizesRegularized[i];
                }

                if (obj->probType != 4) {
                  int colOffsetAineq;
                  int idx_col;
                  int idx_lb;
                  idx_lb = 8;
                  for (idx_col = 0; idx_col < 19; idx_col++) {
                    colOffsetAineq = 28 * idx_col - 1;
                    i = idx_col + 8;
                    if (9 <= i) {
                      std::memset(&obj->Aineq[colOffsetAineq + 9], 0, (((i +
                        colOffsetAineq) - colOffsetAineq) + -8) * sizeof(double));
                    }

                    obj->Aineq[(idx_col + colOffsetAineq) + 9] = -1.0;
                    i = idx_col + 10;
                    if (i <= 27) {
                      std::memset(&obj->Aineq[i + colOffsetAineq], 0,
                                  (((colOffsetAineq - i) - colOffsetAineq) + 28)
                                  * sizeof(double));
                    }

                    idx_lb++;
                    obj->indexLB[idx_col] = idx_lb;
                  }

                  i = obj->isActiveIdx[4];
                  for (idx_lb = i; idx_lb < 39; idx_lb++) {
                    obj->isActiveConstr[idx_lb - 1] = false;
                  }

                  std::memset(&obj->lb[8], 0, 19U * sizeof(double));
                  i = obj->nActiveConstr;
                  for (idx_col = 1; idx_col <= i; idx_col++) {
                    idx_lb = 28 * (idx_col - 1) - 1;
                    switch (obj->Wid[idx_col - 1]) {
                     case 3:
                      colOffsetAineq = obj->Wlocalidx[idx_col - 1] + 7;
                      if (9 <= colOffsetAineq) {
                        std::memset(&obj->ATwset[idx_lb + 9], 0,
                                    (((colOffsetAineq + idx_lb) - idx_lb) + -8) *
                                    sizeof(double));
                      }

                      obj->ATwset[(obj->Wlocalidx[idx_col - 1] + idx_lb) + 8] =
                        -1.0;
                      colOffsetAineq = obj->Wlocalidx[idx_col - 1] + 9;
                      if (colOffsetAineq <= 27) {
                        std::memset(&obj->ATwset[colOffsetAineq + idx_lb], 0,
                                    (((idx_lb - colOffsetAineq) - idx_lb) + 28) *
                                    sizeof(double));
                      }
                      break;

                     default:
                      std::memset(&obj->ATwset[idx_lb + 9], 0, 19U * sizeof
                                  (double));
                      break;
                    }
                  }
                }

                for (i = 0; i < 6; i++) {
                  obj->isActiveIdx[i] = obj->isActiveIdxRegularized[i];
                }
              }
              break;

             default:
              {
                int i;
                int idx_lb;
                obj->nVar = 28;
                obj->mConstr = 39;
                for (i = 0; i < 5; i++) {
                  obj->sizes[i] = obj->sizesRegPhaseOne[i];
                }

                for (i = 0; i < 6; i++) {
                  obj->isActiveIdx[i] = obj->isActiveIdxRegPhaseOne[i];
                }

                for (idx_lb = 0; idx_lb < 19; idx_lb++) {
                  obj->Aineq[28 * idx_lb + 27] = -1.0;
                }

                obj->indexLB[obj->sizes[3] - 1] = 28;
                obj->lb[27] = 1.0E-5;
                i = obj->nActiveConstr;
                for (idx_lb = 1; idx_lb <= i; idx_lb++) {
                  obj->ATwset[28 * (idx_lb - 1) + 27] = -1.0;
                }

                obj->isActiveConstr[obj->isActiveIdx[4] - 2] = false;
              }
              break;
            }

            obj->probType = PROBLEM_TYPE;
          }
        }
      }
    }
  }
}

// End of code generation (setProblemType.cpp)
