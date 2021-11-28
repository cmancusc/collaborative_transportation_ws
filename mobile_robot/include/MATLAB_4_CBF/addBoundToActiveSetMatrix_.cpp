//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  addBoundToActiveSetMatrix_.cpp
//
//  Code generation for function 'addBoundToActiveSetMatrix_'
//


// Include files
#include "addBoundToActiveSetMatrix_.h"
#include "MATLAB_4_CBF_internal_types.h"
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
          void addBoundToActiveSetMatrix_(e_struct_T *obj, int TYPE, int
            idx_local)
          {
            int i;
            int i1;
            int idx_bnd_local;
            obj->nWConstr[TYPE - 1]++;
            obj->isActiveConstr[(obj->isActiveIdx[TYPE - 1] + idx_local) - 2] =
              true;
            obj->nActiveConstr++;
            obj->Wid[obj->nActiveConstr - 1] = TYPE;
            obj->Wlocalidx[obj->nActiveConstr - 1] = idx_local;
            i = obj->nActiveConstr - 1;
            switch (TYPE) {
             case 5:
              // A check that is always false is detected at compile-time. Eliminating code that follows. 
              break;

             default:
              idx_bnd_local = obj->indexLB[idx_local - 1] - 1;
              obj->bwset[obj->nActiveConstr - 1] = obj->lb[idx_bnd_local];
              break;
            }

            if (0 <= idx_bnd_local - 1) {
              std::memset(&obj->ATwset[i * 30], 0, ((idx_bnd_local + i * 30) - i
                * 30) * sizeof(double));
            }

            obj->ATwset[idx_bnd_local + 30 * (obj->nActiveConstr - 1)] = 2.0 *
              static_cast<double>(TYPE == 5) - 1.0;
            idx_bnd_local += 2;
            i1 = obj->nVar;
            if (idx_bnd_local <= i1) {
              std::memset(&obj->ATwset[(idx_bnd_local + i * 30) + -1], 0, ((((i1
                + i * 30) - idx_bnd_local) - i * 30) + 1) * sizeof(double));
            }

            switch (obj->probType) {
             case 3:
             case 2:
              break;

             default:
              obj->ATwset[(obj->nVar + 30 * (obj->nActiveConstr - 1)) - 1] =
                -1.0;
              break;
            }
          }
        }
      }
    }
  }
}

// End of code generation (addBoundToActiveSetMatrix_.cpp)
