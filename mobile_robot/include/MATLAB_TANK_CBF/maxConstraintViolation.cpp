//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  maxConstraintViolation.cpp
//
//  Code generation for function 'maxConstraintViolation'
//


// Include files
#include "maxConstraintViolation.h"
#include "MATLAB_OPT_TANK_CBF_internal_types.h"
#include "rt_nonfinite.h"
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
        namespace WorkingSet
        {
          double maxConstraintViolation(e_struct_T *obj, const double x[28])
          {
            double c;
            double v;
            int iac;
            int k;
            int mLB;
            mLB = obj->sizes[3];
            switch (obj->probType) {
             case 2:
              {
                v = 0.0;
                for (k = 0; k < 19; k++) {
                  obj->maxConstrWorkspace[k] = obj->bineq[k];
                  obj->maxConstrWorkspace[k] = -obj->maxConstrWorkspace[k];
                }

                k = 0;
                for (iac = 0; iac <= 504; iac += 28) {
                  int i;
                  int ix;
                  ix = 0;
                  c = 0.0;
                  i = iac + 8;
                  for (int ia = iac + 1; ia <= i; ia++) {
                    c += obj->Aineq[ia - 1] * x[ix];
                    ix++;
                  }

                  obj->maxConstrWorkspace[k] += c;
                  k++;
                }

                for (k = 0; k < 19; k++) {
                  obj->maxConstrWorkspace[k] -= x[k + 8];
                  if ((!(v > obj->maxConstrWorkspace[k])) && (!rtIsNaN
                       (obj->maxConstrWorkspace[k]))) {
                    v = obj->maxConstrWorkspace[k];
                  }
                }
              }
              break;

             default:
              {
                v = 0.0;
                for (k = 0; k < 19; k++) {
                  obj->maxConstrWorkspace[k] = obj->bineq[k];
                  obj->maxConstrWorkspace[k] = -obj->maxConstrWorkspace[k];
                }

                k = 0;
                for (iac = 0; iac <= 504; iac += 28) {
                  int i;
                  int ix;
                  ix = 0;
                  c = 0.0;
                  i = iac + obj->nVar;
                  for (int ia = iac + 1; ia <= i; ia++) {
                    c += obj->Aineq[ia - 1] * x[ix];
                    ix++;
                  }

                  obj->maxConstrWorkspace[k] += c;
                  k++;
                }

                for (k = 0; k < 19; k++) {
                  if ((!(v > obj->maxConstrWorkspace[k])) && (!rtIsNaN
                       (obj->maxConstrWorkspace[k]))) {
                    v = obj->maxConstrWorkspace[k];
                  }
                }
              }
              break;
            }

            if (obj->sizes[3] > 0) {
              for (k = 0; k < mLB; k++) {
                c = -x[obj->indexLB[k] - 1] - obj->lb[obj->indexLB[k] - 1];
                if ((!(v > c)) && (!rtIsNaN(c))) {
                  v = c;
                }
              }
            }

            return v;
          }
        }
      }
    }
  }
}

// End of code generation (maxConstraintViolation.cpp)
