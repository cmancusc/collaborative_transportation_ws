//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  solve.cpp
//
//  Code generation for function 'solve'
//


// Include files
#include "solve.h"
#include "MATLAB_OPT_TANK_internal_types.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace CholManager
      {
        void solve(const i_struct_T *obj, double rhs[26])
        {
          double temp;
          int i;
          int j;
          int jA;
          int n;
          n = obj->ndims;
          if (obj->ndims != 0) {
            for (j = 0; j < n; j++) {
              jA = j * 35;
              temp = rhs[j];
              for (i = 0; i < j; i++) {
                temp -= obj->FMat[jA + i] * rhs[i];
              }

              rhs[j] = temp / obj->FMat[jA + j];
            }
          }

          n = obj->ndims;
          if (obj->ndims != 0) {
            for (j = n; j >= 1; j--) {
              jA = (j + (j - 1) * 35) - 1;
              rhs[j - 1] /= obj->FMat[jA];
              for (i = 0; i <= j - 2; i++) {
                int ix;
                ix = (j - i) - 2;
                rhs[ix] -= rhs[j - 1] * obj->FMat[(jA - i) - 1];
              }
            }
          }
        }
      }
    }
  }
}

// End of code generation (solve.cpp)
