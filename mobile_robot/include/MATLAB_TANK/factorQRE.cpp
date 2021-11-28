//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  factorQRE.cpp
//
//  Code generation for function 'factorQRE'
//


// Include files
#include "factorQRE.h"
#include "MATLAB_OPT_TANK_internal_types.h"
#include "rt_nonfinite.h"
#include "xgeqp3.h"

// Function Definitions
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace QRManager
      {
        void factorQRE(g_struct_T *obj, const double A[910], int mrows, int
                       ncols)
        {
          int ix0;
          boolean_T guard1 = false;
          ix0 = mrows * ncols;
          guard1 = false;
          if (ix0 > 0) {
            for (int idx = 0; idx < ncols; idx++) {
              int iy0;
              ix0 = 26 * idx;
              iy0 = 35 * idx;
              for (int k = 0; k < mrows; k++) {
                obj->QR[iy0 + k] = A[ix0 + k];
              }
            }

            guard1 = true;
          } else if (ix0 == 0) {
            obj->mrows = mrows;
            obj->ncols = ncols;
            obj->minRowCol = 0;
          } else {
            guard1 = true;
          }

          if (guard1) {
            obj->usedPivoting = true;
            obj->mrows = mrows;
            obj->ncols = ncols;
            if (mrows < ncols) {
              ix0 = mrows;
            } else {
              ix0 = ncols;
            }

            obj->minRowCol = ix0;
            internal::lapack::xgeqp3(obj->QR, mrows, ncols, obj->jpvt, obj->tau);
          }
        }
      }
    }
  }
}

// End of code generation (factorQRE.cpp)
