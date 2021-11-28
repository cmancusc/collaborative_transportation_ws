//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  deleteColMoveEnd.cpp
//
//  Code generation for function 'deleteColMoveEnd'
//


// Include files
#include "deleteColMoveEnd.h"
#include "MATLAB_OPT_TANK_CBF_internal_types.h"
#include "rt_nonfinite.h"
#include "xrotg.h"
#include <cstring>

// Function Definitions
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace QRManager
      {
        void deleteColMoveEnd(g_struct_T *obj, int idx)
        {
          double x[1521];
          double c;
          double s;
          double temp;
          int i;
          int ix;
          if (obj->usedPivoting) {
            i = 1;
            while ((i <= obj->ncols) && (obj->jpvt[i - 1] != idx)) {
              i++;
            }

            idx = i;
          }

          if (idx >= obj->ncols) {
            obj->ncols--;
          } else {
            int b_i;
            int k;
            obj->jpvt[idx - 1] = obj->jpvt[obj->ncols - 1];
            b_i = obj->minRowCol;
            for (k = 0; k < b_i; k++) {
              obj->QR[k + 39 * (idx - 1)] = obj->QR[k + 39 * (obj->ncols - 1)];
            }

            obj->ncols--;
            ix = obj->mrows;
            i = obj->ncols;
            if (ix < i) {
              i = ix;
            }

            obj->minRowCol = i;
            if (idx < obj->mrows) {
              int b_ix;
              int b_k;
              int endIdx;
              int n;
              ix = obj->mrows - 1;
              endIdx = obj->ncols;
              if (ix < endIdx) {
                endIdx = ix;
              }

              for (k = endIdx; k >= idx; k--) {
                b_i = k + 39 * (idx - 1);
                temp = obj->QR[b_i];
                internal::blas::xrotg(&obj->QR[(k + 39 * (idx - 1)) - 1], &temp,
                                      &c, &s);
                obj->QR[b_i] = temp;
                b_ix = 39 * (k - 1);
                obj->QR[k + b_ix] = 0.0;
                i = k + 39 * idx;
                n = obj->ncols - idx;
                std::memcpy(&x[0], &obj->QR[0], 1521U * sizeof(double));
                if (n >= 1) {
                  ix = i - 1;
                  for (b_k = 0; b_k < n; b_k++) {
                    temp = c * x[ix] + s * x[i];
                    x[i] = c * x[i] - s * x[ix];
                    x[ix] = temp;
                    i += 39;
                    ix += 39;
                  }
                }

                n = obj->mrows;
                for (b_i = 0; b_i < 1521; b_i++) {
                  obj->QR[b_i] = x[b_i];
                  x[b_i] = obj->Q[b_i];
                }

                if (obj->mrows >= 1) {
                  i = b_ix + 39;
                  for (b_k = 0; b_k < n; b_k++) {
                    temp = c * x[b_ix] + s * x[i];
                    x[i] = c * x[i] - s * x[b_ix];
                    x[b_ix] = temp;
                    i++;
                    b_ix++;
                  }
                }

                std::memcpy(&obj->Q[0], &x[0], 1521U * sizeof(double));
              }

              b_i = idx + 1;
              for (k = b_i; k <= endIdx; k++) {
                b_ix = 39 * (k - 1);
                i = k + b_ix;
                temp = obj->QR[i];
                internal::blas::xrotg(&obj->QR[(k + 39 * (k - 1)) - 1], &temp,
                                      &c, &s);
                obj->QR[i] = temp;
                i = k * 40;
                n = obj->ncols - k;
                std::memcpy(&x[0], &obj->QR[0], 1521U * sizeof(double));
                if (n >= 1) {
                  ix = i - 1;
                  for (b_k = 0; b_k < n; b_k++) {
                    temp = c * x[ix] + s * x[i];
                    x[i] = c * x[i] - s * x[ix];
                    x[ix] = temp;
                    i += 39;
                    ix += 39;
                  }
                }

                n = obj->mrows;
                for (i = 0; i < 1521; i++) {
                  obj->QR[i] = x[i];
                  x[i] = obj->Q[i];
                }

                if (obj->mrows >= 1) {
                  i = b_ix + 39;
                  for (b_k = 0; b_k < n; b_k++) {
                    temp = c * x[b_ix] + s * x[i];
                    x[i] = c * x[i] - s * x[b_ix];
                    x[b_ix] = temp;
                    i++;
                    b_ix++;
                  }
                }

                std::memcpy(&obj->Q[0], &x[0], 1521U * sizeof(double));
              }
            }
          }
        }
      }
    }
  }
}

// End of code generation (deleteColMoveEnd.cpp)
