//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeGrad_StoreHx.cpp
//
//  Code generation for function 'computeGrad_StoreHx'
//


// Include files
#include "computeGrad_StoreHx.h"
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
        namespace Objective
        {
          void computeGrad_StoreHx(h_struct_T *obj, const double H[64], const
            double f[28], const double x[28])
          {
            int iac;
            switch (obj->objtype) {
             case 5:
              {
                int i;
                i = obj->nvar;
                if (0 <= i - 2) {
                  std::memset(&obj->grad[0], 0, (i + -1) * sizeof(double));
                }

                obj->grad[obj->nvar - 1] = obj->gammaScalar;
              }
              break;

             case 3:
              {
                int i;
                int ixlast;
                int lda;
                ixlast = obj->nvar - 1;
                lda = obj->nvar;
                if (obj->nvar != 0) {
                  int ix;
                  if (0 <= ixlast) {
                    std::memset(&obj->Hx[0], 0, (ixlast + 1) * sizeof(double));
                  }

                  ix = 0;
                  i = obj->nvar * (obj->nvar - 1) + 1;
                  for (iac = 1; lda < 0 ? iac >= i : iac <= i; iac += lda) {
                    int i1;
                    int iy;
                    iy = 0;
                    i1 = iac + ixlast;
                    for (int ia = iac; ia <= i1; ia++) {
                      obj->Hx[iy] += H[ia - 1] * x[ix];
                      iy++;
                    }

                    ix++;
                  }
                }

                i = obj->nvar;
                if (0 <= i - 1) {
                  std::memcpy(&obj->grad[0], &obj->Hx[0], i * sizeof(double));
                }

                if (obj->hasLinear && (obj->nvar >= 1)) {
                  ixlast = obj->nvar - 1;
                  for (lda = 0; lda <= ixlast; lda++) {
                    obj->grad[lda] += f[lda];
                  }
                }
              }
              break;

             default:
              {
                int i;
                int ixlast;
                int iy;
                int lda;
                ixlast = obj->nvar - 1;
                lda = obj->nvar;
                if (obj->nvar != 0) {
                  int ix;
                  if (0 <= ixlast) {
                    std::memset(&obj->Hx[0], 0, (ixlast + 1) * sizeof(double));
                  }

                  ix = 0;
                  i = obj->nvar * (obj->nvar - 1) + 1;
                  for (iac = 1; lda < 0 ? iac >= i : iac <= i; iac += lda) {
                    int i1;
                    iy = 0;
                    i1 = iac + ixlast;
                    for (int ia = iac; ia <= i1; ia++) {
                      obj->Hx[iy] += H[ia - 1] * x[ix];
                      iy++;
                    }

                    ix++;
                  }
                }

                i = obj->nvar + 1;
                for (ixlast = i; ixlast < 28; ixlast++) {
                  obj->Hx[ixlast - 1] = obj->beta * x[ixlast - 1];
                }

                std::memcpy(&obj->grad[0], &obj->Hx[0], 27U * sizeof(double));
                if (obj->hasLinear && (obj->nvar >= 1)) {
                  ixlast = obj->nvar - 1;
                  for (lda = 0; lda <= ixlast; lda++) {
                    obj->grad[lda] += f[lda];
                  }
                }

                if (27 - obj->nvar >= 1) {
                  iy = obj->nvar;
                  i = 26 - obj->nvar;
                  for (lda = 0; lda <= i; lda++) {
                    obj->grad[iy] += obj->rho;
                    iy++;
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

// End of code generation (computeGrad_StoreHx.cpp)
