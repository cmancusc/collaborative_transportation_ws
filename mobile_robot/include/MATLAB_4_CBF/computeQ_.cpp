//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeQ_.cpp
//
//  Code generation for function 'computeQ_'
//


// Include files
#include "computeQ_.h"
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
      namespace QRManager
      {
        void computeQ_(g_struct_T *obj, int nrows)
        {
          double work[43];
          double c;
          int i;
          int iQR0;
          int idx;
          int jy;
          int m;
          i = obj->minRowCol;
          for (idx = 0; idx < i; idx++) {
            iQR0 = 43 * idx + idx;
            jy = obj->mrows - idx;
            if (0 <= jy - 2) {
              std::memcpy(&obj->Q[iQR0 + 1], &obj->QR[iQR0 + 1], (((jy + iQR0) -
                iQR0) + -1) * sizeof(double));
            }
          }

          m = obj->mrows;
          jy = obj->minRowCol;
          if (nrows >= 1) {
            int i1;
            int ia;
            int itau;
            int j;
            i = nrows - 1;
            for (j = jy; j <= i; j++) {
              ia = j * 43;
              i1 = m - 1;
              if (0 <= i1) {
                std::memset(&obj->Q[ia], 0, (((i1 + ia) - ia) + 1) * sizeof
                            (double));
              }

              obj->Q[ia + j] = 1.0;
            }

            itau = obj->minRowCol - 1;
            std::memset(&work[0], 0, 43U * sizeof(double));
            for (int b_i = obj->minRowCol; b_i >= 1; b_i--) {
              int iaii;
              iaii = b_i + (b_i - 1) * 43;
              if (b_i < nrows) {
                int lastc;
                int lastv;
                obj->Q[iaii - 1] = 1.0;
                jy = iaii + 43;
                if (obj->tau[itau] != 0.0) {
                  boolean_T exitg2;
                  lastv = m - b_i;
                  iQR0 = (iaii + m) - b_i;
                  while ((lastv + 1 > 0) && (obj->Q[iQR0 - 1] == 0.0)) {
                    lastv--;
                    iQR0--;
                  }

                  lastc = (nrows - b_i) - 1;
                  exitg2 = false;
                  while ((!exitg2) && (lastc + 1 > 0)) {
                    int exitg1;
                    iQR0 = (iaii + lastc * 43) + 43;
                    ia = iQR0;
                    do {
                      exitg1 = 0;
                      if (ia <= iQR0 + lastv) {
                        if (obj->Q[ia - 1] != 0.0) {
                          exitg1 = 1;
                        } else {
                          ia++;
                        }
                      } else {
                        lastc--;
                        exitg1 = 2;
                      }
                    } while (exitg1 == 0);

                    if (exitg1 == 1) {
                      exitg2 = true;
                    }
                  }
                } else {
                  lastv = -1;
                  lastc = -1;
                }

                if (lastv + 1 > 0) {
                  int ix;
                  if (lastc + 1 != 0) {
                    if (0 <= lastc) {
                      std::memset(&work[0], 0, (lastc + 1) * sizeof(double));
                    }

                    iQR0 = 0;
                    i = (iaii + 43 * lastc) + 43;
                    for (idx = jy; idx <= i; idx += 43) {
                      ix = iaii;
                      c = 0.0;
                      i1 = idx + lastv;
                      for (ia = idx; ia <= i1; ia++) {
                        c += obj->Q[ia - 1] * obj->Q[ix - 1];
                        ix++;
                      }

                      work[iQR0] += c;
                      iQR0++;
                    }
                  }

                  if (!(-obj->tau[itau] == 0.0)) {
                    iQR0 = iaii;
                    jy = 0;
                    for (j = 0; j <= lastc; j++) {
                      if (work[jy] != 0.0) {
                        c = work[jy] * -obj->tau[itau];
                        ix = iaii;
                        i = iQR0 + 43;
                        i1 = lastv + iQR0;
                        for (idx = i; idx <= i1 + 43; idx++) {
                          obj->Q[idx - 1] += obj->Q[ix - 1] * c;
                          ix++;
                        }
                      }

                      jy++;
                      iQR0 += 43;
                    }
                  }
                }
              }

              if (b_i < m) {
                iQR0 = iaii + 1;
                i = (iaii + m) - b_i;
                for (jy = iQR0; jy <= i; jy++) {
                  obj->Q[jy - 1] *= -obj->tau[itau];
                }
              }

              obj->Q[iaii - 1] = 1.0 - obj->tau[itau];
              for (j = 0; j <= b_i - 2; j++) {
                obj->Q[(iaii - j) - 2] = 0.0;
              }

              itau--;
            }
          }
        }
      }
    }
  }
}

// End of code generation (computeQ_.cpp)
