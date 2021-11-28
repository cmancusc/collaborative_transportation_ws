//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  compute_deltax.cpp
//
//  Code generation for function 'compute_deltax'
//


// Include files
#include "compute_deltax.h"
#include "MATLAB_4_CBF_internal_types.h"
#include "rt_nonfinite.h"
#include "solve.h"
#include "xgemm.h"
#include "xpotrf.h"
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
        void compute_deltax(const double H[64], d_struct_T *solution, b_struct_T
                            *memspace, const g_struct_T *qrmanager, i_struct_T
                            *cholmanager, const h_struct_T *objective)
        {
          double c;
          int iac;
          int mNull;
          int mNull_tmp;
          int nVar;
          nVar = qrmanager->mrows - 1;
          mNull_tmp = qrmanager->mrows - qrmanager->ncols;
          mNull = mNull_tmp - 1;
          if (mNull_tmp <= 0) {
            if (0 <= nVar) {
              std::memset(&solution->searchDir[0], 0, (nVar + 1) * sizeof(double));
            }
          } else {
            int ix;
            for (ix = 0; ix <= nVar; ix++) {
              solution->searchDir[ix] = -objective->grad[ix];
            }

            if (qrmanager->ncols <= 0) {
              switch (objective->objtype) {
               case 5:
                break;

               case 3:
                {
                  cholmanager->ndims = qrmanager->mrows;
                  for (ix = 0; ix <= nVar; ix++) {
                    int idx_col;
                    int idx_row;
                    idx_row = (nVar + 1) * ix;
                    idx_col = 43 * ix;
                    for (iac = 0; iac <= nVar; iac++) {
                      cholmanager->FMat[idx_col + iac] = H[idx_row + iac];
                    }
                  }

                  cholmanager->info = internal::lapack::xpotrf(qrmanager->mrows,
                    cholmanager->FMat);
                  if (cholmanager->info != 0) {
                    solution->state = -6;
                  } else {
                    CholManager::solve(cholmanager, solution->searchDir);
                  }
                }
                break;

               default:
                {
                  int idx_row;
                  int nVars;
                  nVars = objective->nvar;
                  cholmanager->ndims = objective->nvar;
                  for (ix = 0; ix < nVars; ix++) {
                    int idx_col;
                    idx_row = nVars * ix;
                    idx_col = 43 * ix;
                    for (iac = 0; iac < nVars; iac++) {
                      cholmanager->FMat[idx_col + iac] = H[idx_row + iac];
                    }
                  }

                  cholmanager->info = internal::lapack::xpotrf(objective->nvar,
                    cholmanager->FMat);
                  if (cholmanager->info != 0) {
                    solution->state = -6;
                  } else {
                    int i;
                    CholManager::solve(cholmanager, solution->searchDir);
                    c = 1.0 / objective->beta;
                    idx_row = objective->nvar + 1;
                    i = qrmanager->mrows;
                    for (iac = idx_row; iac <= i; iac++) {
                      solution->searchDir[iac - 1] *= c;
                    }
                  }
                }
                break;
              }
            } else {
              int nullStartIdx_tmp;
              nullStartIdx_tmp = 43 * qrmanager->ncols + 1;
              switch (objective->objtype) {
               case 5:
                {
                  int idx_col;
                  for (ix = 0; ix <= mNull; ix++) {
                    memspace->workspace_double[ix] = -qrmanager->Q[nVar + 43 *
                      (qrmanager->ncols + ix)];
                  }

                  idx_col = qrmanager->mrows - 1;
                  if (qrmanager->mrows != 0) {
                    int i;
                    if (0 <= idx_col) {
                      std::memset(&solution->searchDir[0], 0, (idx_col + 1) *
                                  sizeof(double));
                    }

                    ix = 0;
                    i = nullStartIdx_tmp + 43 * (mNull_tmp - 1);
                    for (iac = nullStartIdx_tmp; iac <= i; iac += 43) {
                      int idx_row;
                      int nVars;
                      idx_row = 0;
                      nVars = iac + idx_col;
                      for (int ia = iac; ia <= nVars; ia++) {
                        solution->searchDir[idx_row] += qrmanager->Q[ia - 1] *
                          memspace->workspace_double[ix];
                        idx_row++;
                      }

                      ix++;
                    }
                  }
                }
                break;

               default:
                {
                  int i;
                  int idx_col;
                  int idx_row;
                  int nVars;
                  switch (objective->objtype) {
                   case 3:
                    internal::blas::xgemm(qrmanager->mrows, mNull_tmp,
                                          qrmanager->mrows, H, qrmanager->mrows,
                                          qrmanager->Q, nullStartIdx_tmp,
                                          memspace->workspace_double);
                    internal::blas::xgemm(mNull_tmp, mNull_tmp, qrmanager->mrows,
                                          qrmanager->Q, nullStartIdx_tmp,
                                          memspace->workspace_double,
                                          cholmanager->FMat);
                    break;

                   default:
                    nVars = qrmanager->mrows;
                    internal::blas::xgemm(objective->nvar, mNull_tmp,
                                          objective->nvar, H, objective->nvar,
                                          qrmanager->Q, nullStartIdx_tmp,
                                          memspace->workspace_double);
                    for (idx_col = 0; idx_col < mNull_tmp; idx_col++) {
                      i = objective->nvar + 1;
                      for (idx_row = i; idx_row <= nVars; idx_row++) {
                        memspace->workspace_double[(idx_row + 43 * idx_col) - 1]
                          = objective->beta * qrmanager->Q[(idx_row + 43 *
                          (idx_col + qrmanager->ncols)) - 1];
                      }
                    }

                    internal::blas::xgemm(mNull_tmp, mNull_tmp, qrmanager->mrows,
                                          qrmanager->Q, nullStartIdx_tmp,
                                          memspace->workspace_double,
                                          cholmanager->FMat);
                    break;
                  }

                  cholmanager->ndims = mNull_tmp;
                  cholmanager->info = internal::lapack::xpotrf(mNull_tmp,
                    cholmanager->FMat);
                  if (cholmanager->info != 0) {
                    solution->state = -6;
                  } else {
                    int ia;
                    if (qrmanager->mrows != 0) {
                      if (0 <= mNull) {
                        std::memset(&memspace->workspace_double[0], 0, (mNull +
                          1) * sizeof(double));
                      }

                      idx_row = 0;
                      i = nullStartIdx_tmp + 43 * (mNull_tmp - 1);
                      for (iac = nullStartIdx_tmp; iac <= i; iac += 43) {
                        ix = 0;
                        c = 0.0;
                        nVars = iac + nVar;
                        for (ia = iac; ia <= nVars; ia++) {
                          c += qrmanager->Q[ia - 1] * objective->grad[ix];
                          ix++;
                        }

                        memspace->workspace_double[idx_row] += -c;
                        idx_row++;
                      }
                    }

                    for (iac = 0; iac <= mNull; iac++) {
                      nVars = iac * 43;
                      c = memspace->workspace_double[iac];
                      for (idx_col = 0; idx_col < iac; idx_col++) {
                        c -= cholmanager->FMat[nVars + idx_col] *
                          memspace->workspace_double[idx_col];
                      }

                      memspace->workspace_double[iac] = c / cholmanager->
                        FMat[nVars + iac];
                    }

                    for (iac = mNull + 1; iac >= 1; iac--) {
                      nVars = (iac + (iac - 1) * 43) - 1;
                      memspace->workspace_double[iac - 1] /= cholmanager->
                        FMat[nVars];
                      for (idx_col = 0; idx_col <= iac - 2; idx_col++) {
                        ix = (iac - idx_col) - 2;
                        memspace->workspace_double[ix] -=
                          memspace->workspace_double[iac - 1] *
                          cholmanager->FMat[(nVars - idx_col) - 1];
                      }
                    }

                    idx_col = qrmanager->mrows - 1;
                    if (qrmanager->mrows != 0) {
                      if (0 <= idx_col) {
                        std::memset(&solution->searchDir[0], 0, (idx_col + 1) *
                                    sizeof(double));
                      }

                      ix = 0;
                      i = nullStartIdx_tmp + 43 * (mNull_tmp - 1);
                      for (iac = nullStartIdx_tmp; iac <= i; iac += 43) {
                        idx_row = 0;
                        nVars = iac + idx_col;
                        for (ia = iac; ia <= nVars; ia++) {
                          solution->searchDir[idx_row] += qrmanager->Q[ia - 1] *
                            memspace->workspace_double[ix];
                          idx_row++;
                        }

                        ix++;
                      }
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

// End of code generation (compute_deltax.cpp)
