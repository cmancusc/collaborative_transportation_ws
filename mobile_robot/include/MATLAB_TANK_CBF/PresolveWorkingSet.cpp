//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  PresolveWorkingSet.cpp
//
//  Code generation for function 'PresolveWorkingSet'
//


// Include files
#include "PresolveWorkingSet.h"
#include "MATLAB_OPT_TANK_CBF_internal_types.h"
#include "RemoveDependentIneq_.h"
#include "computeQ_.h"
#include "countsort.h"
#include "factorQRE.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "rt_nonfinite.h"
#include "xgeqp3.h"
#include <cmath>
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
        namespace initialize
        {
          void PresolveWorkingSet(d_struct_T *solution, b_struct_T *memspace,
            e_struct_T *workingset, g_struct_T *qrmanager)
          {
            double qtb;
            double tol;
            int idx_col;
            int ix;
            int mTotalWorkingEq_tmp_tmp;
            int mWorkingFixed;
            int nDepInd;
            int nVar;
            solution->state = 82;
            nVar = workingset->nVar - 1;
            mWorkingFixed = workingset->nWConstr[0];
            mTotalWorkingEq_tmp_tmp = workingset->nWConstr[1] +
              workingset->nWConstr[0];
            nDepInd = 0;
            if (mTotalWorkingEq_tmp_tmp > 0) {
              int idx;
              for (ix = 0; ix < mTotalWorkingEq_tmp_tmp; ix++) {
                for (idx_col = 0; idx_col <= nVar; idx_col++) {
                  qrmanager->QR[ix + 39 * idx_col] = workingset->ATwset[idx_col
                    + 28 * ix];
                }
              }

              nDepInd = mTotalWorkingEq_tmp_tmp - workingset->nVar;
              if (0 > nDepInd) {
                nDepInd = 0;
              }

              if (0 <= nVar) {
                std::memset(&qrmanager->jpvt[0], 0, (nVar + 1) * sizeof(int));
              }

              qrmanager->usedPivoting = true;
              qrmanager->mrows = mTotalWorkingEq_tmp_tmp;
              qrmanager->ncols = workingset->nVar;
              nVar = workingset->nVar;
              if (mTotalWorkingEq_tmp_tmp < nVar) {
                nVar = mTotalWorkingEq_tmp_tmp;
              }

              qrmanager->minRowCol = nVar;
              internal::lapack::xgeqp3(qrmanager->QR, mTotalWorkingEq_tmp_tmp,
                workingset->nVar, qrmanager->jpvt, qrmanager->tau);
              tol = 100.0 * static_cast<double>(workingset->nVar) *
                2.2204460492503131E-16;
              nVar = workingset->nVar;
              if (nVar >= mTotalWorkingEq_tmp_tmp) {
                nVar = mTotalWorkingEq_tmp_tmp;
              }

              while ((nVar > 0) && (std::abs(qrmanager->QR[(nVar + 39 * (nVar -
                         1)) - 1]) < tol)) {
                nVar--;
                nDepInd++;
              }

              if (nDepInd > 0) {
                boolean_T exitg1;
                QRManager::computeQ_(qrmanager, mTotalWorkingEq_tmp_tmp);
                idx = 0;
                exitg1 = false;
                while ((!exitg1) && (idx <= nDepInd - 1)) {
                  ix = 39 * ((mTotalWorkingEq_tmp_tmp - idx) - 1);
                  qtb = 0.0;
                  nVar = 0;
                  for (idx_col = 0; idx_col < mTotalWorkingEq_tmp_tmp; idx_col++)
                  {
                    qtb += qrmanager->Q[ix] * workingset->bwset[nVar];
                    ix++;
                    nVar++;
                  }

                  if (std::abs(qtb) >= tol) {
                    nDepInd = -1;
                    exitg1 = true;
                  } else {
                    idx++;
                  }
                }
              }

              if (nDepInd > 0) {
                for (idx = 0; idx < mWorkingFixed; idx++) {
                  qrmanager->jpvt[idx] = 1;
                }

                mWorkingFixed = workingset->nWConstr[0] + 1;
                if (mWorkingFixed <= mTotalWorkingEq_tmp_tmp) {
                  std::memset(&qrmanager->jpvt[mWorkingFixed + -1], 0,
                              ((mTotalWorkingEq_tmp_tmp - mWorkingFixed) + 1) *
                              sizeof(int));
                }

                QRManager::factorQRE(qrmanager, workingset->ATwset,
                                     workingset->nVar, mTotalWorkingEq_tmp_tmp);
                for (idx = 0; idx < nDepInd; idx++) {
                  memspace->workspace_int[idx] = qrmanager->jpvt
                    [(mTotalWorkingEq_tmp_tmp - nDepInd) + idx];
                }

                utils::countsort(memspace->workspace_int, nDepInd,
                                 memspace->workspace_sort, 1,
                                 mTotalWorkingEq_tmp_tmp);
                for (idx = nDepInd; idx >= 1; idx--) {
                  mWorkingFixed = workingset->nWConstr[0] + workingset->
                    nWConstr[1];
                  if (mWorkingFixed != 0) {
                    idx_col = memspace->workspace_int[idx - 1];
                    if (idx_col <= mWorkingFixed) {
                      if ((workingset->nActiveConstr == mWorkingFixed) ||
                          (idx_col == mWorkingFixed)) {
                        workingset->mEqRemoved++;
                        mWorkingFixed = memspace->workspace_int[idx - 1] - 1;
                        ix = workingset->Wid[mWorkingFixed] - 1;
                        workingset->isActiveConstr[(workingset->isActiveIdx[ix]
                          + workingset->Wlocalidx[mWorkingFixed]) - 2] = false;
                        workingset->Wid[mWorkingFixed] = workingset->
                          Wid[workingset->nActiveConstr - 1];
                        workingset->Wlocalidx[mWorkingFixed] =
                          workingset->Wlocalidx[workingset->nActiveConstr - 1];
                        idx_col = workingset->nVar;
                        for (mTotalWorkingEq_tmp_tmp = 0;
                             mTotalWorkingEq_tmp_tmp < idx_col;
                             mTotalWorkingEq_tmp_tmp++) {
                          workingset->ATwset[mTotalWorkingEq_tmp_tmp + 28 *
                            (memspace->workspace_int[idx - 1] - 1)] =
                            workingset->ATwset[mTotalWorkingEq_tmp_tmp + 28 *
                            (workingset->nActiveConstr - 1)];
                        }

                        workingset->bwset[mWorkingFixed] = workingset->
                          bwset[workingset->nActiveConstr - 1];
                        workingset->nActiveConstr--;
                        workingset->nWConstr[ix]--;
                      } else {
                        workingset->mEqRemoved++;
                        nVar = workingset->Wid[idx_col - 1] - 1;
                        workingset->isActiveConstr[(workingset->isActiveIdx[nVar]
                          + workingset->Wlocalidx[idx_col - 1]) - 2] = false;
                        workingset->Wid[idx_col - 1] = workingset->
                          Wid[mWorkingFixed - 1];
                        workingset->Wlocalidx[idx_col - 1] =
                          workingset->Wlocalidx[mWorkingFixed - 1];
                        ix = workingset->nVar;
                        for (mTotalWorkingEq_tmp_tmp = 0;
                             mTotalWorkingEq_tmp_tmp < ix;
                             mTotalWorkingEq_tmp_tmp++) {
                          workingset->ATwset[mTotalWorkingEq_tmp_tmp + 28 *
                            (idx_col - 1)] = workingset->
                            ATwset[mTotalWorkingEq_tmp_tmp + 28 * (mWorkingFixed
                            - 1)];
                        }

                        workingset->bwset[idx_col - 1] = workingset->
                          bwset[mWorkingFixed - 1];
                        workingset->Wid[mWorkingFixed - 1] = workingset->
                          Wid[workingset->nActiveConstr - 1];
                        workingset->Wlocalidx[mWorkingFixed - 1] =
                          workingset->Wlocalidx[workingset->nActiveConstr - 1];
                        idx_col = workingset->nVar;
                        for (mTotalWorkingEq_tmp_tmp = 0;
                             mTotalWorkingEq_tmp_tmp < idx_col;
                             mTotalWorkingEq_tmp_tmp++) {
                          workingset->ATwset[mTotalWorkingEq_tmp_tmp + 28 *
                            (mWorkingFixed - 1)] = workingset->
                            ATwset[mTotalWorkingEq_tmp_tmp + 28 *
                            (workingset->nActiveConstr - 1)];
                        }

                        workingset->bwset[mWorkingFixed - 1] = workingset->
                          bwset[workingset->nActiveConstr - 1];
                        workingset->nActiveConstr--;
                        workingset->nWConstr[nVar]--;
                      }
                    }
                  }
                }
              }
            }

            if (nDepInd != -1) {
              boolean_T guard1 = false;
              boolean_T okWorkingSet;
              RemoveDependentIneq_(workingset, qrmanager, memspace, 100.0);
              okWorkingSet = feasibleX0ForWorkingSet(memspace->workspace_double,
                solution->xstar, workingset, qrmanager);
              guard1 = false;
              if (!okWorkingSet) {
                RemoveDependentIneq_(workingset, qrmanager, memspace, 1000.0);
                okWorkingSet = feasibleX0ForWorkingSet
                  (memspace->workspace_double, solution->xstar, workingset,
                   qrmanager);
                if (!okWorkingSet) {
                  solution->state = -7;
                } else {
                  guard1 = true;
                }
              } else {
                guard1 = true;
              }

              if (guard1 && (workingset->nWConstr[0] + workingset->nWConstr[1] ==
                             workingset->nVar)) {
                tol = WorkingSet::maxConstraintViolation(workingset,
                  solution->xstar);
                if (tol > 0.001) {
                  solution->state = -2;
                }
              }
            } else {
              solution->state = -3;
              nVar = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
              ix = workingset->nActiveConstr;
              for (idx_col = nVar; idx_col <= ix; idx_col++) {
                workingset->isActiveConstr[(workingset->isActiveIdx
                  [workingset->Wid[idx_col - 1] - 1] + workingset->
                  Wlocalidx[idx_col - 1]) - 2] = false;
              }

              workingset->nWConstr[2] = 0;
              workingset->nWConstr[3] = 0;
              workingset->nWConstr[4] = 0;
              workingset->nActiveConstr = workingset->nWConstr[0] +
                workingset->nWConstr[1];
            }
          }
        }
      }
    }
  }
}

// End of code generation (PresolveWorkingSet.cpp)
