//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  iterate.cpp
//
//  Code generation for function 'iterate'
//


// Include files
#include "iterate.h"
#include "MATLAB_4_CBF_internal_types.h"
#include "addBoundToActiveSetMatrix_.h"
#include "computeFval_ReuseHx.h"
#include "computeGrad_StoreHx.h"
#include "computeQ_.h"
#include "compute_deltax.h"
#include "deleteColMoveEnd.h"
#include "factorQR.h"
#include "feasibleX0ForWorkingSet.h"
#include "feasibleratiotest.h"
#include "maxConstraintViolation.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "xnrm2.h"
#include "xrotg.h"
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
        void iterate(const double H[64], const double f[30], d_struct_T
                     *solution, b_struct_T *memspace, e_struct_T *workingset,
                     g_struct_T *qrmanager, i_struct_T *cholmanager, h_struct_T *
                     objective, double options_StepTolerance, double
                     options_ObjectiveLimit, int runTimeOptions_MaxIterations)
        {
          double c;
          double d;
          double s;
          double temp;
          int TYPE;
          int activeSetChangeID;
          int globalActiveConstrIdx;
          int idx;
          int iyend;
          int j;
          int nVar;
          boolean_T subProblemChanged;
          boolean_T updateFval;
          subProblemChanged = true;
          updateFval = true;
          activeSetChangeID = 0;
          TYPE = objective->objtype;
          nVar = workingset->nVar;
          globalActiveConstrIdx = 0;
          Objective::computeGrad_StoreHx(objective, H, f, solution->xstar);
          solution->fstar = Objective::computeFval_ReuseHx(objective,
            memspace->workspace_double, f, solution->xstar);
          if (solution->iterations < runTimeOptions_MaxIterations) {
            solution->state = -5;
          } else {
            solution->state = 0;
          }

          std::memset(&solution->lambda[0], 0, 43U * sizeof(double));
          int exitg1;
          do {
            exitg1 = 0;
            if (solution->state == -5) {
              int ia;
              int ix;
              int iy;
              boolean_T guard1 = false;
              boolean_T guard2 = false;
              guard1 = false;
              guard2 = false;
              if (subProblemChanged) {
                int ix0;
                switch (activeSetChangeID) {
                 case 1:
                  ix0 = 30 * (workingset->nActiveConstr - 1);
                  iyend = qrmanager->mrows;
                  iy = qrmanager->ncols + 1;
                  if (iyend < iy) {
                    iy = iyend;
                  }

                  qrmanager->minRowCol = iy;
                  iy = 43 * qrmanager->ncols;
                  if (qrmanager->mrows != 0) {
                    iyend = iy + qrmanager->mrows;
                    if (iy + 1 <= iyend) {
                      std::memset(&qrmanager->QR[iy], 0, (iyend - iy) * sizeof
                                  (double));
                    }

                    j = 43 * (qrmanager->mrows - 1) + 1;
                    for (idx = 1; idx <= j; idx += 43) {
                      ix = ix0;
                      c = 0.0;
                      iyend = (idx + qrmanager->mrows) - 1;
                      for (ia = idx; ia <= iyend; ia++) {
                        c += qrmanager->Q[ia - 1] * workingset->ATwset[ix];
                        ix++;
                      }

                      qrmanager->QR[iy] += c;
                      iy++;
                    }
                  }

                  qrmanager->ncols++;
                  qrmanager->jpvt[qrmanager->ncols - 1] = qrmanager->ncols;
                  for (idx = qrmanager->mrows - 1; idx + 1 > qrmanager->ncols;
                       idx--) {
                    d = qrmanager->QR[idx + 43 * (qrmanager->ncols - 1)];
                    internal::blas::xrotg(&qrmanager->QR[(idx + 43 *
                      (qrmanager->ncols - 1)) - 1], &d, &c, &s);
                    qrmanager->QR[idx + 43 * (qrmanager->ncols - 1)] = d;
                    iyend = 43 * (idx - 1);
                    ia = qrmanager->mrows;
                    if (qrmanager->mrows >= 1) {
                      iy = iyend + 43;
                      for (ix = 0; ix < ia; ix++) {
                        temp = c * qrmanager->Q[iyend] + s * qrmanager->Q[iy];
                        qrmanager->Q[iy] = c * qrmanager->Q[iy] - s *
                          qrmanager->Q[iyend];
                        qrmanager->Q[iyend] = temp;
                        iy++;
                        iyend++;
                      }
                    }
                  }
                  break;

                 case -1:
                  QRManager::deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
                  break;

                 default:
                  QRManager::factorQR(qrmanager, workingset->ATwset, nVar,
                                      workingset->nActiveConstr);
                  QRManager::computeQ_(qrmanager, qrmanager->mrows);
                  break;
                }

                compute_deltax(H, solution, memspace, qrmanager, cholmanager,
                               objective);
                if (solution->state != -5) {
                  exitg1 = 1;
                } else if ((internal::blas::xnrm2(nVar, solution->searchDir) <
                            options_StepTolerance) || (workingset->nActiveConstr
                            >= nVar)) {
                  guard2 = true;
                } else {
                  feasibleratiotest(solution->xstar, solution->searchDir,
                                    memspace->workspace_double, workingset->nVar,
                                    workingset->Aineq, workingset->bineq,
                                    workingset->lb, workingset->indexLB,
                                    workingset->sizes, workingset->isActiveIdx,
                                    workingset->isActiveConstr,
                                    workingset->nWConstr, TYPE == 5, &temp,
                                    &updateFval, &j, &iyend);
                  if (updateFval) {
                    switch (j) {
                     case 3:
                      workingset->nWConstr[2]++;
                      workingset->isActiveConstr[(workingset->isActiveIdx[2] +
                        iyend) - 2] = true;
                      workingset->nActiveConstr++;
                      workingset->Wid[workingset->nActiveConstr - 1] = 3;
                      workingset->Wlocalidx[workingset->nActiveConstr - 1] =
                        iyend;
                      ix0 = 30 * (iyend - 1);
                      iy = 30 * (workingset->nActiveConstr - 1);
                      ia = workingset->nVar;
                      for (ix = 0; ix < ia; ix++) {
                        workingset->ATwset[iy + ix] = workingset->Aineq[ix0 + ix];
                      }

                      workingset->bwset[workingset->nActiveConstr - 1] =
                        workingset->bineq[iyend - 1];
                      break;

                     case 4:
                      WorkingSet::addBoundToActiveSetMatrix_(workingset, 4,
                        iyend);
                      break;

                     default:
                      WorkingSet::addBoundToActiveSetMatrix_(workingset, 5,
                        iyend);
                      break;
                    }

                    activeSetChangeID = 1;
                  } else {
                    if (objective->objtype == 5) {
                      if (internal::blas::xnrm2(objective->nvar,
                           solution->searchDir) > 100.0 * static_cast<double>
                          (objective->nvar) * 1.4901161193847656E-8) {
                        solution->state = 3;
                      } else {
                        solution->state = 4;
                      }
                    }

                    subProblemChanged = false;
                    if (workingset->nActiveConstr == 0) {
                      solution->state = 1;
                    }
                  }

                  if ((nVar >= 1) && (!(temp == 0.0))) {
                    iyend = nVar - 1;
                    for (ix = 0; ix <= iyend; ix++) {
                      solution->xstar[ix] += temp * solution->searchDir[ix];
                    }
                  }

                  Objective::computeGrad_StoreHx(objective, H, f,
                    solution->xstar);
                  updateFval = true;
                  guard1 = true;
                }
              } else {
                if (0 <= nVar - 1) {
                  std::memset(&solution->searchDir[0], 0, nVar * sizeof(double));
                }

                guard2 = true;
              }

              if (guard2) {
                iy = qrmanager->ncols;
                if (qrmanager->ncols > 0) {
                  temp = 100.0 * static_cast<double>(qrmanager->mrows) *
                    2.2204460492503131E-16;
                  if ((qrmanager->mrows > 0) && (qrmanager->ncols > 0)) {
                    updateFval = true;
                  } else {
                    updateFval = false;
                  }

                  if (updateFval) {
                    boolean_T b_guard1 = false;
                    idx = qrmanager->ncols;
                    b_guard1 = false;
                    if (qrmanager->mrows < qrmanager->ncols) {
                      while ((idx > qrmanager->mrows) && (std::abs(qrmanager->
                               QR[(qrmanager->mrows + 43 * (idx - 1)) - 1]) >=
                              temp)) {
                        idx--;
                      }

                      updateFval = (idx == qrmanager->mrows);
                      if (updateFval) {
                        b_guard1 = true;
                      }
                    } else {
                      b_guard1 = true;
                    }

                    if (b_guard1) {
                      while ((idx >= 1) && (std::abs(qrmanager->QR[(idx + 43 *
                                (idx - 1)) - 1]) >= temp)) {
                        idx--;
                      }

                      updateFval = (idx == 0);
                    }
                  }

                  if (!updateFval) {
                    solution->state = -7;
                  } else {
                    ia = qrmanager->ncols;
                    internal::blas::xgemv(qrmanager->mrows, qrmanager->ncols,
                                          qrmanager->Q, objective->grad,
                                          memspace->workspace_double);
                    if (qrmanager->ncols != 0) {
                      for (j = ia; j >= 1; j--) {
                        iyend = (j + (j - 1) * 43) - 1;
                        memspace->workspace_double[j - 1] /= qrmanager->QR[iyend];
                        for (idx = 0; idx <= j - 2; idx++) {
                          ix = (j - idx) - 2;
                          memspace->workspace_double[ix] -=
                            memspace->workspace_double[j - 1] * qrmanager->QR
                            [(iyend - idx) - 1];
                        }
                      }
                    }

                    for (idx = 0; idx < iy; idx++) {
                      solution->lambda[idx] = -memspace->workspace_double[idx];
                    }
                  }
                }

                if (solution->state != -7) {
                  ix = -1;
                  temp = 0.0;
                  j = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
                  iyend = workingset->nActiveConstr;
                  for (idx = j; idx <= iyend; idx++) {
                    d = solution->lambda[idx - 1];
                    if (d < temp) {
                      temp = d;
                      ix = idx - 1;
                    }
                  }

                  if (ix + 1 == 0) {
                    solution->state = 1;
                  } else {
                    activeSetChangeID = -1;
                    globalActiveConstrIdx = ix + 1;
                    subProblemChanged = true;
                    iy = workingset->Wid[ix] - 1;
                    workingset->isActiveConstr[(workingset->
                      isActiveIdx[workingset->Wid[ix] - 1] +
                      workingset->Wlocalidx[ix]) - 2] = false;
                    workingset->Wid[ix] = workingset->Wid
                      [workingset->nActiveConstr - 1];
                    workingset->Wlocalidx[ix] = workingset->Wlocalidx
                      [workingset->nActiveConstr - 1];
                    j = workingset->nVar;
                    for (idx = 0; idx < j; idx++) {
                      workingset->ATwset[idx + 30 * ix] = workingset->ATwset[idx
                        + 30 * (workingset->nActiveConstr - 1)];
                    }

                    workingset->bwset[ix] = workingset->bwset
                      [workingset->nActiveConstr - 1];
                    workingset->nActiveConstr--;
                    workingset->nWConstr[iy]--;
                    solution->lambda[ix] = 0.0;
                  }
                } else {
                  ix = workingset->nActiveConstr;
                  activeSetChangeID = 0;
                  globalActiveConstrIdx = workingset->nActiveConstr;
                  subProblemChanged = true;
                  iy = workingset->nActiveConstr - 1;
                  iyend = workingset->Wid[iy] - 1;
                  workingset->isActiveConstr[(workingset->isActiveIdx[iyend] +
                    workingset->Wlocalidx[iy]) - 2] = false;
                  workingset->nActiveConstr--;
                  workingset->nWConstr[iyend]--;
                  solution->lambda[ix - 1] = 0.0;
                }

                updateFval = false;
                guard1 = true;
              }

              if (guard1) {
                solution->iterations++;
                iyend = objective->nvar - 1;
                if ((solution->iterations >= runTimeOptions_MaxIterations) &&
                    ((solution->state != 1) || (objective->objtype == 5))) {
                  solution->state = 0;
                }

                if (solution->iterations - solution->iterations / 50 * 50 == 0)
                {
                  solution->maxConstr = WorkingSet::maxConstraintViolation
                    (workingset, solution->xstar);
                  if (solution->maxConstr > 0.001) {
                    boolean_T nonDegenerateWset;
                    if (0 <= iyend) {
                      std::memcpy(&solution->searchDir[0], &solution->xstar[0],
                                  (iyend + 1) * sizeof(double));
                    }

                    nonDegenerateWset = initialize::feasibleX0ForWorkingSet
                      (memspace->workspace_double, solution->searchDir,
                       workingset, qrmanager);
                    if ((!nonDegenerateWset) && (solution->state != 0)) {
                      solution->state = -2;
                    }

                    activeSetChangeID = 0;
                    temp = WorkingSet::maxConstraintViolation(workingset,
                      solution->searchDir);
                    if (temp < solution->maxConstr) {
                      if (0 <= iyend) {
                        std::memcpy(&solution->xstar[0], &solution->searchDir[0],
                                    (iyend + 1) * sizeof(double));
                      }

                      solution->maxConstr = temp;
                    }
                  }
                }

                if ((options_ObjectiveLimit > rtMinusInf) && updateFval) {
                  solution->fstar = Objective::computeFval_ReuseHx(objective,
                    memspace->workspace_double, f, solution->xstar);
                  if ((solution->fstar < options_ObjectiveLimit) &&
                      ((solution->state != 0) || (objective->objtype != 5))) {
                    solution->state = 2;
                  }
                }
              }
            } else {
              if (!updateFval) {
                solution->fstar = Objective::computeFval_ReuseHx(objective,
                  memspace->workspace_double, f, solution->xstar);
              }

              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }
      }
    }
  }
}

// End of code generation (iterate.cpp)
