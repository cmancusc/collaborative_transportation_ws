//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  test_exit.cpp
//
//  Code generation for function 'test_exit'
//


// Include files
#include "test_exit.h"
#include "MATLAB_OPT_TANK_internal_types.h"
#include "computeComplError.h"
#include "computeGradLag.h"
#include "computeQ_.h"
#include "factorQRE.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "updateWorkingSetForNewQP.h"
#include "xgemv.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace fminconsqp
      {
        void b_test_exit(struct_T *Flags, b_struct_T *memspace, f_struct_T
                         *MeritFunction, e_struct_T *WorkingSet, d_struct_T
                         *TrialState, g_struct_T *b_QRManager)
        {
          double optimRelativeFactor;
          double s;
          double smax;
          double val;
          int idx_max;
          int ix;
          int mLB;
          int mLambda;
          int nVar;
          int rankR;
          boolean_T dxTooSmall;
          boolean_T exitg1;
          boolean_T isFeasible;
          nVar = WorkingSet->nVar;
          mLB = WorkingSet->sizes[3];
          mLambda = WorkingSet->sizes[3] + 16;
          stopping::computeGradLag(TrialState->gradLag, WorkingSet->nVar,
            TrialState->grad, WorkingSet->Aineq, WorkingSet->indexLB,
            WorkingSet->sizes[3], TrialState->lambdasqp);
          if (WorkingSet->nVar < 1) {
            idx_max = 0;
          } else {
            idx_max = 1;
            if (WorkingSet->nVar > 1) {
              ix = 0;
              smax = std::abs(TrialState->grad[0]);
              for (rankR = 2; rankR <= nVar; rankR++) {
                ix++;
                s = std::abs(TrialState->grad[ix]);
                if (s > smax) {
                  idx_max = rankR;
                  smax = s;
                }
              }
            }
          }

          smax = std::abs(TrialState->grad[idx_max - 1]);
          if ((1.0 > smax) || rtIsNaN(smax)) {
            optimRelativeFactor = 1.0;
          } else {
            optimRelativeFactor = smax;
          }

          if (rtIsInf(optimRelativeFactor) || rtIsNaN(optimRelativeFactor)) {
            optimRelativeFactor = 1.0;
          }

          smax = 0.0;
          for (idx_max = 0; idx_max < 17; idx_max++) {
            if ((!(smax > TrialState->cIneq[idx_max])) && (!rtIsNaN
                 (TrialState->cIneq[idx_max]))) {
              smax = TrialState->cIneq[idx_max];
            }
          }

          MeritFunction->nlpPrimalFeasError = smax;
          if (TrialState->sqpIterations == 0) {
            if ((1.0 > smax) || rtIsNaN(smax)) {
              MeritFunction->feasRelativeFactor = 1.0;
            } else {
              MeritFunction->feasRelativeFactor = smax;
            }
          }

          isFeasible = (smax <= 0.001 * MeritFunction->feasRelativeFactor);
          dxTooSmall = true;
          val = 0.0;
          idx_max = 0;
          exitg1 = false;
          while ((!exitg1) && (idx_max <= WorkingSet->nVar - 1)) {
            dxTooSmall = ((!rtIsInf(TrialState->gradLag[idx_max])) && (!rtIsNaN
              (TrialState->gradLag[idx_max])));
            if (!dxTooSmall) {
              exitg1 = true;
            } else {
              smax = std::abs(TrialState->gradLag[idx_max]);
              if ((!(val > smax)) && (!rtIsNaN(smax))) {
                val = smax;
              }

              idx_max++;
            }
          }

          Flags->gradOK = dxTooSmall;
          MeritFunction->nlpDualFeasError = val;
          if (!dxTooSmall) {
            Flags->done = true;
            if (isFeasible) {
              TrialState->sqpExitFlag = 2;
            } else {
              TrialState->sqpExitFlag = -2;
            }
          } else {
            MeritFunction->nlpComplError = stopping::computeComplError
              (TrialState->cIneq, TrialState->lambdasqp);
            if ((val > MeritFunction->nlpComplError) || rtIsNaN
                (MeritFunction->nlpComplError)) {
              MeritFunction->firstOrderOpt = val;
            } else {
              MeritFunction->firstOrderOpt = MeritFunction->nlpComplError;
            }

            if (TrialState->sqpIterations > 1) {
              double nlpDualFeasErrorTmp;
              stopping::b_computeGradLag(memspace->workspace_double,
                WorkingSet->nVar, TrialState->grad, WorkingSet->Aineq,
                WorkingSet->indexLB, WorkingSet->sizes[3],
                TrialState->lambdasqp_old);
              nlpDualFeasErrorTmp = 0.0;
              idx_max = 0;
              while ((idx_max <= WorkingSet->nVar - 1) && ((!rtIsInf
                       (memspace->workspace_double[idx_max])) && (!rtIsNaN
                       (memspace->workspace_double[idx_max])))) {
                smax = std::abs(memspace->workspace_double[idx_max]);
                if ((!(nlpDualFeasErrorTmp > smax)) && (!rtIsNaN(smax))) {
                  nlpDualFeasErrorTmp = smax;
                }

                idx_max++;
              }

              smax = stopping::computeComplError(TrialState->cIneq,
                TrialState->lambdasqp_old);
              if ((nlpDualFeasErrorTmp > smax) || rtIsNaN(smax)) {
                s = nlpDualFeasErrorTmp;
              } else {
                s = smax;
              }

              if ((!(val > MeritFunction->nlpComplError)) && (!rtIsNaN
                   (MeritFunction->nlpComplError))) {
                val = MeritFunction->nlpComplError;
              }

              if (s < val) {
                MeritFunction->nlpDualFeasError = nlpDualFeasErrorTmp;
                MeritFunction->nlpComplError = smax;
                MeritFunction->firstOrderOpt = s;
                if (0 <= mLambda) {
                  std::memcpy(&TrialState->lambdasqp[0],
                              &TrialState->lambdasqp_old[0], (mLambda + 1) *
                              sizeof(double));
                }
              } else {
                if (0 <= mLambda) {
                  std::memcpy(&TrialState->lambdasqp_old[0],
                              &TrialState->lambdasqp[0], (mLambda + 1) * sizeof
                              (double));
                }
              }
            } else {
              if (0 <= mLambda) {
                std::memcpy(&TrialState->lambdasqp_old[0],
                            &TrialState->lambdasqp[0], (mLambda + 1) * sizeof
                            (double));
              }
            }

            if (isFeasible && (MeritFunction->nlpDualFeasError <= 1.0E-6 *
                               optimRelativeFactor) &&
                (MeritFunction->nlpComplError <= 1.0E-6 * optimRelativeFactor))
            {
              Flags->done = true;
              TrialState->sqpExitFlag = 1;
            } else {
              Flags->done = false;
              if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
                Flags->done = true;
                TrialState->sqpExitFlag = -3;
              } else {
                boolean_T guard1 = false;
                guard1 = false;
                if (TrialState->sqpIterations > 0) {
                  dxTooSmall = true;
                  idx_max = 0;
                  exitg1 = false;
                  while ((!exitg1) && (idx_max <= nVar - 1)) {
                    smax = std::abs(TrialState->xstarsqp[idx_max]);
                    if ((1.0 > smax) || rtIsNaN(smax)) {
                      smax = 1.0;
                    }

                    if (1.0E-6 * smax <= std::abs(TrialState->delta_x[idx_max]))
                    {
                      dxTooSmall = false;
                      exitg1 = true;
                    } else {
                      idx_max++;
                    }
                  }

                  if (dxTooSmall) {
                    if (!isFeasible) {
                      if (Flags->stepType != 2) {
                        Flags->stepType = 2;
                        Flags->failedLineSearch = false;
                        Flags->stepAccepted = false;
                        guard1 = true;
                      } else {
                        Flags->done = true;
                        TrialState->sqpExitFlag = -2;
                      }
                    } else {
                      int nActiveConstr;
                      nActiveConstr = WorkingSet->nActiveConstr;
                      if (WorkingSet->nActiveConstr > 0) {
                        int fullRank_R;
                        internal::updateWorkingSetForNewQP(WorkingSet,
                          TrialState->cIneq);
                        if (0 <= nActiveConstr - 1) {
                          std::memset(&TrialState->lambda[0], 0, nActiveConstr *
                                      sizeof(double));
                        }

                        QRManager::factorQRE(b_QRManager, WorkingSet->ATwset,
                                             nVar, nActiveConstr);
                        QRManager::computeQ_(b_QRManager, b_QRManager->mrows);
                        if (nVar > nActiveConstr) {
                          idx_max = nVar;
                        } else {
                          idx_max = nActiveConstr;
                        }

                        smax = static_cast<double>(idx_max) *
                          2.2204460492503131E-16;
                        if (1.4901161193847656E-8 < smax) {
                          smax = 1.4901161193847656E-8;
                        }

                        smax *= std::abs(b_QRManager->QR[0]);
                        if (nVar < nActiveConstr) {
                          fullRank_R = nVar;
                        } else {
                          fullRank_R = nActiveConstr;
                        }

                        rankR = 0;
                        idx_max = 0;
                        while ((rankR < fullRank_R) && (std::abs(b_QRManager->
                                 QR[idx_max]) > smax)) {
                          rankR++;
                          idx_max += 36;
                        }

                        ::coder::internal::blas::xgemv(nVar, nVar,
                          b_QRManager->Q, TrialState->grad,
                          memspace->workspace_double);
                        if (rankR != 0) {
                          for (int j = rankR; j >= 1; j--) {
                            idx_max = (j + (j - 1) * 35) - 1;
                            memspace->workspace_double[j - 1] /= b_QRManager->
                              QR[idx_max];
                            for (int i = 0; i <= j - 2; i++) {
                              ix = (j - i) - 2;
                              memspace->workspace_double[ix] -=
                                memspace->workspace_double[j - 1] *
                                b_QRManager->QR[(idx_max - i) - 1];
                            }
                          }
                        }

                        if (nActiveConstr < fullRank_R) {
                          fullRank_R = nActiveConstr;
                        }

                        for (idx_max = 0; idx_max < fullRank_R; idx_max++) {
                          TrialState->lambda[b_QRManager->jpvt[idx_max] - 1] =
                            memspace->workspace_double[idx_max];
                        }

                        qpactiveset::parseoutput::sortLambdaQP
                          (TrialState->lambda, WorkingSet->nActiveConstr,
                           WorkingSet->sizes, WorkingSet->isActiveIdx,
                           WorkingSet->Wid, WorkingSet->Wlocalidx,
                           memspace->workspace_double);
                        stopping::b_computeGradLag(memspace->workspace_double,
                          nVar, TrialState->grad, WorkingSet->Aineq,
                          WorkingSet->indexLB, mLB, TrialState->lambda);
                        s = 0.0;
                        idx_max = 0;
                        while ((idx_max <= nVar - 1) && ((!rtIsInf
                                 (memspace->workspace_double[idx_max])) &&
                                (!rtIsNaN(memspace->workspace_double[idx_max]))))
                        {
                          smax = std::abs(memspace->workspace_double[idx_max]);
                          if ((!(s > smax)) && (!rtIsNaN(smax))) {
                            s = smax;
                          }

                          idx_max++;
                        }

                        smax = stopping::computeComplError(TrialState->cIneq,
                          TrialState->lambda);
                        if ((s <= 1.0E-6 * optimRelativeFactor) && (smax <=
                             1.0E-6 * optimRelativeFactor)) {
                          MeritFunction->nlpDualFeasError = s;
                          MeritFunction->nlpComplError = smax;
                          if (s > smax) {
                            smax = s;
                          }

                          MeritFunction->firstOrderOpt = smax;
                          if (0 <= mLambda) {
                            std::memcpy(&TrialState->lambdasqp[0],
                                        &TrialState->lambda[0], (mLambda + 1) *
                                        sizeof(double));
                          }

                          Flags->done = true;
                          TrialState->sqpExitFlag = 1;
                        } else {
                          Flags->done = true;
                          TrialState->sqpExitFlag = 2;
                        }
                      } else {
                        Flags->done = true;
                        TrialState->sqpExitFlag = 2;
                      }
                    }
                  } else {
                    guard1 = true;
                  }
                } else {
                  guard1 = true;
                }

                if (guard1) {
                  if (TrialState->sqpIterations >= 1500) {
                    Flags->done = true;
                    TrialState->sqpExitFlag = 0;
                  } else {
                    if (TrialState->FunctionEvaluations >= 800) {
                      Flags->done = true;
                      TrialState->sqpExitFlag = 0;
                    }
                  }
                }
              }
            }
          }
        }

        void test_exit(f_struct_T *MeritFunction, const e_struct_T *WorkingSet,
                       d_struct_T *TrialState, boolean_T *Flags_gradOK,
                       boolean_T *Flags_fevalOK, boolean_T *Flags_done,
                       boolean_T *Flags_stepAccepted, boolean_T
                       *Flags_failedLineSearch, int *Flags_stepType)
        {
          double optimRelativeFactor;
          double s;
          double smax;
          int idx_max;
          int mLambda;
          int nVar;
          boolean_T exitg1;
          boolean_T isFeasible;
          *Flags_fevalOK = true;
          *Flags_done = false;
          *Flags_stepAccepted = false;
          *Flags_failedLineSearch = false;
          *Flags_stepType = 1;
          nVar = WorkingSet->nVar;
          mLambda = WorkingSet->sizes[3] + 16;
          stopping::computeGradLag(TrialState->gradLag, WorkingSet->nVar,
            TrialState->grad, WorkingSet->Aineq, WorkingSet->indexLB,
            WorkingSet->sizes[3], TrialState->lambdasqp);
          if (WorkingSet->nVar < 1) {
            idx_max = 0;
          } else {
            idx_max = 1;
            if (WorkingSet->nVar > 1) {
              int ix;
              ix = 0;
              smax = std::abs(TrialState->grad[0]);
              for (int k = 2; k <= nVar; k++) {
                ix++;
                s = std::abs(TrialState->grad[ix]);
                if (s > smax) {
                  idx_max = k;
                  smax = s;
                }
              }
            }
          }

          smax = std::abs(TrialState->grad[idx_max - 1]);
          if ((1.0 > smax) || rtIsNaN(smax)) {
            optimRelativeFactor = 1.0;
          } else {
            optimRelativeFactor = smax;
          }

          if (rtIsInf(optimRelativeFactor) || rtIsNaN(optimRelativeFactor)) {
            optimRelativeFactor = 1.0;
          }

          smax = 0.0;
          for (nVar = 0; nVar < 17; nVar++) {
            if ((!(smax > TrialState->cIneq[nVar])) && (!rtIsNaN
                 (TrialState->cIneq[nVar]))) {
              smax = TrialState->cIneq[nVar];
            }
          }

          MeritFunction->nlpPrimalFeasError = smax;
          if ((1.0 > smax) || rtIsNaN(smax)) {
            MeritFunction->feasRelativeFactor = 1.0;
          } else {
            MeritFunction->feasRelativeFactor = smax;
          }

          isFeasible = (smax <= 0.001 * MeritFunction->feasRelativeFactor);
          *Flags_gradOK = true;
          s = 0.0;
          nVar = 0;
          exitg1 = false;
          while ((!exitg1) && (nVar <= WorkingSet->nVar - 1)) {
            *Flags_gradOK = ((!rtIsInf(TrialState->gradLag[nVar])) && (!rtIsNaN
              (TrialState->gradLag[nVar])));
            if (!*Flags_gradOK) {
              exitg1 = true;
            } else {
              smax = std::abs(TrialState->gradLag[nVar]);
              if ((!(s > smax)) && (!rtIsNaN(smax))) {
                s = smax;
              }

              nVar++;
            }
          }

          MeritFunction->nlpDualFeasError = s;
          if (!*Flags_gradOK) {
            *Flags_done = true;
            if (isFeasible) {
              TrialState->sqpExitFlag = 2;
            } else {
              TrialState->sqpExitFlag = -2;
            }
          } else {
            MeritFunction->nlpComplError = 0.0;
            if (s > 0.0) {
              smax = s;
            } else {
              smax = 0.0;
            }

            MeritFunction->firstOrderOpt = smax;
            if (0 <= mLambda) {
              std::memcpy(&TrialState->lambdasqp_old[0], &TrialState->lambdasqp
                          [0], (mLambda + 1) * sizeof(double));
            }

            if (isFeasible && (s <= 1.0E-6 * optimRelativeFactor)) {
              *Flags_done = true;
              TrialState->sqpExitFlag = 1;
            } else if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
              *Flags_done = true;
              TrialState->sqpExitFlag = -3;
            } else {
              if (TrialState->FunctionEvaluations >= 800) {
                *Flags_done = true;
                TrialState->sqpExitFlag = 0;
              }
            }
          }
        }
      }
    }
  }
}

// End of code generation (test_exit.cpp)
