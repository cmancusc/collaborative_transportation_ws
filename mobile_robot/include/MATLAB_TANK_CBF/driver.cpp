//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  driver.cpp
//
//  Code generation for function 'driver'
//


// Include files
#include "driver.h"
#include "BFGSUpdate.h"
#include "MATLAB_OPT_TANK_CBF.h"
#include "MATLAB_OPT_TANK_CBF_internal_types.h"
#include "anonymous_function.h"
#include "computeConstraints_.h"
#include "computeForwardDifferences.h"
#include "rt_nonfinite.h"
#include "step.h"
#include "test_exit.h"
#include "updateWorkingSetForNewQP.h"
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
        void driver(MATLAB_OPT_TANK_CBF *aInstancePtr, d_struct_T *TrialState,
                    f_struct_T *MeritFunction, const j_struct_T *FcnEvaluator,
                    k_struct_T *FiniteDifferences, b_struct_T *memspace,
                    e_struct_T *WorkingSet, g_struct_T *b_QRManager, h_struct_T *
                    QPObjective, double Hessian[64], i_struct_T *b_CholManager)
        {
          static const signed char iv[64] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 1 };

          static const char qpoptions_SolverName[7] = { 'f', 'm', 'i', 'n', 'c',
            'o', 'n' };

          c_struct_T b_expl_temp;
          c_struct_T expl_temp;
          struct_T Flags;
          double alpha;
          double constrViolationIneq;
          double phi_alpha;
          int iCol;
          int iCol_old;
          int idx_col;
          int k;
          int mConstr;
          int nVar;
          int qpoptions_MaxIterations;
          std::memset(&b_CholManager->FMat[0], 0, 1521U * sizeof(double));
          b_CholManager->ldm = 39;
          b_CholManager->ndims = 0;
          b_CholManager->info = 0;
          for (idx_col = 0; idx_col < 64; idx_col++) {
            Hessian[idx_col] = iv[idx_col];
          }

          nVar = WorkingSet->nVar - 1;
          mConstr = WorkingSet->sizes[3] + 18;
          iCol = WorkingSet->nVar;
          iCol_old = WorkingSet->sizes[3] + 19;
          if (iCol > iCol_old) {
            iCol_old = iCol;
          }

          qpoptions_MaxIterations = 10 * iCol_old;
          TrialState->steplength = 1.0;
          test_exit(MeritFunction, WorkingSet, TrialState, &Flags.gradOK,
                    &Flags.fevalOK, &Flags.done, &Flags.stepAccepted,
                    &Flags.failedLineSearch, &Flags.stepType);
          iCol = -1;
          iCol_old = -1;
          for (idx_col = 0; idx_col < 19; idx_col++) {
            for (k = 0; k <= nVar; k++) {
              TrialState->JacCineqTrans_old[(iCol_old + k) + 1] =
                WorkingSet->Aineq[(iCol + k) + 1];
            }

            iCol += 28;
            iCol_old += 28;
          }

          TrialState->sqpFval_old = TrialState->sqpFval;
          for (k = 0; k < 8; k++) {
            TrialState->xstarsqp_old[k] = TrialState->xstarsqp[k];
            TrialState->grad_old[k] = TrialState->grad[k];
          }

          std::memcpy(&TrialState->cIneq_old[0], &TrialState->cIneq[0], 19U *
                      sizeof(double));
          if (!Flags.done) {
            TrialState->sqpIterations = 1;
          }

          while (!Flags.done) {
            while (!(Flags.stepAccepted || Flags.failedLineSearch)) {
              internal::updateWorkingSetForNewQP(WorkingSet, TrialState->cIneq);
              expl_temp.IterDisplayQP = false;
              expl_temp.RemainFeasible = false;
              expl_temp.ProbRelTolFactor = 1.0;
              expl_temp.ConstrRelTolFactor = 1.0;
              expl_temp.PricingTolerance = 0.0;
              expl_temp.ObjectiveLimit = rtMinusInf;
              expl_temp.ConstraintTolerance = 0.001;
              expl_temp.OptimalityTolerance = 2.2204460492503131E-14;
              expl_temp.StepTolerance = 1.0E-6;
              expl_temp.MaxIterations = qpoptions_MaxIterations;
              for (idx_col = 0; idx_col < 7; idx_col++) {
                expl_temp.SolverName[idx_col] = qpoptions_SolverName[idx_col];
              }

              b_expl_temp = expl_temp;
              Flags.stepAccepted = b_step(&Flags.stepType, Hessian, TrialState,
                MeritFunction, memspace, WorkingSet, b_QRManager, b_CholManager,
                QPObjective, &b_expl_temp);
              if (Flags.stepAccepted) {
                for (iCol = 0; iCol <= nVar; iCol++) {
                  TrialState->xstarsqp[iCol] += TrialState->delta_x[iCol];
                }

                phi_alpha = aInstancePtr->cost_function_cbf(TrialState->xstarsqp,
                  FcnEvaluator->objfun.tunableEnvironment[0].f1);
                iCol = 1;
                if (rtIsInf(phi_alpha) || rtIsNaN(phi_alpha)) {
                  if (rtIsNaN(phi_alpha)) {
                    iCol = -6;
                  } else if (phi_alpha < 0.0) {
                    iCol = -4;
                  } else {
                    iCol = -5;
                  }
                }

                TrialState->sqpFval = phi_alpha;
                if (iCol == 1) {
                  iCol = utils::ObjNonlinEvaluator::computeConstraints_
                    (aInstancePtr, &FcnEvaluator->nonlcon, TrialState->xstarsqp,
                     TrialState->cIneq);
                }

                Flags.fevalOK = (iCol == 1);
                TrialState->FunctionEvaluations++;
                if (Flags.fevalOK) {
                  constrViolationIneq = 0.0;
                  for (iCol = 0; iCol < 19; iCol++) {
                    if (TrialState->cIneq[iCol] > 0.0) {
                      constrViolationIneq += TrialState->cIneq[iCol];
                    }
                  }

                  MeritFunction->phiFullStep = phi_alpha +
                    MeritFunction->penaltyParam * constrViolationIneq;
                } else {
                  MeritFunction->phiFullStep = rtInf;
                }
              }

              if ((Flags.stepType == 1) && Flags.stepAccepted && Flags.fevalOK &&
                  (MeritFunction->phi < MeritFunction->phiFullStep) &&
                  (TrialState->sqpFval < TrialState->sqpFval_old)) {
                Flags.stepType = 3;
                Flags.stepAccepted = false;
              } else {
                boolean_T evalWellDefined;
                boolean_T socTaken;
                if ((Flags.stepType == 3) && Flags.stepAccepted) {
                  socTaken = true;
                } else {
                  socTaken = false;
                }

                evalWellDefined = Flags.fevalOK;
                iCol_old = WorkingSet->nVar - 1;
                alpha = 1.0;
                idx_col = 1;
                phi_alpha = MeritFunction->phiFullStep;
                if (0 <= iCol_old) {
                  std::memcpy(&TrialState->searchDir[0], &TrialState->delta_x[0],
                              (iCol_old + 1) * sizeof(double));
                }

                int exitg1;
                do {
                  exitg1 = 0;
                  if (TrialState->FunctionEvaluations < 800) {
                    if (evalWellDefined && (phi_alpha <= MeritFunction->phi +
                                            alpha * 0.0001 *
                                            MeritFunction->phiPrimePlus)) {
                      exitg1 = 1;
                    } else {
                      boolean_T exitg2;
                      boolean_T tooSmallX;
                      alpha *= 0.7;
                      for (iCol = 0; iCol <= iCol_old; iCol++) {
                        TrialState->delta_x[iCol] = alpha * TrialState->
                          xstar[iCol];
                      }

                      if (socTaken) {
                        phi_alpha = alpha * alpha;
                        if ((iCol_old + 1 >= 1) && (!(phi_alpha == 0.0))) {
                          for (k = 0; k <= iCol_old; k++) {
                            TrialState->delta_x[k] += phi_alpha *
                              TrialState->socDirection[k];
                          }
                        }
                      }

                      tooSmallX = true;
                      iCol = 0;
                      exitg2 = false;
                      while ((!exitg2) && (iCol <= iCol_old)) {
                        phi_alpha = std::abs(TrialState->xstarsqp[iCol]);
                        if ((1.0 > phi_alpha) || rtIsNaN(phi_alpha)) {
                          phi_alpha = 1.0;
                        }

                        if (1.0E-6 * phi_alpha <= std::abs(TrialState->
                             delta_x[iCol])) {
                          tooSmallX = false;
                          exitg2 = true;
                        } else {
                          iCol++;
                        }
                      }

                      if (tooSmallX) {
                        idx_col = -2;
                        exitg1 = 1;
                      } else {
                        for (iCol = 0; iCol <= iCol_old; iCol++) {
                          TrialState->xstarsqp[iCol] = TrialState->
                            xstarsqp_old[iCol] + TrialState->delta_x[iCol];
                        }

                        phi_alpha = aInstancePtr->cost_function_cbf
                          (TrialState->xstarsqp,
                           FcnEvaluator->objfun.tunableEnvironment[0].f1);
                        iCol = 1;
                        if (rtIsInf(phi_alpha) || rtIsNaN(phi_alpha)) {
                          if (rtIsNaN(phi_alpha)) {
                            iCol = -6;
                          } else if (phi_alpha < 0.0) {
                            iCol = -4;
                          } else {
                            iCol = -5;
                          }
                        }

                        TrialState->sqpFval = phi_alpha;
                        if (iCol == 1) {
                          iCol = utils::ObjNonlinEvaluator::computeConstraints_
                            (aInstancePtr, &FcnEvaluator->nonlcon,
                             TrialState->xstarsqp, TrialState->cIneq);
                        }

                        TrialState->FunctionEvaluations++;
                        evalWellDefined = (iCol == 1);
                        if (evalWellDefined) {
                          constrViolationIneq = 0.0;
                          for (iCol = 0; iCol < 19; iCol++) {
                            if (TrialState->cIneq[iCol] > 0.0) {
                              constrViolationIneq += TrialState->cIneq[iCol];
                            }
                          }

                          phi_alpha += MeritFunction->penaltyParam *
                            constrViolationIneq;
                        } else {
                          phi_alpha = rtInf;
                        }
                      }
                    }
                  } else {
                    idx_col = 0;
                    exitg1 = 1;
                  }
                } while (exitg1 == 0);

                Flags.fevalOK = evalWellDefined;
                TrialState->steplength = alpha;
                if (idx_col > 0) {
                  Flags.stepAccepted = true;
                } else {
                  Flags.failedLineSearch = true;
                }
              }
            }

            if (Flags.stepAccepted && (!Flags.failedLineSearch)) {
              for (iCol = 0; iCol <= nVar; iCol++) {
                TrialState->xstarsqp[iCol] = TrialState->xstarsqp_old[iCol] +
                  TrialState->delta_x[iCol];
              }

              for (iCol = 0; iCol <= mConstr; iCol++) {
                TrialState->lambdasqp[iCol] += TrialState->steplength *
                  (TrialState->lambda[iCol] - TrialState->lambdasqp[iCol]);
              }

              TrialState->sqpFval_old = TrialState->sqpFval;
              for (k = 0; k < 8; k++) {
                TrialState->xstarsqp_old[k] = TrialState->xstarsqp[k];
                TrialState->grad_old[k] = TrialState->grad[k];
              }

              std::memcpy(&TrialState->cIneq_old[0], &TrialState->cIneq[0], 19U *
                          sizeof(double));
              Flags.gradOK = utils::FiniteDifferences::internal::
                computeForwardDifferences(aInstancePtr, FiniteDifferences,
                TrialState->sqpFval, TrialState->cIneq, TrialState->xstarsqp,
                TrialState->grad, WorkingSet->Aineq);
              TrialState->FunctionEvaluations += FiniteDifferences->numEvals;
            } else {
              TrialState->sqpFval = TrialState->sqpFval_old;
              std::memcpy(&TrialState->xstarsqp[0], &TrialState->xstarsqp_old[0],
                          8U * sizeof(double));
              std::memcpy(&TrialState->cIneq[0], &TrialState->cIneq_old[0], 19U *
                          sizeof(double));
            }

            b_test_exit(&Flags, memspace, MeritFunction, WorkingSet, TrialState,
                        b_QRManager);
            if ((!Flags.done) && Flags.stepAccepted) {
              int ia;
              Flags.stepAccepted = false;
              Flags.stepType = 1;
              Flags.failedLineSearch = false;
              if (0 <= nVar) {
                std::memcpy(&TrialState->delta_gradLag[0], &TrialState->grad[0],
                            (nVar + 1) * sizeof(double));
              }

              if (nVar + 1 >= 1) {
                for (k = 0; k <= nVar; k++) {
                  TrialState->delta_gradLag[k] += -TrialState->grad_old[k];
                }
              }

              iCol = 0;
              for (k = 0; k <= 504; k += 28) {
                iCol_old = 0;
                idx_col = (k + nVar) + 1;
                for (ia = k + 1; ia <= idx_col; ia++) {
                  TrialState->delta_gradLag[iCol_old] += WorkingSet->Aineq[ia -
                    1] * TrialState->lambdasqp[iCol];
                  iCol_old++;
                }

                iCol++;
              }

              iCol = 0;
              for (k = 0; k <= 504; k += 28) {
                iCol_old = 0;
                idx_col = (k + nVar) + 1;
                for (ia = k + 1; ia <= idx_col; ia++) {
                  TrialState->delta_gradLag[iCol_old] +=
                    TrialState->JacCineqTrans_old[ia - 1] *
                    -TrialState->lambdasqp[iCol];
                  iCol_old++;
                }

                iCol++;
              }

              iCol = -1;
              iCol_old = -1;
              for (idx_col = 0; idx_col < 19; idx_col++) {
                for (k = 0; k <= nVar; k++) {
                  TrialState->JacCineqTrans_old[(iCol_old + k) + 1] =
                    WorkingSet->Aineq[(iCol + k) + 1];
                }

                iCol += 28;
                iCol_old += 28;
              }

              BFGSUpdate(nVar + 1, Hessian, TrialState->delta_x,
                         TrialState->delta_gradLag, memspace->workspace_double);
              TrialState->sqpIterations++;
            }
          }
        }
      }
    }
  }
}

// End of code generation (driver.cpp)
