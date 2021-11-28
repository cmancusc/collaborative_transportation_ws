//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  step.cpp
//
//  Code generation for function 'step'
//


// Include files
#include "step.h"
#include "MATLAB_OPT_TANK_internal_types.h"
#include "addBoundToActiveSetMatrix_.h"
#include "driver1.h"
#include "relaxed.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "xnrm2.h"
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
        boolean_T b_step(int *STEP_TYPE, double Hessian[64], d_struct_T
                         *TrialState, f_struct_T *MeritFunction, b_struct_T
                         *memspace, e_struct_T *WorkingSet, g_struct_T
                         *b_QRManager, i_struct_T *b_CholManager, h_struct_T
                         *QPObjective, c_struct_T *qpoptions)
        {
          c_struct_T b_qpoptions;
          double c;
          double constrViolationIneq;
          int idx_upper;
          int nVar;
          boolean_T stepSuccess;
          stepSuccess = true;
          nVar = WorkingSet->nVar - 1;
          if (*STEP_TYPE != 3) {
            if (0 <= nVar) {
              std::memcpy(&TrialState->xstar[0], &TrialState->xstarsqp[0], (nVar
                + 1) * sizeof(double));
            }
          } else {
            if (0 <= nVar) {
              std::memcpy(&TrialState->searchDir[0], &TrialState->xstar[0],
                          (nVar + 1) * sizeof(double));
            }
          }

          int exitg1;
          boolean_T guard1 = false;
          do {
            double penaltyParamTrial;
            int idx;
            int idx_Partition;
            int ix;
            int iy;
            int k;
            exitg1 = 0;
            guard1 = false;
            switch (*STEP_TYPE) {
             case 1:
              b_qpoptions = *qpoptions;
              ::coder::optim::coder::qpactiveset::driver(Hessian,
                TrialState->grad, TrialState, memspace, WorkingSet, b_QRManager,
                b_CholManager, QPObjective, &b_qpoptions,
                qpoptions->MaxIterations);
              if (TrialState->state > 0) {
                penaltyParamTrial = MeritFunction->penaltyParam;
                constrViolationIneq = 0.0;
                for (idx = 0; idx < 17; idx++) {
                  if (TrialState->cIneq[idx] > 0.0) {
                    constrViolationIneq += TrialState->cIneq[idx];
                  }
                }

                c = MeritFunction->linearizedConstrViol;
                MeritFunction->linearizedConstrViol = 0.0;
                c += constrViolationIneq;
                if ((c > 2.2204460492503131E-16) && (TrialState->fstar > 0.0)) {
                  if (TrialState->sqpFval == 0.0) {
                    penaltyParamTrial = 1.0;
                  } else {
                    penaltyParamTrial = 1.5;
                  }

                  penaltyParamTrial = penaltyParamTrial * TrialState->fstar / c;
                }

                if (penaltyParamTrial < MeritFunction->penaltyParam) {
                  MeritFunction->phi = TrialState->sqpFval + penaltyParamTrial *
                    constrViolationIneq;
                  if ((MeritFunction->initFval + penaltyParamTrial *
                       MeritFunction->initConstrViolationIneq) -
                      MeritFunction->phi > static_cast<double>
                      (MeritFunction->nPenaltyDecreases) *
                      MeritFunction->threshold) {
                    MeritFunction->nPenaltyDecreases++;
                    if ((MeritFunction->nPenaltyDecreases << 1) >
                        TrialState->sqpIterations) {
                      MeritFunction->threshold *= 10.0;
                    }

                    if (!(penaltyParamTrial > 1.0E-10)) {
                      penaltyParamTrial = 1.0E-10;
                    }

                    MeritFunction->penaltyParam = penaltyParamTrial;
                  } else {
                    MeritFunction->phi = TrialState->sqpFval +
                      MeritFunction->penaltyParam * constrViolationIneq;
                  }
                } else {
                  if (!(penaltyParamTrial > 1.0E-10)) {
                    penaltyParamTrial = 1.0E-10;
                  }

                  MeritFunction->penaltyParam = penaltyParamTrial;
                  MeritFunction->phi = TrialState->sqpFval +
                    MeritFunction->penaltyParam * constrViolationIneq;
                }

                c = TrialState->fstar - MeritFunction->penaltyParam *
                  constrViolationIneq;
                if (!(c < 0.0)) {
                  c = 0.0;
                }

                MeritFunction->phiPrimePlus = c;
              }

              qpactiveset::parseoutput::sortLambdaQP(TrialState->lambda,
                WorkingSet->nActiveConstr, WorkingSet->sizes,
                WorkingSet->isActiveIdx, WorkingSet->Wid, WorkingSet->Wlocalidx,
                memspace->workspace_double);
              if ((TrialState->state <= 0) && (TrialState->state != -6)) {
                *STEP_TYPE = 2;
              } else {
                if (0 <= nVar) {
                  std::memcpy(&TrialState->delta_x[0], &TrialState->xstar[0],
                              (nVar + 1) * sizeof(double));
                }

                guard1 = true;
              }
              break;

             case 2:
              iy = (WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1]) + 1;
              ix = WorkingSet->nActiveConstr;
              for (idx_upper = iy; idx_upper <= ix; idx_upper++) {
                WorkingSet->isActiveConstr[(WorkingSet->isActiveIdx
                  [WorkingSet->Wid[idx_upper - 1] - 1] + WorkingSet->
                  Wlocalidx[idx_upper - 1]) - 2] = false;
              }

              WorkingSet->nWConstr[2] = 0;
              WorkingSet->nWConstr[3] = 0;
              WorkingSet->nWConstr[4] = 0;
              WorkingSet->nActiveConstr = WorkingSet->nWConstr[0] +
                WorkingSet->nWConstr[1];
              step::relaxed(Hessian, TrialState->grad, TrialState, MeritFunction,
                            memspace, WorkingSet, b_QRManager, b_CholManager,
                            QPObjective, qpoptions);
              if (0 <= nVar) {
                std::memcpy(&TrialState->delta_x[0], &TrialState->xstar[0],
                            (nVar + 1) * sizeof(double));
              }

              guard1 = true;
              break;

             default:
              {
                int b_nVar;
                int nWIneq_old;
                int nWLower_old;
                int nWUpper_old;
                nWIneq_old = WorkingSet->nWConstr[2];
                nWLower_old = WorkingSet->nWConstr[3];
                nWUpper_old = WorkingSet->nWConstr[4];
                b_nVar = WorkingSet->nVar - 1;
                if (0 <= b_nVar) {
                  std::memcpy(&TrialState->xstarsqp[0],
                              &TrialState->xstarsqp_old[0], (b_nVar + 1) *
                              sizeof(double));
                  std::memcpy(&TrialState->socDirection[0], &TrialState->xstar[0],
                              (b_nVar + 1) * sizeof(double));
                }

                std::memcpy(&TrialState->lambda_old[0], &TrialState->lambda[0],
                            35U * sizeof(double));
                for (idx = 0; idx < 17; idx++) {
                  WorkingSet->bineq[idx] = -TrialState->cIneq[idx];
                }

                iy = 0;
                for (idx_upper = 0; idx_upper <= 416; idx_upper += 26) {
                  ix = 0;
                  c = 0.0;
                  k = idx_upper + WorkingSet->nVar;
                  for (idx_Partition = idx_upper + 1; idx_Partition <= k;
                       idx_Partition++) {
                    c += WorkingSet->Aineq[idx_Partition - 1] *
                      TrialState->searchDir[ix];
                    ix++;
                  }

                  WorkingSet->bineq[iy] += c;
                  iy++;
                }

                iy = 1;
                ix = 18;
                idx_upper = WorkingSet->sizes[3] + 18;
                k = WorkingSet->nActiveConstr;
                for (idx = 1; idx <= k; idx++) {
                  switch (WorkingSet->Wid[idx - 1]) {
                   case 3:
                    idx_Partition = iy;
                    iy++;
                    WorkingSet->bwset[idx - 1] = WorkingSet->bineq
                      [WorkingSet->Wlocalidx[idx - 1] - 1];
                    break;

                   case 4:
                    idx_Partition = ix;
                    ix++;
                    break;

                   default:
                    idx_Partition = idx_upper;
                    idx_upper++;
                    break;
                  }

                  TrialState->workingset_old[idx_Partition - 1] =
                    WorkingSet->Wlocalidx[idx - 1];
                }

                if (0 <= b_nVar) {
                  std::memcpy(&TrialState->xstar[0], &TrialState->xstarsqp[0],
                              (b_nVar + 1) * sizeof(double));
                }

                b_qpoptions = *qpoptions;
                ::coder::optim::coder::qpactiveset::driver(Hessian,
                  TrialState->grad, TrialState, memspace, WorkingSet,
                  b_QRManager, b_CholManager, QPObjective, &b_qpoptions,
                  qpoptions->MaxIterations);
                for (idx = 0; idx <= b_nVar; idx++) {
                  c = TrialState->socDirection[idx];
                  TrialState->socDirection[idx] = TrialState->xstar[idx] -
                    TrialState->socDirection[idx];
                  TrialState->xstar[idx] = c;
                }

                stepSuccess = (::coder::internal::blas::xnrm2(b_nVar + 1,
                  TrialState->socDirection) <= 2.0 * ::coder::internal::blas::
                               xnrm2(b_nVar + 1, TrialState->xstar));
                b_nVar = WorkingSet->sizes[3];
                for (idx = 0; idx < 17; idx++) {
                  WorkingSet->bineq[idx] = -TrialState->cIneq[idx];
                }

                if (!stepSuccess) {
                  iy = (WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1]) + 1;
                  ix = WorkingSet->nActiveConstr;
                  for (idx_upper = iy; idx_upper <= ix; idx_upper++) {
                    WorkingSet->isActiveConstr[(WorkingSet->
                      isActiveIdx[WorkingSet->Wid[idx_upper - 1] - 1] +
                      WorkingSet->Wlocalidx[idx_upper - 1]) - 2] = false;
                  }

                  WorkingSet->nWConstr[2] = 0;
                  WorkingSet->nWConstr[3] = 0;
                  WorkingSet->nWConstr[4] = 0;
                  WorkingSet->nActiveConstr = WorkingSet->nWConstr[0] +
                    WorkingSet->nWConstr[1];
                  for (idx = 0; idx < nWIneq_old; idx++) {
                    iy = TrialState->workingset_old[idx];
                    WorkingSet->nWConstr[2]++;
                    WorkingSet->isActiveConstr[(WorkingSet->isActiveIdx[2] + iy)
                      - 2] = true;
                    WorkingSet->nActiveConstr++;
                    WorkingSet->Wid[WorkingSet->nActiveConstr - 1] = 3;
                    WorkingSet->Wlocalidx[WorkingSet->nActiveConstr - 1] = iy;
                    ix = 26 * (iy - 1);
                    idx_upper = 26 * (WorkingSet->nActiveConstr - 1);
                    idx_Partition = WorkingSet->nVar;
                    for (k = 0; k < idx_Partition; k++) {
                      WorkingSet->ATwset[idx_upper + k] = WorkingSet->Aineq[ix +
                        k];
                    }

                    WorkingSet->bwset[WorkingSet->nActiveConstr - 1] =
                      WorkingSet->bineq[iy - 1];
                  }

                  for (idx = 0; idx < nWLower_old; idx++) {
                    qpactiveset::WorkingSet::addBoundToActiveSetMatrix_
                      (WorkingSet, 4, TrialState->workingset_old[idx + 17]);
                  }

                  for (idx = 0; idx < nWUpper_old; idx++) {
                    qpactiveset::WorkingSet::addBoundToActiveSetMatrix_
                      (WorkingSet, 5, TrialState->workingset_old[(idx + b_nVar)
                       + 17]);
                  }

                  std::memcpy(&TrialState->lambda[0], &TrialState->lambda_old[0],
                              35U * sizeof(double));
                } else {
                  qpactiveset::parseoutput::sortLambdaQP(TrialState->lambda,
                    WorkingSet->nActiveConstr, WorkingSet->sizes,
                    WorkingSet->isActiveIdx, WorkingSet->Wid,
                    WorkingSet->Wlocalidx, memspace->workspace_double);
                }

                if (stepSuccess && (TrialState->state != -6)) {
                  for (idx = 0; idx <= nVar; idx++) {
                    TrialState->delta_x[idx] = TrialState->xstar[idx] +
                      TrialState->socDirection[idx];
                  }
                }

                guard1 = true;
              }
              break;
            }

            if (guard1) {
              if (TrialState->state != -6) {
                exitg1 = 1;
              } else {
                c = 0.0;
                penaltyParamTrial = 1.0;
                for (idx = 0; idx < 8; idx++) {
                  constrViolationIneq = std::abs(TrialState->grad[idx]);
                  if ((!(c > constrViolationIneq)) && (!rtIsNaN
                       (constrViolationIneq))) {
                    c = constrViolationIneq;
                  }

                  constrViolationIneq = std::abs(TrialState->xstar[idx]);
                  if ((!(penaltyParamTrial > constrViolationIneq)) && (!rtIsNaN
                       (constrViolationIneq))) {
                    penaltyParamTrial = constrViolationIneq;
                  }
                }

                constrViolationIneq = c / penaltyParamTrial;
                if ((2.2204460492503131E-16 > constrViolationIneq) || rtIsNaN
                    (constrViolationIneq)) {
                  c = 2.2204460492503131E-16;
                } else {
                  c = constrViolationIneq;
                }

                for (ix = 0; ix < 8; ix++) {
                  iy = ix << 3;
                  for (k = 0; k < ix; k++) {
                    Hessian[iy + k] = 0.0;
                  }

                  idx_upper = ix + iy;
                  Hessian[idx_upper] = c;
                  idx_Partition = 6 - ix;
                  if (0 <= idx_Partition) {
                    std::memset(&Hessian[idx_upper + 1], 0, (idx_Partition + 1) *
                                sizeof(double));
                  }
                }
              }
            }
          } while (exitg1 == 0);

          return stepSuccess;
        }
      }
    }
  }
}

// End of code generation (step.cpp)
