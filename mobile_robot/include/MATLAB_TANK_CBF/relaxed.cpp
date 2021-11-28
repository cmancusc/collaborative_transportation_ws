//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  relaxed.cpp
//
//  Code generation for function 'relaxed'
//


// Include files
#include "relaxed.h"
#include "MATLAB_OPT_TANK_CBF_internal_types.h"
#include "addBoundToActiveSetMatrix_.h"
#include "driver1.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "sortLambdaQP.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace fminconsqp
      {
        namespace step
        {
          void relaxed(const double Hessian[64], const double grad[28],
                       d_struct_T *TrialState, f_struct_T *MeritFunction,
                       b_struct_T *memspace, e_struct_T *WorkingSet, g_struct_T *
                       b_QRManager, i_struct_T *b_CholManager, h_struct_T
                       *QPObjective, c_struct_T *qpoptions)
          {
            c_struct_T b_qpoptions;
            double beta;
            double constrViolationIneq;
            double rho;
            double s;
            double smax;
            int iac;
            int idx_max;
            int ix;
            int iy;
            int k;
            int mLBOrig;
            int nVarOrig;
            nVarOrig = WorkingSet->nVar;
            beta = 0.0;
            for (ix = 0; ix < nVarOrig; ix++) {
              beta += Hessian[ix + (ix << 3)];
            }

            beta /= static_cast<double>(WorkingSet->nVar);
            if (TrialState->sqpIterations <= 1) {
              mLBOrig = QPObjective->nvar;
              if (QPObjective->nvar < 1) {
                idx_max = 0;
              } else {
                idx_max = 1;
                if (QPObjective->nvar > 1) {
                  ix = 0;
                  smax = std::abs(grad[0]);
                  for (k = 2; k <= mLBOrig; k++) {
                    ix++;
                    s = std::abs(grad[ix]);
                    if (s > smax) {
                      idx_max = k;
                      smax = s;
                    }
                  }
                }
              }

              smax = std::abs(grad[idx_max - 1]);
              if ((1.0 > smax) || rtIsNaN(smax)) {
                smax = 1.0;
              }

              rho = 100.0 * smax;
            } else {
              mLBOrig = WorkingSet->mConstr;
              idx_max = 1;
              ix = 0;
              smax = std::abs(TrialState->lambdasqp[0]);
              for (k = 2; k <= mLBOrig; k++) {
                ix++;
                s = std::abs(TrialState->lambdasqp[ix]);
                if (s > smax) {
                  idx_max = k;
                  smax = s;
                }
              }

              rho = std::abs(TrialState->lambdasqp[idx_max - 1]);
            }

            QPObjective->nvar = WorkingSet->nVar;
            QPObjective->beta = beta;
            QPObjective->rho = rho;
            QPObjective->hasLinear = true;
            QPObjective->objtype = 4;
            qpactiveset::WorkingSet::setProblemType(WorkingSet, 2);
            mLBOrig = WorkingSet->sizes[3] - 18;
            for (k = 0; k < 19; k++) {
              memspace->workspace_double[k] = WorkingSet->bineq[k];
              memspace->workspace_double[k] = -memspace->workspace_double[k];
            }

            iy = 0;
            for (iac = 0; iac <= 504; iac += 28) {
              ix = 0;
              smax = 0.0;
              idx_max = iac + nVarOrig;
              for (k = iac + 1; k <= idx_max; k++) {
                smax += WorkingSet->Aineq[k - 1] * TrialState->xstar[ix];
                ix++;
              }

              memspace->workspace_double[iy] += smax;
              iy++;
            }

            for (ix = 0; ix < 19; ix++) {
              TrialState->xstar[nVarOrig + ix] = static_cast<double>
                (memspace->workspace_double[ix] > 0.0) *
                memspace->workspace_double[ix];
              if (memspace->workspace_double[ix] <= 0.001) {
                qpactiveset::WorkingSet::addBoundToActiveSetMatrix_(WorkingSet,
                  4, mLBOrig + ix);
              }
            }

            mLBOrig = qpoptions->MaxIterations;
            qpoptions->MaxIterations = (qpoptions->MaxIterations +
              WorkingSet->nVar) - nVarOrig;
            b_qpoptions = *qpoptions;
            ::coder::optim::coder::qpactiveset::driver(Hessian, grad, TrialState,
              memspace, WorkingSet, b_QRManager, b_CholManager, QPObjective,
              &b_qpoptions, qpoptions->MaxIterations);
            qpoptions->MaxIterations = mLBOrig;
            iac = 0;
            for (ix = 0; ix < 19; ix++) {
              boolean_T tf;
              tf = WorkingSet->isActiveConstr[((WorkingSet->isActiveIdx[3] +
                WorkingSet->sizes[3]) + ix) - 20];
              memspace->workspace_int[ix] = tf;
              iac += tf;
            }

            if (TrialState->state != -6) {
              idx_max = 26 - nVarOrig;
              mLBOrig = nVarOrig + 1;
              smax = 0.0;
              s = 0.0;
              if (27 - nVarOrig >= 1) {
                for (k = mLBOrig; k < 28; k++) {
                  smax += std::abs(TrialState->xstar[k - 1]);
                }

                ix = nVarOrig;
                iy = nVarOrig;
                for (k = 0; k <= idx_max; k++) {
                  s += TrialState->xstar[ix] * TrialState->xstar[iy];
                  ix++;
                  iy++;
                }
              }

              rho = (TrialState->fstar - rho * smax) - beta / 2.0 * s;
              mLBOrig = nVarOrig + 1;
              beta = MeritFunction->penaltyParam;
              constrViolationIneq = 0.0;
              for (ix = 0; ix < 19; ix++) {
                if (TrialState->cIneq[ix] > 0.0) {
                  constrViolationIneq += TrialState->cIneq[ix];
                }
              }

              smax = MeritFunction->linearizedConstrViol;
              s = 0.0;
              if (27 - nVarOrig >= 1) {
                for (k = mLBOrig; k < 28; k++) {
                  s += std::abs(TrialState->xstar[k - 1]);
                }
              }

              MeritFunction->linearizedConstrViol = s;
              smax = (constrViolationIneq + smax) - s;
              if ((smax > 2.2204460492503131E-16) && (rho > 0.0)) {
                if (TrialState->sqpFval == 0.0) {
                  beta = 1.0;
                } else {
                  beta = 1.5;
                }

                beta = beta * rho / smax;
              }

              if (beta < MeritFunction->penaltyParam) {
                MeritFunction->phi = TrialState->sqpFval + beta *
                  constrViolationIneq;
                if ((MeritFunction->initFval + beta *
                     MeritFunction->initConstrViolationIneq) -
                    MeritFunction->phi > static_cast<double>
                    (MeritFunction->nPenaltyDecreases) *
                    MeritFunction->threshold) {
                  MeritFunction->nPenaltyDecreases++;
                  if ((MeritFunction->nPenaltyDecreases << 1) >
                      TrialState->sqpIterations) {
                    MeritFunction->threshold *= 10.0;
                  }

                  if (!(beta > 1.0E-10)) {
                    beta = 1.0E-10;
                  }

                  MeritFunction->penaltyParam = beta;
                } else {
                  MeritFunction->phi = TrialState->sqpFval +
                    MeritFunction->penaltyParam * constrViolationIneq;
                }
              } else {
                if (!(beta > 1.0E-10)) {
                  beta = 1.0E-10;
                }

                MeritFunction->penaltyParam = beta;
                MeritFunction->phi = TrialState->sqpFval +
                  MeritFunction->penaltyParam * constrViolationIneq;
              }

              smax = rho - MeritFunction->penaltyParam * constrViolationIneq;
              if (!(smax < 0.0)) {
                smax = 0.0;
              }

              MeritFunction->phiPrimePlus = smax;
              mLBOrig = WorkingSet->nActiveConstr;
              for (ix = 1; ix <= mLBOrig; ix++) {
                if (WorkingSet->Wid[ix - 1] == 3) {
                  TrialState->lambda[ix - 1] *= static_cast<double>
                    (memspace->workspace_int[WorkingSet->Wlocalidx[ix - 1] - 1]);
                }
              }
            }

            ix = WorkingSet->nActiveConstr - 1;
            while ((ix + 1 > 0) && (iac > 0)) {
              if ((WorkingSet->Wid[ix] == 4) && (WorkingSet->Wlocalidx[ix] >
                   WorkingSet->sizes[3] - 19)) {
                smax = TrialState->lambda[WorkingSet->nActiveConstr - 1];
                TrialState->lambda[WorkingSet->nActiveConstr - 1] = 0.0;
                TrialState->lambda[ix] = smax;
                mLBOrig = WorkingSet->Wid[ix] - 1;
                WorkingSet->isActiveConstr[(WorkingSet->isActiveIdx
                  [WorkingSet->Wid[ix] - 1] + WorkingSet->Wlocalidx[ix]) - 2] =
                  false;
                WorkingSet->Wid[ix] = WorkingSet->Wid[WorkingSet->nActiveConstr
                  - 1];
                WorkingSet->Wlocalidx[ix] = WorkingSet->Wlocalidx
                  [WorkingSet->nActiveConstr - 1];
                idx_max = WorkingSet->nVar;
                for (iy = 0; iy < idx_max; iy++) {
                  WorkingSet->ATwset[iy + 28 * ix] = WorkingSet->ATwset[iy + 28 *
                    (WorkingSet->nActiveConstr - 1)];
                }

                WorkingSet->bwset[ix] = WorkingSet->bwset
                  [WorkingSet->nActiveConstr - 1];
                WorkingSet->nActiveConstr--;
                WorkingSet->nWConstr[mLBOrig]--;
                iac--;
              }

              ix--;
            }

            QPObjective->nvar = nVarOrig;
            QPObjective->hasLinear = true;
            QPObjective->objtype = 3;
            qpactiveset::WorkingSet::setProblemType(WorkingSet, 3);
            qpactiveset::parseoutput::sortLambdaQP(TrialState->lambda,
              WorkingSet->nActiveConstr, WorkingSet->sizes,
              WorkingSet->isActiveIdx, WorkingSet->Wid, WorkingSet->Wlocalidx,
              memspace->workspace_double);
          }
        }
      }
    }
  }
}

// End of code generation (relaxed.cpp)
