//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  driver1.cpp
//
//  Code generation for function 'driver1'
//


// Include files
#include "driver1.h"
#include "MATLAB_OPT_TANK_CBF_internal_types.h"
#include "PresolveWorkingSet.h"
#include "computeFval.h"
#include "iterate.h"
#include "maxConstraintViolation.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
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
        void driver(const double H[64], const double f[28], d_struct_T *solution,
                    b_struct_T *memspace, e_struct_T *workingset, g_struct_T
                    *qrmanager, i_struct_T *cholmanager, h_struct_T *objective,
                    c_struct_T *options, int runTimeOptions_MaxIterations)
        {
          int idx;
          int idxStartIneq;
          int nVar;
          boolean_T guard1 = false;
          solution->iterations = 0;
          nVar = workingset->nVar - 1;
          guard1 = false;
          if (workingset->probType == 3) {
            idxStartIneq = workingset->sizes[3];
            for (idx = 0; idx < idxStartIneq; idx++) {
              if (workingset->isActiveConstr[idx + 19]) {
                solution->xstar[workingset->indexLB[idx] - 1] = -workingset->
                  lb[workingset->indexLB[idx] - 1];
              }
            }

            initialize::PresolveWorkingSet(solution, memspace, workingset,
              qrmanager);
            if (solution->state >= 0) {
              guard1 = true;
            }
          } else {
            solution->state = 82;
            guard1 = true;
          }

          if (guard1) {
            solution->iterations = 0;
            solution->maxConstr = WorkingSet::maxConstraintViolation(workingset,
              solution->xstar);
            if (solution->maxConstr > 0.001) {
              int PHASEONE;
              int PROBTYPE_ORIG;
              int b_nVar;
              int idxEndIneq;
              int nVarP1;
              PROBTYPE_ORIG = workingset->probType;
              b_nVar = workingset->nVar;
              nVarP1 = workingset->nVar;
              solution->xstar[workingset->nVar] = solution->maxConstr + 1.0;
              if (workingset->probType == 3) {
                PHASEONE = 1;
              } else {
                PHASEONE = 4;
              }

              idxStartIneq = (workingset->nWConstr[0] + workingset->nWConstr[1])
                + 1;
              idxEndIneq = workingset->nActiveConstr;
              for (idx = idxStartIneq; idx <= idxEndIneq; idx++) {
                workingset->isActiveConstr[(workingset->isActiveIdx
                  [workingset->Wid[idx - 1] - 1] + workingset->Wlocalidx[idx - 1])
                  - 2] = false;
              }

              workingset->nWConstr[2] = 0;
              workingset->nWConstr[3] = 0;
              workingset->nWConstr[4] = 0;
              workingset->nActiveConstr = workingset->nWConstr[0] +
                workingset->nWConstr[1];
              WorkingSet::setProblemType(workingset, PHASEONE);
              objective->prev_objtype = objective->objtype;
              objective->prev_nvar = objective->nvar;
              objective->prev_hasLinear = objective->hasLinear;
              objective->objtype = 5;
              objective->nvar = nVarP1 + 1;
              objective->gammaScalar = 1.0;
              objective->hasLinear = true;
              solution->fstar = Objective::computeFval(objective,
                memspace->workspace_double, H, f, solution->xstar);
              solution->state = 5;
              iterate(H, f, solution, memspace, workingset, qrmanager,
                      cholmanager, objective, 1.4901161193847657E-10, 0.001,
                      runTimeOptions_MaxIterations);
              if (workingset->isActiveConstr[(workingset->isActiveIdx[3] +
                   workingset->sizes[3]) - 2]) {
                boolean_T exitg1;
                idx = 0;
                exitg1 = false;
                while ((!exitg1) && (idx + 1 <= workingset->nActiveConstr)) {
                  if ((workingset->Wid[idx] == 4) && (workingset->Wlocalidx[idx]
                       == workingset->sizes[3])) {
                    idxEndIneq = workingset->Wid[idx] - 1;
                    workingset->isActiveConstr[(workingset->
                      isActiveIdx[workingset->Wid[idx] - 1] +
                      workingset->Wlocalidx[idx]) - 2] = false;
                    workingset->Wid[idx] = workingset->Wid
                      [workingset->nActiveConstr - 1];
                    workingset->Wlocalidx[idx] = workingset->
                      Wlocalidx[workingset->nActiveConstr - 1];
                    idxStartIneq = workingset->nVar;
                    for (PHASEONE = 0; PHASEONE < idxStartIneq; PHASEONE++) {
                      workingset->ATwset[PHASEONE + 28 * idx] =
                        workingset->ATwset[PHASEONE + 28 *
                        (workingset->nActiveConstr - 1)];
                    }

                    workingset->bwset[idx] = workingset->bwset
                      [workingset->nActiveConstr - 1];
                    workingset->nActiveConstr--;
                    workingset->nWConstr[idxEndIneq]--;
                    exitg1 = true;
                  } else {
                    idx++;
                  }
                }
              }

              PHASEONE = workingset->nActiveConstr - 1;
              while ((PHASEONE + 1 > 0) && (PHASEONE + 1 > b_nVar)) {
                idxEndIneq = workingset->Wid[PHASEONE] - 1;
                workingset->isActiveConstr[(workingset->isActiveIdx
                  [workingset->Wid[PHASEONE] - 1] + workingset->
                  Wlocalidx[PHASEONE]) - 2] = false;
                workingset->Wid[PHASEONE] = workingset->Wid
                  [workingset->nActiveConstr - 1];
                workingset->Wlocalidx[PHASEONE] = workingset->
                  Wlocalidx[workingset->nActiveConstr - 1];
                idxStartIneq = workingset->nVar;
                for (idx = 0; idx < idxStartIneq; idx++) {
                  workingset->ATwset[idx + 28 * PHASEONE] = workingset->
                    ATwset[idx + 28 * (workingset->nActiveConstr - 1)];
                }

                workingset->bwset[PHASEONE] = workingset->bwset
                  [workingset->nActiveConstr - 1];
                workingset->nActiveConstr--;
                workingset->nWConstr[idxEndIneq]--;
                PHASEONE--;
              }

              solution->maxConstr = solution->xstar[nVarP1];
              WorkingSet::setProblemType(workingset, PROBTYPE_ORIG);
              objective->objtype = objective->prev_objtype;
              objective->nvar = objective->prev_nvar;
              objective->hasLinear = objective->prev_hasLinear;
              options->ObjectiveLimit = rtMinusInf;
              options->StepTolerance = 1.0E-6;
              if (solution->state != 0) {
                solution->maxConstr = WorkingSet::maxConstraintViolation
                  (workingset, solution->xstar);
                if (solution->maxConstr > 0.001) {
                  std::memset(&solution->lambda[0], 0, 39U * sizeof(double));
                  solution->fstar = Objective::computeFval(objective,
                    memspace->workspace_double, H, f, solution->xstar);
                  solution->state = -2;
                } else {
                  if (solution->maxConstr > 0.0) {
                    double maxConstr_new;
                    if (0 <= nVar) {
                      std::memcpy(&solution->searchDir[0], &solution->xstar[0],
                                  (nVar + 1) * sizeof(double));
                    }

                    initialize::PresolveWorkingSet(solution, memspace,
                      workingset, qrmanager);
                    maxConstr_new = WorkingSet::maxConstraintViolation
                      (workingset, solution->xstar);
                    if (maxConstr_new >= solution->maxConstr) {
                      solution->maxConstr = maxConstr_new;
                      if (0 <= nVar) {
                        std::memcpy(&solution->xstar[0], &solution->searchDir[0],
                                    (nVar + 1) * sizeof(double));
                      }
                    }
                  }

                  iterate(H, f, solution, memspace, workingset, qrmanager,
                          cholmanager, objective, options->StepTolerance,
                          options->ObjectiveLimit, runTimeOptions_MaxIterations);
                }
              }
            } else {
              iterate(H, f, solution, memspace, workingset, qrmanager,
                      cholmanager, objective, options->StepTolerance,
                      options->ObjectiveLimit, runTimeOptions_MaxIterations);
            }
          }
        }
      }
    }
  }
}

// End of code generation (driver1.cpp)
