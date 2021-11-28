//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  RemoveDependentIneq_.cpp
//
//  Code generation for function 'RemoveDependentIneq_'
//


// Include files
#include "RemoveDependentIneq_.h"
#include "MATLAB_4_CBF_internal_types.h"
#include "countsort.h"
#include "factorQRE.h"
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
      namespace qpactiveset
      {
        namespace initialize
        {
          void RemoveDependentIneq_(e_struct_T *workingset, g_struct_T
            *qrmanager, b_struct_T *memspace, double tolfactor)
          {
            int nActiveConstr;
            int nFixedConstr;
            nActiveConstr = workingset->nActiveConstr;
            nFixedConstr = workingset->nWConstr[1] + workingset->nWConstr[0];
            if ((workingset->nWConstr[2] + workingset->nWConstr[3]) +
                workingset->nWConstr[4] > 0) {
              double tol;
              int i;
              int idx;
              int nDepIneq;
              tol = tolfactor * static_cast<double>(workingset->nVar) *
                2.2204460492503131E-16;
              for (idx = 0; idx < nFixedConstr; idx++) {
                qrmanager->jpvt[idx] = 1;
              }

              i = nFixedConstr + 1;
              if (i <= nActiveConstr) {
                std::memset(&qrmanager->jpvt[i + -1], 0, ((nActiveConstr - i) +
                  1) * sizeof(int));
              }

              QRManager::factorQRE(qrmanager, workingset->ATwset,
                                   workingset->nVar, workingset->nActiveConstr);
              nDepIneq = 0;
              for (idx = workingset->nActiveConstr; idx > workingset->nVar; idx
                   --) {
                nDepIneq++;
                memspace->workspace_int[nDepIneq - 1] = qrmanager->jpvt[idx - 1];
              }

              if (idx <= workingset->nVar) {
                while ((idx > nFixedConstr) && (std::abs(qrmanager->QR[(idx + 43
                          * (idx - 1)) - 1]) < tol)) {
                  nDepIneq++;
                  memspace->workspace_int[nDepIneq - 1] = qrmanager->jpvt[idx -
                    1];
                  idx--;
                }
              }

              utils::countsort(memspace->workspace_int, nDepIneq,
                               memspace->workspace_sort, nFixedConstr + 1,
                               workingset->nActiveConstr);
              for (idx = nDepIneq; idx >= 1; idx--) {
                nActiveConstr = memspace->workspace_int[idx - 1] - 1;
                nFixedConstr = workingset->Wid[nActiveConstr] - 1;
                workingset->isActiveConstr[(workingset->isActiveIdx[nFixedConstr]
                  + workingset->Wlocalidx[nActiveConstr]) - 2] = false;
                workingset->Wid[nActiveConstr] = workingset->Wid
                  [workingset->nActiveConstr - 1];
                workingset->Wlocalidx[nActiveConstr] = workingset->
                  Wlocalidx[workingset->nActiveConstr - 1];
                i = workingset->nVar;
                for (int b_idx = 0; b_idx < i; b_idx++) {
                  workingset->ATwset[b_idx + 30 * nActiveConstr] =
                    workingset->ATwset[b_idx + 30 * (workingset->nActiveConstr -
                    1)];
                }

                workingset->bwset[nActiveConstr] = workingset->bwset
                  [workingset->nActiveConstr - 1];
                workingset->nActiveConstr--;
                workingset->nWConstr[nFixedConstr]--;
              }
            }
          }
        }
      }
    }
  }
}

// End of code generation (RemoveDependentIneq_.cpp)
