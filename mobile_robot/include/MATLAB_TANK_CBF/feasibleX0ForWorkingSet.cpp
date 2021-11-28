//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  feasibleX0ForWorkingSet.cpp
//
//  Code generation for function 'feasibleX0ForWorkingSet'
//


// Include files
#include "feasibleX0ForWorkingSet.h"
#include "MATLAB_OPT_TANK_CBF_internal_types.h"
#include "computeQ_.h"
#include "factorQR.h"
#include "rt_nonfinite.h"
#include "xzgeqp3.h"
#include "rt_nonfinite.h"
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
          boolean_T feasibleX0ForWorkingSet(double workspace[1092], double
            xCurrent[28], e_struct_T *workingset, g_struct_T *qrmanager)
          {
            double B[1092];
            double c;
            int iac;
            int iy;
            int mWConstr;
            int nVar;
            boolean_T nonDegenerateWset;
            mWConstr = workingset->nActiveConstr - 1;
            nVar = workingset->nVar;
            nonDegenerateWset = true;
            if (workingset->nActiveConstr != 0) {
              int i;
              int ia;
              int ix;
              int mLB;
              for (iy = 0; iy <= mWConstr; iy++) {
                workspace[iy] = workingset->bwset[iy];
                workspace[iy + 39] = workingset->bwset[iy];
              }

              if (workingset->nActiveConstr != 0) {
                iy = 0;
                i = 28 * (workingset->nActiveConstr - 1) + 1;
                for (iac = 1; iac <= i; iac += 28) {
                  ix = 0;
                  c = 0.0;
                  mLB = (iac + nVar) - 1;
                  for (ia = iac; ia <= mLB; ia++) {
                    c += workingset->ATwset[ia - 1] * xCurrent[ix];
                    ix++;
                  }

                  workspace[iy] += -c;
                  iy++;
                }
              }

              if (workingset->nActiveConstr >= workingset->nVar) {
                for (iy = 0; iy < nVar; iy++) {
                  for (ix = 0; ix <= mWConstr; ix++) {
                    qrmanager->QR[ix + 39 * iy] = workingset->ATwset[iy + 28 *
                      ix];
                  }
                }

                qrmanager->usedPivoting = false;
                qrmanager->mrows = workingset->nActiveConstr;
                qrmanager->ncols = workingset->nVar;
                i = workingset->nVar;
                for (iy = 0; iy < i; iy++) {
                  qrmanager->jpvt[iy] = iy + 1;
                }

                iy = workingset->nActiveConstr;
                ix = workingset->nVar;
                if (iy < ix) {
                  ix = iy;
                }

                qrmanager->minRowCol = ix;
                std::memset(&qrmanager->tau[0], 0, 39U * sizeof(double));
                if (ix >= 1) {
                  std::memset(&qrmanager->tau[0], 0, 39U * sizeof(double));
                  internal::reflapack::qrf(qrmanager->QR,
                    workingset->nActiveConstr, workingset->nVar, ix,
                    qrmanager->tau);
                }

                QRManager::computeQ_(qrmanager, workingset->nActiveConstr);
                std::memcpy(&B[0], &workspace[0], 1092U * sizeof(double));
                if (1 <= nVar) {
                  std::memset(&workspace[0], 0, nVar * sizeof(double));
                }

                i = nVar + 39;
                if (40 <= i) {
                  std::memset(&workspace[39], 0, (i + -39) * sizeof(double));
                }

                iy = -1;
                for (iac = 1; iac <= nVar; iac++) {
                  c = 0.0;
                  for (ix = 0; ix <= mWConstr; ix++) {
                    c += qrmanager->Q[(ix + iy) + 1] * B[ix];
                  }

                  workspace[iac - 1] += c;
                  iy += 39;
                }

                iy = -1;
                i = nVar + 39;
                for (iac = 40; iac <= i; iac++) {
                  c = 0.0;
                  for (ix = 0; ix <= mWConstr; ix++) {
                    c += qrmanager->Q[(ix + iy) + 1] * B[ix + 39];
                  }

                  workspace[iac - 1] += c;
                  iy += 39;
                }

                for (iac = nVar; iac >= 1; iac--) {
                  iy = 39 * (iac - 1) - 1;
                  c = workspace[iac + -1];
                  if (c != 0.0) {
                    workspace[iac + -1] = c / qrmanager->QR[iac + iy];
                    for (mLB = 0; mLB <= iac - 2; mLB++) {
                      workspace[mLB] -= workspace[iac + -1] * qrmanager->QR[(mLB
                        + iy) + 1];
                    }
                  }
                }

                for (iac = nVar; iac >= 1; iac--) {
                  iy = 39 * (iac - 1) - 1;
                  c = workspace[iac + 38];
                  if (c != 0.0) {
                    workspace[iac + 38] = c / qrmanager->QR[iac + iy];
                    for (mLB = 0; mLB <= iac - 2; mLB++) {
                      workspace[mLB + 39] -= workspace[iac + 38] * qrmanager->
                        QR[(mLB + iy) + 1];
                    }
                  }
                }
              } else {
                QRManager::factorQR(qrmanager, workingset->ATwset,
                                    workingset->nVar, workingset->nActiveConstr);
                QRManager::computeQ_(qrmanager, qrmanager->minRowCol);
                for (mLB = 0; mLB <= mWConstr; mLB++) {
                  ix = 39 * mLB;
                  c = workspace[mLB];
                  for (iac = 0; iac < mLB; iac++) {
                    c -= qrmanager->QR[iac + ix] * workspace[iac];
                  }

                  workspace[mLB] = c / qrmanager->QR[mLB + ix];
                }

                for (mLB = 0; mLB <= mWConstr; mLB++) {
                  ix = 39 * mLB;
                  c = workspace[mLB + 39];
                  for (iac = 0; iac < mLB; iac++) {
                    c -= qrmanager->QR[iac + ix] * workspace[iac + 39];
                  }

                  workspace[mLB + 39] = c / qrmanager->QR[mLB + ix];
                }

                std::memcpy(&B[0], &workspace[0], 1092U * sizeof(double));
                if (1 <= nVar) {
                  std::memset(&workspace[0], 0, nVar * sizeof(double));
                }

                i = nVar + 39;
                if (40 <= i) {
                  std::memset(&workspace[39], 0, (i + -39) * sizeof(double));
                }

                iy = -1;
                i = mWConstr + 1;
                for (ix = 1; ix <= i; ix++) {
                  ia = iy;
                  for (iac = 1; iac <= nVar; iac++) {
                    ia++;
                    workspace[iac - 1] += B[ix - 1] * qrmanager->Q[ia];
                  }

                  iy += 39;
                }

                iy = -1;
                i = mWConstr + 40;
                for (ix = 40; ix <= i; ix++) {
                  ia = iy;
                  mLB = nVar + 39;
                  for (iac = 40; iac <= mLB; iac++) {
                    ia++;
                    workspace[iac - 1] += B[ix - 1] * qrmanager->Q[ia];
                  }

                  iy += 39;
                }
              }

              iy = 0;
              int exitg1;
              do {
                exitg1 = 0;
                if (iy <= nVar - 1) {
                  if (rtIsInf(workspace[iy]) || rtIsNaN(workspace[iy])) {
                    nonDegenerateWset = false;
                    exitg1 = 1;
                  } else {
                    c = workspace[iy + 39];
                    if (rtIsInf(c) || rtIsNaN(c)) {
                      nonDegenerateWset = false;
                      exitg1 = 1;
                    } else {
                      iy++;
                    }
                  }
                } else {
                  double b_v;
                  double v;
                  iy = nVar - 1;
                  for (iac = 0; iac <= iy; iac++) {
                    workspace[iac] += xCurrent[iac];
                  }

                  mLB = workingset->sizes[3];
                  switch (workingset->probType) {
                   case 2:
                    v = 0.0;
                    for (iac = 0; iac < 19; iac++) {
                      workingset->maxConstrWorkspace[iac] = workingset->
                        bineq[iac];
                      workingset->maxConstrWorkspace[iac] =
                        -workingset->maxConstrWorkspace[iac];
                    }

                    iy = 0;
                    for (iac = 0; iac <= 504; iac += 28) {
                      ix = 0;
                      c = 0.0;
                      i = iac + 8;
                      for (ia = iac + 1; ia <= i; ia++) {
                        c += workingset->Aineq[ia - 1] * workspace[ix];
                        ix++;
                      }

                      workingset->maxConstrWorkspace[iy] += c;
                      iy++;
                    }

                    for (iy = 0; iy < 19; iy++) {
                      workingset->maxConstrWorkspace[iy] -= workspace[iy + 8];
                      if ((!(v > workingset->maxConstrWorkspace[iy])) &&
                          (!rtIsNaN(workingset->maxConstrWorkspace[iy]))) {
                        v = workingset->maxConstrWorkspace[iy];
                      }
                    }
                    break;

                   default:
                    v = 0.0;
                    for (iac = 0; iac < 19; iac++) {
                      workingset->maxConstrWorkspace[iac] = workingset->
                        bineq[iac];
                      workingset->maxConstrWorkspace[iac] =
                        -workingset->maxConstrWorkspace[iac];
                    }

                    iy = 0;
                    for (iac = 0; iac <= 504; iac += 28) {
                      ix = 0;
                      c = 0.0;
                      i = iac + workingset->nVar;
                      for (ia = iac + 1; ia <= i; ia++) {
                        c += workingset->Aineq[ia - 1] * workspace[ix];
                        ix++;
                      }

                      workingset->maxConstrWorkspace[iy] += c;
                      iy++;
                    }

                    for (iy = 0; iy < 19; iy++) {
                      if ((!(v > workingset->maxConstrWorkspace[iy])) &&
                          (!rtIsNaN(workingset->maxConstrWorkspace[iy]))) {
                        v = workingset->maxConstrWorkspace[iy];
                      }
                    }
                    break;
                  }

                  if (workingset->sizes[3] > 0) {
                    for (iy = 0; iy < mLB; iy++) {
                      c = -workspace[workingset->indexLB[iy] - 1] -
                        workingset->lb[workingset->indexLB[iy] - 1];
                      if ((!(v > c)) && (!rtIsNaN(c))) {
                        v = c;
                      }
                    }
                  }

                  mLB = workingset->sizes[3];
                  switch (workingset->probType) {
                   case 2:
                    b_v = 0.0;
                    for (iac = 0; iac < 19; iac++) {
                      workingset->maxConstrWorkspace[iac] = workingset->
                        bineq[iac];
                      workingset->maxConstrWorkspace[iac] =
                        -workingset->maxConstrWorkspace[iac];
                    }

                    iy = 0;
                    for (iac = 0; iac <= 504; iac += 28) {
                      ix = 39;
                      c = 0.0;
                      i = iac + 8;
                      for (ia = iac + 1; ia <= i; ia++) {
                        c += workingset->Aineq[ia - 1] * workspace[ix];
                        ix++;
                      }

                      workingset->maxConstrWorkspace[iy] += c;
                      iy++;
                    }

                    for (iy = 0; iy < 19; iy++) {
                      workingset->maxConstrWorkspace[iy] -= workspace[iy + 47];
                      if ((!(b_v > workingset->maxConstrWorkspace[iy])) &&
                          (!rtIsNaN(workingset->maxConstrWorkspace[iy]))) {
                        b_v = workingset->maxConstrWorkspace[iy];
                      }
                    }
                    break;

                   default:
                    b_v = 0.0;
                    for (iac = 0; iac < 19; iac++) {
                      workingset->maxConstrWorkspace[iac] = workingset->
                        bineq[iac];
                      workingset->maxConstrWorkspace[iac] =
                        -workingset->maxConstrWorkspace[iac];
                    }

                    iy = 0;
                    for (iac = 0; iac <= 504; iac += 28) {
                      ix = 39;
                      c = 0.0;
                      i = iac + workingset->nVar;
                      for (ia = iac + 1; ia <= i; ia++) {
                        c += workingset->Aineq[ia - 1] * workspace[ix];
                        ix++;
                      }

                      workingset->maxConstrWorkspace[iy] += c;
                      iy++;
                    }

                    for (iy = 0; iy < 19; iy++) {
                      if ((!(b_v > workingset->maxConstrWorkspace[iy])) &&
                          (!rtIsNaN(workingset->maxConstrWorkspace[iy]))) {
                        b_v = workingset->maxConstrWorkspace[iy];
                      }
                    }
                    break;
                  }

                  if (workingset->sizes[3] > 0) {
                    for (iy = 0; iy < mLB; iy++) {
                      c = -workspace[workingset->indexLB[iy] + 38] -
                        workingset->lb[workingset->indexLB[iy] - 1];
                      if ((!(b_v > c)) && (!rtIsNaN(c))) {
                        b_v = c;
                      }
                    }
                  }

                  if ((v <= 2.2204460492503131E-16) || (v < b_v)) {
                    if (0 <= nVar - 1) {
                      std::memcpy(&xCurrent[0], &workspace[0], nVar * sizeof
                                  (double));
                    }
                  } else {
                    if (0 <= nVar - 1) {
                      std::memcpy(&xCurrent[0], &workspace[39], nVar * sizeof
                                  (double));
                    }
                  }

                  exitg1 = 1;
                }
              } while (exitg1 == 0);
            }

            return nonDegenerateWset;
          }
        }
      }
    }
  }
}

// End of code generation (feasibleX0ForWorkingSet.cpp)
