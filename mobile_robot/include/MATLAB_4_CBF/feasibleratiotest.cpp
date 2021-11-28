//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  feasibleratiotest.cpp
//
//  Code generation for function 'feasibleratiotest'
//


// Include files
#include "feasibleratiotest.h"
#include "rt_nonfinite.h"
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
      namespace qpactiveset
      {
        void feasibleratiotest(const double solution_xstar[30], const double
          solution_searchDir[30], double workspace[1290], int workingset_nVar,
          const double workingset_Aineq[630], const double workingset_bineq[21],
          const double workingset_lb[30], const int workingset_indexLB[30],
          const int workingset_sizes[5], const int workingset_isActiveIdx[6],
          const boolean_T workingset_isActiveConstr[43], const int
          workingset_nWConstr[5], boolean_T isPhaseOne, double *alpha, boolean_T
          *newBlocking, int *constrType, int *constrIdx)
        {
          double c;
          double denomTol;
          double u0;
          int i;
          int iac;
          int ix;
          int k;
          *alpha = 1.0E+30;
          *newBlocking = false;
          *constrType = 0;
          *constrIdx = 0;
          denomTol = 2.2204460492503131E-13 * internal::blas::xnrm2
            (workingset_nVar, solution_searchDir);
          if (workingset_nWConstr[2] < 21) {
            int ia;
            for (k = 0; k < 21; k++) {
              workspace[k] = workingset_bineq[k];
              workspace[k] = -workspace[k];
            }

            k = 0;
            for (iac = 0; iac <= 600; iac += 30) {
              ix = 0;
              c = 0.0;
              i = iac + workingset_nVar;
              for (ia = iac + 1; ia <= i; ia++) {
                c += workingset_Aineq[ia - 1] * solution_xstar[ix];
                ix++;
              }

              workspace[k] += c;
              k++;
            }

            std::memset(&workspace[43], 0, 21U * sizeof(double));
            k = 43;
            for (iac = 0; iac <= 600; iac += 30) {
              ix = 0;
              c = 0.0;
              i = iac + workingset_nVar;
              for (ia = iac + 1; ia <= i; ia++) {
                c += workingset_Aineq[ia - 1] * solution_searchDir[ix];
                ix++;
              }

              workspace[k] += c;
              k++;
            }

            for (ix = 0; ix < 21; ix++) {
              if ((workspace[ix + 43] > denomTol) &&
                  (!workingset_isActiveConstr[(workingset_isActiveIdx[2] + ix) -
                   1])) {
                u0 = std::abs(workspace[ix]);
                if ((!(u0 < 0.001 - workspace[ix])) && (!rtIsNaN(0.001 -
                      workspace[ix]))) {
                  u0 = 0.001 - workspace[ix];
                }

                c = u0 / workspace[ix + 43];
                if (c < *alpha) {
                  *alpha = c;
                  *constrType = 3;
                  *constrIdx = ix + 1;
                  *newBlocking = true;
                }
              }
            }
          }

          if (workingset_nWConstr[3] < workingset_sizes[3]) {
            double phaseOneCorrectionP;
            double phaseOneCorrectionX;
            double ratio;
            phaseOneCorrectionX = static_cast<double>(isPhaseOne) *
              solution_xstar[workingset_nVar - 1];
            phaseOneCorrectionP = static_cast<double>(isPhaseOne) *
              solution_searchDir[workingset_nVar - 1];
            i = workingset_sizes[3];
            for (ix = 0; ix <= i - 2; ix++) {
              k = workingset_indexLB[ix];
              c = -solution_searchDir[k - 1] - phaseOneCorrectionP;
              if ((c > denomTol) && (!workingset_isActiveConstr
                                     [(workingset_isActiveIdx[3] + ix) - 1])) {
                ratio = (-solution_xstar[k - 1] - workingset_lb[k - 1]) -
                  phaseOneCorrectionX;
                u0 = std::abs(ratio);
                if ((!(u0 < 0.001 - ratio)) && (!rtIsNaN(0.001 - ratio))) {
                  u0 = 0.001 - ratio;
                }

                c = u0 / c;
                if (c < *alpha) {
                  *alpha = c;
                  *constrType = 4;
                  *constrIdx = ix + 1;
                  *newBlocking = true;
                }
              }
            }

            i = workingset_indexLB[workingset_sizes[3] - 1] - 1;
            c = -solution_searchDir[i];
            if ((c > denomTol) && (!workingset_isActiveConstr
                                   [(workingset_isActiveIdx[3] +
                  workingset_sizes[3]) - 2])) {
              ratio = -solution_xstar[i] - workingset_lb[i];
              u0 = std::abs(ratio);
              if ((!(u0 < 0.001 - ratio)) && (!rtIsNaN(0.001 - ratio))) {
                u0 = 0.001 - ratio;
              }

              c = u0 / c;
              if (c < *alpha) {
                *alpha = c;
                *constrType = 4;
                *constrIdx = workingset_sizes[3];
                *newBlocking = true;
              }
            }
          }

          if (!isPhaseOne) {
            if ((*newBlocking) && (*alpha > 1.0)) {
              *newBlocking = false;
            }

            if (!(*alpha < 1.0)) {
              *alpha = 1.0;
            }
          }
        }
      }
    }
  }
}

// End of code generation (feasibleratiotest.cpp)
