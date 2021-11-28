//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeConstraints_.cpp
//
//  Code generation for function 'computeConstraints_'
//


// Include files
#include "computeConstraints_.h"
#include "MATLAB_4_CBF.h"
#include "anonymous_function.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace utils
      {
        namespace ObjNonlinEvaluator
        {
          int computeConstraints_(MATLAB_4_CBF *aInstancePtr, const
            b_anonymous_function *obj_nonlcon, const double x[8], double
            Cineq_workspace[21])
          {
            double varargout_1[21];
            int idx_current;
            int status;
            boolean_T allFinite;
            aInstancePtr->constraints_4_cbf(x,
              obj_nonlcon->tunableEnvironment.f1,
              obj_nonlcon->tunableEnvironment.f2,
              obj_nonlcon->tunableEnvironment.f3,
              obj_nonlcon->tunableEnvironment.f4,
              obj_nonlcon->tunableEnvironment.f5,
              obj_nonlcon->tunableEnvironment.f6,
              obj_nonlcon->tunableEnvironment.f7,
              obj_nonlcon->tunableEnvironment.f8,
              obj_nonlcon->tunableEnvironment.f9,
              obj_nonlcon->tunableEnvironment.f10,
              obj_nonlcon->tunableEnvironment.f11,
              obj_nonlcon->tunableEnvironment.f12,
              obj_nonlcon->tunableEnvironment.f13,
              obj_nonlcon->tunableEnvironment.f14, varargout_1);
            std::memcpy(&Cineq_workspace[0], &varargout_1[0], 21U * sizeof
                        (double));
            status = 1;
            allFinite = true;
            idx_current = 0;
            while (allFinite && (idx_current + 1 <= 21)) {
              allFinite = ((!rtIsInf(Cineq_workspace[idx_current])) && (!rtIsNaN
                (Cineq_workspace[idx_current])));
              idx_current++;
            }

            if (!allFinite) {
              idx_current--;
              if (rtIsNaN(Cineq_workspace[idx_current])) {
                status = -3;
              } else if (Cineq_workspace[idx_current] < 0.0) {
                status = -1;
              } else {
                status = -2;
              }
            }

            return status;
          }
        }
      }
    }
  }
}

// End of code generation (computeConstraints_.cpp)
