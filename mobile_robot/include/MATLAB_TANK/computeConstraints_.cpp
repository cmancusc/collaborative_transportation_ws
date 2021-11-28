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
#include "MATLAB_OPT_TANK.h"
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
          int computeConstraints_(MATLAB_OPT_TANK *aInstancePtr, const
            b_anonymous_function *obj_nonlcon, const double x[8], double
            Cineq_workspace[17])
          {
            double varargout_1[17];
            int idx_current;
            int status;
            boolean_T allFinite;
            aInstancePtr->constraints(x, obj_nonlcon->tunableEnvironment.f1,
              obj_nonlcon->tunableEnvironment.f2,
              obj_nonlcon->tunableEnvironment.f3,
              obj_nonlcon->tunableEnvironment.f4,
              obj_nonlcon->tunableEnvironment.f5, varargout_1);
            std::memcpy(&Cineq_workspace[0], &varargout_1[0], 17U * sizeof
                        (double));
            status = 1;
            allFinite = true;
            idx_current = 0;
            while (allFinite && (idx_current + 1 <= 17)) {
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
