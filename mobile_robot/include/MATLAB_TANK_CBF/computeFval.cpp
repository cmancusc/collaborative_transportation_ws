//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeFval.cpp
//
//  Code generation for function 'computeFval'
//


// Include files
#include "computeFval.h"
#include "MATLAB_OPT_TANK_CBF_internal_types.h"
#include "linearForm_.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace qpactiveset
      {
        namespace Objective
        {
          double computeFval(const h_struct_T *obj, double workspace[1092],
                             const double H[64], const double f[28], const
                             double x[28])
          {
            double val;
            switch (obj->objtype) {
             case 5:
              val = obj->gammaScalar * x[obj->nvar - 1];
              break;

             case 3:
              {
                linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
                val = 0.0;
                if (obj->nvar >= 1) {
                  int ixlast;
                  ixlast = obj->nvar;
                  for (int idx = 0; idx < ixlast; idx++) {
                    val += x[idx] * workspace[idx];
                  }
                }
              }
              break;

             default:
              {
                int idx;
                int ixlast;
                linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
                ixlast = obj->nvar + 1;
                for (idx = ixlast; idx < 28; idx++) {
                  workspace[idx - 1] = 0.5 * obj->beta * x[idx - 1] + obj->rho;
                }

                val = 0.0;
                for (idx = 0; idx < 27; idx++) {
                  val += x[idx] * workspace[idx];
                }
              }
              break;
            }

            return val;
          }
        }
      }
    }
  }
}

// End of code generation (computeFval.cpp)
