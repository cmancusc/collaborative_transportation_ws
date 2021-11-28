//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  linearForm_.cpp
//
//  Code generation for function 'linearForm_'
//


// Include files
#include "linearForm_.h"
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
        namespace Objective
        {
          void linearForm_(boolean_T obj_hasLinear, int obj_nvar, double
                           workspace[910], const double H[64], const double f[26],
                           const double x[26])
          {
            int iac;
            int ix;
            ix = 0;
            if (obj_hasLinear) {
              if (0 <= obj_nvar - 1) {
                std::memcpy(&workspace[0], &f[0], obj_nvar * sizeof(double));
              }

              ix = 1;
            }

            if (obj_nvar != 0) {
              int i;
              if ((ix != 1) && (0 <= obj_nvar - 1)) {
                std::memset(&workspace[0], 0, obj_nvar * sizeof(double));
              }

              ix = 0;
              i = obj_nvar * (obj_nvar - 1) + 1;
              for (iac = 1; obj_nvar < 0 ? iac >= i : iac <= i; iac += obj_nvar)
              {
                double c;
                int i1;
                int iy;
                c = 0.5 * x[ix];
                iy = 0;
                i1 = (iac + obj_nvar) - 1;
                for (int ia = iac; ia <= i1; ia++) {
                  workspace[iy] += H[ia - 1] * c;
                  iy++;
                }

                ix++;
              }
            }
          }
        }
      }
    }
  }
}

// End of code generation (linearForm_.cpp)
