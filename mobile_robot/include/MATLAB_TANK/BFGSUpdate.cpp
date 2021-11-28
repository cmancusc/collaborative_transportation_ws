//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  BFGSUpdate.cpp
//
//  Code generation for function 'BFGSUpdate'
//


// Include files
#include "BFGSUpdate.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace fminconsqp
      {
        boolean_T BFGSUpdate(int nvar, double Bk[64], const double sk[26],
                             double yk[26], double workspace[910])
        {
          double curvatureS;
          double dotSY;
          double theta;
          int i;
          int i1;
          int ia;
          int ix;
          int ixlast;
          int jy;
          boolean_T success;
          dotSY = 0.0;
          for (jy = 0; jy < nvar; jy++) {
            dotSY += sk[jy] * yk[jy];
            workspace[jy] = 0.0;
          }

          ix = 0;
          i = ((nvar - 1) << 3) + 1;
          for (jy = 1; jy <= i; jy += 8) {
            ixlast = 0;
            i1 = (jy + nvar) - 1;
            for (ia = jy; ia <= i1; ia++) {
              workspace[ixlast] += Bk[ia - 1] * sk[ix];
              ixlast++;
            }

            ix++;
          }

          curvatureS = 0.0;
          if (nvar >= 1) {
            for (jy = 0; jy < nvar; jy++) {
              curvatureS += sk[jy] * workspace[jy];
            }
          }

          if (dotSY < 0.2 * curvatureS) {
            theta = 0.8 * curvatureS / (curvatureS - dotSY);
            for (jy = 0; jy < nvar; jy++) {
              yk[jy] *= theta;
            }

            if (!(1.0 - theta == 0.0)) {
              ixlast = nvar - 1;
              for (jy = 0; jy <= ixlast; jy++) {
                yk[jy] += (1.0 - theta) * workspace[jy];
              }
            }

            dotSY = 0.0;
            for (jy = 0; jy < nvar; jy++) {
              dotSY += sk[jy] * yk[jy];
            }
          }

          if ((curvatureS > 2.2204460492503131E-16) && (dotSY >
               2.2204460492503131E-16)) {
            success = true;
          } else {
            success = false;
          }

          if (success) {
            int ijA;
            theta = -1.0 / curvatureS;
            if (!(theta == 0.0)) {
              ixlast = 0;
              jy = 0;
              for (ia = 0; ia < nvar; ia++) {
                if (workspace[jy] != 0.0) {
                  curvatureS = workspace[jy] * theta;
                  ix = 0;
                  i = ixlast + 1;
                  i1 = nvar + ixlast;
                  for (ijA = i; ijA <= i1; ijA++) {
                    Bk[ijA - 1] += workspace[ix] * curvatureS;
                    ix++;
                  }
                }

                jy++;
                ixlast += 8;
              }
            }

            theta = 1.0 / dotSY;
            if (!(theta == 0.0)) {
              ixlast = 0;
              jy = 0;
              for (ia = 0; ia < nvar; ia++) {
                if (yk[jy] != 0.0) {
                  curvatureS = yk[jy] * theta;
                  ix = 0;
                  i = ixlast + 1;
                  i1 = nvar + ixlast;
                  for (ijA = i; ijA <= i1; ijA++) {
                    Bk[ijA - 1] += yk[ix] * curvatureS;
                    ix++;
                  }
                }

                jy++;
                ixlast += 8;
              }
            }
          }

          return success;
        }
      }
    }
  }
}

// End of code generation (BFGSUpdate.cpp)
