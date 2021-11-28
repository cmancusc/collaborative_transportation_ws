//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  driver.h
//
//  Code generation for function 'driver'
//


#ifndef DRIVER_H
#define DRIVER_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
class MATLAB_OPT_TANK;
struct d_struct_T;
struct f_struct_T;
struct j_struct_T;
struct k_struct_T;
struct b_struct_T;
struct e_struct_T;
struct g_struct_T;
struct h_struct_T;
struct i_struct_T;

// Function Declarations
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace fminconsqp
      {
        void driver(MATLAB_OPT_TANK *aInstancePtr, d_struct_T *TrialState,
                    f_struct_T *MeritFunction, const j_struct_T *FcnEvaluator,
                    k_struct_T *FiniteDifferences, b_struct_T *memspace,
                    e_struct_T *WorkingSet, g_struct_T *b_QRManager, h_struct_T *
                    QPObjective, double Hessian[64], i_struct_T *b_CholManager);
      }
    }
  }
}

#endif

// End of code generation (driver.h)
