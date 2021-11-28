//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  step.h
//
//  Code generation for function 'step'
//


#ifndef STEP_H
#define STEP_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct d_struct_T;
struct f_struct_T;
struct b_struct_T;
struct e_struct_T;
struct g_struct_T;
struct i_struct_T;
struct h_struct_T;
struct c_struct_T;

// Function Declarations
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace fminconsqp
      {
        boolean_T b_step(int *STEP_TYPE, double Hessian[64], d_struct_T
                         *TrialState, f_struct_T *MeritFunction, b_struct_T
                         *memspace, e_struct_T *WorkingSet, g_struct_T
                         *b_QRManager, i_struct_T *b_CholManager, h_struct_T
                         *QPObjective, c_struct_T *qpoptions);
      }
    }
  }
}

#endif

// End of code generation (step.h)
