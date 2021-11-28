//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  iterate.h
//
//  Code generation for function 'iterate'
//


#ifndef ITERATE_H
#define ITERATE_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct d_struct_T;
struct b_struct_T;
struct e_struct_T;
struct g_struct_T;
struct i_struct_T;
struct h_struct_T;

// Function Declarations
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace qpactiveset
      {
        void iterate(const double H[64], const double f[26], d_struct_T
                     *solution, b_struct_T *memspace, e_struct_T *workingset,
                     g_struct_T *qrmanager, i_struct_T *cholmanager, h_struct_T *
                     objective, double options_StepTolerance, double
                     options_ObjectiveLimit, int runTimeOptions_MaxIterations);
      }
    }
  }
}

#endif

// End of code generation (iterate.h)
