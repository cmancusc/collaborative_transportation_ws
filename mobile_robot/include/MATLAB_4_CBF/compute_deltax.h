//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  compute_deltax.h
//
//  Code generation for function 'compute_deltax'
//


#ifndef COMPUTE_DELTAX_H
#define COMPUTE_DELTAX_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct d_struct_T;
struct b_struct_T;
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
        void compute_deltax(const double H[64], d_struct_T *solution, b_struct_T
                            *memspace, const g_struct_T *qrmanager, i_struct_T
                            *cholmanager, const h_struct_T *objective);
      }
    }
  }
}

#endif

// End of code generation (compute_deltax.h)
