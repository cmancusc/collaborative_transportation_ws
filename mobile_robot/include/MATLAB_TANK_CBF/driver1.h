//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  driver1.h
//
//  Code generation for function 'driver1'
//


#ifndef DRIVER1_H
#define DRIVER1_H

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
struct c_struct_T;

// Function Declarations
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace qpactiveset
      {
        void driver(const double H[64], const double f[28], d_struct_T *solution,
                    b_struct_T *memspace, e_struct_T *workingset, g_struct_T
                    *qrmanager, i_struct_T *cholmanager, h_struct_T *objective,
                    c_struct_T *options, int runTimeOptions_MaxIterations);
      }
    }
  }
}

#endif

// End of code generation (driver1.h)
