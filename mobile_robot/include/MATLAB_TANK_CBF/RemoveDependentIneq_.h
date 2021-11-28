//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  RemoveDependentIneq_.h
//
//  Code generation for function 'RemoveDependentIneq_'
//


#ifndef REMOVEDEPENDENTINEQ__H
#define REMOVEDEPENDENTINEQ__H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct e_struct_T;
struct g_struct_T;
struct b_struct_T;

// Function Declarations
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace qpactiveset
      {
        namespace initialize
        {
          void RemoveDependentIneq_(e_struct_T *workingset, g_struct_T
            *qrmanager, b_struct_T *memspace, double tolfactor);
        }
      }
    }
  }
}

#endif

// End of code generation (RemoveDependentIneq_.h)
