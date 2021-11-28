//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  PresolveWorkingSet.h
//
//  Code generation for function 'PresolveWorkingSet'
//


#ifndef PRESOLVEWORKINGSET_H
#define PRESOLVEWORKINGSET_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct d_struct_T;
struct b_struct_T;
struct e_struct_T;
struct g_struct_T;

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
          void PresolveWorkingSet(d_struct_T *solution, b_struct_T *memspace,
            e_struct_T *workingset, g_struct_T *qrmanager);
        }
      }
    }
  }
}

#endif

// End of code generation (PresolveWorkingSet.h)
