//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  test_exit.h
//
//  Code generation for function 'test_exit'
//


#ifndef TEST_EXIT_H
#define TEST_EXIT_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct f_struct_T;
struct e_struct_T;
struct d_struct_T;
struct struct_T;
struct b_struct_T;
struct g_struct_T;

// Function Declarations
namespace coder
{
  namespace optim
  {
    namespace coder
    {
      namespace fminconsqp
      {
        void b_test_exit(struct_T *Flags, b_struct_T *memspace, f_struct_T
                         *MeritFunction, e_struct_T *WorkingSet, d_struct_T
                         *TrialState, g_struct_T *b_QRManager);
        void test_exit(f_struct_T *MeritFunction, const e_struct_T *WorkingSet,
                       d_struct_T *TrialState, boolean_T *Flags_gradOK,
                       boolean_T *Flags_fevalOK, boolean_T *Flags_done,
                       boolean_T *Flags_stepAccepted, boolean_T
                       *Flags_failedLineSearch, int *Flags_stepType);
      }
    }
  }
}

#endif

// End of code generation (test_exit.h)
