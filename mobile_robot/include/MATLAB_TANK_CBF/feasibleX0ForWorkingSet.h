//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  feasibleX0ForWorkingSet.h
//
//  Code generation for function 'feasibleX0ForWorkingSet'
//


#ifndef FEASIBLEX0FORWORKINGSET_H
#define FEASIBLEX0FORWORKINGSET_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
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
          boolean_T feasibleX0ForWorkingSet(double workspace[1092], double
            xCurrent[28], e_struct_T *workingset, g_struct_T *qrmanager);
        }
      }
    }
  }
}

#endif

// End of code generation (feasibleX0ForWorkingSet.h)
