//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  anonymous_function.h
//
//  Code generation for function 'anonymous_function'
//


#ifndef ANONYMOUS_FUNCTION_H
#define ANONYMOUS_FUNCTION_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
struct cell_wrap_6
{
  double f1[8];
};

struct cell_7
{
  double f1[6];
  double f2[48];
  double f3[8];
  double f4;
  double f5[8];
};

namespace coder
{
  class anonymous_function
  {
   public:
    cell_wrap_6 tunableEnvironment[1];
  };

  class b_anonymous_function
  {
   public:
    cell_7 tunableEnvironment;
  };
}

#endif

// End of code generation (anonymous_function.h)
