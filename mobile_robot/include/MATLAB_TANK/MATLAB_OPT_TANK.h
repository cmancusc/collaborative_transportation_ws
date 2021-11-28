//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  MATLAB_OPT_TANK.h
//
//  Code generation for function 'MATLAB_OPT_TANK'
//


#ifndef MATLAB_OPT_TANK_H
#define MATLAB_OPT_TANK_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class MATLAB_OPT_TANK
{
 public:
  MATLAB_OPT_TANK();
  ~MATLAB_OPT_TANK();
  void constraints(const double opt_qdot[8], const double A[6], const double
                   Jacob[48], const double a_max[8], double B, const double
                   opt_qdot_prev[8], double c[17]);
  double cost_function(const double opt_qdot[8], const double adm_qdot[8]);
  void matlab_opt_tank(const double adm_qdot[8], const double q_dot_0[8], const
                       double A[6], const double Jacob[48], const double a_max[8],
                       double B, const double opt_qdot_prev[8], double opt_qdot
                       [8], double *costfun_val, double *exit_flag);
};

#endif

// End of code generation (MATLAB_OPT_TANK.h)
