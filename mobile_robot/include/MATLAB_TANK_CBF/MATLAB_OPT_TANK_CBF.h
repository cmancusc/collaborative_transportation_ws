//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  MATLAB_OPT_TANK_CBF.h
//
//  Code generation for function 'MATLAB_OPT_TANK_CBF'
//


#ifndef MATLAB_OPT_TANK_CBF_H
#define MATLAB_OPT_TANK_CBF_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class MATLAB_OPT_TANK_CBF
{
 public:
  MATLAB_OPT_TANK_CBF();
  ~MATLAB_OPT_TANK_CBF();
  void constraints_cbf(const double opt_qdot[8], const double A[6], double B,
                       const double x[6], const double x_0[6], double alpha_h1,
                       double alpha_h2, const double Jacob[48], const double
                       Jacob_limit[48], const double a_max[8], const double
                       opt_qdot_prev[8], double c[19]);
  double cost_function_cbf(const double opt_qdot[8], const double adm_qdot[8]);
  void matlab_opt_tank_cbf(const double adm_qdot[8], const double q_dot_0[8],
    const double A[6], double B, const double x[6], const double x_0[6], double
    alpha_h1, double alpha_h2, const double Jacob[48], const double Jacob_limit
    [48], const double a_max[8], const double opt_qdot_prev[8], double opt_qdot
    [8], double *costfun_val, double *exit_flag);
};

#endif

// End of code generation (MATLAB_OPT_TANK_CBF.h)
