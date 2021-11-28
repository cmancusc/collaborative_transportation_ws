/* Produced by CVXGEN, 2021-05-13 12:21:53 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-1)-rhs[6]*(params.P[0]*params.Jacob_limit[0])-rhs[7]*(params.P[0]*params.Jacob_limit[6])-rhs[8]*(params.P[0]*params.Jacob_limit[12])-rhs[9]*(params.P[0]*params.Jacob_limit[18])-rhs[10]*(params.P[0]*params.Jacob_limit[24])-rhs[11]*(params.P[0]*params.Jacob_limit[30])-rhs[12]*(params.P[0]*params.Jacob_limit[36])-rhs[13]*(params.P[0]*params.Jacob_limit[42]);
  lhs[1] = -rhs[1]*(-1)-rhs[6]*(params.P[1]*params.Jacob_limit[1])-rhs[7]*(params.P[1]*params.Jacob_limit[7])-rhs[8]*(params.P[1]*params.Jacob_limit[13])-rhs[9]*(params.P[1]*params.Jacob_limit[19])-rhs[10]*(params.P[1]*params.Jacob_limit[25])-rhs[11]*(params.P[1]*params.Jacob_limit[31])-rhs[12]*(params.P[1]*params.Jacob_limit[37])-rhs[13]*(params.P[1]*params.Jacob_limit[43]);
  lhs[2] = -rhs[2]*(-1)-rhs[6]*(params.P[2]*params.Jacob_limit[2])-rhs[7]*(params.P[2]*params.Jacob_limit[8])-rhs[8]*(params.P[2]*params.Jacob_limit[14])-rhs[9]*(params.P[2]*params.Jacob_limit[20])-rhs[10]*(params.P[2]*params.Jacob_limit[26])-rhs[11]*(params.P[2]*params.Jacob_limit[32])-rhs[12]*(params.P[2]*params.Jacob_limit[38])-rhs[13]*(params.P[2]*params.Jacob_limit[44]);
  lhs[3] = -rhs[3]*(-1)-rhs[6]*(params.P[3]*params.Jacob_limit[3])-rhs[7]*(params.P[3]*params.Jacob_limit[9])-rhs[8]*(params.P[3]*params.Jacob_limit[15])-rhs[9]*(params.P[3]*params.Jacob_limit[21])-rhs[10]*(params.P[3]*params.Jacob_limit[27])-rhs[11]*(params.P[3]*params.Jacob_limit[33])-rhs[12]*(params.P[3]*params.Jacob_limit[39])-rhs[13]*(params.P[3]*params.Jacob_limit[45]);
  lhs[4] = -rhs[4]*(-1)-rhs[6]*(params.P[4]*params.Jacob_limit[4])-rhs[7]*(params.P[4]*params.Jacob_limit[10])-rhs[8]*(params.P[4]*params.Jacob_limit[16])-rhs[9]*(params.P[4]*params.Jacob_limit[22])-rhs[10]*(params.P[4]*params.Jacob_limit[28])-rhs[11]*(params.P[4]*params.Jacob_limit[34])-rhs[12]*(params.P[4]*params.Jacob_limit[40])-rhs[13]*(params.P[4]*params.Jacob_limit[46]);
  lhs[5] = -rhs[5]*(-1)-rhs[6]*(params.P[5]*params.Jacob_limit[5])-rhs[7]*(params.P[5]*params.Jacob_limit[11])-rhs[8]*(params.P[5]*params.Jacob_limit[17])-rhs[9]*(params.P[5]*params.Jacob_limit[23])-rhs[10]*(params.P[5]*params.Jacob_limit[29])-rhs[11]*(params.P[5]*params.Jacob_limit[35])-rhs[12]*(params.P[5]*params.Jacob_limit[41])-rhs[13]*(params.P[5]*params.Jacob_limit[47]);
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-1);
  lhs[1] = -rhs[1]*(-1);
  lhs[2] = -rhs[2]*(-1);
  lhs[3] = -rhs[3]*(-1);
  lhs[4] = -rhs[4]*(-1);
  lhs[5] = -rhs[5]*(-1);
  lhs[6] = -rhs[0]*(params.P[0]*params.Jacob_limit[0])-rhs[1]*(params.P[1]*params.Jacob_limit[1])-rhs[2]*(params.P[2]*params.Jacob_limit[2])-rhs[3]*(params.P[3]*params.Jacob_limit[3])-rhs[4]*(params.P[4]*params.Jacob_limit[4])-rhs[5]*(params.P[5]*params.Jacob_limit[5]);
  lhs[7] = -rhs[0]*(params.P[0]*params.Jacob_limit[6])-rhs[1]*(params.P[1]*params.Jacob_limit[7])-rhs[2]*(params.P[2]*params.Jacob_limit[8])-rhs[3]*(params.P[3]*params.Jacob_limit[9])-rhs[4]*(params.P[4]*params.Jacob_limit[10])-rhs[5]*(params.P[5]*params.Jacob_limit[11]);
  lhs[8] = -rhs[0]*(params.P[0]*params.Jacob_limit[12])-rhs[1]*(params.P[1]*params.Jacob_limit[13])-rhs[2]*(params.P[2]*params.Jacob_limit[14])-rhs[3]*(params.P[3]*params.Jacob_limit[15])-rhs[4]*(params.P[4]*params.Jacob_limit[16])-rhs[5]*(params.P[5]*params.Jacob_limit[17]);
  lhs[9] = -rhs[0]*(params.P[0]*params.Jacob_limit[18])-rhs[1]*(params.P[1]*params.Jacob_limit[19])-rhs[2]*(params.P[2]*params.Jacob_limit[20])-rhs[3]*(params.P[3]*params.Jacob_limit[21])-rhs[4]*(params.P[4]*params.Jacob_limit[22])-rhs[5]*(params.P[5]*params.Jacob_limit[23]);
  lhs[10] = -rhs[0]*(params.P[0]*params.Jacob_limit[24])-rhs[1]*(params.P[1]*params.Jacob_limit[25])-rhs[2]*(params.P[2]*params.Jacob_limit[26])-rhs[3]*(params.P[3]*params.Jacob_limit[27])-rhs[4]*(params.P[4]*params.Jacob_limit[28])-rhs[5]*(params.P[5]*params.Jacob_limit[29]);
  lhs[11] = -rhs[0]*(params.P[0]*params.Jacob_limit[30])-rhs[1]*(params.P[1]*params.Jacob_limit[31])-rhs[2]*(params.P[2]*params.Jacob_limit[32])-rhs[3]*(params.P[3]*params.Jacob_limit[33])-rhs[4]*(params.P[4]*params.Jacob_limit[34])-rhs[5]*(params.P[5]*params.Jacob_limit[35]);
  lhs[12] = -rhs[0]*(params.P[0]*params.Jacob_limit[36])-rhs[1]*(params.P[1]*params.Jacob_limit[37])-rhs[2]*(params.P[2]*params.Jacob_limit[38])-rhs[3]*(params.P[3]*params.Jacob_limit[39])-rhs[4]*(params.P[4]*params.Jacob_limit[40])-rhs[5]*(params.P[5]*params.Jacob_limit[41]);
  lhs[13] = -rhs[0]*(params.P[0]*params.Jacob_limit[42])-rhs[1]*(params.P[1]*params.Jacob_limit[43])-rhs[2]*(params.P[2]*params.Jacob_limit[44])-rhs[3]*(params.P[3]*params.Jacob_limit[45])-rhs[4]*(params.P[4]*params.Jacob_limit[46])-rhs[5]*(params.P[5]*params.Jacob_limit[47]);
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[6]*(params.A[0]*params.Jacob[0]+params.A[1]*params.Jacob[1]+params.A[2]*params.Jacob[2]+params.A[3]*params.Jacob[3]+params.A[4]*params.Jacob[4]+params.A[5]*params.Jacob[5])-rhs[7]*(params.A[0]*params.Jacob[6]+params.A[1]*params.Jacob[7]+params.A[2]*params.Jacob[8]+params.A[3]*params.Jacob[9]+params.A[4]*params.Jacob[10]+params.A[5]*params.Jacob[11])-rhs[8]*(params.A[0]*params.Jacob[12]+params.A[1]*params.Jacob[13]+params.A[2]*params.Jacob[14]+params.A[3]*params.Jacob[15]+params.A[4]*params.Jacob[16]+params.A[5]*params.Jacob[17])-rhs[9]*(params.A[0]*params.Jacob[18]+params.A[1]*params.Jacob[19]+params.A[2]*params.Jacob[20]+params.A[3]*params.Jacob[21]+params.A[4]*params.Jacob[22]+params.A[5]*params.Jacob[23])-rhs[10]*(params.A[0]*params.Jacob[24]+params.A[1]*params.Jacob[25]+params.A[2]*params.Jacob[26]+params.A[3]*params.Jacob[27]+params.A[4]*params.Jacob[28]+params.A[5]*params.Jacob[29])-rhs[11]*(params.A[0]*params.Jacob[30]+params.A[1]*params.Jacob[31]+params.A[2]*params.Jacob[32]+params.A[3]*params.Jacob[33]+params.A[4]*params.Jacob[34]+params.A[5]*params.Jacob[35])-rhs[12]*(params.A[0]*params.Jacob[36]+params.A[1]*params.Jacob[37]+params.A[2]*params.Jacob[38]+params.A[3]*params.Jacob[39]+params.A[4]*params.Jacob[40]+params.A[5]*params.Jacob[41])-rhs[13]*(params.A[0]*params.Jacob[42]+params.A[1]*params.Jacob[43]+params.A[2]*params.Jacob[44]+params.A[3]*params.Jacob[45]+params.A[4]*params.Jacob[46]+params.A[5]*params.Jacob[47]);
  lhs[1] = -rhs[6]*(-1);
  lhs[2] = -rhs[7]*(-1);
  lhs[3] = -rhs[8]*(-1);
  lhs[4] = -rhs[9]*(-1);
  lhs[5] = -rhs[10]*(-1);
  lhs[6] = -rhs[11]*(-1);
  lhs[7] = -rhs[12]*(-1);
  lhs[8] = -rhs[13]*(-1);
  lhs[9] = -rhs[6]*(1);
  lhs[10] = -rhs[7]*(1);
  lhs[11] = -rhs[8]*(1);
  lhs[12] = -rhs[9]*(1);
  lhs[13] = -rhs[10]*(1);
  lhs[14] = -rhs[11]*(1);
  lhs[15] = -rhs[12]*(1);
  lhs[16] = -rhs[13]*(1);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
  lhs[6] = -rhs[0]*(params.A[0]*params.Jacob[0]+params.A[1]*params.Jacob[1]+params.A[2]*params.Jacob[2]+params.A[3]*params.Jacob[3]+params.A[4]*params.Jacob[4]+params.A[5]*params.Jacob[5])-rhs[1]*(-1)-rhs[9]*(1);
  lhs[7] = -rhs[0]*(params.A[0]*params.Jacob[6]+params.A[1]*params.Jacob[7]+params.A[2]*params.Jacob[8]+params.A[3]*params.Jacob[9]+params.A[4]*params.Jacob[10]+params.A[5]*params.Jacob[11])-rhs[2]*(-1)-rhs[10]*(1);
  lhs[8] = -rhs[0]*(params.A[0]*params.Jacob[12]+params.A[1]*params.Jacob[13]+params.A[2]*params.Jacob[14]+params.A[3]*params.Jacob[15]+params.A[4]*params.Jacob[16]+params.A[5]*params.Jacob[17])-rhs[3]*(-1)-rhs[11]*(1);
  lhs[9] = -rhs[0]*(params.A[0]*params.Jacob[18]+params.A[1]*params.Jacob[19]+params.A[2]*params.Jacob[20]+params.A[3]*params.Jacob[21]+params.A[4]*params.Jacob[22]+params.A[5]*params.Jacob[23])-rhs[4]*(-1)-rhs[12]*(1);
  lhs[10] = -rhs[0]*(params.A[0]*params.Jacob[24]+params.A[1]*params.Jacob[25]+params.A[2]*params.Jacob[26]+params.A[3]*params.Jacob[27]+params.A[4]*params.Jacob[28]+params.A[5]*params.Jacob[29])-rhs[5]*(-1)-rhs[13]*(1);
  lhs[11] = -rhs[0]*(params.A[0]*params.Jacob[30]+params.A[1]*params.Jacob[31]+params.A[2]*params.Jacob[32]+params.A[3]*params.Jacob[33]+params.A[4]*params.Jacob[34]+params.A[5]*params.Jacob[35])-rhs[6]*(-1)-rhs[14]*(1);
  lhs[12] = -rhs[0]*(params.A[0]*params.Jacob[36]+params.A[1]*params.Jacob[37]+params.A[2]*params.Jacob[38]+params.A[3]*params.Jacob[39]+params.A[4]*params.Jacob[40]+params.A[5]*params.Jacob[41])-rhs[7]*(-1)-rhs[15]*(1);
  lhs[13] = -rhs[0]*(params.A[0]*params.Jacob[42]+params.A[1]*params.Jacob[43]+params.A[2]*params.Jacob[44]+params.A[3]*params.Jacob[45]+params.A[4]*params.Jacob[46]+params.A[5]*params.Jacob[47])-rhs[8]*(-1)-rhs[16]*(1);
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.l[0]);
  lhs[1] = rhs[1]*(2*params.l[0]);
  lhs[2] = rhs[2]*(2*params.l[0]);
  lhs[3] = rhs[3]*(2*params.l[0]);
  lhs[4] = rhs[4]*(2*params.l[0]);
  lhs[5] = rhs[5]*(2*params.l[0]);
  lhs[6] = rhs[6]*(2);
  lhs[7] = rhs[7]*(2);
  lhs[8] = rhs[8]*(2);
  lhs[9] = rhs[9]*(2);
  lhs[10] = rhs[10]*(2);
  lhs[11] = rhs[11]*(2);
  lhs[12] = rhs[12]*(2);
  lhs[13] = rhs[13]*(2);
}
void fillq(void) {
  work.q[0] = 0;
  work.q[1] = 0;
  work.q[2] = 0;
  work.q[3] = 0;
  work.q[4] = 0;
  work.q[5] = 0;
  work.q[6] = -2*params.adm_qdot[0];
  work.q[7] = -2*params.adm_qdot[1];
  work.q[8] = -2*params.adm_qdot[2];
  work.q[9] = -2*params.adm_qdot[3];
  work.q[10] = -2*params.adm_qdot[4];
  work.q[11] = -2*params.adm_qdot[5];
  work.q[12] = -2*params.adm_qdot[6];
  work.q[13] = -2*params.adm_qdot[7];
}
void fillh(void) {
  work.h[0] = params.B[0];
  work.h[1] = -(-params.a_max[0]+params.opt_qdot_prev[0]);
  work.h[2] = -(-params.a_max[1]+params.opt_qdot_prev[1]);
  work.h[3] = -(-params.a_max[2]+params.opt_qdot_prev[2]);
  work.h[4] = -(-params.a_max[3]+params.opt_qdot_prev[3]);
  work.h[5] = -(-params.a_max[4]+params.opt_qdot_prev[4]);
  work.h[6] = -(-params.a_max[5]+params.opt_qdot_prev[5]);
  work.h[7] = -(-params.a_max[6]+params.opt_qdot_prev[6]);
  work.h[8] = -(-params.a_max[7]+params.opt_qdot_prev[7]);
  work.h[9] = -(-params.opt_qdot_prev[0]-params.a_max[0]);
  work.h[10] = -(-params.opt_qdot_prev[1]-params.a_max[1]);
  work.h[11] = -(-params.opt_qdot_prev[2]-params.a_max[2]);
  work.h[12] = -(-params.opt_qdot_prev[3]-params.a_max[3]);
  work.h[13] = -(-params.opt_qdot_prev[4]-params.a_max[4]);
  work.h[14] = -(-params.opt_qdot_prev[5]-params.a_max[5]);
  work.h[15] = -(-params.opt_qdot_prev[6]-params.a_max[6]);
  work.h[16] = -(-params.opt_qdot_prev[7]-params.a_max[7]);
}
void fillb(void) {
  work.b[0] = 0;
  work.b[1] = 0;
  work.b[2] = 0;
  work.b[3] = 0;
  work.b[4] = 0;
  work.b[5] = 0;
}
void pre_ops(void) {
  work.quad_955409002496[0] = params.adm_qdot[0]*params.adm_qdot[0]+params.adm_qdot[1]*params.adm_qdot[1]+params.adm_qdot[2]*params.adm_qdot[2]+params.adm_qdot[3]*params.adm_qdot[3]+params.adm_qdot[4]*params.adm_qdot[4]+params.adm_qdot[5]*params.adm_qdot[5]+params.adm_qdot[6]*params.adm_qdot[6]+params.adm_qdot[7]*params.adm_qdot[7];
}
