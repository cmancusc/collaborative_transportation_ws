//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  MATLAB_4_CBF.cpp
//
//  Code generation for function 'MATLAB_4_CBF'
//


// Include files
#include "MATLAB_4_CBF.h"
#include "MATLAB_4_CBF_internal_types.h"
#include "anonymous_function.h"
#include "computeConstraints_.h"
#include "computeForwardDifferences.h"
#include "computeObjective_.h"
#include "driver.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include <cstring>

// Function Definitions
MATLAB_4_CBF::MATLAB_4_CBF()
{
}

MATLAB_4_CBF::~MATLAB_4_CBF()
{
  // (no terminate code required)
}

void MATLAB_4_CBF::constraints_4_cbf(const double opt_qdot[8], const double A[6],
  double B, const double x[6], const double x_0[6], const double x_up[6], const
  double x_dx[6], double alpha_h1, double alpha_h2, double alpha_h2_up, double
  alpha_h2_dx, const double Jacob[48], const double Jacob_limit[48], const
  double a_max[8], const double opt_qdot_prev[8], double c[21])
{
  double c_tmp[8];
  double dv[6];
  double b_A;
  double c_A;
  double d;
  double d_A;
  double e_A;
  double f_A;
  int b_i;
  int i;

  //  nonlinear inequality equation <=0
  b_A = 0.0;
  for (i = 0; i < 8; i++) {
    d = opt_qdot[i];
    c_tmp[i] = d - opt_qdot_prev[i];
    d_A = 0.0;
    for (b_i = 0; b_i < 6; b_i++) {
      d_A += 2.0 * x[b_i] * Jacob_limit[b_i + 6 * i];
    }

    b_A += d_A * d;
  }

  for (b_i = 0; b_i < 6; b_i++) {
    dv[b_i] = -2.0 * (x[b_i] + x_0[b_i]);
  }

  c_A = 0.0;
  for (b_i = 0; b_i < 8; b_i++) {
    d = 0.0;
    for (i = 0; i < 6; i++) {
      d += dv[i] * Jacob_limit[i + 6 * b_i];
    }

    c_A += d * opt_qdot[b_i];
  }

  for (b_i = 0; b_i < 6; b_i++) {
    dv[b_i] = -2.0 * (x_up[b_i] + x_0[b_i]);
  }

  d_A = 0.0;
  for (b_i = 0; b_i < 8; b_i++) {
    d = 0.0;
    for (i = 0; i < 6; i++) {
      d += dv[i] * Jacob_limit[i + 6 * b_i];
    }

    d_A += d * opt_qdot[b_i];
  }

  for (b_i = 0; b_i < 6; b_i++) {
    dv[b_i] = -2.0 * (x_dx[b_i] + x_0[b_i]);
  }

  e_A = 0.0;
  f_A = 0.0;
  c[0] = -alpha_h1 - b_A;
  c[1] = -alpha_h2 - c_A;
  c[2] = -alpha_h2_up - d_A;
  for (b_i = 0; b_i < 8; b_i++) {
    d = 0.0;
    d_A = 0.0;
    for (i = 0; i < 6; i++) {
      int i1;
      i1 = i + 6 * b_i;
      d += dv[i] * Jacob_limit[i1];
      d_A += A[i] * Jacob[i1];
    }

    c_A = opt_qdot[b_i];
    e_A += d * c_A;
    f_A += d_A * c_A;
    d = c_tmp[b_i];
    d_A = a_max[b_i];
    c[b_i + 5] = d - d_A;
    c[b_i + 13] = -d_A - d;
  }

  c[3] = -alpha_h2_dx - e_A;
  c[4] = f_A - B;

  //  nonlinear equality constraints =0
}

double MATLAB_4_CBF::cost_function_4_cbf(const double opt_qdot[8], const double
  adm_qdot[8])
{
  double f;

  // opt_qdot = [0;0;0;0;0;0;0;0];
  //  cost function
  f = 0.0;
  for (int i = 0; i < 8; i++) {
    double d;
    d = opt_qdot[i] - adm_qdot[i];
    f += 0.5 * d * d;
  }

  return f;
}

void MATLAB_4_CBF::matlab_opt_4_cbf(const double adm_qdot[8], const double
  q_dot_0[8], const double A[6], double B, const double x[6], const double x_0[6],
  const double x_up[6], const double x_dx[6], double alpha_h1, double alpha_h2,
  double alpha_h2_up, double alpha_h2_dx, const double Jacob[48], const double
  Jacob_limit[48], const double a_max[8], const double opt_qdot_prev[8], double
  opt_qdot[8], double *costfun_val, double *exit_flag)
{
  static const signed char iv1[6] = { 1, 1, 1, 22, 22, 22 };

  static const signed char iv4[6] = { 1, 1, 1, 22, 23, 23 };

  static const signed char iv6[6] = { 1, 1, 1, 22, 43, 43 };

  static const signed char iv7[6] = { 1, 1, 1, 22, 44, 44 };

  static const signed char iv[5] = { 0, 0, 21, 0, 0 };

  static const signed char iv2[5] = { 0, 0, 21, 1, 0 };

  static const signed char iv3[5] = { 0, 0, 21, 21, 0 };

  static const signed char iv5[5] = { 0, 0, 21, 22, 0 };

  b_struct_T memspace;
  d_struct_T TrialState;
  e_struct_T WorkingSet;
  f_struct_T MeritFunction;
  g_struct_T s;
  h_struct_T QPObjective;
  i_struct_T CholManager;
  j_struct_T FcnEvaluator;
  k_struct_T FiniteDifferences;
  double Hessian[64];
  double unusedExpr[21];
  double b_TrialState[8];
  double normResid;
  int i;
  signed char b_i;

  // %%%%%%% OPTIMIZATION %%%%%%%
  std::memcpy(&FcnEvaluator.objfun.tunableEnvironment[0].f1[0], &adm_qdot[0], 8U
              * sizeof(double));
  FcnEvaluator.nonlcon.tunableEnvironment.f2 = B;
  FcnEvaluator.nonlcon.tunableEnvironment.f7 = alpha_h1;
  FcnEvaluator.nonlcon.tunableEnvironment.f8 = alpha_h2;
  FcnEvaluator.nonlcon.tunableEnvironment.f9 = alpha_h2_up;
  FcnEvaluator.nonlcon.tunableEnvironment.f10 = alpha_h2_dx;
  for (i = 0; i < 6; i++) {
    FcnEvaluator.nonlcon.tunableEnvironment.f1[i] = A[i];
    FcnEvaluator.nonlcon.tunableEnvironment.f3[i] = x[i];
    FcnEvaluator.nonlcon.tunableEnvironment.f4[i] = x_0[i];
    FcnEvaluator.nonlcon.tunableEnvironment.f5[i] = x_up[i];
    FcnEvaluator.nonlcon.tunableEnvironment.f6[i] = x_dx[i];
  }

  std::memcpy(&FcnEvaluator.nonlcon.tunableEnvironment.f11[0], &Jacob[0], 48U *
              sizeof(double));
  std::memcpy(&FcnEvaluator.nonlcon.tunableEnvironment.f12[0], &Jacob_limit[0],
              48U * sizeof(double));
  std::memcpy(&FcnEvaluator.nonlcon.tunableEnvironment.f13[0], &a_max[0], 8U *
              sizeof(double));
  std::memcpy(&FcnEvaluator.nonlcon.tunableEnvironment.f14[0], &opt_qdot_prev[0],
              8U * sizeof(double));
  this->constraints_4_cbf(q_dot_0, A, B, x, x_0, x_up, x_dx, alpha_h1, alpha_h2,
    alpha_h2_up, alpha_h2_dx, Jacob, Jacob_limit, a_max, opt_qdot_prev,
    unusedExpr);
  TrialState.nVarMax = 30;
  TrialState.mNonlinIneq = 21;
  TrialState.mNonlinEq = 0;
  TrialState.mIneq = 21;
  TrialState.mEq = 0;
  TrialState.iNonIneq0 = 1;
  TrialState.iNonEq0 = 1;
  TrialState.sqpFval_old = 0.0;
  std::memset(&TrialState.xstarsqp_old[0], 0, 8U * sizeof(double));
  std::memset(&TrialState.cIneq[0], 0, 21U * sizeof(double));
  std::memset(&TrialState.cIneq_old[0], 0, 21U * sizeof(double));
  std::memset(&TrialState.grad[0], 0, 30U * sizeof(double));
  std::memset(&TrialState.grad_old[0], 0, 30U * sizeof(double));
  TrialState.FunctionEvaluations = 0;
  TrialState.sqpIterations = 0;
  TrialState.sqpExitFlag = 0;
  std::memset(&TrialState.lambdasqp[0], 0, 43U * sizeof(double));
  std::memset(&TrialState.lambdasqp_old[0], 0, 43U * sizeof(double));
  TrialState.steplength = 1.0;
  std::memset(&TrialState.delta_x[0], 0, 30U * sizeof(double));
  std::memset(&TrialState.socDirection[0], 0, 30U * sizeof(double));
  std::memset(&TrialState.lambda_old[0], 0, 43U * sizeof(double));
  std::memset(&TrialState.workingset_old[0], 0, 43U * sizeof(int));
  std::memset(&TrialState.JacCineqTrans_old[0], 0, 630U * sizeof(double));
  std::memset(&TrialState.gradLag[0], 0, 30U * sizeof(double));
  std::memset(&TrialState.delta_gradLag[0], 0, 30U * sizeof(double));
  std::memset(&TrialState.xstar[0], 0, 30U * sizeof(double));
  TrialState.fstar = 0.0;
  TrialState.firstorderopt = 0.0;
  std::memset(&TrialState.lambda[0], 0, 43U * sizeof(double));
  TrialState.state = 0;
  TrialState.maxConstr = 0.0;
  TrialState.iterations = 0;
  std::memset(&TrialState.searchDir[0], 0, 30U * sizeof(double));
  FcnEvaluator.nVar = 8;
  FcnEvaluator.mCineq = 21;
  FcnEvaluator.mCeq = 0;
  FcnEvaluator.NonFiniteSupport = true;
  FcnEvaluator.SpecifyObjectiveGradient = false;
  FcnEvaluator.SpecifyConstraintGradient = false;
  FcnEvaluator.ScaleProblem = false;
  FiniteDifferences.objfun = FcnEvaluator.objfun;
  FiniteDifferences.nonlin = FcnEvaluator.nonlcon;
  FiniteDifferences.f_1 = 0.0;
  FiniteDifferences.f_2 = 0.0;
  FiniteDifferences.nVar = 8;
  FiniteDifferences.mIneq = 21;
  FiniteDifferences.mEq = 0;
  FiniteDifferences.numEvals = 0;
  FiniteDifferences.SpecifyObjectiveGradient = false;
  FiniteDifferences.SpecifyConstraintGradient = false;
  FiniteDifferences.FiniteDifferenceType = 0;
  for (i = 0; i < 8; i++) {
    TrialState.xstarsqp[i] = q_dot_0[i];
    FiniteDifferences.hasLB[i] = false;
    FiniteDifferences.hasUB[i] = false;
  }

  FiniteDifferences.hasBounds = false;
  WorkingSet.mConstr = 21;
  WorkingSet.mConstrOrig = 21;
  WorkingSet.mConstrMax = 43;
  WorkingSet.nVar = 8;
  WorkingSet.nVarOrig = 8;
  WorkingSet.nVarMax = 30;
  WorkingSet.ldA = 30;
  std::memset(&WorkingSet.bineq[0], 0, 21U * sizeof(double));
  std::memset(&WorkingSet.lb[0], 0, 30U * sizeof(double));
  std::memset(&WorkingSet.ub[0], 0, 30U * sizeof(double));
  std::memset(&WorkingSet.indexLB[0], 0, 30U * sizeof(int));
  std::memset(&WorkingSet.indexUB[0], 0, 30U * sizeof(int));
  std::memset(&WorkingSet.indexFixed[0], 0, 30U * sizeof(int));
  WorkingSet.mEqRemoved = 0;
  std::memset(&WorkingSet.ATwset[0], 0, 1290U * sizeof(double));
  WorkingSet.nActiveConstr = 0;
  std::memset(&WorkingSet.bwset[0], 0, 43U * sizeof(double));
  std::memset(&WorkingSet.maxConstrWorkspace[0], 0, 43U * sizeof(double));
  for (i = 0; i < 5; i++) {
    b_i = iv[i];
    WorkingSet.sizes[i] = b_i;
    WorkingSet.sizesNormal[i] = b_i;
    WorkingSet.sizesPhaseOne[i] = iv2[i];
    WorkingSet.sizesRegularized[i] = iv3[i];
    WorkingSet.sizesRegPhaseOne[i] = iv5[i];
  }

  for (i = 0; i < 6; i++) {
    b_i = iv1[i];
    WorkingSet.isActiveIdx[i] = b_i;
    WorkingSet.isActiveIdxNormal[i] = b_i;
    WorkingSet.isActiveIdxPhaseOne[i] = iv4[i];
    WorkingSet.isActiveIdxRegularized[i] = iv6[i];
    WorkingSet.isActiveIdxRegPhaseOne[i] = iv7[i];
  }

  std::memset(&WorkingSet.Wid[0], 0, 43U * sizeof(int));
  std::memset(&WorkingSet.Wlocalidx[0], 0, 43U * sizeof(int));
  for (i = 0; i < 43; i++) {
    WorkingSet.isActiveConstr[i] = false;
  }

  for (i = 0; i < 5; i++) {
    WorkingSet.nWConstr[i] = 0;
  }

  WorkingSet.probType = 3;
  WorkingSet.SLACK0 = 1.0E-5;
  std::memcpy(&b_TrialState[0], &TrialState.xstarsqp[0], 8U * sizeof(double));
  coder::optim::coder::utils::ObjNonlinEvaluator::computeObjective_(this,
    FcnEvaluator.objfun, b_TrialState, &TrialState.sqpFval, &i);
  if (i == 1) {
    coder::optim::coder::utils::ObjNonlinEvaluator::computeConstraints_(this,
      &FcnEvaluator.nonlcon, TrialState.xstarsqp, TrialState.cIneq);
  }

  std::memset(&WorkingSet.Aineq[0], 0, 630U * sizeof(double));
  coder::optim::coder::utils::FiniteDifferences::internal::
    computeForwardDifferences(this, &FiniteDifferences, TrialState.sqpFval,
    TrialState.cIneq, TrialState.xstarsqp, TrialState.grad, WorkingSet.Aineq);
  TrialState.FunctionEvaluations = FiniteDifferences.numEvals + 1;
  for (i = 0; i < 21; i++) {
    WorkingSet.bineq[i] = -TrialState.cIneq[i];
  }

  coder::optim::coder::qpactiveset::WorkingSet::setProblemType(&WorkingSet, 3);
  for (i = 0; i < 43; i++) {
    WorkingSet.isActiveConstr[i] = false;
  }

  WorkingSet.nWConstr[0] = 0;
  WorkingSet.nWConstr[1] = 0;
  WorkingSet.nWConstr[2] = 0;
  WorkingSet.nWConstr[3] = 0;
  WorkingSet.nWConstr[4] = 0;
  WorkingSet.nActiveConstr = 0;
  MeritFunction.initFval = TrialState.sqpFval;
  MeritFunction.penaltyParam = 1.0;
  MeritFunction.threshold = 0.0001;
  MeritFunction.nPenaltyDecreases = 0;
  MeritFunction.linearizedConstrViol = 0.0;
  MeritFunction.initConstrViolationEq = 0.0;
  normResid = 0.0;
  for (i = 0; i < 21; i++) {
    double d;
    d = TrialState.cIneq[i];
    if (d > 0.0) {
      normResid += d;
    }
  }

  MeritFunction.initConstrViolationIneq = normResid;
  MeritFunction.phi = 0.0;
  MeritFunction.phiPrimePlus = 0.0;
  MeritFunction.phiFullStep = 0.0;
  MeritFunction.feasRelativeFactor = 0.0;
  MeritFunction.nlpPrimalFeasError = 0.0;
  MeritFunction.nlpDualFeasError = 0.0;
  MeritFunction.nlpComplError = 0.0;
  MeritFunction.firstOrderOpt = 0.0;
  MeritFunction.hasObjective = true;
  s.ldq = 43;
  std::memset(&s.QR[0], 0, 1849U * sizeof(double));
  std::memset(&s.Q[0], 0, 1849U * sizeof(double));
  s.mrows = 0;
  s.ncols = 0;
  std::memset(&s.jpvt[0], 0, 43U * sizeof(int));
  std::memset(&s.tau[0], 0, 43U * sizeof(double));
  s.minRowCol = 0;
  s.usedPivoting = false;
  std::memset(&QPObjective.grad[0], 0, 30U * sizeof(double));
  std::memset(&QPObjective.Hx[0], 0, 29U * sizeof(double));
  QPObjective.hasLinear = true;
  QPObjective.nvar = 8;
  QPObjective.maxVar = 30;
  QPObjective.beta = 0.0;
  QPObjective.rho = 0.0;
  QPObjective.objtype = 3;
  QPObjective.prev_objtype = 3;
  QPObjective.prev_nvar = 0;
  QPObjective.prev_hasLinear = false;
  QPObjective.gammaScalar = 0.0;
  coder::optim::coder::fminconsqp::driver(this, &TrialState, &MeritFunction,
    &FcnEvaluator, &FiniteDifferences, &memspace, &WorkingSet, &s, &QPObjective,
    Hessian, &CholManager);
  std::memcpy(&opt_qdot[0], &TrialState.xstarsqp[0], 8U * sizeof(double));
  *costfun_val = TrialState.sqpFval;
  *exit_flag = TrialState.sqpExitFlag;
}

// End of code generation (MATLAB_4_CBF.cpp)
