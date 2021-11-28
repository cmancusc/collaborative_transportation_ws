//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  MATLAB_OPT_TANK_internal_types.h
//
//  Code generation for function 'constraints'
//


#ifndef MATLAB_OPT_TANK_INTERNAL_TYPES_H
#define MATLAB_OPT_TANK_INTERNAL_TYPES_H

// Include files
#include "MATLAB_OPT_TANK_types.h"
#include "anonymous_function.h"
#include "rtwtypes.h"

// Type Definitions
struct struct_T
{
  boolean_T gradOK;
  boolean_T fevalOK;
  boolean_T done;
  boolean_T stepAccepted;
  boolean_T failedLineSearch;
  int stepType;
};

struct b_struct_T
{
  double workspace_double[910];
  int workspace_int[35];
  int workspace_sort[35];
};

struct c_struct_T
{
  char SolverName[7];
  int MaxIterations;
  double StepTolerance;
  double OptimalityTolerance;
  double ConstraintTolerance;
  double ObjectiveLimit;
  double PricingTolerance;
  double ConstrRelTolFactor;
  double ProbRelTolFactor;
  boolean_T RemainFeasible;
  boolean_T IterDisplayQP;
};

struct d_struct_T
{
  int nVarMax;
  int mNonlinIneq;
  int mNonlinEq;
  int mIneq;
  int mEq;
  int iNonIneq0;
  int iNonEq0;
  double sqpFval;
  double sqpFval_old;
  double xstarsqp[8];
  double xstarsqp_old[8];
  double cIneq[17];
  double cIneq_old[17];
  double grad[26];
  double grad_old[26];
  int FunctionEvaluations;
  int sqpIterations;
  int sqpExitFlag;
  double lambdasqp[35];
  double lambdasqp_old[35];
  double steplength;
  double delta_x[26];
  double socDirection[26];
  double lambda_old[35];
  int workingset_old[35];
  double JacCineqTrans_old[442];
  double gradLag[26];
  double delta_gradLag[26];
  double xstar[26];
  double fstar;
  double firstorderopt;
  double lambda[35];
  int state;
  double maxConstr;
  int iterations;
  double searchDir[26];
};

struct e_struct_T
{
  int mConstr;
  int mConstrOrig;
  int mConstrMax;
  int nVar;
  int nVarOrig;
  int nVarMax;
  int ldA;
  double Aineq[442];
  double bineq[17];
  double lb[26];
  double ub[26];
  int indexLB[26];
  int indexUB[26];
  int indexFixed[26];
  int mEqRemoved;
  double ATwset[910];
  double bwset[35];
  int nActiveConstr;
  double maxConstrWorkspace[35];
  int sizes[5];
  int sizesNormal[5];
  int sizesPhaseOne[5];
  int sizesRegularized[5];
  int sizesRegPhaseOne[5];
  int isActiveIdx[6];
  int isActiveIdxNormal[6];
  int isActiveIdxPhaseOne[6];
  int isActiveIdxRegularized[6];
  int isActiveIdxRegPhaseOne[6];
  boolean_T isActiveConstr[35];
  int Wid[35];
  int Wlocalidx[35];
  int nWConstr[5];
  int probType;
  double SLACK0;
};

struct f_struct_T
{
  double penaltyParam;
  double threshold;
  int nPenaltyDecreases;
  double linearizedConstrViol;
  double initFval;
  double initConstrViolationEq;
  double initConstrViolationIneq;
  double phi;
  double phiPrimePlus;
  double phiFullStep;
  double feasRelativeFactor;
  double nlpPrimalFeasError;
  double nlpDualFeasError;
  double nlpComplError;
  double firstOrderOpt;
  boolean_T hasObjective;
};

struct g_struct_T
{
  int ldq;
  double QR[1225];
  double Q[1225];
  int jpvt[35];
  int mrows;
  int ncols;
  double tau[35];
  int minRowCol;
  boolean_T usedPivoting;
};

struct h_struct_T
{
  double grad[26];
  double Hx[25];
  boolean_T hasLinear;
  int nvar;
  int maxVar;
  double beta;
  double rho;
  int objtype;
  int prev_objtype;
  int prev_nvar;
  boolean_T prev_hasLinear;
  double gammaScalar;
};

struct i_struct_T
{
  double FMat[1225];
  int ldm;
  int ndims;
  int info;
};

struct j_struct_T
{
  coder::anonymous_function objfun;
  coder::b_anonymous_function nonlcon;
  int nVar;
  int mCineq;
  int mCeq;
  boolean_T NonFiniteSupport;
  boolean_T SpecifyObjectiveGradient;
  boolean_T SpecifyConstraintGradient;
  boolean_T ScaleProblem;
};

struct k_struct_T
{
  coder::anonymous_function objfun;
  coder::b_anonymous_function nonlin;
  double f_1;
  double cIneq_1[17];
  double f_2;
  double cIneq_2[17];
  int nVar;
  int mIneq;
  int mEq;
  int numEvals;
  boolean_T SpecifyObjectiveGradient;
  boolean_T SpecifyConstraintGradient;
  boolean_T hasLB[8];
  boolean_T hasUB[8];
  boolean_T hasBounds;
  int FiniteDifferenceType;
};

#endif

// End of code generation (MATLAB_OPT_TANK_internal_types.h)
