//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  MATLAB_OPT_TANK_CBF_internal_types.h
//
//  Code generation for function 'constraints_cbf'
//


#ifndef MATLAB_OPT_TANK_CBF_INTERNAL_TYPES_H
#define MATLAB_OPT_TANK_CBF_INTERNAL_TYPES_H

// Include files
#include "MATLAB_OPT_TANK_CBF_types.h"
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
  double workspace_double[1092];
  int workspace_int[39];
  int workspace_sort[39];
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
  double cIneq[19];
  double cIneq_old[19];
  double grad[28];
  double grad_old[28];
  int FunctionEvaluations;
  int sqpIterations;
  int sqpExitFlag;
  double lambdasqp[39];
  double lambdasqp_old[39];
  double steplength;
  double delta_x[28];
  double socDirection[28];
  double lambda_old[39];
  int workingset_old[39];
  double JacCineqTrans_old[532];
  double gradLag[28];
  double delta_gradLag[28];
  double xstar[28];
  double fstar;
  double firstorderopt;
  double lambda[39];
  int state;
  double maxConstr;
  int iterations;
  double searchDir[28];
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
  double Aineq[532];
  double bineq[19];
  double lb[28];
  double ub[28];
  int indexLB[28];
  int indexUB[28];
  int indexFixed[28];
  int mEqRemoved;
  double ATwset[1092];
  double bwset[39];
  int nActiveConstr;
  double maxConstrWorkspace[39];
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
  boolean_T isActiveConstr[39];
  int Wid[39];
  int Wlocalidx[39];
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
  double QR[1521];
  double Q[1521];
  int jpvt[39];
  int mrows;
  int ncols;
  double tau[39];
  int minRowCol;
  boolean_T usedPivoting;
};

struct h_struct_T
{
  double grad[28];
  double Hx[27];
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
  double FMat[1521];
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
  double cIneq_1[19];
  double f_2;
  double cIneq_2[19];
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

// End of code generation (MATLAB_OPT_TANK_CBF_internal_types.h)
