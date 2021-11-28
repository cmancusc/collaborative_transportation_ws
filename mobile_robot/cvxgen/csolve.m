% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(opt_qdot - adm_qdot, eye(8)) + l*quad_form(delta, eye(6)))
%   subject to
%     P*Jacob_limit*opt_qdot == delta
%     A'*Jacob*opt_qdot <= B
%     -a_max <= opt_qdot - opt_qdot_prev
%     opt_qdot - opt_qdot_prev <= a_max
%
% with variables
%    delta   6 x 1
% opt_qdot   8 x 1
%
% and parameters
%        A   6 x 1
%        B   1 x 1
%    Jacob   6 x 8
% Jacob_limit   6 x 8
%        P   6 x 6    diagonal
%    a_max   8 x 1
% adm_qdot   8 x 1
%        l   1 x 1    positive
% opt_qdot_prev   8 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.A, ..., params.opt_qdot_prev, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2021-05-13 12:21:53 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
