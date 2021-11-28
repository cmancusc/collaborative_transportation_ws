/* Produced by CVXGEN, 2021-05-13 12:21:53 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.adm_qdot[0] = 0.20319161029830202;
  params.adm_qdot[1] = 0.8325912904724193;
  params.adm_qdot[2] = -0.8363810443482227;
  params.adm_qdot[3] = 0.04331042079065206;
  params.adm_qdot[4] = 1.5717878173906188;
  params.adm_qdot[5] = 1.5851723557337523;
  params.adm_qdot[6] = -1.497658758144655;
  params.adm_qdot[7] = -1.171028487447253;
  params.l[0] = 0.10293440660165976;
  params.P[0] = -0.23676062539745413;
  params.P[1] = -1.8804951564857322;
  params.P[2] = -0.17266710242115568;
  params.P[3] = 0.596576190459043;
  params.P[4] = -0.8860508694080989;
  params.P[5] = 0.7050196079205251;
  params.Jacob_limit[0] = 0.3634512696654033;
  params.Jacob_limit[1] = -1.9040724704913385;
  params.Jacob_limit[2] = 0.23541635196352795;
  params.Jacob_limit[3] = -0.9629902123701384;
  params.Jacob_limit[4] = -0.3395952119597214;
  params.Jacob_limit[5] = -0.865899672914725;
  params.Jacob_limit[6] = 0.7725516732519853;
  params.Jacob_limit[7] = -0.23818512931704205;
  params.Jacob_limit[8] = -1.372529046100147;
  params.Jacob_limit[9] = 0.17859607212737894;
  params.Jacob_limit[10] = 1.1212590580454682;
  params.Jacob_limit[11] = -0.774545870495281;
  params.Jacob_limit[12] = -1.1121684642712744;
  params.Jacob_limit[13] = -0.44811496977740495;
  params.Jacob_limit[14] = 1.7455345994417217;
  params.Jacob_limit[15] = 1.9039816898917352;
  params.Jacob_limit[16] = 0.6895347036512547;
  params.Jacob_limit[17] = 1.6113364341535923;
  params.Jacob_limit[18] = 1.383003485172717;
  params.Jacob_limit[19] = -0.48802383468444344;
  params.Jacob_limit[20] = -1.631131964513103;
  params.Jacob_limit[21] = 0.6136436100941447;
  params.Jacob_limit[22] = 0.2313630495538037;
  params.Jacob_limit[23] = -0.5537409477496875;
  params.Jacob_limit[24] = -1.0997819806406723;
  params.Jacob_limit[25] = -0.3739203344950055;
  params.Jacob_limit[26] = -0.12423900520332376;
  params.Jacob_limit[27] = -0.923057686995755;
  params.Jacob_limit[28] = -0.8328289030982696;
  params.Jacob_limit[29] = -0.16925440270808823;
  params.Jacob_limit[30] = 1.442135651787706;
  params.Jacob_limit[31] = 0.34501161787128565;
  params.Jacob_limit[32] = -0.8660485502711608;
  params.Jacob_limit[33] = -0.8880899735055947;
  params.Jacob_limit[34] = -0.1815116979122129;
  params.Jacob_limit[35] = -1.17835862158005;
  params.Jacob_limit[36] = -1.1944851558277074;
  params.Jacob_limit[37] = 0.05614023926976763;
  params.Jacob_limit[38] = -1.6510825248767813;
  params.Jacob_limit[39] = -0.06565787059365391;
  params.Jacob_limit[40] = -0.5512951504486665;
  params.Jacob_limit[41] = 0.8307464872626844;
  params.Jacob_limit[42] = 0.9869848924080182;
  params.Jacob_limit[43] = 0.7643716874230573;
  params.Jacob_limit[44] = 0.7567216550196565;
  params.Jacob_limit[45] = -0.5055995034042868;
  params.Jacob_limit[46] = 0.6725392189410702;
  params.Jacob_limit[47] = -0.6406053441727284;
  params.A[0] = 0.29117547947550015;
  params.A[1] = -0.6967713677405021;
  params.A[2] = -0.21941980294587182;
  params.A[3] = -1.753884276680243;
  params.A[4] = -1.0292983112626475;
  params.A[5] = 1.8864104246942706;
  params.Jacob[0] = -1.077663182579704;
  params.Jacob[1] = 0.7659100437893209;
  params.Jacob[2] = 0.6019074328549583;
  params.Jacob[3] = 0.8957565577499285;
  params.Jacob[4] = -0.09964555746227477;
  params.Jacob[5] = 0.38665509840745127;
  params.Jacob[6] = -1.7321223042686946;
  params.Jacob[7] = -1.7097514487110663;
  params.Jacob[8] = -1.2040958948116867;
  params.Jacob[9] = -1.3925560119658358;
  params.Jacob[10] = -1.5995826216742213;
  params.Jacob[11] = -1.4828245415645833;
  params.Jacob[12] = 0.21311092723061398;
  params.Jacob[13] = -1.248740700304487;
  params.Jacob[14] = 1.808404972124833;
  params.Jacob[15] = 0.7264471152297065;
  params.Jacob[16] = 0.16407869343908477;
  params.Jacob[17] = 0.8287224032315907;
  params.Jacob[18] = -0.9444533161899464;
  params.Jacob[19] = 1.7069027370149112;
  params.Jacob[20] = 1.3567722311998827;
  params.Jacob[21] = 0.9052779937121489;
  params.Jacob[22] = -0.07904017565835986;
  params.Jacob[23] = 1.3684127435065871;
  params.Jacob[24] = 0.979009293697437;
  params.Jacob[25] = 0.6413036255984501;
  params.Jacob[26] = 1.6559010680237511;
  params.Jacob[27] = 0.5346622551502991;
  params.Jacob[28] = -0.5362376605895625;
  params.Jacob[29] = 0.2113782926017822;
  params.Jacob[30] = -1.2144776931994525;
  params.Jacob[31] = -1.2317108144255875;
  params.Jacob[32] = 0.9026784957312834;
  params.Jacob[33] = 1.1397468137245244;
  params.Jacob[34] = 1.8883934547350631;
  params.Jacob[35] = 1.4038856681660068;
  params.Jacob[36] = 0.17437730638329096;
  params.Jacob[37] = -1.6408365219077408;
  params.Jacob[38] = -0.04450702153554875;
  params.Jacob[39] = 1.7117453902485025;
  params.Jacob[40] = 1.1504727980139053;
  params.Jacob[41] = -0.05962309578364744;
  params.Jacob[42] = -0.1788825540764547;
  params.Jacob[43] = -1.1280569263625857;
  params.Jacob[44] = -1.2911464767927057;
  params.Jacob[45] = -1.7055053231225696;
  params.Jacob[46] = 1.56957275034837;
  params.Jacob[47] = 0.5607064675962357;
  params.B[0] = -1.4266707301147146;
  params.a_max[0] = -0.3434923211351708;
  params.a_max[1] = -1.8035643024085055;
  params.a_max[2] = -1.1625066019105454;
  params.a_max[3] = 0.9228324965161532;
  params.a_max[4] = 0.6044910817663975;
  params.a_max[5] = -0.0840868104920891;
  params.a_max[6] = -0.900877978017443;
  params.a_max[7] = 0.608892500264739;
  params.opt_qdot_prev[0] = 1.8257980452695217;
  params.opt_qdot_prev[1] = -0.25791777529922877;
  params.opt_qdot_prev[2] = -1.7194699796493191;
  params.opt_qdot_prev[3] = -1.7690740487081298;
  params.opt_qdot_prev[4] = -1.6685159248097703;
  params.opt_qdot_prev[5] = 1.8388287490128845;
  params.opt_qdot_prev[6] = 0.16304334474597537;
  params.opt_qdot_prev[7] = 1.3498497306788897;
}
