/* Produced by CVXGEN, 2017-11-22 15:58:03 -0500.  */
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
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q_final[0] = 1.5507979025745755;
  params.Q_final[3] = 0;
  params.Q_final[6] = 0;
  params.Q_final[1] = 0;
  params.Q_final[4] = 1.7081478226181048;
  params.Q_final[7] = 0;
  params.Q_final[2] = 0;
  params.Q_final[5] = 0;
  params.Q_final[8] = 1.2909047389129444;
  params.A[0] = 0.04331042079065206;
  params.A[1] = 1.5717878173906188;
  params.A[2] = 1.5851723557337523;
  params.A[3] = -1.497658758144655;
  params.A[4] = -1.171028487447253;
  params.A[5] = -1.7941311867966805;
  params.A[6] = -0.23676062539745413;
  params.A[7] = -1.8804951564857322;
  params.A[8] = -0.17266710242115568;
  params.x_0[0] = 0.596576190459043;
  params.x_0[1] = -0.8860508694080989;
  params.x_0[2] = 0.7050196079205251;
  params.B[0] = 0.3634512696654033;
  params.B[1] = -1.9040724704913385;
  params.B[2] = 0.23541635196352795;
  params.u_max[0] = 0.5185048938149308;
}
