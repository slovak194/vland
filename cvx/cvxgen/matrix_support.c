/* Produced by CVXGEN, 2017-11-22 15:58:02 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
  lhs[0] = -rhs[11]*(-params.B[0])-rhs[22]*(1);
  lhs[1] = -rhs[11]*(-params.B[1])-rhs[23]*(1);
  lhs[2] = -rhs[11]*(-params.B[2])-rhs[24]*(1);
  lhs[3] = -rhs[12]*(-params.B[0])-rhs[22]*(-params.A[0])-rhs[23]*(-params.A[3])-rhs[24]*(-params.A[6])-rhs[25]*(1);
  lhs[4] = -rhs[12]*(-params.B[1])-rhs[22]*(-params.A[1])-rhs[23]*(-params.A[4])-rhs[24]*(-params.A[7])-rhs[26]*(1);
  lhs[5] = -rhs[12]*(-params.B[2])-rhs[22]*(-params.A[2])-rhs[23]*(-params.A[5])-rhs[24]*(-params.A[8])-rhs[27]*(1);
  lhs[6] = -rhs[13]*(-params.B[0])-rhs[25]*(-params.A[0])-rhs[26]*(-params.A[3])-rhs[27]*(-params.A[6])-rhs[28]*(1);
  lhs[7] = -rhs[13]*(-params.B[1])-rhs[25]*(-params.A[1])-rhs[26]*(-params.A[4])-rhs[27]*(-params.A[7])-rhs[29]*(1);
  lhs[8] = -rhs[13]*(-params.B[2])-rhs[25]*(-params.A[2])-rhs[26]*(-params.A[5])-rhs[27]*(-params.A[8])-rhs[30]*(1);
  lhs[9] = -rhs[14]*(-params.B[0])-rhs[28]*(-params.A[0])-rhs[29]*(-params.A[3])-rhs[30]*(-params.A[6])-rhs[31]*(1);
  lhs[10] = -rhs[14]*(-params.B[1])-rhs[28]*(-params.A[1])-rhs[29]*(-params.A[4])-rhs[30]*(-params.A[7])-rhs[32]*(1);
  lhs[11] = -rhs[14]*(-params.B[2])-rhs[28]*(-params.A[2])-rhs[29]*(-params.A[5])-rhs[30]*(-params.A[8])-rhs[33]*(1);
  lhs[12] = -rhs[15]*(-params.B[0])-rhs[31]*(-params.A[0])-rhs[32]*(-params.A[3])-rhs[33]*(-params.A[6])-rhs[34]*(1);
  lhs[13] = -rhs[15]*(-params.B[1])-rhs[31]*(-params.A[1])-rhs[32]*(-params.A[4])-rhs[33]*(-params.A[7])-rhs[35]*(1);
  lhs[14] = -rhs[15]*(-params.B[2])-rhs[31]*(-params.A[2])-rhs[32]*(-params.A[5])-rhs[33]*(-params.A[8])-rhs[36]*(1);
  lhs[15] = -rhs[16]*(-params.B[0])-rhs[34]*(-params.A[0])-rhs[35]*(-params.A[3])-rhs[36]*(-params.A[6])-rhs[37]*(1);
  lhs[16] = -rhs[16]*(-params.B[1])-rhs[34]*(-params.A[1])-rhs[35]*(-params.A[4])-rhs[36]*(-params.A[7])-rhs[38]*(1);
  lhs[17] = -rhs[16]*(-params.B[2])-rhs[34]*(-params.A[2])-rhs[35]*(-params.A[5])-rhs[36]*(-params.A[8])-rhs[39]*(1);
  lhs[18] = -rhs[17]*(-params.B[0])-rhs[37]*(-params.A[0])-rhs[38]*(-params.A[3])-rhs[39]*(-params.A[6])-rhs[40]*(1);
  lhs[19] = -rhs[17]*(-params.B[1])-rhs[37]*(-params.A[1])-rhs[38]*(-params.A[4])-rhs[39]*(-params.A[7])-rhs[41]*(1);
  lhs[20] = -rhs[17]*(-params.B[2])-rhs[37]*(-params.A[2])-rhs[38]*(-params.A[5])-rhs[39]*(-params.A[8])-rhs[42]*(1);
  lhs[21] = -rhs[18]*(-params.B[0])-rhs[40]*(-params.A[0])-rhs[41]*(-params.A[3])-rhs[42]*(-params.A[6])-rhs[43]*(1);
  lhs[22] = -rhs[18]*(-params.B[1])-rhs[40]*(-params.A[1])-rhs[41]*(-params.A[4])-rhs[42]*(-params.A[7])-rhs[44]*(1);
  lhs[23] = -rhs[18]*(-params.B[2])-rhs[40]*(-params.A[2])-rhs[41]*(-params.A[5])-rhs[42]*(-params.A[8])-rhs[45]*(1);
  lhs[24] = -rhs[19]*(-params.B[0])-rhs[43]*(-params.A[0])-rhs[44]*(-params.A[3])-rhs[45]*(-params.A[6])-rhs[46]*(1);
  lhs[25] = -rhs[19]*(-params.B[1])-rhs[43]*(-params.A[1])-rhs[44]*(-params.A[4])-rhs[45]*(-params.A[7])-rhs[47]*(1);
  lhs[26] = -rhs[19]*(-params.B[2])-rhs[43]*(-params.A[2])-rhs[44]*(-params.A[5])-rhs[45]*(-params.A[8])-rhs[48]*(1);
  lhs[27] = -rhs[20]*(-params.B[0])-rhs[46]*(-params.A[0])-rhs[47]*(-params.A[3])-rhs[48]*(-params.A[6])-rhs[49]*(1);
  lhs[28] = -rhs[20]*(-params.B[1])-rhs[46]*(-params.A[1])-rhs[47]*(-params.A[4])-rhs[48]*(-params.A[7])-rhs[50]*(1);
  lhs[29] = -rhs[20]*(-params.B[2])-rhs[46]*(-params.A[2])-rhs[47]*(-params.A[5])-rhs[48]*(-params.A[8])-rhs[51]*(1);
  lhs[30] = -rhs[21]*(-params.B[0])-rhs[49]*(-params.A[0])-rhs[50]*(-params.A[3])-rhs[51]*(-params.A[6])-rhs[52]*(1);
  lhs[31] = -rhs[21]*(-params.B[1])-rhs[49]*(-params.A[1])-rhs[50]*(-params.A[4])-rhs[51]*(-params.A[7])-rhs[53]*(1);
  lhs[32] = -rhs[21]*(-params.B[2])-rhs[49]*(-params.A[2])-rhs[50]*(-params.A[5])-rhs[51]*(-params.A[8])-rhs[54]*(1);
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
  lhs[6] = 0;
  lhs[7] = 0;
  lhs[8] = 0;
  lhs[9] = 0;
  lhs[10] = 0;
  lhs[11] = -rhs[0]*(-params.B[0])-rhs[1]*(-params.B[1])-rhs[2]*(-params.B[2]);
  lhs[12] = -rhs[3]*(-params.B[0])-rhs[4]*(-params.B[1])-rhs[5]*(-params.B[2]);
  lhs[13] = -rhs[6]*(-params.B[0])-rhs[7]*(-params.B[1])-rhs[8]*(-params.B[2]);
  lhs[14] = -rhs[9]*(-params.B[0])-rhs[10]*(-params.B[1])-rhs[11]*(-params.B[2]);
  lhs[15] = -rhs[12]*(-params.B[0])-rhs[13]*(-params.B[1])-rhs[14]*(-params.B[2]);
  lhs[16] = -rhs[15]*(-params.B[0])-rhs[16]*(-params.B[1])-rhs[17]*(-params.B[2]);
  lhs[17] = -rhs[18]*(-params.B[0])-rhs[19]*(-params.B[1])-rhs[20]*(-params.B[2]);
  lhs[18] = -rhs[21]*(-params.B[0])-rhs[22]*(-params.B[1])-rhs[23]*(-params.B[2]);
  lhs[19] = -rhs[24]*(-params.B[0])-rhs[25]*(-params.B[1])-rhs[26]*(-params.B[2]);
  lhs[20] = -rhs[27]*(-params.B[0])-rhs[28]*(-params.B[1])-rhs[29]*(-params.B[2]);
  lhs[21] = -rhs[30]*(-params.B[0])-rhs[31]*(-params.B[1])-rhs[32]*(-params.B[2]);
  lhs[22] = -rhs[0]*(1)-rhs[3]*(-params.A[0])-rhs[4]*(-params.A[1])-rhs[5]*(-params.A[2]);
  lhs[23] = -rhs[1]*(1)-rhs[3]*(-params.A[3])-rhs[4]*(-params.A[4])-rhs[5]*(-params.A[5]);
  lhs[24] = -rhs[2]*(1)-rhs[3]*(-params.A[6])-rhs[4]*(-params.A[7])-rhs[5]*(-params.A[8]);
  lhs[25] = -rhs[3]*(1)-rhs[6]*(-params.A[0])-rhs[7]*(-params.A[1])-rhs[8]*(-params.A[2]);
  lhs[26] = -rhs[4]*(1)-rhs[6]*(-params.A[3])-rhs[7]*(-params.A[4])-rhs[8]*(-params.A[5]);
  lhs[27] = -rhs[5]*(1)-rhs[6]*(-params.A[6])-rhs[7]*(-params.A[7])-rhs[8]*(-params.A[8]);
  lhs[28] = -rhs[6]*(1)-rhs[9]*(-params.A[0])-rhs[10]*(-params.A[1])-rhs[11]*(-params.A[2]);
  lhs[29] = -rhs[7]*(1)-rhs[9]*(-params.A[3])-rhs[10]*(-params.A[4])-rhs[11]*(-params.A[5]);
  lhs[30] = -rhs[8]*(1)-rhs[9]*(-params.A[6])-rhs[10]*(-params.A[7])-rhs[11]*(-params.A[8]);
  lhs[31] = -rhs[9]*(1)-rhs[12]*(-params.A[0])-rhs[13]*(-params.A[1])-rhs[14]*(-params.A[2]);
  lhs[32] = -rhs[10]*(1)-rhs[12]*(-params.A[3])-rhs[13]*(-params.A[4])-rhs[14]*(-params.A[5]);
  lhs[33] = -rhs[11]*(1)-rhs[12]*(-params.A[6])-rhs[13]*(-params.A[7])-rhs[14]*(-params.A[8]);
  lhs[34] = -rhs[12]*(1)-rhs[15]*(-params.A[0])-rhs[16]*(-params.A[1])-rhs[17]*(-params.A[2]);
  lhs[35] = -rhs[13]*(1)-rhs[15]*(-params.A[3])-rhs[16]*(-params.A[4])-rhs[17]*(-params.A[5]);
  lhs[36] = -rhs[14]*(1)-rhs[15]*(-params.A[6])-rhs[16]*(-params.A[7])-rhs[17]*(-params.A[8]);
  lhs[37] = -rhs[15]*(1)-rhs[18]*(-params.A[0])-rhs[19]*(-params.A[1])-rhs[20]*(-params.A[2]);
  lhs[38] = -rhs[16]*(1)-rhs[18]*(-params.A[3])-rhs[19]*(-params.A[4])-rhs[20]*(-params.A[5]);
  lhs[39] = -rhs[17]*(1)-rhs[18]*(-params.A[6])-rhs[19]*(-params.A[7])-rhs[20]*(-params.A[8]);
  lhs[40] = -rhs[18]*(1)-rhs[21]*(-params.A[0])-rhs[22]*(-params.A[1])-rhs[23]*(-params.A[2]);
  lhs[41] = -rhs[19]*(1)-rhs[21]*(-params.A[3])-rhs[22]*(-params.A[4])-rhs[23]*(-params.A[5]);
  lhs[42] = -rhs[20]*(1)-rhs[21]*(-params.A[6])-rhs[22]*(-params.A[7])-rhs[23]*(-params.A[8]);
  lhs[43] = -rhs[21]*(1)-rhs[24]*(-params.A[0])-rhs[25]*(-params.A[1])-rhs[26]*(-params.A[2]);
  lhs[44] = -rhs[22]*(1)-rhs[24]*(-params.A[3])-rhs[25]*(-params.A[4])-rhs[26]*(-params.A[5]);
  lhs[45] = -rhs[23]*(1)-rhs[24]*(-params.A[6])-rhs[25]*(-params.A[7])-rhs[26]*(-params.A[8]);
  lhs[46] = -rhs[24]*(1)-rhs[27]*(-params.A[0])-rhs[28]*(-params.A[1])-rhs[29]*(-params.A[2]);
  lhs[47] = -rhs[25]*(1)-rhs[27]*(-params.A[3])-rhs[28]*(-params.A[4])-rhs[29]*(-params.A[5]);
  lhs[48] = -rhs[26]*(1)-rhs[27]*(-params.A[6])-rhs[28]*(-params.A[7])-rhs[29]*(-params.A[8]);
  lhs[49] = -rhs[27]*(1)-rhs[30]*(-params.A[0])-rhs[31]*(-params.A[1])-rhs[32]*(-params.A[2]);
  lhs[50] = -rhs[28]*(1)-rhs[30]*(-params.A[3])-rhs[31]*(-params.A[4])-rhs[32]*(-params.A[5]);
  lhs[51] = -rhs[29]*(1)-rhs[30]*(-params.A[6])-rhs[31]*(-params.A[7])-rhs[32]*(-params.A[8]);
  lhs[52] = -rhs[30]*(1);
  lhs[53] = -rhs[31]*(1);
  lhs[54] = -rhs[32]*(1);
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(1);
  lhs[1] = -rhs[0]*(-1)-rhs[11]*(1);
  lhs[2] = -rhs[0]*(-1)-rhs[11]*(-1);
  lhs[3] = -rhs[1]*(1);
  lhs[4] = -rhs[1]*(-1)-rhs[12]*(1);
  lhs[5] = -rhs[1]*(-1)-rhs[12]*(-1);
  lhs[6] = -rhs[2]*(1);
  lhs[7] = -rhs[2]*(-1)-rhs[13]*(1);
  lhs[8] = -rhs[2]*(-1)-rhs[13]*(-1);
  lhs[9] = -rhs[3]*(1);
  lhs[10] = -rhs[3]*(-1)-rhs[14]*(1);
  lhs[11] = -rhs[3]*(-1)-rhs[14]*(-1);
  lhs[12] = -rhs[4]*(1);
  lhs[13] = -rhs[4]*(-1)-rhs[15]*(1);
  lhs[14] = -rhs[4]*(-1)-rhs[15]*(-1);
  lhs[15] = -rhs[5]*(1);
  lhs[16] = -rhs[5]*(-1)-rhs[16]*(1);
  lhs[17] = -rhs[5]*(-1)-rhs[16]*(-1);
  lhs[18] = -rhs[6]*(1);
  lhs[19] = -rhs[6]*(-1)-rhs[17]*(1);
  lhs[20] = -rhs[6]*(-1)-rhs[17]*(-1);
  lhs[21] = -rhs[7]*(1);
  lhs[22] = -rhs[7]*(-1)-rhs[18]*(1);
  lhs[23] = -rhs[7]*(-1)-rhs[18]*(-1);
  lhs[24] = -rhs[8]*(1);
  lhs[25] = -rhs[8]*(-1)-rhs[19]*(1);
  lhs[26] = -rhs[8]*(-1)-rhs[19]*(-1);
  lhs[27] = -rhs[9]*(1);
  lhs[28] = -rhs[9]*(-1)-rhs[20]*(1);
  lhs[29] = -rhs[9]*(-1)-rhs[20]*(-1);
  lhs[30] = -rhs[10]*(1);
  lhs[31] = -rhs[10]*(-1)-rhs[21]*(1);
  lhs[32] = -rhs[10]*(-1)-rhs[21]*(-1);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(1)-rhs[1]*(-1)-rhs[2]*(-1);
  lhs[1] = -rhs[3]*(1)-rhs[4]*(-1)-rhs[5]*(-1);
  lhs[2] = -rhs[6]*(1)-rhs[7]*(-1)-rhs[8]*(-1);
  lhs[3] = -rhs[9]*(1)-rhs[10]*(-1)-rhs[11]*(-1);
  lhs[4] = -rhs[12]*(1)-rhs[13]*(-1)-rhs[14]*(-1);
  lhs[5] = -rhs[15]*(1)-rhs[16]*(-1)-rhs[17]*(-1);
  lhs[6] = -rhs[18]*(1)-rhs[19]*(-1)-rhs[20]*(-1);
  lhs[7] = -rhs[21]*(1)-rhs[22]*(-1)-rhs[23]*(-1);
  lhs[8] = -rhs[24]*(1)-rhs[25]*(-1)-rhs[26]*(-1);
  lhs[9] = -rhs[27]*(1)-rhs[28]*(-1)-rhs[29]*(-1);
  lhs[10] = -rhs[30]*(1)-rhs[31]*(-1)-rhs[32]*(-1);
  lhs[11] = -rhs[1]*(1)-rhs[2]*(-1);
  lhs[12] = -rhs[4]*(1)-rhs[5]*(-1);
  lhs[13] = -rhs[7]*(1)-rhs[8]*(-1);
  lhs[14] = -rhs[10]*(1)-rhs[11]*(-1);
  lhs[15] = -rhs[13]*(1)-rhs[14]*(-1);
  lhs[16] = -rhs[16]*(1)-rhs[17]*(-1);
  lhs[17] = -rhs[19]*(1)-rhs[20]*(-1);
  lhs[18] = -rhs[22]*(1)-rhs[23]*(-1);
  lhs[19] = -rhs[25]*(1)-rhs[26]*(-1);
  lhs[20] = -rhs[28]*(1)-rhs[29]*(-1);
  lhs[21] = -rhs[31]*(1)-rhs[32]*(-1);
  lhs[22] = 0;
  lhs[23] = 0;
  lhs[24] = 0;
  lhs[25] = 0;
  lhs[26] = 0;
  lhs[27] = 0;
  lhs[28] = 0;
  lhs[29] = 0;
  lhs[30] = 0;
  lhs[31] = 0;
  lhs[32] = 0;
  lhs[33] = 0;
  lhs[34] = 0;
  lhs[35] = 0;
  lhs[36] = 0;
  lhs[37] = 0;
  lhs[38] = 0;
  lhs[39] = 0;
  lhs[40] = 0;
  lhs[41] = 0;
  lhs[42] = 0;
  lhs[43] = 0;
  lhs[44] = 0;
  lhs[45] = 0;
  lhs[46] = 0;
  lhs[47] = 0;
  lhs[48] = 0;
  lhs[49] = 0;
  lhs[50] = 0;
  lhs[51] = 0;
  lhs[52] = 0;
  lhs[53] = 0;
  lhs[54] = 0;
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
  lhs[6] = 0;
  lhs[7] = 0;
  lhs[8] = 0;
  lhs[9] = 0;
  lhs[10] = 0;
  lhs[11] = 0;
  lhs[12] = 0;
  lhs[13] = 0;
  lhs[14] = 0;
  lhs[15] = 0;
  lhs[16] = 0;
  lhs[17] = 0;
  lhs[18] = 0;
  lhs[19] = 0;
  lhs[20] = 0;
  lhs[21] = 0;
  lhs[22] = 0;
  lhs[23] = 0;
  lhs[24] = 0;
  lhs[25] = 0;
  lhs[26] = 0;
  lhs[27] = 0;
  lhs[28] = 0;
  lhs[29] = 0;
  lhs[30] = 0;
  lhs[31] = 0;
  lhs[32] = 0;
  lhs[33] = 0;
  lhs[34] = 0;
  lhs[35] = 0;
  lhs[36] = 0;
  lhs[37] = 0;
  lhs[38] = 0;
  lhs[39] = 0;
  lhs[40] = 0;
  lhs[41] = 0;
  lhs[42] = 0;
  lhs[43] = 0;
  lhs[44] = 0;
  lhs[45] = 0;
  lhs[46] = 0;
  lhs[47] = 0;
  lhs[48] = 0;
  lhs[49] = 0;
  lhs[50] = 0;
  lhs[51] = 0;
  lhs[52] = rhs[52]*(2*params.Q_final[0])+rhs[53]*(2*params.Q_final[3])+rhs[54]*(2*params.Q_final[6]);
  lhs[53] = rhs[52]*(2*params.Q_final[1])+rhs[53]*(2*params.Q_final[4])+rhs[54]*(2*params.Q_final[7]);
  lhs[54] = rhs[52]*(2*params.Q_final[2])+rhs[53]*(2*params.Q_final[5])+rhs[54]*(2*params.Q_final[8]);
}
void fillq(void) {
  work.q[0] = 0;
  work.q[1] = 0;
  work.q[2] = 0;
  work.q[3] = 0;
  work.q[4] = 0;
  work.q[5] = 0;
  work.q[6] = 0;
  work.q[7] = 0;
  work.q[8] = 0;
  work.q[9] = 0;
  work.q[10] = 0;
  work.q[11] = 0;
  work.q[12] = 0;
  work.q[13] = 0;
  work.q[14] = 0;
  work.q[15] = 0;
  work.q[16] = 0;
  work.q[17] = 0;
  work.q[18] = 0;
  work.q[19] = 0;
  work.q[20] = 0;
  work.q[21] = 0;
  work.q[22] = 0;
  work.q[23] = 0;
  work.q[24] = 0;
  work.q[25] = 0;
  work.q[26] = 0;
  work.q[27] = 0;
  work.q[28] = 0;
  work.q[29] = 0;
  work.q[30] = 0;
  work.q[31] = 0;
  work.q[32] = 0;
  work.q[33] = 0;
  work.q[34] = 0;
  work.q[35] = 0;
  work.q[36] = 0;
  work.q[37] = 0;
  work.q[38] = 0;
  work.q[39] = 0;
  work.q[40] = 0;
  work.q[41] = 0;
  work.q[42] = 0;
  work.q[43] = 0;
  work.q[44] = 0;
  work.q[45] = 0;
  work.q[46] = 0;
  work.q[47] = 0;
  work.q[48] = 0;
  work.q[49] = 0;
  work.q[50] = 0;
  work.q[51] = 0;
  work.q[52] = 0;
  work.q[53] = 0;
  work.q[54] = 0;
}
void fillh(void) {
  work.h[0] = params.u_max[0];
  work.h[1] = 0;
  work.h[2] = 0;
  work.h[3] = params.u_max[0];
  work.h[4] = 0;
  work.h[5] = 0;
  work.h[6] = params.u_max[0];
  work.h[7] = 0;
  work.h[8] = 0;
  work.h[9] = params.u_max[0];
  work.h[10] = 0;
  work.h[11] = 0;
  work.h[12] = params.u_max[0];
  work.h[13] = 0;
  work.h[14] = 0;
  work.h[15] = params.u_max[0];
  work.h[16] = 0;
  work.h[17] = 0;
  work.h[18] = params.u_max[0];
  work.h[19] = 0;
  work.h[20] = 0;
  work.h[21] = params.u_max[0];
  work.h[22] = 0;
  work.h[23] = 0;
  work.h[24] = params.u_max[0];
  work.h[25] = 0;
  work.h[26] = 0;
  work.h[27] = params.u_max[0];
  work.h[28] = 0;
  work.h[29] = 0;
  work.h[30] = params.u_max[0];
  work.h[31] = 0;
  work.h[32] = 0;
}
void fillb(void) {
  work.b[0] = params.A[0]*params.x_0[0]+params.A[3]*params.x_0[1]+params.A[6]*params.x_0[2];
  work.b[1] = params.A[1]*params.x_0[0]+params.A[4]*params.x_0[1]+params.A[7]*params.x_0[2];
  work.b[2] = params.A[2]*params.x_0[0]+params.A[5]*params.x_0[1]+params.A[8]*params.x_0[2];
  work.b[3] = 0;
  work.b[4] = 0;
  work.b[5] = 0;
  work.b[6] = 0;
  work.b[7] = 0;
  work.b[8] = 0;
  work.b[9] = 0;
  work.b[10] = 0;
  work.b[11] = 0;
  work.b[12] = 0;
  work.b[13] = 0;
  work.b[14] = 0;
  work.b[15] = 0;
  work.b[16] = 0;
  work.b[17] = 0;
  work.b[18] = 0;
  work.b[19] = 0;
  work.b[20] = 0;
  work.b[21] = 0;
  work.b[22] = 0;
  work.b[23] = 0;
  work.b[24] = 0;
  work.b[25] = 0;
  work.b[26] = 0;
  work.b[27] = 0;
  work.b[28] = 0;
  work.b[29] = 0;
  work.b[30] = 0;
  work.b[31] = 0;
  work.b[32] = 0;
}
void pre_ops(void) {
}
