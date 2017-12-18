/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
int lRun2;
ret = 0;
for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 13];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 13 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 13 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 13 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 13 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 13 + 5];
acadoWorkspace.state[6] = acadoVariables.x[lRun1 * 13 + 6];
acadoWorkspace.state[7] = acadoVariables.x[lRun1 * 13 + 7];
acadoWorkspace.state[8] = acadoVariables.x[lRun1 * 13 + 8];
acadoWorkspace.state[9] = acadoVariables.x[lRun1 * 13 + 9];
acadoWorkspace.state[10] = acadoVariables.x[lRun1 * 13 + 10];
acadoWorkspace.state[11] = acadoVariables.x[lRun1 * 13 + 11];
acadoWorkspace.state[12] = acadoVariables.x[lRun1 * 13 + 12];

acadoWorkspace.state[221] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.state[222] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.state[223] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.state[224] = acadoVariables.od[lRun1 * 3];
acadoWorkspace.state[225] = acadoVariables.od[lRun1 * 3 + 1];
acadoWorkspace.state[226] = acadoVariables.od[lRun1 * 3 + 2];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 13] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 13 + 13];
acadoWorkspace.d[lRun1 * 13 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 13 + 14];
acadoWorkspace.d[lRun1 * 13 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 13 + 15];
acadoWorkspace.d[lRun1 * 13 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 13 + 16];
acadoWorkspace.d[lRun1 * 13 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 13 + 17];
acadoWorkspace.d[lRun1 * 13 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 13 + 18];
acadoWorkspace.d[lRun1 * 13 + 6] = acadoWorkspace.state[6] - acadoVariables.x[lRun1 * 13 + 19];
acadoWorkspace.d[lRun1 * 13 + 7] = acadoWorkspace.state[7] - acadoVariables.x[lRun1 * 13 + 20];
acadoWorkspace.d[lRun1 * 13 + 8] = acadoWorkspace.state[8] - acadoVariables.x[lRun1 * 13 + 21];
acadoWorkspace.d[lRun1 * 13 + 9] = acadoWorkspace.state[9] - acadoVariables.x[lRun1 * 13 + 22];
acadoWorkspace.d[lRun1 * 13 + 10] = acadoWorkspace.state[10] - acadoVariables.x[lRun1 * 13 + 23];
acadoWorkspace.d[lRun1 * 13 + 11] = acadoWorkspace.state[11] - acadoVariables.x[lRun1 * 13 + 24];
acadoWorkspace.d[lRun1 * 13 + 12] = acadoWorkspace.state[12] - acadoVariables.x[lRun1 * 13 + 25];

for (lRun2 = 0; lRun2 < 169; ++lRun2)
acadoWorkspace.evGx[(0) + ((lRun2) + (lRun1 * 169))] = acadoWorkspace.state[lRun2 + 13];


acadoWorkspace.evGu[lRun1 * 39] = acadoWorkspace.state[182];
acadoWorkspace.evGu[lRun1 * 39 + 1] = acadoWorkspace.state[183];
acadoWorkspace.evGu[lRun1 * 39 + 2] = acadoWorkspace.state[184];
acadoWorkspace.evGu[lRun1 * 39 + 3] = acadoWorkspace.state[185];
acadoWorkspace.evGu[lRun1 * 39 + 4] = acadoWorkspace.state[186];
acadoWorkspace.evGu[lRun1 * 39 + 5] = acadoWorkspace.state[187];
acadoWorkspace.evGu[lRun1 * 39 + 6] = acadoWorkspace.state[188];
acadoWorkspace.evGu[lRun1 * 39 + 7] = acadoWorkspace.state[189];
acadoWorkspace.evGu[lRun1 * 39 + 8] = acadoWorkspace.state[190];
acadoWorkspace.evGu[lRun1 * 39 + 9] = acadoWorkspace.state[191];
acadoWorkspace.evGu[lRun1 * 39 + 10] = acadoWorkspace.state[192];
acadoWorkspace.evGu[lRun1 * 39 + 11] = acadoWorkspace.state[193];
acadoWorkspace.evGu[lRun1 * 39 + 12] = acadoWorkspace.state[194];
acadoWorkspace.evGu[lRun1 * 39 + 13] = acadoWorkspace.state[195];
acadoWorkspace.evGu[lRun1 * 39 + 14] = acadoWorkspace.state[196];
acadoWorkspace.evGu[lRun1 * 39 + 15] = acadoWorkspace.state[197];
acadoWorkspace.evGu[lRun1 * 39 + 16] = acadoWorkspace.state[198];
acadoWorkspace.evGu[lRun1 * 39 + 17] = acadoWorkspace.state[199];
acadoWorkspace.evGu[lRun1 * 39 + 18] = acadoWorkspace.state[200];
acadoWorkspace.evGu[lRun1 * 39 + 19] = acadoWorkspace.state[201];
acadoWorkspace.evGu[lRun1 * 39 + 20] = acadoWorkspace.state[202];
acadoWorkspace.evGu[lRun1 * 39 + 21] = acadoWorkspace.state[203];
acadoWorkspace.evGu[lRun1 * 39 + 22] = acadoWorkspace.state[204];
acadoWorkspace.evGu[lRun1 * 39 + 23] = acadoWorkspace.state[205];
acadoWorkspace.evGu[lRun1 * 39 + 24] = acadoWorkspace.state[206];
acadoWorkspace.evGu[lRun1 * 39 + 25] = acadoWorkspace.state[207];
acadoWorkspace.evGu[lRun1 * 39 + 26] = acadoWorkspace.state[208];
acadoWorkspace.evGu[lRun1 * 39 + 27] = acadoWorkspace.state[209];
acadoWorkspace.evGu[lRun1 * 39 + 28] = acadoWorkspace.state[210];
acadoWorkspace.evGu[lRun1 * 39 + 29] = acadoWorkspace.state[211];
acadoWorkspace.evGu[lRun1 * 39 + 30] = acadoWorkspace.state[212];
acadoWorkspace.evGu[lRun1 * 39 + 31] = acadoWorkspace.state[213];
acadoWorkspace.evGu[lRun1 * 39 + 32] = acadoWorkspace.state[214];
acadoWorkspace.evGu[lRun1 * 39 + 33] = acadoWorkspace.state[215];
acadoWorkspace.evGu[lRun1 * 39 + 34] = acadoWorkspace.state[216];
acadoWorkspace.evGu[lRun1 * 39 + 35] = acadoWorkspace.state[217];
acadoWorkspace.evGu[lRun1 * 39 + 36] = acadoWorkspace.state[218];
acadoWorkspace.evGu[lRun1 * 39 + 37] = acadoWorkspace.state[219];
acadoWorkspace.evGu[lRun1 * 39 + 38] = acadoWorkspace.state[220];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 13;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = xd[8];
out[9] = xd[9];
out[10] = xd[10];
out[11] = xd[11];
out[12] = xd[12];
out[13] = u[0];
out[14] = u[1];
out[15] = u[2];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = xd[8];
out[9] = xd[9];
out[10] = xd[10];
out[11] = xd[11];
out[12] = xd[12];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = +tmpObjS[20];
tmpQ2[21] = +tmpObjS[21];
tmpQ2[22] = +tmpObjS[22];
tmpQ2[23] = +tmpObjS[23];
tmpQ2[24] = +tmpObjS[24];
tmpQ2[25] = +tmpObjS[25];
tmpQ2[26] = +tmpObjS[26];
tmpQ2[27] = +tmpObjS[27];
tmpQ2[28] = +tmpObjS[28];
tmpQ2[29] = +tmpObjS[29];
tmpQ2[30] = +tmpObjS[30];
tmpQ2[31] = +tmpObjS[31];
tmpQ2[32] = +tmpObjS[32];
tmpQ2[33] = +tmpObjS[33];
tmpQ2[34] = +tmpObjS[34];
tmpQ2[35] = +tmpObjS[35];
tmpQ2[36] = +tmpObjS[36];
tmpQ2[37] = +tmpObjS[37];
tmpQ2[38] = +tmpObjS[38];
tmpQ2[39] = +tmpObjS[39];
tmpQ2[40] = +tmpObjS[40];
tmpQ2[41] = +tmpObjS[41];
tmpQ2[42] = +tmpObjS[42];
tmpQ2[43] = +tmpObjS[43];
tmpQ2[44] = +tmpObjS[44];
tmpQ2[45] = +tmpObjS[45];
tmpQ2[46] = +tmpObjS[46];
tmpQ2[47] = +tmpObjS[47];
tmpQ2[48] = +tmpObjS[48];
tmpQ2[49] = +tmpObjS[49];
tmpQ2[50] = +tmpObjS[50];
tmpQ2[51] = +tmpObjS[51];
tmpQ2[52] = +tmpObjS[52];
tmpQ2[53] = +tmpObjS[53];
tmpQ2[54] = +tmpObjS[54];
tmpQ2[55] = +tmpObjS[55];
tmpQ2[56] = +tmpObjS[56];
tmpQ2[57] = +tmpObjS[57];
tmpQ2[58] = +tmpObjS[58];
tmpQ2[59] = +tmpObjS[59];
tmpQ2[60] = +tmpObjS[60];
tmpQ2[61] = +tmpObjS[61];
tmpQ2[62] = +tmpObjS[62];
tmpQ2[63] = +tmpObjS[63];
tmpQ2[64] = +tmpObjS[64];
tmpQ2[65] = +tmpObjS[65];
tmpQ2[66] = +tmpObjS[66];
tmpQ2[67] = +tmpObjS[67];
tmpQ2[68] = +tmpObjS[68];
tmpQ2[69] = +tmpObjS[69];
tmpQ2[70] = +tmpObjS[70];
tmpQ2[71] = +tmpObjS[71];
tmpQ2[72] = +tmpObjS[72];
tmpQ2[73] = +tmpObjS[73];
tmpQ2[74] = +tmpObjS[74];
tmpQ2[75] = +tmpObjS[75];
tmpQ2[76] = +tmpObjS[76];
tmpQ2[77] = +tmpObjS[77];
tmpQ2[78] = +tmpObjS[78];
tmpQ2[79] = +tmpObjS[79];
tmpQ2[80] = +tmpObjS[80];
tmpQ2[81] = +tmpObjS[81];
tmpQ2[82] = +tmpObjS[82];
tmpQ2[83] = +tmpObjS[83];
tmpQ2[84] = +tmpObjS[84];
tmpQ2[85] = +tmpObjS[85];
tmpQ2[86] = +tmpObjS[86];
tmpQ2[87] = +tmpObjS[87];
tmpQ2[88] = +tmpObjS[88];
tmpQ2[89] = +tmpObjS[89];
tmpQ2[90] = +tmpObjS[90];
tmpQ2[91] = +tmpObjS[91];
tmpQ2[92] = +tmpObjS[92];
tmpQ2[93] = +tmpObjS[93];
tmpQ2[94] = +tmpObjS[94];
tmpQ2[95] = +tmpObjS[95];
tmpQ2[96] = +tmpObjS[96];
tmpQ2[97] = +tmpObjS[97];
tmpQ2[98] = +tmpObjS[98];
tmpQ2[99] = +tmpObjS[99];
tmpQ2[100] = +tmpObjS[100];
tmpQ2[101] = +tmpObjS[101];
tmpQ2[102] = +tmpObjS[102];
tmpQ2[103] = +tmpObjS[103];
tmpQ2[104] = +tmpObjS[104];
tmpQ2[105] = +tmpObjS[105];
tmpQ2[106] = +tmpObjS[106];
tmpQ2[107] = +tmpObjS[107];
tmpQ2[108] = +tmpObjS[108];
tmpQ2[109] = +tmpObjS[109];
tmpQ2[110] = +tmpObjS[110];
tmpQ2[111] = +tmpObjS[111];
tmpQ2[112] = +tmpObjS[112];
tmpQ2[113] = +tmpObjS[113];
tmpQ2[114] = +tmpObjS[114];
tmpQ2[115] = +tmpObjS[115];
tmpQ2[116] = +tmpObjS[116];
tmpQ2[117] = +tmpObjS[117];
tmpQ2[118] = +tmpObjS[118];
tmpQ2[119] = +tmpObjS[119];
tmpQ2[120] = +tmpObjS[120];
tmpQ2[121] = +tmpObjS[121];
tmpQ2[122] = +tmpObjS[122];
tmpQ2[123] = +tmpObjS[123];
tmpQ2[124] = +tmpObjS[124];
tmpQ2[125] = +tmpObjS[125];
tmpQ2[126] = +tmpObjS[126];
tmpQ2[127] = +tmpObjS[127];
tmpQ2[128] = +tmpObjS[128];
tmpQ2[129] = +tmpObjS[129];
tmpQ2[130] = +tmpObjS[130];
tmpQ2[131] = +tmpObjS[131];
tmpQ2[132] = +tmpObjS[132];
tmpQ2[133] = +tmpObjS[133];
tmpQ2[134] = +tmpObjS[134];
tmpQ2[135] = +tmpObjS[135];
tmpQ2[136] = +tmpObjS[136];
tmpQ2[137] = +tmpObjS[137];
tmpQ2[138] = +tmpObjS[138];
tmpQ2[139] = +tmpObjS[139];
tmpQ2[140] = +tmpObjS[140];
tmpQ2[141] = +tmpObjS[141];
tmpQ2[142] = +tmpObjS[142];
tmpQ2[143] = +tmpObjS[143];
tmpQ2[144] = +tmpObjS[144];
tmpQ2[145] = +tmpObjS[145];
tmpQ2[146] = +tmpObjS[146];
tmpQ2[147] = +tmpObjS[147];
tmpQ2[148] = +tmpObjS[148];
tmpQ2[149] = +tmpObjS[149];
tmpQ2[150] = +tmpObjS[150];
tmpQ2[151] = +tmpObjS[151];
tmpQ2[152] = +tmpObjS[152];
tmpQ2[153] = +tmpObjS[153];
tmpQ2[154] = +tmpObjS[154];
tmpQ2[155] = +tmpObjS[155];
tmpQ2[156] = +tmpObjS[156];
tmpQ2[157] = +tmpObjS[157];
tmpQ2[158] = +tmpObjS[158];
tmpQ2[159] = +tmpObjS[159];
tmpQ2[160] = +tmpObjS[160];
tmpQ2[161] = +tmpObjS[161];
tmpQ2[162] = +tmpObjS[162];
tmpQ2[163] = +tmpObjS[163];
tmpQ2[164] = +tmpObjS[164];
tmpQ2[165] = +tmpObjS[165];
tmpQ2[166] = +tmpObjS[166];
tmpQ2[167] = +tmpObjS[167];
tmpQ2[168] = +tmpObjS[168];
tmpQ2[169] = +tmpObjS[169];
tmpQ2[170] = +tmpObjS[170];
tmpQ2[171] = +tmpObjS[171];
tmpQ2[172] = +tmpObjS[172];
tmpQ2[173] = +tmpObjS[173];
tmpQ2[174] = +tmpObjS[174];
tmpQ2[175] = +tmpObjS[175];
tmpQ2[176] = +tmpObjS[176];
tmpQ2[177] = +tmpObjS[177];
tmpQ2[178] = +tmpObjS[178];
tmpQ2[179] = +tmpObjS[179];
tmpQ2[180] = +tmpObjS[180];
tmpQ2[181] = +tmpObjS[181];
tmpQ2[182] = +tmpObjS[182];
tmpQ2[183] = +tmpObjS[183];
tmpQ2[184] = +tmpObjS[184];
tmpQ2[185] = +tmpObjS[185];
tmpQ2[186] = +tmpObjS[186];
tmpQ2[187] = +tmpObjS[187];
tmpQ2[188] = +tmpObjS[188];
tmpQ2[189] = +tmpObjS[189];
tmpQ2[190] = +tmpObjS[190];
tmpQ2[191] = +tmpObjS[191];
tmpQ2[192] = +tmpObjS[192];
tmpQ2[193] = +tmpObjS[193];
tmpQ2[194] = +tmpObjS[194];
tmpQ2[195] = +tmpObjS[195];
tmpQ2[196] = +tmpObjS[196];
tmpQ2[197] = +tmpObjS[197];
tmpQ2[198] = +tmpObjS[198];
tmpQ2[199] = +tmpObjS[199];
tmpQ2[200] = +tmpObjS[200];
tmpQ2[201] = +tmpObjS[201];
tmpQ2[202] = +tmpObjS[202];
tmpQ2[203] = +tmpObjS[203];
tmpQ2[204] = +tmpObjS[204];
tmpQ2[205] = +tmpObjS[205];
tmpQ2[206] = +tmpObjS[206];
tmpQ2[207] = +tmpObjS[207];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[4];
tmpQ1[5] = + tmpQ2[5];
tmpQ1[6] = + tmpQ2[6];
tmpQ1[7] = + tmpQ2[7];
tmpQ1[8] = + tmpQ2[8];
tmpQ1[9] = + tmpQ2[9];
tmpQ1[10] = + tmpQ2[10];
tmpQ1[11] = + tmpQ2[11];
tmpQ1[12] = + tmpQ2[12];
tmpQ1[13] = + tmpQ2[16];
tmpQ1[14] = + tmpQ2[17];
tmpQ1[15] = + tmpQ2[18];
tmpQ1[16] = + tmpQ2[19];
tmpQ1[17] = + tmpQ2[20];
tmpQ1[18] = + tmpQ2[21];
tmpQ1[19] = + tmpQ2[22];
tmpQ1[20] = + tmpQ2[23];
tmpQ1[21] = + tmpQ2[24];
tmpQ1[22] = + tmpQ2[25];
tmpQ1[23] = + tmpQ2[26];
tmpQ1[24] = + tmpQ2[27];
tmpQ1[25] = + tmpQ2[28];
tmpQ1[26] = + tmpQ2[32];
tmpQ1[27] = + tmpQ2[33];
tmpQ1[28] = + tmpQ2[34];
tmpQ1[29] = + tmpQ2[35];
tmpQ1[30] = + tmpQ2[36];
tmpQ1[31] = + tmpQ2[37];
tmpQ1[32] = + tmpQ2[38];
tmpQ1[33] = + tmpQ2[39];
tmpQ1[34] = + tmpQ2[40];
tmpQ1[35] = + tmpQ2[41];
tmpQ1[36] = + tmpQ2[42];
tmpQ1[37] = + tmpQ2[43];
tmpQ1[38] = + tmpQ2[44];
tmpQ1[39] = + tmpQ2[48];
tmpQ1[40] = + tmpQ2[49];
tmpQ1[41] = + tmpQ2[50];
tmpQ1[42] = + tmpQ2[51];
tmpQ1[43] = + tmpQ2[52];
tmpQ1[44] = + tmpQ2[53];
tmpQ1[45] = + tmpQ2[54];
tmpQ1[46] = + tmpQ2[55];
tmpQ1[47] = + tmpQ2[56];
tmpQ1[48] = + tmpQ2[57];
tmpQ1[49] = + tmpQ2[58];
tmpQ1[50] = + tmpQ2[59];
tmpQ1[51] = + tmpQ2[60];
tmpQ1[52] = + tmpQ2[64];
tmpQ1[53] = + tmpQ2[65];
tmpQ1[54] = + tmpQ2[66];
tmpQ1[55] = + tmpQ2[67];
tmpQ1[56] = + tmpQ2[68];
tmpQ1[57] = + tmpQ2[69];
tmpQ1[58] = + tmpQ2[70];
tmpQ1[59] = + tmpQ2[71];
tmpQ1[60] = + tmpQ2[72];
tmpQ1[61] = + tmpQ2[73];
tmpQ1[62] = + tmpQ2[74];
tmpQ1[63] = + tmpQ2[75];
tmpQ1[64] = + tmpQ2[76];
tmpQ1[65] = + tmpQ2[80];
tmpQ1[66] = + tmpQ2[81];
tmpQ1[67] = + tmpQ2[82];
tmpQ1[68] = + tmpQ2[83];
tmpQ1[69] = + tmpQ2[84];
tmpQ1[70] = + tmpQ2[85];
tmpQ1[71] = + tmpQ2[86];
tmpQ1[72] = + tmpQ2[87];
tmpQ1[73] = + tmpQ2[88];
tmpQ1[74] = + tmpQ2[89];
tmpQ1[75] = + tmpQ2[90];
tmpQ1[76] = + tmpQ2[91];
tmpQ1[77] = + tmpQ2[92];
tmpQ1[78] = + tmpQ2[96];
tmpQ1[79] = + tmpQ2[97];
tmpQ1[80] = + tmpQ2[98];
tmpQ1[81] = + tmpQ2[99];
tmpQ1[82] = + tmpQ2[100];
tmpQ1[83] = + tmpQ2[101];
tmpQ1[84] = + tmpQ2[102];
tmpQ1[85] = + tmpQ2[103];
tmpQ1[86] = + tmpQ2[104];
tmpQ1[87] = + tmpQ2[105];
tmpQ1[88] = + tmpQ2[106];
tmpQ1[89] = + tmpQ2[107];
tmpQ1[90] = + tmpQ2[108];
tmpQ1[91] = + tmpQ2[112];
tmpQ1[92] = + tmpQ2[113];
tmpQ1[93] = + tmpQ2[114];
tmpQ1[94] = + tmpQ2[115];
tmpQ1[95] = + tmpQ2[116];
tmpQ1[96] = + tmpQ2[117];
tmpQ1[97] = + tmpQ2[118];
tmpQ1[98] = + tmpQ2[119];
tmpQ1[99] = + tmpQ2[120];
tmpQ1[100] = + tmpQ2[121];
tmpQ1[101] = + tmpQ2[122];
tmpQ1[102] = + tmpQ2[123];
tmpQ1[103] = + tmpQ2[124];
tmpQ1[104] = + tmpQ2[128];
tmpQ1[105] = + tmpQ2[129];
tmpQ1[106] = + tmpQ2[130];
tmpQ1[107] = + tmpQ2[131];
tmpQ1[108] = + tmpQ2[132];
tmpQ1[109] = + tmpQ2[133];
tmpQ1[110] = + tmpQ2[134];
tmpQ1[111] = + tmpQ2[135];
tmpQ1[112] = + tmpQ2[136];
tmpQ1[113] = + tmpQ2[137];
tmpQ1[114] = + tmpQ2[138];
tmpQ1[115] = + tmpQ2[139];
tmpQ1[116] = + tmpQ2[140];
tmpQ1[117] = + tmpQ2[144];
tmpQ1[118] = + tmpQ2[145];
tmpQ1[119] = + tmpQ2[146];
tmpQ1[120] = + tmpQ2[147];
tmpQ1[121] = + tmpQ2[148];
tmpQ1[122] = + tmpQ2[149];
tmpQ1[123] = + tmpQ2[150];
tmpQ1[124] = + tmpQ2[151];
tmpQ1[125] = + tmpQ2[152];
tmpQ1[126] = + tmpQ2[153];
tmpQ1[127] = + tmpQ2[154];
tmpQ1[128] = + tmpQ2[155];
tmpQ1[129] = + tmpQ2[156];
tmpQ1[130] = + tmpQ2[160];
tmpQ1[131] = + tmpQ2[161];
tmpQ1[132] = + tmpQ2[162];
tmpQ1[133] = + tmpQ2[163];
tmpQ1[134] = + tmpQ2[164];
tmpQ1[135] = + tmpQ2[165];
tmpQ1[136] = + tmpQ2[166];
tmpQ1[137] = + tmpQ2[167];
tmpQ1[138] = + tmpQ2[168];
tmpQ1[139] = + tmpQ2[169];
tmpQ1[140] = + tmpQ2[170];
tmpQ1[141] = + tmpQ2[171];
tmpQ1[142] = + tmpQ2[172];
tmpQ1[143] = + tmpQ2[176];
tmpQ1[144] = + tmpQ2[177];
tmpQ1[145] = + tmpQ2[178];
tmpQ1[146] = + tmpQ2[179];
tmpQ1[147] = + tmpQ2[180];
tmpQ1[148] = + tmpQ2[181];
tmpQ1[149] = + tmpQ2[182];
tmpQ1[150] = + tmpQ2[183];
tmpQ1[151] = + tmpQ2[184];
tmpQ1[152] = + tmpQ2[185];
tmpQ1[153] = + tmpQ2[186];
tmpQ1[154] = + tmpQ2[187];
tmpQ1[155] = + tmpQ2[188];
tmpQ1[156] = + tmpQ2[192];
tmpQ1[157] = + tmpQ2[193];
tmpQ1[158] = + tmpQ2[194];
tmpQ1[159] = + tmpQ2[195];
tmpQ1[160] = + tmpQ2[196];
tmpQ1[161] = + tmpQ2[197];
tmpQ1[162] = + tmpQ2[198];
tmpQ1[163] = + tmpQ2[199];
tmpQ1[164] = + tmpQ2[200];
tmpQ1[165] = + tmpQ2[201];
tmpQ1[166] = + tmpQ2[202];
tmpQ1[167] = + tmpQ2[203];
tmpQ1[168] = + tmpQ2[204];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[208];
tmpR2[1] = +tmpObjS[209];
tmpR2[2] = +tmpObjS[210];
tmpR2[3] = +tmpObjS[211];
tmpR2[4] = +tmpObjS[212];
tmpR2[5] = +tmpObjS[213];
tmpR2[6] = +tmpObjS[214];
tmpR2[7] = +tmpObjS[215];
tmpR2[8] = +tmpObjS[216];
tmpR2[9] = +tmpObjS[217];
tmpR2[10] = +tmpObjS[218];
tmpR2[11] = +tmpObjS[219];
tmpR2[12] = +tmpObjS[220];
tmpR2[13] = +tmpObjS[221];
tmpR2[14] = +tmpObjS[222];
tmpR2[15] = +tmpObjS[223];
tmpR2[16] = +tmpObjS[224];
tmpR2[17] = +tmpObjS[225];
tmpR2[18] = +tmpObjS[226];
tmpR2[19] = +tmpObjS[227];
tmpR2[20] = +tmpObjS[228];
tmpR2[21] = +tmpObjS[229];
tmpR2[22] = +tmpObjS[230];
tmpR2[23] = +tmpObjS[231];
tmpR2[24] = +tmpObjS[232];
tmpR2[25] = +tmpObjS[233];
tmpR2[26] = +tmpObjS[234];
tmpR2[27] = +tmpObjS[235];
tmpR2[28] = +tmpObjS[236];
tmpR2[29] = +tmpObjS[237];
tmpR2[30] = +tmpObjS[238];
tmpR2[31] = +tmpObjS[239];
tmpR2[32] = +tmpObjS[240];
tmpR2[33] = +tmpObjS[241];
tmpR2[34] = +tmpObjS[242];
tmpR2[35] = +tmpObjS[243];
tmpR2[36] = +tmpObjS[244];
tmpR2[37] = +tmpObjS[245];
tmpR2[38] = +tmpObjS[246];
tmpR2[39] = +tmpObjS[247];
tmpR2[40] = +tmpObjS[248];
tmpR2[41] = +tmpObjS[249];
tmpR2[42] = +tmpObjS[250];
tmpR2[43] = +tmpObjS[251];
tmpR2[44] = +tmpObjS[252];
tmpR2[45] = +tmpObjS[253];
tmpR2[46] = +tmpObjS[254];
tmpR2[47] = +tmpObjS[255];
tmpR1[0] = + tmpR2[13];
tmpR1[1] = + tmpR2[14];
tmpR1[2] = + tmpR2[15];
tmpR1[3] = + tmpR2[29];
tmpR1[4] = + tmpR2[30];
tmpR1[5] = + tmpR2[31];
tmpR1[6] = + tmpR2[45];
tmpR1[7] = + tmpR2[46];
tmpR1[8] = + tmpR2[47];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN2[16] = +tmpObjSEndTerm[16];
tmpQN2[17] = +tmpObjSEndTerm[17];
tmpQN2[18] = +tmpObjSEndTerm[18];
tmpQN2[19] = +tmpObjSEndTerm[19];
tmpQN2[20] = +tmpObjSEndTerm[20];
tmpQN2[21] = +tmpObjSEndTerm[21];
tmpQN2[22] = +tmpObjSEndTerm[22];
tmpQN2[23] = +tmpObjSEndTerm[23];
tmpQN2[24] = +tmpObjSEndTerm[24];
tmpQN2[25] = +tmpObjSEndTerm[25];
tmpQN2[26] = +tmpObjSEndTerm[26];
tmpQN2[27] = +tmpObjSEndTerm[27];
tmpQN2[28] = +tmpObjSEndTerm[28];
tmpQN2[29] = +tmpObjSEndTerm[29];
tmpQN2[30] = +tmpObjSEndTerm[30];
tmpQN2[31] = +tmpObjSEndTerm[31];
tmpQN2[32] = +tmpObjSEndTerm[32];
tmpQN2[33] = +tmpObjSEndTerm[33];
tmpQN2[34] = +tmpObjSEndTerm[34];
tmpQN2[35] = +tmpObjSEndTerm[35];
tmpQN2[36] = +tmpObjSEndTerm[36];
tmpQN2[37] = +tmpObjSEndTerm[37];
tmpQN2[38] = +tmpObjSEndTerm[38];
tmpQN2[39] = +tmpObjSEndTerm[39];
tmpQN2[40] = +tmpObjSEndTerm[40];
tmpQN2[41] = +tmpObjSEndTerm[41];
tmpQN2[42] = +tmpObjSEndTerm[42];
tmpQN2[43] = +tmpObjSEndTerm[43];
tmpQN2[44] = +tmpObjSEndTerm[44];
tmpQN2[45] = +tmpObjSEndTerm[45];
tmpQN2[46] = +tmpObjSEndTerm[46];
tmpQN2[47] = +tmpObjSEndTerm[47];
tmpQN2[48] = +tmpObjSEndTerm[48];
tmpQN2[49] = +tmpObjSEndTerm[49];
tmpQN2[50] = +tmpObjSEndTerm[50];
tmpQN2[51] = +tmpObjSEndTerm[51];
tmpQN2[52] = +tmpObjSEndTerm[52];
tmpQN2[53] = +tmpObjSEndTerm[53];
tmpQN2[54] = +tmpObjSEndTerm[54];
tmpQN2[55] = +tmpObjSEndTerm[55];
tmpQN2[56] = +tmpObjSEndTerm[56];
tmpQN2[57] = +tmpObjSEndTerm[57];
tmpQN2[58] = +tmpObjSEndTerm[58];
tmpQN2[59] = +tmpObjSEndTerm[59];
tmpQN2[60] = +tmpObjSEndTerm[60];
tmpQN2[61] = +tmpObjSEndTerm[61];
tmpQN2[62] = +tmpObjSEndTerm[62];
tmpQN2[63] = +tmpObjSEndTerm[63];
tmpQN2[64] = +tmpObjSEndTerm[64];
tmpQN2[65] = +tmpObjSEndTerm[65];
tmpQN2[66] = +tmpObjSEndTerm[66];
tmpQN2[67] = +tmpObjSEndTerm[67];
tmpQN2[68] = +tmpObjSEndTerm[68];
tmpQN2[69] = +tmpObjSEndTerm[69];
tmpQN2[70] = +tmpObjSEndTerm[70];
tmpQN2[71] = +tmpObjSEndTerm[71];
tmpQN2[72] = +tmpObjSEndTerm[72];
tmpQN2[73] = +tmpObjSEndTerm[73];
tmpQN2[74] = +tmpObjSEndTerm[74];
tmpQN2[75] = +tmpObjSEndTerm[75];
tmpQN2[76] = +tmpObjSEndTerm[76];
tmpQN2[77] = +tmpObjSEndTerm[77];
tmpQN2[78] = +tmpObjSEndTerm[78];
tmpQN2[79] = +tmpObjSEndTerm[79];
tmpQN2[80] = +tmpObjSEndTerm[80];
tmpQN2[81] = +tmpObjSEndTerm[81];
tmpQN2[82] = +tmpObjSEndTerm[82];
tmpQN2[83] = +tmpObjSEndTerm[83];
tmpQN2[84] = +tmpObjSEndTerm[84];
tmpQN2[85] = +tmpObjSEndTerm[85];
tmpQN2[86] = +tmpObjSEndTerm[86];
tmpQN2[87] = +tmpObjSEndTerm[87];
tmpQN2[88] = +tmpObjSEndTerm[88];
tmpQN2[89] = +tmpObjSEndTerm[89];
tmpQN2[90] = +tmpObjSEndTerm[90];
tmpQN2[91] = +tmpObjSEndTerm[91];
tmpQN2[92] = +tmpObjSEndTerm[92];
tmpQN2[93] = +tmpObjSEndTerm[93];
tmpQN2[94] = +tmpObjSEndTerm[94];
tmpQN2[95] = +tmpObjSEndTerm[95];
tmpQN2[96] = +tmpObjSEndTerm[96];
tmpQN2[97] = +tmpObjSEndTerm[97];
tmpQN2[98] = +tmpObjSEndTerm[98];
tmpQN2[99] = +tmpObjSEndTerm[99];
tmpQN2[100] = +tmpObjSEndTerm[100];
tmpQN2[101] = +tmpObjSEndTerm[101];
tmpQN2[102] = +tmpObjSEndTerm[102];
tmpQN2[103] = +tmpObjSEndTerm[103];
tmpQN2[104] = +tmpObjSEndTerm[104];
tmpQN2[105] = +tmpObjSEndTerm[105];
tmpQN2[106] = +tmpObjSEndTerm[106];
tmpQN2[107] = +tmpObjSEndTerm[107];
tmpQN2[108] = +tmpObjSEndTerm[108];
tmpQN2[109] = +tmpObjSEndTerm[109];
tmpQN2[110] = +tmpObjSEndTerm[110];
tmpQN2[111] = +tmpObjSEndTerm[111];
tmpQN2[112] = +tmpObjSEndTerm[112];
tmpQN2[113] = +tmpObjSEndTerm[113];
tmpQN2[114] = +tmpObjSEndTerm[114];
tmpQN2[115] = +tmpObjSEndTerm[115];
tmpQN2[116] = +tmpObjSEndTerm[116];
tmpQN2[117] = +tmpObjSEndTerm[117];
tmpQN2[118] = +tmpObjSEndTerm[118];
tmpQN2[119] = +tmpObjSEndTerm[119];
tmpQN2[120] = +tmpObjSEndTerm[120];
tmpQN2[121] = +tmpObjSEndTerm[121];
tmpQN2[122] = +tmpObjSEndTerm[122];
tmpQN2[123] = +tmpObjSEndTerm[123];
tmpQN2[124] = +tmpObjSEndTerm[124];
tmpQN2[125] = +tmpObjSEndTerm[125];
tmpQN2[126] = +tmpObjSEndTerm[126];
tmpQN2[127] = +tmpObjSEndTerm[127];
tmpQN2[128] = +tmpObjSEndTerm[128];
tmpQN2[129] = +tmpObjSEndTerm[129];
tmpQN2[130] = +tmpObjSEndTerm[130];
tmpQN2[131] = +tmpObjSEndTerm[131];
tmpQN2[132] = +tmpObjSEndTerm[132];
tmpQN2[133] = +tmpObjSEndTerm[133];
tmpQN2[134] = +tmpObjSEndTerm[134];
tmpQN2[135] = +tmpObjSEndTerm[135];
tmpQN2[136] = +tmpObjSEndTerm[136];
tmpQN2[137] = +tmpObjSEndTerm[137];
tmpQN2[138] = +tmpObjSEndTerm[138];
tmpQN2[139] = +tmpObjSEndTerm[139];
tmpQN2[140] = +tmpObjSEndTerm[140];
tmpQN2[141] = +tmpObjSEndTerm[141];
tmpQN2[142] = +tmpObjSEndTerm[142];
tmpQN2[143] = +tmpObjSEndTerm[143];
tmpQN2[144] = +tmpObjSEndTerm[144];
tmpQN2[145] = +tmpObjSEndTerm[145];
tmpQN2[146] = +tmpObjSEndTerm[146];
tmpQN2[147] = +tmpObjSEndTerm[147];
tmpQN2[148] = +tmpObjSEndTerm[148];
tmpQN2[149] = +tmpObjSEndTerm[149];
tmpQN2[150] = +tmpObjSEndTerm[150];
tmpQN2[151] = +tmpObjSEndTerm[151];
tmpQN2[152] = +tmpObjSEndTerm[152];
tmpQN2[153] = +tmpObjSEndTerm[153];
tmpQN2[154] = +tmpObjSEndTerm[154];
tmpQN2[155] = +tmpObjSEndTerm[155];
tmpQN2[156] = +tmpObjSEndTerm[156];
tmpQN2[157] = +tmpObjSEndTerm[157];
tmpQN2[158] = +tmpObjSEndTerm[158];
tmpQN2[159] = +tmpObjSEndTerm[159];
tmpQN2[160] = +tmpObjSEndTerm[160];
tmpQN2[161] = +tmpObjSEndTerm[161];
tmpQN2[162] = +tmpObjSEndTerm[162];
tmpQN2[163] = +tmpObjSEndTerm[163];
tmpQN2[164] = +tmpObjSEndTerm[164];
tmpQN2[165] = +tmpObjSEndTerm[165];
tmpQN2[166] = +tmpObjSEndTerm[166];
tmpQN2[167] = +tmpObjSEndTerm[167];
tmpQN2[168] = +tmpObjSEndTerm[168];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
tmpQN1[9] = + tmpQN2[9];
tmpQN1[10] = + tmpQN2[10];
tmpQN1[11] = + tmpQN2[11];
tmpQN1[12] = + tmpQN2[12];
tmpQN1[13] = + tmpQN2[13];
tmpQN1[14] = + tmpQN2[14];
tmpQN1[15] = + tmpQN2[15];
tmpQN1[16] = + tmpQN2[16];
tmpQN1[17] = + tmpQN2[17];
tmpQN1[18] = + tmpQN2[18];
tmpQN1[19] = + tmpQN2[19];
tmpQN1[20] = + tmpQN2[20];
tmpQN1[21] = + tmpQN2[21];
tmpQN1[22] = + tmpQN2[22];
tmpQN1[23] = + tmpQN2[23];
tmpQN1[24] = + tmpQN2[24];
tmpQN1[25] = + tmpQN2[25];
tmpQN1[26] = + tmpQN2[26];
tmpQN1[27] = + tmpQN2[27];
tmpQN1[28] = + tmpQN2[28];
tmpQN1[29] = + tmpQN2[29];
tmpQN1[30] = + tmpQN2[30];
tmpQN1[31] = + tmpQN2[31];
tmpQN1[32] = + tmpQN2[32];
tmpQN1[33] = + tmpQN2[33];
tmpQN1[34] = + tmpQN2[34];
tmpQN1[35] = + tmpQN2[35];
tmpQN1[36] = + tmpQN2[36];
tmpQN1[37] = + tmpQN2[37];
tmpQN1[38] = + tmpQN2[38];
tmpQN1[39] = + tmpQN2[39];
tmpQN1[40] = + tmpQN2[40];
tmpQN1[41] = + tmpQN2[41];
tmpQN1[42] = + tmpQN2[42];
tmpQN1[43] = + tmpQN2[43];
tmpQN1[44] = + tmpQN2[44];
tmpQN1[45] = + tmpQN2[45];
tmpQN1[46] = + tmpQN2[46];
tmpQN1[47] = + tmpQN2[47];
tmpQN1[48] = + tmpQN2[48];
tmpQN1[49] = + tmpQN2[49];
tmpQN1[50] = + tmpQN2[50];
tmpQN1[51] = + tmpQN2[51];
tmpQN1[52] = + tmpQN2[52];
tmpQN1[53] = + tmpQN2[53];
tmpQN1[54] = + tmpQN2[54];
tmpQN1[55] = + tmpQN2[55];
tmpQN1[56] = + tmpQN2[56];
tmpQN1[57] = + tmpQN2[57];
tmpQN1[58] = + tmpQN2[58];
tmpQN1[59] = + tmpQN2[59];
tmpQN1[60] = + tmpQN2[60];
tmpQN1[61] = + tmpQN2[61];
tmpQN1[62] = + tmpQN2[62];
tmpQN1[63] = + tmpQN2[63];
tmpQN1[64] = + tmpQN2[64];
tmpQN1[65] = + tmpQN2[65];
tmpQN1[66] = + tmpQN2[66];
tmpQN1[67] = + tmpQN2[67];
tmpQN1[68] = + tmpQN2[68];
tmpQN1[69] = + tmpQN2[69];
tmpQN1[70] = + tmpQN2[70];
tmpQN1[71] = + tmpQN2[71];
tmpQN1[72] = + tmpQN2[72];
tmpQN1[73] = + tmpQN2[73];
tmpQN1[74] = + tmpQN2[74];
tmpQN1[75] = + tmpQN2[75];
tmpQN1[76] = + tmpQN2[76];
tmpQN1[77] = + tmpQN2[77];
tmpQN1[78] = + tmpQN2[78];
tmpQN1[79] = + tmpQN2[79];
tmpQN1[80] = + tmpQN2[80];
tmpQN1[81] = + tmpQN2[81];
tmpQN1[82] = + tmpQN2[82];
tmpQN1[83] = + tmpQN2[83];
tmpQN1[84] = + tmpQN2[84];
tmpQN1[85] = + tmpQN2[85];
tmpQN1[86] = + tmpQN2[86];
tmpQN1[87] = + tmpQN2[87];
tmpQN1[88] = + tmpQN2[88];
tmpQN1[89] = + tmpQN2[89];
tmpQN1[90] = + tmpQN2[90];
tmpQN1[91] = + tmpQN2[91];
tmpQN1[92] = + tmpQN2[92];
tmpQN1[93] = + tmpQN2[93];
tmpQN1[94] = + tmpQN2[94];
tmpQN1[95] = + tmpQN2[95];
tmpQN1[96] = + tmpQN2[96];
tmpQN1[97] = + tmpQN2[97];
tmpQN1[98] = + tmpQN2[98];
tmpQN1[99] = + tmpQN2[99];
tmpQN1[100] = + tmpQN2[100];
tmpQN1[101] = + tmpQN2[101];
tmpQN1[102] = + tmpQN2[102];
tmpQN1[103] = + tmpQN2[103];
tmpQN1[104] = + tmpQN2[104];
tmpQN1[105] = + tmpQN2[105];
tmpQN1[106] = + tmpQN2[106];
tmpQN1[107] = + tmpQN2[107];
tmpQN1[108] = + tmpQN2[108];
tmpQN1[109] = + tmpQN2[109];
tmpQN1[110] = + tmpQN2[110];
tmpQN1[111] = + tmpQN2[111];
tmpQN1[112] = + tmpQN2[112];
tmpQN1[113] = + tmpQN2[113];
tmpQN1[114] = + tmpQN2[114];
tmpQN1[115] = + tmpQN2[115];
tmpQN1[116] = + tmpQN2[116];
tmpQN1[117] = + tmpQN2[117];
tmpQN1[118] = + tmpQN2[118];
tmpQN1[119] = + tmpQN2[119];
tmpQN1[120] = + tmpQN2[120];
tmpQN1[121] = + tmpQN2[121];
tmpQN1[122] = + tmpQN2[122];
tmpQN1[123] = + tmpQN2[123];
tmpQN1[124] = + tmpQN2[124];
tmpQN1[125] = + tmpQN2[125];
tmpQN1[126] = + tmpQN2[126];
tmpQN1[127] = + tmpQN2[127];
tmpQN1[128] = + tmpQN2[128];
tmpQN1[129] = + tmpQN2[129];
tmpQN1[130] = + tmpQN2[130];
tmpQN1[131] = + tmpQN2[131];
tmpQN1[132] = + tmpQN2[132];
tmpQN1[133] = + tmpQN2[133];
tmpQN1[134] = + tmpQN2[134];
tmpQN1[135] = + tmpQN2[135];
tmpQN1[136] = + tmpQN2[136];
tmpQN1[137] = + tmpQN2[137];
tmpQN1[138] = + tmpQN2[138];
tmpQN1[139] = + tmpQN2[139];
tmpQN1[140] = + tmpQN2[140];
tmpQN1[141] = + tmpQN2[141];
tmpQN1[142] = + tmpQN2[142];
tmpQN1[143] = + tmpQN2[143];
tmpQN1[144] = + tmpQN2[144];
tmpQN1[145] = + tmpQN2[145];
tmpQN1[146] = + tmpQN2[146];
tmpQN1[147] = + tmpQN2[147];
tmpQN1[148] = + tmpQN2[148];
tmpQN1[149] = + tmpQN2[149];
tmpQN1[150] = + tmpQN2[150];
tmpQN1[151] = + tmpQN2[151];
tmpQN1[152] = + tmpQN2[152];
tmpQN1[153] = + tmpQN2[153];
tmpQN1[154] = + tmpQN2[154];
tmpQN1[155] = + tmpQN2[155];
tmpQN1[156] = + tmpQN2[156];
tmpQN1[157] = + tmpQN2[157];
tmpQN1[158] = + tmpQN2[158];
tmpQN1[159] = + tmpQN2[159];
tmpQN1[160] = + tmpQN2[160];
tmpQN1[161] = + tmpQN2[161];
tmpQN1[162] = + tmpQN2[162];
tmpQN1[163] = + tmpQN2[163];
tmpQN1[164] = + tmpQN2[164];
tmpQN1[165] = + tmpQN2[165];
tmpQN1[166] = + tmpQN2[166];
tmpQN1[167] = + tmpQN2[167];
tmpQN1[168] = + tmpQN2[168];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 50; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 13];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 13 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 13 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 13 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 13 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 13 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 13 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 13 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[runObj * 13 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[runObj * 13 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.x[runObj * 13 + 10];
acadoWorkspace.objValueIn[11] = acadoVariables.x[runObj * 13 + 11];
acadoWorkspace.objValueIn[12] = acadoVariables.x[runObj * 13 + 12];
acadoWorkspace.objValueIn[13] = acadoVariables.u[runObj * 3];
acadoWorkspace.objValueIn[14] = acadoVariables.u[runObj * 3 + 1];
acadoWorkspace.objValueIn[15] = acadoVariables.u[runObj * 3 + 2];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 3];
acadoWorkspace.objValueIn[17] = acadoVariables.od[runObj * 3 + 1];
acadoWorkspace.objValueIn[18] = acadoVariables.od[runObj * 3 + 2];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 16] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 16 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 16 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 16 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 16 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 16 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 16 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 16 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 16 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 16 + 9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.Dy[runObj * 16 + 10] = acadoWorkspace.objValueOut[10];
acadoWorkspace.Dy[runObj * 16 + 11] = acadoWorkspace.objValueOut[11];
acadoWorkspace.Dy[runObj * 16 + 12] = acadoWorkspace.objValueOut[12];
acadoWorkspace.Dy[runObj * 16 + 13] = acadoWorkspace.objValueOut[13];
acadoWorkspace.Dy[runObj * 16 + 14] = acadoWorkspace.objValueOut[14];
acadoWorkspace.Dy[runObj * 16 + 15] = acadoWorkspace.objValueOut[15];

acado_setObjQ1Q2( acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 169 ]), &(acadoWorkspace.Q2[ runObj * 208 ]) );

acado_setObjR1R2( acadoVariables.W, &(acadoWorkspace.R1[ runObj * 9 ]), &(acadoWorkspace.R2[ runObj * 48 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[650];
acadoWorkspace.objValueIn[1] = acadoVariables.x[651];
acadoWorkspace.objValueIn[2] = acadoVariables.x[652];
acadoWorkspace.objValueIn[3] = acadoVariables.x[653];
acadoWorkspace.objValueIn[4] = acadoVariables.x[654];
acadoWorkspace.objValueIn[5] = acadoVariables.x[655];
acadoWorkspace.objValueIn[6] = acadoVariables.x[656];
acadoWorkspace.objValueIn[7] = acadoVariables.x[657];
acadoWorkspace.objValueIn[8] = acadoVariables.x[658];
acadoWorkspace.objValueIn[9] = acadoVariables.x[659];
acadoWorkspace.objValueIn[10] = acadoVariables.x[660];
acadoWorkspace.objValueIn[11] = acadoVariables.x[661];
acadoWorkspace.objValueIn[12] = acadoVariables.x[662];
acadoWorkspace.objValueIn[13] = acadoVariables.od[150];
acadoWorkspace.objValueIn[14] = acadoVariables.od[151];
acadoWorkspace.objValueIn[15] = acadoVariables.od[152];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.DyN[8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.DyN[9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.DyN[10] = acadoWorkspace.objValueOut[10];
acadoWorkspace.DyN[11] = acadoWorkspace.objValueOut[11];
acadoWorkspace.DyN[12] = acadoWorkspace.objValueOut[12];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[6] + Gx1[3]*Gu1[9] + Gx1[4]*Gu1[12] + Gx1[5]*Gu1[15] + Gx1[6]*Gu1[18] + Gx1[7]*Gu1[21] + Gx1[8]*Gu1[24] + Gx1[9]*Gu1[27] + Gx1[10]*Gu1[30] + Gx1[11]*Gu1[33] + Gx1[12]*Gu1[36];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[7] + Gx1[3]*Gu1[10] + Gx1[4]*Gu1[13] + Gx1[5]*Gu1[16] + Gx1[6]*Gu1[19] + Gx1[7]*Gu1[22] + Gx1[8]*Gu1[25] + Gx1[9]*Gu1[28] + Gx1[10]*Gu1[31] + Gx1[11]*Gu1[34] + Gx1[12]*Gu1[37];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[11] + Gx1[4]*Gu1[14] + Gx1[5]*Gu1[17] + Gx1[6]*Gu1[20] + Gx1[7]*Gu1[23] + Gx1[8]*Gu1[26] + Gx1[9]*Gu1[29] + Gx1[10]*Gu1[32] + Gx1[11]*Gu1[35] + Gx1[12]*Gu1[38];
Gu2[3] = + Gx1[13]*Gu1[0] + Gx1[14]*Gu1[3] + Gx1[15]*Gu1[6] + Gx1[16]*Gu1[9] + Gx1[17]*Gu1[12] + Gx1[18]*Gu1[15] + Gx1[19]*Gu1[18] + Gx1[20]*Gu1[21] + Gx1[21]*Gu1[24] + Gx1[22]*Gu1[27] + Gx1[23]*Gu1[30] + Gx1[24]*Gu1[33] + Gx1[25]*Gu1[36];
Gu2[4] = + Gx1[13]*Gu1[1] + Gx1[14]*Gu1[4] + Gx1[15]*Gu1[7] + Gx1[16]*Gu1[10] + Gx1[17]*Gu1[13] + Gx1[18]*Gu1[16] + Gx1[19]*Gu1[19] + Gx1[20]*Gu1[22] + Gx1[21]*Gu1[25] + Gx1[22]*Gu1[28] + Gx1[23]*Gu1[31] + Gx1[24]*Gu1[34] + Gx1[25]*Gu1[37];
Gu2[5] = + Gx1[13]*Gu1[2] + Gx1[14]*Gu1[5] + Gx1[15]*Gu1[8] + Gx1[16]*Gu1[11] + Gx1[17]*Gu1[14] + Gx1[18]*Gu1[17] + Gx1[19]*Gu1[20] + Gx1[20]*Gu1[23] + Gx1[21]*Gu1[26] + Gx1[22]*Gu1[29] + Gx1[23]*Gu1[32] + Gx1[24]*Gu1[35] + Gx1[25]*Gu1[38];
Gu2[6] = + Gx1[26]*Gu1[0] + Gx1[27]*Gu1[3] + Gx1[28]*Gu1[6] + Gx1[29]*Gu1[9] + Gx1[30]*Gu1[12] + Gx1[31]*Gu1[15] + Gx1[32]*Gu1[18] + Gx1[33]*Gu1[21] + Gx1[34]*Gu1[24] + Gx1[35]*Gu1[27] + Gx1[36]*Gu1[30] + Gx1[37]*Gu1[33] + Gx1[38]*Gu1[36];
Gu2[7] = + Gx1[26]*Gu1[1] + Gx1[27]*Gu1[4] + Gx1[28]*Gu1[7] + Gx1[29]*Gu1[10] + Gx1[30]*Gu1[13] + Gx1[31]*Gu1[16] + Gx1[32]*Gu1[19] + Gx1[33]*Gu1[22] + Gx1[34]*Gu1[25] + Gx1[35]*Gu1[28] + Gx1[36]*Gu1[31] + Gx1[37]*Gu1[34] + Gx1[38]*Gu1[37];
Gu2[8] = + Gx1[26]*Gu1[2] + Gx1[27]*Gu1[5] + Gx1[28]*Gu1[8] + Gx1[29]*Gu1[11] + Gx1[30]*Gu1[14] + Gx1[31]*Gu1[17] + Gx1[32]*Gu1[20] + Gx1[33]*Gu1[23] + Gx1[34]*Gu1[26] + Gx1[35]*Gu1[29] + Gx1[36]*Gu1[32] + Gx1[37]*Gu1[35] + Gx1[38]*Gu1[38];
Gu2[9] = + Gx1[39]*Gu1[0] + Gx1[40]*Gu1[3] + Gx1[41]*Gu1[6] + Gx1[42]*Gu1[9] + Gx1[43]*Gu1[12] + Gx1[44]*Gu1[15] + Gx1[45]*Gu1[18] + Gx1[46]*Gu1[21] + Gx1[47]*Gu1[24] + Gx1[48]*Gu1[27] + Gx1[49]*Gu1[30] + Gx1[50]*Gu1[33] + Gx1[51]*Gu1[36];
Gu2[10] = + Gx1[39]*Gu1[1] + Gx1[40]*Gu1[4] + Gx1[41]*Gu1[7] + Gx1[42]*Gu1[10] + Gx1[43]*Gu1[13] + Gx1[44]*Gu1[16] + Gx1[45]*Gu1[19] + Gx1[46]*Gu1[22] + Gx1[47]*Gu1[25] + Gx1[48]*Gu1[28] + Gx1[49]*Gu1[31] + Gx1[50]*Gu1[34] + Gx1[51]*Gu1[37];
Gu2[11] = + Gx1[39]*Gu1[2] + Gx1[40]*Gu1[5] + Gx1[41]*Gu1[8] + Gx1[42]*Gu1[11] + Gx1[43]*Gu1[14] + Gx1[44]*Gu1[17] + Gx1[45]*Gu1[20] + Gx1[46]*Gu1[23] + Gx1[47]*Gu1[26] + Gx1[48]*Gu1[29] + Gx1[49]*Gu1[32] + Gx1[50]*Gu1[35] + Gx1[51]*Gu1[38];
Gu2[12] = + Gx1[52]*Gu1[0] + Gx1[53]*Gu1[3] + Gx1[54]*Gu1[6] + Gx1[55]*Gu1[9] + Gx1[56]*Gu1[12] + Gx1[57]*Gu1[15] + Gx1[58]*Gu1[18] + Gx1[59]*Gu1[21] + Gx1[60]*Gu1[24] + Gx1[61]*Gu1[27] + Gx1[62]*Gu1[30] + Gx1[63]*Gu1[33] + Gx1[64]*Gu1[36];
Gu2[13] = + Gx1[52]*Gu1[1] + Gx1[53]*Gu1[4] + Gx1[54]*Gu1[7] + Gx1[55]*Gu1[10] + Gx1[56]*Gu1[13] + Gx1[57]*Gu1[16] + Gx1[58]*Gu1[19] + Gx1[59]*Gu1[22] + Gx1[60]*Gu1[25] + Gx1[61]*Gu1[28] + Gx1[62]*Gu1[31] + Gx1[63]*Gu1[34] + Gx1[64]*Gu1[37];
Gu2[14] = + Gx1[52]*Gu1[2] + Gx1[53]*Gu1[5] + Gx1[54]*Gu1[8] + Gx1[55]*Gu1[11] + Gx1[56]*Gu1[14] + Gx1[57]*Gu1[17] + Gx1[58]*Gu1[20] + Gx1[59]*Gu1[23] + Gx1[60]*Gu1[26] + Gx1[61]*Gu1[29] + Gx1[62]*Gu1[32] + Gx1[63]*Gu1[35] + Gx1[64]*Gu1[38];
Gu2[15] = + Gx1[65]*Gu1[0] + Gx1[66]*Gu1[3] + Gx1[67]*Gu1[6] + Gx1[68]*Gu1[9] + Gx1[69]*Gu1[12] + Gx1[70]*Gu1[15] + Gx1[71]*Gu1[18] + Gx1[72]*Gu1[21] + Gx1[73]*Gu1[24] + Gx1[74]*Gu1[27] + Gx1[75]*Gu1[30] + Gx1[76]*Gu1[33] + Gx1[77]*Gu1[36];
Gu2[16] = + Gx1[65]*Gu1[1] + Gx1[66]*Gu1[4] + Gx1[67]*Gu1[7] + Gx1[68]*Gu1[10] + Gx1[69]*Gu1[13] + Gx1[70]*Gu1[16] + Gx1[71]*Gu1[19] + Gx1[72]*Gu1[22] + Gx1[73]*Gu1[25] + Gx1[74]*Gu1[28] + Gx1[75]*Gu1[31] + Gx1[76]*Gu1[34] + Gx1[77]*Gu1[37];
Gu2[17] = + Gx1[65]*Gu1[2] + Gx1[66]*Gu1[5] + Gx1[67]*Gu1[8] + Gx1[68]*Gu1[11] + Gx1[69]*Gu1[14] + Gx1[70]*Gu1[17] + Gx1[71]*Gu1[20] + Gx1[72]*Gu1[23] + Gx1[73]*Gu1[26] + Gx1[74]*Gu1[29] + Gx1[75]*Gu1[32] + Gx1[76]*Gu1[35] + Gx1[77]*Gu1[38];
Gu2[18] = + Gx1[78]*Gu1[0] + Gx1[79]*Gu1[3] + Gx1[80]*Gu1[6] + Gx1[81]*Gu1[9] + Gx1[82]*Gu1[12] + Gx1[83]*Gu1[15] + Gx1[84]*Gu1[18] + Gx1[85]*Gu1[21] + Gx1[86]*Gu1[24] + Gx1[87]*Gu1[27] + Gx1[88]*Gu1[30] + Gx1[89]*Gu1[33] + Gx1[90]*Gu1[36];
Gu2[19] = + Gx1[78]*Gu1[1] + Gx1[79]*Gu1[4] + Gx1[80]*Gu1[7] + Gx1[81]*Gu1[10] + Gx1[82]*Gu1[13] + Gx1[83]*Gu1[16] + Gx1[84]*Gu1[19] + Gx1[85]*Gu1[22] + Gx1[86]*Gu1[25] + Gx1[87]*Gu1[28] + Gx1[88]*Gu1[31] + Gx1[89]*Gu1[34] + Gx1[90]*Gu1[37];
Gu2[20] = + Gx1[78]*Gu1[2] + Gx1[79]*Gu1[5] + Gx1[80]*Gu1[8] + Gx1[81]*Gu1[11] + Gx1[82]*Gu1[14] + Gx1[83]*Gu1[17] + Gx1[84]*Gu1[20] + Gx1[85]*Gu1[23] + Gx1[86]*Gu1[26] + Gx1[87]*Gu1[29] + Gx1[88]*Gu1[32] + Gx1[89]*Gu1[35] + Gx1[90]*Gu1[38];
Gu2[21] = + Gx1[91]*Gu1[0] + Gx1[92]*Gu1[3] + Gx1[93]*Gu1[6] + Gx1[94]*Gu1[9] + Gx1[95]*Gu1[12] + Gx1[96]*Gu1[15] + Gx1[97]*Gu1[18] + Gx1[98]*Gu1[21] + Gx1[99]*Gu1[24] + Gx1[100]*Gu1[27] + Gx1[101]*Gu1[30] + Gx1[102]*Gu1[33] + Gx1[103]*Gu1[36];
Gu2[22] = + Gx1[91]*Gu1[1] + Gx1[92]*Gu1[4] + Gx1[93]*Gu1[7] + Gx1[94]*Gu1[10] + Gx1[95]*Gu1[13] + Gx1[96]*Gu1[16] + Gx1[97]*Gu1[19] + Gx1[98]*Gu1[22] + Gx1[99]*Gu1[25] + Gx1[100]*Gu1[28] + Gx1[101]*Gu1[31] + Gx1[102]*Gu1[34] + Gx1[103]*Gu1[37];
Gu2[23] = + Gx1[91]*Gu1[2] + Gx1[92]*Gu1[5] + Gx1[93]*Gu1[8] + Gx1[94]*Gu1[11] + Gx1[95]*Gu1[14] + Gx1[96]*Gu1[17] + Gx1[97]*Gu1[20] + Gx1[98]*Gu1[23] + Gx1[99]*Gu1[26] + Gx1[100]*Gu1[29] + Gx1[101]*Gu1[32] + Gx1[102]*Gu1[35] + Gx1[103]*Gu1[38];
Gu2[24] = + Gx1[104]*Gu1[0] + Gx1[105]*Gu1[3] + Gx1[106]*Gu1[6] + Gx1[107]*Gu1[9] + Gx1[108]*Gu1[12] + Gx1[109]*Gu1[15] + Gx1[110]*Gu1[18] + Gx1[111]*Gu1[21] + Gx1[112]*Gu1[24] + Gx1[113]*Gu1[27] + Gx1[114]*Gu1[30] + Gx1[115]*Gu1[33] + Gx1[116]*Gu1[36];
Gu2[25] = + Gx1[104]*Gu1[1] + Gx1[105]*Gu1[4] + Gx1[106]*Gu1[7] + Gx1[107]*Gu1[10] + Gx1[108]*Gu1[13] + Gx1[109]*Gu1[16] + Gx1[110]*Gu1[19] + Gx1[111]*Gu1[22] + Gx1[112]*Gu1[25] + Gx1[113]*Gu1[28] + Gx1[114]*Gu1[31] + Gx1[115]*Gu1[34] + Gx1[116]*Gu1[37];
Gu2[26] = + Gx1[104]*Gu1[2] + Gx1[105]*Gu1[5] + Gx1[106]*Gu1[8] + Gx1[107]*Gu1[11] + Gx1[108]*Gu1[14] + Gx1[109]*Gu1[17] + Gx1[110]*Gu1[20] + Gx1[111]*Gu1[23] + Gx1[112]*Gu1[26] + Gx1[113]*Gu1[29] + Gx1[114]*Gu1[32] + Gx1[115]*Gu1[35] + Gx1[116]*Gu1[38];
Gu2[27] = + Gx1[117]*Gu1[0] + Gx1[118]*Gu1[3] + Gx1[119]*Gu1[6] + Gx1[120]*Gu1[9] + Gx1[121]*Gu1[12] + Gx1[122]*Gu1[15] + Gx1[123]*Gu1[18] + Gx1[124]*Gu1[21] + Gx1[125]*Gu1[24] + Gx1[126]*Gu1[27] + Gx1[127]*Gu1[30] + Gx1[128]*Gu1[33] + Gx1[129]*Gu1[36];
Gu2[28] = + Gx1[117]*Gu1[1] + Gx1[118]*Gu1[4] + Gx1[119]*Gu1[7] + Gx1[120]*Gu1[10] + Gx1[121]*Gu1[13] + Gx1[122]*Gu1[16] + Gx1[123]*Gu1[19] + Gx1[124]*Gu1[22] + Gx1[125]*Gu1[25] + Gx1[126]*Gu1[28] + Gx1[127]*Gu1[31] + Gx1[128]*Gu1[34] + Gx1[129]*Gu1[37];
Gu2[29] = + Gx1[117]*Gu1[2] + Gx1[118]*Gu1[5] + Gx1[119]*Gu1[8] + Gx1[120]*Gu1[11] + Gx1[121]*Gu1[14] + Gx1[122]*Gu1[17] + Gx1[123]*Gu1[20] + Gx1[124]*Gu1[23] + Gx1[125]*Gu1[26] + Gx1[126]*Gu1[29] + Gx1[127]*Gu1[32] + Gx1[128]*Gu1[35] + Gx1[129]*Gu1[38];
Gu2[30] = + Gx1[130]*Gu1[0] + Gx1[131]*Gu1[3] + Gx1[132]*Gu1[6] + Gx1[133]*Gu1[9] + Gx1[134]*Gu1[12] + Gx1[135]*Gu1[15] + Gx1[136]*Gu1[18] + Gx1[137]*Gu1[21] + Gx1[138]*Gu1[24] + Gx1[139]*Gu1[27] + Gx1[140]*Gu1[30] + Gx1[141]*Gu1[33] + Gx1[142]*Gu1[36];
Gu2[31] = + Gx1[130]*Gu1[1] + Gx1[131]*Gu1[4] + Gx1[132]*Gu1[7] + Gx1[133]*Gu1[10] + Gx1[134]*Gu1[13] + Gx1[135]*Gu1[16] + Gx1[136]*Gu1[19] + Gx1[137]*Gu1[22] + Gx1[138]*Gu1[25] + Gx1[139]*Gu1[28] + Gx1[140]*Gu1[31] + Gx1[141]*Gu1[34] + Gx1[142]*Gu1[37];
Gu2[32] = + Gx1[130]*Gu1[2] + Gx1[131]*Gu1[5] + Gx1[132]*Gu1[8] + Gx1[133]*Gu1[11] + Gx1[134]*Gu1[14] + Gx1[135]*Gu1[17] + Gx1[136]*Gu1[20] + Gx1[137]*Gu1[23] + Gx1[138]*Gu1[26] + Gx1[139]*Gu1[29] + Gx1[140]*Gu1[32] + Gx1[141]*Gu1[35] + Gx1[142]*Gu1[38];
Gu2[33] = + Gx1[143]*Gu1[0] + Gx1[144]*Gu1[3] + Gx1[145]*Gu1[6] + Gx1[146]*Gu1[9] + Gx1[147]*Gu1[12] + Gx1[148]*Gu1[15] + Gx1[149]*Gu1[18] + Gx1[150]*Gu1[21] + Gx1[151]*Gu1[24] + Gx1[152]*Gu1[27] + Gx1[153]*Gu1[30] + Gx1[154]*Gu1[33] + Gx1[155]*Gu1[36];
Gu2[34] = + Gx1[143]*Gu1[1] + Gx1[144]*Gu1[4] + Gx1[145]*Gu1[7] + Gx1[146]*Gu1[10] + Gx1[147]*Gu1[13] + Gx1[148]*Gu1[16] + Gx1[149]*Gu1[19] + Gx1[150]*Gu1[22] + Gx1[151]*Gu1[25] + Gx1[152]*Gu1[28] + Gx1[153]*Gu1[31] + Gx1[154]*Gu1[34] + Gx1[155]*Gu1[37];
Gu2[35] = + Gx1[143]*Gu1[2] + Gx1[144]*Gu1[5] + Gx1[145]*Gu1[8] + Gx1[146]*Gu1[11] + Gx1[147]*Gu1[14] + Gx1[148]*Gu1[17] + Gx1[149]*Gu1[20] + Gx1[150]*Gu1[23] + Gx1[151]*Gu1[26] + Gx1[152]*Gu1[29] + Gx1[153]*Gu1[32] + Gx1[154]*Gu1[35] + Gx1[155]*Gu1[38];
Gu2[36] = + Gx1[156]*Gu1[0] + Gx1[157]*Gu1[3] + Gx1[158]*Gu1[6] + Gx1[159]*Gu1[9] + Gx1[160]*Gu1[12] + Gx1[161]*Gu1[15] + Gx1[162]*Gu1[18] + Gx1[163]*Gu1[21] + Gx1[164]*Gu1[24] + Gx1[165]*Gu1[27] + Gx1[166]*Gu1[30] + Gx1[167]*Gu1[33] + Gx1[168]*Gu1[36];
Gu2[37] = + Gx1[156]*Gu1[1] + Gx1[157]*Gu1[4] + Gx1[158]*Gu1[7] + Gx1[159]*Gu1[10] + Gx1[160]*Gu1[13] + Gx1[161]*Gu1[16] + Gx1[162]*Gu1[19] + Gx1[163]*Gu1[22] + Gx1[164]*Gu1[25] + Gx1[165]*Gu1[28] + Gx1[166]*Gu1[31] + Gx1[167]*Gu1[34] + Gx1[168]*Gu1[37];
Gu2[38] = + Gx1[156]*Gu1[2] + Gx1[157]*Gu1[5] + Gx1[158]*Gu1[8] + Gx1[159]*Gu1[11] + Gx1[160]*Gu1[14] + Gx1[161]*Gu1[17] + Gx1[162]*Gu1[20] + Gx1[163]*Gu1[23] + Gx1[164]*Gu1[26] + Gx1[165]*Gu1[29] + Gx1[166]*Gu1[32] + Gx1[167]*Gu1[35] + Gx1[168]*Gu1[38];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
Gu2[27] = Gu1[27];
Gu2[28] = Gu1[28];
Gu2[29] = Gu1[29];
Gu2[30] = Gu1[30];
Gu2[31] = Gu1[31];
Gu2[32] = Gu1[32];
Gu2[33] = Gu1[33];
Gu2[34] = Gu1[34];
Gu2[35] = Gu1[35];
Gu2[36] = Gu1[36];
Gu2[37] = Gu1[37];
Gu2[38] = Gu1[38];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 450) + (iCol * 3)] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24] + Gu1[27]*Gu2[27] + Gu1[30]*Gu2[30] + Gu1[33]*Gu2[33] + Gu1[36]*Gu2[36];
acadoWorkspace.H[(iRow * 450) + (iCol * 3 + 1)] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25] + Gu1[27]*Gu2[28] + Gu1[30]*Gu2[31] + Gu1[33]*Gu2[34] + Gu1[36]*Gu2[37];
acadoWorkspace.H[(iRow * 450) + (iCol * 3 + 2)] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26] + Gu1[27]*Gu2[29] + Gu1[30]*Gu2[32] + Gu1[33]*Gu2[35] + Gu1[36]*Gu2[38];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3)] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24] + Gu1[28]*Gu2[27] + Gu1[31]*Gu2[30] + Gu1[34]*Gu2[33] + Gu1[37]*Gu2[36];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3 + 1)] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25] + Gu1[28]*Gu2[28] + Gu1[31]*Gu2[31] + Gu1[34]*Gu2[34] + Gu1[37]*Gu2[37];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3 + 2)] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26] + Gu1[28]*Gu2[29] + Gu1[31]*Gu2[32] + Gu1[34]*Gu2[35] + Gu1[37]*Gu2[38];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3)] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24] + Gu1[29]*Gu2[27] + Gu1[32]*Gu2[30] + Gu1[35]*Gu2[33] + Gu1[38]*Gu2[36];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3 + 1)] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25] + Gu1[29]*Gu2[28] + Gu1[32]*Gu2[31] + Gu1[35]*Gu2[34] + Gu1[38]*Gu2[37];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3 + 2)] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26] + Gu1[29]*Gu2[29] + Gu1[32]*Gu2[32] + Gu1[35]*Gu2[35] + Gu1[38]*Gu2[38];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 453] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24] + Gu1[27]*Gu2[27] + Gu1[30]*Gu2[30] + Gu1[33]*Gu2[33] + Gu1[36]*Gu2[36] + R11[0];
acadoWorkspace.H[iRow * 453 + 1] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25] + Gu1[27]*Gu2[28] + Gu1[30]*Gu2[31] + Gu1[33]*Gu2[34] + Gu1[36]*Gu2[37] + R11[1];
acadoWorkspace.H[iRow * 453 + 2] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26] + Gu1[27]*Gu2[29] + Gu1[30]*Gu2[32] + Gu1[33]*Gu2[35] + Gu1[36]*Gu2[38] + R11[2];
acadoWorkspace.H[iRow * 453 + 150] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24] + Gu1[28]*Gu2[27] + Gu1[31]*Gu2[30] + Gu1[34]*Gu2[33] + Gu1[37]*Gu2[36] + R11[3];
acadoWorkspace.H[iRow * 453 + 151] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25] + Gu1[28]*Gu2[28] + Gu1[31]*Gu2[31] + Gu1[34]*Gu2[34] + Gu1[37]*Gu2[37] + R11[4];
acadoWorkspace.H[iRow * 453 + 152] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26] + Gu1[28]*Gu2[29] + Gu1[31]*Gu2[32] + Gu1[34]*Gu2[35] + Gu1[37]*Gu2[38] + R11[5];
acadoWorkspace.H[iRow * 453 + 300] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24] + Gu1[29]*Gu2[27] + Gu1[32]*Gu2[30] + Gu1[35]*Gu2[33] + Gu1[38]*Gu2[36] + R11[6];
acadoWorkspace.H[iRow * 453 + 301] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25] + Gu1[29]*Gu2[28] + Gu1[32]*Gu2[31] + Gu1[35]*Gu2[34] + Gu1[38]*Gu2[37] + R11[7];
acadoWorkspace.H[iRow * 453 + 302] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26] + Gu1[29]*Gu2[29] + Gu1[32]*Gu2[32] + Gu1[35]*Gu2[35] + Gu1[38]*Gu2[38] + R11[8];
acadoWorkspace.H[iRow * 453] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 453 + 151] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 453 + 302] += 1.0000000000000000e-10;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[13]*Gu1[3] + Gx1[26]*Gu1[6] + Gx1[39]*Gu1[9] + Gx1[52]*Gu1[12] + Gx1[65]*Gu1[15] + Gx1[78]*Gu1[18] + Gx1[91]*Gu1[21] + Gx1[104]*Gu1[24] + Gx1[117]*Gu1[27] + Gx1[130]*Gu1[30] + Gx1[143]*Gu1[33] + Gx1[156]*Gu1[36];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[13]*Gu1[4] + Gx1[26]*Gu1[7] + Gx1[39]*Gu1[10] + Gx1[52]*Gu1[13] + Gx1[65]*Gu1[16] + Gx1[78]*Gu1[19] + Gx1[91]*Gu1[22] + Gx1[104]*Gu1[25] + Gx1[117]*Gu1[28] + Gx1[130]*Gu1[31] + Gx1[143]*Gu1[34] + Gx1[156]*Gu1[37];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[13]*Gu1[5] + Gx1[26]*Gu1[8] + Gx1[39]*Gu1[11] + Gx1[52]*Gu1[14] + Gx1[65]*Gu1[17] + Gx1[78]*Gu1[20] + Gx1[91]*Gu1[23] + Gx1[104]*Gu1[26] + Gx1[117]*Gu1[29] + Gx1[130]*Gu1[32] + Gx1[143]*Gu1[35] + Gx1[156]*Gu1[38];
Gu2[3] = + Gx1[1]*Gu1[0] + Gx1[14]*Gu1[3] + Gx1[27]*Gu1[6] + Gx1[40]*Gu1[9] + Gx1[53]*Gu1[12] + Gx1[66]*Gu1[15] + Gx1[79]*Gu1[18] + Gx1[92]*Gu1[21] + Gx1[105]*Gu1[24] + Gx1[118]*Gu1[27] + Gx1[131]*Gu1[30] + Gx1[144]*Gu1[33] + Gx1[157]*Gu1[36];
Gu2[4] = + Gx1[1]*Gu1[1] + Gx1[14]*Gu1[4] + Gx1[27]*Gu1[7] + Gx1[40]*Gu1[10] + Gx1[53]*Gu1[13] + Gx1[66]*Gu1[16] + Gx1[79]*Gu1[19] + Gx1[92]*Gu1[22] + Gx1[105]*Gu1[25] + Gx1[118]*Gu1[28] + Gx1[131]*Gu1[31] + Gx1[144]*Gu1[34] + Gx1[157]*Gu1[37];
Gu2[5] = + Gx1[1]*Gu1[2] + Gx1[14]*Gu1[5] + Gx1[27]*Gu1[8] + Gx1[40]*Gu1[11] + Gx1[53]*Gu1[14] + Gx1[66]*Gu1[17] + Gx1[79]*Gu1[20] + Gx1[92]*Gu1[23] + Gx1[105]*Gu1[26] + Gx1[118]*Gu1[29] + Gx1[131]*Gu1[32] + Gx1[144]*Gu1[35] + Gx1[157]*Gu1[38];
Gu2[6] = + Gx1[2]*Gu1[0] + Gx1[15]*Gu1[3] + Gx1[28]*Gu1[6] + Gx1[41]*Gu1[9] + Gx1[54]*Gu1[12] + Gx1[67]*Gu1[15] + Gx1[80]*Gu1[18] + Gx1[93]*Gu1[21] + Gx1[106]*Gu1[24] + Gx1[119]*Gu1[27] + Gx1[132]*Gu1[30] + Gx1[145]*Gu1[33] + Gx1[158]*Gu1[36];
Gu2[7] = + Gx1[2]*Gu1[1] + Gx1[15]*Gu1[4] + Gx1[28]*Gu1[7] + Gx1[41]*Gu1[10] + Gx1[54]*Gu1[13] + Gx1[67]*Gu1[16] + Gx1[80]*Gu1[19] + Gx1[93]*Gu1[22] + Gx1[106]*Gu1[25] + Gx1[119]*Gu1[28] + Gx1[132]*Gu1[31] + Gx1[145]*Gu1[34] + Gx1[158]*Gu1[37];
Gu2[8] = + Gx1[2]*Gu1[2] + Gx1[15]*Gu1[5] + Gx1[28]*Gu1[8] + Gx1[41]*Gu1[11] + Gx1[54]*Gu1[14] + Gx1[67]*Gu1[17] + Gx1[80]*Gu1[20] + Gx1[93]*Gu1[23] + Gx1[106]*Gu1[26] + Gx1[119]*Gu1[29] + Gx1[132]*Gu1[32] + Gx1[145]*Gu1[35] + Gx1[158]*Gu1[38];
Gu2[9] = + Gx1[3]*Gu1[0] + Gx1[16]*Gu1[3] + Gx1[29]*Gu1[6] + Gx1[42]*Gu1[9] + Gx1[55]*Gu1[12] + Gx1[68]*Gu1[15] + Gx1[81]*Gu1[18] + Gx1[94]*Gu1[21] + Gx1[107]*Gu1[24] + Gx1[120]*Gu1[27] + Gx1[133]*Gu1[30] + Gx1[146]*Gu1[33] + Gx1[159]*Gu1[36];
Gu2[10] = + Gx1[3]*Gu1[1] + Gx1[16]*Gu1[4] + Gx1[29]*Gu1[7] + Gx1[42]*Gu1[10] + Gx1[55]*Gu1[13] + Gx1[68]*Gu1[16] + Gx1[81]*Gu1[19] + Gx1[94]*Gu1[22] + Gx1[107]*Gu1[25] + Gx1[120]*Gu1[28] + Gx1[133]*Gu1[31] + Gx1[146]*Gu1[34] + Gx1[159]*Gu1[37];
Gu2[11] = + Gx1[3]*Gu1[2] + Gx1[16]*Gu1[5] + Gx1[29]*Gu1[8] + Gx1[42]*Gu1[11] + Gx1[55]*Gu1[14] + Gx1[68]*Gu1[17] + Gx1[81]*Gu1[20] + Gx1[94]*Gu1[23] + Gx1[107]*Gu1[26] + Gx1[120]*Gu1[29] + Gx1[133]*Gu1[32] + Gx1[146]*Gu1[35] + Gx1[159]*Gu1[38];
Gu2[12] = + Gx1[4]*Gu1[0] + Gx1[17]*Gu1[3] + Gx1[30]*Gu1[6] + Gx1[43]*Gu1[9] + Gx1[56]*Gu1[12] + Gx1[69]*Gu1[15] + Gx1[82]*Gu1[18] + Gx1[95]*Gu1[21] + Gx1[108]*Gu1[24] + Gx1[121]*Gu1[27] + Gx1[134]*Gu1[30] + Gx1[147]*Gu1[33] + Gx1[160]*Gu1[36];
Gu2[13] = + Gx1[4]*Gu1[1] + Gx1[17]*Gu1[4] + Gx1[30]*Gu1[7] + Gx1[43]*Gu1[10] + Gx1[56]*Gu1[13] + Gx1[69]*Gu1[16] + Gx1[82]*Gu1[19] + Gx1[95]*Gu1[22] + Gx1[108]*Gu1[25] + Gx1[121]*Gu1[28] + Gx1[134]*Gu1[31] + Gx1[147]*Gu1[34] + Gx1[160]*Gu1[37];
Gu2[14] = + Gx1[4]*Gu1[2] + Gx1[17]*Gu1[5] + Gx1[30]*Gu1[8] + Gx1[43]*Gu1[11] + Gx1[56]*Gu1[14] + Gx1[69]*Gu1[17] + Gx1[82]*Gu1[20] + Gx1[95]*Gu1[23] + Gx1[108]*Gu1[26] + Gx1[121]*Gu1[29] + Gx1[134]*Gu1[32] + Gx1[147]*Gu1[35] + Gx1[160]*Gu1[38];
Gu2[15] = + Gx1[5]*Gu1[0] + Gx1[18]*Gu1[3] + Gx1[31]*Gu1[6] + Gx1[44]*Gu1[9] + Gx1[57]*Gu1[12] + Gx1[70]*Gu1[15] + Gx1[83]*Gu1[18] + Gx1[96]*Gu1[21] + Gx1[109]*Gu1[24] + Gx1[122]*Gu1[27] + Gx1[135]*Gu1[30] + Gx1[148]*Gu1[33] + Gx1[161]*Gu1[36];
Gu2[16] = + Gx1[5]*Gu1[1] + Gx1[18]*Gu1[4] + Gx1[31]*Gu1[7] + Gx1[44]*Gu1[10] + Gx1[57]*Gu1[13] + Gx1[70]*Gu1[16] + Gx1[83]*Gu1[19] + Gx1[96]*Gu1[22] + Gx1[109]*Gu1[25] + Gx1[122]*Gu1[28] + Gx1[135]*Gu1[31] + Gx1[148]*Gu1[34] + Gx1[161]*Gu1[37];
Gu2[17] = + Gx1[5]*Gu1[2] + Gx1[18]*Gu1[5] + Gx1[31]*Gu1[8] + Gx1[44]*Gu1[11] + Gx1[57]*Gu1[14] + Gx1[70]*Gu1[17] + Gx1[83]*Gu1[20] + Gx1[96]*Gu1[23] + Gx1[109]*Gu1[26] + Gx1[122]*Gu1[29] + Gx1[135]*Gu1[32] + Gx1[148]*Gu1[35] + Gx1[161]*Gu1[38];
Gu2[18] = + Gx1[6]*Gu1[0] + Gx1[19]*Gu1[3] + Gx1[32]*Gu1[6] + Gx1[45]*Gu1[9] + Gx1[58]*Gu1[12] + Gx1[71]*Gu1[15] + Gx1[84]*Gu1[18] + Gx1[97]*Gu1[21] + Gx1[110]*Gu1[24] + Gx1[123]*Gu1[27] + Gx1[136]*Gu1[30] + Gx1[149]*Gu1[33] + Gx1[162]*Gu1[36];
Gu2[19] = + Gx1[6]*Gu1[1] + Gx1[19]*Gu1[4] + Gx1[32]*Gu1[7] + Gx1[45]*Gu1[10] + Gx1[58]*Gu1[13] + Gx1[71]*Gu1[16] + Gx1[84]*Gu1[19] + Gx1[97]*Gu1[22] + Gx1[110]*Gu1[25] + Gx1[123]*Gu1[28] + Gx1[136]*Gu1[31] + Gx1[149]*Gu1[34] + Gx1[162]*Gu1[37];
Gu2[20] = + Gx1[6]*Gu1[2] + Gx1[19]*Gu1[5] + Gx1[32]*Gu1[8] + Gx1[45]*Gu1[11] + Gx1[58]*Gu1[14] + Gx1[71]*Gu1[17] + Gx1[84]*Gu1[20] + Gx1[97]*Gu1[23] + Gx1[110]*Gu1[26] + Gx1[123]*Gu1[29] + Gx1[136]*Gu1[32] + Gx1[149]*Gu1[35] + Gx1[162]*Gu1[38];
Gu2[21] = + Gx1[7]*Gu1[0] + Gx1[20]*Gu1[3] + Gx1[33]*Gu1[6] + Gx1[46]*Gu1[9] + Gx1[59]*Gu1[12] + Gx1[72]*Gu1[15] + Gx1[85]*Gu1[18] + Gx1[98]*Gu1[21] + Gx1[111]*Gu1[24] + Gx1[124]*Gu1[27] + Gx1[137]*Gu1[30] + Gx1[150]*Gu1[33] + Gx1[163]*Gu1[36];
Gu2[22] = + Gx1[7]*Gu1[1] + Gx1[20]*Gu1[4] + Gx1[33]*Gu1[7] + Gx1[46]*Gu1[10] + Gx1[59]*Gu1[13] + Gx1[72]*Gu1[16] + Gx1[85]*Gu1[19] + Gx1[98]*Gu1[22] + Gx1[111]*Gu1[25] + Gx1[124]*Gu1[28] + Gx1[137]*Gu1[31] + Gx1[150]*Gu1[34] + Gx1[163]*Gu1[37];
Gu2[23] = + Gx1[7]*Gu1[2] + Gx1[20]*Gu1[5] + Gx1[33]*Gu1[8] + Gx1[46]*Gu1[11] + Gx1[59]*Gu1[14] + Gx1[72]*Gu1[17] + Gx1[85]*Gu1[20] + Gx1[98]*Gu1[23] + Gx1[111]*Gu1[26] + Gx1[124]*Gu1[29] + Gx1[137]*Gu1[32] + Gx1[150]*Gu1[35] + Gx1[163]*Gu1[38];
Gu2[24] = + Gx1[8]*Gu1[0] + Gx1[21]*Gu1[3] + Gx1[34]*Gu1[6] + Gx1[47]*Gu1[9] + Gx1[60]*Gu1[12] + Gx1[73]*Gu1[15] + Gx1[86]*Gu1[18] + Gx1[99]*Gu1[21] + Gx1[112]*Gu1[24] + Gx1[125]*Gu1[27] + Gx1[138]*Gu1[30] + Gx1[151]*Gu1[33] + Gx1[164]*Gu1[36];
Gu2[25] = + Gx1[8]*Gu1[1] + Gx1[21]*Gu1[4] + Gx1[34]*Gu1[7] + Gx1[47]*Gu1[10] + Gx1[60]*Gu1[13] + Gx1[73]*Gu1[16] + Gx1[86]*Gu1[19] + Gx1[99]*Gu1[22] + Gx1[112]*Gu1[25] + Gx1[125]*Gu1[28] + Gx1[138]*Gu1[31] + Gx1[151]*Gu1[34] + Gx1[164]*Gu1[37];
Gu2[26] = + Gx1[8]*Gu1[2] + Gx1[21]*Gu1[5] + Gx1[34]*Gu1[8] + Gx1[47]*Gu1[11] + Gx1[60]*Gu1[14] + Gx1[73]*Gu1[17] + Gx1[86]*Gu1[20] + Gx1[99]*Gu1[23] + Gx1[112]*Gu1[26] + Gx1[125]*Gu1[29] + Gx1[138]*Gu1[32] + Gx1[151]*Gu1[35] + Gx1[164]*Gu1[38];
Gu2[27] = + Gx1[9]*Gu1[0] + Gx1[22]*Gu1[3] + Gx1[35]*Gu1[6] + Gx1[48]*Gu1[9] + Gx1[61]*Gu1[12] + Gx1[74]*Gu1[15] + Gx1[87]*Gu1[18] + Gx1[100]*Gu1[21] + Gx1[113]*Gu1[24] + Gx1[126]*Gu1[27] + Gx1[139]*Gu1[30] + Gx1[152]*Gu1[33] + Gx1[165]*Gu1[36];
Gu2[28] = + Gx1[9]*Gu1[1] + Gx1[22]*Gu1[4] + Gx1[35]*Gu1[7] + Gx1[48]*Gu1[10] + Gx1[61]*Gu1[13] + Gx1[74]*Gu1[16] + Gx1[87]*Gu1[19] + Gx1[100]*Gu1[22] + Gx1[113]*Gu1[25] + Gx1[126]*Gu1[28] + Gx1[139]*Gu1[31] + Gx1[152]*Gu1[34] + Gx1[165]*Gu1[37];
Gu2[29] = + Gx1[9]*Gu1[2] + Gx1[22]*Gu1[5] + Gx1[35]*Gu1[8] + Gx1[48]*Gu1[11] + Gx1[61]*Gu1[14] + Gx1[74]*Gu1[17] + Gx1[87]*Gu1[20] + Gx1[100]*Gu1[23] + Gx1[113]*Gu1[26] + Gx1[126]*Gu1[29] + Gx1[139]*Gu1[32] + Gx1[152]*Gu1[35] + Gx1[165]*Gu1[38];
Gu2[30] = + Gx1[10]*Gu1[0] + Gx1[23]*Gu1[3] + Gx1[36]*Gu1[6] + Gx1[49]*Gu1[9] + Gx1[62]*Gu1[12] + Gx1[75]*Gu1[15] + Gx1[88]*Gu1[18] + Gx1[101]*Gu1[21] + Gx1[114]*Gu1[24] + Gx1[127]*Gu1[27] + Gx1[140]*Gu1[30] + Gx1[153]*Gu1[33] + Gx1[166]*Gu1[36];
Gu2[31] = + Gx1[10]*Gu1[1] + Gx1[23]*Gu1[4] + Gx1[36]*Gu1[7] + Gx1[49]*Gu1[10] + Gx1[62]*Gu1[13] + Gx1[75]*Gu1[16] + Gx1[88]*Gu1[19] + Gx1[101]*Gu1[22] + Gx1[114]*Gu1[25] + Gx1[127]*Gu1[28] + Gx1[140]*Gu1[31] + Gx1[153]*Gu1[34] + Gx1[166]*Gu1[37];
Gu2[32] = + Gx1[10]*Gu1[2] + Gx1[23]*Gu1[5] + Gx1[36]*Gu1[8] + Gx1[49]*Gu1[11] + Gx1[62]*Gu1[14] + Gx1[75]*Gu1[17] + Gx1[88]*Gu1[20] + Gx1[101]*Gu1[23] + Gx1[114]*Gu1[26] + Gx1[127]*Gu1[29] + Gx1[140]*Gu1[32] + Gx1[153]*Gu1[35] + Gx1[166]*Gu1[38];
Gu2[33] = + Gx1[11]*Gu1[0] + Gx1[24]*Gu1[3] + Gx1[37]*Gu1[6] + Gx1[50]*Gu1[9] + Gx1[63]*Gu1[12] + Gx1[76]*Gu1[15] + Gx1[89]*Gu1[18] + Gx1[102]*Gu1[21] + Gx1[115]*Gu1[24] + Gx1[128]*Gu1[27] + Gx1[141]*Gu1[30] + Gx1[154]*Gu1[33] + Gx1[167]*Gu1[36];
Gu2[34] = + Gx1[11]*Gu1[1] + Gx1[24]*Gu1[4] + Gx1[37]*Gu1[7] + Gx1[50]*Gu1[10] + Gx1[63]*Gu1[13] + Gx1[76]*Gu1[16] + Gx1[89]*Gu1[19] + Gx1[102]*Gu1[22] + Gx1[115]*Gu1[25] + Gx1[128]*Gu1[28] + Gx1[141]*Gu1[31] + Gx1[154]*Gu1[34] + Gx1[167]*Gu1[37];
Gu2[35] = + Gx1[11]*Gu1[2] + Gx1[24]*Gu1[5] + Gx1[37]*Gu1[8] + Gx1[50]*Gu1[11] + Gx1[63]*Gu1[14] + Gx1[76]*Gu1[17] + Gx1[89]*Gu1[20] + Gx1[102]*Gu1[23] + Gx1[115]*Gu1[26] + Gx1[128]*Gu1[29] + Gx1[141]*Gu1[32] + Gx1[154]*Gu1[35] + Gx1[167]*Gu1[38];
Gu2[36] = + Gx1[12]*Gu1[0] + Gx1[25]*Gu1[3] + Gx1[38]*Gu1[6] + Gx1[51]*Gu1[9] + Gx1[64]*Gu1[12] + Gx1[77]*Gu1[15] + Gx1[90]*Gu1[18] + Gx1[103]*Gu1[21] + Gx1[116]*Gu1[24] + Gx1[129]*Gu1[27] + Gx1[142]*Gu1[30] + Gx1[155]*Gu1[33] + Gx1[168]*Gu1[36];
Gu2[37] = + Gx1[12]*Gu1[1] + Gx1[25]*Gu1[4] + Gx1[38]*Gu1[7] + Gx1[51]*Gu1[10] + Gx1[64]*Gu1[13] + Gx1[77]*Gu1[16] + Gx1[90]*Gu1[19] + Gx1[103]*Gu1[22] + Gx1[116]*Gu1[25] + Gx1[129]*Gu1[28] + Gx1[142]*Gu1[31] + Gx1[155]*Gu1[34] + Gx1[168]*Gu1[37];
Gu2[38] = + Gx1[12]*Gu1[2] + Gx1[25]*Gu1[5] + Gx1[38]*Gu1[8] + Gx1[51]*Gu1[11] + Gx1[64]*Gu1[14] + Gx1[77]*Gu1[17] + Gx1[90]*Gu1[20] + Gx1[103]*Gu1[23] + Gx1[116]*Gu1[26] + Gx1[129]*Gu1[29] + Gx1[142]*Gu1[32] + Gx1[155]*Gu1[35] + Gx1[168]*Gu1[38];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[3] + Q11[2]*Gu1[6] + Q11[3]*Gu1[9] + Q11[4]*Gu1[12] + Q11[5]*Gu1[15] + Q11[6]*Gu1[18] + Q11[7]*Gu1[21] + Q11[8]*Gu1[24] + Q11[9]*Gu1[27] + Q11[10]*Gu1[30] + Q11[11]*Gu1[33] + Q11[12]*Gu1[36] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[4] + Q11[2]*Gu1[7] + Q11[3]*Gu1[10] + Q11[4]*Gu1[13] + Q11[5]*Gu1[16] + Q11[6]*Gu1[19] + Q11[7]*Gu1[22] + Q11[8]*Gu1[25] + Q11[9]*Gu1[28] + Q11[10]*Gu1[31] + Q11[11]*Gu1[34] + Q11[12]*Gu1[37] + Gu2[1];
Gu3[2] = + Q11[0]*Gu1[2] + Q11[1]*Gu1[5] + Q11[2]*Gu1[8] + Q11[3]*Gu1[11] + Q11[4]*Gu1[14] + Q11[5]*Gu1[17] + Q11[6]*Gu1[20] + Q11[7]*Gu1[23] + Q11[8]*Gu1[26] + Q11[9]*Gu1[29] + Q11[10]*Gu1[32] + Q11[11]*Gu1[35] + Q11[12]*Gu1[38] + Gu2[2];
Gu3[3] = + Q11[13]*Gu1[0] + Q11[14]*Gu1[3] + Q11[15]*Gu1[6] + Q11[16]*Gu1[9] + Q11[17]*Gu1[12] + Q11[18]*Gu1[15] + Q11[19]*Gu1[18] + Q11[20]*Gu1[21] + Q11[21]*Gu1[24] + Q11[22]*Gu1[27] + Q11[23]*Gu1[30] + Q11[24]*Gu1[33] + Q11[25]*Gu1[36] + Gu2[3];
Gu3[4] = + Q11[13]*Gu1[1] + Q11[14]*Gu1[4] + Q11[15]*Gu1[7] + Q11[16]*Gu1[10] + Q11[17]*Gu1[13] + Q11[18]*Gu1[16] + Q11[19]*Gu1[19] + Q11[20]*Gu1[22] + Q11[21]*Gu1[25] + Q11[22]*Gu1[28] + Q11[23]*Gu1[31] + Q11[24]*Gu1[34] + Q11[25]*Gu1[37] + Gu2[4];
Gu3[5] = + Q11[13]*Gu1[2] + Q11[14]*Gu1[5] + Q11[15]*Gu1[8] + Q11[16]*Gu1[11] + Q11[17]*Gu1[14] + Q11[18]*Gu1[17] + Q11[19]*Gu1[20] + Q11[20]*Gu1[23] + Q11[21]*Gu1[26] + Q11[22]*Gu1[29] + Q11[23]*Gu1[32] + Q11[24]*Gu1[35] + Q11[25]*Gu1[38] + Gu2[5];
Gu3[6] = + Q11[26]*Gu1[0] + Q11[27]*Gu1[3] + Q11[28]*Gu1[6] + Q11[29]*Gu1[9] + Q11[30]*Gu1[12] + Q11[31]*Gu1[15] + Q11[32]*Gu1[18] + Q11[33]*Gu1[21] + Q11[34]*Gu1[24] + Q11[35]*Gu1[27] + Q11[36]*Gu1[30] + Q11[37]*Gu1[33] + Q11[38]*Gu1[36] + Gu2[6];
Gu3[7] = + Q11[26]*Gu1[1] + Q11[27]*Gu1[4] + Q11[28]*Gu1[7] + Q11[29]*Gu1[10] + Q11[30]*Gu1[13] + Q11[31]*Gu1[16] + Q11[32]*Gu1[19] + Q11[33]*Gu1[22] + Q11[34]*Gu1[25] + Q11[35]*Gu1[28] + Q11[36]*Gu1[31] + Q11[37]*Gu1[34] + Q11[38]*Gu1[37] + Gu2[7];
Gu3[8] = + Q11[26]*Gu1[2] + Q11[27]*Gu1[5] + Q11[28]*Gu1[8] + Q11[29]*Gu1[11] + Q11[30]*Gu1[14] + Q11[31]*Gu1[17] + Q11[32]*Gu1[20] + Q11[33]*Gu1[23] + Q11[34]*Gu1[26] + Q11[35]*Gu1[29] + Q11[36]*Gu1[32] + Q11[37]*Gu1[35] + Q11[38]*Gu1[38] + Gu2[8];
Gu3[9] = + Q11[39]*Gu1[0] + Q11[40]*Gu1[3] + Q11[41]*Gu1[6] + Q11[42]*Gu1[9] + Q11[43]*Gu1[12] + Q11[44]*Gu1[15] + Q11[45]*Gu1[18] + Q11[46]*Gu1[21] + Q11[47]*Gu1[24] + Q11[48]*Gu1[27] + Q11[49]*Gu1[30] + Q11[50]*Gu1[33] + Q11[51]*Gu1[36] + Gu2[9];
Gu3[10] = + Q11[39]*Gu1[1] + Q11[40]*Gu1[4] + Q11[41]*Gu1[7] + Q11[42]*Gu1[10] + Q11[43]*Gu1[13] + Q11[44]*Gu1[16] + Q11[45]*Gu1[19] + Q11[46]*Gu1[22] + Q11[47]*Gu1[25] + Q11[48]*Gu1[28] + Q11[49]*Gu1[31] + Q11[50]*Gu1[34] + Q11[51]*Gu1[37] + Gu2[10];
Gu3[11] = + Q11[39]*Gu1[2] + Q11[40]*Gu1[5] + Q11[41]*Gu1[8] + Q11[42]*Gu1[11] + Q11[43]*Gu1[14] + Q11[44]*Gu1[17] + Q11[45]*Gu1[20] + Q11[46]*Gu1[23] + Q11[47]*Gu1[26] + Q11[48]*Gu1[29] + Q11[49]*Gu1[32] + Q11[50]*Gu1[35] + Q11[51]*Gu1[38] + Gu2[11];
Gu3[12] = + Q11[52]*Gu1[0] + Q11[53]*Gu1[3] + Q11[54]*Gu1[6] + Q11[55]*Gu1[9] + Q11[56]*Gu1[12] + Q11[57]*Gu1[15] + Q11[58]*Gu1[18] + Q11[59]*Gu1[21] + Q11[60]*Gu1[24] + Q11[61]*Gu1[27] + Q11[62]*Gu1[30] + Q11[63]*Gu1[33] + Q11[64]*Gu1[36] + Gu2[12];
Gu3[13] = + Q11[52]*Gu1[1] + Q11[53]*Gu1[4] + Q11[54]*Gu1[7] + Q11[55]*Gu1[10] + Q11[56]*Gu1[13] + Q11[57]*Gu1[16] + Q11[58]*Gu1[19] + Q11[59]*Gu1[22] + Q11[60]*Gu1[25] + Q11[61]*Gu1[28] + Q11[62]*Gu1[31] + Q11[63]*Gu1[34] + Q11[64]*Gu1[37] + Gu2[13];
Gu3[14] = + Q11[52]*Gu1[2] + Q11[53]*Gu1[5] + Q11[54]*Gu1[8] + Q11[55]*Gu1[11] + Q11[56]*Gu1[14] + Q11[57]*Gu1[17] + Q11[58]*Gu1[20] + Q11[59]*Gu1[23] + Q11[60]*Gu1[26] + Q11[61]*Gu1[29] + Q11[62]*Gu1[32] + Q11[63]*Gu1[35] + Q11[64]*Gu1[38] + Gu2[14];
Gu3[15] = + Q11[65]*Gu1[0] + Q11[66]*Gu1[3] + Q11[67]*Gu1[6] + Q11[68]*Gu1[9] + Q11[69]*Gu1[12] + Q11[70]*Gu1[15] + Q11[71]*Gu1[18] + Q11[72]*Gu1[21] + Q11[73]*Gu1[24] + Q11[74]*Gu1[27] + Q11[75]*Gu1[30] + Q11[76]*Gu1[33] + Q11[77]*Gu1[36] + Gu2[15];
Gu3[16] = + Q11[65]*Gu1[1] + Q11[66]*Gu1[4] + Q11[67]*Gu1[7] + Q11[68]*Gu1[10] + Q11[69]*Gu1[13] + Q11[70]*Gu1[16] + Q11[71]*Gu1[19] + Q11[72]*Gu1[22] + Q11[73]*Gu1[25] + Q11[74]*Gu1[28] + Q11[75]*Gu1[31] + Q11[76]*Gu1[34] + Q11[77]*Gu1[37] + Gu2[16];
Gu3[17] = + Q11[65]*Gu1[2] + Q11[66]*Gu1[5] + Q11[67]*Gu1[8] + Q11[68]*Gu1[11] + Q11[69]*Gu1[14] + Q11[70]*Gu1[17] + Q11[71]*Gu1[20] + Q11[72]*Gu1[23] + Q11[73]*Gu1[26] + Q11[74]*Gu1[29] + Q11[75]*Gu1[32] + Q11[76]*Gu1[35] + Q11[77]*Gu1[38] + Gu2[17];
Gu3[18] = + Q11[78]*Gu1[0] + Q11[79]*Gu1[3] + Q11[80]*Gu1[6] + Q11[81]*Gu1[9] + Q11[82]*Gu1[12] + Q11[83]*Gu1[15] + Q11[84]*Gu1[18] + Q11[85]*Gu1[21] + Q11[86]*Gu1[24] + Q11[87]*Gu1[27] + Q11[88]*Gu1[30] + Q11[89]*Gu1[33] + Q11[90]*Gu1[36] + Gu2[18];
Gu3[19] = + Q11[78]*Gu1[1] + Q11[79]*Gu1[4] + Q11[80]*Gu1[7] + Q11[81]*Gu1[10] + Q11[82]*Gu1[13] + Q11[83]*Gu1[16] + Q11[84]*Gu1[19] + Q11[85]*Gu1[22] + Q11[86]*Gu1[25] + Q11[87]*Gu1[28] + Q11[88]*Gu1[31] + Q11[89]*Gu1[34] + Q11[90]*Gu1[37] + Gu2[19];
Gu3[20] = + Q11[78]*Gu1[2] + Q11[79]*Gu1[5] + Q11[80]*Gu1[8] + Q11[81]*Gu1[11] + Q11[82]*Gu1[14] + Q11[83]*Gu1[17] + Q11[84]*Gu1[20] + Q11[85]*Gu1[23] + Q11[86]*Gu1[26] + Q11[87]*Gu1[29] + Q11[88]*Gu1[32] + Q11[89]*Gu1[35] + Q11[90]*Gu1[38] + Gu2[20];
Gu3[21] = + Q11[91]*Gu1[0] + Q11[92]*Gu1[3] + Q11[93]*Gu1[6] + Q11[94]*Gu1[9] + Q11[95]*Gu1[12] + Q11[96]*Gu1[15] + Q11[97]*Gu1[18] + Q11[98]*Gu1[21] + Q11[99]*Gu1[24] + Q11[100]*Gu1[27] + Q11[101]*Gu1[30] + Q11[102]*Gu1[33] + Q11[103]*Gu1[36] + Gu2[21];
Gu3[22] = + Q11[91]*Gu1[1] + Q11[92]*Gu1[4] + Q11[93]*Gu1[7] + Q11[94]*Gu1[10] + Q11[95]*Gu1[13] + Q11[96]*Gu1[16] + Q11[97]*Gu1[19] + Q11[98]*Gu1[22] + Q11[99]*Gu1[25] + Q11[100]*Gu1[28] + Q11[101]*Gu1[31] + Q11[102]*Gu1[34] + Q11[103]*Gu1[37] + Gu2[22];
Gu3[23] = + Q11[91]*Gu1[2] + Q11[92]*Gu1[5] + Q11[93]*Gu1[8] + Q11[94]*Gu1[11] + Q11[95]*Gu1[14] + Q11[96]*Gu1[17] + Q11[97]*Gu1[20] + Q11[98]*Gu1[23] + Q11[99]*Gu1[26] + Q11[100]*Gu1[29] + Q11[101]*Gu1[32] + Q11[102]*Gu1[35] + Q11[103]*Gu1[38] + Gu2[23];
Gu3[24] = + Q11[104]*Gu1[0] + Q11[105]*Gu1[3] + Q11[106]*Gu1[6] + Q11[107]*Gu1[9] + Q11[108]*Gu1[12] + Q11[109]*Gu1[15] + Q11[110]*Gu1[18] + Q11[111]*Gu1[21] + Q11[112]*Gu1[24] + Q11[113]*Gu1[27] + Q11[114]*Gu1[30] + Q11[115]*Gu1[33] + Q11[116]*Gu1[36] + Gu2[24];
Gu3[25] = + Q11[104]*Gu1[1] + Q11[105]*Gu1[4] + Q11[106]*Gu1[7] + Q11[107]*Gu1[10] + Q11[108]*Gu1[13] + Q11[109]*Gu1[16] + Q11[110]*Gu1[19] + Q11[111]*Gu1[22] + Q11[112]*Gu1[25] + Q11[113]*Gu1[28] + Q11[114]*Gu1[31] + Q11[115]*Gu1[34] + Q11[116]*Gu1[37] + Gu2[25];
Gu3[26] = + Q11[104]*Gu1[2] + Q11[105]*Gu1[5] + Q11[106]*Gu1[8] + Q11[107]*Gu1[11] + Q11[108]*Gu1[14] + Q11[109]*Gu1[17] + Q11[110]*Gu1[20] + Q11[111]*Gu1[23] + Q11[112]*Gu1[26] + Q11[113]*Gu1[29] + Q11[114]*Gu1[32] + Q11[115]*Gu1[35] + Q11[116]*Gu1[38] + Gu2[26];
Gu3[27] = + Q11[117]*Gu1[0] + Q11[118]*Gu1[3] + Q11[119]*Gu1[6] + Q11[120]*Gu1[9] + Q11[121]*Gu1[12] + Q11[122]*Gu1[15] + Q11[123]*Gu1[18] + Q11[124]*Gu1[21] + Q11[125]*Gu1[24] + Q11[126]*Gu1[27] + Q11[127]*Gu1[30] + Q11[128]*Gu1[33] + Q11[129]*Gu1[36] + Gu2[27];
Gu3[28] = + Q11[117]*Gu1[1] + Q11[118]*Gu1[4] + Q11[119]*Gu1[7] + Q11[120]*Gu1[10] + Q11[121]*Gu1[13] + Q11[122]*Gu1[16] + Q11[123]*Gu1[19] + Q11[124]*Gu1[22] + Q11[125]*Gu1[25] + Q11[126]*Gu1[28] + Q11[127]*Gu1[31] + Q11[128]*Gu1[34] + Q11[129]*Gu1[37] + Gu2[28];
Gu3[29] = + Q11[117]*Gu1[2] + Q11[118]*Gu1[5] + Q11[119]*Gu1[8] + Q11[120]*Gu1[11] + Q11[121]*Gu1[14] + Q11[122]*Gu1[17] + Q11[123]*Gu1[20] + Q11[124]*Gu1[23] + Q11[125]*Gu1[26] + Q11[126]*Gu1[29] + Q11[127]*Gu1[32] + Q11[128]*Gu1[35] + Q11[129]*Gu1[38] + Gu2[29];
Gu3[30] = + Q11[130]*Gu1[0] + Q11[131]*Gu1[3] + Q11[132]*Gu1[6] + Q11[133]*Gu1[9] + Q11[134]*Gu1[12] + Q11[135]*Gu1[15] + Q11[136]*Gu1[18] + Q11[137]*Gu1[21] + Q11[138]*Gu1[24] + Q11[139]*Gu1[27] + Q11[140]*Gu1[30] + Q11[141]*Gu1[33] + Q11[142]*Gu1[36] + Gu2[30];
Gu3[31] = + Q11[130]*Gu1[1] + Q11[131]*Gu1[4] + Q11[132]*Gu1[7] + Q11[133]*Gu1[10] + Q11[134]*Gu1[13] + Q11[135]*Gu1[16] + Q11[136]*Gu1[19] + Q11[137]*Gu1[22] + Q11[138]*Gu1[25] + Q11[139]*Gu1[28] + Q11[140]*Gu1[31] + Q11[141]*Gu1[34] + Q11[142]*Gu1[37] + Gu2[31];
Gu3[32] = + Q11[130]*Gu1[2] + Q11[131]*Gu1[5] + Q11[132]*Gu1[8] + Q11[133]*Gu1[11] + Q11[134]*Gu1[14] + Q11[135]*Gu1[17] + Q11[136]*Gu1[20] + Q11[137]*Gu1[23] + Q11[138]*Gu1[26] + Q11[139]*Gu1[29] + Q11[140]*Gu1[32] + Q11[141]*Gu1[35] + Q11[142]*Gu1[38] + Gu2[32];
Gu3[33] = + Q11[143]*Gu1[0] + Q11[144]*Gu1[3] + Q11[145]*Gu1[6] + Q11[146]*Gu1[9] + Q11[147]*Gu1[12] + Q11[148]*Gu1[15] + Q11[149]*Gu1[18] + Q11[150]*Gu1[21] + Q11[151]*Gu1[24] + Q11[152]*Gu1[27] + Q11[153]*Gu1[30] + Q11[154]*Gu1[33] + Q11[155]*Gu1[36] + Gu2[33];
Gu3[34] = + Q11[143]*Gu1[1] + Q11[144]*Gu1[4] + Q11[145]*Gu1[7] + Q11[146]*Gu1[10] + Q11[147]*Gu1[13] + Q11[148]*Gu1[16] + Q11[149]*Gu1[19] + Q11[150]*Gu1[22] + Q11[151]*Gu1[25] + Q11[152]*Gu1[28] + Q11[153]*Gu1[31] + Q11[154]*Gu1[34] + Q11[155]*Gu1[37] + Gu2[34];
Gu3[35] = + Q11[143]*Gu1[2] + Q11[144]*Gu1[5] + Q11[145]*Gu1[8] + Q11[146]*Gu1[11] + Q11[147]*Gu1[14] + Q11[148]*Gu1[17] + Q11[149]*Gu1[20] + Q11[150]*Gu1[23] + Q11[151]*Gu1[26] + Q11[152]*Gu1[29] + Q11[153]*Gu1[32] + Q11[154]*Gu1[35] + Q11[155]*Gu1[38] + Gu2[35];
Gu3[36] = + Q11[156]*Gu1[0] + Q11[157]*Gu1[3] + Q11[158]*Gu1[6] + Q11[159]*Gu1[9] + Q11[160]*Gu1[12] + Q11[161]*Gu1[15] + Q11[162]*Gu1[18] + Q11[163]*Gu1[21] + Q11[164]*Gu1[24] + Q11[165]*Gu1[27] + Q11[166]*Gu1[30] + Q11[167]*Gu1[33] + Q11[168]*Gu1[36] + Gu2[36];
Gu3[37] = + Q11[156]*Gu1[1] + Q11[157]*Gu1[4] + Q11[158]*Gu1[7] + Q11[159]*Gu1[10] + Q11[160]*Gu1[13] + Q11[161]*Gu1[16] + Q11[162]*Gu1[19] + Q11[163]*Gu1[22] + Q11[164]*Gu1[25] + Q11[165]*Gu1[28] + Q11[166]*Gu1[31] + Q11[167]*Gu1[34] + Q11[168]*Gu1[37] + Gu2[37];
Gu3[38] = + Q11[156]*Gu1[2] + Q11[157]*Gu1[5] + Q11[158]*Gu1[8] + Q11[159]*Gu1[11] + Q11[160]*Gu1[14] + Q11[161]*Gu1[17] + Q11[162]*Gu1[20] + Q11[163]*Gu1[23] + Q11[164]*Gu1[26] + Q11[165]*Gu1[29] + Q11[166]*Gu1[32] + Q11[167]*Gu1[35] + Q11[168]*Gu1[38] + Gu2[38];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[13]*w11[1] + Gx1[26]*w11[2] + Gx1[39]*w11[3] + Gx1[52]*w11[4] + Gx1[65]*w11[5] + Gx1[78]*w11[6] + Gx1[91]*w11[7] + Gx1[104]*w11[8] + Gx1[117]*w11[9] + Gx1[130]*w11[10] + Gx1[143]*w11[11] + Gx1[156]*w11[12] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[14]*w11[1] + Gx1[27]*w11[2] + Gx1[40]*w11[3] + Gx1[53]*w11[4] + Gx1[66]*w11[5] + Gx1[79]*w11[6] + Gx1[92]*w11[7] + Gx1[105]*w11[8] + Gx1[118]*w11[9] + Gx1[131]*w11[10] + Gx1[144]*w11[11] + Gx1[157]*w11[12] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[15]*w11[1] + Gx1[28]*w11[2] + Gx1[41]*w11[3] + Gx1[54]*w11[4] + Gx1[67]*w11[5] + Gx1[80]*w11[6] + Gx1[93]*w11[7] + Gx1[106]*w11[8] + Gx1[119]*w11[9] + Gx1[132]*w11[10] + Gx1[145]*w11[11] + Gx1[158]*w11[12] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[16]*w11[1] + Gx1[29]*w11[2] + Gx1[42]*w11[3] + Gx1[55]*w11[4] + Gx1[68]*w11[5] + Gx1[81]*w11[6] + Gx1[94]*w11[7] + Gx1[107]*w11[8] + Gx1[120]*w11[9] + Gx1[133]*w11[10] + Gx1[146]*w11[11] + Gx1[159]*w11[12] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[17]*w11[1] + Gx1[30]*w11[2] + Gx1[43]*w11[3] + Gx1[56]*w11[4] + Gx1[69]*w11[5] + Gx1[82]*w11[6] + Gx1[95]*w11[7] + Gx1[108]*w11[8] + Gx1[121]*w11[9] + Gx1[134]*w11[10] + Gx1[147]*w11[11] + Gx1[160]*w11[12] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[18]*w11[1] + Gx1[31]*w11[2] + Gx1[44]*w11[3] + Gx1[57]*w11[4] + Gx1[70]*w11[5] + Gx1[83]*w11[6] + Gx1[96]*w11[7] + Gx1[109]*w11[8] + Gx1[122]*w11[9] + Gx1[135]*w11[10] + Gx1[148]*w11[11] + Gx1[161]*w11[12] + w12[5];
w13[6] = + Gx1[6]*w11[0] + Gx1[19]*w11[1] + Gx1[32]*w11[2] + Gx1[45]*w11[3] + Gx1[58]*w11[4] + Gx1[71]*w11[5] + Gx1[84]*w11[6] + Gx1[97]*w11[7] + Gx1[110]*w11[8] + Gx1[123]*w11[9] + Gx1[136]*w11[10] + Gx1[149]*w11[11] + Gx1[162]*w11[12] + w12[6];
w13[7] = + Gx1[7]*w11[0] + Gx1[20]*w11[1] + Gx1[33]*w11[2] + Gx1[46]*w11[3] + Gx1[59]*w11[4] + Gx1[72]*w11[5] + Gx1[85]*w11[6] + Gx1[98]*w11[7] + Gx1[111]*w11[8] + Gx1[124]*w11[9] + Gx1[137]*w11[10] + Gx1[150]*w11[11] + Gx1[163]*w11[12] + w12[7];
w13[8] = + Gx1[8]*w11[0] + Gx1[21]*w11[1] + Gx1[34]*w11[2] + Gx1[47]*w11[3] + Gx1[60]*w11[4] + Gx1[73]*w11[5] + Gx1[86]*w11[6] + Gx1[99]*w11[7] + Gx1[112]*w11[8] + Gx1[125]*w11[9] + Gx1[138]*w11[10] + Gx1[151]*w11[11] + Gx1[164]*w11[12] + w12[8];
w13[9] = + Gx1[9]*w11[0] + Gx1[22]*w11[1] + Gx1[35]*w11[2] + Gx1[48]*w11[3] + Gx1[61]*w11[4] + Gx1[74]*w11[5] + Gx1[87]*w11[6] + Gx1[100]*w11[7] + Gx1[113]*w11[8] + Gx1[126]*w11[9] + Gx1[139]*w11[10] + Gx1[152]*w11[11] + Gx1[165]*w11[12] + w12[9];
w13[10] = + Gx1[10]*w11[0] + Gx1[23]*w11[1] + Gx1[36]*w11[2] + Gx1[49]*w11[3] + Gx1[62]*w11[4] + Gx1[75]*w11[5] + Gx1[88]*w11[6] + Gx1[101]*w11[7] + Gx1[114]*w11[8] + Gx1[127]*w11[9] + Gx1[140]*w11[10] + Gx1[153]*w11[11] + Gx1[166]*w11[12] + w12[10];
w13[11] = + Gx1[11]*w11[0] + Gx1[24]*w11[1] + Gx1[37]*w11[2] + Gx1[50]*w11[3] + Gx1[63]*w11[4] + Gx1[76]*w11[5] + Gx1[89]*w11[6] + Gx1[102]*w11[7] + Gx1[115]*w11[8] + Gx1[128]*w11[9] + Gx1[141]*w11[10] + Gx1[154]*w11[11] + Gx1[167]*w11[12] + w12[11];
w13[12] = + Gx1[12]*w11[0] + Gx1[25]*w11[1] + Gx1[38]*w11[2] + Gx1[51]*w11[3] + Gx1[64]*w11[4] + Gx1[77]*w11[5] + Gx1[90]*w11[6] + Gx1[103]*w11[7] + Gx1[116]*w11[8] + Gx1[129]*w11[9] + Gx1[142]*w11[10] + Gx1[155]*w11[11] + Gx1[168]*w11[12] + w12[12];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[3]*w11[1] + Gu1[6]*w11[2] + Gu1[9]*w11[3] + Gu1[12]*w11[4] + Gu1[15]*w11[5] + Gu1[18]*w11[6] + Gu1[21]*w11[7] + Gu1[24]*w11[8] + Gu1[27]*w11[9] + Gu1[30]*w11[10] + Gu1[33]*w11[11] + Gu1[36]*w11[12];
U1[1] += + Gu1[1]*w11[0] + Gu1[4]*w11[1] + Gu1[7]*w11[2] + Gu1[10]*w11[3] + Gu1[13]*w11[4] + Gu1[16]*w11[5] + Gu1[19]*w11[6] + Gu1[22]*w11[7] + Gu1[25]*w11[8] + Gu1[28]*w11[9] + Gu1[31]*w11[10] + Gu1[34]*w11[11] + Gu1[37]*w11[12];
U1[2] += + Gu1[2]*w11[0] + Gu1[5]*w11[1] + Gu1[8]*w11[2] + Gu1[11]*w11[3] + Gu1[14]*w11[4] + Gu1[17]*w11[5] + Gu1[20]*w11[6] + Gu1[23]*w11[7] + Gu1[26]*w11[8] + Gu1[29]*w11[9] + Gu1[32]*w11[10] + Gu1[35]*w11[11] + Gu1[38]*w11[12];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + Q11[5]*w11[5] + Q11[6]*w11[6] + Q11[7]*w11[7] + Q11[8]*w11[8] + Q11[9]*w11[9] + Q11[10]*w11[10] + Q11[11]*w11[11] + Q11[12]*w11[12] + w12[0];
w13[1] = + Q11[13]*w11[0] + Q11[14]*w11[1] + Q11[15]*w11[2] + Q11[16]*w11[3] + Q11[17]*w11[4] + Q11[18]*w11[5] + Q11[19]*w11[6] + Q11[20]*w11[7] + Q11[21]*w11[8] + Q11[22]*w11[9] + Q11[23]*w11[10] + Q11[24]*w11[11] + Q11[25]*w11[12] + w12[1];
w13[2] = + Q11[26]*w11[0] + Q11[27]*w11[1] + Q11[28]*w11[2] + Q11[29]*w11[3] + Q11[30]*w11[4] + Q11[31]*w11[5] + Q11[32]*w11[6] + Q11[33]*w11[7] + Q11[34]*w11[8] + Q11[35]*w11[9] + Q11[36]*w11[10] + Q11[37]*w11[11] + Q11[38]*w11[12] + w12[2];
w13[3] = + Q11[39]*w11[0] + Q11[40]*w11[1] + Q11[41]*w11[2] + Q11[42]*w11[3] + Q11[43]*w11[4] + Q11[44]*w11[5] + Q11[45]*w11[6] + Q11[46]*w11[7] + Q11[47]*w11[8] + Q11[48]*w11[9] + Q11[49]*w11[10] + Q11[50]*w11[11] + Q11[51]*w11[12] + w12[3];
w13[4] = + Q11[52]*w11[0] + Q11[53]*w11[1] + Q11[54]*w11[2] + Q11[55]*w11[3] + Q11[56]*w11[4] + Q11[57]*w11[5] + Q11[58]*w11[6] + Q11[59]*w11[7] + Q11[60]*w11[8] + Q11[61]*w11[9] + Q11[62]*w11[10] + Q11[63]*w11[11] + Q11[64]*w11[12] + w12[4];
w13[5] = + Q11[65]*w11[0] + Q11[66]*w11[1] + Q11[67]*w11[2] + Q11[68]*w11[3] + Q11[69]*w11[4] + Q11[70]*w11[5] + Q11[71]*w11[6] + Q11[72]*w11[7] + Q11[73]*w11[8] + Q11[74]*w11[9] + Q11[75]*w11[10] + Q11[76]*w11[11] + Q11[77]*w11[12] + w12[5];
w13[6] = + Q11[78]*w11[0] + Q11[79]*w11[1] + Q11[80]*w11[2] + Q11[81]*w11[3] + Q11[82]*w11[4] + Q11[83]*w11[5] + Q11[84]*w11[6] + Q11[85]*w11[7] + Q11[86]*w11[8] + Q11[87]*w11[9] + Q11[88]*w11[10] + Q11[89]*w11[11] + Q11[90]*w11[12] + w12[6];
w13[7] = + Q11[91]*w11[0] + Q11[92]*w11[1] + Q11[93]*w11[2] + Q11[94]*w11[3] + Q11[95]*w11[4] + Q11[96]*w11[5] + Q11[97]*w11[6] + Q11[98]*w11[7] + Q11[99]*w11[8] + Q11[100]*w11[9] + Q11[101]*w11[10] + Q11[102]*w11[11] + Q11[103]*w11[12] + w12[7];
w13[8] = + Q11[104]*w11[0] + Q11[105]*w11[1] + Q11[106]*w11[2] + Q11[107]*w11[3] + Q11[108]*w11[4] + Q11[109]*w11[5] + Q11[110]*w11[6] + Q11[111]*w11[7] + Q11[112]*w11[8] + Q11[113]*w11[9] + Q11[114]*w11[10] + Q11[115]*w11[11] + Q11[116]*w11[12] + w12[8];
w13[9] = + Q11[117]*w11[0] + Q11[118]*w11[1] + Q11[119]*w11[2] + Q11[120]*w11[3] + Q11[121]*w11[4] + Q11[122]*w11[5] + Q11[123]*w11[6] + Q11[124]*w11[7] + Q11[125]*w11[8] + Q11[126]*w11[9] + Q11[127]*w11[10] + Q11[128]*w11[11] + Q11[129]*w11[12] + w12[9];
w13[10] = + Q11[130]*w11[0] + Q11[131]*w11[1] + Q11[132]*w11[2] + Q11[133]*w11[3] + Q11[134]*w11[4] + Q11[135]*w11[5] + Q11[136]*w11[6] + Q11[137]*w11[7] + Q11[138]*w11[8] + Q11[139]*w11[9] + Q11[140]*w11[10] + Q11[141]*w11[11] + Q11[142]*w11[12] + w12[10];
w13[11] = + Q11[143]*w11[0] + Q11[144]*w11[1] + Q11[145]*w11[2] + Q11[146]*w11[3] + Q11[147]*w11[4] + Q11[148]*w11[5] + Q11[149]*w11[6] + Q11[150]*w11[7] + Q11[151]*w11[8] + Q11[152]*w11[9] + Q11[153]*w11[10] + Q11[154]*w11[11] + Q11[155]*w11[12] + w12[11];
w13[12] = + Q11[156]*w11[0] + Q11[157]*w11[1] + Q11[158]*w11[2] + Q11[159]*w11[3] + Q11[160]*w11[4] + Q11[161]*w11[5] + Q11[162]*w11[6] + Q11[163]*w11[7] + Q11[164]*w11[8] + Q11[165]*w11[9] + Q11[166]*w11[10] + Q11[167]*w11[11] + Q11[168]*w11[12] + w12[12];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8] + Gx1[9]*w11[9] + Gx1[10]*w11[10] + Gx1[11]*w11[11] + Gx1[12]*w11[12];
w12[1] += + Gx1[13]*w11[0] + Gx1[14]*w11[1] + Gx1[15]*w11[2] + Gx1[16]*w11[3] + Gx1[17]*w11[4] + Gx1[18]*w11[5] + Gx1[19]*w11[6] + Gx1[20]*w11[7] + Gx1[21]*w11[8] + Gx1[22]*w11[9] + Gx1[23]*w11[10] + Gx1[24]*w11[11] + Gx1[25]*w11[12];
w12[2] += + Gx1[26]*w11[0] + Gx1[27]*w11[1] + Gx1[28]*w11[2] + Gx1[29]*w11[3] + Gx1[30]*w11[4] + Gx1[31]*w11[5] + Gx1[32]*w11[6] + Gx1[33]*w11[7] + Gx1[34]*w11[8] + Gx1[35]*w11[9] + Gx1[36]*w11[10] + Gx1[37]*w11[11] + Gx1[38]*w11[12];
w12[3] += + Gx1[39]*w11[0] + Gx1[40]*w11[1] + Gx1[41]*w11[2] + Gx1[42]*w11[3] + Gx1[43]*w11[4] + Gx1[44]*w11[5] + Gx1[45]*w11[6] + Gx1[46]*w11[7] + Gx1[47]*w11[8] + Gx1[48]*w11[9] + Gx1[49]*w11[10] + Gx1[50]*w11[11] + Gx1[51]*w11[12];
w12[4] += + Gx1[52]*w11[0] + Gx1[53]*w11[1] + Gx1[54]*w11[2] + Gx1[55]*w11[3] + Gx1[56]*w11[4] + Gx1[57]*w11[5] + Gx1[58]*w11[6] + Gx1[59]*w11[7] + Gx1[60]*w11[8] + Gx1[61]*w11[9] + Gx1[62]*w11[10] + Gx1[63]*w11[11] + Gx1[64]*w11[12];
w12[5] += + Gx1[65]*w11[0] + Gx1[66]*w11[1] + Gx1[67]*w11[2] + Gx1[68]*w11[3] + Gx1[69]*w11[4] + Gx1[70]*w11[5] + Gx1[71]*w11[6] + Gx1[72]*w11[7] + Gx1[73]*w11[8] + Gx1[74]*w11[9] + Gx1[75]*w11[10] + Gx1[76]*w11[11] + Gx1[77]*w11[12];
w12[6] += + Gx1[78]*w11[0] + Gx1[79]*w11[1] + Gx1[80]*w11[2] + Gx1[81]*w11[3] + Gx1[82]*w11[4] + Gx1[83]*w11[5] + Gx1[84]*w11[6] + Gx1[85]*w11[7] + Gx1[86]*w11[8] + Gx1[87]*w11[9] + Gx1[88]*w11[10] + Gx1[89]*w11[11] + Gx1[90]*w11[12];
w12[7] += + Gx1[91]*w11[0] + Gx1[92]*w11[1] + Gx1[93]*w11[2] + Gx1[94]*w11[3] + Gx1[95]*w11[4] + Gx1[96]*w11[5] + Gx1[97]*w11[6] + Gx1[98]*w11[7] + Gx1[99]*w11[8] + Gx1[100]*w11[9] + Gx1[101]*w11[10] + Gx1[102]*w11[11] + Gx1[103]*w11[12];
w12[8] += + Gx1[104]*w11[0] + Gx1[105]*w11[1] + Gx1[106]*w11[2] + Gx1[107]*w11[3] + Gx1[108]*w11[4] + Gx1[109]*w11[5] + Gx1[110]*w11[6] + Gx1[111]*w11[7] + Gx1[112]*w11[8] + Gx1[113]*w11[9] + Gx1[114]*w11[10] + Gx1[115]*w11[11] + Gx1[116]*w11[12];
w12[9] += + Gx1[117]*w11[0] + Gx1[118]*w11[1] + Gx1[119]*w11[2] + Gx1[120]*w11[3] + Gx1[121]*w11[4] + Gx1[122]*w11[5] + Gx1[123]*w11[6] + Gx1[124]*w11[7] + Gx1[125]*w11[8] + Gx1[126]*w11[9] + Gx1[127]*w11[10] + Gx1[128]*w11[11] + Gx1[129]*w11[12];
w12[10] += + Gx1[130]*w11[0] + Gx1[131]*w11[1] + Gx1[132]*w11[2] + Gx1[133]*w11[3] + Gx1[134]*w11[4] + Gx1[135]*w11[5] + Gx1[136]*w11[6] + Gx1[137]*w11[7] + Gx1[138]*w11[8] + Gx1[139]*w11[9] + Gx1[140]*w11[10] + Gx1[141]*w11[11] + Gx1[142]*w11[12];
w12[11] += + Gx1[143]*w11[0] + Gx1[144]*w11[1] + Gx1[145]*w11[2] + Gx1[146]*w11[3] + Gx1[147]*w11[4] + Gx1[148]*w11[5] + Gx1[149]*w11[6] + Gx1[150]*w11[7] + Gx1[151]*w11[8] + Gx1[152]*w11[9] + Gx1[153]*w11[10] + Gx1[154]*w11[11] + Gx1[155]*w11[12];
w12[12] += + Gx1[156]*w11[0] + Gx1[157]*w11[1] + Gx1[158]*w11[2] + Gx1[159]*w11[3] + Gx1[160]*w11[4] + Gx1[161]*w11[5] + Gx1[162]*w11[6] + Gx1[163]*w11[7] + Gx1[164]*w11[8] + Gx1[165]*w11[9] + Gx1[166]*w11[10] + Gx1[167]*w11[11] + Gx1[168]*w11[12];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8] + Gx1[9]*w11[9] + Gx1[10]*w11[10] + Gx1[11]*w11[11] + Gx1[12]*w11[12];
w12[1] += + Gx1[13]*w11[0] + Gx1[14]*w11[1] + Gx1[15]*w11[2] + Gx1[16]*w11[3] + Gx1[17]*w11[4] + Gx1[18]*w11[5] + Gx1[19]*w11[6] + Gx1[20]*w11[7] + Gx1[21]*w11[8] + Gx1[22]*w11[9] + Gx1[23]*w11[10] + Gx1[24]*w11[11] + Gx1[25]*w11[12];
w12[2] += + Gx1[26]*w11[0] + Gx1[27]*w11[1] + Gx1[28]*w11[2] + Gx1[29]*w11[3] + Gx1[30]*w11[4] + Gx1[31]*w11[5] + Gx1[32]*w11[6] + Gx1[33]*w11[7] + Gx1[34]*w11[8] + Gx1[35]*w11[9] + Gx1[36]*w11[10] + Gx1[37]*w11[11] + Gx1[38]*w11[12];
w12[3] += + Gx1[39]*w11[0] + Gx1[40]*w11[1] + Gx1[41]*w11[2] + Gx1[42]*w11[3] + Gx1[43]*w11[4] + Gx1[44]*w11[5] + Gx1[45]*w11[6] + Gx1[46]*w11[7] + Gx1[47]*w11[8] + Gx1[48]*w11[9] + Gx1[49]*w11[10] + Gx1[50]*w11[11] + Gx1[51]*w11[12];
w12[4] += + Gx1[52]*w11[0] + Gx1[53]*w11[1] + Gx1[54]*w11[2] + Gx1[55]*w11[3] + Gx1[56]*w11[4] + Gx1[57]*w11[5] + Gx1[58]*w11[6] + Gx1[59]*w11[7] + Gx1[60]*w11[8] + Gx1[61]*w11[9] + Gx1[62]*w11[10] + Gx1[63]*w11[11] + Gx1[64]*w11[12];
w12[5] += + Gx1[65]*w11[0] + Gx1[66]*w11[1] + Gx1[67]*w11[2] + Gx1[68]*w11[3] + Gx1[69]*w11[4] + Gx1[70]*w11[5] + Gx1[71]*w11[6] + Gx1[72]*w11[7] + Gx1[73]*w11[8] + Gx1[74]*w11[9] + Gx1[75]*w11[10] + Gx1[76]*w11[11] + Gx1[77]*w11[12];
w12[6] += + Gx1[78]*w11[0] + Gx1[79]*w11[1] + Gx1[80]*w11[2] + Gx1[81]*w11[3] + Gx1[82]*w11[4] + Gx1[83]*w11[5] + Gx1[84]*w11[6] + Gx1[85]*w11[7] + Gx1[86]*w11[8] + Gx1[87]*w11[9] + Gx1[88]*w11[10] + Gx1[89]*w11[11] + Gx1[90]*w11[12];
w12[7] += + Gx1[91]*w11[0] + Gx1[92]*w11[1] + Gx1[93]*w11[2] + Gx1[94]*w11[3] + Gx1[95]*w11[4] + Gx1[96]*w11[5] + Gx1[97]*w11[6] + Gx1[98]*w11[7] + Gx1[99]*w11[8] + Gx1[100]*w11[9] + Gx1[101]*w11[10] + Gx1[102]*w11[11] + Gx1[103]*w11[12];
w12[8] += + Gx1[104]*w11[0] + Gx1[105]*w11[1] + Gx1[106]*w11[2] + Gx1[107]*w11[3] + Gx1[108]*w11[4] + Gx1[109]*w11[5] + Gx1[110]*w11[6] + Gx1[111]*w11[7] + Gx1[112]*w11[8] + Gx1[113]*w11[9] + Gx1[114]*w11[10] + Gx1[115]*w11[11] + Gx1[116]*w11[12];
w12[9] += + Gx1[117]*w11[0] + Gx1[118]*w11[1] + Gx1[119]*w11[2] + Gx1[120]*w11[3] + Gx1[121]*w11[4] + Gx1[122]*w11[5] + Gx1[123]*w11[6] + Gx1[124]*w11[7] + Gx1[125]*w11[8] + Gx1[126]*w11[9] + Gx1[127]*w11[10] + Gx1[128]*w11[11] + Gx1[129]*w11[12];
w12[10] += + Gx1[130]*w11[0] + Gx1[131]*w11[1] + Gx1[132]*w11[2] + Gx1[133]*w11[3] + Gx1[134]*w11[4] + Gx1[135]*w11[5] + Gx1[136]*w11[6] + Gx1[137]*w11[7] + Gx1[138]*w11[8] + Gx1[139]*w11[9] + Gx1[140]*w11[10] + Gx1[141]*w11[11] + Gx1[142]*w11[12];
w12[11] += + Gx1[143]*w11[0] + Gx1[144]*w11[1] + Gx1[145]*w11[2] + Gx1[146]*w11[3] + Gx1[147]*w11[4] + Gx1[148]*w11[5] + Gx1[149]*w11[6] + Gx1[150]*w11[7] + Gx1[151]*w11[8] + Gx1[152]*w11[9] + Gx1[153]*w11[10] + Gx1[154]*w11[11] + Gx1[155]*w11[12];
w12[12] += + Gx1[156]*w11[0] + Gx1[157]*w11[1] + Gx1[158]*w11[2] + Gx1[159]*w11[3] + Gx1[160]*w11[4] + Gx1[161]*w11[5] + Gx1[162]*w11[6] + Gx1[163]*w11[7] + Gx1[164]*w11[8] + Gx1[165]*w11[9] + Gx1[166]*w11[10] + Gx1[167]*w11[11] + Gx1[168]*w11[12];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2];
w12[1] += + Gu1[3]*U1[0] + Gu1[4]*U1[1] + Gu1[5]*U1[2];
w12[2] += + Gu1[6]*U1[0] + Gu1[7]*U1[1] + Gu1[8]*U1[2];
w12[3] += + Gu1[9]*U1[0] + Gu1[10]*U1[1] + Gu1[11]*U1[2];
w12[4] += + Gu1[12]*U1[0] + Gu1[13]*U1[1] + Gu1[14]*U1[2];
w12[5] += + Gu1[15]*U1[0] + Gu1[16]*U1[1] + Gu1[17]*U1[2];
w12[6] += + Gu1[18]*U1[0] + Gu1[19]*U1[1] + Gu1[20]*U1[2];
w12[7] += + Gu1[21]*U1[0] + Gu1[22]*U1[1] + Gu1[23]*U1[2];
w12[8] += + Gu1[24]*U1[0] + Gu1[25]*U1[1] + Gu1[26]*U1[2];
w12[9] += + Gu1[27]*U1[0] + Gu1[28]*U1[1] + Gu1[29]*U1[2];
w12[10] += + Gu1[30]*U1[0] + Gu1[31]*U1[1] + Gu1[32]*U1[2];
w12[11] += + Gu1[33]*U1[0] + Gu1[34]*U1[1] + Gu1[35]*U1[2];
w12[12] += + Gu1[36]*U1[0] + Gu1[37]*U1[1] + Gu1[38]*U1[2];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 450) + (iCol * 3)] = acadoWorkspace.H[(iCol * 450) + (iRow * 3)];
acadoWorkspace.H[(iRow * 450) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 450 + 150) + (iRow * 3)];
acadoWorkspace.H[(iRow * 450) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 450 + 300) + (iRow * 3)];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3)] = acadoWorkspace.H[(iCol * 450) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 450 + 150) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 450 + 300) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3)] = acadoWorkspace.H[(iCol * 450) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 450 + 150) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 450 + 300) + (iRow * 3 + 2)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11] + R2[12]*Dy1[12] + R2[13]*Dy1[13] + R2[14]*Dy1[14] + R2[15]*Dy1[15];
RDy1[1] = + R2[16]*Dy1[0] + R2[17]*Dy1[1] + R2[18]*Dy1[2] + R2[19]*Dy1[3] + R2[20]*Dy1[4] + R2[21]*Dy1[5] + R2[22]*Dy1[6] + R2[23]*Dy1[7] + R2[24]*Dy1[8] + R2[25]*Dy1[9] + R2[26]*Dy1[10] + R2[27]*Dy1[11] + R2[28]*Dy1[12] + R2[29]*Dy1[13] + R2[30]*Dy1[14] + R2[31]*Dy1[15];
RDy1[2] = + R2[32]*Dy1[0] + R2[33]*Dy1[1] + R2[34]*Dy1[2] + R2[35]*Dy1[3] + R2[36]*Dy1[4] + R2[37]*Dy1[5] + R2[38]*Dy1[6] + R2[39]*Dy1[7] + R2[40]*Dy1[8] + R2[41]*Dy1[9] + R2[42]*Dy1[10] + R2[43]*Dy1[11] + R2[44]*Dy1[12] + R2[45]*Dy1[13] + R2[46]*Dy1[14] + R2[47]*Dy1[15];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10] + Q2[11]*Dy1[11] + Q2[12]*Dy1[12] + Q2[13]*Dy1[13] + Q2[14]*Dy1[14] + Q2[15]*Dy1[15];
QDy1[1] = + Q2[16]*Dy1[0] + Q2[17]*Dy1[1] + Q2[18]*Dy1[2] + Q2[19]*Dy1[3] + Q2[20]*Dy1[4] + Q2[21]*Dy1[5] + Q2[22]*Dy1[6] + Q2[23]*Dy1[7] + Q2[24]*Dy1[8] + Q2[25]*Dy1[9] + Q2[26]*Dy1[10] + Q2[27]*Dy1[11] + Q2[28]*Dy1[12] + Q2[29]*Dy1[13] + Q2[30]*Dy1[14] + Q2[31]*Dy1[15];
QDy1[2] = + Q2[32]*Dy1[0] + Q2[33]*Dy1[1] + Q2[34]*Dy1[2] + Q2[35]*Dy1[3] + Q2[36]*Dy1[4] + Q2[37]*Dy1[5] + Q2[38]*Dy1[6] + Q2[39]*Dy1[7] + Q2[40]*Dy1[8] + Q2[41]*Dy1[9] + Q2[42]*Dy1[10] + Q2[43]*Dy1[11] + Q2[44]*Dy1[12] + Q2[45]*Dy1[13] + Q2[46]*Dy1[14] + Q2[47]*Dy1[15];
QDy1[3] = + Q2[48]*Dy1[0] + Q2[49]*Dy1[1] + Q2[50]*Dy1[2] + Q2[51]*Dy1[3] + Q2[52]*Dy1[4] + Q2[53]*Dy1[5] + Q2[54]*Dy1[6] + Q2[55]*Dy1[7] + Q2[56]*Dy1[8] + Q2[57]*Dy1[9] + Q2[58]*Dy1[10] + Q2[59]*Dy1[11] + Q2[60]*Dy1[12] + Q2[61]*Dy1[13] + Q2[62]*Dy1[14] + Q2[63]*Dy1[15];
QDy1[4] = + Q2[64]*Dy1[0] + Q2[65]*Dy1[1] + Q2[66]*Dy1[2] + Q2[67]*Dy1[3] + Q2[68]*Dy1[4] + Q2[69]*Dy1[5] + Q2[70]*Dy1[6] + Q2[71]*Dy1[7] + Q2[72]*Dy1[8] + Q2[73]*Dy1[9] + Q2[74]*Dy1[10] + Q2[75]*Dy1[11] + Q2[76]*Dy1[12] + Q2[77]*Dy1[13] + Q2[78]*Dy1[14] + Q2[79]*Dy1[15];
QDy1[5] = + Q2[80]*Dy1[0] + Q2[81]*Dy1[1] + Q2[82]*Dy1[2] + Q2[83]*Dy1[3] + Q2[84]*Dy1[4] + Q2[85]*Dy1[5] + Q2[86]*Dy1[6] + Q2[87]*Dy1[7] + Q2[88]*Dy1[8] + Q2[89]*Dy1[9] + Q2[90]*Dy1[10] + Q2[91]*Dy1[11] + Q2[92]*Dy1[12] + Q2[93]*Dy1[13] + Q2[94]*Dy1[14] + Q2[95]*Dy1[15];
QDy1[6] = + Q2[96]*Dy1[0] + Q2[97]*Dy1[1] + Q2[98]*Dy1[2] + Q2[99]*Dy1[3] + Q2[100]*Dy1[4] + Q2[101]*Dy1[5] + Q2[102]*Dy1[6] + Q2[103]*Dy1[7] + Q2[104]*Dy1[8] + Q2[105]*Dy1[9] + Q2[106]*Dy1[10] + Q2[107]*Dy1[11] + Q2[108]*Dy1[12] + Q2[109]*Dy1[13] + Q2[110]*Dy1[14] + Q2[111]*Dy1[15];
QDy1[7] = + Q2[112]*Dy1[0] + Q2[113]*Dy1[1] + Q2[114]*Dy1[2] + Q2[115]*Dy1[3] + Q2[116]*Dy1[4] + Q2[117]*Dy1[5] + Q2[118]*Dy1[6] + Q2[119]*Dy1[7] + Q2[120]*Dy1[8] + Q2[121]*Dy1[9] + Q2[122]*Dy1[10] + Q2[123]*Dy1[11] + Q2[124]*Dy1[12] + Q2[125]*Dy1[13] + Q2[126]*Dy1[14] + Q2[127]*Dy1[15];
QDy1[8] = + Q2[128]*Dy1[0] + Q2[129]*Dy1[1] + Q2[130]*Dy1[2] + Q2[131]*Dy1[3] + Q2[132]*Dy1[4] + Q2[133]*Dy1[5] + Q2[134]*Dy1[6] + Q2[135]*Dy1[7] + Q2[136]*Dy1[8] + Q2[137]*Dy1[9] + Q2[138]*Dy1[10] + Q2[139]*Dy1[11] + Q2[140]*Dy1[12] + Q2[141]*Dy1[13] + Q2[142]*Dy1[14] + Q2[143]*Dy1[15];
QDy1[9] = + Q2[144]*Dy1[0] + Q2[145]*Dy1[1] + Q2[146]*Dy1[2] + Q2[147]*Dy1[3] + Q2[148]*Dy1[4] + Q2[149]*Dy1[5] + Q2[150]*Dy1[6] + Q2[151]*Dy1[7] + Q2[152]*Dy1[8] + Q2[153]*Dy1[9] + Q2[154]*Dy1[10] + Q2[155]*Dy1[11] + Q2[156]*Dy1[12] + Q2[157]*Dy1[13] + Q2[158]*Dy1[14] + Q2[159]*Dy1[15];
QDy1[10] = + Q2[160]*Dy1[0] + Q2[161]*Dy1[1] + Q2[162]*Dy1[2] + Q2[163]*Dy1[3] + Q2[164]*Dy1[4] + Q2[165]*Dy1[5] + Q2[166]*Dy1[6] + Q2[167]*Dy1[7] + Q2[168]*Dy1[8] + Q2[169]*Dy1[9] + Q2[170]*Dy1[10] + Q2[171]*Dy1[11] + Q2[172]*Dy1[12] + Q2[173]*Dy1[13] + Q2[174]*Dy1[14] + Q2[175]*Dy1[15];
QDy1[11] = + Q2[176]*Dy1[0] + Q2[177]*Dy1[1] + Q2[178]*Dy1[2] + Q2[179]*Dy1[3] + Q2[180]*Dy1[4] + Q2[181]*Dy1[5] + Q2[182]*Dy1[6] + Q2[183]*Dy1[7] + Q2[184]*Dy1[8] + Q2[185]*Dy1[9] + Q2[186]*Dy1[10] + Q2[187]*Dy1[11] + Q2[188]*Dy1[12] + Q2[189]*Dy1[13] + Q2[190]*Dy1[14] + Q2[191]*Dy1[15];
QDy1[12] = + Q2[192]*Dy1[0] + Q2[193]*Dy1[1] + Q2[194]*Dy1[2] + Q2[195]*Dy1[3] + Q2[196]*Dy1[4] + Q2[197]*Dy1[5] + Q2[198]*Dy1[6] + Q2[199]*Dy1[7] + Q2[200]*Dy1[8] + Q2[201]*Dy1[9] + Q2[202]*Dy1[10] + Q2[203]*Dy1[11] + Q2[204]*Dy1[12] + Q2[205]*Dy1[13] + Q2[206]*Dy1[14] + Q2[207]*Dy1[15];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun2 = 0; lRun2 < 50; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 101)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 39 ]), &(acadoWorkspace.E[ lRun3 * 39 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 50; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (13)) * (13)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (13)) * (3)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (13)) * (3)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (50)) - (1)) * (13)) * (3)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 49; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 39 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 169 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 169 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (13)) * (3)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 9 ]), &(acadoWorkspace.evGu[ lRun2 * 39 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

for (lRun1 = 0; lRun1 < 650; ++lRun1)
acadoWorkspace.sbar[lRun1 + 13] = acadoWorkspace.d[lRun1];

acadoWorkspace.lb[0] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)1.7329781022933570e+01 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)1.7329781022933570e+01 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)1.7329781022933570e+01 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)1.7329781022933570e+01 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)1.7329781022933570e+01 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)1.7329781022933570e+01 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)1.7329781022933570e+01 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)1.7329781022933570e+01 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)1.7329781022933570e+01 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)1.7329781022933570e+01 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)1.7329781022933570e+01 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)1.7329781022933570e+01 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)1.7329781022933570e+01 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[39];
acadoWorkspace.lb[40] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[40];
acadoWorkspace.lb[41] = (real_t)1.7329781022933570e+01 - acadoVariables.u[41];
acadoWorkspace.lb[42] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[42];
acadoWorkspace.lb[43] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[43];
acadoWorkspace.lb[44] = (real_t)1.7329781022933570e+01 - acadoVariables.u[44];
acadoWorkspace.lb[45] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[45];
acadoWorkspace.lb[46] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[46];
acadoWorkspace.lb[47] = (real_t)1.7329781022933570e+01 - acadoVariables.u[47];
acadoWorkspace.lb[48] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[48];
acadoWorkspace.lb[49] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[49];
acadoWorkspace.lb[50] = (real_t)1.7329781022933570e+01 - acadoVariables.u[50];
acadoWorkspace.lb[51] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[51];
acadoWorkspace.lb[52] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[52];
acadoWorkspace.lb[53] = (real_t)1.7329781022933570e+01 - acadoVariables.u[53];
acadoWorkspace.lb[54] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[54];
acadoWorkspace.lb[55] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[55];
acadoWorkspace.lb[56] = (real_t)1.7329781022933570e+01 - acadoVariables.u[56];
acadoWorkspace.lb[57] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[57];
acadoWorkspace.lb[58] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[58];
acadoWorkspace.lb[59] = (real_t)1.7329781022933570e+01 - acadoVariables.u[59];
acadoWorkspace.lb[60] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[60];
acadoWorkspace.lb[61] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[61];
acadoWorkspace.lb[62] = (real_t)1.7329781022933570e+01 - acadoVariables.u[62];
acadoWorkspace.lb[63] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[63];
acadoWorkspace.lb[64] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[64];
acadoWorkspace.lb[65] = (real_t)1.7329781022933570e+01 - acadoVariables.u[65];
acadoWorkspace.lb[66] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[66];
acadoWorkspace.lb[67] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[67];
acadoWorkspace.lb[68] = (real_t)1.7329781022933570e+01 - acadoVariables.u[68];
acadoWorkspace.lb[69] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[69];
acadoWorkspace.lb[70] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[70];
acadoWorkspace.lb[71] = (real_t)1.7329781022933570e+01 - acadoVariables.u[71];
acadoWorkspace.lb[72] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[72];
acadoWorkspace.lb[73] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[73];
acadoWorkspace.lb[74] = (real_t)1.7329781022933570e+01 - acadoVariables.u[74];
acadoWorkspace.lb[75] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[75];
acadoWorkspace.lb[76] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[76];
acadoWorkspace.lb[77] = (real_t)1.7329781022933570e+01 - acadoVariables.u[77];
acadoWorkspace.lb[78] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[78];
acadoWorkspace.lb[79] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[79];
acadoWorkspace.lb[80] = (real_t)1.7329781022933570e+01 - acadoVariables.u[80];
acadoWorkspace.lb[81] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[81];
acadoWorkspace.lb[82] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[82];
acadoWorkspace.lb[83] = (real_t)1.7329781022933570e+01 - acadoVariables.u[83];
acadoWorkspace.lb[84] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[84];
acadoWorkspace.lb[85] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[85];
acadoWorkspace.lb[86] = (real_t)1.7329781022933570e+01 - acadoVariables.u[86];
acadoWorkspace.lb[87] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[87];
acadoWorkspace.lb[88] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[88];
acadoWorkspace.lb[89] = (real_t)1.7329781022933570e+01 - acadoVariables.u[89];
acadoWorkspace.lb[90] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[90];
acadoWorkspace.lb[91] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[91];
acadoWorkspace.lb[92] = (real_t)1.7329781022933570e+01 - acadoVariables.u[92];
acadoWorkspace.lb[93] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[93];
acadoWorkspace.lb[94] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[94];
acadoWorkspace.lb[95] = (real_t)1.7329781022933570e+01 - acadoVariables.u[95];
acadoWorkspace.lb[96] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[96];
acadoWorkspace.lb[97] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[97];
acadoWorkspace.lb[98] = (real_t)1.7329781022933570e+01 - acadoVariables.u[98];
acadoWorkspace.lb[99] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[99];
acadoWorkspace.lb[100] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[100];
acadoWorkspace.lb[101] = (real_t)1.7329781022933570e+01 - acadoVariables.u[101];
acadoWorkspace.lb[102] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[102];
acadoWorkspace.lb[103] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[103];
acadoWorkspace.lb[104] = (real_t)1.7329781022933570e+01 - acadoVariables.u[104];
acadoWorkspace.lb[105] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[105];
acadoWorkspace.lb[106] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[106];
acadoWorkspace.lb[107] = (real_t)1.7329781022933570e+01 - acadoVariables.u[107];
acadoWorkspace.lb[108] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[108];
acadoWorkspace.lb[109] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[109];
acadoWorkspace.lb[110] = (real_t)1.7329781022933570e+01 - acadoVariables.u[110];
acadoWorkspace.lb[111] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[111];
acadoWorkspace.lb[112] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[112];
acadoWorkspace.lb[113] = (real_t)1.7329781022933570e+01 - acadoVariables.u[113];
acadoWorkspace.lb[114] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[114];
acadoWorkspace.lb[115] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[115];
acadoWorkspace.lb[116] = (real_t)1.7329781022933570e+01 - acadoVariables.u[116];
acadoWorkspace.lb[117] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[117];
acadoWorkspace.lb[118] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[118];
acadoWorkspace.lb[119] = (real_t)1.7329781022933570e+01 - acadoVariables.u[119];
acadoWorkspace.lb[120] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[120];
acadoWorkspace.lb[121] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[121];
acadoWorkspace.lb[122] = (real_t)1.7329781022933570e+01 - acadoVariables.u[122];
acadoWorkspace.lb[123] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[123];
acadoWorkspace.lb[124] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[124];
acadoWorkspace.lb[125] = (real_t)1.7329781022933570e+01 - acadoVariables.u[125];
acadoWorkspace.lb[126] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[126];
acadoWorkspace.lb[127] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[127];
acadoWorkspace.lb[128] = (real_t)1.7329781022933570e+01 - acadoVariables.u[128];
acadoWorkspace.lb[129] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[129];
acadoWorkspace.lb[130] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[130];
acadoWorkspace.lb[131] = (real_t)1.7329781022933570e+01 - acadoVariables.u[131];
acadoWorkspace.lb[132] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[132];
acadoWorkspace.lb[133] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[133];
acadoWorkspace.lb[134] = (real_t)1.7329781022933570e+01 - acadoVariables.u[134];
acadoWorkspace.lb[135] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[135];
acadoWorkspace.lb[136] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[136];
acadoWorkspace.lb[137] = (real_t)1.7329781022933570e+01 - acadoVariables.u[137];
acadoWorkspace.lb[138] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[138];
acadoWorkspace.lb[139] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[139];
acadoWorkspace.lb[140] = (real_t)1.7329781022933570e+01 - acadoVariables.u[140];
acadoWorkspace.lb[141] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[141];
acadoWorkspace.lb[142] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[142];
acadoWorkspace.lb[143] = (real_t)1.7329781022933570e+01 - acadoVariables.u[143];
acadoWorkspace.lb[144] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[144];
acadoWorkspace.lb[145] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[145];
acadoWorkspace.lb[146] = (real_t)1.7329781022933570e+01 - acadoVariables.u[146];
acadoWorkspace.lb[147] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[147];
acadoWorkspace.lb[148] = (real_t)-9.6276561238519831e-01 - acadoVariables.u[148];
acadoWorkspace.lb[149] = (real_t)1.7329781022933570e+01 - acadoVariables.u[149];
acadoWorkspace.ub[0] = (real_t)9.6276561238519831e-01 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)9.6276561238519831e-01 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)2.1180843472474361e+01 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)9.6276561238519831e-01 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)9.6276561238519831e-01 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)2.1180843472474361e+01 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)9.6276561238519831e-01 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)9.6276561238519831e-01 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)2.1180843472474361e+01 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)9.6276561238519831e-01 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)9.6276561238519831e-01 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)2.1180843472474361e+01 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)9.6276561238519831e-01 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)9.6276561238519831e-01 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)2.1180843472474361e+01 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)9.6276561238519831e-01 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)9.6276561238519831e-01 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)2.1180843472474361e+01 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)9.6276561238519831e-01 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)9.6276561238519831e-01 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)2.1180843472474361e+01 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)9.6276561238519831e-01 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)9.6276561238519831e-01 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)2.1180843472474361e+01 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)9.6276561238519831e-01 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)9.6276561238519831e-01 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)2.1180843472474361e+01 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)9.6276561238519831e-01 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)9.6276561238519831e-01 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)2.1180843472474361e+01 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)9.6276561238519831e-01 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)9.6276561238519831e-01 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)2.1180843472474361e+01 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)9.6276561238519831e-01 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)9.6276561238519831e-01 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)2.1180843472474361e+01 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)9.6276561238519831e-01 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)9.6276561238519831e-01 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)2.1180843472474361e+01 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)9.6276561238519831e-01 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)9.6276561238519831e-01 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)2.1180843472474361e+01 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)9.6276561238519831e-01 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)9.6276561238519831e-01 - acadoVariables.u[43];
acadoWorkspace.ub[44] = (real_t)2.1180843472474361e+01 - acadoVariables.u[44];
acadoWorkspace.ub[45] = (real_t)9.6276561238519831e-01 - acadoVariables.u[45];
acadoWorkspace.ub[46] = (real_t)9.6276561238519831e-01 - acadoVariables.u[46];
acadoWorkspace.ub[47] = (real_t)2.1180843472474361e+01 - acadoVariables.u[47];
acadoWorkspace.ub[48] = (real_t)9.6276561238519831e-01 - acadoVariables.u[48];
acadoWorkspace.ub[49] = (real_t)9.6276561238519831e-01 - acadoVariables.u[49];
acadoWorkspace.ub[50] = (real_t)2.1180843472474361e+01 - acadoVariables.u[50];
acadoWorkspace.ub[51] = (real_t)9.6276561238519831e-01 - acadoVariables.u[51];
acadoWorkspace.ub[52] = (real_t)9.6276561238519831e-01 - acadoVariables.u[52];
acadoWorkspace.ub[53] = (real_t)2.1180843472474361e+01 - acadoVariables.u[53];
acadoWorkspace.ub[54] = (real_t)9.6276561238519831e-01 - acadoVariables.u[54];
acadoWorkspace.ub[55] = (real_t)9.6276561238519831e-01 - acadoVariables.u[55];
acadoWorkspace.ub[56] = (real_t)2.1180843472474361e+01 - acadoVariables.u[56];
acadoWorkspace.ub[57] = (real_t)9.6276561238519831e-01 - acadoVariables.u[57];
acadoWorkspace.ub[58] = (real_t)9.6276561238519831e-01 - acadoVariables.u[58];
acadoWorkspace.ub[59] = (real_t)2.1180843472474361e+01 - acadoVariables.u[59];
acadoWorkspace.ub[60] = (real_t)9.6276561238519831e-01 - acadoVariables.u[60];
acadoWorkspace.ub[61] = (real_t)9.6276561238519831e-01 - acadoVariables.u[61];
acadoWorkspace.ub[62] = (real_t)2.1180843472474361e+01 - acadoVariables.u[62];
acadoWorkspace.ub[63] = (real_t)9.6276561238519831e-01 - acadoVariables.u[63];
acadoWorkspace.ub[64] = (real_t)9.6276561238519831e-01 - acadoVariables.u[64];
acadoWorkspace.ub[65] = (real_t)2.1180843472474361e+01 - acadoVariables.u[65];
acadoWorkspace.ub[66] = (real_t)9.6276561238519831e-01 - acadoVariables.u[66];
acadoWorkspace.ub[67] = (real_t)9.6276561238519831e-01 - acadoVariables.u[67];
acadoWorkspace.ub[68] = (real_t)2.1180843472474361e+01 - acadoVariables.u[68];
acadoWorkspace.ub[69] = (real_t)9.6276561238519831e-01 - acadoVariables.u[69];
acadoWorkspace.ub[70] = (real_t)9.6276561238519831e-01 - acadoVariables.u[70];
acadoWorkspace.ub[71] = (real_t)2.1180843472474361e+01 - acadoVariables.u[71];
acadoWorkspace.ub[72] = (real_t)9.6276561238519831e-01 - acadoVariables.u[72];
acadoWorkspace.ub[73] = (real_t)9.6276561238519831e-01 - acadoVariables.u[73];
acadoWorkspace.ub[74] = (real_t)2.1180843472474361e+01 - acadoVariables.u[74];
acadoWorkspace.ub[75] = (real_t)9.6276561238519831e-01 - acadoVariables.u[75];
acadoWorkspace.ub[76] = (real_t)9.6276561238519831e-01 - acadoVariables.u[76];
acadoWorkspace.ub[77] = (real_t)2.1180843472474361e+01 - acadoVariables.u[77];
acadoWorkspace.ub[78] = (real_t)9.6276561238519831e-01 - acadoVariables.u[78];
acadoWorkspace.ub[79] = (real_t)9.6276561238519831e-01 - acadoVariables.u[79];
acadoWorkspace.ub[80] = (real_t)2.1180843472474361e+01 - acadoVariables.u[80];
acadoWorkspace.ub[81] = (real_t)9.6276561238519831e-01 - acadoVariables.u[81];
acadoWorkspace.ub[82] = (real_t)9.6276561238519831e-01 - acadoVariables.u[82];
acadoWorkspace.ub[83] = (real_t)2.1180843472474361e+01 - acadoVariables.u[83];
acadoWorkspace.ub[84] = (real_t)9.6276561238519831e-01 - acadoVariables.u[84];
acadoWorkspace.ub[85] = (real_t)9.6276561238519831e-01 - acadoVariables.u[85];
acadoWorkspace.ub[86] = (real_t)2.1180843472474361e+01 - acadoVariables.u[86];
acadoWorkspace.ub[87] = (real_t)9.6276561238519831e-01 - acadoVariables.u[87];
acadoWorkspace.ub[88] = (real_t)9.6276561238519831e-01 - acadoVariables.u[88];
acadoWorkspace.ub[89] = (real_t)2.1180843472474361e+01 - acadoVariables.u[89];
acadoWorkspace.ub[90] = (real_t)9.6276561238519831e-01 - acadoVariables.u[90];
acadoWorkspace.ub[91] = (real_t)9.6276561238519831e-01 - acadoVariables.u[91];
acadoWorkspace.ub[92] = (real_t)2.1180843472474361e+01 - acadoVariables.u[92];
acadoWorkspace.ub[93] = (real_t)9.6276561238519831e-01 - acadoVariables.u[93];
acadoWorkspace.ub[94] = (real_t)9.6276561238519831e-01 - acadoVariables.u[94];
acadoWorkspace.ub[95] = (real_t)2.1180843472474361e+01 - acadoVariables.u[95];
acadoWorkspace.ub[96] = (real_t)9.6276561238519831e-01 - acadoVariables.u[96];
acadoWorkspace.ub[97] = (real_t)9.6276561238519831e-01 - acadoVariables.u[97];
acadoWorkspace.ub[98] = (real_t)2.1180843472474361e+01 - acadoVariables.u[98];
acadoWorkspace.ub[99] = (real_t)9.6276561238519831e-01 - acadoVariables.u[99];
acadoWorkspace.ub[100] = (real_t)9.6276561238519831e-01 - acadoVariables.u[100];
acadoWorkspace.ub[101] = (real_t)2.1180843472474361e+01 - acadoVariables.u[101];
acadoWorkspace.ub[102] = (real_t)9.6276561238519831e-01 - acadoVariables.u[102];
acadoWorkspace.ub[103] = (real_t)9.6276561238519831e-01 - acadoVariables.u[103];
acadoWorkspace.ub[104] = (real_t)2.1180843472474361e+01 - acadoVariables.u[104];
acadoWorkspace.ub[105] = (real_t)9.6276561238519831e-01 - acadoVariables.u[105];
acadoWorkspace.ub[106] = (real_t)9.6276561238519831e-01 - acadoVariables.u[106];
acadoWorkspace.ub[107] = (real_t)2.1180843472474361e+01 - acadoVariables.u[107];
acadoWorkspace.ub[108] = (real_t)9.6276561238519831e-01 - acadoVariables.u[108];
acadoWorkspace.ub[109] = (real_t)9.6276561238519831e-01 - acadoVariables.u[109];
acadoWorkspace.ub[110] = (real_t)2.1180843472474361e+01 - acadoVariables.u[110];
acadoWorkspace.ub[111] = (real_t)9.6276561238519831e-01 - acadoVariables.u[111];
acadoWorkspace.ub[112] = (real_t)9.6276561238519831e-01 - acadoVariables.u[112];
acadoWorkspace.ub[113] = (real_t)2.1180843472474361e+01 - acadoVariables.u[113];
acadoWorkspace.ub[114] = (real_t)9.6276561238519831e-01 - acadoVariables.u[114];
acadoWorkspace.ub[115] = (real_t)9.6276561238519831e-01 - acadoVariables.u[115];
acadoWorkspace.ub[116] = (real_t)2.1180843472474361e+01 - acadoVariables.u[116];
acadoWorkspace.ub[117] = (real_t)9.6276561238519831e-01 - acadoVariables.u[117];
acadoWorkspace.ub[118] = (real_t)9.6276561238519831e-01 - acadoVariables.u[118];
acadoWorkspace.ub[119] = (real_t)2.1180843472474361e+01 - acadoVariables.u[119];
acadoWorkspace.ub[120] = (real_t)9.6276561238519831e-01 - acadoVariables.u[120];
acadoWorkspace.ub[121] = (real_t)9.6276561238519831e-01 - acadoVariables.u[121];
acadoWorkspace.ub[122] = (real_t)2.1180843472474361e+01 - acadoVariables.u[122];
acadoWorkspace.ub[123] = (real_t)9.6276561238519831e-01 - acadoVariables.u[123];
acadoWorkspace.ub[124] = (real_t)9.6276561238519831e-01 - acadoVariables.u[124];
acadoWorkspace.ub[125] = (real_t)2.1180843472474361e+01 - acadoVariables.u[125];
acadoWorkspace.ub[126] = (real_t)9.6276561238519831e-01 - acadoVariables.u[126];
acadoWorkspace.ub[127] = (real_t)9.6276561238519831e-01 - acadoVariables.u[127];
acadoWorkspace.ub[128] = (real_t)2.1180843472474361e+01 - acadoVariables.u[128];
acadoWorkspace.ub[129] = (real_t)9.6276561238519831e-01 - acadoVariables.u[129];
acadoWorkspace.ub[130] = (real_t)9.6276561238519831e-01 - acadoVariables.u[130];
acadoWorkspace.ub[131] = (real_t)2.1180843472474361e+01 - acadoVariables.u[131];
acadoWorkspace.ub[132] = (real_t)9.6276561238519831e-01 - acadoVariables.u[132];
acadoWorkspace.ub[133] = (real_t)9.6276561238519831e-01 - acadoVariables.u[133];
acadoWorkspace.ub[134] = (real_t)2.1180843472474361e+01 - acadoVariables.u[134];
acadoWorkspace.ub[135] = (real_t)9.6276561238519831e-01 - acadoVariables.u[135];
acadoWorkspace.ub[136] = (real_t)9.6276561238519831e-01 - acadoVariables.u[136];
acadoWorkspace.ub[137] = (real_t)2.1180843472474361e+01 - acadoVariables.u[137];
acadoWorkspace.ub[138] = (real_t)9.6276561238519831e-01 - acadoVariables.u[138];
acadoWorkspace.ub[139] = (real_t)9.6276561238519831e-01 - acadoVariables.u[139];
acadoWorkspace.ub[140] = (real_t)2.1180843472474361e+01 - acadoVariables.u[140];
acadoWorkspace.ub[141] = (real_t)9.6276561238519831e-01 - acadoVariables.u[141];
acadoWorkspace.ub[142] = (real_t)9.6276561238519831e-01 - acadoVariables.u[142];
acadoWorkspace.ub[143] = (real_t)2.1180843472474361e+01 - acadoVariables.u[143];
acadoWorkspace.ub[144] = (real_t)9.6276561238519831e-01 - acadoVariables.u[144];
acadoWorkspace.ub[145] = (real_t)9.6276561238519831e-01 - acadoVariables.u[145];
acadoWorkspace.ub[146] = (real_t)2.1180843472474361e+01 - acadoVariables.u[146];
acadoWorkspace.ub[147] = (real_t)9.6276561238519831e-01 - acadoVariables.u[147];
acadoWorkspace.ub[148] = (real_t)9.6276561238519831e-01 - acadoVariables.u[148];
acadoWorkspace.ub[149] = (real_t)2.1180843472474361e+01 - acadoVariables.u[149];

}

void acado_condenseFdb(  )
{
int lRun1;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];
acadoWorkspace.Dx0[9] = acadoVariables.x0[9] - acadoVariables.x[9];
acadoWorkspace.Dx0[10] = acadoVariables.x0[10] - acadoVariables.x[10];
acadoWorkspace.Dx0[11] = acadoVariables.x0[11] - acadoVariables.x[11];
acadoWorkspace.Dx0[12] = acadoVariables.x0[12] - acadoVariables.x[12];
for (lRun1 = 0; lRun1 < 800; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];
acadoWorkspace.DyN[6] -= acadoVariables.yN[6];
acadoWorkspace.DyN[7] -= acadoVariables.yN[7];
acadoWorkspace.DyN[8] -= acadoVariables.yN[8];
acadoWorkspace.DyN[9] -= acadoVariables.yN[9];
acadoWorkspace.DyN[10] -= acadoVariables.yN[10];
acadoWorkspace.DyN[11] -= acadoVariables.yN[11];
acadoWorkspace.DyN[12] -= acadoVariables.yN[12];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 48 ]), &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 96 ]), &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 144 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 192 ]), &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 240 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 288 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 336 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 384 ]), &(acadoWorkspace.Dy[ 128 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 432 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 480 ]), &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 528 ]), &(acadoWorkspace.Dy[ 176 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 576 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 624 ]), &(acadoWorkspace.Dy[ 208 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 672 ]), &(acadoWorkspace.Dy[ 224 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 720 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 768 ]), &(acadoWorkspace.Dy[ 256 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 816 ]), &(acadoWorkspace.Dy[ 272 ]), &(acadoWorkspace.g[ 51 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 864 ]), &(acadoWorkspace.Dy[ 288 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 912 ]), &(acadoWorkspace.Dy[ 304 ]), &(acadoWorkspace.g[ 57 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 960 ]), &(acadoWorkspace.Dy[ 320 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1008 ]), &(acadoWorkspace.Dy[ 336 ]), &(acadoWorkspace.g[ 63 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1056 ]), &(acadoWorkspace.Dy[ 352 ]), &(acadoWorkspace.g[ 66 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1104 ]), &(acadoWorkspace.Dy[ 368 ]), &(acadoWorkspace.g[ 69 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1152 ]), &(acadoWorkspace.Dy[ 384 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1200 ]), &(acadoWorkspace.Dy[ 400 ]), &(acadoWorkspace.g[ 75 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1248 ]), &(acadoWorkspace.Dy[ 416 ]), &(acadoWorkspace.g[ 78 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1296 ]), &(acadoWorkspace.Dy[ 432 ]), &(acadoWorkspace.g[ 81 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1344 ]), &(acadoWorkspace.Dy[ 448 ]), &(acadoWorkspace.g[ 84 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1392 ]), &(acadoWorkspace.Dy[ 464 ]), &(acadoWorkspace.g[ 87 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1440 ]), &(acadoWorkspace.Dy[ 480 ]), &(acadoWorkspace.g[ 90 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1488 ]), &(acadoWorkspace.Dy[ 496 ]), &(acadoWorkspace.g[ 93 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1536 ]), &(acadoWorkspace.Dy[ 512 ]), &(acadoWorkspace.g[ 96 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1584 ]), &(acadoWorkspace.Dy[ 528 ]), &(acadoWorkspace.g[ 99 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1632 ]), &(acadoWorkspace.Dy[ 544 ]), &(acadoWorkspace.g[ 102 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1680 ]), &(acadoWorkspace.Dy[ 560 ]), &(acadoWorkspace.g[ 105 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1728 ]), &(acadoWorkspace.Dy[ 576 ]), &(acadoWorkspace.g[ 108 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1776 ]), &(acadoWorkspace.Dy[ 592 ]), &(acadoWorkspace.g[ 111 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1824 ]), &(acadoWorkspace.Dy[ 608 ]), &(acadoWorkspace.g[ 114 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1872 ]), &(acadoWorkspace.Dy[ 624 ]), &(acadoWorkspace.g[ 117 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1920 ]), &(acadoWorkspace.Dy[ 640 ]), &(acadoWorkspace.g[ 120 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1968 ]), &(acadoWorkspace.Dy[ 656 ]), &(acadoWorkspace.g[ 123 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 2016 ]), &(acadoWorkspace.Dy[ 672 ]), &(acadoWorkspace.g[ 126 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 2064 ]), &(acadoWorkspace.Dy[ 688 ]), &(acadoWorkspace.g[ 129 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 2112 ]), &(acadoWorkspace.Dy[ 704 ]), &(acadoWorkspace.g[ 132 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 2160 ]), &(acadoWorkspace.Dy[ 720 ]), &(acadoWorkspace.g[ 135 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 2208 ]), &(acadoWorkspace.Dy[ 736 ]), &(acadoWorkspace.g[ 138 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 2256 ]), &(acadoWorkspace.Dy[ 752 ]), &(acadoWorkspace.g[ 141 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 2304 ]), &(acadoWorkspace.Dy[ 768 ]), &(acadoWorkspace.g[ 144 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 2352 ]), &(acadoWorkspace.Dy[ 784 ]), &(acadoWorkspace.g[ 147 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 208 ]), &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.QDy[ 13 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 416 ]), &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.QDy[ 26 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 624 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 39 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 832 ]), &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.QDy[ 52 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1040 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 65 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1248 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.QDy[ 78 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1456 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.QDy[ 91 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1664 ]), &(acadoWorkspace.Dy[ 128 ]), &(acadoWorkspace.QDy[ 104 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1872 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.QDy[ 117 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2080 ]), &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.QDy[ 130 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2288 ]), &(acadoWorkspace.Dy[ 176 ]), &(acadoWorkspace.QDy[ 143 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2496 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.QDy[ 156 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2704 ]), &(acadoWorkspace.Dy[ 208 ]), &(acadoWorkspace.QDy[ 169 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2912 ]), &(acadoWorkspace.Dy[ 224 ]), &(acadoWorkspace.QDy[ 182 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3120 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.QDy[ 195 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3328 ]), &(acadoWorkspace.Dy[ 256 ]), &(acadoWorkspace.QDy[ 208 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3536 ]), &(acadoWorkspace.Dy[ 272 ]), &(acadoWorkspace.QDy[ 221 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3744 ]), &(acadoWorkspace.Dy[ 288 ]), &(acadoWorkspace.QDy[ 234 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3952 ]), &(acadoWorkspace.Dy[ 304 ]), &(acadoWorkspace.QDy[ 247 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 4160 ]), &(acadoWorkspace.Dy[ 320 ]), &(acadoWorkspace.QDy[ 260 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 4368 ]), &(acadoWorkspace.Dy[ 336 ]), &(acadoWorkspace.QDy[ 273 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 4576 ]), &(acadoWorkspace.Dy[ 352 ]), &(acadoWorkspace.QDy[ 286 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 4784 ]), &(acadoWorkspace.Dy[ 368 ]), &(acadoWorkspace.QDy[ 299 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 4992 ]), &(acadoWorkspace.Dy[ 384 ]), &(acadoWorkspace.QDy[ 312 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 5200 ]), &(acadoWorkspace.Dy[ 400 ]), &(acadoWorkspace.QDy[ 325 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 5408 ]), &(acadoWorkspace.Dy[ 416 ]), &(acadoWorkspace.QDy[ 338 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 5616 ]), &(acadoWorkspace.Dy[ 432 ]), &(acadoWorkspace.QDy[ 351 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 5824 ]), &(acadoWorkspace.Dy[ 448 ]), &(acadoWorkspace.QDy[ 364 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 6032 ]), &(acadoWorkspace.Dy[ 464 ]), &(acadoWorkspace.QDy[ 377 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 6240 ]), &(acadoWorkspace.Dy[ 480 ]), &(acadoWorkspace.QDy[ 390 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 6448 ]), &(acadoWorkspace.Dy[ 496 ]), &(acadoWorkspace.QDy[ 403 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 6656 ]), &(acadoWorkspace.Dy[ 512 ]), &(acadoWorkspace.QDy[ 416 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 6864 ]), &(acadoWorkspace.Dy[ 528 ]), &(acadoWorkspace.QDy[ 429 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 7072 ]), &(acadoWorkspace.Dy[ 544 ]), &(acadoWorkspace.QDy[ 442 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 7280 ]), &(acadoWorkspace.Dy[ 560 ]), &(acadoWorkspace.QDy[ 455 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 7488 ]), &(acadoWorkspace.Dy[ 576 ]), &(acadoWorkspace.QDy[ 468 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 7696 ]), &(acadoWorkspace.Dy[ 592 ]), &(acadoWorkspace.QDy[ 481 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 7904 ]), &(acadoWorkspace.Dy[ 608 ]), &(acadoWorkspace.QDy[ 494 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 8112 ]), &(acadoWorkspace.Dy[ 624 ]), &(acadoWorkspace.QDy[ 507 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 8320 ]), &(acadoWorkspace.Dy[ 640 ]), &(acadoWorkspace.QDy[ 520 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 8528 ]), &(acadoWorkspace.Dy[ 656 ]), &(acadoWorkspace.QDy[ 533 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 8736 ]), &(acadoWorkspace.Dy[ 672 ]), &(acadoWorkspace.QDy[ 546 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 8944 ]), &(acadoWorkspace.Dy[ 688 ]), &(acadoWorkspace.QDy[ 559 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 9152 ]), &(acadoWorkspace.Dy[ 704 ]), &(acadoWorkspace.QDy[ 572 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 9360 ]), &(acadoWorkspace.Dy[ 720 ]), &(acadoWorkspace.QDy[ 585 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 9568 ]), &(acadoWorkspace.Dy[ 736 ]), &(acadoWorkspace.QDy[ 598 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 9776 ]), &(acadoWorkspace.Dy[ 752 ]), &(acadoWorkspace.QDy[ 611 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 9984 ]), &(acadoWorkspace.Dy[ 768 ]), &(acadoWorkspace.QDy[ 624 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 10192 ]), &(acadoWorkspace.Dy[ 784 ]), &(acadoWorkspace.QDy[ 637 ]) );

acadoWorkspace.QDy[650] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[651] = + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[25]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[652] = + acadoWorkspace.QN2[26]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[27]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[28]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[29]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[30]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[31]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[32]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[33]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[34]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[35]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[36]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[37]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[38]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[653] = + acadoWorkspace.QN2[39]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[40]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[41]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[42]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[43]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[44]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[45]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[46]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[47]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[48]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[49]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[50]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[51]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[654] = + acadoWorkspace.QN2[52]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[53]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[54]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[55]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[56]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[57]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[58]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[59]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[60]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[61]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[62]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[63]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[64]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[655] = + acadoWorkspace.QN2[65]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[66]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[67]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[68]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[69]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[70]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[71]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[72]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[73]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[74]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[75]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[76]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[77]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[656] = + acadoWorkspace.QN2[78]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[79]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[80]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[81]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[82]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[83]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[84]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[85]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[86]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[87]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[88]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[89]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[90]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[657] = + acadoWorkspace.QN2[91]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[92]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[93]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[94]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[95]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[96]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[97]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[98]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[99]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[100]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[101]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[102]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[103]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[658] = + acadoWorkspace.QN2[104]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[105]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[106]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[107]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[108]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[109]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[110]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[111]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[112]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[113]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[114]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[115]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[116]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[659] = + acadoWorkspace.QN2[117]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[118]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[119]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[120]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[121]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[122]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[123]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[124]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[125]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[126]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[127]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[128]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[129]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[660] = + acadoWorkspace.QN2[130]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[131]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[132]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[133]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[134]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[135]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[136]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[137]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[138]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[139]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[140]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[141]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[142]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[661] = + acadoWorkspace.QN2[143]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[144]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[145]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[146]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[147]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[148]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[149]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[150]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[151]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[152]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[153]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[154]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[155]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[662] = + acadoWorkspace.QN2[156]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[157]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[158]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[159]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[160]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[161]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[162]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[163]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[164]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[165]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[166]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[167]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[168]*acadoWorkspace.DyN[12];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acadoWorkspace.sbar[9] = acadoWorkspace.Dx0[9];
acadoWorkspace.sbar[10] = acadoWorkspace.Dx0[10];
acadoWorkspace.sbar[11] = acadoWorkspace.Dx0[11];
acadoWorkspace.sbar[12] = acadoWorkspace.Dx0[12];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 13 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 169 ]), &(acadoWorkspace.sbar[ 13 ]), &(acadoWorkspace.sbar[ 26 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 338 ]), &(acadoWorkspace.sbar[ 26 ]), &(acadoWorkspace.sbar[ 39 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 507 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.sbar[ 52 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 676 ]), &(acadoWorkspace.sbar[ 52 ]), &(acadoWorkspace.sbar[ 65 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 845 ]), &(acadoWorkspace.sbar[ 65 ]), &(acadoWorkspace.sbar[ 78 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1014 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.sbar[ 91 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1183 ]), &(acadoWorkspace.sbar[ 91 ]), &(acadoWorkspace.sbar[ 104 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1352 ]), &(acadoWorkspace.sbar[ 104 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1521 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 130 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1690 ]), &(acadoWorkspace.sbar[ 130 ]), &(acadoWorkspace.sbar[ 143 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1859 ]), &(acadoWorkspace.sbar[ 143 ]), &(acadoWorkspace.sbar[ 156 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2028 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.sbar[ 169 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2197 ]), &(acadoWorkspace.sbar[ 169 ]), &(acadoWorkspace.sbar[ 182 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2366 ]), &(acadoWorkspace.sbar[ 182 ]), &(acadoWorkspace.sbar[ 195 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2535 ]), &(acadoWorkspace.sbar[ 195 ]), &(acadoWorkspace.sbar[ 208 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2704 ]), &(acadoWorkspace.sbar[ 208 ]), &(acadoWorkspace.sbar[ 221 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2873 ]), &(acadoWorkspace.sbar[ 221 ]), &(acadoWorkspace.sbar[ 234 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 3042 ]), &(acadoWorkspace.sbar[ 234 ]), &(acadoWorkspace.sbar[ 247 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 3211 ]), &(acadoWorkspace.sbar[ 247 ]), &(acadoWorkspace.sbar[ 260 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 3380 ]), &(acadoWorkspace.sbar[ 260 ]), &(acadoWorkspace.sbar[ 273 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 3549 ]), &(acadoWorkspace.sbar[ 273 ]), &(acadoWorkspace.sbar[ 286 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 3718 ]), &(acadoWorkspace.sbar[ 286 ]), &(acadoWorkspace.sbar[ 299 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 3887 ]), &(acadoWorkspace.sbar[ 299 ]), &(acadoWorkspace.sbar[ 312 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 4056 ]), &(acadoWorkspace.sbar[ 312 ]), &(acadoWorkspace.sbar[ 325 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 4225 ]), &(acadoWorkspace.sbar[ 325 ]), &(acadoWorkspace.sbar[ 338 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 4394 ]), &(acadoWorkspace.sbar[ 338 ]), &(acadoWorkspace.sbar[ 351 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 4563 ]), &(acadoWorkspace.sbar[ 351 ]), &(acadoWorkspace.sbar[ 364 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 4732 ]), &(acadoWorkspace.sbar[ 364 ]), &(acadoWorkspace.sbar[ 377 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 4901 ]), &(acadoWorkspace.sbar[ 377 ]), &(acadoWorkspace.sbar[ 390 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 5070 ]), &(acadoWorkspace.sbar[ 390 ]), &(acadoWorkspace.sbar[ 403 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 5239 ]), &(acadoWorkspace.sbar[ 403 ]), &(acadoWorkspace.sbar[ 416 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 5408 ]), &(acadoWorkspace.sbar[ 416 ]), &(acadoWorkspace.sbar[ 429 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 5577 ]), &(acadoWorkspace.sbar[ 429 ]), &(acadoWorkspace.sbar[ 442 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 5746 ]), &(acadoWorkspace.sbar[ 442 ]), &(acadoWorkspace.sbar[ 455 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 5915 ]), &(acadoWorkspace.sbar[ 455 ]), &(acadoWorkspace.sbar[ 468 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 6084 ]), &(acadoWorkspace.sbar[ 468 ]), &(acadoWorkspace.sbar[ 481 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 6253 ]), &(acadoWorkspace.sbar[ 481 ]), &(acadoWorkspace.sbar[ 494 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 6422 ]), &(acadoWorkspace.sbar[ 494 ]), &(acadoWorkspace.sbar[ 507 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 6591 ]), &(acadoWorkspace.sbar[ 507 ]), &(acadoWorkspace.sbar[ 520 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 6760 ]), &(acadoWorkspace.sbar[ 520 ]), &(acadoWorkspace.sbar[ 533 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 6929 ]), &(acadoWorkspace.sbar[ 533 ]), &(acadoWorkspace.sbar[ 546 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 7098 ]), &(acadoWorkspace.sbar[ 546 ]), &(acadoWorkspace.sbar[ 559 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 7267 ]), &(acadoWorkspace.sbar[ 559 ]), &(acadoWorkspace.sbar[ 572 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 7436 ]), &(acadoWorkspace.sbar[ 572 ]), &(acadoWorkspace.sbar[ 585 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 7605 ]), &(acadoWorkspace.sbar[ 585 ]), &(acadoWorkspace.sbar[ 598 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 7774 ]), &(acadoWorkspace.sbar[ 598 ]), &(acadoWorkspace.sbar[ 611 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 7943 ]), &(acadoWorkspace.sbar[ 611 ]), &(acadoWorkspace.sbar[ 624 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 8112 ]), &(acadoWorkspace.sbar[ 624 ]), &(acadoWorkspace.sbar[ 637 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 8281 ]), &(acadoWorkspace.sbar[ 637 ]), &(acadoWorkspace.sbar[ 650 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[650] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[651] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[652] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[653] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[654] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[655] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[656] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[657] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[658] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[659] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[660] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[661] + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[662] + acadoWorkspace.QDy[650];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[650] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[651] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[652] + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[653] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[654] + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[655] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[656] + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[657] + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[658] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[659] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[660] + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[661] + acadoWorkspace.QN1[25]*acadoWorkspace.sbar[662] + acadoWorkspace.QDy[651];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[26]*acadoWorkspace.sbar[650] + acadoWorkspace.QN1[27]*acadoWorkspace.sbar[651] + acadoWorkspace.QN1[28]*acadoWorkspace.sbar[652] + acadoWorkspace.QN1[29]*acadoWorkspace.sbar[653] + acadoWorkspace.QN1[30]*acadoWorkspace.sbar[654] + acadoWorkspace.QN1[31]*acadoWorkspace.sbar[655] + acadoWorkspace.QN1[32]*acadoWorkspace.sbar[656] + acadoWorkspace.QN1[33]*acadoWorkspace.sbar[657] + acadoWorkspace.QN1[34]*acadoWorkspace.sbar[658] + acadoWorkspace.QN1[35]*acadoWorkspace.sbar[659] + acadoWorkspace.QN1[36]*acadoWorkspace.sbar[660] + acadoWorkspace.QN1[37]*acadoWorkspace.sbar[661] + acadoWorkspace.QN1[38]*acadoWorkspace.sbar[662] + acadoWorkspace.QDy[652];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[39]*acadoWorkspace.sbar[650] + acadoWorkspace.QN1[40]*acadoWorkspace.sbar[651] + acadoWorkspace.QN1[41]*acadoWorkspace.sbar[652] + acadoWorkspace.QN1[42]*acadoWorkspace.sbar[653] + acadoWorkspace.QN1[43]*acadoWorkspace.sbar[654] + acadoWorkspace.QN1[44]*acadoWorkspace.sbar[655] + acadoWorkspace.QN1[45]*acadoWorkspace.sbar[656] + acadoWorkspace.QN1[46]*acadoWorkspace.sbar[657] + acadoWorkspace.QN1[47]*acadoWorkspace.sbar[658] + acadoWorkspace.QN1[48]*acadoWorkspace.sbar[659] + acadoWorkspace.QN1[49]*acadoWorkspace.sbar[660] + acadoWorkspace.QN1[50]*acadoWorkspace.sbar[661] + acadoWorkspace.QN1[51]*acadoWorkspace.sbar[662] + acadoWorkspace.QDy[653];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[52]*acadoWorkspace.sbar[650] + acadoWorkspace.QN1[53]*acadoWorkspace.sbar[651] + acadoWorkspace.QN1[54]*acadoWorkspace.sbar[652] + acadoWorkspace.QN1[55]*acadoWorkspace.sbar[653] + acadoWorkspace.QN1[56]*acadoWorkspace.sbar[654] + acadoWorkspace.QN1[57]*acadoWorkspace.sbar[655] + acadoWorkspace.QN1[58]*acadoWorkspace.sbar[656] + acadoWorkspace.QN1[59]*acadoWorkspace.sbar[657] + acadoWorkspace.QN1[60]*acadoWorkspace.sbar[658] + acadoWorkspace.QN1[61]*acadoWorkspace.sbar[659] + acadoWorkspace.QN1[62]*acadoWorkspace.sbar[660] + acadoWorkspace.QN1[63]*acadoWorkspace.sbar[661] + acadoWorkspace.QN1[64]*acadoWorkspace.sbar[662] + acadoWorkspace.QDy[654];
acadoWorkspace.w1[5] = + acadoWorkspace.QN1[65]*acadoWorkspace.sbar[650] + acadoWorkspace.QN1[66]*acadoWorkspace.sbar[651] + acadoWorkspace.QN1[67]*acadoWorkspace.sbar[652] + acadoWorkspace.QN1[68]*acadoWorkspace.sbar[653] + acadoWorkspace.QN1[69]*acadoWorkspace.sbar[654] + acadoWorkspace.QN1[70]*acadoWorkspace.sbar[655] + acadoWorkspace.QN1[71]*acadoWorkspace.sbar[656] + acadoWorkspace.QN1[72]*acadoWorkspace.sbar[657] + acadoWorkspace.QN1[73]*acadoWorkspace.sbar[658] + acadoWorkspace.QN1[74]*acadoWorkspace.sbar[659] + acadoWorkspace.QN1[75]*acadoWorkspace.sbar[660] + acadoWorkspace.QN1[76]*acadoWorkspace.sbar[661] + acadoWorkspace.QN1[77]*acadoWorkspace.sbar[662] + acadoWorkspace.QDy[655];
acadoWorkspace.w1[6] = + acadoWorkspace.QN1[78]*acadoWorkspace.sbar[650] + acadoWorkspace.QN1[79]*acadoWorkspace.sbar[651] + acadoWorkspace.QN1[80]*acadoWorkspace.sbar[652] + acadoWorkspace.QN1[81]*acadoWorkspace.sbar[653] + acadoWorkspace.QN1[82]*acadoWorkspace.sbar[654] + acadoWorkspace.QN1[83]*acadoWorkspace.sbar[655] + acadoWorkspace.QN1[84]*acadoWorkspace.sbar[656] + acadoWorkspace.QN1[85]*acadoWorkspace.sbar[657] + acadoWorkspace.QN1[86]*acadoWorkspace.sbar[658] + acadoWorkspace.QN1[87]*acadoWorkspace.sbar[659] + acadoWorkspace.QN1[88]*acadoWorkspace.sbar[660] + acadoWorkspace.QN1[89]*acadoWorkspace.sbar[661] + acadoWorkspace.QN1[90]*acadoWorkspace.sbar[662] + acadoWorkspace.QDy[656];
acadoWorkspace.w1[7] = + acadoWorkspace.QN1[91]*acadoWorkspace.sbar[650] + acadoWorkspace.QN1[92]*acadoWorkspace.sbar[651] + acadoWorkspace.QN1[93]*acadoWorkspace.sbar[652] + acadoWorkspace.QN1[94]*acadoWorkspace.sbar[653] + acadoWorkspace.QN1[95]*acadoWorkspace.sbar[654] + acadoWorkspace.QN1[96]*acadoWorkspace.sbar[655] + acadoWorkspace.QN1[97]*acadoWorkspace.sbar[656] + acadoWorkspace.QN1[98]*acadoWorkspace.sbar[657] + acadoWorkspace.QN1[99]*acadoWorkspace.sbar[658] + acadoWorkspace.QN1[100]*acadoWorkspace.sbar[659] + acadoWorkspace.QN1[101]*acadoWorkspace.sbar[660] + acadoWorkspace.QN1[102]*acadoWorkspace.sbar[661] + acadoWorkspace.QN1[103]*acadoWorkspace.sbar[662] + acadoWorkspace.QDy[657];
acadoWorkspace.w1[8] = + acadoWorkspace.QN1[104]*acadoWorkspace.sbar[650] + acadoWorkspace.QN1[105]*acadoWorkspace.sbar[651] + acadoWorkspace.QN1[106]*acadoWorkspace.sbar[652] + acadoWorkspace.QN1[107]*acadoWorkspace.sbar[653] + acadoWorkspace.QN1[108]*acadoWorkspace.sbar[654] + acadoWorkspace.QN1[109]*acadoWorkspace.sbar[655] + acadoWorkspace.QN1[110]*acadoWorkspace.sbar[656] + acadoWorkspace.QN1[111]*acadoWorkspace.sbar[657] + acadoWorkspace.QN1[112]*acadoWorkspace.sbar[658] + acadoWorkspace.QN1[113]*acadoWorkspace.sbar[659] + acadoWorkspace.QN1[114]*acadoWorkspace.sbar[660] + acadoWorkspace.QN1[115]*acadoWorkspace.sbar[661] + acadoWorkspace.QN1[116]*acadoWorkspace.sbar[662] + acadoWorkspace.QDy[658];
acadoWorkspace.w1[9] = + acadoWorkspace.QN1[117]*acadoWorkspace.sbar[650] + acadoWorkspace.QN1[118]*acadoWorkspace.sbar[651] + acadoWorkspace.QN1[119]*acadoWorkspace.sbar[652] + acadoWorkspace.QN1[120]*acadoWorkspace.sbar[653] + acadoWorkspace.QN1[121]*acadoWorkspace.sbar[654] + acadoWorkspace.QN1[122]*acadoWorkspace.sbar[655] + acadoWorkspace.QN1[123]*acadoWorkspace.sbar[656] + acadoWorkspace.QN1[124]*acadoWorkspace.sbar[657] + acadoWorkspace.QN1[125]*acadoWorkspace.sbar[658] + acadoWorkspace.QN1[126]*acadoWorkspace.sbar[659] + acadoWorkspace.QN1[127]*acadoWorkspace.sbar[660] + acadoWorkspace.QN1[128]*acadoWorkspace.sbar[661] + acadoWorkspace.QN1[129]*acadoWorkspace.sbar[662] + acadoWorkspace.QDy[659];
acadoWorkspace.w1[10] = + acadoWorkspace.QN1[130]*acadoWorkspace.sbar[650] + acadoWorkspace.QN1[131]*acadoWorkspace.sbar[651] + acadoWorkspace.QN1[132]*acadoWorkspace.sbar[652] + acadoWorkspace.QN1[133]*acadoWorkspace.sbar[653] + acadoWorkspace.QN1[134]*acadoWorkspace.sbar[654] + acadoWorkspace.QN1[135]*acadoWorkspace.sbar[655] + acadoWorkspace.QN1[136]*acadoWorkspace.sbar[656] + acadoWorkspace.QN1[137]*acadoWorkspace.sbar[657] + acadoWorkspace.QN1[138]*acadoWorkspace.sbar[658] + acadoWorkspace.QN1[139]*acadoWorkspace.sbar[659] + acadoWorkspace.QN1[140]*acadoWorkspace.sbar[660] + acadoWorkspace.QN1[141]*acadoWorkspace.sbar[661] + acadoWorkspace.QN1[142]*acadoWorkspace.sbar[662] + acadoWorkspace.QDy[660];
acadoWorkspace.w1[11] = + acadoWorkspace.QN1[143]*acadoWorkspace.sbar[650] + acadoWorkspace.QN1[144]*acadoWorkspace.sbar[651] + acadoWorkspace.QN1[145]*acadoWorkspace.sbar[652] + acadoWorkspace.QN1[146]*acadoWorkspace.sbar[653] + acadoWorkspace.QN1[147]*acadoWorkspace.sbar[654] + acadoWorkspace.QN1[148]*acadoWorkspace.sbar[655] + acadoWorkspace.QN1[149]*acadoWorkspace.sbar[656] + acadoWorkspace.QN1[150]*acadoWorkspace.sbar[657] + acadoWorkspace.QN1[151]*acadoWorkspace.sbar[658] + acadoWorkspace.QN1[152]*acadoWorkspace.sbar[659] + acadoWorkspace.QN1[153]*acadoWorkspace.sbar[660] + acadoWorkspace.QN1[154]*acadoWorkspace.sbar[661] + acadoWorkspace.QN1[155]*acadoWorkspace.sbar[662] + acadoWorkspace.QDy[661];
acadoWorkspace.w1[12] = + acadoWorkspace.QN1[156]*acadoWorkspace.sbar[650] + acadoWorkspace.QN1[157]*acadoWorkspace.sbar[651] + acadoWorkspace.QN1[158]*acadoWorkspace.sbar[652] + acadoWorkspace.QN1[159]*acadoWorkspace.sbar[653] + acadoWorkspace.QN1[160]*acadoWorkspace.sbar[654] + acadoWorkspace.QN1[161]*acadoWorkspace.sbar[655] + acadoWorkspace.QN1[162]*acadoWorkspace.sbar[656] + acadoWorkspace.QN1[163]*acadoWorkspace.sbar[657] + acadoWorkspace.QN1[164]*acadoWorkspace.sbar[658] + acadoWorkspace.QN1[165]*acadoWorkspace.sbar[659] + acadoWorkspace.QN1[166]*acadoWorkspace.sbar[660] + acadoWorkspace.QN1[167]*acadoWorkspace.sbar[661] + acadoWorkspace.QN1[168]*acadoWorkspace.sbar[662] + acadoWorkspace.QDy[662];
acado_macBTw1( &(acadoWorkspace.evGu[ 1911 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 147 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 8281 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 637 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 8281 ]), &(acadoWorkspace.sbar[ 637 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1872 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 144 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 8112 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 624 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 8112 ]), &(acadoWorkspace.sbar[ 624 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1833 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 141 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 7943 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 611 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 7943 ]), &(acadoWorkspace.sbar[ 611 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1794 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 138 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 7774 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 598 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 7774 ]), &(acadoWorkspace.sbar[ 598 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1755 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 135 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 7605 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 585 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 7605 ]), &(acadoWorkspace.sbar[ 585 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1716 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 132 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 7436 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 572 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 7436 ]), &(acadoWorkspace.sbar[ 572 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1677 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 129 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 7267 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 559 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 7267 ]), &(acadoWorkspace.sbar[ 559 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1638 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 126 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 7098 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 546 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 7098 ]), &(acadoWorkspace.sbar[ 546 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1599 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 123 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 6929 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 533 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 6929 ]), &(acadoWorkspace.sbar[ 533 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1560 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 120 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 6760 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 520 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 6760 ]), &(acadoWorkspace.sbar[ 520 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1521 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 117 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 6591 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 507 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 6591 ]), &(acadoWorkspace.sbar[ 507 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1482 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 114 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 6422 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 494 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 6422 ]), &(acadoWorkspace.sbar[ 494 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1443 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 111 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 6253 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 481 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 6253 ]), &(acadoWorkspace.sbar[ 481 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1404 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 108 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 6084 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 468 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 6084 ]), &(acadoWorkspace.sbar[ 468 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1365 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 105 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 5915 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 455 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 5915 ]), &(acadoWorkspace.sbar[ 455 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1326 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 102 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 5746 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 442 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 5746 ]), &(acadoWorkspace.sbar[ 442 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1287 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 99 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 5577 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 429 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 5577 ]), &(acadoWorkspace.sbar[ 429 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1248 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 96 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 5408 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 416 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 5408 ]), &(acadoWorkspace.sbar[ 416 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1209 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 93 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 5239 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 403 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 5239 ]), &(acadoWorkspace.sbar[ 403 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1170 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 90 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 5070 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 390 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 5070 ]), &(acadoWorkspace.sbar[ 390 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1131 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 87 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4901 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 377 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 4901 ]), &(acadoWorkspace.sbar[ 377 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1092 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 84 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4732 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 364 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 4732 ]), &(acadoWorkspace.sbar[ 364 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1053 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 81 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4563 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 351 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 4563 ]), &(acadoWorkspace.sbar[ 351 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1014 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 78 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4394 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 338 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 4394 ]), &(acadoWorkspace.sbar[ 338 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 975 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 75 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4225 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 325 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 4225 ]), &(acadoWorkspace.sbar[ 325 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 936 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 72 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4056 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 312 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 4056 ]), &(acadoWorkspace.sbar[ 312 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 897 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 69 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3887 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 299 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 3887 ]), &(acadoWorkspace.sbar[ 299 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 858 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 66 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3718 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 286 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 3718 ]), &(acadoWorkspace.sbar[ 286 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 819 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 63 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3549 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 273 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 3549 ]), &(acadoWorkspace.sbar[ 273 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 780 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 60 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3380 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 260 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 3380 ]), &(acadoWorkspace.sbar[ 260 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 741 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 57 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3211 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 247 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 3211 ]), &(acadoWorkspace.sbar[ 247 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 702 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 54 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3042 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 234 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 3042 ]), &(acadoWorkspace.sbar[ 234 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 663 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 51 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2873 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 221 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2873 ]), &(acadoWorkspace.sbar[ 221 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 624 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2704 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 208 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2704 ]), &(acadoWorkspace.sbar[ 208 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 585 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 45 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2535 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 195 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2535 ]), &(acadoWorkspace.sbar[ 195 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 546 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 42 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2366 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 182 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2366 ]), &(acadoWorkspace.sbar[ 182 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 507 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 39 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2197 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 169 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2197 ]), &(acadoWorkspace.sbar[ 169 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 468 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2028 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 156 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2028 ]), &(acadoWorkspace.sbar[ 156 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 429 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 33 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1859 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 143 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1859 ]), &(acadoWorkspace.sbar[ 143 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 390 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1690 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 130 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1690 ]), &(acadoWorkspace.sbar[ 130 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 351 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 27 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1521 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 117 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1521 ]), &(acadoWorkspace.sbar[ 117 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 312 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1352 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 104 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1352 ]), &(acadoWorkspace.sbar[ 104 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 273 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 21 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1183 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 91 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1183 ]), &(acadoWorkspace.sbar[ 91 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 234 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1014 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 78 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1014 ]), &(acadoWorkspace.sbar[ 78 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 195 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 15 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 845 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 65 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 845 ]), &(acadoWorkspace.sbar[ 65 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 156 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 676 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 52 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 676 ]), &(acadoWorkspace.sbar[ 52 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 117 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 9 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 507 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 39 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 507 ]), &(acadoWorkspace.sbar[ 39 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 78 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 338 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 26 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 338 ]), &(acadoWorkspace.sbar[ 26 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 39 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 169 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 13 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 169 ]), &(acadoWorkspace.sbar[ 13 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );


}

void acado_expand(  )
{
int lRun1;
for (lRun1 = 0; lRun1 < 150; ++lRun1)
acadoVariables.u[lRun1] += acadoWorkspace.x[lRun1];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acadoWorkspace.sbar[9] = acadoWorkspace.Dx0[9];
acadoWorkspace.sbar[10] = acadoWorkspace.Dx0[10];
acadoWorkspace.sbar[11] = acadoWorkspace.Dx0[11];
acadoWorkspace.sbar[12] = acadoWorkspace.Dx0[12];
for (lRun1 = 0; lRun1 < 650; ++lRun1)
acadoWorkspace.sbar[lRun1 + 13] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 13 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 169 ]), &(acadoWorkspace.evGu[ 39 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.sbar[ 13 ]), &(acadoWorkspace.sbar[ 26 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 338 ]), &(acadoWorkspace.evGu[ 78 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 26 ]), &(acadoWorkspace.sbar[ 39 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 507 ]), &(acadoWorkspace.evGu[ 117 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.sbar[ 52 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 676 ]), &(acadoWorkspace.evGu[ 156 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 52 ]), &(acadoWorkspace.sbar[ 65 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 845 ]), &(acadoWorkspace.evGu[ 195 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.sbar[ 65 ]), &(acadoWorkspace.sbar[ 78 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1014 ]), &(acadoWorkspace.evGu[ 234 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.sbar[ 91 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1183 ]), &(acadoWorkspace.evGu[ 273 ]), &(acadoWorkspace.x[ 21 ]), &(acadoWorkspace.sbar[ 91 ]), &(acadoWorkspace.sbar[ 104 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1352 ]), &(acadoWorkspace.evGu[ 312 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 104 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1521 ]), &(acadoWorkspace.evGu[ 351 ]), &(acadoWorkspace.x[ 27 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 130 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1690 ]), &(acadoWorkspace.evGu[ 390 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 130 ]), &(acadoWorkspace.sbar[ 143 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1859 ]), &(acadoWorkspace.evGu[ 429 ]), &(acadoWorkspace.x[ 33 ]), &(acadoWorkspace.sbar[ 143 ]), &(acadoWorkspace.sbar[ 156 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2028 ]), &(acadoWorkspace.evGu[ 468 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.sbar[ 169 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2197 ]), &(acadoWorkspace.evGu[ 507 ]), &(acadoWorkspace.x[ 39 ]), &(acadoWorkspace.sbar[ 169 ]), &(acadoWorkspace.sbar[ 182 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2366 ]), &(acadoWorkspace.evGu[ 546 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.sbar[ 182 ]), &(acadoWorkspace.sbar[ 195 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2535 ]), &(acadoWorkspace.evGu[ 585 ]), &(acadoWorkspace.x[ 45 ]), &(acadoWorkspace.sbar[ 195 ]), &(acadoWorkspace.sbar[ 208 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2704 ]), &(acadoWorkspace.evGu[ 624 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 208 ]), &(acadoWorkspace.sbar[ 221 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2873 ]), &(acadoWorkspace.evGu[ 663 ]), &(acadoWorkspace.x[ 51 ]), &(acadoWorkspace.sbar[ 221 ]), &(acadoWorkspace.sbar[ 234 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3042 ]), &(acadoWorkspace.evGu[ 702 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.sbar[ 234 ]), &(acadoWorkspace.sbar[ 247 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3211 ]), &(acadoWorkspace.evGu[ 741 ]), &(acadoWorkspace.x[ 57 ]), &(acadoWorkspace.sbar[ 247 ]), &(acadoWorkspace.sbar[ 260 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3380 ]), &(acadoWorkspace.evGu[ 780 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.sbar[ 260 ]), &(acadoWorkspace.sbar[ 273 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3549 ]), &(acadoWorkspace.evGu[ 819 ]), &(acadoWorkspace.x[ 63 ]), &(acadoWorkspace.sbar[ 273 ]), &(acadoWorkspace.sbar[ 286 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3718 ]), &(acadoWorkspace.evGu[ 858 ]), &(acadoWorkspace.x[ 66 ]), &(acadoWorkspace.sbar[ 286 ]), &(acadoWorkspace.sbar[ 299 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3887 ]), &(acadoWorkspace.evGu[ 897 ]), &(acadoWorkspace.x[ 69 ]), &(acadoWorkspace.sbar[ 299 ]), &(acadoWorkspace.sbar[ 312 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4056 ]), &(acadoWorkspace.evGu[ 936 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.sbar[ 312 ]), &(acadoWorkspace.sbar[ 325 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4225 ]), &(acadoWorkspace.evGu[ 975 ]), &(acadoWorkspace.x[ 75 ]), &(acadoWorkspace.sbar[ 325 ]), &(acadoWorkspace.sbar[ 338 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4394 ]), &(acadoWorkspace.evGu[ 1014 ]), &(acadoWorkspace.x[ 78 ]), &(acadoWorkspace.sbar[ 338 ]), &(acadoWorkspace.sbar[ 351 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4563 ]), &(acadoWorkspace.evGu[ 1053 ]), &(acadoWorkspace.x[ 81 ]), &(acadoWorkspace.sbar[ 351 ]), &(acadoWorkspace.sbar[ 364 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4732 ]), &(acadoWorkspace.evGu[ 1092 ]), &(acadoWorkspace.x[ 84 ]), &(acadoWorkspace.sbar[ 364 ]), &(acadoWorkspace.sbar[ 377 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4901 ]), &(acadoWorkspace.evGu[ 1131 ]), &(acadoWorkspace.x[ 87 ]), &(acadoWorkspace.sbar[ 377 ]), &(acadoWorkspace.sbar[ 390 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 5070 ]), &(acadoWorkspace.evGu[ 1170 ]), &(acadoWorkspace.x[ 90 ]), &(acadoWorkspace.sbar[ 390 ]), &(acadoWorkspace.sbar[ 403 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 5239 ]), &(acadoWorkspace.evGu[ 1209 ]), &(acadoWorkspace.x[ 93 ]), &(acadoWorkspace.sbar[ 403 ]), &(acadoWorkspace.sbar[ 416 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 5408 ]), &(acadoWorkspace.evGu[ 1248 ]), &(acadoWorkspace.x[ 96 ]), &(acadoWorkspace.sbar[ 416 ]), &(acadoWorkspace.sbar[ 429 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 5577 ]), &(acadoWorkspace.evGu[ 1287 ]), &(acadoWorkspace.x[ 99 ]), &(acadoWorkspace.sbar[ 429 ]), &(acadoWorkspace.sbar[ 442 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 5746 ]), &(acadoWorkspace.evGu[ 1326 ]), &(acadoWorkspace.x[ 102 ]), &(acadoWorkspace.sbar[ 442 ]), &(acadoWorkspace.sbar[ 455 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 5915 ]), &(acadoWorkspace.evGu[ 1365 ]), &(acadoWorkspace.x[ 105 ]), &(acadoWorkspace.sbar[ 455 ]), &(acadoWorkspace.sbar[ 468 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 6084 ]), &(acadoWorkspace.evGu[ 1404 ]), &(acadoWorkspace.x[ 108 ]), &(acadoWorkspace.sbar[ 468 ]), &(acadoWorkspace.sbar[ 481 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 6253 ]), &(acadoWorkspace.evGu[ 1443 ]), &(acadoWorkspace.x[ 111 ]), &(acadoWorkspace.sbar[ 481 ]), &(acadoWorkspace.sbar[ 494 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 6422 ]), &(acadoWorkspace.evGu[ 1482 ]), &(acadoWorkspace.x[ 114 ]), &(acadoWorkspace.sbar[ 494 ]), &(acadoWorkspace.sbar[ 507 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 6591 ]), &(acadoWorkspace.evGu[ 1521 ]), &(acadoWorkspace.x[ 117 ]), &(acadoWorkspace.sbar[ 507 ]), &(acadoWorkspace.sbar[ 520 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 6760 ]), &(acadoWorkspace.evGu[ 1560 ]), &(acadoWorkspace.x[ 120 ]), &(acadoWorkspace.sbar[ 520 ]), &(acadoWorkspace.sbar[ 533 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 6929 ]), &(acadoWorkspace.evGu[ 1599 ]), &(acadoWorkspace.x[ 123 ]), &(acadoWorkspace.sbar[ 533 ]), &(acadoWorkspace.sbar[ 546 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 7098 ]), &(acadoWorkspace.evGu[ 1638 ]), &(acadoWorkspace.x[ 126 ]), &(acadoWorkspace.sbar[ 546 ]), &(acadoWorkspace.sbar[ 559 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 7267 ]), &(acadoWorkspace.evGu[ 1677 ]), &(acadoWorkspace.x[ 129 ]), &(acadoWorkspace.sbar[ 559 ]), &(acadoWorkspace.sbar[ 572 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 7436 ]), &(acadoWorkspace.evGu[ 1716 ]), &(acadoWorkspace.x[ 132 ]), &(acadoWorkspace.sbar[ 572 ]), &(acadoWorkspace.sbar[ 585 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 7605 ]), &(acadoWorkspace.evGu[ 1755 ]), &(acadoWorkspace.x[ 135 ]), &(acadoWorkspace.sbar[ 585 ]), &(acadoWorkspace.sbar[ 598 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 7774 ]), &(acadoWorkspace.evGu[ 1794 ]), &(acadoWorkspace.x[ 138 ]), &(acadoWorkspace.sbar[ 598 ]), &(acadoWorkspace.sbar[ 611 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 7943 ]), &(acadoWorkspace.evGu[ 1833 ]), &(acadoWorkspace.x[ 141 ]), &(acadoWorkspace.sbar[ 611 ]), &(acadoWorkspace.sbar[ 624 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 8112 ]), &(acadoWorkspace.evGu[ 1872 ]), &(acadoWorkspace.x[ 144 ]), &(acadoWorkspace.sbar[ 624 ]), &(acadoWorkspace.sbar[ 637 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 8281 ]), &(acadoWorkspace.evGu[ 1911 ]), &(acadoWorkspace.x[ 147 ]), &(acadoWorkspace.sbar[ 637 ]), &(acadoWorkspace.sbar[ 650 ]) );
for (lRun1 = 0; lRun1 < 663; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 50; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 13];
acadoWorkspace.state[1] = acadoVariables.x[index * 13 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 13 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 13 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 13 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 13 + 5];
acadoWorkspace.state[6] = acadoVariables.x[index * 13 + 6];
acadoWorkspace.state[7] = acadoVariables.x[index * 13 + 7];
acadoWorkspace.state[8] = acadoVariables.x[index * 13 + 8];
acadoWorkspace.state[9] = acadoVariables.x[index * 13 + 9];
acadoWorkspace.state[10] = acadoVariables.x[index * 13 + 10];
acadoWorkspace.state[11] = acadoVariables.x[index * 13 + 11];
acadoWorkspace.state[12] = acadoVariables.x[index * 13 + 12];
acadoWorkspace.state[221] = acadoVariables.u[index * 3];
acadoWorkspace.state[222] = acadoVariables.u[index * 3 + 1];
acadoWorkspace.state[223] = acadoVariables.u[index * 3 + 2];
acadoWorkspace.state[224] = acadoVariables.od[index * 3];
acadoWorkspace.state[225] = acadoVariables.od[index * 3 + 1];
acadoWorkspace.state[226] = acadoVariables.od[index * 3 + 2];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 13 + 13] = acadoWorkspace.state[0];
acadoVariables.x[index * 13 + 14] = acadoWorkspace.state[1];
acadoVariables.x[index * 13 + 15] = acadoWorkspace.state[2];
acadoVariables.x[index * 13 + 16] = acadoWorkspace.state[3];
acadoVariables.x[index * 13 + 17] = acadoWorkspace.state[4];
acadoVariables.x[index * 13 + 18] = acadoWorkspace.state[5];
acadoVariables.x[index * 13 + 19] = acadoWorkspace.state[6];
acadoVariables.x[index * 13 + 20] = acadoWorkspace.state[7];
acadoVariables.x[index * 13 + 21] = acadoWorkspace.state[8];
acadoVariables.x[index * 13 + 22] = acadoWorkspace.state[9];
acadoVariables.x[index * 13 + 23] = acadoWorkspace.state[10];
acadoVariables.x[index * 13 + 24] = acadoWorkspace.state[11];
acadoVariables.x[index * 13 + 25] = acadoWorkspace.state[12];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 50; ++index)
{
acadoVariables.x[index * 13] = acadoVariables.x[index * 13 + 13];
acadoVariables.x[index * 13 + 1] = acadoVariables.x[index * 13 + 14];
acadoVariables.x[index * 13 + 2] = acadoVariables.x[index * 13 + 15];
acadoVariables.x[index * 13 + 3] = acadoVariables.x[index * 13 + 16];
acadoVariables.x[index * 13 + 4] = acadoVariables.x[index * 13 + 17];
acadoVariables.x[index * 13 + 5] = acadoVariables.x[index * 13 + 18];
acadoVariables.x[index * 13 + 6] = acadoVariables.x[index * 13 + 19];
acadoVariables.x[index * 13 + 7] = acadoVariables.x[index * 13 + 20];
acadoVariables.x[index * 13 + 8] = acadoVariables.x[index * 13 + 21];
acadoVariables.x[index * 13 + 9] = acadoVariables.x[index * 13 + 22];
acadoVariables.x[index * 13 + 10] = acadoVariables.x[index * 13 + 23];
acadoVariables.x[index * 13 + 11] = acadoVariables.x[index * 13 + 24];
acadoVariables.x[index * 13 + 12] = acadoVariables.x[index * 13 + 25];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[650] = xEnd[0];
acadoVariables.x[651] = xEnd[1];
acadoVariables.x[652] = xEnd[2];
acadoVariables.x[653] = xEnd[3];
acadoVariables.x[654] = xEnd[4];
acadoVariables.x[655] = xEnd[5];
acadoVariables.x[656] = xEnd[6];
acadoVariables.x[657] = xEnd[7];
acadoVariables.x[658] = xEnd[8];
acadoVariables.x[659] = xEnd[9];
acadoVariables.x[660] = xEnd[10];
acadoVariables.x[661] = xEnd[11];
acadoVariables.x[662] = xEnd[12];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[650];
acadoWorkspace.state[1] = acadoVariables.x[651];
acadoWorkspace.state[2] = acadoVariables.x[652];
acadoWorkspace.state[3] = acadoVariables.x[653];
acadoWorkspace.state[4] = acadoVariables.x[654];
acadoWorkspace.state[5] = acadoVariables.x[655];
acadoWorkspace.state[6] = acadoVariables.x[656];
acadoWorkspace.state[7] = acadoVariables.x[657];
acadoWorkspace.state[8] = acadoVariables.x[658];
acadoWorkspace.state[9] = acadoVariables.x[659];
acadoWorkspace.state[10] = acadoVariables.x[660];
acadoWorkspace.state[11] = acadoVariables.x[661];
acadoWorkspace.state[12] = acadoVariables.x[662];
if (uEnd != 0)
{
acadoWorkspace.state[221] = uEnd[0];
acadoWorkspace.state[222] = uEnd[1];
acadoWorkspace.state[223] = uEnd[2];
}
else
{
acadoWorkspace.state[221] = acadoVariables.u[147];
acadoWorkspace.state[222] = acadoVariables.u[148];
acadoWorkspace.state[223] = acadoVariables.u[149];
}
acadoWorkspace.state[224] = acadoVariables.od[150];
acadoWorkspace.state[225] = acadoVariables.od[151];
acadoWorkspace.state[226] = acadoVariables.od[152];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[650] = acadoWorkspace.state[0];
acadoVariables.x[651] = acadoWorkspace.state[1];
acadoVariables.x[652] = acadoWorkspace.state[2];
acadoVariables.x[653] = acadoWorkspace.state[3];
acadoVariables.x[654] = acadoWorkspace.state[4];
acadoVariables.x[655] = acadoWorkspace.state[5];
acadoVariables.x[656] = acadoWorkspace.state[6];
acadoVariables.x[657] = acadoWorkspace.state[7];
acadoVariables.x[658] = acadoWorkspace.state[8];
acadoVariables.x[659] = acadoWorkspace.state[9];
acadoVariables.x[660] = acadoWorkspace.state[10];
acadoVariables.x[661] = acadoWorkspace.state[11];
acadoVariables.x[662] = acadoWorkspace.state[12];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 49; ++index)
{
acadoVariables.u[index * 3] = acadoVariables.u[index * 3 + 3];
acadoVariables.u[index * 3 + 1] = acadoVariables.u[index * 3 + 4];
acadoVariables.u[index * 3 + 2] = acadoVariables.u[index * 3 + 5];
}

if (uEnd != 0)
{
acadoVariables.u[147] = uEnd[0];
acadoVariables.u[148] = uEnd[1];
acadoVariables.u[149] = uEnd[2];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79] + acadoWorkspace.g[80]*acadoWorkspace.x[80] + acadoWorkspace.g[81]*acadoWorkspace.x[81] + acadoWorkspace.g[82]*acadoWorkspace.x[82] + acadoWorkspace.g[83]*acadoWorkspace.x[83] + acadoWorkspace.g[84]*acadoWorkspace.x[84] + acadoWorkspace.g[85]*acadoWorkspace.x[85] + acadoWorkspace.g[86]*acadoWorkspace.x[86] + acadoWorkspace.g[87]*acadoWorkspace.x[87] + acadoWorkspace.g[88]*acadoWorkspace.x[88] + acadoWorkspace.g[89]*acadoWorkspace.x[89] + acadoWorkspace.g[90]*acadoWorkspace.x[90] + acadoWorkspace.g[91]*acadoWorkspace.x[91] + acadoWorkspace.g[92]*acadoWorkspace.x[92] + acadoWorkspace.g[93]*acadoWorkspace.x[93] + acadoWorkspace.g[94]*acadoWorkspace.x[94] + acadoWorkspace.g[95]*acadoWorkspace.x[95] + acadoWorkspace.g[96]*acadoWorkspace.x[96] + acadoWorkspace.g[97]*acadoWorkspace.x[97] + acadoWorkspace.g[98]*acadoWorkspace.x[98] + acadoWorkspace.g[99]*acadoWorkspace.x[99] + acadoWorkspace.g[100]*acadoWorkspace.x[100] + acadoWorkspace.g[101]*acadoWorkspace.x[101] + acadoWorkspace.g[102]*acadoWorkspace.x[102] + acadoWorkspace.g[103]*acadoWorkspace.x[103] + acadoWorkspace.g[104]*acadoWorkspace.x[104] + acadoWorkspace.g[105]*acadoWorkspace.x[105] + acadoWorkspace.g[106]*acadoWorkspace.x[106] + acadoWorkspace.g[107]*acadoWorkspace.x[107] + acadoWorkspace.g[108]*acadoWorkspace.x[108] + acadoWorkspace.g[109]*acadoWorkspace.x[109] + acadoWorkspace.g[110]*acadoWorkspace.x[110] + acadoWorkspace.g[111]*acadoWorkspace.x[111] + acadoWorkspace.g[112]*acadoWorkspace.x[112] + acadoWorkspace.g[113]*acadoWorkspace.x[113] + acadoWorkspace.g[114]*acadoWorkspace.x[114] + acadoWorkspace.g[115]*acadoWorkspace.x[115] + acadoWorkspace.g[116]*acadoWorkspace.x[116] + acadoWorkspace.g[117]*acadoWorkspace.x[117] + acadoWorkspace.g[118]*acadoWorkspace.x[118] + acadoWorkspace.g[119]*acadoWorkspace.x[119] + acadoWorkspace.g[120]*acadoWorkspace.x[120] + acadoWorkspace.g[121]*acadoWorkspace.x[121] + acadoWorkspace.g[122]*acadoWorkspace.x[122] + acadoWorkspace.g[123]*acadoWorkspace.x[123] + acadoWorkspace.g[124]*acadoWorkspace.x[124] + acadoWorkspace.g[125]*acadoWorkspace.x[125] + acadoWorkspace.g[126]*acadoWorkspace.x[126] + acadoWorkspace.g[127]*acadoWorkspace.x[127] + acadoWorkspace.g[128]*acadoWorkspace.x[128] + acadoWorkspace.g[129]*acadoWorkspace.x[129] + acadoWorkspace.g[130]*acadoWorkspace.x[130] + acadoWorkspace.g[131]*acadoWorkspace.x[131] + acadoWorkspace.g[132]*acadoWorkspace.x[132] + acadoWorkspace.g[133]*acadoWorkspace.x[133] + acadoWorkspace.g[134]*acadoWorkspace.x[134] + acadoWorkspace.g[135]*acadoWorkspace.x[135] + acadoWorkspace.g[136]*acadoWorkspace.x[136] + acadoWorkspace.g[137]*acadoWorkspace.x[137] + acadoWorkspace.g[138]*acadoWorkspace.x[138] + acadoWorkspace.g[139]*acadoWorkspace.x[139] + acadoWorkspace.g[140]*acadoWorkspace.x[140] + acadoWorkspace.g[141]*acadoWorkspace.x[141] + acadoWorkspace.g[142]*acadoWorkspace.x[142] + acadoWorkspace.g[143]*acadoWorkspace.x[143] + acadoWorkspace.g[144]*acadoWorkspace.x[144] + acadoWorkspace.g[145]*acadoWorkspace.x[145] + acadoWorkspace.g[146]*acadoWorkspace.x[146] + acadoWorkspace.g[147]*acadoWorkspace.x[147] + acadoWorkspace.g[148]*acadoWorkspace.x[148] + acadoWorkspace.g[149]*acadoWorkspace.x[149];
kkt = fabs( kkt );
for (index = 0; index < 150; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 16 */
real_t tmpDy[ 16 ];

/** Row vector of size: 13 */
real_t tmpDyN[ 13 ];

for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 13];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 13 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 13 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 13 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 13 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 13 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 13 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 13 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[lRun1 * 13 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[lRun1 * 13 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.x[lRun1 * 13 + 10];
acadoWorkspace.objValueIn[11] = acadoVariables.x[lRun1 * 13 + 11];
acadoWorkspace.objValueIn[12] = acadoVariables.x[lRun1 * 13 + 12];
acadoWorkspace.objValueIn[13] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.objValueIn[14] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[15] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 3];
acadoWorkspace.objValueIn[17] = acadoVariables.od[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[18] = acadoVariables.od[lRun1 * 3 + 2];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 16] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 16];
acadoWorkspace.Dy[lRun1 * 16 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 16 + 1];
acadoWorkspace.Dy[lRun1 * 16 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 16 + 2];
acadoWorkspace.Dy[lRun1 * 16 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 16 + 3];
acadoWorkspace.Dy[lRun1 * 16 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 16 + 4];
acadoWorkspace.Dy[lRun1 * 16 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 16 + 5];
acadoWorkspace.Dy[lRun1 * 16 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 16 + 6];
acadoWorkspace.Dy[lRun1 * 16 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 16 + 7];
acadoWorkspace.Dy[lRun1 * 16 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 16 + 8];
acadoWorkspace.Dy[lRun1 * 16 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 16 + 9];
acadoWorkspace.Dy[lRun1 * 16 + 10] = acadoWorkspace.objValueOut[10] - acadoVariables.y[lRun1 * 16 + 10];
acadoWorkspace.Dy[lRun1 * 16 + 11] = acadoWorkspace.objValueOut[11] - acadoVariables.y[lRun1 * 16 + 11];
acadoWorkspace.Dy[lRun1 * 16 + 12] = acadoWorkspace.objValueOut[12] - acadoVariables.y[lRun1 * 16 + 12];
acadoWorkspace.Dy[lRun1 * 16 + 13] = acadoWorkspace.objValueOut[13] - acadoVariables.y[lRun1 * 16 + 13];
acadoWorkspace.Dy[lRun1 * 16 + 14] = acadoWorkspace.objValueOut[14] - acadoVariables.y[lRun1 * 16 + 14];
acadoWorkspace.Dy[lRun1 * 16 + 15] = acadoWorkspace.objValueOut[15] - acadoVariables.y[lRun1 * 16 + 15];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[650];
acadoWorkspace.objValueIn[1] = acadoVariables.x[651];
acadoWorkspace.objValueIn[2] = acadoVariables.x[652];
acadoWorkspace.objValueIn[3] = acadoVariables.x[653];
acadoWorkspace.objValueIn[4] = acadoVariables.x[654];
acadoWorkspace.objValueIn[5] = acadoVariables.x[655];
acadoWorkspace.objValueIn[6] = acadoVariables.x[656];
acadoWorkspace.objValueIn[7] = acadoVariables.x[657];
acadoWorkspace.objValueIn[8] = acadoVariables.x[658];
acadoWorkspace.objValueIn[9] = acadoVariables.x[659];
acadoWorkspace.objValueIn[10] = acadoVariables.x[660];
acadoWorkspace.objValueIn[11] = acadoVariables.x[661];
acadoWorkspace.objValueIn[12] = acadoVariables.x[662];
acadoWorkspace.objValueIn[13] = acadoVariables.od[150];
acadoWorkspace.objValueIn[14] = acadoVariables.od[151];
acadoWorkspace.objValueIn[15] = acadoVariables.od[152];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6] - acadoVariables.yN[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7] - acadoVariables.yN[7];
acadoWorkspace.DyN[8] = acadoWorkspace.objValueOut[8] - acadoVariables.yN[8];
acadoWorkspace.DyN[9] = acadoWorkspace.objValueOut[9] - acadoVariables.yN[9];
acadoWorkspace.DyN[10] = acadoWorkspace.objValueOut[10] - acadoVariables.yN[10];
acadoWorkspace.DyN[11] = acadoWorkspace.objValueOut[11] - acadoVariables.yN[11];
acadoWorkspace.DyN[12] = acadoWorkspace.objValueOut[12] - acadoVariables.yN[12];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 16]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 16 + 1]*acadoVariables.W[17];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 16 + 2]*acadoVariables.W[34];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 16 + 3]*acadoVariables.W[51];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 16 + 4]*acadoVariables.W[68];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 16 + 5]*acadoVariables.W[85];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 16 + 6]*acadoVariables.W[102];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 16 + 7]*acadoVariables.W[119];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 16 + 8]*acadoVariables.W[136];
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 16 + 9]*acadoVariables.W[153];
tmpDy[10] = + acadoWorkspace.Dy[lRun1 * 16 + 10]*acadoVariables.W[170];
tmpDy[11] = + acadoWorkspace.Dy[lRun1 * 16 + 11]*acadoVariables.W[187];
tmpDy[12] = + acadoWorkspace.Dy[lRun1 * 16 + 12]*acadoVariables.W[204];
tmpDy[13] = + acadoWorkspace.Dy[lRun1 * 16 + 13]*acadoVariables.W[221];
tmpDy[14] = + acadoWorkspace.Dy[lRun1 * 16 + 14]*acadoVariables.W[238];
tmpDy[15] = + acadoWorkspace.Dy[lRun1 * 16 + 15]*acadoVariables.W[255];
objVal += + acadoWorkspace.Dy[lRun1 * 16]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 16 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 16 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 16 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 16 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 16 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 16 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 16 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 16 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 16 + 9]*tmpDy[9] + acadoWorkspace.Dy[lRun1 * 16 + 10]*tmpDy[10] + acadoWorkspace.Dy[lRun1 * 16 + 11]*tmpDy[11] + acadoWorkspace.Dy[lRun1 * 16 + 12]*tmpDy[12] + acadoWorkspace.Dy[lRun1 * 16 + 13]*tmpDy[13] + acadoWorkspace.Dy[lRun1 * 16 + 14]*tmpDy[14] + acadoWorkspace.Dy[lRun1 * 16 + 15]*tmpDy[15];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[14];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[28];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[42];
tmpDyN[4] = + acadoWorkspace.DyN[4]*acadoVariables.WN[56];
tmpDyN[5] = + acadoWorkspace.DyN[5]*acadoVariables.WN[70];
tmpDyN[6] = + acadoWorkspace.DyN[6]*acadoVariables.WN[84];
tmpDyN[7] = + acadoWorkspace.DyN[7]*acadoVariables.WN[98];
tmpDyN[8] = + acadoWorkspace.DyN[8]*acadoVariables.WN[112];
tmpDyN[9] = + acadoWorkspace.DyN[9]*acadoVariables.WN[126];
tmpDyN[10] = + acadoWorkspace.DyN[10]*acadoVariables.WN[140];
tmpDyN[11] = + acadoWorkspace.DyN[11]*acadoVariables.WN[154];
tmpDyN[12] = + acadoWorkspace.DyN[12]*acadoVariables.WN[168];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5] + acadoWorkspace.DyN[6]*tmpDyN[6] + acadoWorkspace.DyN[7]*tmpDyN[7] + acadoWorkspace.DyN[8]*tmpDyN[8] + acadoWorkspace.DyN[9]*tmpDyN[9] + acadoWorkspace.DyN[10]*tmpDyN[10] + acadoWorkspace.DyN[11]*tmpDyN[11] + acadoWorkspace.DyN[12]*tmpDyN[12];

objVal *= 0.5;
return objVal;
}

