/* Produced by CVXGEN, 2017-11-22 15:58:00 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: ldl.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
/* Be sure to place ldl_solve first, so storage schemes are defined by it. */
void ldl_solve(double *target, double *var) {
  int i;
  /* Find var = (L*diag(work.d)*L') \ target, then unpermute. */
  /* Answer goes into var. */
  /* Forward substitution. */
  /* Include permutation as we retrieve from target. Use v so we can unpermute */
  /* later. */
  work.v[0] = target[55];
  work.v[1] = target[56];
  work.v[2] = target[57];
  work.v[3] = target[58];
  work.v[4] = target[59];
  work.v[5] = target[60];
  work.v[6] = target[61];
  work.v[7] = target[62];
  work.v[8] = target[63];
  work.v[9] = target[64];
  work.v[10] = target[65];
  work.v[11] = target[66];
  work.v[12] = target[67];
  work.v[13] = target[68];
  work.v[14] = target[69];
  work.v[15] = target[70];
  work.v[16] = target[71];
  work.v[17] = target[72];
  work.v[18] = target[73];
  work.v[19] = target[74];
  work.v[20] = target[75];
  work.v[21] = target[76];
  work.v[22] = target[77];
  work.v[23] = target[78];
  work.v[24] = target[79];
  work.v[25] = target[80];
  work.v[26] = target[81];
  work.v[27] = target[82];
  work.v[28] = target[83];
  work.v[29] = target[84];
  work.v[30] = target[85];
  work.v[31] = target[86];
  work.v[32] = target[87];
  work.v[33] = target[88]-work.L[0]*work.v[0];
  work.v[34] = target[0]-work.L[1]*work.v[33];
  work.v[35] = target[89]-work.L[2]*work.v[1]-work.L[3]*work.v[34];
  work.v[36] = target[90]-work.L[4]*work.v[2]-work.L[5]*work.v[34]-work.L[6]*work.v[35];
  work.v[37] = target[91]-work.L[7]*work.v[3];
  work.v[38] = target[1]-work.L[8]*work.v[37];
  work.v[39] = target[92]-work.L[9]*work.v[4]-work.L[10]*work.v[38];
  work.v[40] = target[93]-work.L[11]*work.v[5]-work.L[12]*work.v[38]-work.L[13]*work.v[39];
  work.v[41] = target[94]-work.L[14]*work.v[6];
  work.v[42] = target[2]-work.L[15]*work.v[41];
  work.v[43] = target[95]-work.L[16]*work.v[7]-work.L[17]*work.v[42];
  work.v[44] = target[96]-work.L[18]*work.v[8]-work.L[19]*work.v[42]-work.L[20]*work.v[43];
  work.v[45] = target[97]-work.L[21]*work.v[9];
  work.v[46] = target[3]-work.L[22]*work.v[45];
  work.v[47] = target[98]-work.L[23]*work.v[10]-work.L[24]*work.v[46];
  work.v[48] = target[99]-work.L[25]*work.v[11]-work.L[26]*work.v[46]-work.L[27]*work.v[47];
  work.v[49] = target[100]-work.L[28]*work.v[12];
  work.v[50] = target[4]-work.L[29]*work.v[49];
  work.v[51] = target[101]-work.L[30]*work.v[13]-work.L[31]*work.v[50];
  work.v[52] = target[102]-work.L[32]*work.v[14]-work.L[33]*work.v[50]-work.L[34]*work.v[51];
  work.v[53] = target[103]-work.L[35]*work.v[15];
  work.v[54] = target[5]-work.L[36]*work.v[53];
  work.v[55] = target[104]-work.L[37]*work.v[16]-work.L[38]*work.v[54];
  work.v[56] = target[105]-work.L[39]*work.v[17]-work.L[40]*work.v[54]-work.L[41]*work.v[55];
  work.v[57] = target[106]-work.L[42]*work.v[18];
  work.v[58] = target[6]-work.L[43]*work.v[57];
  work.v[59] = target[107]-work.L[44]*work.v[19]-work.L[45]*work.v[58];
  work.v[60] = target[108]-work.L[46]*work.v[20]-work.L[47]*work.v[58]-work.L[48]*work.v[59];
  work.v[61] = target[109]-work.L[49]*work.v[21];
  work.v[62] = target[7]-work.L[50]*work.v[61];
  work.v[63] = target[110]-work.L[51]*work.v[22]-work.L[52]*work.v[62];
  work.v[64] = target[111]-work.L[53]*work.v[23]-work.L[54]*work.v[62]-work.L[55]*work.v[63];
  work.v[65] = target[112]-work.L[56]*work.v[24];
  work.v[66] = target[8]-work.L[57]*work.v[65];
  work.v[67] = target[113]-work.L[58]*work.v[25]-work.L[59]*work.v[66];
  work.v[68] = target[114]-work.L[60]*work.v[26]-work.L[61]*work.v[66]-work.L[62]*work.v[67];
  work.v[69] = target[115]-work.L[63]*work.v[27];
  work.v[70] = target[9]-work.L[64]*work.v[69];
  work.v[71] = target[116]-work.L[65]*work.v[28]-work.L[66]*work.v[70];
  work.v[72] = target[117]-work.L[67]*work.v[29]-work.L[68]*work.v[70]-work.L[69]*work.v[71];
  work.v[73] = target[118]-work.L[70]*work.v[30];
  work.v[74] = target[10]-work.L[71]*work.v[73];
  work.v[75] = target[119]-work.L[72]*work.v[31]-work.L[73]*work.v[74];
  work.v[76] = target[120]-work.L[74]*work.v[32]-work.L[75]*work.v[74]-work.L[76]*work.v[75];
  work.v[77] = target[121];
  work.v[78] = target[122];
  work.v[79] = target[123];
  work.v[80] = target[11]-work.L[77]*work.v[35]-work.L[78]*work.v[36]-work.L[79]*work.v[77]-work.L[80]*work.v[78]-work.L[81]*work.v[79];
  work.v[81] = target[52];
  work.v[82] = target[53]-work.L[82]*work.v[81];
  work.v[83] = target[21]-work.L[83]*work.v[75]-work.L[84]*work.v[76];
  work.v[84] = target[54]-work.L[85]*work.v[81]-work.L[86]*work.v[82];
  work.v[85] = target[49];
  work.v[86] = target[50];
  work.v[87] = target[20]-work.L[87]*work.v[71]-work.L[88]*work.v[72];
  work.v[88] = target[51];
  work.v[89] = target[151]-work.L[89]*work.v[81]-work.L[90]*work.v[82]-work.L[91]*work.v[83]-work.L[92]*work.v[84]-work.L[93]*work.v[85]-work.L[94]*work.v[86]-work.L[95]*work.v[88];
  work.v[90] = target[152]-work.L[96]*work.v[82]-work.L[97]*work.v[83]-work.L[98]*work.v[84]-work.L[99]*work.v[85]-work.L[100]*work.v[86]-work.L[101]*work.v[88]-work.L[102]*work.v[89];
  work.v[91] = target[153]-work.L[103]*work.v[83]-work.L[104]*work.v[84]-work.L[105]*work.v[85]-work.L[106]*work.v[86]-work.L[107]*work.v[88]-work.L[108]*work.v[89]-work.L[109]*work.v[90];
  work.v[92] = target[46];
  work.v[93] = target[47];
  work.v[94] = target[19]-work.L[110]*work.v[67]-work.L[111]*work.v[68];
  work.v[95] = target[48];
  work.v[96] = target[148]-work.L[112]*work.v[85]-work.L[113]*work.v[87]-work.L[114]*work.v[89]-work.L[115]*work.v[90]-work.L[116]*work.v[91]-work.L[117]*work.v[92]-work.L[118]*work.v[93]-work.L[119]*work.v[95];
  work.v[97] = target[149]-work.L[120]*work.v[86]-work.L[121]*work.v[87]-work.L[122]*work.v[89]-work.L[123]*work.v[90]-work.L[124]*work.v[91]-work.L[125]*work.v[92]-work.L[126]*work.v[93]-work.L[127]*work.v[95]-work.L[128]*work.v[96];
  work.v[98] = target[150]-work.L[129]*work.v[87]-work.L[130]*work.v[88]-work.L[131]*work.v[89]-work.L[132]*work.v[90]-work.L[133]*work.v[91]-work.L[134]*work.v[92]-work.L[135]*work.v[93]-work.L[136]*work.v[95]-work.L[137]*work.v[96]-work.L[138]*work.v[97];
  work.v[99] = target[43];
  work.v[100] = target[44];
  work.v[101] = target[18]-work.L[139]*work.v[63]-work.L[140]*work.v[64];
  work.v[102] = target[45];
  work.v[103] = target[145]-work.L[141]*work.v[92]-work.L[142]*work.v[94]-work.L[143]*work.v[96]-work.L[144]*work.v[97]-work.L[145]*work.v[98]-work.L[146]*work.v[99]-work.L[147]*work.v[100]-work.L[148]*work.v[102];
  work.v[104] = target[146]-work.L[149]*work.v[93]-work.L[150]*work.v[94]-work.L[151]*work.v[96]-work.L[152]*work.v[97]-work.L[153]*work.v[98]-work.L[154]*work.v[99]-work.L[155]*work.v[100]-work.L[156]*work.v[102]-work.L[157]*work.v[103];
  work.v[105] = target[147]-work.L[158]*work.v[94]-work.L[159]*work.v[95]-work.L[160]*work.v[96]-work.L[161]*work.v[97]-work.L[162]*work.v[98]-work.L[163]*work.v[99]-work.L[164]*work.v[100]-work.L[165]*work.v[102]-work.L[166]*work.v[103]-work.L[167]*work.v[104];
  work.v[106] = target[40];
  work.v[107] = target[41];
  work.v[108] = target[17]-work.L[168]*work.v[59]-work.L[169]*work.v[60];
  work.v[109] = target[42];
  work.v[110] = target[142]-work.L[170]*work.v[99]-work.L[171]*work.v[101]-work.L[172]*work.v[103]-work.L[173]*work.v[104]-work.L[174]*work.v[105]-work.L[175]*work.v[106]-work.L[176]*work.v[107]-work.L[177]*work.v[109];
  work.v[111] = target[143]-work.L[178]*work.v[100]-work.L[179]*work.v[101]-work.L[180]*work.v[103]-work.L[181]*work.v[104]-work.L[182]*work.v[105]-work.L[183]*work.v[106]-work.L[184]*work.v[107]-work.L[185]*work.v[109]-work.L[186]*work.v[110];
  work.v[112] = target[144]-work.L[187]*work.v[101]-work.L[188]*work.v[102]-work.L[189]*work.v[103]-work.L[190]*work.v[104]-work.L[191]*work.v[105]-work.L[192]*work.v[106]-work.L[193]*work.v[107]-work.L[194]*work.v[109]-work.L[195]*work.v[110]-work.L[196]*work.v[111];
  work.v[113] = target[37];
  work.v[114] = target[38];
  work.v[115] = target[16]-work.L[197]*work.v[55]-work.L[198]*work.v[56];
  work.v[116] = target[39];
  work.v[117] = target[139]-work.L[199]*work.v[106]-work.L[200]*work.v[108]-work.L[201]*work.v[110]-work.L[202]*work.v[111]-work.L[203]*work.v[112]-work.L[204]*work.v[113]-work.L[205]*work.v[114]-work.L[206]*work.v[116];
  work.v[118] = target[140]-work.L[207]*work.v[107]-work.L[208]*work.v[108]-work.L[209]*work.v[110]-work.L[210]*work.v[111]-work.L[211]*work.v[112]-work.L[212]*work.v[113]-work.L[213]*work.v[114]-work.L[214]*work.v[116]-work.L[215]*work.v[117];
  work.v[119] = target[141]-work.L[216]*work.v[108]-work.L[217]*work.v[109]-work.L[218]*work.v[110]-work.L[219]*work.v[111]-work.L[220]*work.v[112]-work.L[221]*work.v[113]-work.L[222]*work.v[114]-work.L[223]*work.v[116]-work.L[224]*work.v[117]-work.L[225]*work.v[118];
  work.v[120] = target[34];
  work.v[121] = target[35];
  work.v[122] = target[15]-work.L[226]*work.v[51]-work.L[227]*work.v[52];
  work.v[123] = target[36];
  work.v[124] = target[136]-work.L[228]*work.v[113]-work.L[229]*work.v[115]-work.L[230]*work.v[117]-work.L[231]*work.v[118]-work.L[232]*work.v[119]-work.L[233]*work.v[120]-work.L[234]*work.v[121]-work.L[235]*work.v[123];
  work.v[125] = target[137]-work.L[236]*work.v[114]-work.L[237]*work.v[115]-work.L[238]*work.v[117]-work.L[239]*work.v[118]-work.L[240]*work.v[119]-work.L[241]*work.v[120]-work.L[242]*work.v[121]-work.L[243]*work.v[123]-work.L[244]*work.v[124];
  work.v[126] = target[138]-work.L[245]*work.v[115]-work.L[246]*work.v[116]-work.L[247]*work.v[117]-work.L[248]*work.v[118]-work.L[249]*work.v[119]-work.L[250]*work.v[120]-work.L[251]*work.v[121]-work.L[252]*work.v[123]-work.L[253]*work.v[124]-work.L[254]*work.v[125];
  work.v[127] = target[31];
  work.v[128] = target[32];
  work.v[129] = target[14]-work.L[255]*work.v[47]-work.L[256]*work.v[48];
  work.v[130] = target[33];
  work.v[131] = target[133]-work.L[257]*work.v[120]-work.L[258]*work.v[122]-work.L[259]*work.v[124]-work.L[260]*work.v[125]-work.L[261]*work.v[126]-work.L[262]*work.v[127]-work.L[263]*work.v[128]-work.L[264]*work.v[130];
  work.v[132] = target[134]-work.L[265]*work.v[121]-work.L[266]*work.v[122]-work.L[267]*work.v[124]-work.L[268]*work.v[125]-work.L[269]*work.v[126]-work.L[270]*work.v[127]-work.L[271]*work.v[128]-work.L[272]*work.v[130]-work.L[273]*work.v[131];
  work.v[133] = target[135]-work.L[274]*work.v[122]-work.L[275]*work.v[123]-work.L[276]*work.v[124]-work.L[277]*work.v[125]-work.L[278]*work.v[126]-work.L[279]*work.v[127]-work.L[280]*work.v[128]-work.L[281]*work.v[130]-work.L[282]*work.v[131]-work.L[283]*work.v[132];
  work.v[134] = target[28];
  work.v[135] = target[29];
  work.v[136] = target[13]-work.L[284]*work.v[43]-work.L[285]*work.v[44];
  work.v[137] = target[30];
  work.v[138] = target[130]-work.L[286]*work.v[127]-work.L[287]*work.v[129]-work.L[288]*work.v[131]-work.L[289]*work.v[132]-work.L[290]*work.v[133]-work.L[291]*work.v[134]-work.L[292]*work.v[135]-work.L[293]*work.v[137];
  work.v[139] = target[131]-work.L[294]*work.v[128]-work.L[295]*work.v[129]-work.L[296]*work.v[131]-work.L[297]*work.v[132]-work.L[298]*work.v[133]-work.L[299]*work.v[134]-work.L[300]*work.v[135]-work.L[301]*work.v[137]-work.L[302]*work.v[138];
  work.v[140] = target[132]-work.L[303]*work.v[129]-work.L[304]*work.v[130]-work.L[305]*work.v[131]-work.L[306]*work.v[132]-work.L[307]*work.v[133]-work.L[308]*work.v[134]-work.L[309]*work.v[135]-work.L[310]*work.v[137]-work.L[311]*work.v[138]-work.L[312]*work.v[139];
  work.v[141] = target[25];
  work.v[142] = target[26];
  work.v[143] = target[12]-work.L[313]*work.v[39]-work.L[314]*work.v[40];
  work.v[144] = target[22]-work.L[315]*work.v[77]-work.L[316]*work.v[80];
  work.v[145] = target[23]-work.L[317]*work.v[78]-work.L[318]*work.v[80]-work.L[319]*work.v[144];
  work.v[146] = target[24]-work.L[320]*work.v[79]-work.L[321]*work.v[80]-work.L[322]*work.v[144]-work.L[323]*work.v[145];
  work.v[147] = target[27];
  work.v[148] = target[124]-work.L[324]*work.v[141]-work.L[325]*work.v[143]-work.L[326]*work.v[144]-work.L[327]*work.v[145]-work.L[328]*work.v[146];
  work.v[149] = target[125]-work.L[329]*work.v[142]-work.L[330]*work.v[143]-work.L[331]*work.v[144]-work.L[332]*work.v[145]-work.L[333]*work.v[146]-work.L[334]*work.v[148];
  work.v[150] = target[126]-work.L[335]*work.v[143]-work.L[336]*work.v[144]-work.L[337]*work.v[145]-work.L[338]*work.v[146]-work.L[339]*work.v[147]-work.L[340]*work.v[148]-work.L[341]*work.v[149];
  work.v[151] = target[127]-work.L[342]*work.v[134]-work.L[343]*work.v[136]-work.L[344]*work.v[138]-work.L[345]*work.v[139]-work.L[346]*work.v[140]-work.L[347]*work.v[141]-work.L[348]*work.v[142]-work.L[349]*work.v[147]-work.L[350]*work.v[148]-work.L[351]*work.v[149]-work.L[352]*work.v[150];
  work.v[152] = target[128]-work.L[353]*work.v[135]-work.L[354]*work.v[136]-work.L[355]*work.v[138]-work.L[356]*work.v[139]-work.L[357]*work.v[140]-work.L[358]*work.v[141]-work.L[359]*work.v[142]-work.L[360]*work.v[147]-work.L[361]*work.v[148]-work.L[362]*work.v[149]-work.L[363]*work.v[150]-work.L[364]*work.v[151];
  work.v[153] = target[129]-work.L[365]*work.v[136]-work.L[366]*work.v[137]-work.L[367]*work.v[138]-work.L[368]*work.v[139]-work.L[369]*work.v[140]-work.L[370]*work.v[141]-work.L[371]*work.v[142]-work.L[372]*work.v[147]-work.L[373]*work.v[148]-work.L[374]*work.v[149]-work.L[375]*work.v[150]-work.L[376]*work.v[151]-work.L[377]*work.v[152];
  /* Diagonal scaling. Assume correctness of work.d_inv. */
  for (i = 0; i < 154; i++)
    work.v[i] *= work.d_inv[i];
  /* Back substitution */
  work.v[152] -= work.L[377]*work.v[153];
  work.v[151] -= work.L[364]*work.v[152]+work.L[376]*work.v[153];
  work.v[150] -= work.L[352]*work.v[151]+work.L[363]*work.v[152]+work.L[375]*work.v[153];
  work.v[149] -= work.L[341]*work.v[150]+work.L[351]*work.v[151]+work.L[362]*work.v[152]+work.L[374]*work.v[153];
  work.v[148] -= work.L[334]*work.v[149]+work.L[340]*work.v[150]+work.L[350]*work.v[151]+work.L[361]*work.v[152]+work.L[373]*work.v[153];
  work.v[147] -= work.L[339]*work.v[150]+work.L[349]*work.v[151]+work.L[360]*work.v[152]+work.L[372]*work.v[153];
  work.v[146] -= work.L[328]*work.v[148]+work.L[333]*work.v[149]+work.L[338]*work.v[150];
  work.v[145] -= work.L[323]*work.v[146]+work.L[327]*work.v[148]+work.L[332]*work.v[149]+work.L[337]*work.v[150];
  work.v[144] -= work.L[319]*work.v[145]+work.L[322]*work.v[146]+work.L[326]*work.v[148]+work.L[331]*work.v[149]+work.L[336]*work.v[150];
  work.v[143] -= work.L[325]*work.v[148]+work.L[330]*work.v[149]+work.L[335]*work.v[150];
  work.v[142] -= work.L[329]*work.v[149]+work.L[348]*work.v[151]+work.L[359]*work.v[152]+work.L[371]*work.v[153];
  work.v[141] -= work.L[324]*work.v[148]+work.L[347]*work.v[151]+work.L[358]*work.v[152]+work.L[370]*work.v[153];
  work.v[140] -= work.L[346]*work.v[151]+work.L[357]*work.v[152]+work.L[369]*work.v[153];
  work.v[139] -= work.L[312]*work.v[140]+work.L[345]*work.v[151]+work.L[356]*work.v[152]+work.L[368]*work.v[153];
  work.v[138] -= work.L[302]*work.v[139]+work.L[311]*work.v[140]+work.L[344]*work.v[151]+work.L[355]*work.v[152]+work.L[367]*work.v[153];
  work.v[137] -= work.L[293]*work.v[138]+work.L[301]*work.v[139]+work.L[310]*work.v[140]+work.L[366]*work.v[153];
  work.v[136] -= work.L[343]*work.v[151]+work.L[354]*work.v[152]+work.L[365]*work.v[153];
  work.v[135] -= work.L[292]*work.v[138]+work.L[300]*work.v[139]+work.L[309]*work.v[140]+work.L[353]*work.v[152];
  work.v[134] -= work.L[291]*work.v[138]+work.L[299]*work.v[139]+work.L[308]*work.v[140]+work.L[342]*work.v[151];
  work.v[133] -= work.L[290]*work.v[138]+work.L[298]*work.v[139]+work.L[307]*work.v[140];
  work.v[132] -= work.L[283]*work.v[133]+work.L[289]*work.v[138]+work.L[297]*work.v[139]+work.L[306]*work.v[140];
  work.v[131] -= work.L[273]*work.v[132]+work.L[282]*work.v[133]+work.L[288]*work.v[138]+work.L[296]*work.v[139]+work.L[305]*work.v[140];
  work.v[130] -= work.L[264]*work.v[131]+work.L[272]*work.v[132]+work.L[281]*work.v[133]+work.L[304]*work.v[140];
  work.v[129] -= work.L[287]*work.v[138]+work.L[295]*work.v[139]+work.L[303]*work.v[140];
  work.v[128] -= work.L[263]*work.v[131]+work.L[271]*work.v[132]+work.L[280]*work.v[133]+work.L[294]*work.v[139];
  work.v[127] -= work.L[262]*work.v[131]+work.L[270]*work.v[132]+work.L[279]*work.v[133]+work.L[286]*work.v[138];
  work.v[126] -= work.L[261]*work.v[131]+work.L[269]*work.v[132]+work.L[278]*work.v[133];
  work.v[125] -= work.L[254]*work.v[126]+work.L[260]*work.v[131]+work.L[268]*work.v[132]+work.L[277]*work.v[133];
  work.v[124] -= work.L[244]*work.v[125]+work.L[253]*work.v[126]+work.L[259]*work.v[131]+work.L[267]*work.v[132]+work.L[276]*work.v[133];
  work.v[123] -= work.L[235]*work.v[124]+work.L[243]*work.v[125]+work.L[252]*work.v[126]+work.L[275]*work.v[133];
  work.v[122] -= work.L[258]*work.v[131]+work.L[266]*work.v[132]+work.L[274]*work.v[133];
  work.v[121] -= work.L[234]*work.v[124]+work.L[242]*work.v[125]+work.L[251]*work.v[126]+work.L[265]*work.v[132];
  work.v[120] -= work.L[233]*work.v[124]+work.L[241]*work.v[125]+work.L[250]*work.v[126]+work.L[257]*work.v[131];
  work.v[119] -= work.L[232]*work.v[124]+work.L[240]*work.v[125]+work.L[249]*work.v[126];
  work.v[118] -= work.L[225]*work.v[119]+work.L[231]*work.v[124]+work.L[239]*work.v[125]+work.L[248]*work.v[126];
  work.v[117] -= work.L[215]*work.v[118]+work.L[224]*work.v[119]+work.L[230]*work.v[124]+work.L[238]*work.v[125]+work.L[247]*work.v[126];
  work.v[116] -= work.L[206]*work.v[117]+work.L[214]*work.v[118]+work.L[223]*work.v[119]+work.L[246]*work.v[126];
  work.v[115] -= work.L[229]*work.v[124]+work.L[237]*work.v[125]+work.L[245]*work.v[126];
  work.v[114] -= work.L[205]*work.v[117]+work.L[213]*work.v[118]+work.L[222]*work.v[119]+work.L[236]*work.v[125];
  work.v[113] -= work.L[204]*work.v[117]+work.L[212]*work.v[118]+work.L[221]*work.v[119]+work.L[228]*work.v[124];
  work.v[112] -= work.L[203]*work.v[117]+work.L[211]*work.v[118]+work.L[220]*work.v[119];
  work.v[111] -= work.L[196]*work.v[112]+work.L[202]*work.v[117]+work.L[210]*work.v[118]+work.L[219]*work.v[119];
  work.v[110] -= work.L[186]*work.v[111]+work.L[195]*work.v[112]+work.L[201]*work.v[117]+work.L[209]*work.v[118]+work.L[218]*work.v[119];
  work.v[109] -= work.L[177]*work.v[110]+work.L[185]*work.v[111]+work.L[194]*work.v[112]+work.L[217]*work.v[119];
  work.v[108] -= work.L[200]*work.v[117]+work.L[208]*work.v[118]+work.L[216]*work.v[119];
  work.v[107] -= work.L[176]*work.v[110]+work.L[184]*work.v[111]+work.L[193]*work.v[112]+work.L[207]*work.v[118];
  work.v[106] -= work.L[175]*work.v[110]+work.L[183]*work.v[111]+work.L[192]*work.v[112]+work.L[199]*work.v[117];
  work.v[105] -= work.L[174]*work.v[110]+work.L[182]*work.v[111]+work.L[191]*work.v[112];
  work.v[104] -= work.L[167]*work.v[105]+work.L[173]*work.v[110]+work.L[181]*work.v[111]+work.L[190]*work.v[112];
  work.v[103] -= work.L[157]*work.v[104]+work.L[166]*work.v[105]+work.L[172]*work.v[110]+work.L[180]*work.v[111]+work.L[189]*work.v[112];
  work.v[102] -= work.L[148]*work.v[103]+work.L[156]*work.v[104]+work.L[165]*work.v[105]+work.L[188]*work.v[112];
  work.v[101] -= work.L[171]*work.v[110]+work.L[179]*work.v[111]+work.L[187]*work.v[112];
  work.v[100] -= work.L[147]*work.v[103]+work.L[155]*work.v[104]+work.L[164]*work.v[105]+work.L[178]*work.v[111];
  work.v[99] -= work.L[146]*work.v[103]+work.L[154]*work.v[104]+work.L[163]*work.v[105]+work.L[170]*work.v[110];
  work.v[98] -= work.L[145]*work.v[103]+work.L[153]*work.v[104]+work.L[162]*work.v[105];
  work.v[97] -= work.L[138]*work.v[98]+work.L[144]*work.v[103]+work.L[152]*work.v[104]+work.L[161]*work.v[105];
  work.v[96] -= work.L[128]*work.v[97]+work.L[137]*work.v[98]+work.L[143]*work.v[103]+work.L[151]*work.v[104]+work.L[160]*work.v[105];
  work.v[95] -= work.L[119]*work.v[96]+work.L[127]*work.v[97]+work.L[136]*work.v[98]+work.L[159]*work.v[105];
  work.v[94] -= work.L[142]*work.v[103]+work.L[150]*work.v[104]+work.L[158]*work.v[105];
  work.v[93] -= work.L[118]*work.v[96]+work.L[126]*work.v[97]+work.L[135]*work.v[98]+work.L[149]*work.v[104];
  work.v[92] -= work.L[117]*work.v[96]+work.L[125]*work.v[97]+work.L[134]*work.v[98]+work.L[141]*work.v[103];
  work.v[91] -= work.L[116]*work.v[96]+work.L[124]*work.v[97]+work.L[133]*work.v[98];
  work.v[90] -= work.L[109]*work.v[91]+work.L[115]*work.v[96]+work.L[123]*work.v[97]+work.L[132]*work.v[98];
  work.v[89] -= work.L[102]*work.v[90]+work.L[108]*work.v[91]+work.L[114]*work.v[96]+work.L[122]*work.v[97]+work.L[131]*work.v[98];
  work.v[88] -= work.L[95]*work.v[89]+work.L[101]*work.v[90]+work.L[107]*work.v[91]+work.L[130]*work.v[98];
  work.v[87] -= work.L[113]*work.v[96]+work.L[121]*work.v[97]+work.L[129]*work.v[98];
  work.v[86] -= work.L[94]*work.v[89]+work.L[100]*work.v[90]+work.L[106]*work.v[91]+work.L[120]*work.v[97];
  work.v[85] -= work.L[93]*work.v[89]+work.L[99]*work.v[90]+work.L[105]*work.v[91]+work.L[112]*work.v[96];
  work.v[84] -= work.L[92]*work.v[89]+work.L[98]*work.v[90]+work.L[104]*work.v[91];
  work.v[83] -= work.L[91]*work.v[89]+work.L[97]*work.v[90]+work.L[103]*work.v[91];
  work.v[82] -= work.L[86]*work.v[84]+work.L[90]*work.v[89]+work.L[96]*work.v[90];
  work.v[81] -= work.L[82]*work.v[82]+work.L[85]*work.v[84]+work.L[89]*work.v[89];
  work.v[80] -= work.L[316]*work.v[144]+work.L[318]*work.v[145]+work.L[321]*work.v[146];
  work.v[79] -= work.L[81]*work.v[80]+work.L[320]*work.v[146];
  work.v[78] -= work.L[80]*work.v[80]+work.L[317]*work.v[145];
  work.v[77] -= work.L[79]*work.v[80]+work.L[315]*work.v[144];
  work.v[76] -= work.L[84]*work.v[83];
  work.v[75] -= work.L[76]*work.v[76]+work.L[83]*work.v[83];
  work.v[74] -= work.L[73]*work.v[75]+work.L[75]*work.v[76];
  work.v[73] -= work.L[71]*work.v[74];
  work.v[72] -= work.L[88]*work.v[87];
  work.v[71] -= work.L[69]*work.v[72]+work.L[87]*work.v[87];
  work.v[70] -= work.L[66]*work.v[71]+work.L[68]*work.v[72];
  work.v[69] -= work.L[64]*work.v[70];
  work.v[68] -= work.L[111]*work.v[94];
  work.v[67] -= work.L[62]*work.v[68]+work.L[110]*work.v[94];
  work.v[66] -= work.L[59]*work.v[67]+work.L[61]*work.v[68];
  work.v[65] -= work.L[57]*work.v[66];
  work.v[64] -= work.L[140]*work.v[101];
  work.v[63] -= work.L[55]*work.v[64]+work.L[139]*work.v[101];
  work.v[62] -= work.L[52]*work.v[63]+work.L[54]*work.v[64];
  work.v[61] -= work.L[50]*work.v[62];
  work.v[60] -= work.L[169]*work.v[108];
  work.v[59] -= work.L[48]*work.v[60]+work.L[168]*work.v[108];
  work.v[58] -= work.L[45]*work.v[59]+work.L[47]*work.v[60];
  work.v[57] -= work.L[43]*work.v[58];
  work.v[56] -= work.L[198]*work.v[115];
  work.v[55] -= work.L[41]*work.v[56]+work.L[197]*work.v[115];
  work.v[54] -= work.L[38]*work.v[55]+work.L[40]*work.v[56];
  work.v[53] -= work.L[36]*work.v[54];
  work.v[52] -= work.L[227]*work.v[122];
  work.v[51] -= work.L[34]*work.v[52]+work.L[226]*work.v[122];
  work.v[50] -= work.L[31]*work.v[51]+work.L[33]*work.v[52];
  work.v[49] -= work.L[29]*work.v[50];
  work.v[48] -= work.L[256]*work.v[129];
  work.v[47] -= work.L[27]*work.v[48]+work.L[255]*work.v[129];
  work.v[46] -= work.L[24]*work.v[47]+work.L[26]*work.v[48];
  work.v[45] -= work.L[22]*work.v[46];
  work.v[44] -= work.L[285]*work.v[136];
  work.v[43] -= work.L[20]*work.v[44]+work.L[284]*work.v[136];
  work.v[42] -= work.L[17]*work.v[43]+work.L[19]*work.v[44];
  work.v[41] -= work.L[15]*work.v[42];
  work.v[40] -= work.L[314]*work.v[143];
  work.v[39] -= work.L[13]*work.v[40]+work.L[313]*work.v[143];
  work.v[38] -= work.L[10]*work.v[39]+work.L[12]*work.v[40];
  work.v[37] -= work.L[8]*work.v[38];
  work.v[36] -= work.L[78]*work.v[80];
  work.v[35] -= work.L[6]*work.v[36]+work.L[77]*work.v[80];
  work.v[34] -= work.L[3]*work.v[35]+work.L[5]*work.v[36];
  work.v[33] -= work.L[1]*work.v[34];
  work.v[32] -= work.L[74]*work.v[76];
  work.v[31] -= work.L[72]*work.v[75];
  work.v[30] -= work.L[70]*work.v[73];
  work.v[29] -= work.L[67]*work.v[72];
  work.v[28] -= work.L[65]*work.v[71];
  work.v[27] -= work.L[63]*work.v[69];
  work.v[26] -= work.L[60]*work.v[68];
  work.v[25] -= work.L[58]*work.v[67];
  work.v[24] -= work.L[56]*work.v[65];
  work.v[23] -= work.L[53]*work.v[64];
  work.v[22] -= work.L[51]*work.v[63];
  work.v[21] -= work.L[49]*work.v[61];
  work.v[20] -= work.L[46]*work.v[60];
  work.v[19] -= work.L[44]*work.v[59];
  work.v[18] -= work.L[42]*work.v[57];
  work.v[17] -= work.L[39]*work.v[56];
  work.v[16] -= work.L[37]*work.v[55];
  work.v[15] -= work.L[35]*work.v[53];
  work.v[14] -= work.L[32]*work.v[52];
  work.v[13] -= work.L[30]*work.v[51];
  work.v[12] -= work.L[28]*work.v[49];
  work.v[11] -= work.L[25]*work.v[48];
  work.v[10] -= work.L[23]*work.v[47];
  work.v[9] -= work.L[21]*work.v[45];
  work.v[8] -= work.L[18]*work.v[44];
  work.v[7] -= work.L[16]*work.v[43];
  work.v[6] -= work.L[14]*work.v[41];
  work.v[5] -= work.L[11]*work.v[40];
  work.v[4] -= work.L[9]*work.v[39];
  work.v[3] -= work.L[7]*work.v[37];
  work.v[2] -= work.L[4]*work.v[36];
  work.v[1] -= work.L[2]*work.v[35];
  work.v[0] -= work.L[0]*work.v[33];
  /* Unpermute the result, from v to var. */
  var[0] = work.v[34];
  var[1] = work.v[38];
  var[2] = work.v[42];
  var[3] = work.v[46];
  var[4] = work.v[50];
  var[5] = work.v[54];
  var[6] = work.v[58];
  var[7] = work.v[62];
  var[8] = work.v[66];
  var[9] = work.v[70];
  var[10] = work.v[74];
  var[11] = work.v[80];
  var[12] = work.v[143];
  var[13] = work.v[136];
  var[14] = work.v[129];
  var[15] = work.v[122];
  var[16] = work.v[115];
  var[17] = work.v[108];
  var[18] = work.v[101];
  var[19] = work.v[94];
  var[20] = work.v[87];
  var[21] = work.v[83];
  var[22] = work.v[144];
  var[23] = work.v[145];
  var[24] = work.v[146];
  var[25] = work.v[141];
  var[26] = work.v[142];
  var[27] = work.v[147];
  var[28] = work.v[134];
  var[29] = work.v[135];
  var[30] = work.v[137];
  var[31] = work.v[127];
  var[32] = work.v[128];
  var[33] = work.v[130];
  var[34] = work.v[120];
  var[35] = work.v[121];
  var[36] = work.v[123];
  var[37] = work.v[113];
  var[38] = work.v[114];
  var[39] = work.v[116];
  var[40] = work.v[106];
  var[41] = work.v[107];
  var[42] = work.v[109];
  var[43] = work.v[99];
  var[44] = work.v[100];
  var[45] = work.v[102];
  var[46] = work.v[92];
  var[47] = work.v[93];
  var[48] = work.v[95];
  var[49] = work.v[85];
  var[50] = work.v[86];
  var[51] = work.v[88];
  var[52] = work.v[81];
  var[53] = work.v[82];
  var[54] = work.v[84];
  var[55] = work.v[0];
  var[56] = work.v[1];
  var[57] = work.v[2];
  var[58] = work.v[3];
  var[59] = work.v[4];
  var[60] = work.v[5];
  var[61] = work.v[6];
  var[62] = work.v[7];
  var[63] = work.v[8];
  var[64] = work.v[9];
  var[65] = work.v[10];
  var[66] = work.v[11];
  var[67] = work.v[12];
  var[68] = work.v[13];
  var[69] = work.v[14];
  var[70] = work.v[15];
  var[71] = work.v[16];
  var[72] = work.v[17];
  var[73] = work.v[18];
  var[74] = work.v[19];
  var[75] = work.v[20];
  var[76] = work.v[21];
  var[77] = work.v[22];
  var[78] = work.v[23];
  var[79] = work.v[24];
  var[80] = work.v[25];
  var[81] = work.v[26];
  var[82] = work.v[27];
  var[83] = work.v[28];
  var[84] = work.v[29];
  var[85] = work.v[30];
  var[86] = work.v[31];
  var[87] = work.v[32];
  var[88] = work.v[33];
  var[89] = work.v[35];
  var[90] = work.v[36];
  var[91] = work.v[37];
  var[92] = work.v[39];
  var[93] = work.v[40];
  var[94] = work.v[41];
  var[95] = work.v[43];
  var[96] = work.v[44];
  var[97] = work.v[45];
  var[98] = work.v[47];
  var[99] = work.v[48];
  var[100] = work.v[49];
  var[101] = work.v[51];
  var[102] = work.v[52];
  var[103] = work.v[53];
  var[104] = work.v[55];
  var[105] = work.v[56];
  var[106] = work.v[57];
  var[107] = work.v[59];
  var[108] = work.v[60];
  var[109] = work.v[61];
  var[110] = work.v[63];
  var[111] = work.v[64];
  var[112] = work.v[65];
  var[113] = work.v[67];
  var[114] = work.v[68];
  var[115] = work.v[69];
  var[116] = work.v[71];
  var[117] = work.v[72];
  var[118] = work.v[73];
  var[119] = work.v[75];
  var[120] = work.v[76];
  var[121] = work.v[77];
  var[122] = work.v[78];
  var[123] = work.v[79];
  var[124] = work.v[148];
  var[125] = work.v[149];
  var[126] = work.v[150];
  var[127] = work.v[151];
  var[128] = work.v[152];
  var[129] = work.v[153];
  var[130] = work.v[138];
  var[131] = work.v[139];
  var[132] = work.v[140];
  var[133] = work.v[131];
  var[134] = work.v[132];
  var[135] = work.v[133];
  var[136] = work.v[124];
  var[137] = work.v[125];
  var[138] = work.v[126];
  var[139] = work.v[117];
  var[140] = work.v[118];
  var[141] = work.v[119];
  var[142] = work.v[110];
  var[143] = work.v[111];
  var[144] = work.v[112];
  var[145] = work.v[103];
  var[146] = work.v[104];
  var[147] = work.v[105];
  var[148] = work.v[96];
  var[149] = work.v[97];
  var[150] = work.v[98];
  var[151] = work.v[89];
  var[152] = work.v[90];
  var[153] = work.v[91];
#ifndef ZERO_LIBRARY_MODE
  if (settings.debug) {
    printf("Squared norm for solution is %.8g.\n", check_residual(target, var));
  }
#endif
}
void ldl_factor(void) {
  work.d[0] = work.KKT[0];
  if (work.d[0] < 0)
    work.d[0] = settings.kkt_reg;
  else
    work.d[0] += settings.kkt_reg;
  work.d_inv[0] = 1/work.d[0];
  work.L[0] = work.KKT[1]*work.d_inv[0];
  work.v[1] = work.KKT[2];
  work.d[1] = work.v[1];
  if (work.d[1] < 0)
    work.d[1] = settings.kkt_reg;
  else
    work.d[1] += settings.kkt_reg;
  work.d_inv[1] = 1/work.d[1];
  work.L[2] = (work.KKT[3])*work.d_inv[1];
  work.v[2] = work.KKT[4];
  work.d[2] = work.v[2];
  if (work.d[2] < 0)
    work.d[2] = settings.kkt_reg;
  else
    work.d[2] += settings.kkt_reg;
  work.d_inv[2] = 1/work.d[2];
  work.L[4] = (work.KKT[5])*work.d_inv[2];
  work.v[3] = work.KKT[6];
  work.d[3] = work.v[3];
  if (work.d[3] < 0)
    work.d[3] = settings.kkt_reg;
  else
    work.d[3] += settings.kkt_reg;
  work.d_inv[3] = 1/work.d[3];
  work.L[7] = (work.KKT[7])*work.d_inv[3];
  work.v[4] = work.KKT[8];
  work.d[4] = work.v[4];
  if (work.d[4] < 0)
    work.d[4] = settings.kkt_reg;
  else
    work.d[4] += settings.kkt_reg;
  work.d_inv[4] = 1/work.d[4];
  work.L[9] = (work.KKT[9])*work.d_inv[4];
  work.v[5] = work.KKT[10];
  work.d[5] = work.v[5];
  if (work.d[5] < 0)
    work.d[5] = settings.kkt_reg;
  else
    work.d[5] += settings.kkt_reg;
  work.d_inv[5] = 1/work.d[5];
  work.L[11] = (work.KKT[11])*work.d_inv[5];
  work.v[6] = work.KKT[12];
  work.d[6] = work.v[6];
  if (work.d[6] < 0)
    work.d[6] = settings.kkt_reg;
  else
    work.d[6] += settings.kkt_reg;
  work.d_inv[6] = 1/work.d[6];
  work.L[14] = (work.KKT[13])*work.d_inv[6];
  work.v[7] = work.KKT[14];
  work.d[7] = work.v[7];
  if (work.d[7] < 0)
    work.d[7] = settings.kkt_reg;
  else
    work.d[7] += settings.kkt_reg;
  work.d_inv[7] = 1/work.d[7];
  work.L[16] = (work.KKT[15])*work.d_inv[7];
  work.v[8] = work.KKT[16];
  work.d[8] = work.v[8];
  if (work.d[8] < 0)
    work.d[8] = settings.kkt_reg;
  else
    work.d[8] += settings.kkt_reg;
  work.d_inv[8] = 1/work.d[8];
  work.L[18] = (work.KKT[17])*work.d_inv[8];
  work.v[9] = work.KKT[18];
  work.d[9] = work.v[9];
  if (work.d[9] < 0)
    work.d[9] = settings.kkt_reg;
  else
    work.d[9] += settings.kkt_reg;
  work.d_inv[9] = 1/work.d[9];
  work.L[21] = (work.KKT[19])*work.d_inv[9];
  work.v[10] = work.KKT[20];
  work.d[10] = work.v[10];
  if (work.d[10] < 0)
    work.d[10] = settings.kkt_reg;
  else
    work.d[10] += settings.kkt_reg;
  work.d_inv[10] = 1/work.d[10];
  work.L[23] = (work.KKT[21])*work.d_inv[10];
  work.v[11] = work.KKT[22];
  work.d[11] = work.v[11];
  if (work.d[11] < 0)
    work.d[11] = settings.kkt_reg;
  else
    work.d[11] += settings.kkt_reg;
  work.d_inv[11] = 1/work.d[11];
  work.L[25] = (work.KKT[23])*work.d_inv[11];
  work.v[12] = work.KKT[24];
  work.d[12] = work.v[12];
  if (work.d[12] < 0)
    work.d[12] = settings.kkt_reg;
  else
    work.d[12] += settings.kkt_reg;
  work.d_inv[12] = 1/work.d[12];
  work.L[28] = (work.KKT[25])*work.d_inv[12];
  work.v[13] = work.KKT[26];
  work.d[13] = work.v[13];
  if (work.d[13] < 0)
    work.d[13] = settings.kkt_reg;
  else
    work.d[13] += settings.kkt_reg;
  work.d_inv[13] = 1/work.d[13];
  work.L[30] = (work.KKT[27])*work.d_inv[13];
  work.v[14] = work.KKT[28];
  work.d[14] = work.v[14];
  if (work.d[14] < 0)
    work.d[14] = settings.kkt_reg;
  else
    work.d[14] += settings.kkt_reg;
  work.d_inv[14] = 1/work.d[14];
  work.L[32] = (work.KKT[29])*work.d_inv[14];
  work.v[15] = work.KKT[30];
  work.d[15] = work.v[15];
  if (work.d[15] < 0)
    work.d[15] = settings.kkt_reg;
  else
    work.d[15] += settings.kkt_reg;
  work.d_inv[15] = 1/work.d[15];
  work.L[35] = (work.KKT[31])*work.d_inv[15];
  work.v[16] = work.KKT[32];
  work.d[16] = work.v[16];
  if (work.d[16] < 0)
    work.d[16] = settings.kkt_reg;
  else
    work.d[16] += settings.kkt_reg;
  work.d_inv[16] = 1/work.d[16];
  work.L[37] = (work.KKT[33])*work.d_inv[16];
  work.v[17] = work.KKT[34];
  work.d[17] = work.v[17];
  if (work.d[17] < 0)
    work.d[17] = settings.kkt_reg;
  else
    work.d[17] += settings.kkt_reg;
  work.d_inv[17] = 1/work.d[17];
  work.L[39] = (work.KKT[35])*work.d_inv[17];
  work.v[18] = work.KKT[36];
  work.d[18] = work.v[18];
  if (work.d[18] < 0)
    work.d[18] = settings.kkt_reg;
  else
    work.d[18] += settings.kkt_reg;
  work.d_inv[18] = 1/work.d[18];
  work.L[42] = (work.KKT[37])*work.d_inv[18];
  work.v[19] = work.KKT[38];
  work.d[19] = work.v[19];
  if (work.d[19] < 0)
    work.d[19] = settings.kkt_reg;
  else
    work.d[19] += settings.kkt_reg;
  work.d_inv[19] = 1/work.d[19];
  work.L[44] = (work.KKT[39])*work.d_inv[19];
  work.v[20] = work.KKT[40];
  work.d[20] = work.v[20];
  if (work.d[20] < 0)
    work.d[20] = settings.kkt_reg;
  else
    work.d[20] += settings.kkt_reg;
  work.d_inv[20] = 1/work.d[20];
  work.L[46] = (work.KKT[41])*work.d_inv[20];
  work.v[21] = work.KKT[42];
  work.d[21] = work.v[21];
  if (work.d[21] < 0)
    work.d[21] = settings.kkt_reg;
  else
    work.d[21] += settings.kkt_reg;
  work.d_inv[21] = 1/work.d[21];
  work.L[49] = (work.KKT[43])*work.d_inv[21];
  work.v[22] = work.KKT[44];
  work.d[22] = work.v[22];
  if (work.d[22] < 0)
    work.d[22] = settings.kkt_reg;
  else
    work.d[22] += settings.kkt_reg;
  work.d_inv[22] = 1/work.d[22];
  work.L[51] = (work.KKT[45])*work.d_inv[22];
  work.v[23] = work.KKT[46];
  work.d[23] = work.v[23];
  if (work.d[23] < 0)
    work.d[23] = settings.kkt_reg;
  else
    work.d[23] += settings.kkt_reg;
  work.d_inv[23] = 1/work.d[23];
  work.L[53] = (work.KKT[47])*work.d_inv[23];
  work.v[24] = work.KKT[48];
  work.d[24] = work.v[24];
  if (work.d[24] < 0)
    work.d[24] = settings.kkt_reg;
  else
    work.d[24] += settings.kkt_reg;
  work.d_inv[24] = 1/work.d[24];
  work.L[56] = (work.KKT[49])*work.d_inv[24];
  work.v[25] = work.KKT[50];
  work.d[25] = work.v[25];
  if (work.d[25] < 0)
    work.d[25] = settings.kkt_reg;
  else
    work.d[25] += settings.kkt_reg;
  work.d_inv[25] = 1/work.d[25];
  work.L[58] = (work.KKT[51])*work.d_inv[25];
  work.v[26] = work.KKT[52];
  work.d[26] = work.v[26];
  if (work.d[26] < 0)
    work.d[26] = settings.kkt_reg;
  else
    work.d[26] += settings.kkt_reg;
  work.d_inv[26] = 1/work.d[26];
  work.L[60] = (work.KKT[53])*work.d_inv[26];
  work.v[27] = work.KKT[54];
  work.d[27] = work.v[27];
  if (work.d[27] < 0)
    work.d[27] = settings.kkt_reg;
  else
    work.d[27] += settings.kkt_reg;
  work.d_inv[27] = 1/work.d[27];
  work.L[63] = (work.KKT[55])*work.d_inv[27];
  work.v[28] = work.KKT[56];
  work.d[28] = work.v[28];
  if (work.d[28] < 0)
    work.d[28] = settings.kkt_reg;
  else
    work.d[28] += settings.kkt_reg;
  work.d_inv[28] = 1/work.d[28];
  work.L[65] = (work.KKT[57])*work.d_inv[28];
  work.v[29] = work.KKT[58];
  work.d[29] = work.v[29];
  if (work.d[29] < 0)
    work.d[29] = settings.kkt_reg;
  else
    work.d[29] += settings.kkt_reg;
  work.d_inv[29] = 1/work.d[29];
  work.L[67] = (work.KKT[59])*work.d_inv[29];
  work.v[30] = work.KKT[60];
  work.d[30] = work.v[30];
  if (work.d[30] < 0)
    work.d[30] = settings.kkt_reg;
  else
    work.d[30] += settings.kkt_reg;
  work.d_inv[30] = 1/work.d[30];
  work.L[70] = (work.KKT[61])*work.d_inv[30];
  work.v[31] = work.KKT[62];
  work.d[31] = work.v[31];
  if (work.d[31] < 0)
    work.d[31] = settings.kkt_reg;
  else
    work.d[31] += settings.kkt_reg;
  work.d_inv[31] = 1/work.d[31];
  work.L[72] = (work.KKT[63])*work.d_inv[31];
  work.v[32] = work.KKT[64];
  work.d[32] = work.v[32];
  if (work.d[32] < 0)
    work.d[32] = settings.kkt_reg;
  else
    work.d[32] += settings.kkt_reg;
  work.d_inv[32] = 1/work.d[32];
  work.L[74] = (work.KKT[65])*work.d_inv[32];
  work.v[0] = work.L[0]*work.d[0];
  work.v[33] = work.KKT[66]-work.L[0]*work.v[0];
  work.d[33] = work.v[33];
  if (work.d[33] > 0)
    work.d[33] = -settings.kkt_reg;
  else
    work.d[33] -= settings.kkt_reg;
  work.d_inv[33] = 1/work.d[33];
  work.L[1] = (work.KKT[67])*work.d_inv[33];
  work.v[33] = work.L[1]*work.d[33];
  work.v[34] = 0-work.L[1]*work.v[33];
  work.d[34] = work.v[34];
  if (work.d[34] < 0)
    work.d[34] = settings.kkt_reg;
  else
    work.d[34] += settings.kkt_reg;
  work.d_inv[34] = 1/work.d[34];
  work.L[3] = (work.KKT[68])*work.d_inv[34];
  work.L[5] = (work.KKT[69])*work.d_inv[34];
  work.v[1] = work.L[2]*work.d[1];
  work.v[34] = work.L[3]*work.d[34];
  work.v[35] = work.KKT[70]-work.L[2]*work.v[1]-work.L[3]*work.v[34];
  work.d[35] = work.v[35];
  if (work.d[35] > 0)
    work.d[35] = -settings.kkt_reg;
  else
    work.d[35] -= settings.kkt_reg;
  work.d_inv[35] = 1/work.d[35];
  work.L[6] = (-work.L[5]*work.v[34])*work.d_inv[35];
  work.L[77] = (work.KKT[71])*work.d_inv[35];
  work.v[2] = work.L[4]*work.d[2];
  work.v[34] = work.L[5]*work.d[34];
  work.v[35] = work.L[6]*work.d[35];
  work.v[36] = work.KKT[72]-work.L[4]*work.v[2]-work.L[5]*work.v[34]-work.L[6]*work.v[35];
  work.d[36] = work.v[36];
  if (work.d[36] > 0)
    work.d[36] = -settings.kkt_reg;
  else
    work.d[36] -= settings.kkt_reg;
  work.d_inv[36] = 1/work.d[36];
  work.L[78] = (work.KKT[73]-work.L[77]*work.v[35])*work.d_inv[36];
  work.v[3] = work.L[7]*work.d[3];
  work.v[37] = work.KKT[74]-work.L[7]*work.v[3];
  work.d[37] = work.v[37];
  if (work.d[37] > 0)
    work.d[37] = -settings.kkt_reg;
  else
    work.d[37] -= settings.kkt_reg;
  work.d_inv[37] = 1/work.d[37];
  work.L[8] = (work.KKT[75])*work.d_inv[37];
  work.v[37] = work.L[8]*work.d[37];
  work.v[38] = 0-work.L[8]*work.v[37];
  work.d[38] = work.v[38];
  if (work.d[38] < 0)
    work.d[38] = settings.kkt_reg;
  else
    work.d[38] += settings.kkt_reg;
  work.d_inv[38] = 1/work.d[38];
  work.L[10] = (work.KKT[76])*work.d_inv[38];
  work.L[12] = (work.KKT[77])*work.d_inv[38];
  work.v[4] = work.L[9]*work.d[4];
  work.v[38] = work.L[10]*work.d[38];
  work.v[39] = work.KKT[78]-work.L[9]*work.v[4]-work.L[10]*work.v[38];
  work.d[39] = work.v[39];
  if (work.d[39] > 0)
    work.d[39] = -settings.kkt_reg;
  else
    work.d[39] -= settings.kkt_reg;
  work.d_inv[39] = 1/work.d[39];
  work.L[13] = (-work.L[12]*work.v[38])*work.d_inv[39];
  work.L[313] = (work.KKT[79])*work.d_inv[39];
  work.v[5] = work.L[11]*work.d[5];
  work.v[38] = work.L[12]*work.d[38];
  work.v[39] = work.L[13]*work.d[39];
  work.v[40] = work.KKT[80]-work.L[11]*work.v[5]-work.L[12]*work.v[38]-work.L[13]*work.v[39];
  work.d[40] = work.v[40];
  if (work.d[40] > 0)
    work.d[40] = -settings.kkt_reg;
  else
    work.d[40] -= settings.kkt_reg;
  work.d_inv[40] = 1/work.d[40];
  work.L[314] = (work.KKT[81]-work.L[313]*work.v[39])*work.d_inv[40];
  work.v[6] = work.L[14]*work.d[6];
  work.v[41] = work.KKT[82]-work.L[14]*work.v[6];
  work.d[41] = work.v[41];
  if (work.d[41] > 0)
    work.d[41] = -settings.kkt_reg;
  else
    work.d[41] -= settings.kkt_reg;
  work.d_inv[41] = 1/work.d[41];
  work.L[15] = (work.KKT[83])*work.d_inv[41];
  work.v[41] = work.L[15]*work.d[41];
  work.v[42] = 0-work.L[15]*work.v[41];
  work.d[42] = work.v[42];
  if (work.d[42] < 0)
    work.d[42] = settings.kkt_reg;
  else
    work.d[42] += settings.kkt_reg;
  work.d_inv[42] = 1/work.d[42];
  work.L[17] = (work.KKT[84])*work.d_inv[42];
  work.L[19] = (work.KKT[85])*work.d_inv[42];
  work.v[7] = work.L[16]*work.d[7];
  work.v[42] = work.L[17]*work.d[42];
  work.v[43] = work.KKT[86]-work.L[16]*work.v[7]-work.L[17]*work.v[42];
  work.d[43] = work.v[43];
  if (work.d[43] > 0)
    work.d[43] = -settings.kkt_reg;
  else
    work.d[43] -= settings.kkt_reg;
  work.d_inv[43] = 1/work.d[43];
  work.L[20] = (-work.L[19]*work.v[42])*work.d_inv[43];
  work.L[284] = (work.KKT[87])*work.d_inv[43];
  work.v[8] = work.L[18]*work.d[8];
  work.v[42] = work.L[19]*work.d[42];
  work.v[43] = work.L[20]*work.d[43];
  work.v[44] = work.KKT[88]-work.L[18]*work.v[8]-work.L[19]*work.v[42]-work.L[20]*work.v[43];
  work.d[44] = work.v[44];
  if (work.d[44] > 0)
    work.d[44] = -settings.kkt_reg;
  else
    work.d[44] -= settings.kkt_reg;
  work.d_inv[44] = 1/work.d[44];
  work.L[285] = (work.KKT[89]-work.L[284]*work.v[43])*work.d_inv[44];
  work.v[9] = work.L[21]*work.d[9];
  work.v[45] = work.KKT[90]-work.L[21]*work.v[9];
  work.d[45] = work.v[45];
  if (work.d[45] > 0)
    work.d[45] = -settings.kkt_reg;
  else
    work.d[45] -= settings.kkt_reg;
  work.d_inv[45] = 1/work.d[45];
  work.L[22] = (work.KKT[91])*work.d_inv[45];
  work.v[45] = work.L[22]*work.d[45];
  work.v[46] = 0-work.L[22]*work.v[45];
  work.d[46] = work.v[46];
  if (work.d[46] < 0)
    work.d[46] = settings.kkt_reg;
  else
    work.d[46] += settings.kkt_reg;
  work.d_inv[46] = 1/work.d[46];
  work.L[24] = (work.KKT[92])*work.d_inv[46];
  work.L[26] = (work.KKT[93])*work.d_inv[46];
  work.v[10] = work.L[23]*work.d[10];
  work.v[46] = work.L[24]*work.d[46];
  work.v[47] = work.KKT[94]-work.L[23]*work.v[10]-work.L[24]*work.v[46];
  work.d[47] = work.v[47];
  if (work.d[47] > 0)
    work.d[47] = -settings.kkt_reg;
  else
    work.d[47] -= settings.kkt_reg;
  work.d_inv[47] = 1/work.d[47];
  work.L[27] = (-work.L[26]*work.v[46])*work.d_inv[47];
  work.L[255] = (work.KKT[95])*work.d_inv[47];
  work.v[11] = work.L[25]*work.d[11];
  work.v[46] = work.L[26]*work.d[46];
  work.v[47] = work.L[27]*work.d[47];
  work.v[48] = work.KKT[96]-work.L[25]*work.v[11]-work.L[26]*work.v[46]-work.L[27]*work.v[47];
  work.d[48] = work.v[48];
  if (work.d[48] > 0)
    work.d[48] = -settings.kkt_reg;
  else
    work.d[48] -= settings.kkt_reg;
  work.d_inv[48] = 1/work.d[48];
  work.L[256] = (work.KKT[97]-work.L[255]*work.v[47])*work.d_inv[48];
  work.v[12] = work.L[28]*work.d[12];
  work.v[49] = work.KKT[98]-work.L[28]*work.v[12];
  work.d[49] = work.v[49];
  if (work.d[49] > 0)
    work.d[49] = -settings.kkt_reg;
  else
    work.d[49] -= settings.kkt_reg;
  work.d_inv[49] = 1/work.d[49];
  work.L[29] = (work.KKT[99])*work.d_inv[49];
  work.v[49] = work.L[29]*work.d[49];
  work.v[50] = 0-work.L[29]*work.v[49];
  work.d[50] = work.v[50];
  if (work.d[50] < 0)
    work.d[50] = settings.kkt_reg;
  else
    work.d[50] += settings.kkt_reg;
  work.d_inv[50] = 1/work.d[50];
  work.L[31] = (work.KKT[100])*work.d_inv[50];
  work.L[33] = (work.KKT[101])*work.d_inv[50];
  work.v[13] = work.L[30]*work.d[13];
  work.v[50] = work.L[31]*work.d[50];
  work.v[51] = work.KKT[102]-work.L[30]*work.v[13]-work.L[31]*work.v[50];
  work.d[51] = work.v[51];
  if (work.d[51] > 0)
    work.d[51] = -settings.kkt_reg;
  else
    work.d[51] -= settings.kkt_reg;
  work.d_inv[51] = 1/work.d[51];
  work.L[34] = (-work.L[33]*work.v[50])*work.d_inv[51];
  work.L[226] = (work.KKT[103])*work.d_inv[51];
  work.v[14] = work.L[32]*work.d[14];
  work.v[50] = work.L[33]*work.d[50];
  work.v[51] = work.L[34]*work.d[51];
  work.v[52] = work.KKT[104]-work.L[32]*work.v[14]-work.L[33]*work.v[50]-work.L[34]*work.v[51];
  work.d[52] = work.v[52];
  if (work.d[52] > 0)
    work.d[52] = -settings.kkt_reg;
  else
    work.d[52] -= settings.kkt_reg;
  work.d_inv[52] = 1/work.d[52];
  work.L[227] = (work.KKT[105]-work.L[226]*work.v[51])*work.d_inv[52];
  work.v[15] = work.L[35]*work.d[15];
  work.v[53] = work.KKT[106]-work.L[35]*work.v[15];
  work.d[53] = work.v[53];
  if (work.d[53] > 0)
    work.d[53] = -settings.kkt_reg;
  else
    work.d[53] -= settings.kkt_reg;
  work.d_inv[53] = 1/work.d[53];
  work.L[36] = (work.KKT[107])*work.d_inv[53];
  work.v[53] = work.L[36]*work.d[53];
  work.v[54] = 0-work.L[36]*work.v[53];
  work.d[54] = work.v[54];
  if (work.d[54] < 0)
    work.d[54] = settings.kkt_reg;
  else
    work.d[54] += settings.kkt_reg;
  work.d_inv[54] = 1/work.d[54];
  work.L[38] = (work.KKT[108])*work.d_inv[54];
  work.L[40] = (work.KKT[109])*work.d_inv[54];
  work.v[16] = work.L[37]*work.d[16];
  work.v[54] = work.L[38]*work.d[54];
  work.v[55] = work.KKT[110]-work.L[37]*work.v[16]-work.L[38]*work.v[54];
  work.d[55] = work.v[55];
  if (work.d[55] > 0)
    work.d[55] = -settings.kkt_reg;
  else
    work.d[55] -= settings.kkt_reg;
  work.d_inv[55] = 1/work.d[55];
  work.L[41] = (-work.L[40]*work.v[54])*work.d_inv[55];
  work.L[197] = (work.KKT[111])*work.d_inv[55];
  work.v[17] = work.L[39]*work.d[17];
  work.v[54] = work.L[40]*work.d[54];
  work.v[55] = work.L[41]*work.d[55];
  work.v[56] = work.KKT[112]-work.L[39]*work.v[17]-work.L[40]*work.v[54]-work.L[41]*work.v[55];
  work.d[56] = work.v[56];
  if (work.d[56] > 0)
    work.d[56] = -settings.kkt_reg;
  else
    work.d[56] -= settings.kkt_reg;
  work.d_inv[56] = 1/work.d[56];
  work.L[198] = (work.KKT[113]-work.L[197]*work.v[55])*work.d_inv[56];
  work.v[18] = work.L[42]*work.d[18];
  work.v[57] = work.KKT[114]-work.L[42]*work.v[18];
  work.d[57] = work.v[57];
  if (work.d[57] > 0)
    work.d[57] = -settings.kkt_reg;
  else
    work.d[57] -= settings.kkt_reg;
  work.d_inv[57] = 1/work.d[57];
  work.L[43] = (work.KKT[115])*work.d_inv[57];
  work.v[57] = work.L[43]*work.d[57];
  work.v[58] = 0-work.L[43]*work.v[57];
  work.d[58] = work.v[58];
  if (work.d[58] < 0)
    work.d[58] = settings.kkt_reg;
  else
    work.d[58] += settings.kkt_reg;
  work.d_inv[58] = 1/work.d[58];
  work.L[45] = (work.KKT[116])*work.d_inv[58];
  work.L[47] = (work.KKT[117])*work.d_inv[58];
  work.v[19] = work.L[44]*work.d[19];
  work.v[58] = work.L[45]*work.d[58];
  work.v[59] = work.KKT[118]-work.L[44]*work.v[19]-work.L[45]*work.v[58];
  work.d[59] = work.v[59];
  if (work.d[59] > 0)
    work.d[59] = -settings.kkt_reg;
  else
    work.d[59] -= settings.kkt_reg;
  work.d_inv[59] = 1/work.d[59];
  work.L[48] = (-work.L[47]*work.v[58])*work.d_inv[59];
  work.L[168] = (work.KKT[119])*work.d_inv[59];
  work.v[20] = work.L[46]*work.d[20];
  work.v[58] = work.L[47]*work.d[58];
  work.v[59] = work.L[48]*work.d[59];
  work.v[60] = work.KKT[120]-work.L[46]*work.v[20]-work.L[47]*work.v[58]-work.L[48]*work.v[59];
  work.d[60] = work.v[60];
  if (work.d[60] > 0)
    work.d[60] = -settings.kkt_reg;
  else
    work.d[60] -= settings.kkt_reg;
  work.d_inv[60] = 1/work.d[60];
  work.L[169] = (work.KKT[121]-work.L[168]*work.v[59])*work.d_inv[60];
  work.v[21] = work.L[49]*work.d[21];
  work.v[61] = work.KKT[122]-work.L[49]*work.v[21];
  work.d[61] = work.v[61];
  if (work.d[61] > 0)
    work.d[61] = -settings.kkt_reg;
  else
    work.d[61] -= settings.kkt_reg;
  work.d_inv[61] = 1/work.d[61];
  work.L[50] = (work.KKT[123])*work.d_inv[61];
  work.v[61] = work.L[50]*work.d[61];
  work.v[62] = 0-work.L[50]*work.v[61];
  work.d[62] = work.v[62];
  if (work.d[62] < 0)
    work.d[62] = settings.kkt_reg;
  else
    work.d[62] += settings.kkt_reg;
  work.d_inv[62] = 1/work.d[62];
  work.L[52] = (work.KKT[124])*work.d_inv[62];
  work.L[54] = (work.KKT[125])*work.d_inv[62];
  work.v[22] = work.L[51]*work.d[22];
  work.v[62] = work.L[52]*work.d[62];
  work.v[63] = work.KKT[126]-work.L[51]*work.v[22]-work.L[52]*work.v[62];
  work.d[63] = work.v[63];
  if (work.d[63] > 0)
    work.d[63] = -settings.kkt_reg;
  else
    work.d[63] -= settings.kkt_reg;
  work.d_inv[63] = 1/work.d[63];
  work.L[55] = (-work.L[54]*work.v[62])*work.d_inv[63];
  work.L[139] = (work.KKT[127])*work.d_inv[63];
  work.v[23] = work.L[53]*work.d[23];
  work.v[62] = work.L[54]*work.d[62];
  work.v[63] = work.L[55]*work.d[63];
  work.v[64] = work.KKT[128]-work.L[53]*work.v[23]-work.L[54]*work.v[62]-work.L[55]*work.v[63];
  work.d[64] = work.v[64];
  if (work.d[64] > 0)
    work.d[64] = -settings.kkt_reg;
  else
    work.d[64] -= settings.kkt_reg;
  work.d_inv[64] = 1/work.d[64];
  work.L[140] = (work.KKT[129]-work.L[139]*work.v[63])*work.d_inv[64];
  work.v[24] = work.L[56]*work.d[24];
  work.v[65] = work.KKT[130]-work.L[56]*work.v[24];
  work.d[65] = work.v[65];
  if (work.d[65] > 0)
    work.d[65] = -settings.kkt_reg;
  else
    work.d[65] -= settings.kkt_reg;
  work.d_inv[65] = 1/work.d[65];
  work.L[57] = (work.KKT[131])*work.d_inv[65];
  work.v[65] = work.L[57]*work.d[65];
  work.v[66] = 0-work.L[57]*work.v[65];
  work.d[66] = work.v[66];
  if (work.d[66] < 0)
    work.d[66] = settings.kkt_reg;
  else
    work.d[66] += settings.kkt_reg;
  work.d_inv[66] = 1/work.d[66];
  work.L[59] = (work.KKT[132])*work.d_inv[66];
  work.L[61] = (work.KKT[133])*work.d_inv[66];
  work.v[25] = work.L[58]*work.d[25];
  work.v[66] = work.L[59]*work.d[66];
  work.v[67] = work.KKT[134]-work.L[58]*work.v[25]-work.L[59]*work.v[66];
  work.d[67] = work.v[67];
  if (work.d[67] > 0)
    work.d[67] = -settings.kkt_reg;
  else
    work.d[67] -= settings.kkt_reg;
  work.d_inv[67] = 1/work.d[67];
  work.L[62] = (-work.L[61]*work.v[66])*work.d_inv[67];
  work.L[110] = (work.KKT[135])*work.d_inv[67];
  work.v[26] = work.L[60]*work.d[26];
  work.v[66] = work.L[61]*work.d[66];
  work.v[67] = work.L[62]*work.d[67];
  work.v[68] = work.KKT[136]-work.L[60]*work.v[26]-work.L[61]*work.v[66]-work.L[62]*work.v[67];
  work.d[68] = work.v[68];
  if (work.d[68] > 0)
    work.d[68] = -settings.kkt_reg;
  else
    work.d[68] -= settings.kkt_reg;
  work.d_inv[68] = 1/work.d[68];
  work.L[111] = (work.KKT[137]-work.L[110]*work.v[67])*work.d_inv[68];
  work.v[27] = work.L[63]*work.d[27];
  work.v[69] = work.KKT[138]-work.L[63]*work.v[27];
  work.d[69] = work.v[69];
  if (work.d[69] > 0)
    work.d[69] = -settings.kkt_reg;
  else
    work.d[69] -= settings.kkt_reg;
  work.d_inv[69] = 1/work.d[69];
  work.L[64] = (work.KKT[139])*work.d_inv[69];
  work.v[69] = work.L[64]*work.d[69];
  work.v[70] = 0-work.L[64]*work.v[69];
  work.d[70] = work.v[70];
  if (work.d[70] < 0)
    work.d[70] = settings.kkt_reg;
  else
    work.d[70] += settings.kkt_reg;
  work.d_inv[70] = 1/work.d[70];
  work.L[66] = (work.KKT[140])*work.d_inv[70];
  work.L[68] = (work.KKT[141])*work.d_inv[70];
  work.v[28] = work.L[65]*work.d[28];
  work.v[70] = work.L[66]*work.d[70];
  work.v[71] = work.KKT[142]-work.L[65]*work.v[28]-work.L[66]*work.v[70];
  work.d[71] = work.v[71];
  if (work.d[71] > 0)
    work.d[71] = -settings.kkt_reg;
  else
    work.d[71] -= settings.kkt_reg;
  work.d_inv[71] = 1/work.d[71];
  work.L[69] = (-work.L[68]*work.v[70])*work.d_inv[71];
  work.L[87] = (work.KKT[143])*work.d_inv[71];
  work.v[29] = work.L[67]*work.d[29];
  work.v[70] = work.L[68]*work.d[70];
  work.v[71] = work.L[69]*work.d[71];
  work.v[72] = work.KKT[144]-work.L[67]*work.v[29]-work.L[68]*work.v[70]-work.L[69]*work.v[71];
  work.d[72] = work.v[72];
  if (work.d[72] > 0)
    work.d[72] = -settings.kkt_reg;
  else
    work.d[72] -= settings.kkt_reg;
  work.d_inv[72] = 1/work.d[72];
  work.L[88] = (work.KKT[145]-work.L[87]*work.v[71])*work.d_inv[72];
  work.v[30] = work.L[70]*work.d[30];
  work.v[73] = work.KKT[146]-work.L[70]*work.v[30];
  work.d[73] = work.v[73];
  if (work.d[73] > 0)
    work.d[73] = -settings.kkt_reg;
  else
    work.d[73] -= settings.kkt_reg;
  work.d_inv[73] = 1/work.d[73];
  work.L[71] = (work.KKT[147])*work.d_inv[73];
  work.v[73] = work.L[71]*work.d[73];
  work.v[74] = 0-work.L[71]*work.v[73];
  work.d[74] = work.v[74];
  if (work.d[74] < 0)
    work.d[74] = settings.kkt_reg;
  else
    work.d[74] += settings.kkt_reg;
  work.d_inv[74] = 1/work.d[74];
  work.L[73] = (work.KKT[148])*work.d_inv[74];
  work.L[75] = (work.KKT[149])*work.d_inv[74];
  work.v[31] = work.L[72]*work.d[31];
  work.v[74] = work.L[73]*work.d[74];
  work.v[75] = work.KKT[150]-work.L[72]*work.v[31]-work.L[73]*work.v[74];
  work.d[75] = work.v[75];
  if (work.d[75] > 0)
    work.d[75] = -settings.kkt_reg;
  else
    work.d[75] -= settings.kkt_reg;
  work.d_inv[75] = 1/work.d[75];
  work.L[76] = (-work.L[75]*work.v[74])*work.d_inv[75];
  work.L[83] = (work.KKT[151])*work.d_inv[75];
  work.v[32] = work.L[74]*work.d[32];
  work.v[74] = work.L[75]*work.d[74];
  work.v[75] = work.L[76]*work.d[75];
  work.v[76] = work.KKT[152]-work.L[74]*work.v[32]-work.L[75]*work.v[74]-work.L[76]*work.v[75];
  work.d[76] = work.v[76];
  if (work.d[76] > 0)
    work.d[76] = -settings.kkt_reg;
  else
    work.d[76] -= settings.kkt_reg;
  work.d_inv[76] = 1/work.d[76];
  work.L[84] = (work.KKT[153]-work.L[83]*work.v[75])*work.d_inv[76];
  work.v[77] = 0;
  work.d[77] = work.v[77];
  if (work.d[77] > 0)
    work.d[77] = -settings.kkt_reg;
  else
    work.d[77] -= settings.kkt_reg;
  work.d_inv[77] = 1/work.d[77];
  work.L[79] = (work.KKT[154])*work.d_inv[77];
  work.L[315] = (work.KKT[155])*work.d_inv[77];
  work.v[78] = 0;
  work.d[78] = work.v[78];
  if (work.d[78] > 0)
    work.d[78] = -settings.kkt_reg;
  else
    work.d[78] -= settings.kkt_reg;
  work.d_inv[78] = 1/work.d[78];
  work.L[80] = (work.KKT[156])*work.d_inv[78];
  work.L[317] = (work.KKT[157])*work.d_inv[78];
  work.v[79] = 0;
  work.d[79] = work.v[79];
  if (work.d[79] > 0)
    work.d[79] = -settings.kkt_reg;
  else
    work.d[79] -= settings.kkt_reg;
  work.d_inv[79] = 1/work.d[79];
  work.L[81] = (work.KKT[158])*work.d_inv[79];
  work.L[320] = (work.KKT[159])*work.d_inv[79];
  work.v[35] = work.L[77]*work.d[35];
  work.v[36] = work.L[78]*work.d[36];
  work.v[77] = work.L[79]*work.d[77];
  work.v[78] = work.L[80]*work.d[78];
  work.v[79] = work.L[81]*work.d[79];
  work.v[80] = 0-work.L[77]*work.v[35]-work.L[78]*work.v[36]-work.L[79]*work.v[77]-work.L[80]*work.v[78]-work.L[81]*work.v[79];
  work.d[80] = work.v[80];
  if (work.d[80] < 0)
    work.d[80] = settings.kkt_reg;
  else
    work.d[80] += settings.kkt_reg;
  work.d_inv[80] = 1/work.d[80];
  work.L[316] = (-work.L[315]*work.v[77])*work.d_inv[80];
  work.L[318] = (-work.L[317]*work.v[78])*work.d_inv[80];
  work.L[321] = (-work.L[320]*work.v[79])*work.d_inv[80];
  work.v[81] = work.KKT[160];
  work.d[81] = work.v[81];
  if (work.d[81] < 0)
    work.d[81] = settings.kkt_reg;
  else
    work.d[81] += settings.kkt_reg;
  work.d_inv[81] = 1/work.d[81];
  work.L[82] = (work.KKT[161])*work.d_inv[81];
  work.L[85] = (work.KKT[162])*work.d_inv[81];
  work.L[89] = (work.KKT[163])*work.d_inv[81];
  work.v[81] = work.L[82]*work.d[81];
  work.v[82] = work.KKT[164]-work.L[82]*work.v[81];
  work.d[82] = work.v[82];
  if (work.d[82] < 0)
    work.d[82] = settings.kkt_reg;
  else
    work.d[82] += settings.kkt_reg;
  work.d_inv[82] = 1/work.d[82];
  work.L[86] = (work.KKT[165]-work.L[85]*work.v[81])*work.d_inv[82];
  work.L[90] = (-work.L[89]*work.v[81])*work.d_inv[82];
  work.L[96] = (work.KKT[166])*work.d_inv[82];
  work.v[75] = work.L[83]*work.d[75];
  work.v[76] = work.L[84]*work.d[76];
  work.v[83] = 0-work.L[83]*work.v[75]-work.L[84]*work.v[76];
  work.d[83] = work.v[83];
  if (work.d[83] < 0)
    work.d[83] = settings.kkt_reg;
  else
    work.d[83] += settings.kkt_reg;
  work.d_inv[83] = 1/work.d[83];
  work.L[91] = (work.KKT[167])*work.d_inv[83];
  work.L[97] = (work.KKT[168])*work.d_inv[83];
  work.L[103] = (work.KKT[169])*work.d_inv[83];
  work.v[81] = work.L[85]*work.d[81];
  work.v[82] = work.L[86]*work.d[82];
  work.v[84] = work.KKT[170]-work.L[85]*work.v[81]-work.L[86]*work.v[82];
  work.d[84] = work.v[84];
  if (work.d[84] < 0)
    work.d[84] = settings.kkt_reg;
  else
    work.d[84] += settings.kkt_reg;
  work.d_inv[84] = 1/work.d[84];
  work.L[92] = (-work.L[89]*work.v[81]-work.L[90]*work.v[82])*work.d_inv[84];
  work.L[98] = (-work.L[96]*work.v[82])*work.d_inv[84];
  work.L[104] = (work.KKT[171])*work.d_inv[84];
  work.v[85] = 0;
  work.d[85] = work.v[85];
  if (work.d[85] < 0)
    work.d[85] = settings.kkt_reg;
  else
    work.d[85] += settings.kkt_reg;
  work.d_inv[85] = 1/work.d[85];
  work.L[93] = (work.KKT[172])*work.d_inv[85];
  work.L[99] = (work.KKT[173])*work.d_inv[85];
  work.L[105] = (work.KKT[174])*work.d_inv[85];
  work.L[112] = (work.KKT[175])*work.d_inv[85];
  work.v[86] = 0;
  work.d[86] = work.v[86];
  if (work.d[86] < 0)
    work.d[86] = settings.kkt_reg;
  else
    work.d[86] += settings.kkt_reg;
  work.d_inv[86] = 1/work.d[86];
  work.L[94] = (work.KKT[176])*work.d_inv[86];
  work.L[100] = (work.KKT[177])*work.d_inv[86];
  work.L[106] = (work.KKT[178])*work.d_inv[86];
  work.L[120] = (work.KKT[179])*work.d_inv[86];
  work.v[71] = work.L[87]*work.d[71];
  work.v[72] = work.L[88]*work.d[72];
  work.v[87] = 0-work.L[87]*work.v[71]-work.L[88]*work.v[72];
  work.d[87] = work.v[87];
  if (work.d[87] < 0)
    work.d[87] = settings.kkt_reg;
  else
    work.d[87] += settings.kkt_reg;
  work.d_inv[87] = 1/work.d[87];
  work.L[113] = (work.KKT[180])*work.d_inv[87];
  work.L[121] = (work.KKT[181])*work.d_inv[87];
  work.L[129] = (work.KKT[182])*work.d_inv[87];
  work.v[88] = 0;
  work.d[88] = work.v[88];
  if (work.d[88] < 0)
    work.d[88] = settings.kkt_reg;
  else
    work.d[88] += settings.kkt_reg;
  work.d_inv[88] = 1/work.d[88];
  work.L[95] = (work.KKT[183])*work.d_inv[88];
  work.L[101] = (work.KKT[184])*work.d_inv[88];
  work.L[107] = (work.KKT[185])*work.d_inv[88];
  work.L[130] = (work.KKT[186])*work.d_inv[88];
  work.v[81] = work.L[89]*work.d[81];
  work.v[82] = work.L[90]*work.d[82];
  work.v[83] = work.L[91]*work.d[83];
  work.v[84] = work.L[92]*work.d[84];
  work.v[85] = work.L[93]*work.d[85];
  work.v[86] = work.L[94]*work.d[86];
  work.v[88] = work.L[95]*work.d[88];
  work.v[89] = 0-work.L[89]*work.v[81]-work.L[90]*work.v[82]-work.L[91]*work.v[83]-work.L[92]*work.v[84]-work.L[93]*work.v[85]-work.L[94]*work.v[86]-work.L[95]*work.v[88];
  work.d[89] = work.v[89];
  if (work.d[89] > 0)
    work.d[89] = -settings.kkt_reg;
  else
    work.d[89] -= settings.kkt_reg;
  work.d_inv[89] = 1/work.d[89];
  work.L[102] = (-work.L[96]*work.v[82]-work.L[97]*work.v[83]-work.L[98]*work.v[84]-work.L[99]*work.v[85]-work.L[100]*work.v[86]-work.L[101]*work.v[88])*work.d_inv[89];
  work.L[108] = (-work.L[103]*work.v[83]-work.L[104]*work.v[84]-work.L[105]*work.v[85]-work.L[106]*work.v[86]-work.L[107]*work.v[88])*work.d_inv[89];
  work.L[114] = (-work.L[112]*work.v[85])*work.d_inv[89];
  work.L[122] = (-work.L[120]*work.v[86])*work.d_inv[89];
  work.L[131] = (-work.L[130]*work.v[88])*work.d_inv[89];
  work.v[82] = work.L[96]*work.d[82];
  work.v[83] = work.L[97]*work.d[83];
  work.v[84] = work.L[98]*work.d[84];
  work.v[85] = work.L[99]*work.d[85];
  work.v[86] = work.L[100]*work.d[86];
  work.v[88] = work.L[101]*work.d[88];
  work.v[89] = work.L[102]*work.d[89];
  work.v[90] = 0-work.L[96]*work.v[82]-work.L[97]*work.v[83]-work.L[98]*work.v[84]-work.L[99]*work.v[85]-work.L[100]*work.v[86]-work.L[101]*work.v[88]-work.L[102]*work.v[89];
  work.d[90] = work.v[90];
  if (work.d[90] > 0)
    work.d[90] = -settings.kkt_reg;
  else
    work.d[90] -= settings.kkt_reg;
  work.d_inv[90] = 1/work.d[90];
  work.L[109] = (-work.L[103]*work.v[83]-work.L[104]*work.v[84]-work.L[105]*work.v[85]-work.L[106]*work.v[86]-work.L[107]*work.v[88]-work.L[108]*work.v[89])*work.d_inv[90];
  work.L[115] = (-work.L[112]*work.v[85]-work.L[114]*work.v[89])*work.d_inv[90];
  work.L[123] = (-work.L[120]*work.v[86]-work.L[122]*work.v[89])*work.d_inv[90];
  work.L[132] = (-work.L[130]*work.v[88]-work.L[131]*work.v[89])*work.d_inv[90];
  work.v[83] = work.L[103]*work.d[83];
  work.v[84] = work.L[104]*work.d[84];
  work.v[85] = work.L[105]*work.d[85];
  work.v[86] = work.L[106]*work.d[86];
  work.v[88] = work.L[107]*work.d[88];
  work.v[89] = work.L[108]*work.d[89];
  work.v[90] = work.L[109]*work.d[90];
  work.v[91] = 0-work.L[103]*work.v[83]-work.L[104]*work.v[84]-work.L[105]*work.v[85]-work.L[106]*work.v[86]-work.L[107]*work.v[88]-work.L[108]*work.v[89]-work.L[109]*work.v[90];
  work.d[91] = work.v[91];
  if (work.d[91] > 0)
    work.d[91] = -settings.kkt_reg;
  else
    work.d[91] -= settings.kkt_reg;
  work.d_inv[91] = 1/work.d[91];
  work.L[116] = (-work.L[112]*work.v[85]-work.L[114]*work.v[89]-work.L[115]*work.v[90])*work.d_inv[91];
  work.L[124] = (-work.L[120]*work.v[86]-work.L[122]*work.v[89]-work.L[123]*work.v[90])*work.d_inv[91];
  work.L[133] = (-work.L[130]*work.v[88]-work.L[131]*work.v[89]-work.L[132]*work.v[90])*work.d_inv[91];
  work.v[92] = 0;
  work.d[92] = work.v[92];
  if (work.d[92] < 0)
    work.d[92] = settings.kkt_reg;
  else
    work.d[92] += settings.kkt_reg;
  work.d_inv[92] = 1/work.d[92];
  work.L[117] = (work.KKT[187])*work.d_inv[92];
  work.L[125] = (work.KKT[188])*work.d_inv[92];
  work.L[134] = (work.KKT[189])*work.d_inv[92];
  work.L[141] = (work.KKT[190])*work.d_inv[92];
  work.v[93] = 0;
  work.d[93] = work.v[93];
  if (work.d[93] < 0)
    work.d[93] = settings.kkt_reg;
  else
    work.d[93] += settings.kkt_reg;
  work.d_inv[93] = 1/work.d[93];
  work.L[118] = (work.KKT[191])*work.d_inv[93];
  work.L[126] = (work.KKT[192])*work.d_inv[93];
  work.L[135] = (work.KKT[193])*work.d_inv[93];
  work.L[149] = (work.KKT[194])*work.d_inv[93];
  work.v[67] = work.L[110]*work.d[67];
  work.v[68] = work.L[111]*work.d[68];
  work.v[94] = 0-work.L[110]*work.v[67]-work.L[111]*work.v[68];
  work.d[94] = work.v[94];
  if (work.d[94] < 0)
    work.d[94] = settings.kkt_reg;
  else
    work.d[94] += settings.kkt_reg;
  work.d_inv[94] = 1/work.d[94];
  work.L[142] = (work.KKT[195])*work.d_inv[94];
  work.L[150] = (work.KKT[196])*work.d_inv[94];
  work.L[158] = (work.KKT[197])*work.d_inv[94];
  work.v[95] = 0;
  work.d[95] = work.v[95];
  if (work.d[95] < 0)
    work.d[95] = settings.kkt_reg;
  else
    work.d[95] += settings.kkt_reg;
  work.d_inv[95] = 1/work.d[95];
  work.L[119] = (work.KKT[198])*work.d_inv[95];
  work.L[127] = (work.KKT[199])*work.d_inv[95];
  work.L[136] = (work.KKT[200])*work.d_inv[95];
  work.L[159] = (work.KKT[201])*work.d_inv[95];
  work.v[85] = work.L[112]*work.d[85];
  work.v[87] = work.L[113]*work.d[87];
  work.v[89] = work.L[114]*work.d[89];
  work.v[90] = work.L[115]*work.d[90];
  work.v[91] = work.L[116]*work.d[91];
  work.v[92] = work.L[117]*work.d[92];
  work.v[93] = work.L[118]*work.d[93];
  work.v[95] = work.L[119]*work.d[95];
  work.v[96] = 0-work.L[112]*work.v[85]-work.L[113]*work.v[87]-work.L[114]*work.v[89]-work.L[115]*work.v[90]-work.L[116]*work.v[91]-work.L[117]*work.v[92]-work.L[118]*work.v[93]-work.L[119]*work.v[95];
  work.d[96] = work.v[96];
  if (work.d[96] > 0)
    work.d[96] = -settings.kkt_reg;
  else
    work.d[96] -= settings.kkt_reg;
  work.d_inv[96] = 1/work.d[96];
  work.L[128] = (-work.L[121]*work.v[87]-work.L[122]*work.v[89]-work.L[123]*work.v[90]-work.L[124]*work.v[91]-work.L[125]*work.v[92]-work.L[126]*work.v[93]-work.L[127]*work.v[95])*work.d_inv[96];
  work.L[137] = (-work.L[129]*work.v[87]-work.L[131]*work.v[89]-work.L[132]*work.v[90]-work.L[133]*work.v[91]-work.L[134]*work.v[92]-work.L[135]*work.v[93]-work.L[136]*work.v[95])*work.d_inv[96];
  work.L[143] = (-work.L[141]*work.v[92])*work.d_inv[96];
  work.L[151] = (-work.L[149]*work.v[93])*work.d_inv[96];
  work.L[160] = (-work.L[159]*work.v[95])*work.d_inv[96];
  work.v[86] = work.L[120]*work.d[86];
  work.v[87] = work.L[121]*work.d[87];
  work.v[89] = work.L[122]*work.d[89];
  work.v[90] = work.L[123]*work.d[90];
  work.v[91] = work.L[124]*work.d[91];
  work.v[92] = work.L[125]*work.d[92];
  work.v[93] = work.L[126]*work.d[93];
  work.v[95] = work.L[127]*work.d[95];
  work.v[96] = work.L[128]*work.d[96];
  work.v[97] = 0-work.L[120]*work.v[86]-work.L[121]*work.v[87]-work.L[122]*work.v[89]-work.L[123]*work.v[90]-work.L[124]*work.v[91]-work.L[125]*work.v[92]-work.L[126]*work.v[93]-work.L[127]*work.v[95]-work.L[128]*work.v[96];
  work.d[97] = work.v[97];
  if (work.d[97] > 0)
    work.d[97] = -settings.kkt_reg;
  else
    work.d[97] -= settings.kkt_reg;
  work.d_inv[97] = 1/work.d[97];
  work.L[138] = (-work.L[129]*work.v[87]-work.L[131]*work.v[89]-work.L[132]*work.v[90]-work.L[133]*work.v[91]-work.L[134]*work.v[92]-work.L[135]*work.v[93]-work.L[136]*work.v[95]-work.L[137]*work.v[96])*work.d_inv[97];
  work.L[144] = (-work.L[141]*work.v[92]-work.L[143]*work.v[96])*work.d_inv[97];
  work.L[152] = (-work.L[149]*work.v[93]-work.L[151]*work.v[96])*work.d_inv[97];
  work.L[161] = (-work.L[159]*work.v[95]-work.L[160]*work.v[96])*work.d_inv[97];
  work.v[87] = work.L[129]*work.d[87];
  work.v[88] = work.L[130]*work.d[88];
  work.v[89] = work.L[131]*work.d[89];
  work.v[90] = work.L[132]*work.d[90];
  work.v[91] = work.L[133]*work.d[91];
  work.v[92] = work.L[134]*work.d[92];
  work.v[93] = work.L[135]*work.d[93];
  work.v[95] = work.L[136]*work.d[95];
  work.v[96] = work.L[137]*work.d[96];
  work.v[97] = work.L[138]*work.d[97];
  work.v[98] = 0-work.L[129]*work.v[87]-work.L[130]*work.v[88]-work.L[131]*work.v[89]-work.L[132]*work.v[90]-work.L[133]*work.v[91]-work.L[134]*work.v[92]-work.L[135]*work.v[93]-work.L[136]*work.v[95]-work.L[137]*work.v[96]-work.L[138]*work.v[97];
  work.d[98] = work.v[98];
  if (work.d[98] > 0)
    work.d[98] = -settings.kkt_reg;
  else
    work.d[98] -= settings.kkt_reg;
  work.d_inv[98] = 1/work.d[98];
  work.L[145] = (-work.L[141]*work.v[92]-work.L[143]*work.v[96]-work.L[144]*work.v[97])*work.d_inv[98];
  work.L[153] = (-work.L[149]*work.v[93]-work.L[151]*work.v[96]-work.L[152]*work.v[97])*work.d_inv[98];
  work.L[162] = (-work.L[159]*work.v[95]-work.L[160]*work.v[96]-work.L[161]*work.v[97])*work.d_inv[98];
  work.v[99] = 0;
  work.d[99] = work.v[99];
  if (work.d[99] < 0)
    work.d[99] = settings.kkt_reg;
  else
    work.d[99] += settings.kkt_reg;
  work.d_inv[99] = 1/work.d[99];
  work.L[146] = (work.KKT[202])*work.d_inv[99];
  work.L[154] = (work.KKT[203])*work.d_inv[99];
  work.L[163] = (work.KKT[204])*work.d_inv[99];
  work.L[170] = (work.KKT[205])*work.d_inv[99];
  work.v[100] = 0;
  work.d[100] = work.v[100];
  if (work.d[100] < 0)
    work.d[100] = settings.kkt_reg;
  else
    work.d[100] += settings.kkt_reg;
  work.d_inv[100] = 1/work.d[100];
  work.L[147] = (work.KKT[206])*work.d_inv[100];
  work.L[155] = (work.KKT[207])*work.d_inv[100];
  work.L[164] = (work.KKT[208])*work.d_inv[100];
  work.L[178] = (work.KKT[209])*work.d_inv[100];
  work.v[63] = work.L[139]*work.d[63];
  work.v[64] = work.L[140]*work.d[64];
  work.v[101] = 0-work.L[139]*work.v[63]-work.L[140]*work.v[64];
  work.d[101] = work.v[101];
  if (work.d[101] < 0)
    work.d[101] = settings.kkt_reg;
  else
    work.d[101] += settings.kkt_reg;
  work.d_inv[101] = 1/work.d[101];
  work.L[171] = (work.KKT[210])*work.d_inv[101];
  work.L[179] = (work.KKT[211])*work.d_inv[101];
  work.L[187] = (work.KKT[212])*work.d_inv[101];
  work.v[102] = 0;
  work.d[102] = work.v[102];
  if (work.d[102] < 0)
    work.d[102] = settings.kkt_reg;
  else
    work.d[102] += settings.kkt_reg;
  work.d_inv[102] = 1/work.d[102];
  work.L[148] = (work.KKT[213])*work.d_inv[102];
  work.L[156] = (work.KKT[214])*work.d_inv[102];
  work.L[165] = (work.KKT[215])*work.d_inv[102];
  work.L[188] = (work.KKT[216])*work.d_inv[102];
  work.v[92] = work.L[141]*work.d[92];
  work.v[94] = work.L[142]*work.d[94];
  work.v[96] = work.L[143]*work.d[96];
  work.v[97] = work.L[144]*work.d[97];
  work.v[98] = work.L[145]*work.d[98];
  work.v[99] = work.L[146]*work.d[99];
  work.v[100] = work.L[147]*work.d[100];
  work.v[102] = work.L[148]*work.d[102];
  work.v[103] = 0-work.L[141]*work.v[92]-work.L[142]*work.v[94]-work.L[143]*work.v[96]-work.L[144]*work.v[97]-work.L[145]*work.v[98]-work.L[146]*work.v[99]-work.L[147]*work.v[100]-work.L[148]*work.v[102];
  work.d[103] = work.v[103];
  if (work.d[103] > 0)
    work.d[103] = -settings.kkt_reg;
  else
    work.d[103] -= settings.kkt_reg;
  work.d_inv[103] = 1/work.d[103];
  work.L[157] = (-work.L[150]*work.v[94]-work.L[151]*work.v[96]-work.L[152]*work.v[97]-work.L[153]*work.v[98]-work.L[154]*work.v[99]-work.L[155]*work.v[100]-work.L[156]*work.v[102])*work.d_inv[103];
  work.L[166] = (-work.L[158]*work.v[94]-work.L[160]*work.v[96]-work.L[161]*work.v[97]-work.L[162]*work.v[98]-work.L[163]*work.v[99]-work.L[164]*work.v[100]-work.L[165]*work.v[102])*work.d_inv[103];
  work.L[172] = (-work.L[170]*work.v[99])*work.d_inv[103];
  work.L[180] = (-work.L[178]*work.v[100])*work.d_inv[103];
  work.L[189] = (-work.L[188]*work.v[102])*work.d_inv[103];
  work.v[93] = work.L[149]*work.d[93];
  work.v[94] = work.L[150]*work.d[94];
  work.v[96] = work.L[151]*work.d[96];
  work.v[97] = work.L[152]*work.d[97];
  work.v[98] = work.L[153]*work.d[98];
  work.v[99] = work.L[154]*work.d[99];
  work.v[100] = work.L[155]*work.d[100];
  work.v[102] = work.L[156]*work.d[102];
  work.v[103] = work.L[157]*work.d[103];
  work.v[104] = 0-work.L[149]*work.v[93]-work.L[150]*work.v[94]-work.L[151]*work.v[96]-work.L[152]*work.v[97]-work.L[153]*work.v[98]-work.L[154]*work.v[99]-work.L[155]*work.v[100]-work.L[156]*work.v[102]-work.L[157]*work.v[103];
  work.d[104] = work.v[104];
  if (work.d[104] > 0)
    work.d[104] = -settings.kkt_reg;
  else
    work.d[104] -= settings.kkt_reg;
  work.d_inv[104] = 1/work.d[104];
  work.L[167] = (-work.L[158]*work.v[94]-work.L[160]*work.v[96]-work.L[161]*work.v[97]-work.L[162]*work.v[98]-work.L[163]*work.v[99]-work.L[164]*work.v[100]-work.L[165]*work.v[102]-work.L[166]*work.v[103])*work.d_inv[104];
  work.L[173] = (-work.L[170]*work.v[99]-work.L[172]*work.v[103])*work.d_inv[104];
  work.L[181] = (-work.L[178]*work.v[100]-work.L[180]*work.v[103])*work.d_inv[104];
  work.L[190] = (-work.L[188]*work.v[102]-work.L[189]*work.v[103])*work.d_inv[104];
  work.v[94] = work.L[158]*work.d[94];
  work.v[95] = work.L[159]*work.d[95];
  work.v[96] = work.L[160]*work.d[96];
  work.v[97] = work.L[161]*work.d[97];
  work.v[98] = work.L[162]*work.d[98];
  work.v[99] = work.L[163]*work.d[99];
  work.v[100] = work.L[164]*work.d[100];
  work.v[102] = work.L[165]*work.d[102];
  work.v[103] = work.L[166]*work.d[103];
  work.v[104] = work.L[167]*work.d[104];
  work.v[105] = 0-work.L[158]*work.v[94]-work.L[159]*work.v[95]-work.L[160]*work.v[96]-work.L[161]*work.v[97]-work.L[162]*work.v[98]-work.L[163]*work.v[99]-work.L[164]*work.v[100]-work.L[165]*work.v[102]-work.L[166]*work.v[103]-work.L[167]*work.v[104];
  work.d[105] = work.v[105];
  if (work.d[105] > 0)
    work.d[105] = -settings.kkt_reg;
  else
    work.d[105] -= settings.kkt_reg;
  work.d_inv[105] = 1/work.d[105];
  work.L[174] = (-work.L[170]*work.v[99]-work.L[172]*work.v[103]-work.L[173]*work.v[104])*work.d_inv[105];
  work.L[182] = (-work.L[178]*work.v[100]-work.L[180]*work.v[103]-work.L[181]*work.v[104])*work.d_inv[105];
  work.L[191] = (-work.L[188]*work.v[102]-work.L[189]*work.v[103]-work.L[190]*work.v[104])*work.d_inv[105];
  work.v[106] = 0;
  work.d[106] = work.v[106];
  if (work.d[106] < 0)
    work.d[106] = settings.kkt_reg;
  else
    work.d[106] += settings.kkt_reg;
  work.d_inv[106] = 1/work.d[106];
  work.L[175] = (work.KKT[217])*work.d_inv[106];
  work.L[183] = (work.KKT[218])*work.d_inv[106];
  work.L[192] = (work.KKT[219])*work.d_inv[106];
  work.L[199] = (work.KKT[220])*work.d_inv[106];
  work.v[107] = 0;
  work.d[107] = work.v[107];
  if (work.d[107] < 0)
    work.d[107] = settings.kkt_reg;
  else
    work.d[107] += settings.kkt_reg;
  work.d_inv[107] = 1/work.d[107];
  work.L[176] = (work.KKT[221])*work.d_inv[107];
  work.L[184] = (work.KKT[222])*work.d_inv[107];
  work.L[193] = (work.KKT[223])*work.d_inv[107];
  work.L[207] = (work.KKT[224])*work.d_inv[107];
  work.v[59] = work.L[168]*work.d[59];
  work.v[60] = work.L[169]*work.d[60];
  work.v[108] = 0-work.L[168]*work.v[59]-work.L[169]*work.v[60];
  work.d[108] = work.v[108];
  if (work.d[108] < 0)
    work.d[108] = settings.kkt_reg;
  else
    work.d[108] += settings.kkt_reg;
  work.d_inv[108] = 1/work.d[108];
  work.L[200] = (work.KKT[225])*work.d_inv[108];
  work.L[208] = (work.KKT[226])*work.d_inv[108];
  work.L[216] = (work.KKT[227])*work.d_inv[108];
  work.v[109] = 0;
  work.d[109] = work.v[109];
  if (work.d[109] < 0)
    work.d[109] = settings.kkt_reg;
  else
    work.d[109] += settings.kkt_reg;
  work.d_inv[109] = 1/work.d[109];
  work.L[177] = (work.KKT[228])*work.d_inv[109];
  work.L[185] = (work.KKT[229])*work.d_inv[109];
  work.L[194] = (work.KKT[230])*work.d_inv[109];
  work.L[217] = (work.KKT[231])*work.d_inv[109];
  work.v[99] = work.L[170]*work.d[99];
  work.v[101] = work.L[171]*work.d[101];
  work.v[103] = work.L[172]*work.d[103];
  work.v[104] = work.L[173]*work.d[104];
  work.v[105] = work.L[174]*work.d[105];
  work.v[106] = work.L[175]*work.d[106];
  work.v[107] = work.L[176]*work.d[107];
  work.v[109] = work.L[177]*work.d[109];
  work.v[110] = 0-work.L[170]*work.v[99]-work.L[171]*work.v[101]-work.L[172]*work.v[103]-work.L[173]*work.v[104]-work.L[174]*work.v[105]-work.L[175]*work.v[106]-work.L[176]*work.v[107]-work.L[177]*work.v[109];
  work.d[110] = work.v[110];
  if (work.d[110] > 0)
    work.d[110] = -settings.kkt_reg;
  else
    work.d[110] -= settings.kkt_reg;
  work.d_inv[110] = 1/work.d[110];
  work.L[186] = (-work.L[179]*work.v[101]-work.L[180]*work.v[103]-work.L[181]*work.v[104]-work.L[182]*work.v[105]-work.L[183]*work.v[106]-work.L[184]*work.v[107]-work.L[185]*work.v[109])*work.d_inv[110];
  work.L[195] = (-work.L[187]*work.v[101]-work.L[189]*work.v[103]-work.L[190]*work.v[104]-work.L[191]*work.v[105]-work.L[192]*work.v[106]-work.L[193]*work.v[107]-work.L[194]*work.v[109])*work.d_inv[110];
  work.L[201] = (-work.L[199]*work.v[106])*work.d_inv[110];
  work.L[209] = (-work.L[207]*work.v[107])*work.d_inv[110];
  work.L[218] = (-work.L[217]*work.v[109])*work.d_inv[110];
  work.v[100] = work.L[178]*work.d[100];
  work.v[101] = work.L[179]*work.d[101];
  work.v[103] = work.L[180]*work.d[103];
  work.v[104] = work.L[181]*work.d[104];
  work.v[105] = work.L[182]*work.d[105];
  work.v[106] = work.L[183]*work.d[106];
  work.v[107] = work.L[184]*work.d[107];
  work.v[109] = work.L[185]*work.d[109];
  work.v[110] = work.L[186]*work.d[110];
  work.v[111] = 0-work.L[178]*work.v[100]-work.L[179]*work.v[101]-work.L[180]*work.v[103]-work.L[181]*work.v[104]-work.L[182]*work.v[105]-work.L[183]*work.v[106]-work.L[184]*work.v[107]-work.L[185]*work.v[109]-work.L[186]*work.v[110];
  work.d[111] = work.v[111];
  if (work.d[111] > 0)
    work.d[111] = -settings.kkt_reg;
  else
    work.d[111] -= settings.kkt_reg;
  work.d_inv[111] = 1/work.d[111];
  work.L[196] = (-work.L[187]*work.v[101]-work.L[189]*work.v[103]-work.L[190]*work.v[104]-work.L[191]*work.v[105]-work.L[192]*work.v[106]-work.L[193]*work.v[107]-work.L[194]*work.v[109]-work.L[195]*work.v[110])*work.d_inv[111];
  work.L[202] = (-work.L[199]*work.v[106]-work.L[201]*work.v[110])*work.d_inv[111];
  work.L[210] = (-work.L[207]*work.v[107]-work.L[209]*work.v[110])*work.d_inv[111];
  work.L[219] = (-work.L[217]*work.v[109]-work.L[218]*work.v[110])*work.d_inv[111];
  work.v[101] = work.L[187]*work.d[101];
  work.v[102] = work.L[188]*work.d[102];
  work.v[103] = work.L[189]*work.d[103];
  work.v[104] = work.L[190]*work.d[104];
  work.v[105] = work.L[191]*work.d[105];
  work.v[106] = work.L[192]*work.d[106];
  work.v[107] = work.L[193]*work.d[107];
  work.v[109] = work.L[194]*work.d[109];
  work.v[110] = work.L[195]*work.d[110];
  work.v[111] = work.L[196]*work.d[111];
  work.v[112] = 0-work.L[187]*work.v[101]-work.L[188]*work.v[102]-work.L[189]*work.v[103]-work.L[190]*work.v[104]-work.L[191]*work.v[105]-work.L[192]*work.v[106]-work.L[193]*work.v[107]-work.L[194]*work.v[109]-work.L[195]*work.v[110]-work.L[196]*work.v[111];
  work.d[112] = work.v[112];
  if (work.d[112] > 0)
    work.d[112] = -settings.kkt_reg;
  else
    work.d[112] -= settings.kkt_reg;
  work.d_inv[112] = 1/work.d[112];
  work.L[203] = (-work.L[199]*work.v[106]-work.L[201]*work.v[110]-work.L[202]*work.v[111])*work.d_inv[112];
  work.L[211] = (-work.L[207]*work.v[107]-work.L[209]*work.v[110]-work.L[210]*work.v[111])*work.d_inv[112];
  work.L[220] = (-work.L[217]*work.v[109]-work.L[218]*work.v[110]-work.L[219]*work.v[111])*work.d_inv[112];
  work.v[113] = 0;
  work.d[113] = work.v[113];
  if (work.d[113] < 0)
    work.d[113] = settings.kkt_reg;
  else
    work.d[113] += settings.kkt_reg;
  work.d_inv[113] = 1/work.d[113];
  work.L[204] = (work.KKT[232])*work.d_inv[113];
  work.L[212] = (work.KKT[233])*work.d_inv[113];
  work.L[221] = (work.KKT[234])*work.d_inv[113];
  work.L[228] = (work.KKT[235])*work.d_inv[113];
  work.v[114] = 0;
  work.d[114] = work.v[114];
  if (work.d[114] < 0)
    work.d[114] = settings.kkt_reg;
  else
    work.d[114] += settings.kkt_reg;
  work.d_inv[114] = 1/work.d[114];
  work.L[205] = (work.KKT[236])*work.d_inv[114];
  work.L[213] = (work.KKT[237])*work.d_inv[114];
  work.L[222] = (work.KKT[238])*work.d_inv[114];
  work.L[236] = (work.KKT[239])*work.d_inv[114];
  work.v[55] = work.L[197]*work.d[55];
  work.v[56] = work.L[198]*work.d[56];
  work.v[115] = 0-work.L[197]*work.v[55]-work.L[198]*work.v[56];
  work.d[115] = work.v[115];
  if (work.d[115] < 0)
    work.d[115] = settings.kkt_reg;
  else
    work.d[115] += settings.kkt_reg;
  work.d_inv[115] = 1/work.d[115];
  work.L[229] = (work.KKT[240])*work.d_inv[115];
  work.L[237] = (work.KKT[241])*work.d_inv[115];
  work.L[245] = (work.KKT[242])*work.d_inv[115];
  work.v[116] = 0;
  work.d[116] = work.v[116];
  if (work.d[116] < 0)
    work.d[116] = settings.kkt_reg;
  else
    work.d[116] += settings.kkt_reg;
  work.d_inv[116] = 1/work.d[116];
  work.L[206] = (work.KKT[243])*work.d_inv[116];
  work.L[214] = (work.KKT[244])*work.d_inv[116];
  work.L[223] = (work.KKT[245])*work.d_inv[116];
  work.L[246] = (work.KKT[246])*work.d_inv[116];
  work.v[106] = work.L[199]*work.d[106];
  work.v[108] = work.L[200]*work.d[108];
  work.v[110] = work.L[201]*work.d[110];
  work.v[111] = work.L[202]*work.d[111];
  work.v[112] = work.L[203]*work.d[112];
  work.v[113] = work.L[204]*work.d[113];
  work.v[114] = work.L[205]*work.d[114];
  work.v[116] = work.L[206]*work.d[116];
  work.v[117] = 0-work.L[199]*work.v[106]-work.L[200]*work.v[108]-work.L[201]*work.v[110]-work.L[202]*work.v[111]-work.L[203]*work.v[112]-work.L[204]*work.v[113]-work.L[205]*work.v[114]-work.L[206]*work.v[116];
  work.d[117] = work.v[117];
  if (work.d[117] > 0)
    work.d[117] = -settings.kkt_reg;
  else
    work.d[117] -= settings.kkt_reg;
  work.d_inv[117] = 1/work.d[117];
  work.L[215] = (-work.L[208]*work.v[108]-work.L[209]*work.v[110]-work.L[210]*work.v[111]-work.L[211]*work.v[112]-work.L[212]*work.v[113]-work.L[213]*work.v[114]-work.L[214]*work.v[116])*work.d_inv[117];
  work.L[224] = (-work.L[216]*work.v[108]-work.L[218]*work.v[110]-work.L[219]*work.v[111]-work.L[220]*work.v[112]-work.L[221]*work.v[113]-work.L[222]*work.v[114]-work.L[223]*work.v[116])*work.d_inv[117];
  work.L[230] = (-work.L[228]*work.v[113])*work.d_inv[117];
  work.L[238] = (-work.L[236]*work.v[114])*work.d_inv[117];
  work.L[247] = (-work.L[246]*work.v[116])*work.d_inv[117];
  work.v[107] = work.L[207]*work.d[107];
  work.v[108] = work.L[208]*work.d[108];
  work.v[110] = work.L[209]*work.d[110];
  work.v[111] = work.L[210]*work.d[111];
  work.v[112] = work.L[211]*work.d[112];
  work.v[113] = work.L[212]*work.d[113];
  work.v[114] = work.L[213]*work.d[114];
  work.v[116] = work.L[214]*work.d[116];
  work.v[117] = work.L[215]*work.d[117];
  work.v[118] = 0-work.L[207]*work.v[107]-work.L[208]*work.v[108]-work.L[209]*work.v[110]-work.L[210]*work.v[111]-work.L[211]*work.v[112]-work.L[212]*work.v[113]-work.L[213]*work.v[114]-work.L[214]*work.v[116]-work.L[215]*work.v[117];
  work.d[118] = work.v[118];
  if (work.d[118] > 0)
    work.d[118] = -settings.kkt_reg;
  else
    work.d[118] -= settings.kkt_reg;
  work.d_inv[118] = 1/work.d[118];
  work.L[225] = (-work.L[216]*work.v[108]-work.L[218]*work.v[110]-work.L[219]*work.v[111]-work.L[220]*work.v[112]-work.L[221]*work.v[113]-work.L[222]*work.v[114]-work.L[223]*work.v[116]-work.L[224]*work.v[117])*work.d_inv[118];
  work.L[231] = (-work.L[228]*work.v[113]-work.L[230]*work.v[117])*work.d_inv[118];
  work.L[239] = (-work.L[236]*work.v[114]-work.L[238]*work.v[117])*work.d_inv[118];
  work.L[248] = (-work.L[246]*work.v[116]-work.L[247]*work.v[117])*work.d_inv[118];
  work.v[108] = work.L[216]*work.d[108];
  work.v[109] = work.L[217]*work.d[109];
  work.v[110] = work.L[218]*work.d[110];
  work.v[111] = work.L[219]*work.d[111];
  work.v[112] = work.L[220]*work.d[112];
  work.v[113] = work.L[221]*work.d[113];
  work.v[114] = work.L[222]*work.d[114];
  work.v[116] = work.L[223]*work.d[116];
  work.v[117] = work.L[224]*work.d[117];
  work.v[118] = work.L[225]*work.d[118];
  work.v[119] = 0-work.L[216]*work.v[108]-work.L[217]*work.v[109]-work.L[218]*work.v[110]-work.L[219]*work.v[111]-work.L[220]*work.v[112]-work.L[221]*work.v[113]-work.L[222]*work.v[114]-work.L[223]*work.v[116]-work.L[224]*work.v[117]-work.L[225]*work.v[118];
  work.d[119] = work.v[119];
  if (work.d[119] > 0)
    work.d[119] = -settings.kkt_reg;
  else
    work.d[119] -= settings.kkt_reg;
  work.d_inv[119] = 1/work.d[119];
  work.L[232] = (-work.L[228]*work.v[113]-work.L[230]*work.v[117]-work.L[231]*work.v[118])*work.d_inv[119];
  work.L[240] = (-work.L[236]*work.v[114]-work.L[238]*work.v[117]-work.L[239]*work.v[118])*work.d_inv[119];
  work.L[249] = (-work.L[246]*work.v[116]-work.L[247]*work.v[117]-work.L[248]*work.v[118])*work.d_inv[119];
  work.v[120] = 0;
  work.d[120] = work.v[120];
  if (work.d[120] < 0)
    work.d[120] = settings.kkt_reg;
  else
    work.d[120] += settings.kkt_reg;
  work.d_inv[120] = 1/work.d[120];
  work.L[233] = (work.KKT[247])*work.d_inv[120];
  work.L[241] = (work.KKT[248])*work.d_inv[120];
  work.L[250] = (work.KKT[249])*work.d_inv[120];
  work.L[257] = (work.KKT[250])*work.d_inv[120];
  work.v[121] = 0;
  work.d[121] = work.v[121];
  if (work.d[121] < 0)
    work.d[121] = settings.kkt_reg;
  else
    work.d[121] += settings.kkt_reg;
  work.d_inv[121] = 1/work.d[121];
  work.L[234] = (work.KKT[251])*work.d_inv[121];
  work.L[242] = (work.KKT[252])*work.d_inv[121];
  work.L[251] = (work.KKT[253])*work.d_inv[121];
  work.L[265] = (work.KKT[254])*work.d_inv[121];
  work.v[51] = work.L[226]*work.d[51];
  work.v[52] = work.L[227]*work.d[52];
  work.v[122] = 0-work.L[226]*work.v[51]-work.L[227]*work.v[52];
  work.d[122] = work.v[122];
  if (work.d[122] < 0)
    work.d[122] = settings.kkt_reg;
  else
    work.d[122] += settings.kkt_reg;
  work.d_inv[122] = 1/work.d[122];
  work.L[258] = (work.KKT[255])*work.d_inv[122];
  work.L[266] = (work.KKT[256])*work.d_inv[122];
  work.L[274] = (work.KKT[257])*work.d_inv[122];
  work.v[123] = 0;
  work.d[123] = work.v[123];
  if (work.d[123] < 0)
    work.d[123] = settings.kkt_reg;
  else
    work.d[123] += settings.kkt_reg;
  work.d_inv[123] = 1/work.d[123];
  work.L[235] = (work.KKT[258])*work.d_inv[123];
  work.L[243] = (work.KKT[259])*work.d_inv[123];
  work.L[252] = (work.KKT[260])*work.d_inv[123];
  work.L[275] = (work.KKT[261])*work.d_inv[123];
  work.v[113] = work.L[228]*work.d[113];
  work.v[115] = work.L[229]*work.d[115];
  work.v[117] = work.L[230]*work.d[117];
  work.v[118] = work.L[231]*work.d[118];
  work.v[119] = work.L[232]*work.d[119];
  work.v[120] = work.L[233]*work.d[120];
  work.v[121] = work.L[234]*work.d[121];
  work.v[123] = work.L[235]*work.d[123];
  work.v[124] = 0-work.L[228]*work.v[113]-work.L[229]*work.v[115]-work.L[230]*work.v[117]-work.L[231]*work.v[118]-work.L[232]*work.v[119]-work.L[233]*work.v[120]-work.L[234]*work.v[121]-work.L[235]*work.v[123];
  work.d[124] = work.v[124];
  if (work.d[124] > 0)
    work.d[124] = -settings.kkt_reg;
  else
    work.d[124] -= settings.kkt_reg;
  work.d_inv[124] = 1/work.d[124];
  work.L[244] = (-work.L[237]*work.v[115]-work.L[238]*work.v[117]-work.L[239]*work.v[118]-work.L[240]*work.v[119]-work.L[241]*work.v[120]-work.L[242]*work.v[121]-work.L[243]*work.v[123])*work.d_inv[124];
  work.L[253] = (-work.L[245]*work.v[115]-work.L[247]*work.v[117]-work.L[248]*work.v[118]-work.L[249]*work.v[119]-work.L[250]*work.v[120]-work.L[251]*work.v[121]-work.L[252]*work.v[123])*work.d_inv[124];
  work.L[259] = (-work.L[257]*work.v[120])*work.d_inv[124];
  work.L[267] = (-work.L[265]*work.v[121])*work.d_inv[124];
  work.L[276] = (-work.L[275]*work.v[123])*work.d_inv[124];
  work.v[114] = work.L[236]*work.d[114];
  work.v[115] = work.L[237]*work.d[115];
  work.v[117] = work.L[238]*work.d[117];
  work.v[118] = work.L[239]*work.d[118];
  work.v[119] = work.L[240]*work.d[119];
  work.v[120] = work.L[241]*work.d[120];
  work.v[121] = work.L[242]*work.d[121];
  work.v[123] = work.L[243]*work.d[123];
  work.v[124] = work.L[244]*work.d[124];
  work.v[125] = 0-work.L[236]*work.v[114]-work.L[237]*work.v[115]-work.L[238]*work.v[117]-work.L[239]*work.v[118]-work.L[240]*work.v[119]-work.L[241]*work.v[120]-work.L[242]*work.v[121]-work.L[243]*work.v[123]-work.L[244]*work.v[124];
  work.d[125] = work.v[125];
  if (work.d[125] > 0)
    work.d[125] = -settings.kkt_reg;
  else
    work.d[125] -= settings.kkt_reg;
  work.d_inv[125] = 1/work.d[125];
  work.L[254] = (-work.L[245]*work.v[115]-work.L[247]*work.v[117]-work.L[248]*work.v[118]-work.L[249]*work.v[119]-work.L[250]*work.v[120]-work.L[251]*work.v[121]-work.L[252]*work.v[123]-work.L[253]*work.v[124])*work.d_inv[125];
  work.L[260] = (-work.L[257]*work.v[120]-work.L[259]*work.v[124])*work.d_inv[125];
  work.L[268] = (-work.L[265]*work.v[121]-work.L[267]*work.v[124])*work.d_inv[125];
  work.L[277] = (-work.L[275]*work.v[123]-work.L[276]*work.v[124])*work.d_inv[125];
  work.v[115] = work.L[245]*work.d[115];
  work.v[116] = work.L[246]*work.d[116];
  work.v[117] = work.L[247]*work.d[117];
  work.v[118] = work.L[248]*work.d[118];
  work.v[119] = work.L[249]*work.d[119];
  work.v[120] = work.L[250]*work.d[120];
  work.v[121] = work.L[251]*work.d[121];
  work.v[123] = work.L[252]*work.d[123];
  work.v[124] = work.L[253]*work.d[124];
  work.v[125] = work.L[254]*work.d[125];
  work.v[126] = 0-work.L[245]*work.v[115]-work.L[246]*work.v[116]-work.L[247]*work.v[117]-work.L[248]*work.v[118]-work.L[249]*work.v[119]-work.L[250]*work.v[120]-work.L[251]*work.v[121]-work.L[252]*work.v[123]-work.L[253]*work.v[124]-work.L[254]*work.v[125];
  work.d[126] = work.v[126];
  if (work.d[126] > 0)
    work.d[126] = -settings.kkt_reg;
  else
    work.d[126] -= settings.kkt_reg;
  work.d_inv[126] = 1/work.d[126];
  work.L[261] = (-work.L[257]*work.v[120]-work.L[259]*work.v[124]-work.L[260]*work.v[125])*work.d_inv[126];
  work.L[269] = (-work.L[265]*work.v[121]-work.L[267]*work.v[124]-work.L[268]*work.v[125])*work.d_inv[126];
  work.L[278] = (-work.L[275]*work.v[123]-work.L[276]*work.v[124]-work.L[277]*work.v[125])*work.d_inv[126];
  work.v[127] = 0;
  work.d[127] = work.v[127];
  if (work.d[127] < 0)
    work.d[127] = settings.kkt_reg;
  else
    work.d[127] += settings.kkt_reg;
  work.d_inv[127] = 1/work.d[127];
  work.L[262] = (work.KKT[262])*work.d_inv[127];
  work.L[270] = (work.KKT[263])*work.d_inv[127];
  work.L[279] = (work.KKT[264])*work.d_inv[127];
  work.L[286] = (work.KKT[265])*work.d_inv[127];
  work.v[128] = 0;
  work.d[128] = work.v[128];
  if (work.d[128] < 0)
    work.d[128] = settings.kkt_reg;
  else
    work.d[128] += settings.kkt_reg;
  work.d_inv[128] = 1/work.d[128];
  work.L[263] = (work.KKT[266])*work.d_inv[128];
  work.L[271] = (work.KKT[267])*work.d_inv[128];
  work.L[280] = (work.KKT[268])*work.d_inv[128];
  work.L[294] = (work.KKT[269])*work.d_inv[128];
  work.v[47] = work.L[255]*work.d[47];
  work.v[48] = work.L[256]*work.d[48];
  work.v[129] = 0-work.L[255]*work.v[47]-work.L[256]*work.v[48];
  work.d[129] = work.v[129];
  if (work.d[129] < 0)
    work.d[129] = settings.kkt_reg;
  else
    work.d[129] += settings.kkt_reg;
  work.d_inv[129] = 1/work.d[129];
  work.L[287] = (work.KKT[270])*work.d_inv[129];
  work.L[295] = (work.KKT[271])*work.d_inv[129];
  work.L[303] = (work.KKT[272])*work.d_inv[129];
  work.v[130] = 0;
  work.d[130] = work.v[130];
  if (work.d[130] < 0)
    work.d[130] = settings.kkt_reg;
  else
    work.d[130] += settings.kkt_reg;
  work.d_inv[130] = 1/work.d[130];
  work.L[264] = (work.KKT[273])*work.d_inv[130];
  work.L[272] = (work.KKT[274])*work.d_inv[130];
  work.L[281] = (work.KKT[275])*work.d_inv[130];
  work.L[304] = (work.KKT[276])*work.d_inv[130];
  work.v[120] = work.L[257]*work.d[120];
  work.v[122] = work.L[258]*work.d[122];
  work.v[124] = work.L[259]*work.d[124];
  work.v[125] = work.L[260]*work.d[125];
  work.v[126] = work.L[261]*work.d[126];
  work.v[127] = work.L[262]*work.d[127];
  work.v[128] = work.L[263]*work.d[128];
  work.v[130] = work.L[264]*work.d[130];
  work.v[131] = 0-work.L[257]*work.v[120]-work.L[258]*work.v[122]-work.L[259]*work.v[124]-work.L[260]*work.v[125]-work.L[261]*work.v[126]-work.L[262]*work.v[127]-work.L[263]*work.v[128]-work.L[264]*work.v[130];
  work.d[131] = work.v[131];
  if (work.d[131] > 0)
    work.d[131] = -settings.kkt_reg;
  else
    work.d[131] -= settings.kkt_reg;
  work.d_inv[131] = 1/work.d[131];
  work.L[273] = (-work.L[266]*work.v[122]-work.L[267]*work.v[124]-work.L[268]*work.v[125]-work.L[269]*work.v[126]-work.L[270]*work.v[127]-work.L[271]*work.v[128]-work.L[272]*work.v[130])*work.d_inv[131];
  work.L[282] = (-work.L[274]*work.v[122]-work.L[276]*work.v[124]-work.L[277]*work.v[125]-work.L[278]*work.v[126]-work.L[279]*work.v[127]-work.L[280]*work.v[128]-work.L[281]*work.v[130])*work.d_inv[131];
  work.L[288] = (-work.L[286]*work.v[127])*work.d_inv[131];
  work.L[296] = (-work.L[294]*work.v[128])*work.d_inv[131];
  work.L[305] = (-work.L[304]*work.v[130])*work.d_inv[131];
  work.v[121] = work.L[265]*work.d[121];
  work.v[122] = work.L[266]*work.d[122];
  work.v[124] = work.L[267]*work.d[124];
  work.v[125] = work.L[268]*work.d[125];
  work.v[126] = work.L[269]*work.d[126];
  work.v[127] = work.L[270]*work.d[127];
  work.v[128] = work.L[271]*work.d[128];
  work.v[130] = work.L[272]*work.d[130];
  work.v[131] = work.L[273]*work.d[131];
  work.v[132] = 0-work.L[265]*work.v[121]-work.L[266]*work.v[122]-work.L[267]*work.v[124]-work.L[268]*work.v[125]-work.L[269]*work.v[126]-work.L[270]*work.v[127]-work.L[271]*work.v[128]-work.L[272]*work.v[130]-work.L[273]*work.v[131];
  work.d[132] = work.v[132];
  if (work.d[132] > 0)
    work.d[132] = -settings.kkt_reg;
  else
    work.d[132] -= settings.kkt_reg;
  work.d_inv[132] = 1/work.d[132];
  work.L[283] = (-work.L[274]*work.v[122]-work.L[276]*work.v[124]-work.L[277]*work.v[125]-work.L[278]*work.v[126]-work.L[279]*work.v[127]-work.L[280]*work.v[128]-work.L[281]*work.v[130]-work.L[282]*work.v[131])*work.d_inv[132];
  work.L[289] = (-work.L[286]*work.v[127]-work.L[288]*work.v[131])*work.d_inv[132];
  work.L[297] = (-work.L[294]*work.v[128]-work.L[296]*work.v[131])*work.d_inv[132];
  work.L[306] = (-work.L[304]*work.v[130]-work.L[305]*work.v[131])*work.d_inv[132];
  work.v[122] = work.L[274]*work.d[122];
  work.v[123] = work.L[275]*work.d[123];
  work.v[124] = work.L[276]*work.d[124];
  work.v[125] = work.L[277]*work.d[125];
  work.v[126] = work.L[278]*work.d[126];
  work.v[127] = work.L[279]*work.d[127];
  work.v[128] = work.L[280]*work.d[128];
  work.v[130] = work.L[281]*work.d[130];
  work.v[131] = work.L[282]*work.d[131];
  work.v[132] = work.L[283]*work.d[132];
  work.v[133] = 0-work.L[274]*work.v[122]-work.L[275]*work.v[123]-work.L[276]*work.v[124]-work.L[277]*work.v[125]-work.L[278]*work.v[126]-work.L[279]*work.v[127]-work.L[280]*work.v[128]-work.L[281]*work.v[130]-work.L[282]*work.v[131]-work.L[283]*work.v[132];
  work.d[133] = work.v[133];
  if (work.d[133] > 0)
    work.d[133] = -settings.kkt_reg;
  else
    work.d[133] -= settings.kkt_reg;
  work.d_inv[133] = 1/work.d[133];
  work.L[290] = (-work.L[286]*work.v[127]-work.L[288]*work.v[131]-work.L[289]*work.v[132])*work.d_inv[133];
  work.L[298] = (-work.L[294]*work.v[128]-work.L[296]*work.v[131]-work.L[297]*work.v[132])*work.d_inv[133];
  work.L[307] = (-work.L[304]*work.v[130]-work.L[305]*work.v[131]-work.L[306]*work.v[132])*work.d_inv[133];
  work.v[134] = 0;
  work.d[134] = work.v[134];
  if (work.d[134] < 0)
    work.d[134] = settings.kkt_reg;
  else
    work.d[134] += settings.kkt_reg;
  work.d_inv[134] = 1/work.d[134];
  work.L[291] = (work.KKT[277])*work.d_inv[134];
  work.L[299] = (work.KKT[278])*work.d_inv[134];
  work.L[308] = (work.KKT[279])*work.d_inv[134];
  work.L[342] = (work.KKT[280])*work.d_inv[134];
  work.v[135] = 0;
  work.d[135] = work.v[135];
  if (work.d[135] < 0)
    work.d[135] = settings.kkt_reg;
  else
    work.d[135] += settings.kkt_reg;
  work.d_inv[135] = 1/work.d[135];
  work.L[292] = (work.KKT[281])*work.d_inv[135];
  work.L[300] = (work.KKT[282])*work.d_inv[135];
  work.L[309] = (work.KKT[283])*work.d_inv[135];
  work.L[353] = (work.KKT[284])*work.d_inv[135];
  work.v[43] = work.L[284]*work.d[43];
  work.v[44] = work.L[285]*work.d[44];
  work.v[136] = 0-work.L[284]*work.v[43]-work.L[285]*work.v[44];
  work.d[136] = work.v[136];
  if (work.d[136] < 0)
    work.d[136] = settings.kkt_reg;
  else
    work.d[136] += settings.kkt_reg;
  work.d_inv[136] = 1/work.d[136];
  work.L[343] = (work.KKT[285])*work.d_inv[136];
  work.L[354] = (work.KKT[286])*work.d_inv[136];
  work.L[365] = (work.KKT[287])*work.d_inv[136];
  work.v[137] = 0;
  work.d[137] = work.v[137];
  if (work.d[137] < 0)
    work.d[137] = settings.kkt_reg;
  else
    work.d[137] += settings.kkt_reg;
  work.d_inv[137] = 1/work.d[137];
  work.L[293] = (work.KKT[288])*work.d_inv[137];
  work.L[301] = (work.KKT[289])*work.d_inv[137];
  work.L[310] = (work.KKT[290])*work.d_inv[137];
  work.L[366] = (work.KKT[291])*work.d_inv[137];
  work.v[127] = work.L[286]*work.d[127];
  work.v[129] = work.L[287]*work.d[129];
  work.v[131] = work.L[288]*work.d[131];
  work.v[132] = work.L[289]*work.d[132];
  work.v[133] = work.L[290]*work.d[133];
  work.v[134] = work.L[291]*work.d[134];
  work.v[135] = work.L[292]*work.d[135];
  work.v[137] = work.L[293]*work.d[137];
  work.v[138] = 0-work.L[286]*work.v[127]-work.L[287]*work.v[129]-work.L[288]*work.v[131]-work.L[289]*work.v[132]-work.L[290]*work.v[133]-work.L[291]*work.v[134]-work.L[292]*work.v[135]-work.L[293]*work.v[137];
  work.d[138] = work.v[138];
  if (work.d[138] > 0)
    work.d[138] = -settings.kkt_reg;
  else
    work.d[138] -= settings.kkt_reg;
  work.d_inv[138] = 1/work.d[138];
  work.L[302] = (-work.L[295]*work.v[129]-work.L[296]*work.v[131]-work.L[297]*work.v[132]-work.L[298]*work.v[133]-work.L[299]*work.v[134]-work.L[300]*work.v[135]-work.L[301]*work.v[137])*work.d_inv[138];
  work.L[311] = (-work.L[303]*work.v[129]-work.L[305]*work.v[131]-work.L[306]*work.v[132]-work.L[307]*work.v[133]-work.L[308]*work.v[134]-work.L[309]*work.v[135]-work.L[310]*work.v[137])*work.d_inv[138];
  work.L[344] = (-work.L[342]*work.v[134])*work.d_inv[138];
  work.L[355] = (-work.L[353]*work.v[135])*work.d_inv[138];
  work.L[367] = (-work.L[366]*work.v[137])*work.d_inv[138];
  work.v[128] = work.L[294]*work.d[128];
  work.v[129] = work.L[295]*work.d[129];
  work.v[131] = work.L[296]*work.d[131];
  work.v[132] = work.L[297]*work.d[132];
  work.v[133] = work.L[298]*work.d[133];
  work.v[134] = work.L[299]*work.d[134];
  work.v[135] = work.L[300]*work.d[135];
  work.v[137] = work.L[301]*work.d[137];
  work.v[138] = work.L[302]*work.d[138];
  work.v[139] = 0-work.L[294]*work.v[128]-work.L[295]*work.v[129]-work.L[296]*work.v[131]-work.L[297]*work.v[132]-work.L[298]*work.v[133]-work.L[299]*work.v[134]-work.L[300]*work.v[135]-work.L[301]*work.v[137]-work.L[302]*work.v[138];
  work.d[139] = work.v[139];
  if (work.d[139] > 0)
    work.d[139] = -settings.kkt_reg;
  else
    work.d[139] -= settings.kkt_reg;
  work.d_inv[139] = 1/work.d[139];
  work.L[312] = (-work.L[303]*work.v[129]-work.L[305]*work.v[131]-work.L[306]*work.v[132]-work.L[307]*work.v[133]-work.L[308]*work.v[134]-work.L[309]*work.v[135]-work.L[310]*work.v[137]-work.L[311]*work.v[138])*work.d_inv[139];
  work.L[345] = (-work.L[342]*work.v[134]-work.L[344]*work.v[138])*work.d_inv[139];
  work.L[356] = (-work.L[353]*work.v[135]-work.L[355]*work.v[138])*work.d_inv[139];
  work.L[368] = (-work.L[366]*work.v[137]-work.L[367]*work.v[138])*work.d_inv[139];
  work.v[129] = work.L[303]*work.d[129];
  work.v[130] = work.L[304]*work.d[130];
  work.v[131] = work.L[305]*work.d[131];
  work.v[132] = work.L[306]*work.d[132];
  work.v[133] = work.L[307]*work.d[133];
  work.v[134] = work.L[308]*work.d[134];
  work.v[135] = work.L[309]*work.d[135];
  work.v[137] = work.L[310]*work.d[137];
  work.v[138] = work.L[311]*work.d[138];
  work.v[139] = work.L[312]*work.d[139];
  work.v[140] = 0-work.L[303]*work.v[129]-work.L[304]*work.v[130]-work.L[305]*work.v[131]-work.L[306]*work.v[132]-work.L[307]*work.v[133]-work.L[308]*work.v[134]-work.L[309]*work.v[135]-work.L[310]*work.v[137]-work.L[311]*work.v[138]-work.L[312]*work.v[139];
  work.d[140] = work.v[140];
  if (work.d[140] > 0)
    work.d[140] = -settings.kkt_reg;
  else
    work.d[140] -= settings.kkt_reg;
  work.d_inv[140] = 1/work.d[140];
  work.L[346] = (-work.L[342]*work.v[134]-work.L[344]*work.v[138]-work.L[345]*work.v[139])*work.d_inv[140];
  work.L[357] = (-work.L[353]*work.v[135]-work.L[355]*work.v[138]-work.L[356]*work.v[139])*work.d_inv[140];
  work.L[369] = (-work.L[366]*work.v[137]-work.L[367]*work.v[138]-work.L[368]*work.v[139])*work.d_inv[140];
  work.v[141] = 0;
  work.d[141] = work.v[141];
  if (work.d[141] < 0)
    work.d[141] = settings.kkt_reg;
  else
    work.d[141] += settings.kkt_reg;
  work.d_inv[141] = 1/work.d[141];
  work.L[324] = (work.KKT[292])*work.d_inv[141];
  work.L[347] = (work.KKT[293])*work.d_inv[141];
  work.L[358] = (work.KKT[294])*work.d_inv[141];
  work.L[370] = (work.KKT[295])*work.d_inv[141];
  work.v[142] = 0;
  work.d[142] = work.v[142];
  if (work.d[142] < 0)
    work.d[142] = settings.kkt_reg;
  else
    work.d[142] += settings.kkt_reg;
  work.d_inv[142] = 1/work.d[142];
  work.L[329] = (work.KKT[296])*work.d_inv[142];
  work.L[348] = (work.KKT[297])*work.d_inv[142];
  work.L[359] = (work.KKT[298])*work.d_inv[142];
  work.L[371] = (work.KKT[299])*work.d_inv[142];
  work.v[39] = work.L[313]*work.d[39];
  work.v[40] = work.L[314]*work.d[40];
  work.v[143] = 0-work.L[313]*work.v[39]-work.L[314]*work.v[40];
  work.d[143] = work.v[143];
  if (work.d[143] < 0)
    work.d[143] = settings.kkt_reg;
  else
    work.d[143] += settings.kkt_reg;
  work.d_inv[143] = 1/work.d[143];
  work.L[325] = (work.KKT[300])*work.d_inv[143];
  work.L[330] = (work.KKT[301])*work.d_inv[143];
  work.L[335] = (work.KKT[302])*work.d_inv[143];
  work.v[77] = work.L[315]*work.d[77];
  work.v[80] = work.L[316]*work.d[80];
  work.v[144] = 0-work.L[315]*work.v[77]-work.L[316]*work.v[80];
  work.d[144] = work.v[144];
  if (work.d[144] < 0)
    work.d[144] = settings.kkt_reg;
  else
    work.d[144] += settings.kkt_reg;
  work.d_inv[144] = 1/work.d[144];
  work.L[319] = (-work.L[318]*work.v[80])*work.d_inv[144];
  work.L[322] = (-work.L[321]*work.v[80])*work.d_inv[144];
  work.L[326] = (work.KKT[303])*work.d_inv[144];
  work.L[331] = (work.KKT[304])*work.d_inv[144];
  work.L[336] = (work.KKT[305])*work.d_inv[144];
  work.v[78] = work.L[317]*work.d[78];
  work.v[80] = work.L[318]*work.d[80];
  work.v[144] = work.L[319]*work.d[144];
  work.v[145] = 0-work.L[317]*work.v[78]-work.L[318]*work.v[80]-work.L[319]*work.v[144];
  work.d[145] = work.v[145];
  if (work.d[145] < 0)
    work.d[145] = settings.kkt_reg;
  else
    work.d[145] += settings.kkt_reg;
  work.d_inv[145] = 1/work.d[145];
  work.L[323] = (-work.L[321]*work.v[80]-work.L[322]*work.v[144])*work.d_inv[145];
  work.L[327] = (work.KKT[306]-work.L[326]*work.v[144])*work.d_inv[145];
  work.L[332] = (work.KKT[307]-work.L[331]*work.v[144])*work.d_inv[145];
  work.L[337] = (work.KKT[308]-work.L[336]*work.v[144])*work.d_inv[145];
  work.v[79] = work.L[320]*work.d[79];
  work.v[80] = work.L[321]*work.d[80];
  work.v[144] = work.L[322]*work.d[144];
  work.v[145] = work.L[323]*work.d[145];
  work.v[146] = 0-work.L[320]*work.v[79]-work.L[321]*work.v[80]-work.L[322]*work.v[144]-work.L[323]*work.v[145];
  work.d[146] = work.v[146];
  if (work.d[146] < 0)
    work.d[146] = settings.kkt_reg;
  else
    work.d[146] += settings.kkt_reg;
  work.d_inv[146] = 1/work.d[146];
  work.L[328] = (work.KKT[309]-work.L[326]*work.v[144]-work.L[327]*work.v[145])*work.d_inv[146];
  work.L[333] = (work.KKT[310]-work.L[331]*work.v[144]-work.L[332]*work.v[145])*work.d_inv[146];
  work.L[338] = (work.KKT[311]-work.L[336]*work.v[144]-work.L[337]*work.v[145])*work.d_inv[146];
  work.v[147] = 0;
  work.d[147] = work.v[147];
  if (work.d[147] < 0)
    work.d[147] = settings.kkt_reg;
  else
    work.d[147] += settings.kkt_reg;
  work.d_inv[147] = 1/work.d[147];
  work.L[339] = (work.KKT[312])*work.d_inv[147];
  work.L[349] = (work.KKT[313])*work.d_inv[147];
  work.L[360] = (work.KKT[314])*work.d_inv[147];
  work.L[372] = (work.KKT[315])*work.d_inv[147];
  work.v[141] = work.L[324]*work.d[141];
  work.v[143] = work.L[325]*work.d[143];
  work.v[144] = work.L[326]*work.d[144];
  work.v[145] = work.L[327]*work.d[145];
  work.v[146] = work.L[328]*work.d[146];
  work.v[148] = 0-work.L[324]*work.v[141]-work.L[325]*work.v[143]-work.L[326]*work.v[144]-work.L[327]*work.v[145]-work.L[328]*work.v[146];
  work.d[148] = work.v[148];
  if (work.d[148] > 0)
    work.d[148] = -settings.kkt_reg;
  else
    work.d[148] -= settings.kkt_reg;
  work.d_inv[148] = 1/work.d[148];
  work.L[334] = (-work.L[330]*work.v[143]-work.L[331]*work.v[144]-work.L[332]*work.v[145]-work.L[333]*work.v[146])*work.d_inv[148];
  work.L[340] = (-work.L[335]*work.v[143]-work.L[336]*work.v[144]-work.L[337]*work.v[145]-work.L[338]*work.v[146])*work.d_inv[148];
  work.L[350] = (-work.L[347]*work.v[141])*work.d_inv[148];
  work.L[361] = (-work.L[358]*work.v[141])*work.d_inv[148];
  work.L[373] = (-work.L[370]*work.v[141])*work.d_inv[148];
  work.v[142] = work.L[329]*work.d[142];
  work.v[143] = work.L[330]*work.d[143];
  work.v[144] = work.L[331]*work.d[144];
  work.v[145] = work.L[332]*work.d[145];
  work.v[146] = work.L[333]*work.d[146];
  work.v[148] = work.L[334]*work.d[148];
  work.v[149] = 0-work.L[329]*work.v[142]-work.L[330]*work.v[143]-work.L[331]*work.v[144]-work.L[332]*work.v[145]-work.L[333]*work.v[146]-work.L[334]*work.v[148];
  work.d[149] = work.v[149];
  if (work.d[149] > 0)
    work.d[149] = -settings.kkt_reg;
  else
    work.d[149] -= settings.kkt_reg;
  work.d_inv[149] = 1/work.d[149];
  work.L[341] = (-work.L[335]*work.v[143]-work.L[336]*work.v[144]-work.L[337]*work.v[145]-work.L[338]*work.v[146]-work.L[340]*work.v[148])*work.d_inv[149];
  work.L[351] = (-work.L[348]*work.v[142]-work.L[350]*work.v[148])*work.d_inv[149];
  work.L[362] = (-work.L[359]*work.v[142]-work.L[361]*work.v[148])*work.d_inv[149];
  work.L[374] = (-work.L[371]*work.v[142]-work.L[373]*work.v[148])*work.d_inv[149];
  work.v[143] = work.L[335]*work.d[143];
  work.v[144] = work.L[336]*work.d[144];
  work.v[145] = work.L[337]*work.d[145];
  work.v[146] = work.L[338]*work.d[146];
  work.v[147] = work.L[339]*work.d[147];
  work.v[148] = work.L[340]*work.d[148];
  work.v[149] = work.L[341]*work.d[149];
  work.v[150] = 0-work.L[335]*work.v[143]-work.L[336]*work.v[144]-work.L[337]*work.v[145]-work.L[338]*work.v[146]-work.L[339]*work.v[147]-work.L[340]*work.v[148]-work.L[341]*work.v[149];
  work.d[150] = work.v[150];
  if (work.d[150] > 0)
    work.d[150] = -settings.kkt_reg;
  else
    work.d[150] -= settings.kkt_reg;
  work.d_inv[150] = 1/work.d[150];
  work.L[352] = (-work.L[349]*work.v[147]-work.L[350]*work.v[148]-work.L[351]*work.v[149])*work.d_inv[150];
  work.L[363] = (-work.L[360]*work.v[147]-work.L[361]*work.v[148]-work.L[362]*work.v[149])*work.d_inv[150];
  work.L[375] = (-work.L[372]*work.v[147]-work.L[373]*work.v[148]-work.L[374]*work.v[149])*work.d_inv[150];
  work.v[134] = work.L[342]*work.d[134];
  work.v[136] = work.L[343]*work.d[136];
  work.v[138] = work.L[344]*work.d[138];
  work.v[139] = work.L[345]*work.d[139];
  work.v[140] = work.L[346]*work.d[140];
  work.v[141] = work.L[347]*work.d[141];
  work.v[142] = work.L[348]*work.d[142];
  work.v[147] = work.L[349]*work.d[147];
  work.v[148] = work.L[350]*work.d[148];
  work.v[149] = work.L[351]*work.d[149];
  work.v[150] = work.L[352]*work.d[150];
  work.v[151] = 0-work.L[342]*work.v[134]-work.L[343]*work.v[136]-work.L[344]*work.v[138]-work.L[345]*work.v[139]-work.L[346]*work.v[140]-work.L[347]*work.v[141]-work.L[348]*work.v[142]-work.L[349]*work.v[147]-work.L[350]*work.v[148]-work.L[351]*work.v[149]-work.L[352]*work.v[150];
  work.d[151] = work.v[151];
  if (work.d[151] > 0)
    work.d[151] = -settings.kkt_reg;
  else
    work.d[151] -= settings.kkt_reg;
  work.d_inv[151] = 1/work.d[151];
  work.L[364] = (-work.L[354]*work.v[136]-work.L[355]*work.v[138]-work.L[356]*work.v[139]-work.L[357]*work.v[140]-work.L[358]*work.v[141]-work.L[359]*work.v[142]-work.L[360]*work.v[147]-work.L[361]*work.v[148]-work.L[362]*work.v[149]-work.L[363]*work.v[150])*work.d_inv[151];
  work.L[376] = (-work.L[365]*work.v[136]-work.L[367]*work.v[138]-work.L[368]*work.v[139]-work.L[369]*work.v[140]-work.L[370]*work.v[141]-work.L[371]*work.v[142]-work.L[372]*work.v[147]-work.L[373]*work.v[148]-work.L[374]*work.v[149]-work.L[375]*work.v[150])*work.d_inv[151];
  work.v[135] = work.L[353]*work.d[135];
  work.v[136] = work.L[354]*work.d[136];
  work.v[138] = work.L[355]*work.d[138];
  work.v[139] = work.L[356]*work.d[139];
  work.v[140] = work.L[357]*work.d[140];
  work.v[141] = work.L[358]*work.d[141];
  work.v[142] = work.L[359]*work.d[142];
  work.v[147] = work.L[360]*work.d[147];
  work.v[148] = work.L[361]*work.d[148];
  work.v[149] = work.L[362]*work.d[149];
  work.v[150] = work.L[363]*work.d[150];
  work.v[151] = work.L[364]*work.d[151];
  work.v[152] = 0-work.L[353]*work.v[135]-work.L[354]*work.v[136]-work.L[355]*work.v[138]-work.L[356]*work.v[139]-work.L[357]*work.v[140]-work.L[358]*work.v[141]-work.L[359]*work.v[142]-work.L[360]*work.v[147]-work.L[361]*work.v[148]-work.L[362]*work.v[149]-work.L[363]*work.v[150]-work.L[364]*work.v[151];
  work.d[152] = work.v[152];
  if (work.d[152] > 0)
    work.d[152] = -settings.kkt_reg;
  else
    work.d[152] -= settings.kkt_reg;
  work.d_inv[152] = 1/work.d[152];
  work.L[377] = (-work.L[365]*work.v[136]-work.L[367]*work.v[138]-work.L[368]*work.v[139]-work.L[369]*work.v[140]-work.L[370]*work.v[141]-work.L[371]*work.v[142]-work.L[372]*work.v[147]-work.L[373]*work.v[148]-work.L[374]*work.v[149]-work.L[375]*work.v[150]-work.L[376]*work.v[151])*work.d_inv[152];
  work.v[136] = work.L[365]*work.d[136];
  work.v[137] = work.L[366]*work.d[137];
  work.v[138] = work.L[367]*work.d[138];
  work.v[139] = work.L[368]*work.d[139];
  work.v[140] = work.L[369]*work.d[140];
  work.v[141] = work.L[370]*work.d[141];
  work.v[142] = work.L[371]*work.d[142];
  work.v[147] = work.L[372]*work.d[147];
  work.v[148] = work.L[373]*work.d[148];
  work.v[149] = work.L[374]*work.d[149];
  work.v[150] = work.L[375]*work.d[150];
  work.v[151] = work.L[376]*work.d[151];
  work.v[152] = work.L[377]*work.d[152];
  work.v[153] = 0-work.L[365]*work.v[136]-work.L[366]*work.v[137]-work.L[367]*work.v[138]-work.L[368]*work.v[139]-work.L[369]*work.v[140]-work.L[370]*work.v[141]-work.L[371]*work.v[142]-work.L[372]*work.v[147]-work.L[373]*work.v[148]-work.L[374]*work.v[149]-work.L[375]*work.v[150]-work.L[376]*work.v[151]-work.L[377]*work.v[152];
  work.d[153] = work.v[153];
  if (work.d[153] > 0)
    work.d[153] = -settings.kkt_reg;
  else
    work.d[153] -= settings.kkt_reg;
  work.d_inv[153] = 1/work.d[153];
#ifndef ZERO_LIBRARY_MODE
  if (settings.debug) {
    printf("Squared Frobenius for factorization is %.8g.\n", check_factorization());
  }
#endif
}
double check_factorization(void) {
  /* Returns the squared Frobenius norm of A - L*D*L'. */
  double temp, residual;
  /* Only check the lower triangle. */
  residual = 0;
  temp = work.KKT[160]-1*work.d[81]*1;
  residual += temp*temp;
  temp = work.KKT[161]-work.L[82]*work.d[81]*1;
  residual += temp*temp;
  temp = work.KKT[162]-work.L[85]*work.d[81]*1;
  residual += temp*temp;
  temp = work.KKT[164]-work.L[82]*work.d[81]*work.L[82]-1*work.d[82]*1;
  residual += temp*temp;
  temp = work.KKT[165]-work.L[85]*work.d[81]*work.L[82]-work.L[86]*work.d[82]*1;
  residual += temp*temp;
  temp = work.KKT[170]-work.L[85]*work.d[81]*work.L[85]-work.L[86]*work.d[82]*work.L[86]-1*work.d[84]*1;
  residual += temp*temp;
  temp = work.KKT[0]-1*work.d[0]*1;
  residual += temp*temp;
  temp = work.KKT[2]-1*work.d[1]*1;
  residual += temp*temp;
  temp = work.KKT[4]-1*work.d[2]*1;
  residual += temp*temp;
  temp = work.KKT[6]-1*work.d[3]*1;
  residual += temp*temp;
  temp = work.KKT[8]-1*work.d[4]*1;
  residual += temp*temp;
  temp = work.KKT[10]-1*work.d[5]*1;
  residual += temp*temp;
  temp = work.KKT[12]-1*work.d[6]*1;
  residual += temp*temp;
  temp = work.KKT[14]-1*work.d[7]*1;
  residual += temp*temp;
  temp = work.KKT[16]-1*work.d[8]*1;
  residual += temp*temp;
  temp = work.KKT[18]-1*work.d[9]*1;
  residual += temp*temp;
  temp = work.KKT[20]-1*work.d[10]*1;
  residual += temp*temp;
  temp = work.KKT[22]-1*work.d[11]*1;
  residual += temp*temp;
  temp = work.KKT[24]-1*work.d[12]*1;
  residual += temp*temp;
  temp = work.KKT[26]-1*work.d[13]*1;
  residual += temp*temp;
  temp = work.KKT[28]-1*work.d[14]*1;
  residual += temp*temp;
  temp = work.KKT[30]-1*work.d[15]*1;
  residual += temp*temp;
  temp = work.KKT[32]-1*work.d[16]*1;
  residual += temp*temp;
  temp = work.KKT[34]-1*work.d[17]*1;
  residual += temp*temp;
  temp = work.KKT[36]-1*work.d[18]*1;
  residual += temp*temp;
  temp = work.KKT[38]-1*work.d[19]*1;
  residual += temp*temp;
  temp = work.KKT[40]-1*work.d[20]*1;
  residual += temp*temp;
  temp = work.KKT[42]-1*work.d[21]*1;
  residual += temp*temp;
  temp = work.KKT[44]-1*work.d[22]*1;
  residual += temp*temp;
  temp = work.KKT[46]-1*work.d[23]*1;
  residual += temp*temp;
  temp = work.KKT[48]-1*work.d[24]*1;
  residual += temp*temp;
  temp = work.KKT[50]-1*work.d[25]*1;
  residual += temp*temp;
  temp = work.KKT[52]-1*work.d[26]*1;
  residual += temp*temp;
  temp = work.KKT[54]-1*work.d[27]*1;
  residual += temp*temp;
  temp = work.KKT[56]-1*work.d[28]*1;
  residual += temp*temp;
  temp = work.KKT[58]-1*work.d[29]*1;
  residual += temp*temp;
  temp = work.KKT[60]-1*work.d[30]*1;
  residual += temp*temp;
  temp = work.KKT[62]-1*work.d[31]*1;
  residual += temp*temp;
  temp = work.KKT[64]-1*work.d[32]*1;
  residual += temp*temp;
  temp = work.KKT[1]-work.L[0]*work.d[0]*1;
  residual += temp*temp;
  temp = work.KKT[3]-work.L[2]*work.d[1]*1;
  residual += temp*temp;
  temp = work.KKT[5]-work.L[4]*work.d[2]*1;
  residual += temp*temp;
  temp = work.KKT[7]-work.L[7]*work.d[3]*1;
  residual += temp*temp;
  temp = work.KKT[9]-work.L[9]*work.d[4]*1;
  residual += temp*temp;
  temp = work.KKT[11]-work.L[11]*work.d[5]*1;
  residual += temp*temp;
  temp = work.KKT[13]-work.L[14]*work.d[6]*1;
  residual += temp*temp;
  temp = work.KKT[15]-work.L[16]*work.d[7]*1;
  residual += temp*temp;
  temp = work.KKT[17]-work.L[18]*work.d[8]*1;
  residual += temp*temp;
  temp = work.KKT[19]-work.L[21]*work.d[9]*1;
  residual += temp*temp;
  temp = work.KKT[21]-work.L[23]*work.d[10]*1;
  residual += temp*temp;
  temp = work.KKT[23]-work.L[25]*work.d[11]*1;
  residual += temp*temp;
  temp = work.KKT[25]-work.L[28]*work.d[12]*1;
  residual += temp*temp;
  temp = work.KKT[27]-work.L[30]*work.d[13]*1;
  residual += temp*temp;
  temp = work.KKT[29]-work.L[32]*work.d[14]*1;
  residual += temp*temp;
  temp = work.KKT[31]-work.L[35]*work.d[15]*1;
  residual += temp*temp;
  temp = work.KKT[33]-work.L[37]*work.d[16]*1;
  residual += temp*temp;
  temp = work.KKT[35]-work.L[39]*work.d[17]*1;
  residual += temp*temp;
  temp = work.KKT[37]-work.L[42]*work.d[18]*1;
  residual += temp*temp;
  temp = work.KKT[39]-work.L[44]*work.d[19]*1;
  residual += temp*temp;
  temp = work.KKT[41]-work.L[46]*work.d[20]*1;
  residual += temp*temp;
  temp = work.KKT[43]-work.L[49]*work.d[21]*1;
  residual += temp*temp;
  temp = work.KKT[45]-work.L[51]*work.d[22]*1;
  residual += temp*temp;
  temp = work.KKT[47]-work.L[53]*work.d[23]*1;
  residual += temp*temp;
  temp = work.KKT[49]-work.L[56]*work.d[24]*1;
  residual += temp*temp;
  temp = work.KKT[51]-work.L[58]*work.d[25]*1;
  residual += temp*temp;
  temp = work.KKT[53]-work.L[60]*work.d[26]*1;
  residual += temp*temp;
  temp = work.KKT[55]-work.L[63]*work.d[27]*1;
  residual += temp*temp;
  temp = work.KKT[57]-work.L[65]*work.d[28]*1;
  residual += temp*temp;
  temp = work.KKT[59]-work.L[67]*work.d[29]*1;
  residual += temp*temp;
  temp = work.KKT[61]-work.L[70]*work.d[30]*1;
  residual += temp*temp;
  temp = work.KKT[63]-work.L[72]*work.d[31]*1;
  residual += temp*temp;
  temp = work.KKT[65]-work.L[74]*work.d[32]*1;
  residual += temp*temp;
  temp = work.KKT[66]-work.L[0]*work.d[0]*work.L[0]-1*work.d[33]*1;
  residual += temp*temp;
  temp = work.KKT[70]-work.L[2]*work.d[1]*work.L[2]-1*work.d[35]*1-work.L[3]*work.d[34]*work.L[3];
  residual += temp*temp;
  temp = work.KKT[72]-work.L[4]*work.d[2]*work.L[4]-1*work.d[36]*1-work.L[5]*work.d[34]*work.L[5]-work.L[6]*work.d[35]*work.L[6];
  residual += temp*temp;
  temp = work.KKT[74]-work.L[7]*work.d[3]*work.L[7]-1*work.d[37]*1;
  residual += temp*temp;
  temp = work.KKT[78]-work.L[9]*work.d[4]*work.L[9]-1*work.d[39]*1-work.L[10]*work.d[38]*work.L[10];
  residual += temp*temp;
  temp = work.KKT[80]-work.L[11]*work.d[5]*work.L[11]-1*work.d[40]*1-work.L[12]*work.d[38]*work.L[12]-work.L[13]*work.d[39]*work.L[13];
  residual += temp*temp;
  temp = work.KKT[82]-work.L[14]*work.d[6]*work.L[14]-1*work.d[41]*1;
  residual += temp*temp;
  temp = work.KKT[86]-work.L[16]*work.d[7]*work.L[16]-1*work.d[43]*1-work.L[17]*work.d[42]*work.L[17];
  residual += temp*temp;
  temp = work.KKT[88]-work.L[18]*work.d[8]*work.L[18]-1*work.d[44]*1-work.L[19]*work.d[42]*work.L[19]-work.L[20]*work.d[43]*work.L[20];
  residual += temp*temp;
  temp = work.KKT[90]-work.L[21]*work.d[9]*work.L[21]-1*work.d[45]*1;
  residual += temp*temp;
  temp = work.KKT[94]-work.L[23]*work.d[10]*work.L[23]-1*work.d[47]*1-work.L[24]*work.d[46]*work.L[24];
  residual += temp*temp;
  temp = work.KKT[96]-work.L[25]*work.d[11]*work.L[25]-1*work.d[48]*1-work.L[26]*work.d[46]*work.L[26]-work.L[27]*work.d[47]*work.L[27];
  residual += temp*temp;
  temp = work.KKT[98]-work.L[28]*work.d[12]*work.L[28]-1*work.d[49]*1;
  residual += temp*temp;
  temp = work.KKT[102]-work.L[30]*work.d[13]*work.L[30]-1*work.d[51]*1-work.L[31]*work.d[50]*work.L[31];
  residual += temp*temp;
  temp = work.KKT[104]-work.L[32]*work.d[14]*work.L[32]-1*work.d[52]*1-work.L[33]*work.d[50]*work.L[33]-work.L[34]*work.d[51]*work.L[34];
  residual += temp*temp;
  temp = work.KKT[106]-work.L[35]*work.d[15]*work.L[35]-1*work.d[53]*1;
  residual += temp*temp;
  temp = work.KKT[110]-work.L[37]*work.d[16]*work.L[37]-1*work.d[55]*1-work.L[38]*work.d[54]*work.L[38];
  residual += temp*temp;
  temp = work.KKT[112]-work.L[39]*work.d[17]*work.L[39]-1*work.d[56]*1-work.L[40]*work.d[54]*work.L[40]-work.L[41]*work.d[55]*work.L[41];
  residual += temp*temp;
  temp = work.KKT[114]-work.L[42]*work.d[18]*work.L[42]-1*work.d[57]*1;
  residual += temp*temp;
  temp = work.KKT[118]-work.L[44]*work.d[19]*work.L[44]-1*work.d[59]*1-work.L[45]*work.d[58]*work.L[45];
  residual += temp*temp;
  temp = work.KKT[120]-work.L[46]*work.d[20]*work.L[46]-1*work.d[60]*1-work.L[47]*work.d[58]*work.L[47]-work.L[48]*work.d[59]*work.L[48];
  residual += temp*temp;
  temp = work.KKT[122]-work.L[49]*work.d[21]*work.L[49]-1*work.d[61]*1;
  residual += temp*temp;
  temp = work.KKT[126]-work.L[51]*work.d[22]*work.L[51]-1*work.d[63]*1-work.L[52]*work.d[62]*work.L[52];
  residual += temp*temp;
  temp = work.KKT[128]-work.L[53]*work.d[23]*work.L[53]-1*work.d[64]*1-work.L[54]*work.d[62]*work.L[54]-work.L[55]*work.d[63]*work.L[55];
  residual += temp*temp;
  temp = work.KKT[130]-work.L[56]*work.d[24]*work.L[56]-1*work.d[65]*1;
  residual += temp*temp;
  temp = work.KKT[134]-work.L[58]*work.d[25]*work.L[58]-1*work.d[67]*1-work.L[59]*work.d[66]*work.L[59];
  residual += temp*temp;
  temp = work.KKT[136]-work.L[60]*work.d[26]*work.L[60]-1*work.d[68]*1-work.L[61]*work.d[66]*work.L[61]-work.L[62]*work.d[67]*work.L[62];
  residual += temp*temp;
  temp = work.KKT[138]-work.L[63]*work.d[27]*work.L[63]-1*work.d[69]*1;
  residual += temp*temp;
  temp = work.KKT[142]-work.L[65]*work.d[28]*work.L[65]-1*work.d[71]*1-work.L[66]*work.d[70]*work.L[66];
  residual += temp*temp;
  temp = work.KKT[144]-work.L[67]*work.d[29]*work.L[67]-1*work.d[72]*1-work.L[68]*work.d[70]*work.L[68]-work.L[69]*work.d[71]*work.L[69];
  residual += temp*temp;
  temp = work.KKT[146]-work.L[70]*work.d[30]*work.L[70]-1*work.d[73]*1;
  residual += temp*temp;
  temp = work.KKT[150]-work.L[72]*work.d[31]*work.L[72]-1*work.d[75]*1-work.L[73]*work.d[74]*work.L[73];
  residual += temp*temp;
  temp = work.KKT[152]-work.L[74]*work.d[32]*work.L[74]-1*work.d[76]*1-work.L[75]*work.d[74]*work.L[75]-work.L[76]*work.d[75]*work.L[76];
  residual += temp*temp;
  temp = work.KKT[67]-1*work.d[33]*work.L[1];
  residual += temp*temp;
  temp = work.KKT[68]-work.L[3]*work.d[34]*1;
  residual += temp*temp;
  temp = work.KKT[71]-1*work.d[35]*work.L[77];
  residual += temp*temp;
  temp = work.KKT[69]-work.L[5]*work.d[34]*1;
  residual += temp*temp;
  temp = work.KKT[73]-1*work.d[36]*work.L[78]-work.L[6]*work.d[35]*work.L[77];
  residual += temp*temp;
  temp = work.KKT[75]-1*work.d[37]*work.L[8];
  residual += temp*temp;
  temp = work.KKT[76]-work.L[10]*work.d[38]*1;
  residual += temp*temp;
  temp = work.KKT[79]-1*work.d[39]*work.L[313];
  residual += temp*temp;
  temp = work.KKT[77]-work.L[12]*work.d[38]*1;
  residual += temp*temp;
  temp = work.KKT[81]-1*work.d[40]*work.L[314]-work.L[13]*work.d[39]*work.L[313];
  residual += temp*temp;
  temp = work.KKT[83]-1*work.d[41]*work.L[15];
  residual += temp*temp;
  temp = work.KKT[84]-work.L[17]*work.d[42]*1;
  residual += temp*temp;
  temp = work.KKT[87]-1*work.d[43]*work.L[284];
  residual += temp*temp;
  temp = work.KKT[85]-work.L[19]*work.d[42]*1;
  residual += temp*temp;
  temp = work.KKT[89]-1*work.d[44]*work.L[285]-work.L[20]*work.d[43]*work.L[284];
  residual += temp*temp;
  temp = work.KKT[91]-1*work.d[45]*work.L[22];
  residual += temp*temp;
  temp = work.KKT[92]-work.L[24]*work.d[46]*1;
  residual += temp*temp;
  temp = work.KKT[95]-1*work.d[47]*work.L[255];
  residual += temp*temp;
  temp = work.KKT[93]-work.L[26]*work.d[46]*1;
  residual += temp*temp;
  temp = work.KKT[97]-1*work.d[48]*work.L[256]-work.L[27]*work.d[47]*work.L[255];
  residual += temp*temp;
  temp = work.KKT[99]-1*work.d[49]*work.L[29];
  residual += temp*temp;
  temp = work.KKT[100]-work.L[31]*work.d[50]*1;
  residual += temp*temp;
  temp = work.KKT[103]-1*work.d[51]*work.L[226];
  residual += temp*temp;
  temp = work.KKT[101]-work.L[33]*work.d[50]*1;
  residual += temp*temp;
  temp = work.KKT[105]-1*work.d[52]*work.L[227]-work.L[34]*work.d[51]*work.L[226];
  residual += temp*temp;
  temp = work.KKT[107]-1*work.d[53]*work.L[36];
  residual += temp*temp;
  temp = work.KKT[108]-work.L[38]*work.d[54]*1;
  residual += temp*temp;
  temp = work.KKT[111]-1*work.d[55]*work.L[197];
  residual += temp*temp;
  temp = work.KKT[109]-work.L[40]*work.d[54]*1;
  residual += temp*temp;
  temp = work.KKT[113]-1*work.d[56]*work.L[198]-work.L[41]*work.d[55]*work.L[197];
  residual += temp*temp;
  temp = work.KKT[115]-1*work.d[57]*work.L[43];
  residual += temp*temp;
  temp = work.KKT[116]-work.L[45]*work.d[58]*1;
  residual += temp*temp;
  temp = work.KKT[119]-1*work.d[59]*work.L[168];
  residual += temp*temp;
  temp = work.KKT[117]-work.L[47]*work.d[58]*1;
  residual += temp*temp;
  temp = work.KKT[121]-1*work.d[60]*work.L[169]-work.L[48]*work.d[59]*work.L[168];
  residual += temp*temp;
  temp = work.KKT[123]-1*work.d[61]*work.L[50];
  residual += temp*temp;
  temp = work.KKT[124]-work.L[52]*work.d[62]*1;
  residual += temp*temp;
  temp = work.KKT[127]-1*work.d[63]*work.L[139];
  residual += temp*temp;
  temp = work.KKT[125]-work.L[54]*work.d[62]*1;
  residual += temp*temp;
  temp = work.KKT[129]-1*work.d[64]*work.L[140]-work.L[55]*work.d[63]*work.L[139];
  residual += temp*temp;
  temp = work.KKT[131]-1*work.d[65]*work.L[57];
  residual += temp*temp;
  temp = work.KKT[132]-work.L[59]*work.d[66]*1;
  residual += temp*temp;
  temp = work.KKT[135]-1*work.d[67]*work.L[110];
  residual += temp*temp;
  temp = work.KKT[133]-work.L[61]*work.d[66]*1;
  residual += temp*temp;
  temp = work.KKT[137]-1*work.d[68]*work.L[111]-work.L[62]*work.d[67]*work.L[110];
  residual += temp*temp;
  temp = work.KKT[139]-1*work.d[69]*work.L[64];
  residual += temp*temp;
  temp = work.KKT[140]-work.L[66]*work.d[70]*1;
  residual += temp*temp;
  temp = work.KKT[143]-1*work.d[71]*work.L[87];
  residual += temp*temp;
  temp = work.KKT[141]-work.L[68]*work.d[70]*1;
  residual += temp*temp;
  temp = work.KKT[145]-1*work.d[72]*work.L[88]-work.L[69]*work.d[71]*work.L[87];
  residual += temp*temp;
  temp = work.KKT[147]-1*work.d[73]*work.L[71];
  residual += temp*temp;
  temp = work.KKT[148]-work.L[73]*work.d[74]*1;
  residual += temp*temp;
  temp = work.KKT[151]-1*work.d[75]*work.L[83];
  residual += temp*temp;
  temp = work.KKT[149]-work.L[75]*work.d[74]*1;
  residual += temp*temp;
  temp = work.KKT[153]-1*work.d[76]*work.L[84]-work.L[76]*work.d[75]*work.L[83];
  residual += temp*temp;
  temp = work.KKT[154]-1*work.d[77]*work.L[79];
  residual += temp*temp;
  temp = work.KKT[156]-1*work.d[78]*work.L[80];
  residual += temp*temp;
  temp = work.KKT[158]-1*work.d[79]*work.L[81];
  residual += temp*temp;
  temp = work.KKT[155]-1*work.d[77]*work.L[315];
  residual += temp*temp;
  temp = work.KKT[157]-1*work.d[78]*work.L[317];
  residual += temp*temp;
  temp = work.KKT[159]-1*work.d[79]*work.L[320];
  residual += temp*temp;
  temp = work.KKT[300]-work.L[325]*work.d[143]*1;
  residual += temp*temp;
  temp = work.KKT[301]-work.L[330]*work.d[143]*1;
  residual += temp*temp;
  temp = work.KKT[302]-work.L[335]*work.d[143]*1;
  residual += temp*temp;
  temp = work.KKT[303]-work.L[326]*work.d[144]*1;
  residual += temp*temp;
  temp = work.KKT[306]-work.L[326]*work.d[144]*work.L[319]-work.L[327]*work.d[145]*1;
  residual += temp*temp;
  temp = work.KKT[309]-work.L[326]*work.d[144]*work.L[322]-work.L[327]*work.d[145]*work.L[323]-work.L[328]*work.d[146]*1;
  residual += temp*temp;
  temp = work.KKT[304]-work.L[331]*work.d[144]*1;
  residual += temp*temp;
  temp = work.KKT[307]-work.L[331]*work.d[144]*work.L[319]-work.L[332]*work.d[145]*1;
  residual += temp*temp;
  temp = work.KKT[310]-work.L[331]*work.d[144]*work.L[322]-work.L[332]*work.d[145]*work.L[323]-work.L[333]*work.d[146]*1;
  residual += temp*temp;
  temp = work.KKT[305]-work.L[336]*work.d[144]*1;
  residual += temp*temp;
  temp = work.KKT[308]-work.L[336]*work.d[144]*work.L[319]-work.L[337]*work.d[145]*1;
  residual += temp*temp;
  temp = work.KKT[311]-work.L[336]*work.d[144]*work.L[322]-work.L[337]*work.d[145]*work.L[323]-work.L[338]*work.d[146]*1;
  residual += temp*temp;
  temp = work.KKT[292]-work.L[324]*work.d[141]*1;
  residual += temp*temp;
  temp = work.KKT[296]-work.L[329]*work.d[142]*1;
  residual += temp*temp;
  temp = work.KKT[312]-work.L[339]*work.d[147]*1;
  residual += temp*temp;
  temp = work.KKT[285]-work.L[343]*work.d[136]*1;
  residual += temp*temp;
  temp = work.KKT[286]-work.L[354]*work.d[136]*1;
  residual += temp*temp;
  temp = work.KKT[287]-work.L[365]*work.d[136]*1;
  residual += temp*temp;
  temp = work.KKT[293]-work.L[347]*work.d[141]*1;
  residual += temp*temp;
  temp = work.KKT[297]-work.L[348]*work.d[142]*1;
  residual += temp*temp;
  temp = work.KKT[313]-work.L[349]*work.d[147]*1;
  residual += temp*temp;
  temp = work.KKT[294]-work.L[358]*work.d[141]*1;
  residual += temp*temp;
  temp = work.KKT[298]-work.L[359]*work.d[142]*1;
  residual += temp*temp;
  temp = work.KKT[314]-work.L[360]*work.d[147]*1;
  residual += temp*temp;
  temp = work.KKT[295]-work.L[370]*work.d[141]*1;
  residual += temp*temp;
  temp = work.KKT[299]-work.L[371]*work.d[142]*1;
  residual += temp*temp;
  temp = work.KKT[315]-work.L[372]*work.d[147]*1;
  residual += temp*temp;
  temp = work.KKT[280]-work.L[342]*work.d[134]*1;
  residual += temp*temp;
  temp = work.KKT[284]-work.L[353]*work.d[135]*1;
  residual += temp*temp;
  temp = work.KKT[291]-work.L[366]*work.d[137]*1;
  residual += temp*temp;
  temp = work.KKT[270]-work.L[287]*work.d[129]*1;
  residual += temp*temp;
  temp = work.KKT[271]-work.L[295]*work.d[129]*1;
  residual += temp*temp;
  temp = work.KKT[272]-work.L[303]*work.d[129]*1;
  residual += temp*temp;
  temp = work.KKT[277]-work.L[291]*work.d[134]*1;
  residual += temp*temp;
  temp = work.KKT[281]-work.L[292]*work.d[135]*1;
  residual += temp*temp;
  temp = work.KKT[288]-work.L[293]*work.d[137]*1;
  residual += temp*temp;
  temp = work.KKT[278]-work.L[299]*work.d[134]*1;
  residual += temp*temp;
  temp = work.KKT[282]-work.L[300]*work.d[135]*1;
  residual += temp*temp;
  temp = work.KKT[289]-work.L[301]*work.d[137]*1;
  residual += temp*temp;
  temp = work.KKT[279]-work.L[308]*work.d[134]*1;
  residual += temp*temp;
  temp = work.KKT[283]-work.L[309]*work.d[135]*1;
  residual += temp*temp;
  temp = work.KKT[290]-work.L[310]*work.d[137]*1;
  residual += temp*temp;
  temp = work.KKT[265]-work.L[286]*work.d[127]*1;
  residual += temp*temp;
  temp = work.KKT[269]-work.L[294]*work.d[128]*1;
  residual += temp*temp;
  temp = work.KKT[276]-work.L[304]*work.d[130]*1;
  residual += temp*temp;
  temp = work.KKT[255]-work.L[258]*work.d[122]*1;
  residual += temp*temp;
  temp = work.KKT[256]-work.L[266]*work.d[122]*1;
  residual += temp*temp;
  temp = work.KKT[257]-work.L[274]*work.d[122]*1;
  residual += temp*temp;
  temp = work.KKT[262]-work.L[262]*work.d[127]*1;
  residual += temp*temp;
  temp = work.KKT[266]-work.L[263]*work.d[128]*1;
  residual += temp*temp;
  temp = work.KKT[273]-work.L[264]*work.d[130]*1;
  residual += temp*temp;
  temp = work.KKT[263]-work.L[270]*work.d[127]*1;
  residual += temp*temp;
  temp = work.KKT[267]-work.L[271]*work.d[128]*1;
  residual += temp*temp;
  temp = work.KKT[274]-work.L[272]*work.d[130]*1;
  residual += temp*temp;
  temp = work.KKT[264]-work.L[279]*work.d[127]*1;
  residual += temp*temp;
  temp = work.KKT[268]-work.L[280]*work.d[128]*1;
  residual += temp*temp;
  temp = work.KKT[275]-work.L[281]*work.d[130]*1;
  residual += temp*temp;
  temp = work.KKT[250]-work.L[257]*work.d[120]*1;
  residual += temp*temp;
  temp = work.KKT[254]-work.L[265]*work.d[121]*1;
  residual += temp*temp;
  temp = work.KKT[261]-work.L[275]*work.d[123]*1;
  residual += temp*temp;
  temp = work.KKT[240]-work.L[229]*work.d[115]*1;
  residual += temp*temp;
  temp = work.KKT[241]-work.L[237]*work.d[115]*1;
  residual += temp*temp;
  temp = work.KKT[242]-work.L[245]*work.d[115]*1;
  residual += temp*temp;
  temp = work.KKT[247]-work.L[233]*work.d[120]*1;
  residual += temp*temp;
  temp = work.KKT[251]-work.L[234]*work.d[121]*1;
  residual += temp*temp;
  temp = work.KKT[258]-work.L[235]*work.d[123]*1;
  residual += temp*temp;
  temp = work.KKT[248]-work.L[241]*work.d[120]*1;
  residual += temp*temp;
  temp = work.KKT[252]-work.L[242]*work.d[121]*1;
  residual += temp*temp;
  temp = work.KKT[259]-work.L[243]*work.d[123]*1;
  residual += temp*temp;
  temp = work.KKT[249]-work.L[250]*work.d[120]*1;
  residual += temp*temp;
  temp = work.KKT[253]-work.L[251]*work.d[121]*1;
  residual += temp*temp;
  temp = work.KKT[260]-work.L[252]*work.d[123]*1;
  residual += temp*temp;
  temp = work.KKT[235]-work.L[228]*work.d[113]*1;
  residual += temp*temp;
  temp = work.KKT[239]-work.L[236]*work.d[114]*1;
  residual += temp*temp;
  temp = work.KKT[246]-work.L[246]*work.d[116]*1;
  residual += temp*temp;
  temp = work.KKT[225]-work.L[200]*work.d[108]*1;
  residual += temp*temp;
  temp = work.KKT[226]-work.L[208]*work.d[108]*1;
  residual += temp*temp;
  temp = work.KKT[227]-work.L[216]*work.d[108]*1;
  residual += temp*temp;
  temp = work.KKT[232]-work.L[204]*work.d[113]*1;
  residual += temp*temp;
  temp = work.KKT[236]-work.L[205]*work.d[114]*1;
  residual += temp*temp;
  temp = work.KKT[243]-work.L[206]*work.d[116]*1;
  residual += temp*temp;
  temp = work.KKT[233]-work.L[212]*work.d[113]*1;
  residual += temp*temp;
  temp = work.KKT[237]-work.L[213]*work.d[114]*1;
  residual += temp*temp;
  temp = work.KKT[244]-work.L[214]*work.d[116]*1;
  residual += temp*temp;
  temp = work.KKT[234]-work.L[221]*work.d[113]*1;
  residual += temp*temp;
  temp = work.KKT[238]-work.L[222]*work.d[114]*1;
  residual += temp*temp;
  temp = work.KKT[245]-work.L[223]*work.d[116]*1;
  residual += temp*temp;
  temp = work.KKT[220]-work.L[199]*work.d[106]*1;
  residual += temp*temp;
  temp = work.KKT[224]-work.L[207]*work.d[107]*1;
  residual += temp*temp;
  temp = work.KKT[231]-work.L[217]*work.d[109]*1;
  residual += temp*temp;
  temp = work.KKT[210]-work.L[171]*work.d[101]*1;
  residual += temp*temp;
  temp = work.KKT[211]-work.L[179]*work.d[101]*1;
  residual += temp*temp;
  temp = work.KKT[212]-work.L[187]*work.d[101]*1;
  residual += temp*temp;
  temp = work.KKT[217]-work.L[175]*work.d[106]*1;
  residual += temp*temp;
  temp = work.KKT[221]-work.L[176]*work.d[107]*1;
  residual += temp*temp;
  temp = work.KKT[228]-work.L[177]*work.d[109]*1;
  residual += temp*temp;
  temp = work.KKT[218]-work.L[183]*work.d[106]*1;
  residual += temp*temp;
  temp = work.KKT[222]-work.L[184]*work.d[107]*1;
  residual += temp*temp;
  temp = work.KKT[229]-work.L[185]*work.d[109]*1;
  residual += temp*temp;
  temp = work.KKT[219]-work.L[192]*work.d[106]*1;
  residual += temp*temp;
  temp = work.KKT[223]-work.L[193]*work.d[107]*1;
  residual += temp*temp;
  temp = work.KKT[230]-work.L[194]*work.d[109]*1;
  residual += temp*temp;
  temp = work.KKT[205]-work.L[170]*work.d[99]*1;
  residual += temp*temp;
  temp = work.KKT[209]-work.L[178]*work.d[100]*1;
  residual += temp*temp;
  temp = work.KKT[216]-work.L[188]*work.d[102]*1;
  residual += temp*temp;
  temp = work.KKT[195]-work.L[142]*work.d[94]*1;
  residual += temp*temp;
  temp = work.KKT[196]-work.L[150]*work.d[94]*1;
  residual += temp*temp;
  temp = work.KKT[197]-work.L[158]*work.d[94]*1;
  residual += temp*temp;
  temp = work.KKT[202]-work.L[146]*work.d[99]*1;
  residual += temp*temp;
  temp = work.KKT[206]-work.L[147]*work.d[100]*1;
  residual += temp*temp;
  temp = work.KKT[213]-work.L[148]*work.d[102]*1;
  residual += temp*temp;
  temp = work.KKT[203]-work.L[154]*work.d[99]*1;
  residual += temp*temp;
  temp = work.KKT[207]-work.L[155]*work.d[100]*1;
  residual += temp*temp;
  temp = work.KKT[214]-work.L[156]*work.d[102]*1;
  residual += temp*temp;
  temp = work.KKT[204]-work.L[163]*work.d[99]*1;
  residual += temp*temp;
  temp = work.KKT[208]-work.L[164]*work.d[100]*1;
  residual += temp*temp;
  temp = work.KKT[215]-work.L[165]*work.d[102]*1;
  residual += temp*temp;
  temp = work.KKT[190]-work.L[141]*work.d[92]*1;
  residual += temp*temp;
  temp = work.KKT[194]-work.L[149]*work.d[93]*1;
  residual += temp*temp;
  temp = work.KKT[201]-work.L[159]*work.d[95]*1;
  residual += temp*temp;
  temp = work.KKT[180]-work.L[113]*work.d[87]*1;
  residual += temp*temp;
  temp = work.KKT[181]-work.L[121]*work.d[87]*1;
  residual += temp*temp;
  temp = work.KKT[182]-work.L[129]*work.d[87]*1;
  residual += temp*temp;
  temp = work.KKT[187]-work.L[117]*work.d[92]*1;
  residual += temp*temp;
  temp = work.KKT[191]-work.L[118]*work.d[93]*1;
  residual += temp*temp;
  temp = work.KKT[198]-work.L[119]*work.d[95]*1;
  residual += temp*temp;
  temp = work.KKT[188]-work.L[125]*work.d[92]*1;
  residual += temp*temp;
  temp = work.KKT[192]-work.L[126]*work.d[93]*1;
  residual += temp*temp;
  temp = work.KKT[199]-work.L[127]*work.d[95]*1;
  residual += temp*temp;
  temp = work.KKT[189]-work.L[134]*work.d[92]*1;
  residual += temp*temp;
  temp = work.KKT[193]-work.L[135]*work.d[93]*1;
  residual += temp*temp;
  temp = work.KKT[200]-work.L[136]*work.d[95]*1;
  residual += temp*temp;
  temp = work.KKT[175]-work.L[112]*work.d[85]*1;
  residual += temp*temp;
  temp = work.KKT[179]-work.L[120]*work.d[86]*1;
  residual += temp*temp;
  temp = work.KKT[186]-work.L[130]*work.d[88]*1;
  residual += temp*temp;
  temp = work.KKT[167]-work.L[91]*work.d[83]*1;
  residual += temp*temp;
  temp = work.KKT[168]-work.L[97]*work.d[83]*1;
  residual += temp*temp;
  temp = work.KKT[169]-work.L[103]*work.d[83]*1;
  residual += temp*temp;
  temp = work.KKT[172]-work.L[93]*work.d[85]*1;
  residual += temp*temp;
  temp = work.KKT[176]-work.L[94]*work.d[86]*1;
  residual += temp*temp;
  temp = work.KKT[183]-work.L[95]*work.d[88]*1;
  residual += temp*temp;
  temp = work.KKT[173]-work.L[99]*work.d[85]*1;
  residual += temp*temp;
  temp = work.KKT[177]-work.L[100]*work.d[86]*1;
  residual += temp*temp;
  temp = work.KKT[184]-work.L[101]*work.d[88]*1;
  residual += temp*temp;
  temp = work.KKT[174]-work.L[105]*work.d[85]*1;
  residual += temp*temp;
  temp = work.KKT[178]-work.L[106]*work.d[86]*1;
  residual += temp*temp;
  temp = work.KKT[185]-work.L[107]*work.d[88]*1;
  residual += temp*temp;
  temp = work.KKT[163]-work.L[89]*work.d[81]*1;
  residual += temp*temp;
  temp = work.KKT[166]-work.L[96]*work.d[82]*1;
  residual += temp*temp;
  temp = work.KKT[171]-work.L[104]*work.d[84]*1;
  residual += temp*temp;
  return residual;
}
void matrix_multiply(double *result, double *source) {
  /* Finds result = A*source. */
  result[0] = work.KKT[67]*source[88]+work.KKT[68]*source[89]+work.KKT[69]*source[90];
  result[1] = work.KKT[75]*source[91]+work.KKT[76]*source[92]+work.KKT[77]*source[93];
  result[2] = work.KKT[83]*source[94]+work.KKT[84]*source[95]+work.KKT[85]*source[96];
  result[3] = work.KKT[91]*source[97]+work.KKT[92]*source[98]+work.KKT[93]*source[99];
  result[4] = work.KKT[99]*source[100]+work.KKT[100]*source[101]+work.KKT[101]*source[102];
  result[5] = work.KKT[107]*source[103]+work.KKT[108]*source[104]+work.KKT[109]*source[105];
  result[6] = work.KKT[115]*source[106]+work.KKT[116]*source[107]+work.KKT[117]*source[108];
  result[7] = work.KKT[123]*source[109]+work.KKT[124]*source[110]+work.KKT[125]*source[111];
  result[8] = work.KKT[131]*source[112]+work.KKT[132]*source[113]+work.KKT[133]*source[114];
  result[9] = work.KKT[139]*source[115]+work.KKT[140]*source[116]+work.KKT[141]*source[117];
  result[10] = work.KKT[147]*source[118]+work.KKT[148]*source[119]+work.KKT[149]*source[120];
  result[11] = work.KKT[71]*source[89]+work.KKT[73]*source[90]+work.KKT[154]*source[121]+work.KKT[156]*source[122]+work.KKT[158]*source[123];
  result[12] = work.KKT[79]*source[92]+work.KKT[81]*source[93]+work.KKT[300]*source[124]+work.KKT[301]*source[125]+work.KKT[302]*source[126];
  result[13] = work.KKT[87]*source[95]+work.KKT[89]*source[96]+work.KKT[285]*source[127]+work.KKT[286]*source[128]+work.KKT[287]*source[129];
  result[14] = work.KKT[95]*source[98]+work.KKT[97]*source[99]+work.KKT[270]*source[130]+work.KKT[271]*source[131]+work.KKT[272]*source[132];
  result[15] = work.KKT[103]*source[101]+work.KKT[105]*source[102]+work.KKT[255]*source[133]+work.KKT[256]*source[134]+work.KKT[257]*source[135];
  result[16] = work.KKT[111]*source[104]+work.KKT[113]*source[105]+work.KKT[240]*source[136]+work.KKT[241]*source[137]+work.KKT[242]*source[138];
  result[17] = work.KKT[119]*source[107]+work.KKT[121]*source[108]+work.KKT[225]*source[139]+work.KKT[226]*source[140]+work.KKT[227]*source[141];
  result[18] = work.KKT[127]*source[110]+work.KKT[129]*source[111]+work.KKT[210]*source[142]+work.KKT[211]*source[143]+work.KKT[212]*source[144];
  result[19] = work.KKT[135]*source[113]+work.KKT[137]*source[114]+work.KKT[195]*source[145]+work.KKT[196]*source[146]+work.KKT[197]*source[147];
  result[20] = work.KKT[143]*source[116]+work.KKT[145]*source[117]+work.KKT[180]*source[148]+work.KKT[181]*source[149]+work.KKT[182]*source[150];
  result[21] = work.KKT[151]*source[119]+work.KKT[153]*source[120]+work.KKT[167]*source[151]+work.KKT[168]*source[152]+work.KKT[169]*source[153];
  result[22] = work.KKT[155]*source[121]+work.KKT[303]*source[124]+work.KKT[304]*source[125]+work.KKT[305]*source[126];
  result[23] = work.KKT[157]*source[122]+work.KKT[306]*source[124]+work.KKT[307]*source[125]+work.KKT[308]*source[126];
  result[24] = work.KKT[159]*source[123]+work.KKT[309]*source[124]+work.KKT[310]*source[125]+work.KKT[311]*source[126];
  result[25] = work.KKT[292]*source[124]+work.KKT[293]*source[127]+work.KKT[294]*source[128]+work.KKT[295]*source[129];
  result[26] = work.KKT[296]*source[125]+work.KKT[297]*source[127]+work.KKT[298]*source[128]+work.KKT[299]*source[129];
  result[27] = work.KKT[312]*source[126]+work.KKT[313]*source[127]+work.KKT[314]*source[128]+work.KKT[315]*source[129];
  result[28] = work.KKT[280]*source[127]+work.KKT[277]*source[130]+work.KKT[278]*source[131]+work.KKT[279]*source[132];
  result[29] = work.KKT[284]*source[128]+work.KKT[281]*source[130]+work.KKT[282]*source[131]+work.KKT[283]*source[132];
  result[30] = work.KKT[291]*source[129]+work.KKT[288]*source[130]+work.KKT[289]*source[131]+work.KKT[290]*source[132];
  result[31] = work.KKT[265]*source[130]+work.KKT[262]*source[133]+work.KKT[263]*source[134]+work.KKT[264]*source[135];
  result[32] = work.KKT[269]*source[131]+work.KKT[266]*source[133]+work.KKT[267]*source[134]+work.KKT[268]*source[135];
  result[33] = work.KKT[276]*source[132]+work.KKT[273]*source[133]+work.KKT[274]*source[134]+work.KKT[275]*source[135];
  result[34] = work.KKT[250]*source[133]+work.KKT[247]*source[136]+work.KKT[248]*source[137]+work.KKT[249]*source[138];
  result[35] = work.KKT[254]*source[134]+work.KKT[251]*source[136]+work.KKT[252]*source[137]+work.KKT[253]*source[138];
  result[36] = work.KKT[261]*source[135]+work.KKT[258]*source[136]+work.KKT[259]*source[137]+work.KKT[260]*source[138];
  result[37] = work.KKT[235]*source[136]+work.KKT[232]*source[139]+work.KKT[233]*source[140]+work.KKT[234]*source[141];
  result[38] = work.KKT[239]*source[137]+work.KKT[236]*source[139]+work.KKT[237]*source[140]+work.KKT[238]*source[141];
  result[39] = work.KKT[246]*source[138]+work.KKT[243]*source[139]+work.KKT[244]*source[140]+work.KKT[245]*source[141];
  result[40] = work.KKT[220]*source[139]+work.KKT[217]*source[142]+work.KKT[218]*source[143]+work.KKT[219]*source[144];
  result[41] = work.KKT[224]*source[140]+work.KKT[221]*source[142]+work.KKT[222]*source[143]+work.KKT[223]*source[144];
  result[42] = work.KKT[231]*source[141]+work.KKT[228]*source[142]+work.KKT[229]*source[143]+work.KKT[230]*source[144];
  result[43] = work.KKT[205]*source[142]+work.KKT[202]*source[145]+work.KKT[203]*source[146]+work.KKT[204]*source[147];
  result[44] = work.KKT[209]*source[143]+work.KKT[206]*source[145]+work.KKT[207]*source[146]+work.KKT[208]*source[147];
  result[45] = work.KKT[216]*source[144]+work.KKT[213]*source[145]+work.KKT[214]*source[146]+work.KKT[215]*source[147];
  result[46] = work.KKT[190]*source[145]+work.KKT[187]*source[148]+work.KKT[188]*source[149]+work.KKT[189]*source[150];
  result[47] = work.KKT[194]*source[146]+work.KKT[191]*source[148]+work.KKT[192]*source[149]+work.KKT[193]*source[150];
  result[48] = work.KKT[201]*source[147]+work.KKT[198]*source[148]+work.KKT[199]*source[149]+work.KKT[200]*source[150];
  result[49] = work.KKT[175]*source[148]+work.KKT[172]*source[151]+work.KKT[173]*source[152]+work.KKT[174]*source[153];
  result[50] = work.KKT[179]*source[149]+work.KKT[176]*source[151]+work.KKT[177]*source[152]+work.KKT[178]*source[153];
  result[51] = work.KKT[186]*source[150]+work.KKT[183]*source[151]+work.KKT[184]*source[152]+work.KKT[185]*source[153];
  result[52] = work.KKT[160]*source[52]+work.KKT[161]*source[53]+work.KKT[162]*source[54]+work.KKT[163]*source[151];
  result[53] = work.KKT[161]*source[52]+work.KKT[164]*source[53]+work.KKT[165]*source[54]+work.KKT[166]*source[152];
  result[54] = work.KKT[162]*source[52]+work.KKT[165]*source[53]+work.KKT[170]*source[54]+work.KKT[171]*source[153];
  result[55] = work.KKT[0]*source[55]+work.KKT[1]*source[88];
  result[56] = work.KKT[2]*source[56]+work.KKT[3]*source[89];
  result[57] = work.KKT[4]*source[57]+work.KKT[5]*source[90];
  result[58] = work.KKT[6]*source[58]+work.KKT[7]*source[91];
  result[59] = work.KKT[8]*source[59]+work.KKT[9]*source[92];
  result[60] = work.KKT[10]*source[60]+work.KKT[11]*source[93];
  result[61] = work.KKT[12]*source[61]+work.KKT[13]*source[94];
  result[62] = work.KKT[14]*source[62]+work.KKT[15]*source[95];
  result[63] = work.KKT[16]*source[63]+work.KKT[17]*source[96];
  result[64] = work.KKT[18]*source[64]+work.KKT[19]*source[97];
  result[65] = work.KKT[20]*source[65]+work.KKT[21]*source[98];
  result[66] = work.KKT[22]*source[66]+work.KKT[23]*source[99];
  result[67] = work.KKT[24]*source[67]+work.KKT[25]*source[100];
  result[68] = work.KKT[26]*source[68]+work.KKT[27]*source[101];
  result[69] = work.KKT[28]*source[69]+work.KKT[29]*source[102];
  result[70] = work.KKT[30]*source[70]+work.KKT[31]*source[103];
  result[71] = work.KKT[32]*source[71]+work.KKT[33]*source[104];
  result[72] = work.KKT[34]*source[72]+work.KKT[35]*source[105];
  result[73] = work.KKT[36]*source[73]+work.KKT[37]*source[106];
  result[74] = work.KKT[38]*source[74]+work.KKT[39]*source[107];
  result[75] = work.KKT[40]*source[75]+work.KKT[41]*source[108];
  result[76] = work.KKT[42]*source[76]+work.KKT[43]*source[109];
  result[77] = work.KKT[44]*source[77]+work.KKT[45]*source[110];
  result[78] = work.KKT[46]*source[78]+work.KKT[47]*source[111];
  result[79] = work.KKT[48]*source[79]+work.KKT[49]*source[112];
  result[80] = work.KKT[50]*source[80]+work.KKT[51]*source[113];
  result[81] = work.KKT[52]*source[81]+work.KKT[53]*source[114];
  result[82] = work.KKT[54]*source[82]+work.KKT[55]*source[115];
  result[83] = work.KKT[56]*source[83]+work.KKT[57]*source[116];
  result[84] = work.KKT[58]*source[84]+work.KKT[59]*source[117];
  result[85] = work.KKT[60]*source[85]+work.KKT[61]*source[118];
  result[86] = work.KKT[62]*source[86]+work.KKT[63]*source[119];
  result[87] = work.KKT[64]*source[87]+work.KKT[65]*source[120];
  result[88] = work.KKT[1]*source[55]+work.KKT[66]*source[88]+work.KKT[67]*source[0];
  result[89] = work.KKT[3]*source[56]+work.KKT[70]*source[89]+work.KKT[68]*source[0]+work.KKT[71]*source[11];
  result[90] = work.KKT[5]*source[57]+work.KKT[72]*source[90]+work.KKT[69]*source[0]+work.KKT[73]*source[11];
  result[91] = work.KKT[7]*source[58]+work.KKT[74]*source[91]+work.KKT[75]*source[1];
  result[92] = work.KKT[9]*source[59]+work.KKT[78]*source[92]+work.KKT[76]*source[1]+work.KKT[79]*source[12];
  result[93] = work.KKT[11]*source[60]+work.KKT[80]*source[93]+work.KKT[77]*source[1]+work.KKT[81]*source[12];
  result[94] = work.KKT[13]*source[61]+work.KKT[82]*source[94]+work.KKT[83]*source[2];
  result[95] = work.KKT[15]*source[62]+work.KKT[86]*source[95]+work.KKT[84]*source[2]+work.KKT[87]*source[13];
  result[96] = work.KKT[17]*source[63]+work.KKT[88]*source[96]+work.KKT[85]*source[2]+work.KKT[89]*source[13];
  result[97] = work.KKT[19]*source[64]+work.KKT[90]*source[97]+work.KKT[91]*source[3];
  result[98] = work.KKT[21]*source[65]+work.KKT[94]*source[98]+work.KKT[92]*source[3]+work.KKT[95]*source[14];
  result[99] = work.KKT[23]*source[66]+work.KKT[96]*source[99]+work.KKT[93]*source[3]+work.KKT[97]*source[14];
  result[100] = work.KKT[25]*source[67]+work.KKT[98]*source[100]+work.KKT[99]*source[4];
  result[101] = work.KKT[27]*source[68]+work.KKT[102]*source[101]+work.KKT[100]*source[4]+work.KKT[103]*source[15];
  result[102] = work.KKT[29]*source[69]+work.KKT[104]*source[102]+work.KKT[101]*source[4]+work.KKT[105]*source[15];
  result[103] = work.KKT[31]*source[70]+work.KKT[106]*source[103]+work.KKT[107]*source[5];
  result[104] = work.KKT[33]*source[71]+work.KKT[110]*source[104]+work.KKT[108]*source[5]+work.KKT[111]*source[16];
  result[105] = work.KKT[35]*source[72]+work.KKT[112]*source[105]+work.KKT[109]*source[5]+work.KKT[113]*source[16];
  result[106] = work.KKT[37]*source[73]+work.KKT[114]*source[106]+work.KKT[115]*source[6];
  result[107] = work.KKT[39]*source[74]+work.KKT[118]*source[107]+work.KKT[116]*source[6]+work.KKT[119]*source[17];
  result[108] = work.KKT[41]*source[75]+work.KKT[120]*source[108]+work.KKT[117]*source[6]+work.KKT[121]*source[17];
  result[109] = work.KKT[43]*source[76]+work.KKT[122]*source[109]+work.KKT[123]*source[7];
  result[110] = work.KKT[45]*source[77]+work.KKT[126]*source[110]+work.KKT[124]*source[7]+work.KKT[127]*source[18];
  result[111] = work.KKT[47]*source[78]+work.KKT[128]*source[111]+work.KKT[125]*source[7]+work.KKT[129]*source[18];
  result[112] = work.KKT[49]*source[79]+work.KKT[130]*source[112]+work.KKT[131]*source[8];
  result[113] = work.KKT[51]*source[80]+work.KKT[134]*source[113]+work.KKT[132]*source[8]+work.KKT[135]*source[19];
  result[114] = work.KKT[53]*source[81]+work.KKT[136]*source[114]+work.KKT[133]*source[8]+work.KKT[137]*source[19];
  result[115] = work.KKT[55]*source[82]+work.KKT[138]*source[115]+work.KKT[139]*source[9];
  result[116] = work.KKT[57]*source[83]+work.KKT[142]*source[116]+work.KKT[140]*source[9]+work.KKT[143]*source[20];
  result[117] = work.KKT[59]*source[84]+work.KKT[144]*source[117]+work.KKT[141]*source[9]+work.KKT[145]*source[20];
  result[118] = work.KKT[61]*source[85]+work.KKT[146]*source[118]+work.KKT[147]*source[10];
  result[119] = work.KKT[63]*source[86]+work.KKT[150]*source[119]+work.KKT[148]*source[10]+work.KKT[151]*source[21];
  result[120] = work.KKT[65]*source[87]+work.KKT[152]*source[120]+work.KKT[149]*source[10]+work.KKT[153]*source[21];
  result[121] = work.KKT[154]*source[11]+work.KKT[155]*source[22];
  result[122] = work.KKT[156]*source[11]+work.KKT[157]*source[23];
  result[123] = work.KKT[158]*source[11]+work.KKT[159]*source[24];
  result[124] = work.KKT[300]*source[12]+work.KKT[303]*source[22]+work.KKT[306]*source[23]+work.KKT[309]*source[24]+work.KKT[292]*source[25];
  result[125] = work.KKT[301]*source[12]+work.KKT[304]*source[22]+work.KKT[307]*source[23]+work.KKT[310]*source[24]+work.KKT[296]*source[26];
  result[126] = work.KKT[302]*source[12]+work.KKT[305]*source[22]+work.KKT[308]*source[23]+work.KKT[311]*source[24]+work.KKT[312]*source[27];
  result[127] = work.KKT[285]*source[13]+work.KKT[293]*source[25]+work.KKT[297]*source[26]+work.KKT[313]*source[27]+work.KKT[280]*source[28];
  result[128] = work.KKT[286]*source[13]+work.KKT[294]*source[25]+work.KKT[298]*source[26]+work.KKT[314]*source[27]+work.KKT[284]*source[29];
  result[129] = work.KKT[287]*source[13]+work.KKT[295]*source[25]+work.KKT[299]*source[26]+work.KKT[315]*source[27]+work.KKT[291]*source[30];
  result[130] = work.KKT[270]*source[14]+work.KKT[277]*source[28]+work.KKT[281]*source[29]+work.KKT[288]*source[30]+work.KKT[265]*source[31];
  result[131] = work.KKT[271]*source[14]+work.KKT[278]*source[28]+work.KKT[282]*source[29]+work.KKT[289]*source[30]+work.KKT[269]*source[32];
  result[132] = work.KKT[272]*source[14]+work.KKT[279]*source[28]+work.KKT[283]*source[29]+work.KKT[290]*source[30]+work.KKT[276]*source[33];
  result[133] = work.KKT[255]*source[15]+work.KKT[262]*source[31]+work.KKT[266]*source[32]+work.KKT[273]*source[33]+work.KKT[250]*source[34];
  result[134] = work.KKT[256]*source[15]+work.KKT[263]*source[31]+work.KKT[267]*source[32]+work.KKT[274]*source[33]+work.KKT[254]*source[35];
  result[135] = work.KKT[257]*source[15]+work.KKT[264]*source[31]+work.KKT[268]*source[32]+work.KKT[275]*source[33]+work.KKT[261]*source[36];
  result[136] = work.KKT[240]*source[16]+work.KKT[247]*source[34]+work.KKT[251]*source[35]+work.KKT[258]*source[36]+work.KKT[235]*source[37];
  result[137] = work.KKT[241]*source[16]+work.KKT[248]*source[34]+work.KKT[252]*source[35]+work.KKT[259]*source[36]+work.KKT[239]*source[38];
  result[138] = work.KKT[242]*source[16]+work.KKT[249]*source[34]+work.KKT[253]*source[35]+work.KKT[260]*source[36]+work.KKT[246]*source[39];
  result[139] = work.KKT[225]*source[17]+work.KKT[232]*source[37]+work.KKT[236]*source[38]+work.KKT[243]*source[39]+work.KKT[220]*source[40];
  result[140] = work.KKT[226]*source[17]+work.KKT[233]*source[37]+work.KKT[237]*source[38]+work.KKT[244]*source[39]+work.KKT[224]*source[41];
  result[141] = work.KKT[227]*source[17]+work.KKT[234]*source[37]+work.KKT[238]*source[38]+work.KKT[245]*source[39]+work.KKT[231]*source[42];
  result[142] = work.KKT[210]*source[18]+work.KKT[217]*source[40]+work.KKT[221]*source[41]+work.KKT[228]*source[42]+work.KKT[205]*source[43];
  result[143] = work.KKT[211]*source[18]+work.KKT[218]*source[40]+work.KKT[222]*source[41]+work.KKT[229]*source[42]+work.KKT[209]*source[44];
  result[144] = work.KKT[212]*source[18]+work.KKT[219]*source[40]+work.KKT[223]*source[41]+work.KKT[230]*source[42]+work.KKT[216]*source[45];
  result[145] = work.KKT[195]*source[19]+work.KKT[202]*source[43]+work.KKT[206]*source[44]+work.KKT[213]*source[45]+work.KKT[190]*source[46];
  result[146] = work.KKT[196]*source[19]+work.KKT[203]*source[43]+work.KKT[207]*source[44]+work.KKT[214]*source[45]+work.KKT[194]*source[47];
  result[147] = work.KKT[197]*source[19]+work.KKT[204]*source[43]+work.KKT[208]*source[44]+work.KKT[215]*source[45]+work.KKT[201]*source[48];
  result[148] = work.KKT[180]*source[20]+work.KKT[187]*source[46]+work.KKT[191]*source[47]+work.KKT[198]*source[48]+work.KKT[175]*source[49];
  result[149] = work.KKT[181]*source[20]+work.KKT[188]*source[46]+work.KKT[192]*source[47]+work.KKT[199]*source[48]+work.KKT[179]*source[50];
  result[150] = work.KKT[182]*source[20]+work.KKT[189]*source[46]+work.KKT[193]*source[47]+work.KKT[200]*source[48]+work.KKT[186]*source[51];
  result[151] = work.KKT[167]*source[21]+work.KKT[172]*source[49]+work.KKT[176]*source[50]+work.KKT[183]*source[51]+work.KKT[163]*source[52];
  result[152] = work.KKT[168]*source[21]+work.KKT[173]*source[49]+work.KKT[177]*source[50]+work.KKT[184]*source[51]+work.KKT[166]*source[53];
  result[153] = work.KKT[169]*source[21]+work.KKT[174]*source[49]+work.KKT[178]*source[50]+work.KKT[185]*source[51]+work.KKT[171]*source[54];
}
double check_residual(double *target, double *multiplicand) {
  /* Returns the squared 2-norm of lhs - A*rhs. */
  /* Reuses v to find the residual. */
  int i;
  double residual;
  residual = 0;
  matrix_multiply(work.v, multiplicand);
  for (i = 0; i < 55; i++) {
    residual += (target[i] - work.v[i])*(target[i] - work.v[i]);
  }
  return residual;
}
void fill_KKT(void) {
  work.KKT[160] = 2*params.Q_final[0];
  work.KKT[161] = 2*params.Q_final[3];
  work.KKT[162] = 2*params.Q_final[6];
  work.KKT[164] = 2*params.Q_final[4];
  work.KKT[165] = 2*params.Q_final[7];
  work.KKT[170] = 2*params.Q_final[8];
  work.KKT[0] = work.s_inv_z[0];
  work.KKT[2] = work.s_inv_z[1];
  work.KKT[4] = work.s_inv_z[2];
  work.KKT[6] = work.s_inv_z[3];
  work.KKT[8] = work.s_inv_z[4];
  work.KKT[10] = work.s_inv_z[5];
  work.KKT[12] = work.s_inv_z[6];
  work.KKT[14] = work.s_inv_z[7];
  work.KKT[16] = work.s_inv_z[8];
  work.KKT[18] = work.s_inv_z[9];
  work.KKT[20] = work.s_inv_z[10];
  work.KKT[22] = work.s_inv_z[11];
  work.KKT[24] = work.s_inv_z[12];
  work.KKT[26] = work.s_inv_z[13];
  work.KKT[28] = work.s_inv_z[14];
  work.KKT[30] = work.s_inv_z[15];
  work.KKT[32] = work.s_inv_z[16];
  work.KKT[34] = work.s_inv_z[17];
  work.KKT[36] = work.s_inv_z[18];
  work.KKT[38] = work.s_inv_z[19];
  work.KKT[40] = work.s_inv_z[20];
  work.KKT[42] = work.s_inv_z[21];
  work.KKT[44] = work.s_inv_z[22];
  work.KKT[46] = work.s_inv_z[23];
  work.KKT[48] = work.s_inv_z[24];
  work.KKT[50] = work.s_inv_z[25];
  work.KKT[52] = work.s_inv_z[26];
  work.KKT[54] = work.s_inv_z[27];
  work.KKT[56] = work.s_inv_z[28];
  work.KKT[58] = work.s_inv_z[29];
  work.KKT[60] = work.s_inv_z[30];
  work.KKT[62] = work.s_inv_z[31];
  work.KKT[64] = work.s_inv_z[32];
  work.KKT[1] = 1;
  work.KKT[3] = 1;
  work.KKT[5] = 1;
  work.KKT[7] = 1;
  work.KKT[9] = 1;
  work.KKT[11] = 1;
  work.KKT[13] = 1;
  work.KKT[15] = 1;
  work.KKT[17] = 1;
  work.KKT[19] = 1;
  work.KKT[21] = 1;
  work.KKT[23] = 1;
  work.KKT[25] = 1;
  work.KKT[27] = 1;
  work.KKT[29] = 1;
  work.KKT[31] = 1;
  work.KKT[33] = 1;
  work.KKT[35] = 1;
  work.KKT[37] = 1;
  work.KKT[39] = 1;
  work.KKT[41] = 1;
  work.KKT[43] = 1;
  work.KKT[45] = 1;
  work.KKT[47] = 1;
  work.KKT[49] = 1;
  work.KKT[51] = 1;
  work.KKT[53] = 1;
  work.KKT[55] = 1;
  work.KKT[57] = 1;
  work.KKT[59] = 1;
  work.KKT[61] = 1;
  work.KKT[63] = 1;
  work.KKT[65] = 1;
  work.KKT[66] = work.block_33[0];
  work.KKT[70] = work.block_33[0];
  work.KKT[72] = work.block_33[0];
  work.KKT[74] = work.block_33[0];
  work.KKT[78] = work.block_33[0];
  work.KKT[80] = work.block_33[0];
  work.KKT[82] = work.block_33[0];
  work.KKT[86] = work.block_33[0];
  work.KKT[88] = work.block_33[0];
  work.KKT[90] = work.block_33[0];
  work.KKT[94] = work.block_33[0];
  work.KKT[96] = work.block_33[0];
  work.KKT[98] = work.block_33[0];
  work.KKT[102] = work.block_33[0];
  work.KKT[104] = work.block_33[0];
  work.KKT[106] = work.block_33[0];
  work.KKT[110] = work.block_33[0];
  work.KKT[112] = work.block_33[0];
  work.KKT[114] = work.block_33[0];
  work.KKT[118] = work.block_33[0];
  work.KKT[120] = work.block_33[0];
  work.KKT[122] = work.block_33[0];
  work.KKT[126] = work.block_33[0];
  work.KKT[128] = work.block_33[0];
  work.KKT[130] = work.block_33[0];
  work.KKT[134] = work.block_33[0];
  work.KKT[136] = work.block_33[0];
  work.KKT[138] = work.block_33[0];
  work.KKT[142] = work.block_33[0];
  work.KKT[144] = work.block_33[0];
  work.KKT[146] = work.block_33[0];
  work.KKT[150] = work.block_33[0];
  work.KKT[152] = work.block_33[0];
  work.KKT[67] = 1;
  work.KKT[68] = -1;
  work.KKT[71] = 1;
  work.KKT[69] = -1;
  work.KKT[73] = -1;
  work.KKT[75] = 1;
  work.KKT[76] = -1;
  work.KKT[79] = 1;
  work.KKT[77] = -1;
  work.KKT[81] = -1;
  work.KKT[83] = 1;
  work.KKT[84] = -1;
  work.KKT[87] = 1;
  work.KKT[85] = -1;
  work.KKT[89] = -1;
  work.KKT[91] = 1;
  work.KKT[92] = -1;
  work.KKT[95] = 1;
  work.KKT[93] = -1;
  work.KKT[97] = -1;
  work.KKT[99] = 1;
  work.KKT[100] = -1;
  work.KKT[103] = 1;
  work.KKT[101] = -1;
  work.KKT[105] = -1;
  work.KKT[107] = 1;
  work.KKT[108] = -1;
  work.KKT[111] = 1;
  work.KKT[109] = -1;
  work.KKT[113] = -1;
  work.KKT[115] = 1;
  work.KKT[116] = -1;
  work.KKT[119] = 1;
  work.KKT[117] = -1;
  work.KKT[121] = -1;
  work.KKT[123] = 1;
  work.KKT[124] = -1;
  work.KKT[127] = 1;
  work.KKT[125] = -1;
  work.KKT[129] = -1;
  work.KKT[131] = 1;
  work.KKT[132] = -1;
  work.KKT[135] = 1;
  work.KKT[133] = -1;
  work.KKT[137] = -1;
  work.KKT[139] = 1;
  work.KKT[140] = -1;
  work.KKT[143] = 1;
  work.KKT[141] = -1;
  work.KKT[145] = -1;
  work.KKT[147] = 1;
  work.KKT[148] = -1;
  work.KKT[151] = 1;
  work.KKT[149] = -1;
  work.KKT[153] = -1;
  work.KKT[154] = -params.B[0];
  work.KKT[156] = -params.B[1];
  work.KKT[158] = -params.B[2];
  work.KKT[155] = 1;
  work.KKT[157] = 1;
  work.KKT[159] = 1;
  work.KKT[300] = -params.B[0];
  work.KKT[301] = -params.B[1];
  work.KKT[302] = -params.B[2];
  work.KKT[303] = -params.A[0];
  work.KKT[306] = -params.A[3];
  work.KKT[309] = -params.A[6];
  work.KKT[304] = -params.A[1];
  work.KKT[307] = -params.A[4];
  work.KKT[310] = -params.A[7];
  work.KKT[305] = -params.A[2];
  work.KKT[308] = -params.A[5];
  work.KKT[311] = -params.A[8];
  work.KKT[292] = 1;
  work.KKT[296] = 1;
  work.KKT[312] = 1;
  work.KKT[285] = -params.B[0];
  work.KKT[286] = -params.B[1];
  work.KKT[287] = -params.B[2];
  work.KKT[293] = -params.A[0];
  work.KKT[297] = -params.A[3];
  work.KKT[313] = -params.A[6];
  work.KKT[294] = -params.A[1];
  work.KKT[298] = -params.A[4];
  work.KKT[314] = -params.A[7];
  work.KKT[295] = -params.A[2];
  work.KKT[299] = -params.A[5];
  work.KKT[315] = -params.A[8];
  work.KKT[280] = 1;
  work.KKT[284] = 1;
  work.KKT[291] = 1;
  work.KKT[270] = -params.B[0];
  work.KKT[271] = -params.B[1];
  work.KKT[272] = -params.B[2];
  work.KKT[277] = -params.A[0];
  work.KKT[281] = -params.A[3];
  work.KKT[288] = -params.A[6];
  work.KKT[278] = -params.A[1];
  work.KKT[282] = -params.A[4];
  work.KKT[289] = -params.A[7];
  work.KKT[279] = -params.A[2];
  work.KKT[283] = -params.A[5];
  work.KKT[290] = -params.A[8];
  work.KKT[265] = 1;
  work.KKT[269] = 1;
  work.KKT[276] = 1;
  work.KKT[255] = -params.B[0];
  work.KKT[256] = -params.B[1];
  work.KKT[257] = -params.B[2];
  work.KKT[262] = -params.A[0];
  work.KKT[266] = -params.A[3];
  work.KKT[273] = -params.A[6];
  work.KKT[263] = -params.A[1];
  work.KKT[267] = -params.A[4];
  work.KKT[274] = -params.A[7];
  work.KKT[264] = -params.A[2];
  work.KKT[268] = -params.A[5];
  work.KKT[275] = -params.A[8];
  work.KKT[250] = 1;
  work.KKT[254] = 1;
  work.KKT[261] = 1;
  work.KKT[240] = -params.B[0];
  work.KKT[241] = -params.B[1];
  work.KKT[242] = -params.B[2];
  work.KKT[247] = -params.A[0];
  work.KKT[251] = -params.A[3];
  work.KKT[258] = -params.A[6];
  work.KKT[248] = -params.A[1];
  work.KKT[252] = -params.A[4];
  work.KKT[259] = -params.A[7];
  work.KKT[249] = -params.A[2];
  work.KKT[253] = -params.A[5];
  work.KKT[260] = -params.A[8];
  work.KKT[235] = 1;
  work.KKT[239] = 1;
  work.KKT[246] = 1;
  work.KKT[225] = -params.B[0];
  work.KKT[226] = -params.B[1];
  work.KKT[227] = -params.B[2];
  work.KKT[232] = -params.A[0];
  work.KKT[236] = -params.A[3];
  work.KKT[243] = -params.A[6];
  work.KKT[233] = -params.A[1];
  work.KKT[237] = -params.A[4];
  work.KKT[244] = -params.A[7];
  work.KKT[234] = -params.A[2];
  work.KKT[238] = -params.A[5];
  work.KKT[245] = -params.A[8];
  work.KKT[220] = 1;
  work.KKT[224] = 1;
  work.KKT[231] = 1;
  work.KKT[210] = -params.B[0];
  work.KKT[211] = -params.B[1];
  work.KKT[212] = -params.B[2];
  work.KKT[217] = -params.A[0];
  work.KKT[221] = -params.A[3];
  work.KKT[228] = -params.A[6];
  work.KKT[218] = -params.A[1];
  work.KKT[222] = -params.A[4];
  work.KKT[229] = -params.A[7];
  work.KKT[219] = -params.A[2];
  work.KKT[223] = -params.A[5];
  work.KKT[230] = -params.A[8];
  work.KKT[205] = 1;
  work.KKT[209] = 1;
  work.KKT[216] = 1;
  work.KKT[195] = -params.B[0];
  work.KKT[196] = -params.B[1];
  work.KKT[197] = -params.B[2];
  work.KKT[202] = -params.A[0];
  work.KKT[206] = -params.A[3];
  work.KKT[213] = -params.A[6];
  work.KKT[203] = -params.A[1];
  work.KKT[207] = -params.A[4];
  work.KKT[214] = -params.A[7];
  work.KKT[204] = -params.A[2];
  work.KKT[208] = -params.A[5];
  work.KKT[215] = -params.A[8];
  work.KKT[190] = 1;
  work.KKT[194] = 1;
  work.KKT[201] = 1;
  work.KKT[180] = -params.B[0];
  work.KKT[181] = -params.B[1];
  work.KKT[182] = -params.B[2];
  work.KKT[187] = -params.A[0];
  work.KKT[191] = -params.A[3];
  work.KKT[198] = -params.A[6];
  work.KKT[188] = -params.A[1];
  work.KKT[192] = -params.A[4];
  work.KKT[199] = -params.A[7];
  work.KKT[189] = -params.A[2];
  work.KKT[193] = -params.A[5];
  work.KKT[200] = -params.A[8];
  work.KKT[175] = 1;
  work.KKT[179] = 1;
  work.KKT[186] = 1;
  work.KKT[167] = -params.B[0];
  work.KKT[168] = -params.B[1];
  work.KKT[169] = -params.B[2];
  work.KKT[172] = -params.A[0];
  work.KKT[176] = -params.A[3];
  work.KKT[183] = -params.A[6];
  work.KKT[173] = -params.A[1];
  work.KKT[177] = -params.A[4];
  work.KKT[184] = -params.A[7];
  work.KKT[174] = -params.A[2];
  work.KKT[178] = -params.A[5];
  work.KKT[185] = -params.A[8];
  work.KKT[163] = 1;
  work.KKT[166] = 1;
  work.KKT[171] = 1;
}
