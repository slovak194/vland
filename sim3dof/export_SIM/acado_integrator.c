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


void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 6;
/* Vector of auxiliary variables; number of elements: 6. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (cos((xd[2]+u[1])));
a[1] = (u[0]*a[0]);
a[2] = (sin((xd[2]+u[1])));
a[3] = ((real_t)(-9.8066499999999994e+00)+(u[0]*a[2]));
a[4] = (sin(u[1]));
a[5] = (((real_t)(0.0000000000000000e+00)-u[0])*a[4]);

/* Compute outputs: */
out[0] = xd[3];
out[1] = xd[4];
out[2] = xd[5];
out[3] = a[1];
out[4] = a[3];
out[5] = a[5];
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 6;
/* Vector of auxiliary variables; number of elements: 28. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (real_t)(0.0000000000000000e+00);
a[1] = (real_t)(0.0000000000000000e+00);
a[2] = ((real_t)(-1.0000000000000000e+00)*(sin((xd[2]+u[1]))));
a[3] = (u[0]*a[2]);
a[4] = (real_t)(0.0000000000000000e+00);
a[5] = (real_t)(0.0000000000000000e+00);
a[6] = (real_t)(0.0000000000000000e+00);
a[7] = (cos((xd[2]+u[1])));
a[8] = (u[0]*a[2]);
a[9] = (real_t)(0.0000000000000000e+00);
a[10] = (real_t)(0.0000000000000000e+00);
a[11] = (cos((xd[2]+u[1])));
a[12] = (u[0]*a[11]);
a[13] = (real_t)(0.0000000000000000e+00);
a[14] = (real_t)(0.0000000000000000e+00);
a[15] = (real_t)(0.0000000000000000e+00);
a[16] = (sin((xd[2]+u[1])));
a[17] = (u[0]*a[11]);
a[18] = (real_t)(0.0000000000000000e+00);
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (real_t)(0.0000000000000000e+00);
a[22] = (real_t)(0.0000000000000000e+00);
a[23] = (real_t)(0.0000000000000000e+00);
a[24] = (sin(u[1]));
a[25] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[24]);
a[26] = (cos(u[1]));
a[27] = (((real_t)(0.0000000000000000e+00)-u[0])*a[26]);

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (real_t)(1.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(1.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(1.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = a[0];
out[25] = a[1];
out[26] = a[3];
out[27] = a[4];
out[28] = a[5];
out[29] = a[6];
out[30] = a[7];
out[31] = a[8];
out[32] = a[9];
out[33] = a[10];
out[34] = a[12];
out[35] = a[13];
out[36] = a[14];
out[37] = a[15];
out[38] = a[16];
out[39] = a[17];
out[40] = a[18];
out[41] = a[19];
out[42] = a[20];
out[43] = a[21];
out[44] = a[22];
out[45] = a[23];
out[46] = a[25];
out[47] = a[27];
}



void acado_solve_dim18_triangular( real_t* const A, real_t* const b )
{

b[17] = b[17]/A[323];
b[16] -= + A[305]*b[17];
b[16] = b[16]/A[304];
b[15] -= + A[287]*b[17];
b[15] -= + A[286]*b[16];
b[15] = b[15]/A[285];
b[14] -= + A[269]*b[17];
b[14] -= + A[268]*b[16];
b[14] -= + A[267]*b[15];
b[14] = b[14]/A[266];
b[13] -= + A[251]*b[17];
b[13] -= + A[250]*b[16];
b[13] -= + A[249]*b[15];
b[13] -= + A[248]*b[14];
b[13] = b[13]/A[247];
b[12] -= + A[233]*b[17];
b[12] -= + A[232]*b[16];
b[12] -= + A[231]*b[15];
b[12] -= + A[230]*b[14];
b[12] -= + A[229]*b[13];
b[12] = b[12]/A[228];
b[11] -= + A[215]*b[17];
b[11] -= + A[214]*b[16];
b[11] -= + A[213]*b[15];
b[11] -= + A[212]*b[14];
b[11] -= + A[211]*b[13];
b[11] -= + A[210]*b[12];
b[11] = b[11]/A[209];
b[10] -= + A[197]*b[17];
b[10] -= + A[196]*b[16];
b[10] -= + A[195]*b[15];
b[10] -= + A[194]*b[14];
b[10] -= + A[193]*b[13];
b[10] -= + A[192]*b[12];
b[10] -= + A[191]*b[11];
b[10] = b[10]/A[190];
b[9] -= + A[179]*b[17];
b[9] -= + A[178]*b[16];
b[9] -= + A[177]*b[15];
b[9] -= + A[176]*b[14];
b[9] -= + A[175]*b[13];
b[9] -= + A[174]*b[12];
b[9] -= + A[173]*b[11];
b[9] -= + A[172]*b[10];
b[9] = b[9]/A[171];
b[8] -= + A[161]*b[17];
b[8] -= + A[160]*b[16];
b[8] -= + A[159]*b[15];
b[8] -= + A[158]*b[14];
b[8] -= + A[157]*b[13];
b[8] -= + A[156]*b[12];
b[8] -= + A[155]*b[11];
b[8] -= + A[154]*b[10];
b[8] -= + A[153]*b[9];
b[8] = b[8]/A[152];
b[7] -= + A[143]*b[17];
b[7] -= + A[142]*b[16];
b[7] -= + A[141]*b[15];
b[7] -= + A[140]*b[14];
b[7] -= + A[139]*b[13];
b[7] -= + A[138]*b[12];
b[7] -= + A[137]*b[11];
b[7] -= + A[136]*b[10];
b[7] -= + A[135]*b[9];
b[7] -= + A[134]*b[8];
b[7] = b[7]/A[133];
b[6] -= + A[125]*b[17];
b[6] -= + A[124]*b[16];
b[6] -= + A[123]*b[15];
b[6] -= + A[122]*b[14];
b[6] -= + A[121]*b[13];
b[6] -= + A[120]*b[12];
b[6] -= + A[119]*b[11];
b[6] -= + A[118]*b[10];
b[6] -= + A[117]*b[9];
b[6] -= + A[116]*b[8];
b[6] -= + A[115]*b[7];
b[6] = b[6]/A[114];
b[5] -= + A[107]*b[17];
b[5] -= + A[106]*b[16];
b[5] -= + A[105]*b[15];
b[5] -= + A[104]*b[14];
b[5] -= + A[103]*b[13];
b[5] -= + A[102]*b[12];
b[5] -= + A[101]*b[11];
b[5] -= + A[100]*b[10];
b[5] -= + A[99]*b[9];
b[5] -= + A[98]*b[8];
b[5] -= + A[97]*b[7];
b[5] -= + A[96]*b[6];
b[5] = b[5]/A[95];
b[4] -= + A[89]*b[17];
b[4] -= + A[88]*b[16];
b[4] -= + A[87]*b[15];
b[4] -= + A[86]*b[14];
b[4] -= + A[85]*b[13];
b[4] -= + A[84]*b[12];
b[4] -= + A[83]*b[11];
b[4] -= + A[82]*b[10];
b[4] -= + A[81]*b[9];
b[4] -= + A[80]*b[8];
b[4] -= + A[79]*b[7];
b[4] -= + A[78]*b[6];
b[4] -= + A[77]*b[5];
b[4] = b[4]/A[76];
b[3] -= + A[71]*b[17];
b[3] -= + A[70]*b[16];
b[3] -= + A[69]*b[15];
b[3] -= + A[68]*b[14];
b[3] -= + A[67]*b[13];
b[3] -= + A[66]*b[12];
b[3] -= + A[65]*b[11];
b[3] -= + A[64]*b[10];
b[3] -= + A[63]*b[9];
b[3] -= + A[62]*b[8];
b[3] -= + A[61]*b[7];
b[3] -= + A[60]*b[6];
b[3] -= + A[59]*b[5];
b[3] -= + A[58]*b[4];
b[3] = b[3]/A[57];
b[2] -= + A[53]*b[17];
b[2] -= + A[52]*b[16];
b[2] -= + A[51]*b[15];
b[2] -= + A[50]*b[14];
b[2] -= + A[49]*b[13];
b[2] -= + A[48]*b[12];
b[2] -= + A[47]*b[11];
b[2] -= + A[46]*b[10];
b[2] -= + A[45]*b[9];
b[2] -= + A[44]*b[8];
b[2] -= + A[43]*b[7];
b[2] -= + A[42]*b[6];
b[2] -= + A[41]*b[5];
b[2] -= + A[40]*b[4];
b[2] -= + A[39]*b[3];
b[2] = b[2]/A[38];
b[1] -= + A[35]*b[17];
b[1] -= + A[34]*b[16];
b[1] -= + A[33]*b[15];
b[1] -= + A[32]*b[14];
b[1] -= + A[31]*b[13];
b[1] -= + A[30]*b[12];
b[1] -= + A[29]*b[11];
b[1] -= + A[28]*b[10];
b[1] -= + A[27]*b[9];
b[1] -= + A[26]*b[8];
b[1] -= + A[25]*b[7];
b[1] -= + A[24]*b[6];
b[1] -= + A[23]*b[5];
b[1] -= + A[22]*b[4];
b[1] -= + A[21]*b[3];
b[1] -= + A[20]*b[2];
b[1] = b[1]/A[19];
b[0] -= + A[17]*b[17];
b[0] -= + A[16]*b[16];
b[0] -= + A[15]*b[15];
b[0] -= + A[14]*b[14];
b[0] -= + A[13]*b[13];
b[0] -= + A[12]*b[12];
b[0] -= + A[11]*b[11];
b[0] -= + A[10]*b[10];
b[0] -= + A[9]*b[9];
b[0] -= + A[8]*b[8];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim18_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 18; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (17); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*18+i]);
	for( j=(i+1); j < 18; j++ ) {
		temp = fabs(A[j*18+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 18; ++k)
{
	acadoWorkspace.rk_dim18_swap = A[i*18+k];
	A[i*18+k] = A[indexMax*18+k];
	A[indexMax*18+k] = acadoWorkspace.rk_dim18_swap;
}
	acadoWorkspace.rk_dim18_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = acadoWorkspace.rk_dim18_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*18+i];
	for( j=i+1; j < 18; j++ ) {
		A[j*18+i] = -A[j*18+i]/A[i*18+i];
		for( k=i+1; k < 18; k++ ) {
			A[j*18+k] += A[j*18+i] * A[i*18+k];
		}
		b[j] += A[j*18+i] * b[i];
	}
}
det *= A[323];
det = fabs(det);
acado_solve_dim18_triangular( A, b );
return det;
}

void acado_solve_dim18_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

acadoWorkspace.rk_dim18_bPerm[0] = b[rk_perm[0]];
acadoWorkspace.rk_dim18_bPerm[1] = b[rk_perm[1]];
acadoWorkspace.rk_dim18_bPerm[2] = b[rk_perm[2]];
acadoWorkspace.rk_dim18_bPerm[3] = b[rk_perm[3]];
acadoWorkspace.rk_dim18_bPerm[4] = b[rk_perm[4]];
acadoWorkspace.rk_dim18_bPerm[5] = b[rk_perm[5]];
acadoWorkspace.rk_dim18_bPerm[6] = b[rk_perm[6]];
acadoWorkspace.rk_dim18_bPerm[7] = b[rk_perm[7]];
acadoWorkspace.rk_dim18_bPerm[8] = b[rk_perm[8]];
acadoWorkspace.rk_dim18_bPerm[9] = b[rk_perm[9]];
acadoWorkspace.rk_dim18_bPerm[10] = b[rk_perm[10]];
acadoWorkspace.rk_dim18_bPerm[11] = b[rk_perm[11]];
acadoWorkspace.rk_dim18_bPerm[12] = b[rk_perm[12]];
acadoWorkspace.rk_dim18_bPerm[13] = b[rk_perm[13]];
acadoWorkspace.rk_dim18_bPerm[14] = b[rk_perm[14]];
acadoWorkspace.rk_dim18_bPerm[15] = b[rk_perm[15]];
acadoWorkspace.rk_dim18_bPerm[16] = b[rk_perm[16]];
acadoWorkspace.rk_dim18_bPerm[17] = b[rk_perm[17]];
acadoWorkspace.rk_dim18_bPerm[1] += A[18]*acadoWorkspace.rk_dim18_bPerm[0];

acadoWorkspace.rk_dim18_bPerm[2] += A[36]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[2] += A[37]*acadoWorkspace.rk_dim18_bPerm[1];

acadoWorkspace.rk_dim18_bPerm[3] += A[54]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[3] += A[55]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[3] += A[56]*acadoWorkspace.rk_dim18_bPerm[2];

acadoWorkspace.rk_dim18_bPerm[4] += A[72]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[4] += A[73]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[4] += A[74]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[4] += A[75]*acadoWorkspace.rk_dim18_bPerm[3];

acadoWorkspace.rk_dim18_bPerm[5] += A[90]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[5] += A[91]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[5] += A[92]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[5] += A[93]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[5] += A[94]*acadoWorkspace.rk_dim18_bPerm[4];

acadoWorkspace.rk_dim18_bPerm[6] += A[108]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[6] += A[109]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[6] += A[110]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[6] += A[111]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[6] += A[112]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[6] += A[113]*acadoWorkspace.rk_dim18_bPerm[5];

acadoWorkspace.rk_dim18_bPerm[7] += A[126]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[7] += A[127]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[7] += A[128]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[7] += A[129]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[7] += A[130]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[7] += A[131]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[7] += A[132]*acadoWorkspace.rk_dim18_bPerm[6];

acadoWorkspace.rk_dim18_bPerm[8] += A[144]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[8] += A[145]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[8] += A[146]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[8] += A[147]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[8] += A[148]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[8] += A[149]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[8] += A[150]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[8] += A[151]*acadoWorkspace.rk_dim18_bPerm[7];

acadoWorkspace.rk_dim18_bPerm[9] += A[162]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[9] += A[163]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[9] += A[164]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[9] += A[165]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[9] += A[166]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[9] += A[167]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[9] += A[168]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[9] += A[169]*acadoWorkspace.rk_dim18_bPerm[7];
acadoWorkspace.rk_dim18_bPerm[9] += A[170]*acadoWorkspace.rk_dim18_bPerm[8];

acadoWorkspace.rk_dim18_bPerm[10] += A[180]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[10] += A[181]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[10] += A[182]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[10] += A[183]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[10] += A[184]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[10] += A[185]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[10] += A[186]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[10] += A[187]*acadoWorkspace.rk_dim18_bPerm[7];
acadoWorkspace.rk_dim18_bPerm[10] += A[188]*acadoWorkspace.rk_dim18_bPerm[8];
acadoWorkspace.rk_dim18_bPerm[10] += A[189]*acadoWorkspace.rk_dim18_bPerm[9];

acadoWorkspace.rk_dim18_bPerm[11] += A[198]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[11] += A[199]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[11] += A[200]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[11] += A[201]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[11] += A[202]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[11] += A[203]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[11] += A[204]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[11] += A[205]*acadoWorkspace.rk_dim18_bPerm[7];
acadoWorkspace.rk_dim18_bPerm[11] += A[206]*acadoWorkspace.rk_dim18_bPerm[8];
acadoWorkspace.rk_dim18_bPerm[11] += A[207]*acadoWorkspace.rk_dim18_bPerm[9];
acadoWorkspace.rk_dim18_bPerm[11] += A[208]*acadoWorkspace.rk_dim18_bPerm[10];

acadoWorkspace.rk_dim18_bPerm[12] += A[216]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[12] += A[217]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[12] += A[218]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[12] += A[219]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[12] += A[220]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[12] += A[221]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[12] += A[222]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[12] += A[223]*acadoWorkspace.rk_dim18_bPerm[7];
acadoWorkspace.rk_dim18_bPerm[12] += A[224]*acadoWorkspace.rk_dim18_bPerm[8];
acadoWorkspace.rk_dim18_bPerm[12] += A[225]*acadoWorkspace.rk_dim18_bPerm[9];
acadoWorkspace.rk_dim18_bPerm[12] += A[226]*acadoWorkspace.rk_dim18_bPerm[10];
acadoWorkspace.rk_dim18_bPerm[12] += A[227]*acadoWorkspace.rk_dim18_bPerm[11];

acadoWorkspace.rk_dim18_bPerm[13] += A[234]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[13] += A[235]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[13] += A[236]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[13] += A[237]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[13] += A[238]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[13] += A[239]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[13] += A[240]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[13] += A[241]*acadoWorkspace.rk_dim18_bPerm[7];
acadoWorkspace.rk_dim18_bPerm[13] += A[242]*acadoWorkspace.rk_dim18_bPerm[8];
acadoWorkspace.rk_dim18_bPerm[13] += A[243]*acadoWorkspace.rk_dim18_bPerm[9];
acadoWorkspace.rk_dim18_bPerm[13] += A[244]*acadoWorkspace.rk_dim18_bPerm[10];
acadoWorkspace.rk_dim18_bPerm[13] += A[245]*acadoWorkspace.rk_dim18_bPerm[11];
acadoWorkspace.rk_dim18_bPerm[13] += A[246]*acadoWorkspace.rk_dim18_bPerm[12];

acadoWorkspace.rk_dim18_bPerm[14] += A[252]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[14] += A[253]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[14] += A[254]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[14] += A[255]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[14] += A[256]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[14] += A[257]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[14] += A[258]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[14] += A[259]*acadoWorkspace.rk_dim18_bPerm[7];
acadoWorkspace.rk_dim18_bPerm[14] += A[260]*acadoWorkspace.rk_dim18_bPerm[8];
acadoWorkspace.rk_dim18_bPerm[14] += A[261]*acadoWorkspace.rk_dim18_bPerm[9];
acadoWorkspace.rk_dim18_bPerm[14] += A[262]*acadoWorkspace.rk_dim18_bPerm[10];
acadoWorkspace.rk_dim18_bPerm[14] += A[263]*acadoWorkspace.rk_dim18_bPerm[11];
acadoWorkspace.rk_dim18_bPerm[14] += A[264]*acadoWorkspace.rk_dim18_bPerm[12];
acadoWorkspace.rk_dim18_bPerm[14] += A[265]*acadoWorkspace.rk_dim18_bPerm[13];

acadoWorkspace.rk_dim18_bPerm[15] += A[270]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[15] += A[271]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[15] += A[272]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[15] += A[273]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[15] += A[274]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[15] += A[275]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[15] += A[276]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[15] += A[277]*acadoWorkspace.rk_dim18_bPerm[7];
acadoWorkspace.rk_dim18_bPerm[15] += A[278]*acadoWorkspace.rk_dim18_bPerm[8];
acadoWorkspace.rk_dim18_bPerm[15] += A[279]*acadoWorkspace.rk_dim18_bPerm[9];
acadoWorkspace.rk_dim18_bPerm[15] += A[280]*acadoWorkspace.rk_dim18_bPerm[10];
acadoWorkspace.rk_dim18_bPerm[15] += A[281]*acadoWorkspace.rk_dim18_bPerm[11];
acadoWorkspace.rk_dim18_bPerm[15] += A[282]*acadoWorkspace.rk_dim18_bPerm[12];
acadoWorkspace.rk_dim18_bPerm[15] += A[283]*acadoWorkspace.rk_dim18_bPerm[13];
acadoWorkspace.rk_dim18_bPerm[15] += A[284]*acadoWorkspace.rk_dim18_bPerm[14];

acadoWorkspace.rk_dim18_bPerm[16] += A[288]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[16] += A[289]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[16] += A[290]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[16] += A[291]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[16] += A[292]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[16] += A[293]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[16] += A[294]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[16] += A[295]*acadoWorkspace.rk_dim18_bPerm[7];
acadoWorkspace.rk_dim18_bPerm[16] += A[296]*acadoWorkspace.rk_dim18_bPerm[8];
acadoWorkspace.rk_dim18_bPerm[16] += A[297]*acadoWorkspace.rk_dim18_bPerm[9];
acadoWorkspace.rk_dim18_bPerm[16] += A[298]*acadoWorkspace.rk_dim18_bPerm[10];
acadoWorkspace.rk_dim18_bPerm[16] += A[299]*acadoWorkspace.rk_dim18_bPerm[11];
acadoWorkspace.rk_dim18_bPerm[16] += A[300]*acadoWorkspace.rk_dim18_bPerm[12];
acadoWorkspace.rk_dim18_bPerm[16] += A[301]*acadoWorkspace.rk_dim18_bPerm[13];
acadoWorkspace.rk_dim18_bPerm[16] += A[302]*acadoWorkspace.rk_dim18_bPerm[14];
acadoWorkspace.rk_dim18_bPerm[16] += A[303]*acadoWorkspace.rk_dim18_bPerm[15];

acadoWorkspace.rk_dim18_bPerm[17] += A[306]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[17] += A[307]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[17] += A[308]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[17] += A[309]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[17] += A[310]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[17] += A[311]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[17] += A[312]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[17] += A[313]*acadoWorkspace.rk_dim18_bPerm[7];
acadoWorkspace.rk_dim18_bPerm[17] += A[314]*acadoWorkspace.rk_dim18_bPerm[8];
acadoWorkspace.rk_dim18_bPerm[17] += A[315]*acadoWorkspace.rk_dim18_bPerm[9];
acadoWorkspace.rk_dim18_bPerm[17] += A[316]*acadoWorkspace.rk_dim18_bPerm[10];
acadoWorkspace.rk_dim18_bPerm[17] += A[317]*acadoWorkspace.rk_dim18_bPerm[11];
acadoWorkspace.rk_dim18_bPerm[17] += A[318]*acadoWorkspace.rk_dim18_bPerm[12];
acadoWorkspace.rk_dim18_bPerm[17] += A[319]*acadoWorkspace.rk_dim18_bPerm[13];
acadoWorkspace.rk_dim18_bPerm[17] += A[320]*acadoWorkspace.rk_dim18_bPerm[14];
acadoWorkspace.rk_dim18_bPerm[17] += A[321]*acadoWorkspace.rk_dim18_bPerm[15];
acadoWorkspace.rk_dim18_bPerm[17] += A[322]*acadoWorkspace.rk_dim18_bPerm[16];


acado_solve_dim18_triangular( A, acadoWorkspace.rk_dim18_bPerm );
b[0] = acadoWorkspace.rk_dim18_bPerm[0];
b[1] = acadoWorkspace.rk_dim18_bPerm[1];
b[2] = acadoWorkspace.rk_dim18_bPerm[2];
b[3] = acadoWorkspace.rk_dim18_bPerm[3];
b[4] = acadoWorkspace.rk_dim18_bPerm[4];
b[5] = acadoWorkspace.rk_dim18_bPerm[5];
b[6] = acadoWorkspace.rk_dim18_bPerm[6];
b[7] = acadoWorkspace.rk_dim18_bPerm[7];
b[8] = acadoWorkspace.rk_dim18_bPerm[8];
b[9] = acadoWorkspace.rk_dim18_bPerm[9];
b[10] = acadoWorkspace.rk_dim18_bPerm[10];
b[11] = acadoWorkspace.rk_dim18_bPerm[11];
b[12] = acadoWorkspace.rk_dim18_bPerm[12];
b[13] = acadoWorkspace.rk_dim18_bPerm[13];
b[14] = acadoWorkspace.rk_dim18_bPerm[14];
b[15] = acadoWorkspace.rk_dim18_bPerm[15];
b[16] = acadoWorkspace.rk_dim18_bPerm[16];
b[17] = acadoWorkspace.rk_dim18_bPerm[17];
}



/** Matrix of size: 3 x 3 (row major format) */
static const real_t acado_Ah_mat[ 9 ] = 
{ 3.9363095444732085e-03, -1.3107085170039681e-03, 4.7541948696440319e-04, 
7.8884862947817443e-03, 5.8414682333045656e-03, -8.3097504251995822e-04, 
7.5280612540093439e-03, 1.0249716523768430e-02, 2.2222222222222222e-03 };


/* Fixed step size:0.02 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[6] = rk_eta[54];
acadoWorkspace.rk_xxx[7] = rk_eta[55];

for (run = 0; run < 5; ++run)
{
if( run > 0 ) {
for (i = 0; i < 6; ++i)
{
acadoWorkspace.rk_diffsPrev2[i * 8] = rk_eta[i * 6 + 6];
acadoWorkspace.rk_diffsPrev2[i * 8 + 1] = rk_eta[i * 6 + 7];
acadoWorkspace.rk_diffsPrev2[i * 8 + 2] = rk_eta[i * 6 + 8];
acadoWorkspace.rk_diffsPrev2[i * 8 + 3] = rk_eta[i * 6 + 9];
acadoWorkspace.rk_diffsPrev2[i * 8 + 4] = rk_eta[i * 6 + 10];
acadoWorkspace.rk_diffsPrev2[i * 8 + 5] = rk_eta[i * 6 + 11];
acadoWorkspace.rk_diffsPrev2[i * 8 + 6] = rk_eta[i * 2 + 42];
acadoWorkspace.rk_diffsPrev2[i * 8 + 7] = rk_eta[i * 2 + 43];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 3; ++run1)
{
for (j = 0; j < 6; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 3]*acadoWorkspace.rk_kkk[tmp_index1 * 3];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 3 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 3 + 1];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 3 + 2]*acadoWorkspace.rk_kkk[tmp_index1 * 3 + 2];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 48 ]) );
for (j = 0; j < 6; ++j)
{
tmp_index1 = (run1 * 6) + (j);
acadoWorkspace.rk_A[tmp_index1 * 18] = + acado_Ah_mat[run1 * 3]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 1] = + acado_Ah_mat[run1 * 3]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 2] = + acado_Ah_mat[run1 * 3]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 3] = + acado_Ah_mat[run1 * 3]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 4] = + acado_Ah_mat[run1 * 3]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 5] = + acado_Ah_mat[run1 * 3]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 5)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 18) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 18 + 6] = + acado_Ah_mat[run1 * 3 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 7] = + acado_Ah_mat[run1 * 3 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 8] = + acado_Ah_mat[run1 * 3 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 9] = + acado_Ah_mat[run1 * 3 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 10] = + acado_Ah_mat[run1 * 3 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 11] = + acado_Ah_mat[run1 * 3 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 5)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 18) + (j + 6)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 18 + 12] = + acado_Ah_mat[run1 * 3 + 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 13] = + acado_Ah_mat[run1 * 3 + 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 14] = + acado_Ah_mat[run1 * 3 + 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 15] = + acado_Ah_mat[run1 * 3 + 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 16] = + acado_Ah_mat[run1 * 3 + 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 17] = + acado_Ah_mat[run1 * 3 + 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 5)];
if( 2 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 18) + (j + 12)] -= 1.0000000000000000e+00;
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 6] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 6 + 1] = acadoWorkspace.rk_kkk[run1 + 3] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 6 + 2] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 6 + 3] = acadoWorkspace.rk_kkk[run1 + 9] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 6 + 4] = acadoWorkspace.rk_kkk[run1 + 12] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 6 + 5] = acadoWorkspace.rk_kkk[run1 + 15] - acadoWorkspace.rk_rhsTemp[5];
}
det = acado_solve_dim18_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim18_perm );
for (j = 0; j < 3; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 6];
acadoWorkspace.rk_kkk[j + 3] += acadoWorkspace.rk_b[j * 6 + 1];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 6 + 2];
acadoWorkspace.rk_kkk[j + 9] += acadoWorkspace.rk_b[j * 6 + 3];
acadoWorkspace.rk_kkk[j + 12] += acadoWorkspace.rk_b[j * 6 + 4];
acadoWorkspace.rk_kkk[j + 15] += acadoWorkspace.rk_b[j * 6 + 5];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 3; ++run1)
{
for (j = 0; j < 6; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 3]*acadoWorkspace.rk_kkk[tmp_index1 * 3];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 3 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 3 + 1];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 3 + 2]*acadoWorkspace.rk_kkk[tmp_index1 * 3 + 2];
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 6] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 6 + 1] = acadoWorkspace.rk_kkk[run1 + 3] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 6 + 2] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 6 + 3] = acadoWorkspace.rk_kkk[run1 + 9] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 6 + 4] = acadoWorkspace.rk_kkk[run1 + 12] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 6 + 5] = acadoWorkspace.rk_kkk[run1 + 15] - acadoWorkspace.rk_rhsTemp[5];
}
acado_solve_dim18_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim18_perm );
for (j = 0; j < 3; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 6];
acadoWorkspace.rk_kkk[j + 3] += acadoWorkspace.rk_b[j * 6 + 1];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 6 + 2];
acadoWorkspace.rk_kkk[j + 9] += acadoWorkspace.rk_b[j * 6 + 3];
acadoWorkspace.rk_kkk[j + 12] += acadoWorkspace.rk_b[j * 6 + 4];
acadoWorkspace.rk_kkk[j + 15] += acadoWorkspace.rk_b[j * 6 + 5];
}
}
for (run1 = 0; run1 < 3; ++run1)
{
for (j = 0; j < 6; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 3]*acadoWorkspace.rk_kkk[tmp_index1 * 3];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 3 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 3 + 1];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 3 + 2]*acadoWorkspace.rk_kkk[tmp_index1 * 3 + 2];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 48 ]) );
for (j = 0; j < 6; ++j)
{
tmp_index1 = (run1 * 6) + (j);
acadoWorkspace.rk_A[tmp_index1 * 18] = + acado_Ah_mat[run1 * 3]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 1] = + acado_Ah_mat[run1 * 3]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 2] = + acado_Ah_mat[run1 * 3]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 3] = + acado_Ah_mat[run1 * 3]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 4] = + acado_Ah_mat[run1 * 3]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 5] = + acado_Ah_mat[run1 * 3]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 5)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 18) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 18 + 6] = + acado_Ah_mat[run1 * 3 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 7] = + acado_Ah_mat[run1 * 3 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 8] = + acado_Ah_mat[run1 * 3 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 9] = + acado_Ah_mat[run1 * 3 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 10] = + acado_Ah_mat[run1 * 3 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 11] = + acado_Ah_mat[run1 * 3 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 5)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 18) + (j + 6)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 18 + 12] = + acado_Ah_mat[run1 * 3 + 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 13] = + acado_Ah_mat[run1 * 3 + 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 14] = + acado_Ah_mat[run1 * 3 + 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 15] = + acado_Ah_mat[run1 * 3 + 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 16] = + acado_Ah_mat[run1 * 3 + 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 17] = + acado_Ah_mat[run1 * 3 + 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 5)];
if( 2 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 18) + (j + 12)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 6; ++run1)
{
for (i = 0; i < 3; ++i)
{
acadoWorkspace.rk_b[i * 6] = - acadoWorkspace.rk_diffsTemp2[(i * 48) + (run1)];
acadoWorkspace.rk_b[i * 6 + 1] = - acadoWorkspace.rk_diffsTemp2[(i * 48) + (run1 + 8)];
acadoWorkspace.rk_b[i * 6 + 2] = - acadoWorkspace.rk_diffsTemp2[(i * 48) + (run1 + 16)];
acadoWorkspace.rk_b[i * 6 + 3] = - acadoWorkspace.rk_diffsTemp2[(i * 48) + (run1 + 24)];
acadoWorkspace.rk_b[i * 6 + 4] = - acadoWorkspace.rk_diffsTemp2[(i * 48) + (run1 + 32)];
acadoWorkspace.rk_b[i * 6 + 5] = - acadoWorkspace.rk_diffsTemp2[(i * 48) + (run1 + 40)];
}
if( 0 == run1 ) {
det = acado_solve_dim18_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim18_perm );
}
 else {
acado_solve_dim18_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim18_perm );
}
for (i = 0; i < 3; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 6];
acadoWorkspace.rk_diffK[i + 3] = acadoWorkspace.rk_b[i * 6 + 1];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 6 + 2];
acadoWorkspace.rk_diffK[i + 9] = acadoWorkspace.rk_b[i * 6 + 3];
acadoWorkspace.rk_diffK[i + 12] = acadoWorkspace.rk_b[i * 6 + 4];
acadoWorkspace.rk_diffK[i + 15] = acadoWorkspace.rk_b[i * 6 + 5];
}
for (i = 0; i < 6; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 8) + (run1)] = (i == run1-0);
acadoWorkspace.rk_diffsNew2[(i * 8) + (run1)] += + acadoWorkspace.rk_diffK[i * 3]*(real_t)7.5280612540093439e-03 + acadoWorkspace.rk_diffK[i * 3 + 1]*(real_t)1.0249716523768430e-02 + acadoWorkspace.rk_diffK[i * 3 + 2]*(real_t)2.2222222222222222e-03;
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (i = 0; i < 3; ++i)
{
for (j = 0; j < 6; ++j)
{
tmp_index1 = (i * 6) + (j);
tmp_index2 = (run1) + (j * 8);
acadoWorkspace.rk_b[tmp_index1] = - acadoWorkspace.rk_diffsTemp2[(i * 48) + (tmp_index2 + 6)];
}
}
acado_solve_dim18_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim18_perm );
for (i = 0; i < 3; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 6];
acadoWorkspace.rk_diffK[i + 3] = acadoWorkspace.rk_b[i * 6 + 1];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 6 + 2];
acadoWorkspace.rk_diffK[i + 9] = acadoWorkspace.rk_b[i * 6 + 3];
acadoWorkspace.rk_diffK[i + 12] = acadoWorkspace.rk_b[i * 6 + 4];
acadoWorkspace.rk_diffK[i + 15] = acadoWorkspace.rk_b[i * 6 + 5];
}
for (i = 0; i < 6; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 8) + (run1 + 6)] = + acadoWorkspace.rk_diffK[i * 3]*(real_t)7.5280612540093439e-03 + acadoWorkspace.rk_diffK[i * 3 + 1]*(real_t)1.0249716523768430e-02 + acadoWorkspace.rk_diffK[i * 3 + 2]*(real_t)2.2222222222222222e-03;
}
}
rk_eta[0] += + acadoWorkspace.rk_kkk[0]*(real_t)7.5280612540093439e-03 + acadoWorkspace.rk_kkk[1]*(real_t)1.0249716523768430e-02 + acadoWorkspace.rk_kkk[2]*(real_t)2.2222222222222222e-03;
rk_eta[1] += + acadoWorkspace.rk_kkk[3]*(real_t)7.5280612540093439e-03 + acadoWorkspace.rk_kkk[4]*(real_t)1.0249716523768430e-02 + acadoWorkspace.rk_kkk[5]*(real_t)2.2222222222222222e-03;
rk_eta[2] += + acadoWorkspace.rk_kkk[6]*(real_t)7.5280612540093439e-03 + acadoWorkspace.rk_kkk[7]*(real_t)1.0249716523768430e-02 + acadoWorkspace.rk_kkk[8]*(real_t)2.2222222222222222e-03;
rk_eta[3] += + acadoWorkspace.rk_kkk[9]*(real_t)7.5280612540093439e-03 + acadoWorkspace.rk_kkk[10]*(real_t)1.0249716523768430e-02 + acadoWorkspace.rk_kkk[11]*(real_t)2.2222222222222222e-03;
rk_eta[4] += + acadoWorkspace.rk_kkk[12]*(real_t)7.5280612540093439e-03 + acadoWorkspace.rk_kkk[13]*(real_t)1.0249716523768430e-02 + acadoWorkspace.rk_kkk[14]*(real_t)2.2222222222222222e-03;
rk_eta[5] += + acadoWorkspace.rk_kkk[15]*(real_t)7.5280612540093439e-03 + acadoWorkspace.rk_kkk[16]*(real_t)1.0249716523768430e-02 + acadoWorkspace.rk_kkk[17]*(real_t)2.2222222222222222e-03;
if( run == 0 ) {
for (i = 0; i < 6; ++i)
{
for (j = 0; j < 6; ++j)
{
tmp_index2 = (j) + (i * 6);
rk_eta[tmp_index2 + 6] = acadoWorkspace.rk_diffsNew2[(i * 8) + (j)];
}
for (j = 0; j < 2; ++j)
{
tmp_index2 = (j) + (i * 2);
rk_eta[tmp_index2 + 42] = acadoWorkspace.rk_diffsNew2[(i * 8) + (j + 6)];
}
}
}
else {
for (i = 0; i < 6; ++i)
{
for (j = 0; j < 6; ++j)
{
tmp_index2 = (j) + (i * 6);
rk_eta[tmp_index2 + 6] = + acadoWorkspace.rk_diffsNew2[i * 8]*acadoWorkspace.rk_diffsPrev2[j];
rk_eta[tmp_index2 + 6] += + acadoWorkspace.rk_diffsNew2[i * 8 + 1]*acadoWorkspace.rk_diffsPrev2[j + 8];
rk_eta[tmp_index2 + 6] += + acadoWorkspace.rk_diffsNew2[i * 8 + 2]*acadoWorkspace.rk_diffsPrev2[j + 16];
rk_eta[tmp_index2 + 6] += + acadoWorkspace.rk_diffsNew2[i * 8 + 3]*acadoWorkspace.rk_diffsPrev2[j + 24];
rk_eta[tmp_index2 + 6] += + acadoWorkspace.rk_diffsNew2[i * 8 + 4]*acadoWorkspace.rk_diffsPrev2[j + 32];
rk_eta[tmp_index2 + 6] += + acadoWorkspace.rk_diffsNew2[i * 8 + 5]*acadoWorkspace.rk_diffsPrev2[j + 40];
}
for (j = 0; j < 2; ++j)
{
tmp_index2 = (j) + (i * 2);
rk_eta[tmp_index2 + 42] = acadoWorkspace.rk_diffsNew2[(i * 8) + (j + 6)];
rk_eta[tmp_index2 + 42] += + acadoWorkspace.rk_diffsNew2[i * 8]*acadoWorkspace.rk_diffsPrev2[j + 6];
rk_eta[tmp_index2 + 42] += + acadoWorkspace.rk_diffsNew2[i * 8 + 1]*acadoWorkspace.rk_diffsPrev2[j + 14];
rk_eta[tmp_index2 + 42] += + acadoWorkspace.rk_diffsNew2[i * 8 + 2]*acadoWorkspace.rk_diffsPrev2[j + 22];
rk_eta[tmp_index2 + 42] += + acadoWorkspace.rk_diffsNew2[i * 8 + 3]*acadoWorkspace.rk_diffsPrev2[j + 30];
rk_eta[tmp_index2 + 42] += + acadoWorkspace.rk_diffsNew2[i * 8 + 4]*acadoWorkspace.rk_diffsPrev2[j + 38];
rk_eta[tmp_index2 + 42] += + acadoWorkspace.rk_diffsNew2[i * 8 + 5]*acadoWorkspace.rk_diffsPrev2[j + 46];
}
}
}
resetIntegrator = 0;
acadoWorkspace.rk_ttt += 2.0000000000000001e-01;
}
for (i = 0; i < 6; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



