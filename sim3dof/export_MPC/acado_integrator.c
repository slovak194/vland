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



void acado_solve_dim12_triangular( real_t* const A, real_t* const b )
{

b[11] = b[11]/A[143];
b[10] -= + A[131]*b[11];
b[10] = b[10]/A[130];
b[9] -= + A[119]*b[11];
b[9] -= + A[118]*b[10];
b[9] = b[9]/A[117];
b[8] -= + A[107]*b[11];
b[8] -= + A[106]*b[10];
b[8] -= + A[105]*b[9];
b[8] = b[8]/A[104];
b[7] -= + A[95]*b[11];
b[7] -= + A[94]*b[10];
b[7] -= + A[93]*b[9];
b[7] -= + A[92]*b[8];
b[7] = b[7]/A[91];
b[6] -= + A[83]*b[11];
b[6] -= + A[82]*b[10];
b[6] -= + A[81]*b[9];
b[6] -= + A[80]*b[8];
b[6] -= + A[79]*b[7];
b[6] = b[6]/A[78];
b[5] -= + A[71]*b[11];
b[5] -= + A[70]*b[10];
b[5] -= + A[69]*b[9];
b[5] -= + A[68]*b[8];
b[5] -= + A[67]*b[7];
b[5] -= + A[66]*b[6];
b[5] = b[5]/A[65];
b[4] -= + A[59]*b[11];
b[4] -= + A[58]*b[10];
b[4] -= + A[57]*b[9];
b[4] -= + A[56]*b[8];
b[4] -= + A[55]*b[7];
b[4] -= + A[54]*b[6];
b[4] -= + A[53]*b[5];
b[4] = b[4]/A[52];
b[3] -= + A[47]*b[11];
b[3] -= + A[46]*b[10];
b[3] -= + A[45]*b[9];
b[3] -= + A[44]*b[8];
b[3] -= + A[43]*b[7];
b[3] -= + A[42]*b[6];
b[3] -= + A[41]*b[5];
b[3] -= + A[40]*b[4];
b[3] = b[3]/A[39];
b[2] -= + A[35]*b[11];
b[2] -= + A[34]*b[10];
b[2] -= + A[33]*b[9];
b[2] -= + A[32]*b[8];
b[2] -= + A[31]*b[7];
b[2] -= + A[30]*b[6];
b[2] -= + A[29]*b[5];
b[2] -= + A[28]*b[4];
b[2] -= + A[27]*b[3];
b[2] = b[2]/A[26];
b[1] -= + A[23]*b[11];
b[1] -= + A[22]*b[10];
b[1] -= + A[21]*b[9];
b[1] -= + A[20]*b[8];
b[1] -= + A[19]*b[7];
b[1] -= + A[18]*b[6];
b[1] -= + A[17]*b[5];
b[1] -= + A[16]*b[4];
b[1] -= + A[15]*b[3];
b[1] -= + A[14]*b[2];
b[1] = b[1]/A[13];
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

real_t acado_solve_dim12_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 12; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (11); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*12+i]);
	for( j=(i+1); j < 12; j++ ) {
		temp = fabs(A[j*12+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 12; ++k)
{
	acadoWorkspace.rk_dim12_swap = A[i*12+k];
	A[i*12+k] = A[indexMax*12+k];
	A[indexMax*12+k] = acadoWorkspace.rk_dim12_swap;
}
	acadoWorkspace.rk_dim12_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = acadoWorkspace.rk_dim12_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*12+i];
	for( j=i+1; j < 12; j++ ) {
		A[j*12+i] = -A[j*12+i]/A[i*12+i];
		for( k=i+1; k < 12; k++ ) {
			A[j*12+k] += A[j*12+i] * A[i*12+k];
		}
		b[j] += A[j*12+i] * b[i];
	}
}
det *= A[143];
det = fabs(det);
acado_solve_dim12_triangular( A, b );
return det;
}

void acado_solve_dim12_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

acadoWorkspace.rk_dim12_bPerm[0] = b[rk_perm[0]];
acadoWorkspace.rk_dim12_bPerm[1] = b[rk_perm[1]];
acadoWorkspace.rk_dim12_bPerm[2] = b[rk_perm[2]];
acadoWorkspace.rk_dim12_bPerm[3] = b[rk_perm[3]];
acadoWorkspace.rk_dim12_bPerm[4] = b[rk_perm[4]];
acadoWorkspace.rk_dim12_bPerm[5] = b[rk_perm[5]];
acadoWorkspace.rk_dim12_bPerm[6] = b[rk_perm[6]];
acadoWorkspace.rk_dim12_bPerm[7] = b[rk_perm[7]];
acadoWorkspace.rk_dim12_bPerm[8] = b[rk_perm[8]];
acadoWorkspace.rk_dim12_bPerm[9] = b[rk_perm[9]];
acadoWorkspace.rk_dim12_bPerm[10] = b[rk_perm[10]];
acadoWorkspace.rk_dim12_bPerm[11] = b[rk_perm[11]];
acadoWorkspace.rk_dim12_bPerm[1] += A[12]*acadoWorkspace.rk_dim12_bPerm[0];

acadoWorkspace.rk_dim12_bPerm[2] += A[24]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[2] += A[25]*acadoWorkspace.rk_dim12_bPerm[1];

acadoWorkspace.rk_dim12_bPerm[3] += A[36]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[3] += A[37]*acadoWorkspace.rk_dim12_bPerm[1];
acadoWorkspace.rk_dim12_bPerm[3] += A[38]*acadoWorkspace.rk_dim12_bPerm[2];

acadoWorkspace.rk_dim12_bPerm[4] += A[48]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[4] += A[49]*acadoWorkspace.rk_dim12_bPerm[1];
acadoWorkspace.rk_dim12_bPerm[4] += A[50]*acadoWorkspace.rk_dim12_bPerm[2];
acadoWorkspace.rk_dim12_bPerm[4] += A[51]*acadoWorkspace.rk_dim12_bPerm[3];

acadoWorkspace.rk_dim12_bPerm[5] += A[60]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[5] += A[61]*acadoWorkspace.rk_dim12_bPerm[1];
acadoWorkspace.rk_dim12_bPerm[5] += A[62]*acadoWorkspace.rk_dim12_bPerm[2];
acadoWorkspace.rk_dim12_bPerm[5] += A[63]*acadoWorkspace.rk_dim12_bPerm[3];
acadoWorkspace.rk_dim12_bPerm[5] += A[64]*acadoWorkspace.rk_dim12_bPerm[4];

acadoWorkspace.rk_dim12_bPerm[6] += A[72]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[6] += A[73]*acadoWorkspace.rk_dim12_bPerm[1];
acadoWorkspace.rk_dim12_bPerm[6] += A[74]*acadoWorkspace.rk_dim12_bPerm[2];
acadoWorkspace.rk_dim12_bPerm[6] += A[75]*acadoWorkspace.rk_dim12_bPerm[3];
acadoWorkspace.rk_dim12_bPerm[6] += A[76]*acadoWorkspace.rk_dim12_bPerm[4];
acadoWorkspace.rk_dim12_bPerm[6] += A[77]*acadoWorkspace.rk_dim12_bPerm[5];

acadoWorkspace.rk_dim12_bPerm[7] += A[84]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[7] += A[85]*acadoWorkspace.rk_dim12_bPerm[1];
acadoWorkspace.rk_dim12_bPerm[7] += A[86]*acadoWorkspace.rk_dim12_bPerm[2];
acadoWorkspace.rk_dim12_bPerm[7] += A[87]*acadoWorkspace.rk_dim12_bPerm[3];
acadoWorkspace.rk_dim12_bPerm[7] += A[88]*acadoWorkspace.rk_dim12_bPerm[4];
acadoWorkspace.rk_dim12_bPerm[7] += A[89]*acadoWorkspace.rk_dim12_bPerm[5];
acadoWorkspace.rk_dim12_bPerm[7] += A[90]*acadoWorkspace.rk_dim12_bPerm[6];

acadoWorkspace.rk_dim12_bPerm[8] += A[96]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[8] += A[97]*acadoWorkspace.rk_dim12_bPerm[1];
acadoWorkspace.rk_dim12_bPerm[8] += A[98]*acadoWorkspace.rk_dim12_bPerm[2];
acadoWorkspace.rk_dim12_bPerm[8] += A[99]*acadoWorkspace.rk_dim12_bPerm[3];
acadoWorkspace.rk_dim12_bPerm[8] += A[100]*acadoWorkspace.rk_dim12_bPerm[4];
acadoWorkspace.rk_dim12_bPerm[8] += A[101]*acadoWorkspace.rk_dim12_bPerm[5];
acadoWorkspace.rk_dim12_bPerm[8] += A[102]*acadoWorkspace.rk_dim12_bPerm[6];
acadoWorkspace.rk_dim12_bPerm[8] += A[103]*acadoWorkspace.rk_dim12_bPerm[7];

acadoWorkspace.rk_dim12_bPerm[9] += A[108]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[9] += A[109]*acadoWorkspace.rk_dim12_bPerm[1];
acadoWorkspace.rk_dim12_bPerm[9] += A[110]*acadoWorkspace.rk_dim12_bPerm[2];
acadoWorkspace.rk_dim12_bPerm[9] += A[111]*acadoWorkspace.rk_dim12_bPerm[3];
acadoWorkspace.rk_dim12_bPerm[9] += A[112]*acadoWorkspace.rk_dim12_bPerm[4];
acadoWorkspace.rk_dim12_bPerm[9] += A[113]*acadoWorkspace.rk_dim12_bPerm[5];
acadoWorkspace.rk_dim12_bPerm[9] += A[114]*acadoWorkspace.rk_dim12_bPerm[6];
acadoWorkspace.rk_dim12_bPerm[9] += A[115]*acadoWorkspace.rk_dim12_bPerm[7];
acadoWorkspace.rk_dim12_bPerm[9] += A[116]*acadoWorkspace.rk_dim12_bPerm[8];

acadoWorkspace.rk_dim12_bPerm[10] += A[120]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[10] += A[121]*acadoWorkspace.rk_dim12_bPerm[1];
acadoWorkspace.rk_dim12_bPerm[10] += A[122]*acadoWorkspace.rk_dim12_bPerm[2];
acadoWorkspace.rk_dim12_bPerm[10] += A[123]*acadoWorkspace.rk_dim12_bPerm[3];
acadoWorkspace.rk_dim12_bPerm[10] += A[124]*acadoWorkspace.rk_dim12_bPerm[4];
acadoWorkspace.rk_dim12_bPerm[10] += A[125]*acadoWorkspace.rk_dim12_bPerm[5];
acadoWorkspace.rk_dim12_bPerm[10] += A[126]*acadoWorkspace.rk_dim12_bPerm[6];
acadoWorkspace.rk_dim12_bPerm[10] += A[127]*acadoWorkspace.rk_dim12_bPerm[7];
acadoWorkspace.rk_dim12_bPerm[10] += A[128]*acadoWorkspace.rk_dim12_bPerm[8];
acadoWorkspace.rk_dim12_bPerm[10] += A[129]*acadoWorkspace.rk_dim12_bPerm[9];

acadoWorkspace.rk_dim12_bPerm[11] += A[132]*acadoWorkspace.rk_dim12_bPerm[0];
acadoWorkspace.rk_dim12_bPerm[11] += A[133]*acadoWorkspace.rk_dim12_bPerm[1];
acadoWorkspace.rk_dim12_bPerm[11] += A[134]*acadoWorkspace.rk_dim12_bPerm[2];
acadoWorkspace.rk_dim12_bPerm[11] += A[135]*acadoWorkspace.rk_dim12_bPerm[3];
acadoWorkspace.rk_dim12_bPerm[11] += A[136]*acadoWorkspace.rk_dim12_bPerm[4];
acadoWorkspace.rk_dim12_bPerm[11] += A[137]*acadoWorkspace.rk_dim12_bPerm[5];
acadoWorkspace.rk_dim12_bPerm[11] += A[138]*acadoWorkspace.rk_dim12_bPerm[6];
acadoWorkspace.rk_dim12_bPerm[11] += A[139]*acadoWorkspace.rk_dim12_bPerm[7];
acadoWorkspace.rk_dim12_bPerm[11] += A[140]*acadoWorkspace.rk_dim12_bPerm[8];
acadoWorkspace.rk_dim12_bPerm[11] += A[141]*acadoWorkspace.rk_dim12_bPerm[9];
acadoWorkspace.rk_dim12_bPerm[11] += A[142]*acadoWorkspace.rk_dim12_bPerm[10];


acado_solve_dim12_triangular( A, acadoWorkspace.rk_dim12_bPerm );
b[0] = acadoWorkspace.rk_dim12_bPerm[0];
b[1] = acadoWorkspace.rk_dim12_bPerm[1];
b[2] = acadoWorkspace.rk_dim12_bPerm[2];
b[3] = acadoWorkspace.rk_dim12_bPerm[3];
b[4] = acadoWorkspace.rk_dim12_bPerm[4];
b[5] = acadoWorkspace.rk_dim12_bPerm[5];
b[6] = acadoWorkspace.rk_dim12_bPerm[6];
b[7] = acadoWorkspace.rk_dim12_bPerm[7];
b[8] = acadoWorkspace.rk_dim12_bPerm[8];
b[9] = acadoWorkspace.rk_dim12_bPerm[9];
b[10] = acadoWorkspace.rk_dim12_bPerm[10];
b[11] = acadoWorkspace.rk_dim12_bPerm[11];
}



/** Matrix of size: 2 x 2 (row major format) */
static const real_t acado_Ah_mat[ 4 ] = 
{ 1.2500000000000001e-02, 2.6933756729740646e-02, 
-1.9337567297406434e-03, 1.2500000000000001e-02 };


/* Fixed step size:0.05 */
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

for (run = 0; run < 2; ++run)
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
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 6; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 48 ]) );
for (j = 0; j < 6; ++j)
{
tmp_index1 = (run1 * 6) + (j);
acadoWorkspace.rk_A[tmp_index1 * 12] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 1] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 2] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 3] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 4] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 5] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 5)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 12) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 12 + 6] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 7] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 8] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 9] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 10] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 11] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 5)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 12) + (j + 6)] -= 1.0000000000000000e+00;
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 6] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 6 + 1] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 6 + 2] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 6 + 3] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 6 + 4] = acadoWorkspace.rk_kkk[run1 + 8] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 6 + 5] = acadoWorkspace.rk_kkk[run1 + 10] - acadoWorkspace.rk_rhsTemp[5];
}
det = acado_solve_dim12_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim12_perm );
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 6];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 6 + 1];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 6 + 2];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 6 + 3];
acadoWorkspace.rk_kkk[j + 8] += acadoWorkspace.rk_b[j * 6 + 4];
acadoWorkspace.rk_kkk[j + 10] += acadoWorkspace.rk_b[j * 6 + 5];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 6; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 6] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 6 + 1] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 6 + 2] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 6 + 3] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 6 + 4] = acadoWorkspace.rk_kkk[run1 + 8] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 6 + 5] = acadoWorkspace.rk_kkk[run1 + 10] - acadoWorkspace.rk_rhsTemp[5];
}
acado_solve_dim12_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim12_perm );
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 6];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 6 + 1];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 6 + 2];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 6 + 3];
acadoWorkspace.rk_kkk[j + 8] += acadoWorkspace.rk_b[j * 6 + 4];
acadoWorkspace.rk_kkk[j + 10] += acadoWorkspace.rk_b[j * 6 + 5];
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 6; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 48 ]) );
for (j = 0; j < 6; ++j)
{
tmp_index1 = (run1 * 6) + (j);
acadoWorkspace.rk_A[tmp_index1 * 12] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 1] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 2] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 3] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 4] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 5] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 5)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 12) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 12 + 6] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 7] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 8] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 9] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 10] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 12 + 11] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 48) + (j * 8 + 5)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 12) + (j + 6)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 6; ++run1)
{
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_b[i * 6] = - acadoWorkspace.rk_diffsTemp2[(i * 48) + (run1)];
acadoWorkspace.rk_b[i * 6 + 1] = - acadoWorkspace.rk_diffsTemp2[(i * 48) + (run1 + 8)];
acadoWorkspace.rk_b[i * 6 + 2] = - acadoWorkspace.rk_diffsTemp2[(i * 48) + (run1 + 16)];
acadoWorkspace.rk_b[i * 6 + 3] = - acadoWorkspace.rk_diffsTemp2[(i * 48) + (run1 + 24)];
acadoWorkspace.rk_b[i * 6 + 4] = - acadoWorkspace.rk_diffsTemp2[(i * 48) + (run1 + 32)];
acadoWorkspace.rk_b[i * 6 + 5] = - acadoWorkspace.rk_diffsTemp2[(i * 48) + (run1 + 40)];
}
if( 0 == run1 ) {
det = acado_solve_dim12_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim12_perm );
}
 else {
acado_solve_dim12_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim12_perm );
}
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 6];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 6 + 1];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 6 + 2];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 6 + 3];
acadoWorkspace.rk_diffK[i + 8] = acadoWorkspace.rk_b[i * 6 + 4];
acadoWorkspace.rk_diffK[i + 10] = acadoWorkspace.rk_b[i * 6 + 5];
}
for (i = 0; i < 6; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 8) + (run1)] = (i == run1-0);
acadoWorkspace.rk_diffsNew2[(i * 8) + (run1)] += + acadoWorkspace.rk_diffK[i * 2]*(real_t)2.5000000000000001e-02 + acadoWorkspace.rk_diffK[i * 2 + 1]*(real_t)2.5000000000000001e-02;
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 6; ++j)
{
tmp_index1 = (i * 6) + (j);
tmp_index2 = (run1) + (j * 8);
acadoWorkspace.rk_b[tmp_index1] = - acadoWorkspace.rk_diffsTemp2[(i * 48) + (tmp_index2 + 6)];
}
}
acado_solve_dim12_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim12_perm );
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 6];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 6 + 1];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 6 + 2];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 6 + 3];
acadoWorkspace.rk_diffK[i + 8] = acadoWorkspace.rk_b[i * 6 + 4];
acadoWorkspace.rk_diffK[i + 10] = acadoWorkspace.rk_b[i * 6 + 5];
}
for (i = 0; i < 6; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 8) + (run1 + 6)] = + acadoWorkspace.rk_diffK[i * 2]*(real_t)2.5000000000000001e-02 + acadoWorkspace.rk_diffK[i * 2 + 1]*(real_t)2.5000000000000001e-02;
}
}
rk_eta[0] += + acadoWorkspace.rk_kkk[0]*(real_t)2.5000000000000001e-02 + acadoWorkspace.rk_kkk[1]*(real_t)2.5000000000000001e-02;
rk_eta[1] += + acadoWorkspace.rk_kkk[2]*(real_t)2.5000000000000001e-02 + acadoWorkspace.rk_kkk[3]*(real_t)2.5000000000000001e-02;
rk_eta[2] += + acadoWorkspace.rk_kkk[4]*(real_t)2.5000000000000001e-02 + acadoWorkspace.rk_kkk[5]*(real_t)2.5000000000000001e-02;
rk_eta[3] += + acadoWorkspace.rk_kkk[6]*(real_t)2.5000000000000001e-02 + acadoWorkspace.rk_kkk[7]*(real_t)2.5000000000000001e-02;
rk_eta[4] += + acadoWorkspace.rk_kkk[8]*(real_t)2.5000000000000001e-02 + acadoWorkspace.rk_kkk[9]*(real_t)2.5000000000000001e-02;
rk_eta[5] += + acadoWorkspace.rk_kkk[10]*(real_t)2.5000000000000001e-02 + acadoWorkspace.rk_kkk[11]*(real_t)2.5000000000000001e-02;
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
acadoWorkspace.rk_ttt += 5.0000000000000000e-01;
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



