/*
FORCES - Fast interior point code generation for multistage problems.
Copyright (C) 2011-12 Alexander Domahidi [domahidi@control.ee.ethz.ch],
Automatic Control Laboratory, ETH Zurich.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "myMPC.h"

/* for square root */
#include <math.h> 

/* SYSTEM INCLUDES FOR PRINTING ---------------------------------------- */
#ifndef USEMEXPRINTS
#include <stdio.h>
#define PRINTTEXT printf
#else
#include "mex.h"
#define PRINTTEXT mexPrintf
#endif

/* LINEAR ALGEBRA LIBRARY ---------------------------------------------- */
/*
 * Initializes a vector of length 248 with a value.
 */
void myMPC_LA_INITIALIZEVECTOR_248(myMPC_FLOAT* vec, myMPC_FLOAT value)
{
	int i;
	for( i=0; i<248; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 200 with a value.
 */
void myMPC_LA_INITIALIZEVECTOR_200(myMPC_FLOAT* vec, myMPC_FLOAT value)
{
	int i;
	for( i=0; i<200; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 496 with a value.
 */
void myMPC_LA_INITIALIZEVECTOR_496(myMPC_FLOAT* vec, myMPC_FLOAT value)
{
	int i;
	for( i=0; i<496; i++ )
	{
		vec[i] = value;
	}
}


/* 
 * Calculates a dot product and adds it to a variable: z += x'*y; 
 * This function is for vectors of length 496.
 */
void myMPC_LA_DOTACC_496(myMPC_FLOAT *x, myMPC_FLOAT *y, myMPC_FLOAT *z)
{
	int i;
	for( i=0; i<496; i++ ){
		*z += x[i]*y[i];
	}
}


/*
 * Calculates the gradient and the value for a quadratic function 0.5*z'*H*z + f'*z
 *
 * INPUTS:     H  - Symmetric Hessian, dense matrix of size [10 x 10]
 *             f  - column vector of size 10
 *             z  - column vector of size 10
 *
 * OUTPUTS: grad  - gradient at z (= H*z + f), column vector of size 10
 *          value <-- value + 0.5*z'*H*z + f'*z (value will be modified)
 */
void myMPC_LA_DENSE_QUADFCN_10(myMPC_FLOAT* H, myMPC_FLOAT* f, myMPC_FLOAT* z, myMPC_FLOAT* grad, myMPC_FLOAT* value)
{
	int i;
	int j;
	int k = 0;
	myMPC_FLOAT hz;	
	for( i=0; i<10; i++){
		hz = 0;
		for( j=0; j<10; j++ )
		{
			hz += H[k++]*z[j];
		}
		grad[i] = hz + f[i];
		*value += 0.5*hz*z[i] + f[i]*z[i];
	}
}


/*
 * Calculates the gradient and the value for a quadratic function 0.5*z'*H*z + f'*z
 *
 * INPUTS:     H  - Symmetric Hessian, dense matrix of size [8 x 8]
 *             f  - column vector of size 8
 *             z  - column vector of size 8
 *
 * OUTPUTS: grad  - gradient at z (= H*z + f), column vector of size 8
 *          value <-- value + 0.5*z'*H*z + f'*z (value will be modified)
 */
void myMPC_LA_DENSE_QUADFCN_8(myMPC_FLOAT* H, myMPC_FLOAT* f, myMPC_FLOAT* z, myMPC_FLOAT* grad, myMPC_FLOAT* value)
{
	int i;
	int j;
	int k = 0;
	myMPC_FLOAT hz;	
	for( i=0; i<8; i++){
		hz = 0;
		for( j=0; j<8; j++ )
		{
			hz += H[k++]*z[j];
		}
		grad[i] = hz + f[i];
		*value += 0.5*hz*z[i] + f[i]*z[i];
	}
}


/*
 * Vector subtraction and addition.
 *	 Input: five vectors t, tidx, u, v, w and two scalars z and r
 *	 Output: y = t(tidx) - u + w
 *           z = z - v'*x;
 *           r = max([norm(y,inf), z]);
 * for vectors of length 10. Output z is of course scalar.
 */
void myMPC_LA_VSUBADD3_10(myMPC_FLOAT* t, myMPC_FLOAT* u, int* uidx, myMPC_FLOAT* v, myMPC_FLOAT* w, myMPC_FLOAT* y, myMPC_FLOAT* z, myMPC_FLOAT* r)
{
	int i;
	myMPC_FLOAT norm = *r;
	myMPC_FLOAT vx = 0;
	myMPC_FLOAT x;
	for( i=0; i<10; i++){
		x = t[i] - u[uidx[i]];
		y[i] = x + w[i];
		vx += v[i]*x;
		if( y[i] > norm ){
			norm = y[i];
		}
		if( -y[i] > norm ){
			norm = -y[i];
		}
	}
	*z -= vx;
	*r = norm;
}


/*
 * Vector subtraction and addition.
 *	 Input: five vectors t, tidx, u, v, w and two scalars z and r
 *	 Output: y = t(tidx) - u + w
 *           z = z - v'*x;
 *           r = max([norm(y,inf), z]);
 * for vectors of length 10. Output z is of course scalar.
 */
void myMPC_LA_VSUBADD2_10(myMPC_FLOAT* t, int* tidx, myMPC_FLOAT* u, myMPC_FLOAT* v, myMPC_FLOAT* w, myMPC_FLOAT* y, myMPC_FLOAT* z, myMPC_FLOAT* r)
{
	int i;
	myMPC_FLOAT norm = *r;
	myMPC_FLOAT vx = 0;
	myMPC_FLOAT x;
	for( i=0; i<10; i++){
		x = t[tidx[i]] - u[i];
		y[i] = x + w[i];
		vx += v[i]*x;
		if( y[i] > norm ){
			norm = y[i];
		}
		if( -y[i] > norm ){
			norm = -y[i];
		}
	}
	*z -= vx;
	*r = norm;
}


/*
 * Vector subtraction and addition.
 *	 Input: five vectors t, tidx, u, v, w and two scalars z and r
 *	 Output: y = t(tidx) - u + w
 *           z = z - v'*x;
 *           r = max([norm(y,inf), z]);
 * for vectors of length 8. Output z is of course scalar.
 */
void myMPC_LA_VSUBADD3_8(myMPC_FLOAT* t, myMPC_FLOAT* u, int* uidx, myMPC_FLOAT* v, myMPC_FLOAT* w, myMPC_FLOAT* y, myMPC_FLOAT* z, myMPC_FLOAT* r)
{
	int i;
	myMPC_FLOAT norm = *r;
	myMPC_FLOAT vx = 0;
	myMPC_FLOAT x;
	for( i=0; i<8; i++){
		x = t[i] - u[uidx[i]];
		y[i] = x + w[i];
		vx += v[i]*x;
		if( y[i] > norm ){
			norm = y[i];
		}
		if( -y[i] > norm ){
			norm = -y[i];
		}
	}
	*z -= vx;
	*r = norm;
}


/*
 * Vector subtraction and addition.
 *	 Input: five vectors t, tidx, u, v, w and two scalars z and r
 *	 Output: y = t(tidx) - u + w
 *           z = z - v'*x;
 *           r = max([norm(y,inf), z]);
 * for vectors of length 8. Output z is of course scalar.
 */
void myMPC_LA_VSUBADD2_8(myMPC_FLOAT* t, int* tidx, myMPC_FLOAT* u, myMPC_FLOAT* v, myMPC_FLOAT* w, myMPC_FLOAT* y, myMPC_FLOAT* z, myMPC_FLOAT* r)
{
	int i;
	myMPC_FLOAT norm = *r;
	myMPC_FLOAT vx = 0;
	myMPC_FLOAT x;
	for( i=0; i<8; i++){
		x = t[tidx[i]] - u[i];
		y[i] = x + w[i];
		vx += v[i]*x;
		if( y[i] > norm ){
			norm = y[i];
		}
		if( -y[i] > norm ){
			norm = -y[i];
		}
	}
	*z -= vx;
	*r = norm;
}


/*
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 10
 * Returns also L/S, a value that is often used elsewhere.
 */
void myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_FLOAT *lu, myMPC_FLOAT *su, myMPC_FLOAT *ru, myMPC_FLOAT *ll, myMPC_FLOAT *sl, myMPC_FLOAT *rl, int* lbIdx, int* ubIdx, myMPC_FLOAT *grad, myMPC_FLOAT *lubysu, myMPC_FLOAT *llbysl)
{
	int i;
	for( i=0; i<10; i++ ){
		grad[i] = 0;
	}
	for( i=0; i<10; i++ ){		
		llbysl[i] = ll[i] / sl[i];
		grad[lbIdx[i]] -= llbysl[i]*rl[i];
	}
	for( i=0; i<10; i++ ){
		lubysu[i] = lu[i] / su[i];
		grad[ubIdx[i]] += lubysu[i]*ru[i];
	}
}


/*
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 8
 * Returns also L/S, a value that is often used elsewhere.
 */
void myMPC_LA_INEQ_B_GRAD_8_8_8(myMPC_FLOAT *lu, myMPC_FLOAT *su, myMPC_FLOAT *ru, myMPC_FLOAT *ll, myMPC_FLOAT *sl, myMPC_FLOAT *rl, int* lbIdx, int* ubIdx, myMPC_FLOAT *grad, myMPC_FLOAT *lubysu, myMPC_FLOAT *llbysl)
{
	int i;
	for( i=0; i<8; i++ ){
		grad[i] = 0;
	}
	for( i=0; i<8; i++ ){		
		llbysl[i] = ll[i] / sl[i];
		grad[lbIdx[i]] -= llbysl[i]*rl[i];
	}
	for( i=0; i<8; i++ ){
		lubysu[i] = lu[i] / su[i];
		grad[ubIdx[i]] += lubysu[i]*ru[i];
	}
}


/* 
 * Computes r = A*x + B*u - b
 * and      y = max([norm(r,inf), y])
 * and      z -= l'*r
 * where A is stored in column major format
 */
void myMPC_LA_DENSE_2MVMSUB_16_10_10(myMPC_FLOAT *A, myMPC_FLOAT *x, myMPC_FLOAT *B, myMPC_FLOAT *u, myMPC_FLOAT *b, myMPC_FLOAT *l, myMPC_FLOAT *r, myMPC_FLOAT *z, myMPC_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;
	myMPC_FLOAT AxBu[16];
	myMPC_FLOAT norm = *y;
	myMPC_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<16; i++ ){
		AxBu[i] = A[k++]*x[0] + B[m++]*u[0];
	}	
	for( j=1; j<10; j++ ){		
		for( i=0; i<16; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<10; n++ ){
		for( i=0; i<16; i++ ){
			AxBu[i] += B[m++]*u[n];
		}		
	}

	for( i=0; i<16; i++ ){
		r[i] = AxBu[i] - b[i];
		lr += l[i]*r[i];
		if( r[i] > norm ){
			norm = r[i];
		}
		if( -r[i] > norm ){
			norm = -r[i];
		}
	}
	*y = norm;
	*z -= lr;
}


/* 
 * Computes r = A*x + B*u - b
 * and      y = max([norm(r,inf), y])
 * and      z -= l'*r
 * where A is stored in column major format
 */
void myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_FLOAT *A, myMPC_FLOAT *x, myMPC_FLOAT *B, myMPC_FLOAT *u, myMPC_FLOAT *b, myMPC_FLOAT *l, myMPC_FLOAT *r, myMPC_FLOAT *z, myMPC_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;
	myMPC_FLOAT AxBu[8];
	myMPC_FLOAT norm = *y;
	myMPC_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<8; i++ ){
		AxBu[i] = A[k++]*x[0] + B[m++]*u[0];
	}	
	for( j=1; j<10; j++ ){		
		for( i=0; i<8; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<10; n++ ){
		for( i=0; i<8; i++ ){
			AxBu[i] += B[m++]*u[n];
		}		
	}

	for( i=0; i<8; i++ ){
		r[i] = AxBu[i] - b[i];
		lr += l[i]*r[i];
		if( r[i] > norm ){
			norm = r[i];
		}
		if( -r[i] > norm ){
			norm = -r[i];
		}
	}
	*y = norm;
	*z -= lr;
}


/* 
 * Computes r = A*x + B*u - b
 * and      y = max([norm(r,inf), y])
 * and      z -= l'*r
 * where A is stored in column major format
 */
void myMPC_LA_DENSE_2MVMSUB_8_10_8(myMPC_FLOAT *A, myMPC_FLOAT *x, myMPC_FLOAT *B, myMPC_FLOAT *u, myMPC_FLOAT *b, myMPC_FLOAT *l, myMPC_FLOAT *r, myMPC_FLOAT *z, myMPC_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;
	myMPC_FLOAT AxBu[8];
	myMPC_FLOAT norm = *y;
	myMPC_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<8; i++ ){
		AxBu[i] = A[k++]*x[0] + B[m++]*u[0];
	}	
	for( j=1; j<10; j++ ){		
		for( i=0; i<8; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<8; n++ ){
		for( i=0; i<8; i++ ){
			AxBu[i] += B[m++]*u[n];
		}		
	}

	for( i=0; i<8; i++ ){
		r[i] = AxBu[i] - b[i];
		lr += l[i]*r[i];
		if( r[i] > norm ){
			norm = r[i];
		}
		if( -r[i] > norm ){
			norm = -r[i];
		}
	}
	*y = norm;
	*z -= lr;
}


/*
 * Matrix vector multiplication y = M'*x where M is of size [16 x 10]
 * and stored in column major format. Note the transpose of M!
 */
void myMPC_LA_DENSE_MTVM_16_10(myMPC_FLOAT *M, myMPC_FLOAT *x, myMPC_FLOAT *y)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<10; i++ ){
		y[i] = 0;
		for( j=0; j<16; j++ ){
			y[i] += M[k++]*x[j];
		}
	}
}


/*
 * Matrix vector multiplication z = A'*x + B'*y 
 * where A is of size [8 x 10]
 * and B is of size [16 x 10]
 * and stored in column major format. Note the transposes of A and B!
 */
void myMPC_LA_DENSE_MTVM2_8_10_16(myMPC_FLOAT *A, myMPC_FLOAT *x, myMPC_FLOAT *B, myMPC_FLOAT *y, myMPC_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	int n;
	int m = 0;
	for( i=0; i<10; i++ ){
		z[i] = 0;
		for( j=0; j<8; j++ ){
			z[i] += A[k++]*x[j];
		}
		for( n=0; n<16; n++ ){
			z[i] += B[m++]*y[n];
		}
	}
}


/*
 * Matrix vector multiplication z = A'*x + B'*y 
 * where A is of size [8 x 10]
 * and B is of size [8 x 10]
 * and stored in column major format. Note the transposes of A and B!
 */
void myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_FLOAT *A, myMPC_FLOAT *x, myMPC_FLOAT *B, myMPC_FLOAT *y, myMPC_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	int n;
	int m = 0;
	for( i=0; i<10; i++ ){
		z[i] = 0;
		for( j=0; j<8; j++ ){
			z[i] += A[k++]*x[j];
		}
		for( n=0; n<8; n++ ){
			z[i] += B[m++]*y[n];
		}
	}
}


/*
 * Matrix vector multiplication y = M'*x where M is of size [8 x 8]
 * and stored in column major format. Note the transpose of M!
 */
void myMPC_LA_DENSE_MTVM_8_8(myMPC_FLOAT *M, myMPC_FLOAT *x, myMPC_FLOAT *y)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<8; i++ ){
		y[i] = 0;
		for( j=0; j<8; j++ ){
			y[i] += M[k++]*x[j];
		}
	}
}


/*
 * Addition of three vectors  z = u + w + v
 * of length 10.
 */
void myMPC_LA_VVADD3_10(myMPC_FLOAT *u, myMPC_FLOAT *v, myMPC_FLOAT *w, myMPC_FLOAT *z)
{
	int i;
	for( i=0; i<10; i++ ){
		z[i] = u[i] + v[i] + w[i];
	}
}


/*
 * Addition of three vectors  z = u + w + v
 * of length 8.
 */
void myMPC_LA_VVADD3_8(myMPC_FLOAT *u, myMPC_FLOAT *v, myMPC_FLOAT *w, myMPC_FLOAT *z)
{
	int i;
	for( i=0; i<8; i++ ){
		z[i] = u[i] + v[i] + w[i];
	}
}


/*
 * Special function to compute the Dense positive definite 
 * augmented Hessian for block size 10.
 *
 * Inputs: - H = dense cost Hessian in column major storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = H + diag(llbysl) + diag(lubysu)
 * where Phi is stored in lower triangular row major format
 */
void myMPC_LA_INEQ_DENSE_HESS_10_10_10(myMPC_FLOAT *H, myMPC_FLOAT *llbysl, int* lbIdx, myMPC_FLOAT *lubysu, int* ubIdx, myMPC_FLOAT *Phi)
{
	int i;
	int j;
	int k = 0;
	
	/* copy lower triangular part of H into PHI */
	for( i=0; i<10; i++ ){
		for( j=0; j<=i; j++ ){
			Phi[k++] = H[i*10+j];
		}		
	}

	/* add llbysl onto Phi where necessary */
	for( i=0; i<10; i++ ){
		j = lbIdx[i];
		Phi[((j+1)*(j+2))/2-1] += llbysl[i];
	}

	/* add lubysu onto Phi where necessary */
	for( i=0; i<10; i++){
		j = ubIdx[i];
		Phi[((j+1)*(j+2))/2-1] +=  lubysu[i];
	}

}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 10.
 */
void myMPC_LA_DENSE_CHOL2_10(myMPC_FLOAT *A)
{
    int i, j, k, ii, jj, di, dj;
    myMPC_FLOAT l;
    myMPC_FLOAT Mii;
    
	ii=0; di=0;
    for( i=0; i<10; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += A[ii+k]*A[ii+k];
        }        
        
        Mii = A[ii+i] - l;
        
        if( Mii < 1e-13 ){
#if myMPC_SET_PRINTLEVEL > 0
#ifdef PRINTNUMERICALWARNINGS
            PRINTTEXT("WARNING: pivot in Cholesky factorization close to 0, regularizing...\n");
#endif
#endif
            Mii = 4e-4;
        }
            
        A[ii+i] = sqrt(Mii);        

		jj = ((i+1)*(i+2))/2; dj = i+1;
        for( j=i+1; j<10; j++ ){
            l = 0;            
            for( k=0; k<i; k++ ){
                l += A[jj+k]*A[ii+k];
            }
            A[jj+i] = (A[jj+i] - l)/A[ii+i];            
			jj += ++dj;
        }
		ii += ++di;
    }
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [16 x 10],
 * B is given and of size [16 x 10], L is a lower tri-
 * angular matrix of size 10 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void myMPC_LA_DENSE_MATRIXFORWARDSUB_16_10(myMPC_FLOAT *L, myMPC_FLOAT *B, myMPC_FLOAT *A)
{
    int i,j,k,ii,di;
    myMPC_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<10; j++ ){        
        for( i=0; i<16; i++ ){
            a = B[j*16+i];
            for( k=0; k<j; k++ ){
                a -= A[k*16+i]*L[ii+k];
            }
            A[j*16+i] = a/L[ii+j];
        }
        ii += ++di;
    }
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [16 x 10] in column
 * storage format, and B is of size [16 x 10] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void myMPC_LA_DENSE_MMT2_16_10_10(myMPC_FLOAT *A, myMPC_FLOAT *B, myMPC_FLOAT *L)
{
    int i, j, k, ii, di;
    myMPC_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<16; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<10; k++ ){
                ltemp += A[k*16+i]*A[k*16+j];
            }			
			for( k=0; k<10; k++ ){
                ltemp += B[k*16+i]*B[k*16+j];
            }
            L[ii+j] = ltemp;
        }
        ii += ++di;
    }
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 16 and outputting
 * the Cholesky factor to matrix L in lower triangular format.
 */
void myMPC_LA_DENSE_CHOL_16(myMPC_FLOAT *A, myMPC_FLOAT *L)
{
    int i, j, k, ii, jj, di, dj;
    myMPC_FLOAT l;
    myMPC_FLOAT Mii;

	/* copy A to L first and then operate on L */
	/* COULD BE OPTIMIZED */
	ii=0; di=0;
	for( i=0; i<16; i++ ){
		for( j=0; j<=i; j++ ){
			L[ii+j] = A[ii+j];
		}
		ii += ++di;
	}    
	
	/* factor L */
	ii=0; di=0;
    for( i=0; i<16; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += L[ii+k]*L[ii+k];
        }        
        
        Mii = L[ii+i] - l;
        
        if( Mii < 1e-13 ){
#if myMPC_SET_PRINTLEVEL > 0
#ifdef PRINTNUMERICALWARNINGS
            PRINTTEXT("WARNING: pivot in Cholesky factorization close to 0, regularizing...\n");
#endif
#endif
            Mii = 4e-4;
        }
            
        L[ii+i] = sqrt(Mii);        

		jj = ((i+1)*(i+2))/2; dj = i+1;
        for( j=i+1; j<16; j++ ){
            l = 0;            
            for( k=0; k<i; k++ ){
                l += L[jj+k]*L[ii+k];
            }
            L[jj+i] = (L[jj+i] - l)/L[ii+i];            
			jj += ++dj;
        }
		ii += ++di;
    }	
}


/**
 * Forward substitution to solve L*y = b where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * The dimensions involved are 10.
 */
void myMPC_LA_DENSE_FORWARDSUB_10(myMPC_FLOAT *L, myMPC_FLOAT *b, myMPC_FLOAT *y)
{
    int i,j,ii,di;
    myMPC_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<10; i++ ){
        yel = b[i];        
        for( j=0; j<i; j++ ){
            yel -= y[j]*L[ii+j];
        }
        y[i] = yel / L[ii+i];
        ii += ++di;
    }
}


/* 
 * Computes r = b - A*x - B*u
 * where A an B are stored in column major format
 */
void myMPC_LA_DENSE_2MVMSUB2_16_10_10(myMPC_FLOAT *A, myMPC_FLOAT *x, myMPC_FLOAT *B, myMPC_FLOAT *u, myMPC_FLOAT *b, myMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<16; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<10; j++ ){		
		for( i=0; i<16; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<10; n++ ){
		for( i=0; i<16; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Forward substitution to solve L*y = b where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * The dimensions involved are 16.
 */
void myMPC_LA_DENSE_FORWARDSUB_16(myMPC_FLOAT *L, myMPC_FLOAT *b, myMPC_FLOAT *y)
{
    int i,j,ii,di;
    myMPC_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<16; i++ ){
        yel = b[i];        
        for( j=0; j<i; j++ ){
            yel -= y[j]*L[ii+j];
        }
        y[i] = yel / L[ii+i];
        ii += ++di;
    }
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [8 x 10],
 * B is given and of size [8 x 10], L is a lower tri-
 * angular matrix of size 10 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_FLOAT *L, myMPC_FLOAT *B, myMPC_FLOAT *A)
{
    int i,j,k,ii,di;
    myMPC_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<10; j++ ){        
        for( i=0; i<8; i++ ){
            a = B[j*8+i];
            for( k=0; k<j; k++ ){
                a -= A[k*8+i]*L[ii+k];
            }
            A[j*8+i] = a/L[ii+j];
        }
        ii += ++di;
    }
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [8 x 10] in column
 * storage format, and B is of size [8 x 10] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void myMPC_LA_DENSE_MMT2_8_10_10(myMPC_FLOAT *A, myMPC_FLOAT *B, myMPC_FLOAT *L)
{
    int i, j, k, ii, di;
    myMPC_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<8; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<10; k++ ){
                ltemp += A[k*8+i]*A[k*8+j];
            }			
			for( k=0; k<10; k++ ){
                ltemp += B[k*8+i]*B[k*8+j];
            }
            L[ii+j] = ltemp;
        }
        ii += ++di;
    }
}


/**
 * Compute C = A*B' where 
 *
 *	size(A) = [16 x 10]
 *  size(B) = [8 x 10]
 * 
 * and all matrices are stored in column major format.
 *
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE.  
 * 
 */
void myMPC_LA_DENSE_MMTM_16_10_8(myMPC_FLOAT *A, myMPC_FLOAT *B, myMPC_FLOAT *C)
{
    int i, j, k;
    myMPC_FLOAT temp;
    
    for( i=0; i<16; i++ ){        
        for( j=0; j<8; j++ ){
            temp = 0; 
            for( k=0; k<10; k++ ){
                temp += A[k*16+i]*B[k*8+j];
            }						
            C[j*16+i] = temp;
        }
    }
}


/**
 * Forward substitution for the matrix equation A*L' = B'
 * where A is to be computed and is of size [8 x 16],
 * B is given and of size [8 x 16], L is a lower tri-
 * angular matrix of size 16 stored in lower triangular 
 * storage format. Note the transpose of L AND B!
 *
 * Result: A in column major storage format.
 *
 */
void myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_16(myMPC_FLOAT *L, myMPC_FLOAT *B, myMPC_FLOAT *A)
{
    int i,j,k,ii,di;
    myMPC_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<16; j++ ){        
        for( i=0; i<8; i++ ){
            a = B[i*16+j];
            for( k=0; k<j; k++ ){
                a -= A[k*8+i]*L[ii+k];
            }            
			A[j*8+i] = a/L[ii+j];
        }
        ii += ++di;
    }
}


/**
 * Compute L = L - A*A', where L is lower triangular of size 8
 * and A is a dense matrix of size [8 x 16] in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void myMPC_LA_DENSE_MMTSUB_8_16(myMPC_FLOAT *A, myMPC_FLOAT *L)
{
    int i, j, k, ii, di;
    myMPC_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<8; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<16; k++ ){
                ltemp += A[k*8+i]*A[k*8+j];
            }						
            L[ii+j] -= ltemp;
        }
        ii += ++di;
    }
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 8 and outputting
 * the Cholesky factor to matrix L in lower triangular format.
 */
void myMPC_LA_DENSE_CHOL_8(myMPC_FLOAT *A, myMPC_FLOAT *L)
{
    int i, j, k, ii, jj, di, dj;
    myMPC_FLOAT l;
    myMPC_FLOAT Mii;

	/* copy A to L first and then operate on L */
	/* COULD BE OPTIMIZED */
	ii=0; di=0;
	for( i=0; i<8; i++ ){
		for( j=0; j<=i; j++ ){
			L[ii+j] = A[ii+j];
		}
		ii += ++di;
	}    
	
	/* factor L */
	ii=0; di=0;
    for( i=0; i<8; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += L[ii+k]*L[ii+k];
        }        
        
        Mii = L[ii+i] - l;
        
        if( Mii < 1e-13 ){
#if myMPC_SET_PRINTLEVEL > 0
#ifdef PRINTNUMERICALWARNINGS
            PRINTTEXT("WARNING: pivot in Cholesky factorization close to 0, regularizing...\n");
#endif
#endif
            Mii = 4e-4;
        }
            
        L[ii+i] = sqrt(Mii);        

		jj = ((i+1)*(i+2))/2; dj = i+1;
        for( j=i+1; j<8; j++ ){
            l = 0;            
            for( k=0; k<i; k++ ){
                l += L[jj+k]*L[ii+k];
            }
            L[jj+i] = (L[jj+i] - l)/L[ii+i];            
			jj += ++dj;
        }
		ii += ++di;
    }	
}


/* 
 * Computes r = b - A*x - B*u
 * where A an B are stored in column major format
 */
void myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_FLOAT *A, myMPC_FLOAT *x, myMPC_FLOAT *B, myMPC_FLOAT *u, myMPC_FLOAT *b, myMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<8; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<10; j++ ){		
		for( i=0; i<8; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<10; n++ ){
		for( i=0; i<8; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/* 
 * Computes r = b - A*x
 * where A is stored in column major format
 */
void myMPC_LA_DENSE_MVMSUB1_8_16(myMPC_FLOAT *A, myMPC_FLOAT *x, myMPC_FLOAT *b, myMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<8; i++ ){
		r[i] = b[i] - A[k++]*x[0];
	}	
	for( j=1; j<16; j++ ){		
		for( i=0; i<8; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/**
 * Forward substitution to solve L*y = b where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * The dimensions involved are 8.
 */
void myMPC_LA_DENSE_FORWARDSUB_8(myMPC_FLOAT *L, myMPC_FLOAT *b, myMPC_FLOAT *y)
{
    int i,j,ii,di;
    myMPC_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<8; i++ ){
        yel = b[i];        
        for( j=0; j<i; j++ ){
            yel -= y[j]*L[ii+j];
        }
        y[i] = yel / L[ii+i];
        ii += ++di;
    }
}


/**
 * Compute C = A*B' where 
 *
 *	size(A) = [8 x 10]
 *  size(B) = [8 x 10]
 * 
 * and all matrices are stored in column major format.
 *
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE.  
 * 
 */
void myMPC_LA_DENSE_MMTM_8_10_8(myMPC_FLOAT *A, myMPC_FLOAT *B, myMPC_FLOAT *C)
{
    int i, j, k;
    myMPC_FLOAT temp;
    
    for( i=0; i<8; i++ ){        
        for( j=0; j<8; j++ ){
            temp = 0; 
            for( k=0; k<10; k++ ){
                temp += A[k*8+i]*B[k*8+j];
            }						
            C[j*8+i] = temp;
        }
    }
}


/**
 * Forward substitution for the matrix equation A*L' = B'
 * where A is to be computed and is of size [8 x 8],
 * B is given and of size [8 x 8], L is a lower tri-
 * angular matrix of size 8 stored in lower triangular 
 * storage format. Note the transpose of L AND B!
 *
 * Result: A in column major storage format.
 *
 */
void myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_FLOAT *L, myMPC_FLOAT *B, myMPC_FLOAT *A)
{
    int i,j,k,ii,di;
    myMPC_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<8; j++ ){        
        for( i=0; i<8; i++ ){
            a = B[i*8+j];
            for( k=0; k<j; k++ ){
                a -= A[k*8+i]*L[ii+k];
            }            
			A[j*8+i] = a/L[ii+j];
        }
        ii += ++di;
    }
}


/**
 * Compute L = L - A*A', where L is lower triangular of size 8
 * and A is a dense matrix of size [8 x 8] in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void myMPC_LA_DENSE_MMTSUB_8_8(myMPC_FLOAT *A, myMPC_FLOAT *L)
{
    int i, j, k, ii, di;
    myMPC_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<8; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<8; k++ ){
                ltemp += A[k*8+i]*A[k*8+j];
            }						
            L[ii+j] -= ltemp;
        }
        ii += ++di;
    }
}


/* 
 * Computes r = b - A*x
 * where A is stored in column major format
 */
void myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_FLOAT *A, myMPC_FLOAT *x, myMPC_FLOAT *b, myMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<8; i++ ){
		r[i] = b[i] - A[k++]*x[0];
	}	
	for( j=1; j<8; j++ ){		
		for( i=0; i<8; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Special function to compute the Dense positive definite 
 * augmented Hessian for block size 8.
 *
 * Inputs: - H = dense cost Hessian in column major storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = H + diag(llbysl) + diag(lubysu)
 * where Phi is stored in lower triangular row major format
 */
void myMPC_LA_INEQ_DENSE_HESS_8_8_8(myMPC_FLOAT *H, myMPC_FLOAT *llbysl, int* lbIdx, myMPC_FLOAT *lubysu, int* ubIdx, myMPC_FLOAT *Phi)
{
	int i;
	int j;
	int k = 0;
	
	/* copy lower triangular part of H into PHI */
	for( i=0; i<8; i++ ){
		for( j=0; j<=i; j++ ){
			Phi[k++] = H[i*8+j];
		}		
	}

	/* add llbysl onto Phi where necessary */
	for( i=0; i<8; i++ ){
		j = lbIdx[i];
		Phi[((j+1)*(j+2))/2-1] += llbysl[i];
	}

	/* add lubysu onto Phi where necessary */
	for( i=0; i<8; i++){
		j = ubIdx[i];
		Phi[((j+1)*(j+2))/2-1] +=  lubysu[i];
	}

}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 8.
 */
void myMPC_LA_DENSE_CHOL2_8(myMPC_FLOAT *A)
{
    int i, j, k, ii, jj, di, dj;
    myMPC_FLOAT l;
    myMPC_FLOAT Mii;
    
	ii=0; di=0;
    for( i=0; i<8; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += A[ii+k]*A[ii+k];
        }        
        
        Mii = A[ii+i] - l;
        
        if( Mii < 1e-13 ){
#if myMPC_SET_PRINTLEVEL > 0
#ifdef PRINTNUMERICALWARNINGS
            PRINTTEXT("WARNING: pivot in Cholesky factorization close to 0, regularizing...\n");
#endif
#endif
            Mii = 4e-4;
        }
            
        A[ii+i] = sqrt(Mii);        

		jj = ((i+1)*(i+2))/2; dj = i+1;
        for( j=i+1; j<8; j++ ){
            l = 0;            
            for( k=0; k<i; k++ ){
                l += A[jj+k]*A[ii+k];
            }
            A[jj+i] = (A[jj+i] - l)/A[ii+i];            
			jj += ++dj;
        }
		ii += ++di;
    }
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [8 x 8],
 * B is given and of size [8 x 8], L is a lower tri-
 * angular matrix of size 8 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void myMPC_LA_DENSE_MATRIXFORWARDSUB_8_8(myMPC_FLOAT *L, myMPC_FLOAT *B, myMPC_FLOAT *A)
{
    int i,j,k,ii,di;
    myMPC_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<8; j++ ){        
        for( i=0; i<8; i++ ){
            a = B[j*8+i];
            for( k=0; k<j; k++ ){
                a -= A[k*8+i]*L[ii+k];
            }
            A[j*8+i] = a/L[ii+j];
        }
        ii += ++di;
    }
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [8 x 10] in column
 * storage format, and B is of size [8 x 8] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void myMPC_LA_DENSE_MMT2_8_10_8(myMPC_FLOAT *A, myMPC_FLOAT *B, myMPC_FLOAT *L)
{
    int i, j, k, ii, di;
    myMPC_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<8; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<10; k++ ){
                ltemp += A[k*8+i]*A[k*8+j];
            }			
			for( k=0; k<8; k++ ){
                ltemp += B[k*8+i]*B[k*8+j];
            }
            L[ii+j] = ltemp;
        }
        ii += ++di;
    }
}


/* 
 * Computes r = b - A*x - B*u
 * where A an B are stored in column major format
 */
void myMPC_LA_DENSE_2MVMSUB2_8_10_8(myMPC_FLOAT *A, myMPC_FLOAT *x, myMPC_FLOAT *B, myMPC_FLOAT *u, myMPC_FLOAT *b, myMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<8; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<10; j++ ){		
		for( i=0; i<8; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<8; n++ ){
		for( i=0; i<8; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Backward Substitution to solve L^T*x = y where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * All involved dimensions are 8.
 */
void myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_FLOAT *L, myMPC_FLOAT *y, myMPC_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    myMPC_FLOAT xel;    
	int start = 28;
    
    /* now solve L^T*x = y by backward substitution */
    ii = start; di = 7;
    for( i=7; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 7;
        for( j=7; j>i; j-- ){
            xel -= x[j]*L[jj+i];
            jj -= dj--;
        }
        x[i] = xel / L[ii+i];
        ii -= di--;
    }
}


/*
 * Matrix vector multiplication y = b - M'*x where M is of size [8 x 8]
 * and stored in column major format. Note the transpose of M!
 */
void myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_FLOAT *A, myMPC_FLOAT *x, myMPC_FLOAT *b, myMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<8; i++ ){
		r[i] = b[i];
		for( j=0; j<8; j++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Matrix vector multiplication y = b - M'*x where M is of size [8 x 16]
 * and stored in column major format. Note the transpose of M!
 */
void myMPC_LA_DENSE_MTVMSUB_8_16(myMPC_FLOAT *A, myMPC_FLOAT *x, myMPC_FLOAT *b, myMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<16; i++ ){
		r[i] = b[i];
		for( j=0; j<8; j++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/**
 * Backward Substitution to solve L^T*x = y where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * All involved dimensions are 16.
 */
void myMPC_LA_DENSE_BACKWARDSUB_16(myMPC_FLOAT *L, myMPC_FLOAT *y, myMPC_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    myMPC_FLOAT xel;    
	int start = 120;
    
    /* now solve L^T*x = y by backward substitution */
    ii = start; di = 15;
    for( i=15; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 15;
        for( j=15; j>i; j-- ){
            xel -= x[j]*L[jj+i];
            jj -= dj--;
        }
        x[i] = xel / L[ii+i];
        ii -= di--;
    }
}


/*
 * Vector subtraction z = -x - y for vectors of length 10.
 */
void myMPC_LA_VSUB2_10(myMPC_FLOAT *x, myMPC_FLOAT *y, myMPC_FLOAT *z)
{
	int i;
	for( i=0; i<10; i++){
		z[i] = -x[i] - y[i];
	}
}


/*
 * Vector subtraction z = -x - y for vectors of length 8.
 */
void myMPC_LA_VSUB2_8(myMPC_FLOAT *x, myMPC_FLOAT *y, myMPC_FLOAT *z)
{
	int i;
	for( i=0; i<8; i++){
		z[i] = -x[i] - y[i];
	}
}


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * lower triangular matrix of size 10 in lower triangular
 * storage format.
 */
void myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_FLOAT *L, myMPC_FLOAT *b, myMPC_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    myMPC_FLOAT y[10];
    myMPC_FLOAT yel,xel;
	int start = 45;
            
    /* first solve Ly = b by forward substitution */
     ii = 0; di = 0;
    for( i=0; i<10; i++ ){
        yel = b[i];        
        for( j=0; j<i; j++ ){
            yel -= y[j]*L[ii+j];
        }
        y[i] = yel / L[ii+i];
        ii += ++di;
    }
    
    /* now solve L^T*x = y by backward substitution */
    ii = start; di = 9;
    for( i=9; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 9;
        for( j=9; j>i; j-- ){
            xel -= x[j]*L[jj+i];
            jj -= dj--;
        }
        x[i] = xel / L[ii+i];
        ii -= di--;
    }
}


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * lower triangular matrix of size 8 in lower triangular
 * storage format.
 */
void myMPC_LA_DENSE_FORWARDBACKWARDSUB_8(myMPC_FLOAT *L, myMPC_FLOAT *b, myMPC_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    myMPC_FLOAT y[8];
    myMPC_FLOAT yel,xel;
	int start = 28;
            
    /* first solve Ly = b by forward substitution */
     ii = 0; di = 0;
    for( i=0; i<8; i++ ){
        yel = b[i];        
        for( j=0; j<i; j++ ){
            yel -= y[j]*L[ii+j];
        }
        y[i] = yel / L[ii+i];
        ii += ++di;
    }
    
    /* now solve L^T*x = y by backward substitution */
    ii = start; di = 7;
    for( i=7; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 7;
        for( j=7; j>i; j-- ){
            xel -= x[j]*L[jj+i];
            jj -= dj--;
        }
        x[i] = xel / L[ii+i];
        ii -= di--;
    }
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 10,
 * and x has length 10 and is indexed through yidx.
 */
void myMPC_LA_VSUB_INDEXED_10(myMPC_FLOAT *x, int* xidx, myMPC_FLOAT *y, myMPC_FLOAT *z)
{
	int i;
	for( i=0; i<10; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 10.
 */
void myMPC_LA_VSUB3_10(myMPC_FLOAT *u, myMPC_FLOAT *v, myMPC_FLOAT *w, myMPC_FLOAT *x)
{
	int i;
	for( i=0; i<10; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = -x - y(yidx) where y is of length 10
 * and z, x and yidx are of length 10.
 */
void myMPC_LA_VSUB2_INDEXED_10(myMPC_FLOAT *x, myMPC_FLOAT *y, int* yidx, myMPC_FLOAT *z)
{
	int i;
	for( i=0; i<10; i++){
		z[i] = -x[i] - y[yidx[i]];
	}
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 8,
 * and x has length 8 and is indexed through yidx.
 */
void myMPC_LA_VSUB_INDEXED_8(myMPC_FLOAT *x, int* xidx, myMPC_FLOAT *y, myMPC_FLOAT *z)
{
	int i;
	for( i=0; i<8; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 8.
 */
void myMPC_LA_VSUB3_8(myMPC_FLOAT *u, myMPC_FLOAT *v, myMPC_FLOAT *w, myMPC_FLOAT *x)
{
	int i;
	for( i=0; i<8; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = -x - y(yidx) where y is of length 8
 * and z, x and yidx are of length 8.
 */
void myMPC_LA_VSUB2_INDEXED_8(myMPC_FLOAT *x, myMPC_FLOAT *y, int* yidx, myMPC_FLOAT *z)
{
	int i;
	for( i=0; i<8; i++){
		z[i] = -x[i] - y[yidx[i]];
	}
}


/**
 * Backtracking line search.
 * 
 * First determine the maximum line length by a feasibility line
 * search, i.e. a ~= argmax{ a \in [0...1] s.t. l+a*dl >= 0 and s+a*ds >= 0}.
 *
 * The function returns either the number of iterations or exits the error code
 * myMPC_NOPROGRESS (should be negative).
 */
int myMPC_LINESEARCH_BACKTRACKING_AFFINE(myMPC_FLOAT *l, myMPC_FLOAT *s, myMPC_FLOAT *dl, myMPC_FLOAT *ds, myMPC_FLOAT *a, myMPC_FLOAT *mu_aff)
{
    int i;
	int lsIt=1;    
    myMPC_FLOAT dltemp;
    myMPC_FLOAT dstemp;
    myMPC_FLOAT mya = 1.0;
    myMPC_FLOAT mymu;
        
    while( 1 ){                        

        /* 
         * Compute both snew and wnew together.
         * We compute also mu_affine along the way here, as the
         * values might be in registers, so it should be cheaper.
         */
        mymu = 0;
        for( i=0; i<496; i++ ){
            dltemp = l[i] + mya*dl[i];
            dstemp = s[i] + mya*ds[i];
            if( dltemp < 0 || dstemp < 0 ){
                lsIt++;
                break;
            } else {                
                mymu += dstemp*dltemp;
            }
        }
        
        /* 
         * If no early termination of the for-loop above occurred, we
         * found the required value of a and we can quit the while loop.
         */
        if( i == 496 ){
            break;
        } else {
            mya *= myMPC_SET_LS_SCALE_AFF;
            if( mya < myMPC_SET_LS_MINSTEP ){
                return myMPC_NOPROGRESS;
            }
        }
    }
    
    /* return new values and iteration counter */
    *a = mya;
    *mu_aff = mymu / (myMPC_FLOAT)496;
    return lsIt;
}


/*
 * Vector subtraction x = u.*v - a where a is a scalar
*  and x,u,v are vectors of length 496.
 */
void myMPC_LA_VSUB5_496(myMPC_FLOAT *u, myMPC_FLOAT *v, myMPC_FLOAT a, myMPC_FLOAT *x)
{
	int i;
	for( i=0; i<496; i++){
		x[i] = u[i]*v[i] - a;
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 10,
 * u, su, uidx are of length 10 and v, sv, vidx are of length 10.
 */
void myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_FLOAT *u, myMPC_FLOAT *su, int* uidx, myMPC_FLOAT *v, myMPC_FLOAT *sv, int* vidx, myMPC_FLOAT *x)
{
	int i;
	for( i=0; i<10; i++ ){
		x[i] = 0;
	}
	for( i=0; i<10; i++){
		x[uidx[i]] += u[i]/su[i];
	}
	for( i=0; i<10; i++){
		x[vidx[i]] -= v[i]/sv[i];
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void myMPC_LA_DENSE_2MVMADD_16_10_10(myMPC_FLOAT *A, myMPC_FLOAT *x, myMPC_FLOAT *B, myMPC_FLOAT *u, myMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<16; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<10; j++ ){		
		for( i=0; i<16; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<10; n++ ){
		for( i=0; i<16; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_FLOAT *A, myMPC_FLOAT *x, myMPC_FLOAT *B, myMPC_FLOAT *u, myMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<8; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<10; j++ ){		
		for( i=0; i<8; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<10; n++ ){
		for( i=0; i<8; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 8,
 * u, su, uidx are of length 8 and v, sv, vidx are of length 8.
 */
void myMPC_LA_VSUB6_INDEXED_8_8_8(myMPC_FLOAT *u, myMPC_FLOAT *su, int* uidx, myMPC_FLOAT *v, myMPC_FLOAT *sv, int* vidx, myMPC_FLOAT *x)
{
	int i;
	for( i=0; i<8; i++ ){
		x[i] = 0;
	}
	for( i=0; i<8; i++){
		x[uidx[i]] += u[i]/su[i];
	}
	for( i=0; i<8; i++){
		x[vidx[i]] -= v[i]/sv[i];
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void myMPC_LA_DENSE_2MVMADD_8_10_8(myMPC_FLOAT *A, myMPC_FLOAT *x, myMPC_FLOAT *B, myMPC_FLOAT *u, myMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<8; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<10; j++ ){		
		for( i=0; i<8; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<8; n++ ){
		for( i=0; i<8; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/*
 * Vector subtraction z = x - y for vectors of length 10.
 */
void myMPC_LA_VSUB_10(myMPC_FLOAT *x, myMPC_FLOAT *y, myMPC_FLOAT *z)
{
	int i;
	for( i=0; i<10; i++){
		z[i] = x[i] - y[i];
	}
}


/*
 * Vector subtraction z = x - y for vectors of length 8.
 */
void myMPC_LA_VSUB_8(myMPC_FLOAT *x, myMPC_FLOAT *y, myMPC_FLOAT *z)
{
	int i;
	for( i=0; i<8; i++){
		z[i] = x[i] - y[i];
	}
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 10 (length of y >= 10).
 */
void myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_FLOAT *r, myMPC_FLOAT *s, myMPC_FLOAT *u, myMPC_FLOAT *y, int* yidx, myMPC_FLOAT *z)
{
	int i;
	for( i=0; i<10; i++ ){
		z[i] = -r[i]/s[i] - u[i]*y[yidx[i]];
	}
}


/** 
 * Computes z = -r./s + u.*y(y)
 * where all vectors except of y are of length 10 (length of y >= 10).
 */
void myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_FLOAT *r, myMPC_FLOAT *s, myMPC_FLOAT *u, myMPC_FLOAT *y, int* yidx, myMPC_FLOAT *z)
{
	int i;
	for( i=0; i<10; i++ ){
		z[i] = -r[i]/s[i] + u[i]*y[yidx[i]];
	}
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 8 (length of y >= 8).
 */
void myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_8(myMPC_FLOAT *r, myMPC_FLOAT *s, myMPC_FLOAT *u, myMPC_FLOAT *y, int* yidx, myMPC_FLOAT *z)
{
	int i;
	for( i=0; i<8; i++ ){
		z[i] = -r[i]/s[i] - u[i]*y[yidx[i]];
	}
}


/** 
 * Computes z = -r./s + u.*y(y)
 * where all vectors except of y are of length 8 (length of y >= 8).
 */
void myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_8(myMPC_FLOAT *r, myMPC_FLOAT *s, myMPC_FLOAT *u, myMPC_FLOAT *y, int* yidx, myMPC_FLOAT *z)
{
	int i;
	for( i=0; i<8; i++ ){
		z[i] = -r[i]/s[i] + u[i]*y[yidx[i]];
	}
}


/*
 * Computes ds = -l.\(r + s.*dl) for vectors of length 496.
 */
void myMPC_LA_VSUB7_496(myMPC_FLOAT *l, myMPC_FLOAT *r, myMPC_FLOAT *s, myMPC_FLOAT *dl, myMPC_FLOAT *ds)
{
	int i;
	for( i=0; i<496; i++){
		ds[i] = -(r[i] + s[i]*dl[i])/l[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 248.
 */
void myMPC_LA_VADD_248(myMPC_FLOAT *x, myMPC_FLOAT *y)
{
	int i;
	for( i=0; i<248; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 200.
 */
void myMPC_LA_VADD_200(myMPC_FLOAT *x, myMPC_FLOAT *y)
{
	int i;
	for( i=0; i<200; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 496.
 */
void myMPC_LA_VADD_496(myMPC_FLOAT *x, myMPC_FLOAT *y)
{
	int i;
	for( i=0; i<496; i++){
		x[i] += y[i];
	}
}


/**
 * Backtracking line search for combined predictor/corrector step.
 * Update on variables with safety factor gamma (to keep us away from
 * boundary).
 */
int myMPC_LINESEARCH_BACKTRACKING_COMBINED(myMPC_FLOAT *z, myMPC_FLOAT *v, myMPC_FLOAT *l, myMPC_FLOAT *s, myMPC_FLOAT *dz, myMPC_FLOAT *dv, myMPC_FLOAT *dl, myMPC_FLOAT *ds, myMPC_FLOAT *a, myMPC_FLOAT *mu)
{
    int i, lsIt=1;       
    myMPC_FLOAT dltemp;
    myMPC_FLOAT dstemp;    
    myMPC_FLOAT a_gamma;
            
    *a = 1.0;
    while( 1 ){                        

        /* check whether search criterion is fulfilled */
        for( i=0; i<496; i++ ){
            dltemp = l[i] + (*a)*dl[i];
            dstemp = s[i] + (*a)*ds[i];
            if( dltemp < 0 || dstemp < 0 ){
                lsIt++;
                break;
            }
        }
        
        /* 
         * If no early termination of the for-loop above occurred, we
         * found the required value of a and we can quit the while loop.
         */
        if( i == 496 ){
            break;
        } else {
            *a *= myMPC_SET_LS_SCALE;
            if( *a < myMPC_SET_LS_MINSTEP ){
                return myMPC_NOPROGRESS;
            }
        }
    }
    
    /* update variables with safety margin */
    a_gamma = (*a)*myMPC_SET_LS_MAXSTEP;
    
    /* primal variables */
    for( i=0; i<248; i++ ){
        z[i] += a_gamma*dz[i];
    }
    
    /* equality constraint multipliers */
    for( i=0; i<200; i++ ){
        v[i] += a_gamma*dv[i];
    }
    
    /* inequality constraint multipliers & slacks, also update mu */
    *mu = 0;
    for( i=0; i<496; i++ ){
        dltemp = l[i] + a_gamma*dl[i]; l[i] = dltemp;
        dstemp = s[i] + a_gamma*ds[i]; s[i] = dstemp;
        *mu += dltemp*dstemp;
    }
    
    *a = a_gamma;
    *mu /= (myMPC_FLOAT)496;
    return lsIt;
}




/* VARIABLE DEFINITIONS ------------------------------------------------ */
int exitcode;
int i;
myMPC_FLOAT myMPC_z[248];
myMPC_FLOAT myMPC_v[200];
myMPC_FLOAT myMPC_l[496];
myMPC_FLOAT myMPC_s[496];
myMPC_FLOAT myMPC_lbys[496];
myMPC_FLOAT myMPC_dz_aff[248];
myMPC_FLOAT myMPC_dv_aff[200];
myMPC_FLOAT myMPC_dl_aff[496];
myMPC_FLOAT myMPC_ds_aff[496];
myMPC_FLOAT myMPC_dz_cc[248];
myMPC_FLOAT myMPC_dv_cc[200];
myMPC_FLOAT myMPC_dl_cc[496];
myMPC_FLOAT myMPC_ds_cc[496];
myMPC_FLOAT myMPC_ccrhs[496];
myMPC_FLOAT* myMPC_z00 = myMPC_z + 0;
myMPC_FLOAT* myMPC_dzaff00 = myMPC_dz_aff + 0;
myMPC_FLOAT* myMPC_dzcc00 = myMPC_dz_cc + 0;
myMPC_FLOAT myMPC_rd00[10];
myMPC_FLOAT myMPC_Lbyrd00[10];
myMPC_FLOAT myMPC_grad_cost00[10];
myMPC_FLOAT myMPC_grad_eq00[10];
myMPC_FLOAT myMPC_grad_ineq00[10];
myMPC_FLOAT myMPC_ctv00[10];
myMPC_FLOAT myMPC_Phi00[55];
myMPC_FLOAT myMPC_C00[160] = {1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 6.9379989494012485E-003, 0.0000000000000000E+000, 4.5757073746928328E-001, 0.0000000000000000E+000, 2.0412116141763628E-003, 3.6157226873501935E-001, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 8.2230359326776817E-003, 0.0000000000000000E+000, 6.6701875383718912E-001, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9967301782111162E-001, -6.5392871633101576E-002, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9989100356424436E-003, 9.9967301782111140E-001, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 1.4519458761812670E-004, 0.0000000000000000E+000, 2.5721020921841781E-002, 0.0000000000000000E+000, -9.6790586830045088E-005, -1.7145107263391027E-002, 1.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 6.0571973485503729E-005, 0.0000000000000000E+000, 1.1350444043664105E-002, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 5.1378339811266489E-007, 0.0000000000000000E+000, 1.4519458761812670E-004, 0.0000000000000000E+000, -3.4251037260249828E-007, -9.6790586830045088E-005, 1.0000000000000000E-002, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 2.0853032565310755E-007, 0.0000000000000000E+000, 6.0571973485503729E-005, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E-002};
myMPC_FLOAT* myMPC_v00 = myMPC_v + 0;
myMPC_FLOAT myMPC_re00[16];
myMPC_FLOAT myMPC_beta00[16];
myMPC_FLOAT myMPC_betacc00[16];
myMPC_FLOAT* myMPC_dvaff00 = myMPC_dv_aff + 0;
myMPC_FLOAT* myMPC_dvcc00 = myMPC_dv_cc + 0;
myMPC_FLOAT myMPC_V00[160];
myMPC_FLOAT myMPC_Yd00[136];
myMPC_FLOAT myMPC_Ld00[136];
myMPC_FLOAT myMPC_yy00[16];
myMPC_FLOAT myMPC_bmy00[16];
myMPC_FLOAT myMPC_lb00[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx00[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb00 = myMPC_l + 0;
myMPC_FLOAT* myMPC_slb00 = myMPC_s + 0;
myMPC_FLOAT* myMPC_llbbyslb00 = myMPC_lbys + 0;
myMPC_FLOAT myMPC_rilb00[10];
myMPC_FLOAT* myMPC_dllbaff00 = myMPC_dl_aff + 0;
myMPC_FLOAT* myMPC_dslbaff00 = myMPC_ds_aff + 0;
myMPC_FLOAT* myMPC_dllbcc00 = myMPC_dl_cc + 0;
myMPC_FLOAT* myMPC_dslbcc00 = myMPC_ds_cc + 0;
myMPC_FLOAT* myMPC_ccrhsl00 = myMPC_ccrhs + 0;
myMPC_FLOAT myMPC_ub00[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx00[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub00 = myMPC_l + 10;
myMPC_FLOAT* myMPC_sub00 = myMPC_s + 10;
myMPC_FLOAT* myMPC_lubbysub00 = myMPC_lbys + 10;
myMPC_FLOAT myMPC_riub00[10];
myMPC_FLOAT* myMPC_dlubaff00 = myMPC_dl_aff + 10;
myMPC_FLOAT* myMPC_dsubaff00 = myMPC_ds_aff + 10;
myMPC_FLOAT* myMPC_dlubcc00 = myMPC_dl_cc + 10;
myMPC_FLOAT* myMPC_dsubcc00 = myMPC_ds_cc + 10;
myMPC_FLOAT* myMPC_ccrhsub00 = myMPC_ccrhs + 10;
myMPC_FLOAT* myMPC_z01 = myMPC_z + 10;
myMPC_FLOAT* myMPC_dzaff01 = myMPC_dz_aff + 10;
myMPC_FLOAT* myMPC_dzcc01 = myMPC_dz_cc + 10;
myMPC_FLOAT myMPC_rd01[10];
myMPC_FLOAT myMPC_Lbyrd01[10];
myMPC_FLOAT myMPC_grad_cost01[10];
myMPC_FLOAT myMPC_grad_eq01[10];
myMPC_FLOAT myMPC_grad_ineq01[10];
myMPC_FLOAT myMPC_ctv01[10];
myMPC_FLOAT myMPC_Phi01[55];
myMPC_FLOAT myMPC_C01[80] = {1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
6.9379989494012485E-003, 0.0000000000000000E+000, 4.5757073746928328E-001, 0.0000000000000000E+000, 2.0412116141763628E-003, 3.6157226873501935E-001, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 8.2230359326776817E-003, 0.0000000000000000E+000, 6.6701875383718912E-001, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9967301782111162E-001, -6.5392871633101576E-002, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9989100356424436E-003, 9.9967301782111140E-001, 0.0000000000000000E+000, 0.0000000000000000E+000, 
1.4519458761812670E-004, 0.0000000000000000E+000, 2.5721020921841781E-002, 0.0000000000000000E+000, -9.6790586830045088E-005, -1.7145107263391027E-002, 1.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 6.0571973485503729E-005, 0.0000000000000000E+000, 1.1350444043664105E-002, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 
5.1378339811266489E-007, 0.0000000000000000E+000, 1.4519458761812670E-004, 0.0000000000000000E+000, -3.4251037260249828E-007, -9.6790586830045088E-005, 1.0000000000000000E-002, 0.0000000000000000E+000, 
0.0000000000000000E+000, 2.0853032565310755E-007, 0.0000000000000000E+000, 6.0571973485503729E-005, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E-002};
myMPC_FLOAT* myMPC_v01 = myMPC_v + 16;
myMPC_FLOAT myMPC_re01[8];
myMPC_FLOAT myMPC_beta01[8];
myMPC_FLOAT myMPC_betacc01[8];
myMPC_FLOAT* myMPC_dvaff01 = myMPC_dv_aff + 16;
myMPC_FLOAT* myMPC_dvcc01 = myMPC_dv_cc + 16;
myMPC_FLOAT myMPC_V01[80];
myMPC_FLOAT myMPC_Yd01[36];
myMPC_FLOAT myMPC_Ld01[36];
myMPC_FLOAT myMPC_yy01[8];
myMPC_FLOAT myMPC_bmy01[8];
myMPC_FLOAT myMPC_c01[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_D01[160] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W01[160];
myMPC_FLOAT myMPC_lb01[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx01[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb01 = myMPC_l + 20;
myMPC_FLOAT* myMPC_slb01 = myMPC_s + 20;
myMPC_FLOAT* myMPC_llbbyslb01 = myMPC_lbys + 20;
myMPC_FLOAT myMPC_rilb01[10];
myMPC_FLOAT* myMPC_dllbaff01 = myMPC_dl_aff + 20;
myMPC_FLOAT* myMPC_dslbaff01 = myMPC_ds_aff + 20;
myMPC_FLOAT* myMPC_dllbcc01 = myMPC_dl_cc + 20;
myMPC_FLOAT* myMPC_dslbcc01 = myMPC_ds_cc + 20;
myMPC_FLOAT* myMPC_ccrhsl01 = myMPC_ccrhs + 20;
myMPC_FLOAT myMPC_ub01[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx01[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub01 = myMPC_l + 30;
myMPC_FLOAT* myMPC_sub01 = myMPC_s + 30;
myMPC_FLOAT* myMPC_lubbysub01 = myMPC_lbys + 30;
myMPC_FLOAT myMPC_riub01[10];
myMPC_FLOAT* myMPC_dlubaff01 = myMPC_dl_aff + 30;
myMPC_FLOAT* myMPC_dsubaff01 = myMPC_ds_aff + 30;
myMPC_FLOAT* myMPC_dlubcc01 = myMPC_dl_cc + 30;
myMPC_FLOAT* myMPC_dsubcc01 = myMPC_ds_cc + 30;
myMPC_FLOAT* myMPC_ccrhsub01 = myMPC_ccrhs + 30;
myMPC_FLOAT myMPC_Ysd01[128];
myMPC_FLOAT myMPC_Lsd01[128];
myMPC_FLOAT* myMPC_z02 = myMPC_z + 20;
myMPC_FLOAT* myMPC_dzaff02 = myMPC_dz_aff + 20;
myMPC_FLOAT* myMPC_dzcc02 = myMPC_dz_cc + 20;
myMPC_FLOAT myMPC_rd02[10];
myMPC_FLOAT myMPC_Lbyrd02[10];
myMPC_FLOAT myMPC_grad_cost02[10];
myMPC_FLOAT myMPC_grad_eq02[10];
myMPC_FLOAT myMPC_grad_ineq02[10];
myMPC_FLOAT myMPC_ctv02[10];
myMPC_FLOAT myMPC_Phi02[55];
myMPC_FLOAT* myMPC_v02 = myMPC_v + 24;
myMPC_FLOAT myMPC_re02[8];
myMPC_FLOAT myMPC_beta02[8];
myMPC_FLOAT myMPC_betacc02[8];
myMPC_FLOAT* myMPC_dvaff02 = myMPC_dv_aff + 24;
myMPC_FLOAT* myMPC_dvcc02 = myMPC_dv_cc + 24;
myMPC_FLOAT myMPC_V02[80];
myMPC_FLOAT myMPC_Yd02[36];
myMPC_FLOAT myMPC_Ld02[36];
myMPC_FLOAT myMPC_yy02[8];
myMPC_FLOAT myMPC_bmy02[8];
myMPC_FLOAT myMPC_c02[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_D02[80] = {-1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W02[80];
myMPC_FLOAT myMPC_lb02[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx02[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb02 = myMPC_l + 40;
myMPC_FLOAT* myMPC_slb02 = myMPC_s + 40;
myMPC_FLOAT* myMPC_llbbyslb02 = myMPC_lbys + 40;
myMPC_FLOAT myMPC_rilb02[10];
myMPC_FLOAT* myMPC_dllbaff02 = myMPC_dl_aff + 40;
myMPC_FLOAT* myMPC_dslbaff02 = myMPC_ds_aff + 40;
myMPC_FLOAT* myMPC_dllbcc02 = myMPC_dl_cc + 40;
myMPC_FLOAT* myMPC_dslbcc02 = myMPC_ds_cc + 40;
myMPC_FLOAT* myMPC_ccrhsl02 = myMPC_ccrhs + 40;
myMPC_FLOAT myMPC_ub02[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx02[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub02 = myMPC_l + 50;
myMPC_FLOAT* myMPC_sub02 = myMPC_s + 50;
myMPC_FLOAT* myMPC_lubbysub02 = myMPC_lbys + 50;
myMPC_FLOAT myMPC_riub02[10];
myMPC_FLOAT* myMPC_dlubaff02 = myMPC_dl_aff + 50;
myMPC_FLOAT* myMPC_dsubaff02 = myMPC_ds_aff + 50;
myMPC_FLOAT* myMPC_dlubcc02 = myMPC_dl_cc + 50;
myMPC_FLOAT* myMPC_dsubcc02 = myMPC_ds_cc + 50;
myMPC_FLOAT* myMPC_ccrhsub02 = myMPC_ccrhs + 50;
myMPC_FLOAT myMPC_Ysd02[64];
myMPC_FLOAT myMPC_Lsd02[64];
myMPC_FLOAT* myMPC_z03 = myMPC_z + 30;
myMPC_FLOAT* myMPC_dzaff03 = myMPC_dz_aff + 30;
myMPC_FLOAT* myMPC_dzcc03 = myMPC_dz_cc + 30;
myMPC_FLOAT myMPC_rd03[10];
myMPC_FLOAT myMPC_Lbyrd03[10];
myMPC_FLOAT myMPC_grad_cost03[10];
myMPC_FLOAT myMPC_grad_eq03[10];
myMPC_FLOAT myMPC_grad_ineq03[10];
myMPC_FLOAT myMPC_ctv03[10];
myMPC_FLOAT myMPC_Phi03[55];
myMPC_FLOAT* myMPC_v03 = myMPC_v + 32;
myMPC_FLOAT myMPC_re03[8];
myMPC_FLOAT myMPC_beta03[8];
myMPC_FLOAT myMPC_betacc03[8];
myMPC_FLOAT* myMPC_dvaff03 = myMPC_dv_aff + 32;
myMPC_FLOAT* myMPC_dvcc03 = myMPC_dv_cc + 32;
myMPC_FLOAT myMPC_V03[80];
myMPC_FLOAT myMPC_Yd03[36];
myMPC_FLOAT myMPC_Ld03[36];
myMPC_FLOAT myMPC_yy03[8];
myMPC_FLOAT myMPC_bmy03[8];
myMPC_FLOAT myMPC_c03[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W03[80];
myMPC_FLOAT myMPC_lb03[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx03[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb03 = myMPC_l + 60;
myMPC_FLOAT* myMPC_slb03 = myMPC_s + 60;
myMPC_FLOAT* myMPC_llbbyslb03 = myMPC_lbys + 60;
myMPC_FLOAT myMPC_rilb03[10];
myMPC_FLOAT* myMPC_dllbaff03 = myMPC_dl_aff + 60;
myMPC_FLOAT* myMPC_dslbaff03 = myMPC_ds_aff + 60;
myMPC_FLOAT* myMPC_dllbcc03 = myMPC_dl_cc + 60;
myMPC_FLOAT* myMPC_dslbcc03 = myMPC_ds_cc + 60;
myMPC_FLOAT* myMPC_ccrhsl03 = myMPC_ccrhs + 60;
myMPC_FLOAT myMPC_ub03[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx03[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub03 = myMPC_l + 70;
myMPC_FLOAT* myMPC_sub03 = myMPC_s + 70;
myMPC_FLOAT* myMPC_lubbysub03 = myMPC_lbys + 70;
myMPC_FLOAT myMPC_riub03[10];
myMPC_FLOAT* myMPC_dlubaff03 = myMPC_dl_aff + 70;
myMPC_FLOAT* myMPC_dsubaff03 = myMPC_ds_aff + 70;
myMPC_FLOAT* myMPC_dlubcc03 = myMPC_dl_cc + 70;
myMPC_FLOAT* myMPC_dsubcc03 = myMPC_ds_cc + 70;
myMPC_FLOAT* myMPC_ccrhsub03 = myMPC_ccrhs + 70;
myMPC_FLOAT myMPC_Ysd03[64];
myMPC_FLOAT myMPC_Lsd03[64];
myMPC_FLOAT* myMPC_z04 = myMPC_z + 40;
myMPC_FLOAT* myMPC_dzaff04 = myMPC_dz_aff + 40;
myMPC_FLOAT* myMPC_dzcc04 = myMPC_dz_cc + 40;
myMPC_FLOAT myMPC_rd04[10];
myMPC_FLOAT myMPC_Lbyrd04[10];
myMPC_FLOAT myMPC_grad_cost04[10];
myMPC_FLOAT myMPC_grad_eq04[10];
myMPC_FLOAT myMPC_grad_ineq04[10];
myMPC_FLOAT myMPC_ctv04[10];
myMPC_FLOAT myMPC_Phi04[55];
myMPC_FLOAT* myMPC_v04 = myMPC_v + 40;
myMPC_FLOAT myMPC_re04[8];
myMPC_FLOAT myMPC_beta04[8];
myMPC_FLOAT myMPC_betacc04[8];
myMPC_FLOAT* myMPC_dvaff04 = myMPC_dv_aff + 40;
myMPC_FLOAT* myMPC_dvcc04 = myMPC_dv_cc + 40;
myMPC_FLOAT myMPC_V04[80];
myMPC_FLOAT myMPC_Yd04[36];
myMPC_FLOAT myMPC_Ld04[36];
myMPC_FLOAT myMPC_yy04[8];
myMPC_FLOAT myMPC_bmy04[8];
myMPC_FLOAT myMPC_c04[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W04[80];
myMPC_FLOAT myMPC_lb04[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx04[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb04 = myMPC_l + 80;
myMPC_FLOAT* myMPC_slb04 = myMPC_s + 80;
myMPC_FLOAT* myMPC_llbbyslb04 = myMPC_lbys + 80;
myMPC_FLOAT myMPC_rilb04[10];
myMPC_FLOAT* myMPC_dllbaff04 = myMPC_dl_aff + 80;
myMPC_FLOAT* myMPC_dslbaff04 = myMPC_ds_aff + 80;
myMPC_FLOAT* myMPC_dllbcc04 = myMPC_dl_cc + 80;
myMPC_FLOAT* myMPC_dslbcc04 = myMPC_ds_cc + 80;
myMPC_FLOAT* myMPC_ccrhsl04 = myMPC_ccrhs + 80;
myMPC_FLOAT myMPC_ub04[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx04[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub04 = myMPC_l + 90;
myMPC_FLOAT* myMPC_sub04 = myMPC_s + 90;
myMPC_FLOAT* myMPC_lubbysub04 = myMPC_lbys + 90;
myMPC_FLOAT myMPC_riub04[10];
myMPC_FLOAT* myMPC_dlubaff04 = myMPC_dl_aff + 90;
myMPC_FLOAT* myMPC_dsubaff04 = myMPC_ds_aff + 90;
myMPC_FLOAT* myMPC_dlubcc04 = myMPC_dl_cc + 90;
myMPC_FLOAT* myMPC_dsubcc04 = myMPC_ds_cc + 90;
myMPC_FLOAT* myMPC_ccrhsub04 = myMPC_ccrhs + 90;
myMPC_FLOAT myMPC_Ysd04[64];
myMPC_FLOAT myMPC_Lsd04[64];
myMPC_FLOAT* myMPC_z05 = myMPC_z + 50;
myMPC_FLOAT* myMPC_dzaff05 = myMPC_dz_aff + 50;
myMPC_FLOAT* myMPC_dzcc05 = myMPC_dz_cc + 50;
myMPC_FLOAT myMPC_rd05[10];
myMPC_FLOAT myMPC_Lbyrd05[10];
myMPC_FLOAT myMPC_grad_cost05[10];
myMPC_FLOAT myMPC_grad_eq05[10];
myMPC_FLOAT myMPC_grad_ineq05[10];
myMPC_FLOAT myMPC_ctv05[10];
myMPC_FLOAT myMPC_Phi05[55];
myMPC_FLOAT* myMPC_v05 = myMPC_v + 48;
myMPC_FLOAT myMPC_re05[8];
myMPC_FLOAT myMPC_beta05[8];
myMPC_FLOAT myMPC_betacc05[8];
myMPC_FLOAT* myMPC_dvaff05 = myMPC_dv_aff + 48;
myMPC_FLOAT* myMPC_dvcc05 = myMPC_dv_cc + 48;
myMPC_FLOAT myMPC_V05[80];
myMPC_FLOAT myMPC_Yd05[36];
myMPC_FLOAT myMPC_Ld05[36];
myMPC_FLOAT myMPC_yy05[8];
myMPC_FLOAT myMPC_bmy05[8];
myMPC_FLOAT myMPC_c05[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W05[80];
myMPC_FLOAT myMPC_lb05[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx05[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb05 = myMPC_l + 100;
myMPC_FLOAT* myMPC_slb05 = myMPC_s + 100;
myMPC_FLOAT* myMPC_llbbyslb05 = myMPC_lbys + 100;
myMPC_FLOAT myMPC_rilb05[10];
myMPC_FLOAT* myMPC_dllbaff05 = myMPC_dl_aff + 100;
myMPC_FLOAT* myMPC_dslbaff05 = myMPC_ds_aff + 100;
myMPC_FLOAT* myMPC_dllbcc05 = myMPC_dl_cc + 100;
myMPC_FLOAT* myMPC_dslbcc05 = myMPC_ds_cc + 100;
myMPC_FLOAT* myMPC_ccrhsl05 = myMPC_ccrhs + 100;
myMPC_FLOAT myMPC_ub05[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx05[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub05 = myMPC_l + 110;
myMPC_FLOAT* myMPC_sub05 = myMPC_s + 110;
myMPC_FLOAT* myMPC_lubbysub05 = myMPC_lbys + 110;
myMPC_FLOAT myMPC_riub05[10];
myMPC_FLOAT* myMPC_dlubaff05 = myMPC_dl_aff + 110;
myMPC_FLOAT* myMPC_dsubaff05 = myMPC_ds_aff + 110;
myMPC_FLOAT* myMPC_dlubcc05 = myMPC_dl_cc + 110;
myMPC_FLOAT* myMPC_dsubcc05 = myMPC_ds_cc + 110;
myMPC_FLOAT* myMPC_ccrhsub05 = myMPC_ccrhs + 110;
myMPC_FLOAT myMPC_Ysd05[64];
myMPC_FLOAT myMPC_Lsd05[64];
myMPC_FLOAT* myMPC_z06 = myMPC_z + 60;
myMPC_FLOAT* myMPC_dzaff06 = myMPC_dz_aff + 60;
myMPC_FLOAT* myMPC_dzcc06 = myMPC_dz_cc + 60;
myMPC_FLOAT myMPC_rd06[10];
myMPC_FLOAT myMPC_Lbyrd06[10];
myMPC_FLOAT myMPC_grad_cost06[10];
myMPC_FLOAT myMPC_grad_eq06[10];
myMPC_FLOAT myMPC_grad_ineq06[10];
myMPC_FLOAT myMPC_ctv06[10];
myMPC_FLOAT myMPC_Phi06[55];
myMPC_FLOAT* myMPC_v06 = myMPC_v + 56;
myMPC_FLOAT myMPC_re06[8];
myMPC_FLOAT myMPC_beta06[8];
myMPC_FLOAT myMPC_betacc06[8];
myMPC_FLOAT* myMPC_dvaff06 = myMPC_dv_aff + 56;
myMPC_FLOAT* myMPC_dvcc06 = myMPC_dv_cc + 56;
myMPC_FLOAT myMPC_V06[80];
myMPC_FLOAT myMPC_Yd06[36];
myMPC_FLOAT myMPC_Ld06[36];
myMPC_FLOAT myMPC_yy06[8];
myMPC_FLOAT myMPC_bmy06[8];
myMPC_FLOAT myMPC_c06[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W06[80];
myMPC_FLOAT myMPC_lb06[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx06[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb06 = myMPC_l + 120;
myMPC_FLOAT* myMPC_slb06 = myMPC_s + 120;
myMPC_FLOAT* myMPC_llbbyslb06 = myMPC_lbys + 120;
myMPC_FLOAT myMPC_rilb06[10];
myMPC_FLOAT* myMPC_dllbaff06 = myMPC_dl_aff + 120;
myMPC_FLOAT* myMPC_dslbaff06 = myMPC_ds_aff + 120;
myMPC_FLOAT* myMPC_dllbcc06 = myMPC_dl_cc + 120;
myMPC_FLOAT* myMPC_dslbcc06 = myMPC_ds_cc + 120;
myMPC_FLOAT* myMPC_ccrhsl06 = myMPC_ccrhs + 120;
myMPC_FLOAT myMPC_ub06[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx06[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub06 = myMPC_l + 130;
myMPC_FLOAT* myMPC_sub06 = myMPC_s + 130;
myMPC_FLOAT* myMPC_lubbysub06 = myMPC_lbys + 130;
myMPC_FLOAT myMPC_riub06[10];
myMPC_FLOAT* myMPC_dlubaff06 = myMPC_dl_aff + 130;
myMPC_FLOAT* myMPC_dsubaff06 = myMPC_ds_aff + 130;
myMPC_FLOAT* myMPC_dlubcc06 = myMPC_dl_cc + 130;
myMPC_FLOAT* myMPC_dsubcc06 = myMPC_ds_cc + 130;
myMPC_FLOAT* myMPC_ccrhsub06 = myMPC_ccrhs + 130;
myMPC_FLOAT myMPC_Ysd06[64];
myMPC_FLOAT myMPC_Lsd06[64];
myMPC_FLOAT* myMPC_z07 = myMPC_z + 70;
myMPC_FLOAT* myMPC_dzaff07 = myMPC_dz_aff + 70;
myMPC_FLOAT* myMPC_dzcc07 = myMPC_dz_cc + 70;
myMPC_FLOAT myMPC_rd07[10];
myMPC_FLOAT myMPC_Lbyrd07[10];
myMPC_FLOAT myMPC_grad_cost07[10];
myMPC_FLOAT myMPC_grad_eq07[10];
myMPC_FLOAT myMPC_grad_ineq07[10];
myMPC_FLOAT myMPC_ctv07[10];
myMPC_FLOAT myMPC_Phi07[55];
myMPC_FLOAT* myMPC_v07 = myMPC_v + 64;
myMPC_FLOAT myMPC_re07[8];
myMPC_FLOAT myMPC_beta07[8];
myMPC_FLOAT myMPC_betacc07[8];
myMPC_FLOAT* myMPC_dvaff07 = myMPC_dv_aff + 64;
myMPC_FLOAT* myMPC_dvcc07 = myMPC_dv_cc + 64;
myMPC_FLOAT myMPC_V07[80];
myMPC_FLOAT myMPC_Yd07[36];
myMPC_FLOAT myMPC_Ld07[36];
myMPC_FLOAT myMPC_yy07[8];
myMPC_FLOAT myMPC_bmy07[8];
myMPC_FLOAT myMPC_c07[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W07[80];
myMPC_FLOAT myMPC_lb07[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx07[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb07 = myMPC_l + 140;
myMPC_FLOAT* myMPC_slb07 = myMPC_s + 140;
myMPC_FLOAT* myMPC_llbbyslb07 = myMPC_lbys + 140;
myMPC_FLOAT myMPC_rilb07[10];
myMPC_FLOAT* myMPC_dllbaff07 = myMPC_dl_aff + 140;
myMPC_FLOAT* myMPC_dslbaff07 = myMPC_ds_aff + 140;
myMPC_FLOAT* myMPC_dllbcc07 = myMPC_dl_cc + 140;
myMPC_FLOAT* myMPC_dslbcc07 = myMPC_ds_cc + 140;
myMPC_FLOAT* myMPC_ccrhsl07 = myMPC_ccrhs + 140;
myMPC_FLOAT myMPC_ub07[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx07[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub07 = myMPC_l + 150;
myMPC_FLOAT* myMPC_sub07 = myMPC_s + 150;
myMPC_FLOAT* myMPC_lubbysub07 = myMPC_lbys + 150;
myMPC_FLOAT myMPC_riub07[10];
myMPC_FLOAT* myMPC_dlubaff07 = myMPC_dl_aff + 150;
myMPC_FLOAT* myMPC_dsubaff07 = myMPC_ds_aff + 150;
myMPC_FLOAT* myMPC_dlubcc07 = myMPC_dl_cc + 150;
myMPC_FLOAT* myMPC_dsubcc07 = myMPC_ds_cc + 150;
myMPC_FLOAT* myMPC_ccrhsub07 = myMPC_ccrhs + 150;
myMPC_FLOAT myMPC_Ysd07[64];
myMPC_FLOAT myMPC_Lsd07[64];
myMPC_FLOAT* myMPC_z08 = myMPC_z + 80;
myMPC_FLOAT* myMPC_dzaff08 = myMPC_dz_aff + 80;
myMPC_FLOAT* myMPC_dzcc08 = myMPC_dz_cc + 80;
myMPC_FLOAT myMPC_rd08[10];
myMPC_FLOAT myMPC_Lbyrd08[10];
myMPC_FLOAT myMPC_grad_cost08[10];
myMPC_FLOAT myMPC_grad_eq08[10];
myMPC_FLOAT myMPC_grad_ineq08[10];
myMPC_FLOAT myMPC_ctv08[10];
myMPC_FLOAT myMPC_Phi08[55];
myMPC_FLOAT* myMPC_v08 = myMPC_v + 72;
myMPC_FLOAT myMPC_re08[8];
myMPC_FLOAT myMPC_beta08[8];
myMPC_FLOAT myMPC_betacc08[8];
myMPC_FLOAT* myMPC_dvaff08 = myMPC_dv_aff + 72;
myMPC_FLOAT* myMPC_dvcc08 = myMPC_dv_cc + 72;
myMPC_FLOAT myMPC_V08[80];
myMPC_FLOAT myMPC_Yd08[36];
myMPC_FLOAT myMPC_Ld08[36];
myMPC_FLOAT myMPC_yy08[8];
myMPC_FLOAT myMPC_bmy08[8];
myMPC_FLOAT myMPC_c08[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W08[80];
myMPC_FLOAT myMPC_lb08[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx08[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb08 = myMPC_l + 160;
myMPC_FLOAT* myMPC_slb08 = myMPC_s + 160;
myMPC_FLOAT* myMPC_llbbyslb08 = myMPC_lbys + 160;
myMPC_FLOAT myMPC_rilb08[10];
myMPC_FLOAT* myMPC_dllbaff08 = myMPC_dl_aff + 160;
myMPC_FLOAT* myMPC_dslbaff08 = myMPC_ds_aff + 160;
myMPC_FLOAT* myMPC_dllbcc08 = myMPC_dl_cc + 160;
myMPC_FLOAT* myMPC_dslbcc08 = myMPC_ds_cc + 160;
myMPC_FLOAT* myMPC_ccrhsl08 = myMPC_ccrhs + 160;
myMPC_FLOAT myMPC_ub08[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx08[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub08 = myMPC_l + 170;
myMPC_FLOAT* myMPC_sub08 = myMPC_s + 170;
myMPC_FLOAT* myMPC_lubbysub08 = myMPC_lbys + 170;
myMPC_FLOAT myMPC_riub08[10];
myMPC_FLOAT* myMPC_dlubaff08 = myMPC_dl_aff + 170;
myMPC_FLOAT* myMPC_dsubaff08 = myMPC_ds_aff + 170;
myMPC_FLOAT* myMPC_dlubcc08 = myMPC_dl_cc + 170;
myMPC_FLOAT* myMPC_dsubcc08 = myMPC_ds_cc + 170;
myMPC_FLOAT* myMPC_ccrhsub08 = myMPC_ccrhs + 170;
myMPC_FLOAT myMPC_Ysd08[64];
myMPC_FLOAT myMPC_Lsd08[64];
myMPC_FLOAT* myMPC_z09 = myMPC_z + 90;
myMPC_FLOAT* myMPC_dzaff09 = myMPC_dz_aff + 90;
myMPC_FLOAT* myMPC_dzcc09 = myMPC_dz_cc + 90;
myMPC_FLOAT myMPC_rd09[10];
myMPC_FLOAT myMPC_Lbyrd09[10];
myMPC_FLOAT myMPC_grad_cost09[10];
myMPC_FLOAT myMPC_grad_eq09[10];
myMPC_FLOAT myMPC_grad_ineq09[10];
myMPC_FLOAT myMPC_ctv09[10];
myMPC_FLOAT myMPC_Phi09[55];
myMPC_FLOAT* myMPC_v09 = myMPC_v + 80;
myMPC_FLOAT myMPC_re09[8];
myMPC_FLOAT myMPC_beta09[8];
myMPC_FLOAT myMPC_betacc09[8];
myMPC_FLOAT* myMPC_dvaff09 = myMPC_dv_aff + 80;
myMPC_FLOAT* myMPC_dvcc09 = myMPC_dv_cc + 80;
myMPC_FLOAT myMPC_V09[80];
myMPC_FLOAT myMPC_Yd09[36];
myMPC_FLOAT myMPC_Ld09[36];
myMPC_FLOAT myMPC_yy09[8];
myMPC_FLOAT myMPC_bmy09[8];
myMPC_FLOAT myMPC_c09[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W09[80];
myMPC_FLOAT myMPC_lb09[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx09[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb09 = myMPC_l + 180;
myMPC_FLOAT* myMPC_slb09 = myMPC_s + 180;
myMPC_FLOAT* myMPC_llbbyslb09 = myMPC_lbys + 180;
myMPC_FLOAT myMPC_rilb09[10];
myMPC_FLOAT* myMPC_dllbaff09 = myMPC_dl_aff + 180;
myMPC_FLOAT* myMPC_dslbaff09 = myMPC_ds_aff + 180;
myMPC_FLOAT* myMPC_dllbcc09 = myMPC_dl_cc + 180;
myMPC_FLOAT* myMPC_dslbcc09 = myMPC_ds_cc + 180;
myMPC_FLOAT* myMPC_ccrhsl09 = myMPC_ccrhs + 180;
myMPC_FLOAT myMPC_ub09[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx09[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub09 = myMPC_l + 190;
myMPC_FLOAT* myMPC_sub09 = myMPC_s + 190;
myMPC_FLOAT* myMPC_lubbysub09 = myMPC_lbys + 190;
myMPC_FLOAT myMPC_riub09[10];
myMPC_FLOAT* myMPC_dlubaff09 = myMPC_dl_aff + 190;
myMPC_FLOAT* myMPC_dsubaff09 = myMPC_ds_aff + 190;
myMPC_FLOAT* myMPC_dlubcc09 = myMPC_dl_cc + 190;
myMPC_FLOAT* myMPC_dsubcc09 = myMPC_ds_cc + 190;
myMPC_FLOAT* myMPC_ccrhsub09 = myMPC_ccrhs + 190;
myMPC_FLOAT myMPC_Ysd09[64];
myMPC_FLOAT myMPC_Lsd09[64];
myMPC_FLOAT* myMPC_z10 = myMPC_z + 100;
myMPC_FLOAT* myMPC_dzaff10 = myMPC_dz_aff + 100;
myMPC_FLOAT* myMPC_dzcc10 = myMPC_dz_cc + 100;
myMPC_FLOAT myMPC_rd10[10];
myMPC_FLOAT myMPC_Lbyrd10[10];
myMPC_FLOAT myMPC_grad_cost10[10];
myMPC_FLOAT myMPC_grad_eq10[10];
myMPC_FLOAT myMPC_grad_ineq10[10];
myMPC_FLOAT myMPC_ctv10[10];
myMPC_FLOAT myMPC_Phi10[55];
myMPC_FLOAT* myMPC_v10 = myMPC_v + 88;
myMPC_FLOAT myMPC_re10[8];
myMPC_FLOAT myMPC_beta10[8];
myMPC_FLOAT myMPC_betacc10[8];
myMPC_FLOAT* myMPC_dvaff10 = myMPC_dv_aff + 88;
myMPC_FLOAT* myMPC_dvcc10 = myMPC_dv_cc + 88;
myMPC_FLOAT myMPC_V10[80];
myMPC_FLOAT myMPC_Yd10[36];
myMPC_FLOAT myMPC_Ld10[36];
myMPC_FLOAT myMPC_yy10[8];
myMPC_FLOAT myMPC_bmy10[8];
myMPC_FLOAT myMPC_c10[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W10[80];
myMPC_FLOAT myMPC_lb10[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx10[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb10 = myMPC_l + 200;
myMPC_FLOAT* myMPC_slb10 = myMPC_s + 200;
myMPC_FLOAT* myMPC_llbbyslb10 = myMPC_lbys + 200;
myMPC_FLOAT myMPC_rilb10[10];
myMPC_FLOAT* myMPC_dllbaff10 = myMPC_dl_aff + 200;
myMPC_FLOAT* myMPC_dslbaff10 = myMPC_ds_aff + 200;
myMPC_FLOAT* myMPC_dllbcc10 = myMPC_dl_cc + 200;
myMPC_FLOAT* myMPC_dslbcc10 = myMPC_ds_cc + 200;
myMPC_FLOAT* myMPC_ccrhsl10 = myMPC_ccrhs + 200;
myMPC_FLOAT myMPC_ub10[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx10[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub10 = myMPC_l + 210;
myMPC_FLOAT* myMPC_sub10 = myMPC_s + 210;
myMPC_FLOAT* myMPC_lubbysub10 = myMPC_lbys + 210;
myMPC_FLOAT myMPC_riub10[10];
myMPC_FLOAT* myMPC_dlubaff10 = myMPC_dl_aff + 210;
myMPC_FLOAT* myMPC_dsubaff10 = myMPC_ds_aff + 210;
myMPC_FLOAT* myMPC_dlubcc10 = myMPC_dl_cc + 210;
myMPC_FLOAT* myMPC_dsubcc10 = myMPC_ds_cc + 210;
myMPC_FLOAT* myMPC_ccrhsub10 = myMPC_ccrhs + 210;
myMPC_FLOAT myMPC_Ysd10[64];
myMPC_FLOAT myMPC_Lsd10[64];
myMPC_FLOAT* myMPC_z11 = myMPC_z + 110;
myMPC_FLOAT* myMPC_dzaff11 = myMPC_dz_aff + 110;
myMPC_FLOAT* myMPC_dzcc11 = myMPC_dz_cc + 110;
myMPC_FLOAT myMPC_rd11[10];
myMPC_FLOAT myMPC_Lbyrd11[10];
myMPC_FLOAT myMPC_grad_cost11[10];
myMPC_FLOAT myMPC_grad_eq11[10];
myMPC_FLOAT myMPC_grad_ineq11[10];
myMPC_FLOAT myMPC_ctv11[10];
myMPC_FLOAT myMPC_Phi11[55];
myMPC_FLOAT* myMPC_v11 = myMPC_v + 96;
myMPC_FLOAT myMPC_re11[8];
myMPC_FLOAT myMPC_beta11[8];
myMPC_FLOAT myMPC_betacc11[8];
myMPC_FLOAT* myMPC_dvaff11 = myMPC_dv_aff + 96;
myMPC_FLOAT* myMPC_dvcc11 = myMPC_dv_cc + 96;
myMPC_FLOAT myMPC_V11[80];
myMPC_FLOAT myMPC_Yd11[36];
myMPC_FLOAT myMPC_Ld11[36];
myMPC_FLOAT myMPC_yy11[8];
myMPC_FLOAT myMPC_bmy11[8];
myMPC_FLOAT myMPC_c11[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W11[80];
myMPC_FLOAT myMPC_lb11[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx11[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb11 = myMPC_l + 220;
myMPC_FLOAT* myMPC_slb11 = myMPC_s + 220;
myMPC_FLOAT* myMPC_llbbyslb11 = myMPC_lbys + 220;
myMPC_FLOAT myMPC_rilb11[10];
myMPC_FLOAT* myMPC_dllbaff11 = myMPC_dl_aff + 220;
myMPC_FLOAT* myMPC_dslbaff11 = myMPC_ds_aff + 220;
myMPC_FLOAT* myMPC_dllbcc11 = myMPC_dl_cc + 220;
myMPC_FLOAT* myMPC_dslbcc11 = myMPC_ds_cc + 220;
myMPC_FLOAT* myMPC_ccrhsl11 = myMPC_ccrhs + 220;
myMPC_FLOAT myMPC_ub11[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx11[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub11 = myMPC_l + 230;
myMPC_FLOAT* myMPC_sub11 = myMPC_s + 230;
myMPC_FLOAT* myMPC_lubbysub11 = myMPC_lbys + 230;
myMPC_FLOAT myMPC_riub11[10];
myMPC_FLOAT* myMPC_dlubaff11 = myMPC_dl_aff + 230;
myMPC_FLOAT* myMPC_dsubaff11 = myMPC_ds_aff + 230;
myMPC_FLOAT* myMPC_dlubcc11 = myMPC_dl_cc + 230;
myMPC_FLOAT* myMPC_dsubcc11 = myMPC_ds_cc + 230;
myMPC_FLOAT* myMPC_ccrhsub11 = myMPC_ccrhs + 230;
myMPC_FLOAT myMPC_Ysd11[64];
myMPC_FLOAT myMPC_Lsd11[64];
myMPC_FLOAT* myMPC_z12 = myMPC_z + 120;
myMPC_FLOAT* myMPC_dzaff12 = myMPC_dz_aff + 120;
myMPC_FLOAT* myMPC_dzcc12 = myMPC_dz_cc + 120;
myMPC_FLOAT myMPC_rd12[10];
myMPC_FLOAT myMPC_Lbyrd12[10];
myMPC_FLOAT myMPC_grad_cost12[10];
myMPC_FLOAT myMPC_grad_eq12[10];
myMPC_FLOAT myMPC_grad_ineq12[10];
myMPC_FLOAT myMPC_ctv12[10];
myMPC_FLOAT myMPC_Phi12[55];
myMPC_FLOAT* myMPC_v12 = myMPC_v + 104;
myMPC_FLOAT myMPC_re12[8];
myMPC_FLOAT myMPC_beta12[8];
myMPC_FLOAT myMPC_betacc12[8];
myMPC_FLOAT* myMPC_dvaff12 = myMPC_dv_aff + 104;
myMPC_FLOAT* myMPC_dvcc12 = myMPC_dv_cc + 104;
myMPC_FLOAT myMPC_V12[80];
myMPC_FLOAT myMPC_Yd12[36];
myMPC_FLOAT myMPC_Ld12[36];
myMPC_FLOAT myMPC_yy12[8];
myMPC_FLOAT myMPC_bmy12[8];
myMPC_FLOAT myMPC_c12[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W12[80];
myMPC_FLOAT myMPC_lb12[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx12[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb12 = myMPC_l + 240;
myMPC_FLOAT* myMPC_slb12 = myMPC_s + 240;
myMPC_FLOAT* myMPC_llbbyslb12 = myMPC_lbys + 240;
myMPC_FLOAT myMPC_rilb12[10];
myMPC_FLOAT* myMPC_dllbaff12 = myMPC_dl_aff + 240;
myMPC_FLOAT* myMPC_dslbaff12 = myMPC_ds_aff + 240;
myMPC_FLOAT* myMPC_dllbcc12 = myMPC_dl_cc + 240;
myMPC_FLOAT* myMPC_dslbcc12 = myMPC_ds_cc + 240;
myMPC_FLOAT* myMPC_ccrhsl12 = myMPC_ccrhs + 240;
myMPC_FLOAT myMPC_ub12[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx12[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub12 = myMPC_l + 250;
myMPC_FLOAT* myMPC_sub12 = myMPC_s + 250;
myMPC_FLOAT* myMPC_lubbysub12 = myMPC_lbys + 250;
myMPC_FLOAT myMPC_riub12[10];
myMPC_FLOAT* myMPC_dlubaff12 = myMPC_dl_aff + 250;
myMPC_FLOAT* myMPC_dsubaff12 = myMPC_ds_aff + 250;
myMPC_FLOAT* myMPC_dlubcc12 = myMPC_dl_cc + 250;
myMPC_FLOAT* myMPC_dsubcc12 = myMPC_ds_cc + 250;
myMPC_FLOAT* myMPC_ccrhsub12 = myMPC_ccrhs + 250;
myMPC_FLOAT myMPC_Ysd12[64];
myMPC_FLOAT myMPC_Lsd12[64];
myMPC_FLOAT* myMPC_z13 = myMPC_z + 130;
myMPC_FLOAT* myMPC_dzaff13 = myMPC_dz_aff + 130;
myMPC_FLOAT* myMPC_dzcc13 = myMPC_dz_cc + 130;
myMPC_FLOAT myMPC_rd13[10];
myMPC_FLOAT myMPC_Lbyrd13[10];
myMPC_FLOAT myMPC_grad_cost13[10];
myMPC_FLOAT myMPC_grad_eq13[10];
myMPC_FLOAT myMPC_grad_ineq13[10];
myMPC_FLOAT myMPC_ctv13[10];
myMPC_FLOAT myMPC_Phi13[55];
myMPC_FLOAT* myMPC_v13 = myMPC_v + 112;
myMPC_FLOAT myMPC_re13[8];
myMPC_FLOAT myMPC_beta13[8];
myMPC_FLOAT myMPC_betacc13[8];
myMPC_FLOAT* myMPC_dvaff13 = myMPC_dv_aff + 112;
myMPC_FLOAT* myMPC_dvcc13 = myMPC_dv_cc + 112;
myMPC_FLOAT myMPC_V13[80];
myMPC_FLOAT myMPC_Yd13[36];
myMPC_FLOAT myMPC_Ld13[36];
myMPC_FLOAT myMPC_yy13[8];
myMPC_FLOAT myMPC_bmy13[8];
myMPC_FLOAT myMPC_c13[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W13[80];
myMPC_FLOAT myMPC_lb13[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx13[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb13 = myMPC_l + 260;
myMPC_FLOAT* myMPC_slb13 = myMPC_s + 260;
myMPC_FLOAT* myMPC_llbbyslb13 = myMPC_lbys + 260;
myMPC_FLOAT myMPC_rilb13[10];
myMPC_FLOAT* myMPC_dllbaff13 = myMPC_dl_aff + 260;
myMPC_FLOAT* myMPC_dslbaff13 = myMPC_ds_aff + 260;
myMPC_FLOAT* myMPC_dllbcc13 = myMPC_dl_cc + 260;
myMPC_FLOAT* myMPC_dslbcc13 = myMPC_ds_cc + 260;
myMPC_FLOAT* myMPC_ccrhsl13 = myMPC_ccrhs + 260;
myMPC_FLOAT myMPC_ub13[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx13[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub13 = myMPC_l + 270;
myMPC_FLOAT* myMPC_sub13 = myMPC_s + 270;
myMPC_FLOAT* myMPC_lubbysub13 = myMPC_lbys + 270;
myMPC_FLOAT myMPC_riub13[10];
myMPC_FLOAT* myMPC_dlubaff13 = myMPC_dl_aff + 270;
myMPC_FLOAT* myMPC_dsubaff13 = myMPC_ds_aff + 270;
myMPC_FLOAT* myMPC_dlubcc13 = myMPC_dl_cc + 270;
myMPC_FLOAT* myMPC_dsubcc13 = myMPC_ds_cc + 270;
myMPC_FLOAT* myMPC_ccrhsub13 = myMPC_ccrhs + 270;
myMPC_FLOAT myMPC_Ysd13[64];
myMPC_FLOAT myMPC_Lsd13[64];
myMPC_FLOAT* myMPC_z14 = myMPC_z + 140;
myMPC_FLOAT* myMPC_dzaff14 = myMPC_dz_aff + 140;
myMPC_FLOAT* myMPC_dzcc14 = myMPC_dz_cc + 140;
myMPC_FLOAT myMPC_rd14[10];
myMPC_FLOAT myMPC_Lbyrd14[10];
myMPC_FLOAT myMPC_grad_cost14[10];
myMPC_FLOAT myMPC_grad_eq14[10];
myMPC_FLOAT myMPC_grad_ineq14[10];
myMPC_FLOAT myMPC_ctv14[10];
myMPC_FLOAT myMPC_Phi14[55];
myMPC_FLOAT* myMPC_v14 = myMPC_v + 120;
myMPC_FLOAT myMPC_re14[8];
myMPC_FLOAT myMPC_beta14[8];
myMPC_FLOAT myMPC_betacc14[8];
myMPC_FLOAT* myMPC_dvaff14 = myMPC_dv_aff + 120;
myMPC_FLOAT* myMPC_dvcc14 = myMPC_dv_cc + 120;
myMPC_FLOAT myMPC_V14[80];
myMPC_FLOAT myMPC_Yd14[36];
myMPC_FLOAT myMPC_Ld14[36];
myMPC_FLOAT myMPC_yy14[8];
myMPC_FLOAT myMPC_bmy14[8];
myMPC_FLOAT myMPC_c14[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W14[80];
myMPC_FLOAT myMPC_lb14[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx14[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb14 = myMPC_l + 280;
myMPC_FLOAT* myMPC_slb14 = myMPC_s + 280;
myMPC_FLOAT* myMPC_llbbyslb14 = myMPC_lbys + 280;
myMPC_FLOAT myMPC_rilb14[10];
myMPC_FLOAT* myMPC_dllbaff14 = myMPC_dl_aff + 280;
myMPC_FLOAT* myMPC_dslbaff14 = myMPC_ds_aff + 280;
myMPC_FLOAT* myMPC_dllbcc14 = myMPC_dl_cc + 280;
myMPC_FLOAT* myMPC_dslbcc14 = myMPC_ds_cc + 280;
myMPC_FLOAT* myMPC_ccrhsl14 = myMPC_ccrhs + 280;
myMPC_FLOAT myMPC_ub14[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx14[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub14 = myMPC_l + 290;
myMPC_FLOAT* myMPC_sub14 = myMPC_s + 290;
myMPC_FLOAT* myMPC_lubbysub14 = myMPC_lbys + 290;
myMPC_FLOAT myMPC_riub14[10];
myMPC_FLOAT* myMPC_dlubaff14 = myMPC_dl_aff + 290;
myMPC_FLOAT* myMPC_dsubaff14 = myMPC_ds_aff + 290;
myMPC_FLOAT* myMPC_dlubcc14 = myMPC_dl_cc + 290;
myMPC_FLOAT* myMPC_dsubcc14 = myMPC_ds_cc + 290;
myMPC_FLOAT* myMPC_ccrhsub14 = myMPC_ccrhs + 290;
myMPC_FLOAT myMPC_Ysd14[64];
myMPC_FLOAT myMPC_Lsd14[64];
myMPC_FLOAT* myMPC_z15 = myMPC_z + 150;
myMPC_FLOAT* myMPC_dzaff15 = myMPC_dz_aff + 150;
myMPC_FLOAT* myMPC_dzcc15 = myMPC_dz_cc + 150;
myMPC_FLOAT myMPC_rd15[10];
myMPC_FLOAT myMPC_Lbyrd15[10];
myMPC_FLOAT myMPC_grad_cost15[10];
myMPC_FLOAT myMPC_grad_eq15[10];
myMPC_FLOAT myMPC_grad_ineq15[10];
myMPC_FLOAT myMPC_ctv15[10];
myMPC_FLOAT myMPC_Phi15[55];
myMPC_FLOAT* myMPC_v15 = myMPC_v + 128;
myMPC_FLOAT myMPC_re15[8];
myMPC_FLOAT myMPC_beta15[8];
myMPC_FLOAT myMPC_betacc15[8];
myMPC_FLOAT* myMPC_dvaff15 = myMPC_dv_aff + 128;
myMPC_FLOAT* myMPC_dvcc15 = myMPC_dv_cc + 128;
myMPC_FLOAT myMPC_V15[80];
myMPC_FLOAT myMPC_Yd15[36];
myMPC_FLOAT myMPC_Ld15[36];
myMPC_FLOAT myMPC_yy15[8];
myMPC_FLOAT myMPC_bmy15[8];
myMPC_FLOAT myMPC_c15[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W15[80];
myMPC_FLOAT myMPC_lb15[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx15[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb15 = myMPC_l + 300;
myMPC_FLOAT* myMPC_slb15 = myMPC_s + 300;
myMPC_FLOAT* myMPC_llbbyslb15 = myMPC_lbys + 300;
myMPC_FLOAT myMPC_rilb15[10];
myMPC_FLOAT* myMPC_dllbaff15 = myMPC_dl_aff + 300;
myMPC_FLOAT* myMPC_dslbaff15 = myMPC_ds_aff + 300;
myMPC_FLOAT* myMPC_dllbcc15 = myMPC_dl_cc + 300;
myMPC_FLOAT* myMPC_dslbcc15 = myMPC_ds_cc + 300;
myMPC_FLOAT* myMPC_ccrhsl15 = myMPC_ccrhs + 300;
myMPC_FLOAT myMPC_ub15[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx15[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub15 = myMPC_l + 310;
myMPC_FLOAT* myMPC_sub15 = myMPC_s + 310;
myMPC_FLOAT* myMPC_lubbysub15 = myMPC_lbys + 310;
myMPC_FLOAT myMPC_riub15[10];
myMPC_FLOAT* myMPC_dlubaff15 = myMPC_dl_aff + 310;
myMPC_FLOAT* myMPC_dsubaff15 = myMPC_ds_aff + 310;
myMPC_FLOAT* myMPC_dlubcc15 = myMPC_dl_cc + 310;
myMPC_FLOAT* myMPC_dsubcc15 = myMPC_ds_cc + 310;
myMPC_FLOAT* myMPC_ccrhsub15 = myMPC_ccrhs + 310;
myMPC_FLOAT myMPC_Ysd15[64];
myMPC_FLOAT myMPC_Lsd15[64];
myMPC_FLOAT* myMPC_z16 = myMPC_z + 160;
myMPC_FLOAT* myMPC_dzaff16 = myMPC_dz_aff + 160;
myMPC_FLOAT* myMPC_dzcc16 = myMPC_dz_cc + 160;
myMPC_FLOAT myMPC_rd16[10];
myMPC_FLOAT myMPC_Lbyrd16[10];
myMPC_FLOAT myMPC_grad_cost16[10];
myMPC_FLOAT myMPC_grad_eq16[10];
myMPC_FLOAT myMPC_grad_ineq16[10];
myMPC_FLOAT myMPC_ctv16[10];
myMPC_FLOAT myMPC_Phi16[55];
myMPC_FLOAT* myMPC_v16 = myMPC_v + 136;
myMPC_FLOAT myMPC_re16[8];
myMPC_FLOAT myMPC_beta16[8];
myMPC_FLOAT myMPC_betacc16[8];
myMPC_FLOAT* myMPC_dvaff16 = myMPC_dv_aff + 136;
myMPC_FLOAT* myMPC_dvcc16 = myMPC_dv_cc + 136;
myMPC_FLOAT myMPC_V16[80];
myMPC_FLOAT myMPC_Yd16[36];
myMPC_FLOAT myMPC_Ld16[36];
myMPC_FLOAT myMPC_yy16[8];
myMPC_FLOAT myMPC_bmy16[8];
myMPC_FLOAT myMPC_c16[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W16[80];
myMPC_FLOAT myMPC_lb16[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx16[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb16 = myMPC_l + 320;
myMPC_FLOAT* myMPC_slb16 = myMPC_s + 320;
myMPC_FLOAT* myMPC_llbbyslb16 = myMPC_lbys + 320;
myMPC_FLOAT myMPC_rilb16[10];
myMPC_FLOAT* myMPC_dllbaff16 = myMPC_dl_aff + 320;
myMPC_FLOAT* myMPC_dslbaff16 = myMPC_ds_aff + 320;
myMPC_FLOAT* myMPC_dllbcc16 = myMPC_dl_cc + 320;
myMPC_FLOAT* myMPC_dslbcc16 = myMPC_ds_cc + 320;
myMPC_FLOAT* myMPC_ccrhsl16 = myMPC_ccrhs + 320;
myMPC_FLOAT myMPC_ub16[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx16[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub16 = myMPC_l + 330;
myMPC_FLOAT* myMPC_sub16 = myMPC_s + 330;
myMPC_FLOAT* myMPC_lubbysub16 = myMPC_lbys + 330;
myMPC_FLOAT myMPC_riub16[10];
myMPC_FLOAT* myMPC_dlubaff16 = myMPC_dl_aff + 330;
myMPC_FLOAT* myMPC_dsubaff16 = myMPC_ds_aff + 330;
myMPC_FLOAT* myMPC_dlubcc16 = myMPC_dl_cc + 330;
myMPC_FLOAT* myMPC_dsubcc16 = myMPC_ds_cc + 330;
myMPC_FLOAT* myMPC_ccrhsub16 = myMPC_ccrhs + 330;
myMPC_FLOAT myMPC_Ysd16[64];
myMPC_FLOAT myMPC_Lsd16[64];
myMPC_FLOAT* myMPC_z17 = myMPC_z + 170;
myMPC_FLOAT* myMPC_dzaff17 = myMPC_dz_aff + 170;
myMPC_FLOAT* myMPC_dzcc17 = myMPC_dz_cc + 170;
myMPC_FLOAT myMPC_rd17[10];
myMPC_FLOAT myMPC_Lbyrd17[10];
myMPC_FLOAT myMPC_grad_cost17[10];
myMPC_FLOAT myMPC_grad_eq17[10];
myMPC_FLOAT myMPC_grad_ineq17[10];
myMPC_FLOAT myMPC_ctv17[10];
myMPC_FLOAT myMPC_Phi17[55];
myMPC_FLOAT* myMPC_v17 = myMPC_v + 144;
myMPC_FLOAT myMPC_re17[8];
myMPC_FLOAT myMPC_beta17[8];
myMPC_FLOAT myMPC_betacc17[8];
myMPC_FLOAT* myMPC_dvaff17 = myMPC_dv_aff + 144;
myMPC_FLOAT* myMPC_dvcc17 = myMPC_dv_cc + 144;
myMPC_FLOAT myMPC_V17[80];
myMPC_FLOAT myMPC_Yd17[36];
myMPC_FLOAT myMPC_Ld17[36];
myMPC_FLOAT myMPC_yy17[8];
myMPC_FLOAT myMPC_bmy17[8];
myMPC_FLOAT myMPC_c17[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W17[80];
myMPC_FLOAT myMPC_lb17[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx17[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb17 = myMPC_l + 340;
myMPC_FLOAT* myMPC_slb17 = myMPC_s + 340;
myMPC_FLOAT* myMPC_llbbyslb17 = myMPC_lbys + 340;
myMPC_FLOAT myMPC_rilb17[10];
myMPC_FLOAT* myMPC_dllbaff17 = myMPC_dl_aff + 340;
myMPC_FLOAT* myMPC_dslbaff17 = myMPC_ds_aff + 340;
myMPC_FLOAT* myMPC_dllbcc17 = myMPC_dl_cc + 340;
myMPC_FLOAT* myMPC_dslbcc17 = myMPC_ds_cc + 340;
myMPC_FLOAT* myMPC_ccrhsl17 = myMPC_ccrhs + 340;
myMPC_FLOAT myMPC_ub17[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx17[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub17 = myMPC_l + 350;
myMPC_FLOAT* myMPC_sub17 = myMPC_s + 350;
myMPC_FLOAT* myMPC_lubbysub17 = myMPC_lbys + 350;
myMPC_FLOAT myMPC_riub17[10];
myMPC_FLOAT* myMPC_dlubaff17 = myMPC_dl_aff + 350;
myMPC_FLOAT* myMPC_dsubaff17 = myMPC_ds_aff + 350;
myMPC_FLOAT* myMPC_dlubcc17 = myMPC_dl_cc + 350;
myMPC_FLOAT* myMPC_dsubcc17 = myMPC_ds_cc + 350;
myMPC_FLOAT* myMPC_ccrhsub17 = myMPC_ccrhs + 350;
myMPC_FLOAT myMPC_Ysd17[64];
myMPC_FLOAT myMPC_Lsd17[64];
myMPC_FLOAT* myMPC_z18 = myMPC_z + 180;
myMPC_FLOAT* myMPC_dzaff18 = myMPC_dz_aff + 180;
myMPC_FLOAT* myMPC_dzcc18 = myMPC_dz_cc + 180;
myMPC_FLOAT myMPC_rd18[10];
myMPC_FLOAT myMPC_Lbyrd18[10];
myMPC_FLOAT myMPC_grad_cost18[10];
myMPC_FLOAT myMPC_grad_eq18[10];
myMPC_FLOAT myMPC_grad_ineq18[10];
myMPC_FLOAT myMPC_ctv18[10];
myMPC_FLOAT myMPC_Phi18[55];
myMPC_FLOAT* myMPC_v18 = myMPC_v + 152;
myMPC_FLOAT myMPC_re18[8];
myMPC_FLOAT myMPC_beta18[8];
myMPC_FLOAT myMPC_betacc18[8];
myMPC_FLOAT* myMPC_dvaff18 = myMPC_dv_aff + 152;
myMPC_FLOAT* myMPC_dvcc18 = myMPC_dv_cc + 152;
myMPC_FLOAT myMPC_V18[80];
myMPC_FLOAT myMPC_Yd18[36];
myMPC_FLOAT myMPC_Ld18[36];
myMPC_FLOAT myMPC_yy18[8];
myMPC_FLOAT myMPC_bmy18[8];
myMPC_FLOAT myMPC_c18[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W18[80];
myMPC_FLOAT myMPC_lb18[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx18[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb18 = myMPC_l + 360;
myMPC_FLOAT* myMPC_slb18 = myMPC_s + 360;
myMPC_FLOAT* myMPC_llbbyslb18 = myMPC_lbys + 360;
myMPC_FLOAT myMPC_rilb18[10];
myMPC_FLOAT* myMPC_dllbaff18 = myMPC_dl_aff + 360;
myMPC_FLOAT* myMPC_dslbaff18 = myMPC_ds_aff + 360;
myMPC_FLOAT* myMPC_dllbcc18 = myMPC_dl_cc + 360;
myMPC_FLOAT* myMPC_dslbcc18 = myMPC_ds_cc + 360;
myMPC_FLOAT* myMPC_ccrhsl18 = myMPC_ccrhs + 360;
myMPC_FLOAT myMPC_ub18[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx18[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub18 = myMPC_l + 370;
myMPC_FLOAT* myMPC_sub18 = myMPC_s + 370;
myMPC_FLOAT* myMPC_lubbysub18 = myMPC_lbys + 370;
myMPC_FLOAT myMPC_riub18[10];
myMPC_FLOAT* myMPC_dlubaff18 = myMPC_dl_aff + 370;
myMPC_FLOAT* myMPC_dsubaff18 = myMPC_ds_aff + 370;
myMPC_FLOAT* myMPC_dlubcc18 = myMPC_dl_cc + 370;
myMPC_FLOAT* myMPC_dsubcc18 = myMPC_ds_cc + 370;
myMPC_FLOAT* myMPC_ccrhsub18 = myMPC_ccrhs + 370;
myMPC_FLOAT myMPC_Ysd18[64];
myMPC_FLOAT myMPC_Lsd18[64];
myMPC_FLOAT* myMPC_z19 = myMPC_z + 190;
myMPC_FLOAT* myMPC_dzaff19 = myMPC_dz_aff + 190;
myMPC_FLOAT* myMPC_dzcc19 = myMPC_dz_cc + 190;
myMPC_FLOAT myMPC_rd19[10];
myMPC_FLOAT myMPC_Lbyrd19[10];
myMPC_FLOAT myMPC_grad_cost19[10];
myMPC_FLOAT myMPC_grad_eq19[10];
myMPC_FLOAT myMPC_grad_ineq19[10];
myMPC_FLOAT myMPC_ctv19[10];
myMPC_FLOAT myMPC_Phi19[55];
myMPC_FLOAT* myMPC_v19 = myMPC_v + 160;
myMPC_FLOAT myMPC_re19[8];
myMPC_FLOAT myMPC_beta19[8];
myMPC_FLOAT myMPC_betacc19[8];
myMPC_FLOAT* myMPC_dvaff19 = myMPC_dv_aff + 160;
myMPC_FLOAT* myMPC_dvcc19 = myMPC_dv_cc + 160;
myMPC_FLOAT myMPC_V19[80];
myMPC_FLOAT myMPC_Yd19[36];
myMPC_FLOAT myMPC_Ld19[36];
myMPC_FLOAT myMPC_yy19[8];
myMPC_FLOAT myMPC_bmy19[8];
myMPC_FLOAT myMPC_c19[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W19[80];
myMPC_FLOAT myMPC_lb19[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx19[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb19 = myMPC_l + 380;
myMPC_FLOAT* myMPC_slb19 = myMPC_s + 380;
myMPC_FLOAT* myMPC_llbbyslb19 = myMPC_lbys + 380;
myMPC_FLOAT myMPC_rilb19[10];
myMPC_FLOAT* myMPC_dllbaff19 = myMPC_dl_aff + 380;
myMPC_FLOAT* myMPC_dslbaff19 = myMPC_ds_aff + 380;
myMPC_FLOAT* myMPC_dllbcc19 = myMPC_dl_cc + 380;
myMPC_FLOAT* myMPC_dslbcc19 = myMPC_ds_cc + 380;
myMPC_FLOAT* myMPC_ccrhsl19 = myMPC_ccrhs + 380;
myMPC_FLOAT myMPC_ub19[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx19[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub19 = myMPC_l + 390;
myMPC_FLOAT* myMPC_sub19 = myMPC_s + 390;
myMPC_FLOAT* myMPC_lubbysub19 = myMPC_lbys + 390;
myMPC_FLOAT myMPC_riub19[10];
myMPC_FLOAT* myMPC_dlubaff19 = myMPC_dl_aff + 390;
myMPC_FLOAT* myMPC_dsubaff19 = myMPC_ds_aff + 390;
myMPC_FLOAT* myMPC_dlubcc19 = myMPC_dl_cc + 390;
myMPC_FLOAT* myMPC_dsubcc19 = myMPC_ds_cc + 390;
myMPC_FLOAT* myMPC_ccrhsub19 = myMPC_ccrhs + 390;
myMPC_FLOAT myMPC_Ysd19[64];
myMPC_FLOAT myMPC_Lsd19[64];
myMPC_FLOAT* myMPC_z20 = myMPC_z + 200;
myMPC_FLOAT* myMPC_dzaff20 = myMPC_dz_aff + 200;
myMPC_FLOAT* myMPC_dzcc20 = myMPC_dz_cc + 200;
myMPC_FLOAT myMPC_rd20[10];
myMPC_FLOAT myMPC_Lbyrd20[10];
myMPC_FLOAT myMPC_grad_cost20[10];
myMPC_FLOAT myMPC_grad_eq20[10];
myMPC_FLOAT myMPC_grad_ineq20[10];
myMPC_FLOAT myMPC_ctv20[10];
myMPC_FLOAT myMPC_Phi20[55];
myMPC_FLOAT* myMPC_v20 = myMPC_v + 168;
myMPC_FLOAT myMPC_re20[8];
myMPC_FLOAT myMPC_beta20[8];
myMPC_FLOAT myMPC_betacc20[8];
myMPC_FLOAT* myMPC_dvaff20 = myMPC_dv_aff + 168;
myMPC_FLOAT* myMPC_dvcc20 = myMPC_dv_cc + 168;
myMPC_FLOAT myMPC_V20[80];
myMPC_FLOAT myMPC_Yd20[36];
myMPC_FLOAT myMPC_Ld20[36];
myMPC_FLOAT myMPC_yy20[8];
myMPC_FLOAT myMPC_bmy20[8];
myMPC_FLOAT myMPC_c20[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W20[80];
myMPC_FLOAT myMPC_lb20[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx20[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb20 = myMPC_l + 400;
myMPC_FLOAT* myMPC_slb20 = myMPC_s + 400;
myMPC_FLOAT* myMPC_llbbyslb20 = myMPC_lbys + 400;
myMPC_FLOAT myMPC_rilb20[10];
myMPC_FLOAT* myMPC_dllbaff20 = myMPC_dl_aff + 400;
myMPC_FLOAT* myMPC_dslbaff20 = myMPC_ds_aff + 400;
myMPC_FLOAT* myMPC_dllbcc20 = myMPC_dl_cc + 400;
myMPC_FLOAT* myMPC_dslbcc20 = myMPC_ds_cc + 400;
myMPC_FLOAT* myMPC_ccrhsl20 = myMPC_ccrhs + 400;
myMPC_FLOAT myMPC_ub20[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx20[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub20 = myMPC_l + 410;
myMPC_FLOAT* myMPC_sub20 = myMPC_s + 410;
myMPC_FLOAT* myMPC_lubbysub20 = myMPC_lbys + 410;
myMPC_FLOAT myMPC_riub20[10];
myMPC_FLOAT* myMPC_dlubaff20 = myMPC_dl_aff + 410;
myMPC_FLOAT* myMPC_dsubaff20 = myMPC_ds_aff + 410;
myMPC_FLOAT* myMPC_dlubcc20 = myMPC_dl_cc + 410;
myMPC_FLOAT* myMPC_dsubcc20 = myMPC_ds_cc + 410;
myMPC_FLOAT* myMPC_ccrhsub20 = myMPC_ccrhs + 410;
myMPC_FLOAT myMPC_Ysd20[64];
myMPC_FLOAT myMPC_Lsd20[64];
myMPC_FLOAT* myMPC_z21 = myMPC_z + 210;
myMPC_FLOAT* myMPC_dzaff21 = myMPC_dz_aff + 210;
myMPC_FLOAT* myMPC_dzcc21 = myMPC_dz_cc + 210;
myMPC_FLOAT myMPC_rd21[10];
myMPC_FLOAT myMPC_Lbyrd21[10];
myMPC_FLOAT myMPC_grad_cost21[10];
myMPC_FLOAT myMPC_grad_eq21[10];
myMPC_FLOAT myMPC_grad_ineq21[10];
myMPC_FLOAT myMPC_ctv21[10];
myMPC_FLOAT myMPC_Phi21[55];
myMPC_FLOAT* myMPC_v21 = myMPC_v + 176;
myMPC_FLOAT myMPC_re21[8];
myMPC_FLOAT myMPC_beta21[8];
myMPC_FLOAT myMPC_betacc21[8];
myMPC_FLOAT* myMPC_dvaff21 = myMPC_dv_aff + 176;
myMPC_FLOAT* myMPC_dvcc21 = myMPC_dv_cc + 176;
myMPC_FLOAT myMPC_V21[80];
myMPC_FLOAT myMPC_Yd21[36];
myMPC_FLOAT myMPC_Ld21[36];
myMPC_FLOAT myMPC_yy21[8];
myMPC_FLOAT myMPC_bmy21[8];
myMPC_FLOAT myMPC_c21[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W21[80];
myMPC_FLOAT myMPC_lb21[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx21[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb21 = myMPC_l + 420;
myMPC_FLOAT* myMPC_slb21 = myMPC_s + 420;
myMPC_FLOAT* myMPC_llbbyslb21 = myMPC_lbys + 420;
myMPC_FLOAT myMPC_rilb21[10];
myMPC_FLOAT* myMPC_dllbaff21 = myMPC_dl_aff + 420;
myMPC_FLOAT* myMPC_dslbaff21 = myMPC_ds_aff + 420;
myMPC_FLOAT* myMPC_dllbcc21 = myMPC_dl_cc + 420;
myMPC_FLOAT* myMPC_dslbcc21 = myMPC_ds_cc + 420;
myMPC_FLOAT* myMPC_ccrhsl21 = myMPC_ccrhs + 420;
myMPC_FLOAT myMPC_ub21[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx21[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub21 = myMPC_l + 430;
myMPC_FLOAT* myMPC_sub21 = myMPC_s + 430;
myMPC_FLOAT* myMPC_lubbysub21 = myMPC_lbys + 430;
myMPC_FLOAT myMPC_riub21[10];
myMPC_FLOAT* myMPC_dlubaff21 = myMPC_dl_aff + 430;
myMPC_FLOAT* myMPC_dsubaff21 = myMPC_ds_aff + 430;
myMPC_FLOAT* myMPC_dlubcc21 = myMPC_dl_cc + 430;
myMPC_FLOAT* myMPC_dsubcc21 = myMPC_ds_cc + 430;
myMPC_FLOAT* myMPC_ccrhsub21 = myMPC_ccrhs + 430;
myMPC_FLOAT myMPC_Ysd21[64];
myMPC_FLOAT myMPC_Lsd21[64];
myMPC_FLOAT* myMPC_z22 = myMPC_z + 220;
myMPC_FLOAT* myMPC_dzaff22 = myMPC_dz_aff + 220;
myMPC_FLOAT* myMPC_dzcc22 = myMPC_dz_cc + 220;
myMPC_FLOAT myMPC_rd22[10];
myMPC_FLOAT myMPC_Lbyrd22[10];
myMPC_FLOAT myMPC_grad_cost22[10];
myMPC_FLOAT myMPC_grad_eq22[10];
myMPC_FLOAT myMPC_grad_ineq22[10];
myMPC_FLOAT myMPC_ctv22[10];
myMPC_FLOAT myMPC_Phi22[55];
myMPC_FLOAT* myMPC_v22 = myMPC_v + 184;
myMPC_FLOAT myMPC_re22[8];
myMPC_FLOAT myMPC_beta22[8];
myMPC_FLOAT myMPC_betacc22[8];
myMPC_FLOAT* myMPC_dvaff22 = myMPC_dv_aff + 184;
myMPC_FLOAT* myMPC_dvcc22 = myMPC_dv_cc + 184;
myMPC_FLOAT myMPC_V22[80];
myMPC_FLOAT myMPC_Yd22[36];
myMPC_FLOAT myMPC_Ld22[36];
myMPC_FLOAT myMPC_yy22[8];
myMPC_FLOAT myMPC_bmy22[8];
myMPC_FLOAT myMPC_c22[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W22[80];
myMPC_FLOAT myMPC_lb22[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx22[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb22 = myMPC_l + 440;
myMPC_FLOAT* myMPC_slb22 = myMPC_s + 440;
myMPC_FLOAT* myMPC_llbbyslb22 = myMPC_lbys + 440;
myMPC_FLOAT myMPC_rilb22[10];
myMPC_FLOAT* myMPC_dllbaff22 = myMPC_dl_aff + 440;
myMPC_FLOAT* myMPC_dslbaff22 = myMPC_ds_aff + 440;
myMPC_FLOAT* myMPC_dllbcc22 = myMPC_dl_cc + 440;
myMPC_FLOAT* myMPC_dslbcc22 = myMPC_ds_cc + 440;
myMPC_FLOAT* myMPC_ccrhsl22 = myMPC_ccrhs + 440;
myMPC_FLOAT myMPC_ub22[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx22[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub22 = myMPC_l + 450;
myMPC_FLOAT* myMPC_sub22 = myMPC_s + 450;
myMPC_FLOAT* myMPC_lubbysub22 = myMPC_lbys + 450;
myMPC_FLOAT myMPC_riub22[10];
myMPC_FLOAT* myMPC_dlubaff22 = myMPC_dl_aff + 450;
myMPC_FLOAT* myMPC_dsubaff22 = myMPC_ds_aff + 450;
myMPC_FLOAT* myMPC_dlubcc22 = myMPC_dl_cc + 450;
myMPC_FLOAT* myMPC_dsubcc22 = myMPC_ds_cc + 450;
myMPC_FLOAT* myMPC_ccrhsub22 = myMPC_ccrhs + 450;
myMPC_FLOAT myMPC_Ysd22[64];
myMPC_FLOAT myMPC_Lsd22[64];
myMPC_FLOAT* myMPC_z23 = myMPC_z + 230;
myMPC_FLOAT* myMPC_dzaff23 = myMPC_dz_aff + 230;
myMPC_FLOAT* myMPC_dzcc23 = myMPC_dz_cc + 230;
myMPC_FLOAT myMPC_rd23[10];
myMPC_FLOAT myMPC_Lbyrd23[10];
myMPC_FLOAT myMPC_grad_cost23[10];
myMPC_FLOAT myMPC_grad_eq23[10];
myMPC_FLOAT myMPC_grad_ineq23[10];
myMPC_FLOAT myMPC_ctv23[10];
myMPC_FLOAT myMPC_Phi23[55];
myMPC_FLOAT* myMPC_v23 = myMPC_v + 192;
myMPC_FLOAT myMPC_re23[8];
myMPC_FLOAT myMPC_beta23[8];
myMPC_FLOAT myMPC_betacc23[8];
myMPC_FLOAT* myMPC_dvaff23 = myMPC_dv_aff + 192;
myMPC_FLOAT* myMPC_dvcc23 = myMPC_dv_cc + 192;
myMPC_FLOAT myMPC_V23[80];
myMPC_FLOAT myMPC_Yd23[36];
myMPC_FLOAT myMPC_Ld23[36];
myMPC_FLOAT myMPC_yy23[8];
myMPC_FLOAT myMPC_bmy23[8];
myMPC_FLOAT myMPC_c23[8] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
myMPC_FLOAT myMPC_W23[80];
myMPC_FLOAT myMPC_lb23[10] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx23[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_llb23 = myMPC_l + 460;
myMPC_FLOAT* myMPC_slb23 = myMPC_s + 460;
myMPC_FLOAT* myMPC_llbbyslb23 = myMPC_lbys + 460;
myMPC_FLOAT myMPC_rilb23[10];
myMPC_FLOAT* myMPC_dllbaff23 = myMPC_dl_aff + 460;
myMPC_FLOAT* myMPC_dslbaff23 = myMPC_ds_aff + 460;
myMPC_FLOAT* myMPC_dllbcc23 = myMPC_dl_cc + 460;
myMPC_FLOAT* myMPC_dslbcc23 = myMPC_ds_cc + 460;
myMPC_FLOAT* myMPC_ccrhsl23 = myMPC_ccrhs + 460;
myMPC_FLOAT myMPC_ub23[10] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx23[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
myMPC_FLOAT* myMPC_lub23 = myMPC_l + 470;
myMPC_FLOAT* myMPC_sub23 = myMPC_s + 470;
myMPC_FLOAT* myMPC_lubbysub23 = myMPC_lbys + 470;
myMPC_FLOAT myMPC_riub23[10];
myMPC_FLOAT* myMPC_dlubaff23 = myMPC_dl_aff + 470;
myMPC_FLOAT* myMPC_dsubaff23 = myMPC_ds_aff + 470;
myMPC_FLOAT* myMPC_dlubcc23 = myMPC_dl_cc + 470;
myMPC_FLOAT* myMPC_dsubcc23 = myMPC_ds_cc + 470;
myMPC_FLOAT* myMPC_ccrhsub23 = myMPC_ccrhs + 470;
myMPC_FLOAT myMPC_Ysd23[64];
myMPC_FLOAT myMPC_Lsd23[64];
myMPC_FLOAT* myMPC_z24 = myMPC_z + 240;
myMPC_FLOAT* myMPC_dzaff24 = myMPC_dz_aff + 240;
myMPC_FLOAT* myMPC_dzcc24 = myMPC_dz_cc + 240;
myMPC_FLOAT myMPC_rd24[8];
myMPC_FLOAT myMPC_Lbyrd24[8];
myMPC_FLOAT myMPC_grad_cost24[8];
myMPC_FLOAT myMPC_grad_eq24[8];
myMPC_FLOAT myMPC_grad_ineq24[8];
myMPC_FLOAT myMPC_ctv24[8];
myMPC_FLOAT myMPC_Phi24[36];
myMPC_FLOAT myMPC_D24[64] = {-1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000};
myMPC_FLOAT myMPC_W24[64];
myMPC_FLOAT myMPC_lb24[8] = {-1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002, -1.0000000000000000E+002};
int myMPC_lbIdx24[8] = {0, 1, 2, 3, 4, 5, 6, 7};
myMPC_FLOAT* myMPC_llb24 = myMPC_l + 480;
myMPC_FLOAT* myMPC_slb24 = myMPC_s + 480;
myMPC_FLOAT* myMPC_llbbyslb24 = myMPC_lbys + 480;
myMPC_FLOAT myMPC_rilb24[8];
myMPC_FLOAT* myMPC_dllbaff24 = myMPC_dl_aff + 480;
myMPC_FLOAT* myMPC_dslbaff24 = myMPC_ds_aff + 480;
myMPC_FLOAT* myMPC_dllbcc24 = myMPC_dl_cc + 480;
myMPC_FLOAT* myMPC_dslbcc24 = myMPC_ds_cc + 480;
myMPC_FLOAT* myMPC_ccrhsl24 = myMPC_ccrhs + 480;
myMPC_FLOAT myMPC_ub24[8] = {1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002, 1.0000000000000000E+002};
int myMPC_ubIdx24[8] = {0, 1, 2, 3, 4, 5, 6, 7};
myMPC_FLOAT* myMPC_lub24 = myMPC_l + 488;
myMPC_FLOAT* myMPC_sub24 = myMPC_s + 488;
myMPC_FLOAT* myMPC_lubbysub24 = myMPC_lbys + 488;
myMPC_FLOAT myMPC_riub24[8];
myMPC_FLOAT* myMPC_dlubaff24 = myMPC_dl_aff + 488;
myMPC_FLOAT* myMPC_dsubaff24 = myMPC_ds_aff + 488;
myMPC_FLOAT* myMPC_dlubcc24 = myMPC_dl_cc + 488;
myMPC_FLOAT* myMPC_dsubcc24 = myMPC_ds_cc + 488;
myMPC_FLOAT* myMPC_ccrhsub24 = myMPC_ccrhs + 488;
myMPC_FLOAT musigma;
myMPC_FLOAT sigma_3rdroot;


/* SOLVER CODE --------------------------------------------------------- */
int myMPC_solve(myMPC_params* params, myMPC_output* output, myMPC_info* info)
{	
info->it = 0;
myMPC_LA_INITIALIZEVECTOR_248(myMPC_z, 0);
myMPC_LA_INITIALIZEVECTOR_200(myMPC_v, 1);
myMPC_LA_INITIALIZEVECTOR_496(myMPC_l, 1);
myMPC_LA_INITIALIZEVECTOR_496(myMPC_s, 1);
info->mu = 0;
myMPC_LA_DOTACC_496(myMPC_l, myMPC_s, &info->mu);
info->mu /= 496;
PRINTTEXT("This is myMPC, a solver generated by FORCES.\n");
PRINTTEXT("(c) Alexander Domahidi, Automatic Control Laboratory, ETH Zurich, 2011-2012.\n");
PRINTTEXT("\n  #it  res_eq   res_ineq    pobj         dobj       dgap     rdgap      mu\n");
PRINTTEXT("  --------------------------------------------------------------------------\n");
while( 1 ){
info->pobj = 0;
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z00, myMPC_grad_cost00, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z01, myMPC_grad_cost01, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z02, myMPC_grad_cost02, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z03, myMPC_grad_cost03, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z04, myMPC_grad_cost04, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z05, myMPC_grad_cost05, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z06, myMPC_grad_cost06, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z07, myMPC_grad_cost07, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z08, myMPC_grad_cost08, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z09, myMPC_grad_cost09, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z10, myMPC_grad_cost10, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z11, myMPC_grad_cost11, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z12, myMPC_grad_cost12, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z13, myMPC_grad_cost13, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z14, myMPC_grad_cost14, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z15, myMPC_grad_cost15, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z16, myMPC_grad_cost16, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z17, myMPC_grad_cost17, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z18, myMPC_grad_cost18, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z19, myMPC_grad_cost19, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z20, myMPC_grad_cost20, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z21, myMPC_grad_cost21, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z22, myMPC_grad_cost22, &info->pobj);
myMPC_LA_DENSE_QUADFCN_10(params->H, params->f, myMPC_z23, myMPC_grad_cost23, &info->pobj);
myMPC_LA_DENSE_QUADFCN_8(params->H_N, params->f_N, myMPC_z24, myMPC_grad_cost24, &info->pobj);
info->dgap = 0;
info->res_ineq = 0;
myMPC_LA_VSUBADD3_10(myMPC_lb00, myMPC_z00, myMPC_lbIdx00, myMPC_llb00, myMPC_slb00, myMPC_rilb00, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z00, myMPC_ubIdx00, myMPC_ub00, myMPC_lub00, myMPC_sub00, myMPC_riub00, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb01, myMPC_z01, myMPC_lbIdx01, myMPC_llb01, myMPC_slb01, myMPC_rilb01, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z01, myMPC_ubIdx01, myMPC_ub01, myMPC_lub01, myMPC_sub01, myMPC_riub01, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb02, myMPC_z02, myMPC_lbIdx02, myMPC_llb02, myMPC_slb02, myMPC_rilb02, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z02, myMPC_ubIdx02, myMPC_ub02, myMPC_lub02, myMPC_sub02, myMPC_riub02, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb03, myMPC_z03, myMPC_lbIdx03, myMPC_llb03, myMPC_slb03, myMPC_rilb03, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z03, myMPC_ubIdx03, myMPC_ub03, myMPC_lub03, myMPC_sub03, myMPC_riub03, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb04, myMPC_z04, myMPC_lbIdx04, myMPC_llb04, myMPC_slb04, myMPC_rilb04, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z04, myMPC_ubIdx04, myMPC_ub04, myMPC_lub04, myMPC_sub04, myMPC_riub04, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb05, myMPC_z05, myMPC_lbIdx05, myMPC_llb05, myMPC_slb05, myMPC_rilb05, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z05, myMPC_ubIdx05, myMPC_ub05, myMPC_lub05, myMPC_sub05, myMPC_riub05, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb06, myMPC_z06, myMPC_lbIdx06, myMPC_llb06, myMPC_slb06, myMPC_rilb06, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z06, myMPC_ubIdx06, myMPC_ub06, myMPC_lub06, myMPC_sub06, myMPC_riub06, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb07, myMPC_z07, myMPC_lbIdx07, myMPC_llb07, myMPC_slb07, myMPC_rilb07, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z07, myMPC_ubIdx07, myMPC_ub07, myMPC_lub07, myMPC_sub07, myMPC_riub07, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb08, myMPC_z08, myMPC_lbIdx08, myMPC_llb08, myMPC_slb08, myMPC_rilb08, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z08, myMPC_ubIdx08, myMPC_ub08, myMPC_lub08, myMPC_sub08, myMPC_riub08, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb09, myMPC_z09, myMPC_lbIdx09, myMPC_llb09, myMPC_slb09, myMPC_rilb09, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z09, myMPC_ubIdx09, myMPC_ub09, myMPC_lub09, myMPC_sub09, myMPC_riub09, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb10, myMPC_z10, myMPC_lbIdx10, myMPC_llb10, myMPC_slb10, myMPC_rilb10, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z10, myMPC_ubIdx10, myMPC_ub10, myMPC_lub10, myMPC_sub10, myMPC_riub10, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb11, myMPC_z11, myMPC_lbIdx11, myMPC_llb11, myMPC_slb11, myMPC_rilb11, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z11, myMPC_ubIdx11, myMPC_ub11, myMPC_lub11, myMPC_sub11, myMPC_riub11, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb12, myMPC_z12, myMPC_lbIdx12, myMPC_llb12, myMPC_slb12, myMPC_rilb12, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z12, myMPC_ubIdx12, myMPC_ub12, myMPC_lub12, myMPC_sub12, myMPC_riub12, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb13, myMPC_z13, myMPC_lbIdx13, myMPC_llb13, myMPC_slb13, myMPC_rilb13, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z13, myMPC_ubIdx13, myMPC_ub13, myMPC_lub13, myMPC_sub13, myMPC_riub13, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb14, myMPC_z14, myMPC_lbIdx14, myMPC_llb14, myMPC_slb14, myMPC_rilb14, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z14, myMPC_ubIdx14, myMPC_ub14, myMPC_lub14, myMPC_sub14, myMPC_riub14, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb15, myMPC_z15, myMPC_lbIdx15, myMPC_llb15, myMPC_slb15, myMPC_rilb15, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z15, myMPC_ubIdx15, myMPC_ub15, myMPC_lub15, myMPC_sub15, myMPC_riub15, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb16, myMPC_z16, myMPC_lbIdx16, myMPC_llb16, myMPC_slb16, myMPC_rilb16, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z16, myMPC_ubIdx16, myMPC_ub16, myMPC_lub16, myMPC_sub16, myMPC_riub16, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb17, myMPC_z17, myMPC_lbIdx17, myMPC_llb17, myMPC_slb17, myMPC_rilb17, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z17, myMPC_ubIdx17, myMPC_ub17, myMPC_lub17, myMPC_sub17, myMPC_riub17, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb18, myMPC_z18, myMPC_lbIdx18, myMPC_llb18, myMPC_slb18, myMPC_rilb18, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z18, myMPC_ubIdx18, myMPC_ub18, myMPC_lub18, myMPC_sub18, myMPC_riub18, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb19, myMPC_z19, myMPC_lbIdx19, myMPC_llb19, myMPC_slb19, myMPC_rilb19, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z19, myMPC_ubIdx19, myMPC_ub19, myMPC_lub19, myMPC_sub19, myMPC_riub19, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb20, myMPC_z20, myMPC_lbIdx20, myMPC_llb20, myMPC_slb20, myMPC_rilb20, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z20, myMPC_ubIdx20, myMPC_ub20, myMPC_lub20, myMPC_sub20, myMPC_riub20, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb21, myMPC_z21, myMPC_lbIdx21, myMPC_llb21, myMPC_slb21, myMPC_rilb21, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z21, myMPC_ubIdx21, myMPC_ub21, myMPC_lub21, myMPC_sub21, myMPC_riub21, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb22, myMPC_z22, myMPC_lbIdx22, myMPC_llb22, myMPC_slb22, myMPC_rilb22, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z22, myMPC_ubIdx22, myMPC_ub22, myMPC_lub22, myMPC_sub22, myMPC_riub22, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_10(myMPC_lb23, myMPC_z23, myMPC_lbIdx23, myMPC_llb23, myMPC_slb23, myMPC_rilb23, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_10(myMPC_z23, myMPC_ubIdx23, myMPC_ub23, myMPC_lub23, myMPC_sub23, myMPC_riub23, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD3_8(myMPC_lb24, myMPC_z24, myMPC_lbIdx24, myMPC_llb24, myMPC_slb24, myMPC_rilb24, &info->dgap, &info->res_ineq);
myMPC_LA_VSUBADD2_8(myMPC_z24, myMPC_ubIdx24, myMPC_ub24, myMPC_lub24, myMPC_sub24, myMPC_riub24, &info->dgap, &info->res_ineq);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub00, myMPC_sub00, myMPC_riub00, myMPC_llb00, myMPC_slb00, myMPC_rilb00, myMPC_lbIdx00, myMPC_ubIdx00, myMPC_grad_ineq00, myMPC_lubbysub00, myMPC_llbbyslb00);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub01, myMPC_sub01, myMPC_riub01, myMPC_llb01, myMPC_slb01, myMPC_rilb01, myMPC_lbIdx01, myMPC_ubIdx01, myMPC_grad_ineq01, myMPC_lubbysub01, myMPC_llbbyslb01);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub02, myMPC_sub02, myMPC_riub02, myMPC_llb02, myMPC_slb02, myMPC_rilb02, myMPC_lbIdx02, myMPC_ubIdx02, myMPC_grad_ineq02, myMPC_lubbysub02, myMPC_llbbyslb02);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub03, myMPC_sub03, myMPC_riub03, myMPC_llb03, myMPC_slb03, myMPC_rilb03, myMPC_lbIdx03, myMPC_ubIdx03, myMPC_grad_ineq03, myMPC_lubbysub03, myMPC_llbbyslb03);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub04, myMPC_sub04, myMPC_riub04, myMPC_llb04, myMPC_slb04, myMPC_rilb04, myMPC_lbIdx04, myMPC_ubIdx04, myMPC_grad_ineq04, myMPC_lubbysub04, myMPC_llbbyslb04);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub05, myMPC_sub05, myMPC_riub05, myMPC_llb05, myMPC_slb05, myMPC_rilb05, myMPC_lbIdx05, myMPC_ubIdx05, myMPC_grad_ineq05, myMPC_lubbysub05, myMPC_llbbyslb05);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub06, myMPC_sub06, myMPC_riub06, myMPC_llb06, myMPC_slb06, myMPC_rilb06, myMPC_lbIdx06, myMPC_ubIdx06, myMPC_grad_ineq06, myMPC_lubbysub06, myMPC_llbbyslb06);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub07, myMPC_sub07, myMPC_riub07, myMPC_llb07, myMPC_slb07, myMPC_rilb07, myMPC_lbIdx07, myMPC_ubIdx07, myMPC_grad_ineq07, myMPC_lubbysub07, myMPC_llbbyslb07);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub08, myMPC_sub08, myMPC_riub08, myMPC_llb08, myMPC_slb08, myMPC_rilb08, myMPC_lbIdx08, myMPC_ubIdx08, myMPC_grad_ineq08, myMPC_lubbysub08, myMPC_llbbyslb08);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub09, myMPC_sub09, myMPC_riub09, myMPC_llb09, myMPC_slb09, myMPC_rilb09, myMPC_lbIdx09, myMPC_ubIdx09, myMPC_grad_ineq09, myMPC_lubbysub09, myMPC_llbbyslb09);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub10, myMPC_sub10, myMPC_riub10, myMPC_llb10, myMPC_slb10, myMPC_rilb10, myMPC_lbIdx10, myMPC_ubIdx10, myMPC_grad_ineq10, myMPC_lubbysub10, myMPC_llbbyslb10);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub11, myMPC_sub11, myMPC_riub11, myMPC_llb11, myMPC_slb11, myMPC_rilb11, myMPC_lbIdx11, myMPC_ubIdx11, myMPC_grad_ineq11, myMPC_lubbysub11, myMPC_llbbyslb11);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub12, myMPC_sub12, myMPC_riub12, myMPC_llb12, myMPC_slb12, myMPC_rilb12, myMPC_lbIdx12, myMPC_ubIdx12, myMPC_grad_ineq12, myMPC_lubbysub12, myMPC_llbbyslb12);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub13, myMPC_sub13, myMPC_riub13, myMPC_llb13, myMPC_slb13, myMPC_rilb13, myMPC_lbIdx13, myMPC_ubIdx13, myMPC_grad_ineq13, myMPC_lubbysub13, myMPC_llbbyslb13);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub14, myMPC_sub14, myMPC_riub14, myMPC_llb14, myMPC_slb14, myMPC_rilb14, myMPC_lbIdx14, myMPC_ubIdx14, myMPC_grad_ineq14, myMPC_lubbysub14, myMPC_llbbyslb14);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub15, myMPC_sub15, myMPC_riub15, myMPC_llb15, myMPC_slb15, myMPC_rilb15, myMPC_lbIdx15, myMPC_ubIdx15, myMPC_grad_ineq15, myMPC_lubbysub15, myMPC_llbbyslb15);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub16, myMPC_sub16, myMPC_riub16, myMPC_llb16, myMPC_slb16, myMPC_rilb16, myMPC_lbIdx16, myMPC_ubIdx16, myMPC_grad_ineq16, myMPC_lubbysub16, myMPC_llbbyslb16);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub17, myMPC_sub17, myMPC_riub17, myMPC_llb17, myMPC_slb17, myMPC_rilb17, myMPC_lbIdx17, myMPC_ubIdx17, myMPC_grad_ineq17, myMPC_lubbysub17, myMPC_llbbyslb17);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub18, myMPC_sub18, myMPC_riub18, myMPC_llb18, myMPC_slb18, myMPC_rilb18, myMPC_lbIdx18, myMPC_ubIdx18, myMPC_grad_ineq18, myMPC_lubbysub18, myMPC_llbbyslb18);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub19, myMPC_sub19, myMPC_riub19, myMPC_llb19, myMPC_slb19, myMPC_rilb19, myMPC_lbIdx19, myMPC_ubIdx19, myMPC_grad_ineq19, myMPC_lubbysub19, myMPC_llbbyslb19);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub20, myMPC_sub20, myMPC_riub20, myMPC_llb20, myMPC_slb20, myMPC_rilb20, myMPC_lbIdx20, myMPC_ubIdx20, myMPC_grad_ineq20, myMPC_lubbysub20, myMPC_llbbyslb20);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub21, myMPC_sub21, myMPC_riub21, myMPC_llb21, myMPC_slb21, myMPC_rilb21, myMPC_lbIdx21, myMPC_ubIdx21, myMPC_grad_ineq21, myMPC_lubbysub21, myMPC_llbbyslb21);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub22, myMPC_sub22, myMPC_riub22, myMPC_llb22, myMPC_slb22, myMPC_rilb22, myMPC_lbIdx22, myMPC_ubIdx22, myMPC_grad_ineq22, myMPC_lubbysub22, myMPC_llbbyslb22);
myMPC_LA_INEQ_B_GRAD_10_10_10(myMPC_lub23, myMPC_sub23, myMPC_riub23, myMPC_llb23, myMPC_slb23, myMPC_rilb23, myMPC_lbIdx23, myMPC_ubIdx23, myMPC_grad_ineq23, myMPC_lubbysub23, myMPC_llbbyslb23);
myMPC_LA_INEQ_B_GRAD_8_8_8(myMPC_lub24, myMPC_sub24, myMPC_riub24, myMPC_llb24, myMPC_slb24, myMPC_rilb24, myMPC_lbIdx24, myMPC_ubIdx24, myMPC_grad_ineq24, myMPC_lubbysub24, myMPC_llbbyslb24);
info->res_eq = 0;
myMPC_LA_DENSE_2MVMSUB_16_10_10(myMPC_C00, myMPC_z00, myMPC_D01, myMPC_z01, params->z1, myMPC_v00, myMPC_re00, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z01, myMPC_D02, myMPC_z02, myMPC_c01, myMPC_v01, myMPC_re01, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z02, myMPC_D02, myMPC_z03, myMPC_c02, myMPC_v02, myMPC_re02, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z03, myMPC_D02, myMPC_z04, myMPC_c03, myMPC_v03, myMPC_re03, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z04, myMPC_D02, myMPC_z05, myMPC_c04, myMPC_v04, myMPC_re04, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z05, myMPC_D02, myMPC_z06, myMPC_c05, myMPC_v05, myMPC_re05, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z06, myMPC_D02, myMPC_z07, myMPC_c06, myMPC_v06, myMPC_re06, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z07, myMPC_D02, myMPC_z08, myMPC_c07, myMPC_v07, myMPC_re07, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z08, myMPC_D02, myMPC_z09, myMPC_c08, myMPC_v08, myMPC_re08, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z09, myMPC_D02, myMPC_z10, myMPC_c09, myMPC_v09, myMPC_re09, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z10, myMPC_D02, myMPC_z11, myMPC_c10, myMPC_v10, myMPC_re10, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z11, myMPC_D02, myMPC_z12, myMPC_c11, myMPC_v11, myMPC_re11, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z12, myMPC_D02, myMPC_z13, myMPC_c12, myMPC_v12, myMPC_re12, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z13, myMPC_D02, myMPC_z14, myMPC_c13, myMPC_v13, myMPC_re13, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z14, myMPC_D02, myMPC_z15, myMPC_c14, myMPC_v14, myMPC_re14, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z15, myMPC_D02, myMPC_z16, myMPC_c15, myMPC_v15, myMPC_re15, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z16, myMPC_D02, myMPC_z17, myMPC_c16, myMPC_v16, myMPC_re16, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z17, myMPC_D02, myMPC_z18, myMPC_c17, myMPC_v17, myMPC_re17, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z18, myMPC_D02, myMPC_z19, myMPC_c18, myMPC_v18, myMPC_re18, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z19, myMPC_D02, myMPC_z20, myMPC_c19, myMPC_v19, myMPC_re19, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z20, myMPC_D02, myMPC_z21, myMPC_c20, myMPC_v20, myMPC_re20, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z21, myMPC_D02, myMPC_z22, myMPC_c21, myMPC_v21, myMPC_re21, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_10(myMPC_C01, myMPC_z22, myMPC_D02, myMPC_z23, myMPC_c22, myMPC_v22, myMPC_re22, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_2MVMSUB_8_10_8(myMPC_C01, myMPC_z23, myMPC_D24, myMPC_z24, myMPC_c23, myMPC_v23, myMPC_re23, &info->dgap, &info->res_eq);
myMPC_LA_DENSE_MTVM_16_10(myMPC_C00, myMPC_v00, myMPC_grad_eq00);
myMPC_LA_DENSE_MTVM2_8_10_16(myMPC_C01, myMPC_v01, myMPC_D01, myMPC_v00, myMPC_grad_eq01);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v02, myMPC_D02, myMPC_v01, myMPC_grad_eq02);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v03, myMPC_D02, myMPC_v02, myMPC_grad_eq03);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v04, myMPC_D02, myMPC_v03, myMPC_grad_eq04);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v05, myMPC_D02, myMPC_v04, myMPC_grad_eq05);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v06, myMPC_D02, myMPC_v05, myMPC_grad_eq06);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v07, myMPC_D02, myMPC_v06, myMPC_grad_eq07);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v08, myMPC_D02, myMPC_v07, myMPC_grad_eq08);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v09, myMPC_D02, myMPC_v08, myMPC_grad_eq09);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v10, myMPC_D02, myMPC_v09, myMPC_grad_eq10);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v11, myMPC_D02, myMPC_v10, myMPC_grad_eq11);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v12, myMPC_D02, myMPC_v11, myMPC_grad_eq12);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v13, myMPC_D02, myMPC_v12, myMPC_grad_eq13);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v14, myMPC_D02, myMPC_v13, myMPC_grad_eq14);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v15, myMPC_D02, myMPC_v14, myMPC_grad_eq15);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v16, myMPC_D02, myMPC_v15, myMPC_grad_eq16);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v17, myMPC_D02, myMPC_v16, myMPC_grad_eq17);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v18, myMPC_D02, myMPC_v17, myMPC_grad_eq18);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v19, myMPC_D02, myMPC_v18, myMPC_grad_eq19);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v20, myMPC_D02, myMPC_v19, myMPC_grad_eq20);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v21, myMPC_D02, myMPC_v20, myMPC_grad_eq21);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v22, myMPC_D02, myMPC_v21, myMPC_grad_eq22);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_v23, myMPC_D02, myMPC_v22, myMPC_grad_eq23);
myMPC_LA_DENSE_MTVM_8_8(myMPC_D24, myMPC_v23, myMPC_grad_eq24);
info->dobj = info->pobj - info->dgap;
info->rdgap = info->pobj ? info->dgap / info->pobj : 1e6;
if( info->rdgap < 0 ) info->rdgap = -info->rdgap;
PRINTTEXT("  %3d  %3.1e  %3.1e  %+6.4e  %+6.4e  %3.1e  %3.1e  %3.1e\n",info->it, info->res_eq, info->res_ineq, info->pobj, info->dobj, info->dgap, info->rdgap, info->mu);
if( info->mu < myMPC_SET_ACC_KKTCOMPL
    && (info->rdgap < myMPC_SET_ACC_RDGAP || info->dgap < myMPC_SET_ACC_KKTCOMPL)
    && info->res_eq < myMPC_SET_ACC_RESEQ
    && info->res_ineq < myMPC_SET_ACC_RESINEQ ){
PRINTTEXT("OPTIMAL (within RESEQ=%2.1e, RESINEQ=%2.1e, (R)DGAP=(%2.1e)%2.1e, MU=%2.1e).\n\n",myMPC_SET_ACC_RESEQ, myMPC_SET_ACC_RESINEQ,myMPC_SET_ACC_KKTCOMPL,myMPC_SET_ACC_RDGAP,myMPC_SET_ACC_KKTCOMPL);
exitcode = myMPC_OPTIMAL; break; }
if( info->it == myMPC_SET_MAXIT ){
PRINTTEXT("Maximum number of iterations reached, exiting.\n");
exitcode = myMPC_MAXITREACHED; break; }
myMPC_LA_VVADD3_10(myMPC_grad_cost00, myMPC_grad_eq00, myMPC_grad_ineq00, myMPC_rd00);
myMPC_LA_VVADD3_10(myMPC_grad_cost01, myMPC_grad_eq01, myMPC_grad_ineq01, myMPC_rd01);
myMPC_LA_VVADD3_10(myMPC_grad_cost02, myMPC_grad_eq02, myMPC_grad_ineq02, myMPC_rd02);
myMPC_LA_VVADD3_10(myMPC_grad_cost03, myMPC_grad_eq03, myMPC_grad_ineq03, myMPC_rd03);
myMPC_LA_VVADD3_10(myMPC_grad_cost04, myMPC_grad_eq04, myMPC_grad_ineq04, myMPC_rd04);
myMPC_LA_VVADD3_10(myMPC_grad_cost05, myMPC_grad_eq05, myMPC_grad_ineq05, myMPC_rd05);
myMPC_LA_VVADD3_10(myMPC_grad_cost06, myMPC_grad_eq06, myMPC_grad_ineq06, myMPC_rd06);
myMPC_LA_VVADD3_10(myMPC_grad_cost07, myMPC_grad_eq07, myMPC_grad_ineq07, myMPC_rd07);
myMPC_LA_VVADD3_10(myMPC_grad_cost08, myMPC_grad_eq08, myMPC_grad_ineq08, myMPC_rd08);
myMPC_LA_VVADD3_10(myMPC_grad_cost09, myMPC_grad_eq09, myMPC_grad_ineq09, myMPC_rd09);
myMPC_LA_VVADD3_10(myMPC_grad_cost10, myMPC_grad_eq10, myMPC_grad_ineq10, myMPC_rd10);
myMPC_LA_VVADD3_10(myMPC_grad_cost11, myMPC_grad_eq11, myMPC_grad_ineq11, myMPC_rd11);
myMPC_LA_VVADD3_10(myMPC_grad_cost12, myMPC_grad_eq12, myMPC_grad_ineq12, myMPC_rd12);
myMPC_LA_VVADD3_10(myMPC_grad_cost13, myMPC_grad_eq13, myMPC_grad_ineq13, myMPC_rd13);
myMPC_LA_VVADD3_10(myMPC_grad_cost14, myMPC_grad_eq14, myMPC_grad_ineq14, myMPC_rd14);
myMPC_LA_VVADD3_10(myMPC_grad_cost15, myMPC_grad_eq15, myMPC_grad_ineq15, myMPC_rd15);
myMPC_LA_VVADD3_10(myMPC_grad_cost16, myMPC_grad_eq16, myMPC_grad_ineq16, myMPC_rd16);
myMPC_LA_VVADD3_10(myMPC_grad_cost17, myMPC_grad_eq17, myMPC_grad_ineq17, myMPC_rd17);
myMPC_LA_VVADD3_10(myMPC_grad_cost18, myMPC_grad_eq18, myMPC_grad_ineq18, myMPC_rd18);
myMPC_LA_VVADD3_10(myMPC_grad_cost19, myMPC_grad_eq19, myMPC_grad_ineq19, myMPC_rd19);
myMPC_LA_VVADD3_10(myMPC_grad_cost20, myMPC_grad_eq20, myMPC_grad_ineq20, myMPC_rd20);
myMPC_LA_VVADD3_10(myMPC_grad_cost21, myMPC_grad_eq21, myMPC_grad_ineq21, myMPC_rd21);
myMPC_LA_VVADD3_10(myMPC_grad_cost22, myMPC_grad_eq22, myMPC_grad_ineq22, myMPC_rd22);
myMPC_LA_VVADD3_10(myMPC_grad_cost23, myMPC_grad_eq23, myMPC_grad_ineq23, myMPC_rd23);
myMPC_LA_VVADD3_8(myMPC_grad_cost24, myMPC_grad_eq24, myMPC_grad_ineq24, myMPC_rd24);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb00, myMPC_lbIdx00, myMPC_lubbysub00, myMPC_ubIdx00, myMPC_Phi00);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi00);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb01, myMPC_lbIdx01, myMPC_lubbysub01, myMPC_ubIdx01, myMPC_Phi01);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi01);
myMPC_LA_DENSE_MATRIXFORWARDSUB_16_10(myMPC_Phi00, myMPC_C00, myMPC_V00);
myMPC_LA_DENSE_MATRIXFORWARDSUB_16_10(myMPC_Phi01, myMPC_D01, myMPC_W01);
myMPC_LA_DENSE_MMT2_16_10_10(myMPC_V00, myMPC_W01, myMPC_Yd00);
myMPC_LA_DENSE_CHOL_16(myMPC_Yd00, myMPC_Ld00);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi00, myMPC_rd00, myMPC_Lbyrd00);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi01, myMPC_rd01, myMPC_Lbyrd01);
myMPC_LA_DENSE_2MVMSUB2_16_10_10(myMPC_V00, myMPC_Lbyrd00, myMPC_W01, myMPC_Lbyrd01, myMPC_re00, myMPC_beta00);
myMPC_LA_DENSE_FORWARDSUB_16(myMPC_Ld00, myMPC_beta00, myMPC_yy00);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb02, myMPC_lbIdx02, myMPC_lubbysub02, myMPC_ubIdx02, myMPC_Phi02);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi02);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi01, myMPC_C01, myMPC_V01);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi02, myMPC_D02, myMPC_W02);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V01, myMPC_W02, myMPC_Yd01);
myMPC_LA_DENSE_MMTM_16_10_8(myMPC_W01, myMPC_V01, myMPC_Ysd01);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_16(myMPC_Ld00, myMPC_Ysd01, myMPC_Lsd01);
myMPC_LA_DENSE_MMTSUB_8_16(myMPC_Lsd01, myMPC_Yd01);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd01, myMPC_Ld01);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi02, myMPC_rd02, myMPC_Lbyrd02);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi01, myMPC_rd01, myMPC_Lbyrd01);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V01, myMPC_Lbyrd01, myMPC_W02, myMPC_Lbyrd02, myMPC_re01, myMPC_beta01);
myMPC_LA_DENSE_MVMSUB1_8_16(myMPC_Lsd01, myMPC_yy00, myMPC_beta01, myMPC_bmy01);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld01, myMPC_bmy01, myMPC_yy01);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb03, myMPC_lbIdx03, myMPC_lubbysub03, myMPC_ubIdx03, myMPC_Phi03);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi03);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi02, myMPC_C01, myMPC_V02);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi03, myMPC_D02, myMPC_W03);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V02, myMPC_W03, myMPC_Yd02);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W02, myMPC_V02, myMPC_Ysd02);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld01, myMPC_Ysd02, myMPC_Lsd02);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd02, myMPC_Yd02);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd02, myMPC_Ld02);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi03, myMPC_rd03, myMPC_Lbyrd03);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi02, myMPC_rd02, myMPC_Lbyrd02);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V02, myMPC_Lbyrd02, myMPC_W03, myMPC_Lbyrd03, myMPC_re02, myMPC_beta02);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd02, myMPC_yy01, myMPC_beta02, myMPC_bmy02);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld02, myMPC_bmy02, myMPC_yy02);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb04, myMPC_lbIdx04, myMPC_lubbysub04, myMPC_ubIdx04, myMPC_Phi04);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi04);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi03, myMPC_C01, myMPC_V03);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi04, myMPC_D02, myMPC_W04);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V03, myMPC_W04, myMPC_Yd03);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W03, myMPC_V03, myMPC_Ysd03);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld02, myMPC_Ysd03, myMPC_Lsd03);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd03, myMPC_Yd03);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd03, myMPC_Ld03);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi04, myMPC_rd04, myMPC_Lbyrd04);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi03, myMPC_rd03, myMPC_Lbyrd03);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V03, myMPC_Lbyrd03, myMPC_W04, myMPC_Lbyrd04, myMPC_re03, myMPC_beta03);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd03, myMPC_yy02, myMPC_beta03, myMPC_bmy03);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld03, myMPC_bmy03, myMPC_yy03);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb05, myMPC_lbIdx05, myMPC_lubbysub05, myMPC_ubIdx05, myMPC_Phi05);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi05);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi04, myMPC_C01, myMPC_V04);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi05, myMPC_D02, myMPC_W05);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V04, myMPC_W05, myMPC_Yd04);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W04, myMPC_V04, myMPC_Ysd04);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld03, myMPC_Ysd04, myMPC_Lsd04);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd04, myMPC_Yd04);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd04, myMPC_Ld04);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi05, myMPC_rd05, myMPC_Lbyrd05);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi04, myMPC_rd04, myMPC_Lbyrd04);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V04, myMPC_Lbyrd04, myMPC_W05, myMPC_Lbyrd05, myMPC_re04, myMPC_beta04);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd04, myMPC_yy03, myMPC_beta04, myMPC_bmy04);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld04, myMPC_bmy04, myMPC_yy04);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb06, myMPC_lbIdx06, myMPC_lubbysub06, myMPC_ubIdx06, myMPC_Phi06);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi06);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi05, myMPC_C01, myMPC_V05);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi06, myMPC_D02, myMPC_W06);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V05, myMPC_W06, myMPC_Yd05);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W05, myMPC_V05, myMPC_Ysd05);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld04, myMPC_Ysd05, myMPC_Lsd05);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd05, myMPC_Yd05);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd05, myMPC_Ld05);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi06, myMPC_rd06, myMPC_Lbyrd06);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi05, myMPC_rd05, myMPC_Lbyrd05);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V05, myMPC_Lbyrd05, myMPC_W06, myMPC_Lbyrd06, myMPC_re05, myMPC_beta05);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd05, myMPC_yy04, myMPC_beta05, myMPC_bmy05);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld05, myMPC_bmy05, myMPC_yy05);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb07, myMPC_lbIdx07, myMPC_lubbysub07, myMPC_ubIdx07, myMPC_Phi07);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi07);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi06, myMPC_C01, myMPC_V06);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi07, myMPC_D02, myMPC_W07);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V06, myMPC_W07, myMPC_Yd06);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W06, myMPC_V06, myMPC_Ysd06);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld05, myMPC_Ysd06, myMPC_Lsd06);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd06, myMPC_Yd06);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd06, myMPC_Ld06);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi07, myMPC_rd07, myMPC_Lbyrd07);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi06, myMPC_rd06, myMPC_Lbyrd06);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V06, myMPC_Lbyrd06, myMPC_W07, myMPC_Lbyrd07, myMPC_re06, myMPC_beta06);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd06, myMPC_yy05, myMPC_beta06, myMPC_bmy06);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld06, myMPC_bmy06, myMPC_yy06);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb08, myMPC_lbIdx08, myMPC_lubbysub08, myMPC_ubIdx08, myMPC_Phi08);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi08);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi07, myMPC_C01, myMPC_V07);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi08, myMPC_D02, myMPC_W08);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V07, myMPC_W08, myMPC_Yd07);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W07, myMPC_V07, myMPC_Ysd07);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld06, myMPC_Ysd07, myMPC_Lsd07);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd07, myMPC_Yd07);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd07, myMPC_Ld07);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi08, myMPC_rd08, myMPC_Lbyrd08);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi07, myMPC_rd07, myMPC_Lbyrd07);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V07, myMPC_Lbyrd07, myMPC_W08, myMPC_Lbyrd08, myMPC_re07, myMPC_beta07);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd07, myMPC_yy06, myMPC_beta07, myMPC_bmy07);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld07, myMPC_bmy07, myMPC_yy07);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb09, myMPC_lbIdx09, myMPC_lubbysub09, myMPC_ubIdx09, myMPC_Phi09);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi09);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi08, myMPC_C01, myMPC_V08);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi09, myMPC_D02, myMPC_W09);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V08, myMPC_W09, myMPC_Yd08);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W08, myMPC_V08, myMPC_Ysd08);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld07, myMPC_Ysd08, myMPC_Lsd08);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd08, myMPC_Yd08);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd08, myMPC_Ld08);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi09, myMPC_rd09, myMPC_Lbyrd09);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi08, myMPC_rd08, myMPC_Lbyrd08);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V08, myMPC_Lbyrd08, myMPC_W09, myMPC_Lbyrd09, myMPC_re08, myMPC_beta08);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd08, myMPC_yy07, myMPC_beta08, myMPC_bmy08);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld08, myMPC_bmy08, myMPC_yy08);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb10, myMPC_lbIdx10, myMPC_lubbysub10, myMPC_ubIdx10, myMPC_Phi10);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi10);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi09, myMPC_C01, myMPC_V09);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi10, myMPC_D02, myMPC_W10);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V09, myMPC_W10, myMPC_Yd09);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W09, myMPC_V09, myMPC_Ysd09);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld08, myMPC_Ysd09, myMPC_Lsd09);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd09, myMPC_Yd09);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd09, myMPC_Ld09);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi10, myMPC_rd10, myMPC_Lbyrd10);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi09, myMPC_rd09, myMPC_Lbyrd09);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V09, myMPC_Lbyrd09, myMPC_W10, myMPC_Lbyrd10, myMPC_re09, myMPC_beta09);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd09, myMPC_yy08, myMPC_beta09, myMPC_bmy09);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld09, myMPC_bmy09, myMPC_yy09);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb11, myMPC_lbIdx11, myMPC_lubbysub11, myMPC_ubIdx11, myMPC_Phi11);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi11);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi10, myMPC_C01, myMPC_V10);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi11, myMPC_D02, myMPC_W11);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V10, myMPC_W11, myMPC_Yd10);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W10, myMPC_V10, myMPC_Ysd10);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld09, myMPC_Ysd10, myMPC_Lsd10);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd10, myMPC_Yd10);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd10, myMPC_Ld10);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi11, myMPC_rd11, myMPC_Lbyrd11);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi10, myMPC_rd10, myMPC_Lbyrd10);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V10, myMPC_Lbyrd10, myMPC_W11, myMPC_Lbyrd11, myMPC_re10, myMPC_beta10);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd10, myMPC_yy09, myMPC_beta10, myMPC_bmy10);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld10, myMPC_bmy10, myMPC_yy10);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb12, myMPC_lbIdx12, myMPC_lubbysub12, myMPC_ubIdx12, myMPC_Phi12);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi12);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi11, myMPC_C01, myMPC_V11);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi12, myMPC_D02, myMPC_W12);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V11, myMPC_W12, myMPC_Yd11);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W11, myMPC_V11, myMPC_Ysd11);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld10, myMPC_Ysd11, myMPC_Lsd11);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd11, myMPC_Yd11);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd11, myMPC_Ld11);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi12, myMPC_rd12, myMPC_Lbyrd12);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi11, myMPC_rd11, myMPC_Lbyrd11);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V11, myMPC_Lbyrd11, myMPC_W12, myMPC_Lbyrd12, myMPC_re11, myMPC_beta11);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd11, myMPC_yy10, myMPC_beta11, myMPC_bmy11);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld11, myMPC_bmy11, myMPC_yy11);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb13, myMPC_lbIdx13, myMPC_lubbysub13, myMPC_ubIdx13, myMPC_Phi13);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi13);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi12, myMPC_C01, myMPC_V12);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi13, myMPC_D02, myMPC_W13);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V12, myMPC_W13, myMPC_Yd12);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W12, myMPC_V12, myMPC_Ysd12);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld11, myMPC_Ysd12, myMPC_Lsd12);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd12, myMPC_Yd12);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd12, myMPC_Ld12);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi13, myMPC_rd13, myMPC_Lbyrd13);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi12, myMPC_rd12, myMPC_Lbyrd12);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V12, myMPC_Lbyrd12, myMPC_W13, myMPC_Lbyrd13, myMPC_re12, myMPC_beta12);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd12, myMPC_yy11, myMPC_beta12, myMPC_bmy12);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld12, myMPC_bmy12, myMPC_yy12);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb14, myMPC_lbIdx14, myMPC_lubbysub14, myMPC_ubIdx14, myMPC_Phi14);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi14);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi13, myMPC_C01, myMPC_V13);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi14, myMPC_D02, myMPC_W14);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V13, myMPC_W14, myMPC_Yd13);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W13, myMPC_V13, myMPC_Ysd13);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld12, myMPC_Ysd13, myMPC_Lsd13);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd13, myMPC_Yd13);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd13, myMPC_Ld13);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi14, myMPC_rd14, myMPC_Lbyrd14);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi13, myMPC_rd13, myMPC_Lbyrd13);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V13, myMPC_Lbyrd13, myMPC_W14, myMPC_Lbyrd14, myMPC_re13, myMPC_beta13);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd13, myMPC_yy12, myMPC_beta13, myMPC_bmy13);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld13, myMPC_bmy13, myMPC_yy13);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb15, myMPC_lbIdx15, myMPC_lubbysub15, myMPC_ubIdx15, myMPC_Phi15);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi15);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi14, myMPC_C01, myMPC_V14);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi15, myMPC_D02, myMPC_W15);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V14, myMPC_W15, myMPC_Yd14);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W14, myMPC_V14, myMPC_Ysd14);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld13, myMPC_Ysd14, myMPC_Lsd14);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd14, myMPC_Yd14);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd14, myMPC_Ld14);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi15, myMPC_rd15, myMPC_Lbyrd15);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi14, myMPC_rd14, myMPC_Lbyrd14);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V14, myMPC_Lbyrd14, myMPC_W15, myMPC_Lbyrd15, myMPC_re14, myMPC_beta14);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd14, myMPC_yy13, myMPC_beta14, myMPC_bmy14);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld14, myMPC_bmy14, myMPC_yy14);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb16, myMPC_lbIdx16, myMPC_lubbysub16, myMPC_ubIdx16, myMPC_Phi16);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi16);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi15, myMPC_C01, myMPC_V15);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi16, myMPC_D02, myMPC_W16);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V15, myMPC_W16, myMPC_Yd15);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W15, myMPC_V15, myMPC_Ysd15);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld14, myMPC_Ysd15, myMPC_Lsd15);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd15, myMPC_Yd15);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd15, myMPC_Ld15);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi16, myMPC_rd16, myMPC_Lbyrd16);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi15, myMPC_rd15, myMPC_Lbyrd15);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V15, myMPC_Lbyrd15, myMPC_W16, myMPC_Lbyrd16, myMPC_re15, myMPC_beta15);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd15, myMPC_yy14, myMPC_beta15, myMPC_bmy15);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld15, myMPC_bmy15, myMPC_yy15);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb17, myMPC_lbIdx17, myMPC_lubbysub17, myMPC_ubIdx17, myMPC_Phi17);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi17);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi16, myMPC_C01, myMPC_V16);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi17, myMPC_D02, myMPC_W17);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V16, myMPC_W17, myMPC_Yd16);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W16, myMPC_V16, myMPC_Ysd16);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld15, myMPC_Ysd16, myMPC_Lsd16);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd16, myMPC_Yd16);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd16, myMPC_Ld16);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi17, myMPC_rd17, myMPC_Lbyrd17);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi16, myMPC_rd16, myMPC_Lbyrd16);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V16, myMPC_Lbyrd16, myMPC_W17, myMPC_Lbyrd17, myMPC_re16, myMPC_beta16);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd16, myMPC_yy15, myMPC_beta16, myMPC_bmy16);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld16, myMPC_bmy16, myMPC_yy16);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb18, myMPC_lbIdx18, myMPC_lubbysub18, myMPC_ubIdx18, myMPC_Phi18);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi18);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi17, myMPC_C01, myMPC_V17);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi18, myMPC_D02, myMPC_W18);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V17, myMPC_W18, myMPC_Yd17);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W17, myMPC_V17, myMPC_Ysd17);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld16, myMPC_Ysd17, myMPC_Lsd17);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd17, myMPC_Yd17);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd17, myMPC_Ld17);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi18, myMPC_rd18, myMPC_Lbyrd18);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi17, myMPC_rd17, myMPC_Lbyrd17);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V17, myMPC_Lbyrd17, myMPC_W18, myMPC_Lbyrd18, myMPC_re17, myMPC_beta17);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd17, myMPC_yy16, myMPC_beta17, myMPC_bmy17);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld17, myMPC_bmy17, myMPC_yy17);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb19, myMPC_lbIdx19, myMPC_lubbysub19, myMPC_ubIdx19, myMPC_Phi19);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi19);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi18, myMPC_C01, myMPC_V18);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi19, myMPC_D02, myMPC_W19);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V18, myMPC_W19, myMPC_Yd18);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W18, myMPC_V18, myMPC_Ysd18);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld17, myMPC_Ysd18, myMPC_Lsd18);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd18, myMPC_Yd18);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd18, myMPC_Ld18);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi19, myMPC_rd19, myMPC_Lbyrd19);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi18, myMPC_rd18, myMPC_Lbyrd18);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V18, myMPC_Lbyrd18, myMPC_W19, myMPC_Lbyrd19, myMPC_re18, myMPC_beta18);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd18, myMPC_yy17, myMPC_beta18, myMPC_bmy18);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld18, myMPC_bmy18, myMPC_yy18);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb20, myMPC_lbIdx20, myMPC_lubbysub20, myMPC_ubIdx20, myMPC_Phi20);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi20);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi19, myMPC_C01, myMPC_V19);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi20, myMPC_D02, myMPC_W20);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V19, myMPC_W20, myMPC_Yd19);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W19, myMPC_V19, myMPC_Ysd19);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld18, myMPC_Ysd19, myMPC_Lsd19);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd19, myMPC_Yd19);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd19, myMPC_Ld19);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi20, myMPC_rd20, myMPC_Lbyrd20);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi19, myMPC_rd19, myMPC_Lbyrd19);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V19, myMPC_Lbyrd19, myMPC_W20, myMPC_Lbyrd20, myMPC_re19, myMPC_beta19);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd19, myMPC_yy18, myMPC_beta19, myMPC_bmy19);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld19, myMPC_bmy19, myMPC_yy19);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb21, myMPC_lbIdx21, myMPC_lubbysub21, myMPC_ubIdx21, myMPC_Phi21);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi21);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi20, myMPC_C01, myMPC_V20);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi21, myMPC_D02, myMPC_W21);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V20, myMPC_W21, myMPC_Yd20);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W20, myMPC_V20, myMPC_Ysd20);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld19, myMPC_Ysd20, myMPC_Lsd20);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd20, myMPC_Yd20);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd20, myMPC_Ld20);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi21, myMPC_rd21, myMPC_Lbyrd21);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi20, myMPC_rd20, myMPC_Lbyrd20);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V20, myMPC_Lbyrd20, myMPC_W21, myMPC_Lbyrd21, myMPC_re20, myMPC_beta20);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd20, myMPC_yy19, myMPC_beta20, myMPC_bmy20);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld20, myMPC_bmy20, myMPC_yy20);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb22, myMPC_lbIdx22, myMPC_lubbysub22, myMPC_ubIdx22, myMPC_Phi22);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi22);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi21, myMPC_C01, myMPC_V21);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi22, myMPC_D02, myMPC_W22);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V21, myMPC_W22, myMPC_Yd21);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W21, myMPC_V21, myMPC_Ysd21);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld20, myMPC_Ysd21, myMPC_Lsd21);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd21, myMPC_Yd21);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd21, myMPC_Ld21);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi22, myMPC_rd22, myMPC_Lbyrd22);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi21, myMPC_rd21, myMPC_Lbyrd21);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V21, myMPC_Lbyrd21, myMPC_W22, myMPC_Lbyrd22, myMPC_re21, myMPC_beta21);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd21, myMPC_yy20, myMPC_beta21, myMPC_bmy21);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld21, myMPC_bmy21, myMPC_yy21);
myMPC_LA_INEQ_DENSE_HESS_10_10_10(params->H, myMPC_llbbyslb23, myMPC_lbIdx23, myMPC_lubbysub23, myMPC_ubIdx23, myMPC_Phi23);
myMPC_LA_DENSE_CHOL2_10(myMPC_Phi23);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi22, myMPC_C01, myMPC_V22);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi23, myMPC_D02, myMPC_W23);
myMPC_LA_DENSE_MMT2_8_10_10(myMPC_V22, myMPC_W23, myMPC_Yd22);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W22, myMPC_V22, myMPC_Ysd22);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld21, myMPC_Ysd22, myMPC_Lsd22);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd22, myMPC_Yd22);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd22, myMPC_Ld22);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi23, myMPC_rd23, myMPC_Lbyrd23);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi22, myMPC_rd22, myMPC_Lbyrd22);
myMPC_LA_DENSE_2MVMSUB2_8_10_10(myMPC_V22, myMPC_Lbyrd22, myMPC_W23, myMPC_Lbyrd23, myMPC_re22, myMPC_beta22);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd22, myMPC_yy21, myMPC_beta22, myMPC_bmy22);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld22, myMPC_bmy22, myMPC_yy22);
myMPC_LA_INEQ_DENSE_HESS_8_8_8(params->H_N, myMPC_llbbyslb24, myMPC_lbIdx24, myMPC_lubbysub24, myMPC_ubIdx24, myMPC_Phi24);
myMPC_LA_DENSE_CHOL2_8(myMPC_Phi24);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_10(myMPC_Phi23, myMPC_C01, myMPC_V23);
myMPC_LA_DENSE_MATRIXFORWARDSUB_8_8(myMPC_Phi24, myMPC_D24, myMPC_W24);
myMPC_LA_DENSE_MMT2_8_10_8(myMPC_V23, myMPC_W24, myMPC_Yd23);
myMPC_LA_DENSE_MMTM_8_10_8(myMPC_W23, myMPC_V23, myMPC_Ysd23);
myMPC_LA_DENSE_MATRIXTFORWARDSUB_8_8(myMPC_Ld22, myMPC_Ysd23, myMPC_Lsd23);
myMPC_LA_DENSE_MMTSUB_8_8(myMPC_Lsd23, myMPC_Yd23);
myMPC_LA_DENSE_CHOL_8(myMPC_Yd23, myMPC_Ld23);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Phi24, myMPC_rd24, myMPC_Lbyrd24);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi23, myMPC_rd23, myMPC_Lbyrd23);
myMPC_LA_DENSE_2MVMSUB2_8_10_8(myMPC_V23, myMPC_Lbyrd23, myMPC_W24, myMPC_Lbyrd24, myMPC_re23, myMPC_beta23);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd23, myMPC_yy22, myMPC_beta23, myMPC_bmy23);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld23, myMPC_bmy23, myMPC_yy23);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld23, myMPC_yy23, myMPC_dvaff23);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd23, myMPC_dvaff23, myMPC_yy22, myMPC_bmy22);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld22, myMPC_bmy22, myMPC_dvaff22);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd22, myMPC_dvaff22, myMPC_yy21, myMPC_bmy21);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld21, myMPC_bmy21, myMPC_dvaff21);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd21, myMPC_dvaff21, myMPC_yy20, myMPC_bmy20);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld20, myMPC_bmy20, myMPC_dvaff20);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd20, myMPC_dvaff20, myMPC_yy19, myMPC_bmy19);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld19, myMPC_bmy19, myMPC_dvaff19);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd19, myMPC_dvaff19, myMPC_yy18, myMPC_bmy18);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld18, myMPC_bmy18, myMPC_dvaff18);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd18, myMPC_dvaff18, myMPC_yy17, myMPC_bmy17);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld17, myMPC_bmy17, myMPC_dvaff17);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd17, myMPC_dvaff17, myMPC_yy16, myMPC_bmy16);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld16, myMPC_bmy16, myMPC_dvaff16);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd16, myMPC_dvaff16, myMPC_yy15, myMPC_bmy15);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld15, myMPC_bmy15, myMPC_dvaff15);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd15, myMPC_dvaff15, myMPC_yy14, myMPC_bmy14);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld14, myMPC_bmy14, myMPC_dvaff14);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd14, myMPC_dvaff14, myMPC_yy13, myMPC_bmy13);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld13, myMPC_bmy13, myMPC_dvaff13);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd13, myMPC_dvaff13, myMPC_yy12, myMPC_bmy12);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld12, myMPC_bmy12, myMPC_dvaff12);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd12, myMPC_dvaff12, myMPC_yy11, myMPC_bmy11);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld11, myMPC_bmy11, myMPC_dvaff11);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd11, myMPC_dvaff11, myMPC_yy10, myMPC_bmy10);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld10, myMPC_bmy10, myMPC_dvaff10);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd10, myMPC_dvaff10, myMPC_yy09, myMPC_bmy09);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld09, myMPC_bmy09, myMPC_dvaff09);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd09, myMPC_dvaff09, myMPC_yy08, myMPC_bmy08);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld08, myMPC_bmy08, myMPC_dvaff08);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd08, myMPC_dvaff08, myMPC_yy07, myMPC_bmy07);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld07, myMPC_bmy07, myMPC_dvaff07);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd07, myMPC_dvaff07, myMPC_yy06, myMPC_bmy06);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld06, myMPC_bmy06, myMPC_dvaff06);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd06, myMPC_dvaff06, myMPC_yy05, myMPC_bmy05);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld05, myMPC_bmy05, myMPC_dvaff05);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd05, myMPC_dvaff05, myMPC_yy04, myMPC_bmy04);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld04, myMPC_bmy04, myMPC_dvaff04);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd04, myMPC_dvaff04, myMPC_yy03, myMPC_bmy03);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld03, myMPC_bmy03, myMPC_dvaff03);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd03, myMPC_dvaff03, myMPC_yy02, myMPC_bmy02);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld02, myMPC_bmy02, myMPC_dvaff02);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd02, myMPC_dvaff02, myMPC_yy01, myMPC_bmy01);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld01, myMPC_bmy01, myMPC_dvaff01);
myMPC_LA_DENSE_MTVMSUB_8_16(myMPC_Lsd01, myMPC_dvaff01, myMPC_yy00, myMPC_bmy00);
myMPC_LA_DENSE_BACKWARDSUB_16(myMPC_Ld00, myMPC_bmy00, myMPC_dvaff00);
myMPC_LA_DENSE_MTVM_16_10(myMPC_C00, myMPC_dvaff00, myMPC_grad_eq00);
myMPC_LA_DENSE_MTVM2_8_10_16(myMPC_C01, myMPC_dvaff01, myMPC_D01, myMPC_dvaff00, myMPC_grad_eq01);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff02, myMPC_D02, myMPC_dvaff01, myMPC_grad_eq02);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff03, myMPC_D02, myMPC_dvaff02, myMPC_grad_eq03);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff04, myMPC_D02, myMPC_dvaff03, myMPC_grad_eq04);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff05, myMPC_D02, myMPC_dvaff04, myMPC_grad_eq05);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff06, myMPC_D02, myMPC_dvaff05, myMPC_grad_eq06);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff07, myMPC_D02, myMPC_dvaff06, myMPC_grad_eq07);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff08, myMPC_D02, myMPC_dvaff07, myMPC_grad_eq08);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff09, myMPC_D02, myMPC_dvaff08, myMPC_grad_eq09);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff10, myMPC_D02, myMPC_dvaff09, myMPC_grad_eq10);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff11, myMPC_D02, myMPC_dvaff10, myMPC_grad_eq11);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff12, myMPC_D02, myMPC_dvaff11, myMPC_grad_eq12);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff13, myMPC_D02, myMPC_dvaff12, myMPC_grad_eq13);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff14, myMPC_D02, myMPC_dvaff13, myMPC_grad_eq14);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff15, myMPC_D02, myMPC_dvaff14, myMPC_grad_eq15);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff16, myMPC_D02, myMPC_dvaff15, myMPC_grad_eq16);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff17, myMPC_D02, myMPC_dvaff16, myMPC_grad_eq17);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff18, myMPC_D02, myMPC_dvaff17, myMPC_grad_eq18);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff19, myMPC_D02, myMPC_dvaff18, myMPC_grad_eq19);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff20, myMPC_D02, myMPC_dvaff19, myMPC_grad_eq20);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff21, myMPC_D02, myMPC_dvaff20, myMPC_grad_eq21);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff22, myMPC_D02, myMPC_dvaff21, myMPC_grad_eq22);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvaff23, myMPC_D02, myMPC_dvaff22, myMPC_grad_eq23);
myMPC_LA_DENSE_MTVM_8_8(myMPC_D24, myMPC_dvaff23, myMPC_grad_eq24);
myMPC_LA_VSUB2_10(myMPC_rd00, myMPC_grad_eq00, myMPC_rd00);
myMPC_LA_VSUB2_10(myMPC_rd01, myMPC_grad_eq01, myMPC_rd01);
myMPC_LA_VSUB2_10(myMPC_rd02, myMPC_grad_eq02, myMPC_rd02);
myMPC_LA_VSUB2_10(myMPC_rd03, myMPC_grad_eq03, myMPC_rd03);
myMPC_LA_VSUB2_10(myMPC_rd04, myMPC_grad_eq04, myMPC_rd04);
myMPC_LA_VSUB2_10(myMPC_rd05, myMPC_grad_eq05, myMPC_rd05);
myMPC_LA_VSUB2_10(myMPC_rd06, myMPC_grad_eq06, myMPC_rd06);
myMPC_LA_VSUB2_10(myMPC_rd07, myMPC_grad_eq07, myMPC_rd07);
myMPC_LA_VSUB2_10(myMPC_rd08, myMPC_grad_eq08, myMPC_rd08);
myMPC_LA_VSUB2_10(myMPC_rd09, myMPC_grad_eq09, myMPC_rd09);
myMPC_LA_VSUB2_10(myMPC_rd10, myMPC_grad_eq10, myMPC_rd10);
myMPC_LA_VSUB2_10(myMPC_rd11, myMPC_grad_eq11, myMPC_rd11);
myMPC_LA_VSUB2_10(myMPC_rd12, myMPC_grad_eq12, myMPC_rd12);
myMPC_LA_VSUB2_10(myMPC_rd13, myMPC_grad_eq13, myMPC_rd13);
myMPC_LA_VSUB2_10(myMPC_rd14, myMPC_grad_eq14, myMPC_rd14);
myMPC_LA_VSUB2_10(myMPC_rd15, myMPC_grad_eq15, myMPC_rd15);
myMPC_LA_VSUB2_10(myMPC_rd16, myMPC_grad_eq16, myMPC_rd16);
myMPC_LA_VSUB2_10(myMPC_rd17, myMPC_grad_eq17, myMPC_rd17);
myMPC_LA_VSUB2_10(myMPC_rd18, myMPC_grad_eq18, myMPC_rd18);
myMPC_LA_VSUB2_10(myMPC_rd19, myMPC_grad_eq19, myMPC_rd19);
myMPC_LA_VSUB2_10(myMPC_rd20, myMPC_grad_eq20, myMPC_rd20);
myMPC_LA_VSUB2_10(myMPC_rd21, myMPC_grad_eq21, myMPC_rd21);
myMPC_LA_VSUB2_10(myMPC_rd22, myMPC_grad_eq22, myMPC_rd22);
myMPC_LA_VSUB2_10(myMPC_rd23, myMPC_grad_eq23, myMPC_rd23);
myMPC_LA_VSUB2_8(myMPC_rd24, myMPC_grad_eq24, myMPC_rd24);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi00, myMPC_rd00, myMPC_dzaff00);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi01, myMPC_rd01, myMPC_dzaff01);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi02, myMPC_rd02, myMPC_dzaff02);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi03, myMPC_rd03, myMPC_dzaff03);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi04, myMPC_rd04, myMPC_dzaff04);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi05, myMPC_rd05, myMPC_dzaff05);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi06, myMPC_rd06, myMPC_dzaff06);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi07, myMPC_rd07, myMPC_dzaff07);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi08, myMPC_rd08, myMPC_dzaff08);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi09, myMPC_rd09, myMPC_dzaff09);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi10, myMPC_rd10, myMPC_dzaff10);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi11, myMPC_rd11, myMPC_dzaff11);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi12, myMPC_rd12, myMPC_dzaff12);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi13, myMPC_rd13, myMPC_dzaff13);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi14, myMPC_rd14, myMPC_dzaff14);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi15, myMPC_rd15, myMPC_dzaff15);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi16, myMPC_rd16, myMPC_dzaff16);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi17, myMPC_rd17, myMPC_dzaff17);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi18, myMPC_rd18, myMPC_dzaff18);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi19, myMPC_rd19, myMPC_dzaff19);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi20, myMPC_rd20, myMPC_dzaff20);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi21, myMPC_rd21, myMPC_dzaff21);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi22, myMPC_rd22, myMPC_dzaff22);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi23, myMPC_rd23, myMPC_dzaff23);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_8(myMPC_Phi24, myMPC_rd24, myMPC_dzaff24);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff00, myMPC_lbIdx00, myMPC_rilb00, myMPC_dslbaff00);
myMPC_LA_VSUB3_10(myMPC_llbbyslb00, myMPC_dslbaff00, myMPC_llb00, myMPC_dllbaff00);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub00, myMPC_dzaff00, myMPC_ubIdx00, myMPC_dsubaff00);
myMPC_LA_VSUB3_10(myMPC_lubbysub00, myMPC_dsubaff00, myMPC_lub00, myMPC_dlubaff00);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff01, myMPC_lbIdx01, myMPC_rilb01, myMPC_dslbaff01);
myMPC_LA_VSUB3_10(myMPC_llbbyslb01, myMPC_dslbaff01, myMPC_llb01, myMPC_dllbaff01);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub01, myMPC_dzaff01, myMPC_ubIdx01, myMPC_dsubaff01);
myMPC_LA_VSUB3_10(myMPC_lubbysub01, myMPC_dsubaff01, myMPC_lub01, myMPC_dlubaff01);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff02, myMPC_lbIdx02, myMPC_rilb02, myMPC_dslbaff02);
myMPC_LA_VSUB3_10(myMPC_llbbyslb02, myMPC_dslbaff02, myMPC_llb02, myMPC_dllbaff02);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub02, myMPC_dzaff02, myMPC_ubIdx02, myMPC_dsubaff02);
myMPC_LA_VSUB3_10(myMPC_lubbysub02, myMPC_dsubaff02, myMPC_lub02, myMPC_dlubaff02);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff03, myMPC_lbIdx03, myMPC_rilb03, myMPC_dslbaff03);
myMPC_LA_VSUB3_10(myMPC_llbbyslb03, myMPC_dslbaff03, myMPC_llb03, myMPC_dllbaff03);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub03, myMPC_dzaff03, myMPC_ubIdx03, myMPC_dsubaff03);
myMPC_LA_VSUB3_10(myMPC_lubbysub03, myMPC_dsubaff03, myMPC_lub03, myMPC_dlubaff03);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff04, myMPC_lbIdx04, myMPC_rilb04, myMPC_dslbaff04);
myMPC_LA_VSUB3_10(myMPC_llbbyslb04, myMPC_dslbaff04, myMPC_llb04, myMPC_dllbaff04);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub04, myMPC_dzaff04, myMPC_ubIdx04, myMPC_dsubaff04);
myMPC_LA_VSUB3_10(myMPC_lubbysub04, myMPC_dsubaff04, myMPC_lub04, myMPC_dlubaff04);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff05, myMPC_lbIdx05, myMPC_rilb05, myMPC_dslbaff05);
myMPC_LA_VSUB3_10(myMPC_llbbyslb05, myMPC_dslbaff05, myMPC_llb05, myMPC_dllbaff05);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub05, myMPC_dzaff05, myMPC_ubIdx05, myMPC_dsubaff05);
myMPC_LA_VSUB3_10(myMPC_lubbysub05, myMPC_dsubaff05, myMPC_lub05, myMPC_dlubaff05);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff06, myMPC_lbIdx06, myMPC_rilb06, myMPC_dslbaff06);
myMPC_LA_VSUB3_10(myMPC_llbbyslb06, myMPC_dslbaff06, myMPC_llb06, myMPC_dllbaff06);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub06, myMPC_dzaff06, myMPC_ubIdx06, myMPC_dsubaff06);
myMPC_LA_VSUB3_10(myMPC_lubbysub06, myMPC_dsubaff06, myMPC_lub06, myMPC_dlubaff06);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff07, myMPC_lbIdx07, myMPC_rilb07, myMPC_dslbaff07);
myMPC_LA_VSUB3_10(myMPC_llbbyslb07, myMPC_dslbaff07, myMPC_llb07, myMPC_dllbaff07);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub07, myMPC_dzaff07, myMPC_ubIdx07, myMPC_dsubaff07);
myMPC_LA_VSUB3_10(myMPC_lubbysub07, myMPC_dsubaff07, myMPC_lub07, myMPC_dlubaff07);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff08, myMPC_lbIdx08, myMPC_rilb08, myMPC_dslbaff08);
myMPC_LA_VSUB3_10(myMPC_llbbyslb08, myMPC_dslbaff08, myMPC_llb08, myMPC_dllbaff08);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub08, myMPC_dzaff08, myMPC_ubIdx08, myMPC_dsubaff08);
myMPC_LA_VSUB3_10(myMPC_lubbysub08, myMPC_dsubaff08, myMPC_lub08, myMPC_dlubaff08);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff09, myMPC_lbIdx09, myMPC_rilb09, myMPC_dslbaff09);
myMPC_LA_VSUB3_10(myMPC_llbbyslb09, myMPC_dslbaff09, myMPC_llb09, myMPC_dllbaff09);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub09, myMPC_dzaff09, myMPC_ubIdx09, myMPC_dsubaff09);
myMPC_LA_VSUB3_10(myMPC_lubbysub09, myMPC_dsubaff09, myMPC_lub09, myMPC_dlubaff09);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff10, myMPC_lbIdx10, myMPC_rilb10, myMPC_dslbaff10);
myMPC_LA_VSUB3_10(myMPC_llbbyslb10, myMPC_dslbaff10, myMPC_llb10, myMPC_dllbaff10);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub10, myMPC_dzaff10, myMPC_ubIdx10, myMPC_dsubaff10);
myMPC_LA_VSUB3_10(myMPC_lubbysub10, myMPC_dsubaff10, myMPC_lub10, myMPC_dlubaff10);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff11, myMPC_lbIdx11, myMPC_rilb11, myMPC_dslbaff11);
myMPC_LA_VSUB3_10(myMPC_llbbyslb11, myMPC_dslbaff11, myMPC_llb11, myMPC_dllbaff11);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub11, myMPC_dzaff11, myMPC_ubIdx11, myMPC_dsubaff11);
myMPC_LA_VSUB3_10(myMPC_lubbysub11, myMPC_dsubaff11, myMPC_lub11, myMPC_dlubaff11);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff12, myMPC_lbIdx12, myMPC_rilb12, myMPC_dslbaff12);
myMPC_LA_VSUB3_10(myMPC_llbbyslb12, myMPC_dslbaff12, myMPC_llb12, myMPC_dllbaff12);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub12, myMPC_dzaff12, myMPC_ubIdx12, myMPC_dsubaff12);
myMPC_LA_VSUB3_10(myMPC_lubbysub12, myMPC_dsubaff12, myMPC_lub12, myMPC_dlubaff12);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff13, myMPC_lbIdx13, myMPC_rilb13, myMPC_dslbaff13);
myMPC_LA_VSUB3_10(myMPC_llbbyslb13, myMPC_dslbaff13, myMPC_llb13, myMPC_dllbaff13);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub13, myMPC_dzaff13, myMPC_ubIdx13, myMPC_dsubaff13);
myMPC_LA_VSUB3_10(myMPC_lubbysub13, myMPC_dsubaff13, myMPC_lub13, myMPC_dlubaff13);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff14, myMPC_lbIdx14, myMPC_rilb14, myMPC_dslbaff14);
myMPC_LA_VSUB3_10(myMPC_llbbyslb14, myMPC_dslbaff14, myMPC_llb14, myMPC_dllbaff14);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub14, myMPC_dzaff14, myMPC_ubIdx14, myMPC_dsubaff14);
myMPC_LA_VSUB3_10(myMPC_lubbysub14, myMPC_dsubaff14, myMPC_lub14, myMPC_dlubaff14);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff15, myMPC_lbIdx15, myMPC_rilb15, myMPC_dslbaff15);
myMPC_LA_VSUB3_10(myMPC_llbbyslb15, myMPC_dslbaff15, myMPC_llb15, myMPC_dllbaff15);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub15, myMPC_dzaff15, myMPC_ubIdx15, myMPC_dsubaff15);
myMPC_LA_VSUB3_10(myMPC_lubbysub15, myMPC_dsubaff15, myMPC_lub15, myMPC_dlubaff15);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff16, myMPC_lbIdx16, myMPC_rilb16, myMPC_dslbaff16);
myMPC_LA_VSUB3_10(myMPC_llbbyslb16, myMPC_dslbaff16, myMPC_llb16, myMPC_dllbaff16);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub16, myMPC_dzaff16, myMPC_ubIdx16, myMPC_dsubaff16);
myMPC_LA_VSUB3_10(myMPC_lubbysub16, myMPC_dsubaff16, myMPC_lub16, myMPC_dlubaff16);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff17, myMPC_lbIdx17, myMPC_rilb17, myMPC_dslbaff17);
myMPC_LA_VSUB3_10(myMPC_llbbyslb17, myMPC_dslbaff17, myMPC_llb17, myMPC_dllbaff17);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub17, myMPC_dzaff17, myMPC_ubIdx17, myMPC_dsubaff17);
myMPC_LA_VSUB3_10(myMPC_lubbysub17, myMPC_dsubaff17, myMPC_lub17, myMPC_dlubaff17);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff18, myMPC_lbIdx18, myMPC_rilb18, myMPC_dslbaff18);
myMPC_LA_VSUB3_10(myMPC_llbbyslb18, myMPC_dslbaff18, myMPC_llb18, myMPC_dllbaff18);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub18, myMPC_dzaff18, myMPC_ubIdx18, myMPC_dsubaff18);
myMPC_LA_VSUB3_10(myMPC_lubbysub18, myMPC_dsubaff18, myMPC_lub18, myMPC_dlubaff18);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff19, myMPC_lbIdx19, myMPC_rilb19, myMPC_dslbaff19);
myMPC_LA_VSUB3_10(myMPC_llbbyslb19, myMPC_dslbaff19, myMPC_llb19, myMPC_dllbaff19);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub19, myMPC_dzaff19, myMPC_ubIdx19, myMPC_dsubaff19);
myMPC_LA_VSUB3_10(myMPC_lubbysub19, myMPC_dsubaff19, myMPC_lub19, myMPC_dlubaff19);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff20, myMPC_lbIdx20, myMPC_rilb20, myMPC_dslbaff20);
myMPC_LA_VSUB3_10(myMPC_llbbyslb20, myMPC_dslbaff20, myMPC_llb20, myMPC_dllbaff20);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub20, myMPC_dzaff20, myMPC_ubIdx20, myMPC_dsubaff20);
myMPC_LA_VSUB3_10(myMPC_lubbysub20, myMPC_dsubaff20, myMPC_lub20, myMPC_dlubaff20);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff21, myMPC_lbIdx21, myMPC_rilb21, myMPC_dslbaff21);
myMPC_LA_VSUB3_10(myMPC_llbbyslb21, myMPC_dslbaff21, myMPC_llb21, myMPC_dllbaff21);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub21, myMPC_dzaff21, myMPC_ubIdx21, myMPC_dsubaff21);
myMPC_LA_VSUB3_10(myMPC_lubbysub21, myMPC_dsubaff21, myMPC_lub21, myMPC_dlubaff21);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff22, myMPC_lbIdx22, myMPC_rilb22, myMPC_dslbaff22);
myMPC_LA_VSUB3_10(myMPC_llbbyslb22, myMPC_dslbaff22, myMPC_llb22, myMPC_dllbaff22);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub22, myMPC_dzaff22, myMPC_ubIdx22, myMPC_dsubaff22);
myMPC_LA_VSUB3_10(myMPC_lubbysub22, myMPC_dsubaff22, myMPC_lub22, myMPC_dlubaff22);
myMPC_LA_VSUB_INDEXED_10(myMPC_dzaff23, myMPC_lbIdx23, myMPC_rilb23, myMPC_dslbaff23);
myMPC_LA_VSUB3_10(myMPC_llbbyslb23, myMPC_dslbaff23, myMPC_llb23, myMPC_dllbaff23);
myMPC_LA_VSUB2_INDEXED_10(myMPC_riub23, myMPC_dzaff23, myMPC_ubIdx23, myMPC_dsubaff23);
myMPC_LA_VSUB3_10(myMPC_lubbysub23, myMPC_dsubaff23, myMPC_lub23, myMPC_dlubaff23);
myMPC_LA_VSUB_INDEXED_8(myMPC_dzaff24, myMPC_lbIdx24, myMPC_rilb24, myMPC_dslbaff24);
myMPC_LA_VSUB3_8(myMPC_llbbyslb24, myMPC_dslbaff24, myMPC_llb24, myMPC_dllbaff24);
myMPC_LA_VSUB2_INDEXED_8(myMPC_riub24, myMPC_dzaff24, myMPC_ubIdx24, myMPC_dsubaff24);
myMPC_LA_VSUB3_8(myMPC_lubbysub24, myMPC_dsubaff24, myMPC_lub24, myMPC_dlubaff24);
info->lsit_aff = myMPC_LINESEARCH_BACKTRACKING_AFFINE(myMPC_l, myMPC_s, myMPC_dl_aff, myMPC_ds_aff, &info->step_aff, &info->mu_aff);
if( info->lsit_aff == myMPC_NOPROGRESS ){
PRINTTEXT("Affine line search could not proceed at iteration %d, exiting.\n",info->it+1);
return myMPC_NOPROGRESS;
}
sigma_3rdroot = info->mu_aff / info->mu;
info->sigma = sigma_3rdroot*sigma_3rdroot*sigma_3rdroot;
musigma = info->mu * info->sigma;
myMPC_LA_VSUB5_496(myMPC_ds_aff, myMPC_dl_aff, musigma, myMPC_ccrhs);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub00, myMPC_sub00, myMPC_ubIdx00, myMPC_ccrhsl00, myMPC_slb00, myMPC_lbIdx00, myMPC_rd00);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub01, myMPC_sub01, myMPC_ubIdx01, myMPC_ccrhsl01, myMPC_slb01, myMPC_lbIdx01, myMPC_rd01);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi00, myMPC_rd00, myMPC_Lbyrd00);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi01, myMPC_rd01, myMPC_Lbyrd01);
myMPC_LA_DENSE_2MVMADD_16_10_10(myMPC_V00, myMPC_Lbyrd00, myMPC_W01, myMPC_Lbyrd01, myMPC_beta00);
myMPC_LA_DENSE_FORWARDSUB_16(myMPC_Ld00, myMPC_beta00, myMPC_yy00);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub02, myMPC_sub02, myMPC_ubIdx02, myMPC_ccrhsl02, myMPC_slb02, myMPC_lbIdx02, myMPC_rd02);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi02, myMPC_rd02, myMPC_Lbyrd02);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi01, myMPC_rd01, myMPC_Lbyrd01);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V01, myMPC_Lbyrd01, myMPC_W02, myMPC_Lbyrd02, myMPC_beta01);
myMPC_LA_DENSE_MVMSUB1_8_16(myMPC_Lsd01, myMPC_yy00, myMPC_beta01, myMPC_bmy01);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld01, myMPC_bmy01, myMPC_yy01);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub03, myMPC_sub03, myMPC_ubIdx03, myMPC_ccrhsl03, myMPC_slb03, myMPC_lbIdx03, myMPC_rd03);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi03, myMPC_rd03, myMPC_Lbyrd03);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi02, myMPC_rd02, myMPC_Lbyrd02);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V02, myMPC_Lbyrd02, myMPC_W03, myMPC_Lbyrd03, myMPC_beta02);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd02, myMPC_yy01, myMPC_beta02, myMPC_bmy02);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld02, myMPC_bmy02, myMPC_yy02);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub04, myMPC_sub04, myMPC_ubIdx04, myMPC_ccrhsl04, myMPC_slb04, myMPC_lbIdx04, myMPC_rd04);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi04, myMPC_rd04, myMPC_Lbyrd04);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi03, myMPC_rd03, myMPC_Lbyrd03);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V03, myMPC_Lbyrd03, myMPC_W04, myMPC_Lbyrd04, myMPC_beta03);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd03, myMPC_yy02, myMPC_beta03, myMPC_bmy03);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld03, myMPC_bmy03, myMPC_yy03);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub05, myMPC_sub05, myMPC_ubIdx05, myMPC_ccrhsl05, myMPC_slb05, myMPC_lbIdx05, myMPC_rd05);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi05, myMPC_rd05, myMPC_Lbyrd05);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi04, myMPC_rd04, myMPC_Lbyrd04);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V04, myMPC_Lbyrd04, myMPC_W05, myMPC_Lbyrd05, myMPC_beta04);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd04, myMPC_yy03, myMPC_beta04, myMPC_bmy04);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld04, myMPC_bmy04, myMPC_yy04);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub06, myMPC_sub06, myMPC_ubIdx06, myMPC_ccrhsl06, myMPC_slb06, myMPC_lbIdx06, myMPC_rd06);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi06, myMPC_rd06, myMPC_Lbyrd06);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi05, myMPC_rd05, myMPC_Lbyrd05);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V05, myMPC_Lbyrd05, myMPC_W06, myMPC_Lbyrd06, myMPC_beta05);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd05, myMPC_yy04, myMPC_beta05, myMPC_bmy05);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld05, myMPC_bmy05, myMPC_yy05);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub07, myMPC_sub07, myMPC_ubIdx07, myMPC_ccrhsl07, myMPC_slb07, myMPC_lbIdx07, myMPC_rd07);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi07, myMPC_rd07, myMPC_Lbyrd07);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi06, myMPC_rd06, myMPC_Lbyrd06);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V06, myMPC_Lbyrd06, myMPC_W07, myMPC_Lbyrd07, myMPC_beta06);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd06, myMPC_yy05, myMPC_beta06, myMPC_bmy06);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld06, myMPC_bmy06, myMPC_yy06);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub08, myMPC_sub08, myMPC_ubIdx08, myMPC_ccrhsl08, myMPC_slb08, myMPC_lbIdx08, myMPC_rd08);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi08, myMPC_rd08, myMPC_Lbyrd08);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi07, myMPC_rd07, myMPC_Lbyrd07);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V07, myMPC_Lbyrd07, myMPC_W08, myMPC_Lbyrd08, myMPC_beta07);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd07, myMPC_yy06, myMPC_beta07, myMPC_bmy07);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld07, myMPC_bmy07, myMPC_yy07);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub09, myMPC_sub09, myMPC_ubIdx09, myMPC_ccrhsl09, myMPC_slb09, myMPC_lbIdx09, myMPC_rd09);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi09, myMPC_rd09, myMPC_Lbyrd09);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi08, myMPC_rd08, myMPC_Lbyrd08);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V08, myMPC_Lbyrd08, myMPC_W09, myMPC_Lbyrd09, myMPC_beta08);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd08, myMPC_yy07, myMPC_beta08, myMPC_bmy08);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld08, myMPC_bmy08, myMPC_yy08);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub10, myMPC_sub10, myMPC_ubIdx10, myMPC_ccrhsl10, myMPC_slb10, myMPC_lbIdx10, myMPC_rd10);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi10, myMPC_rd10, myMPC_Lbyrd10);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi09, myMPC_rd09, myMPC_Lbyrd09);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V09, myMPC_Lbyrd09, myMPC_W10, myMPC_Lbyrd10, myMPC_beta09);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd09, myMPC_yy08, myMPC_beta09, myMPC_bmy09);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld09, myMPC_bmy09, myMPC_yy09);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub11, myMPC_sub11, myMPC_ubIdx11, myMPC_ccrhsl11, myMPC_slb11, myMPC_lbIdx11, myMPC_rd11);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi11, myMPC_rd11, myMPC_Lbyrd11);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi10, myMPC_rd10, myMPC_Lbyrd10);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V10, myMPC_Lbyrd10, myMPC_W11, myMPC_Lbyrd11, myMPC_beta10);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd10, myMPC_yy09, myMPC_beta10, myMPC_bmy10);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld10, myMPC_bmy10, myMPC_yy10);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub12, myMPC_sub12, myMPC_ubIdx12, myMPC_ccrhsl12, myMPC_slb12, myMPC_lbIdx12, myMPC_rd12);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi12, myMPC_rd12, myMPC_Lbyrd12);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi11, myMPC_rd11, myMPC_Lbyrd11);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V11, myMPC_Lbyrd11, myMPC_W12, myMPC_Lbyrd12, myMPC_beta11);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd11, myMPC_yy10, myMPC_beta11, myMPC_bmy11);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld11, myMPC_bmy11, myMPC_yy11);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub13, myMPC_sub13, myMPC_ubIdx13, myMPC_ccrhsl13, myMPC_slb13, myMPC_lbIdx13, myMPC_rd13);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi13, myMPC_rd13, myMPC_Lbyrd13);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi12, myMPC_rd12, myMPC_Lbyrd12);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V12, myMPC_Lbyrd12, myMPC_W13, myMPC_Lbyrd13, myMPC_beta12);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd12, myMPC_yy11, myMPC_beta12, myMPC_bmy12);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld12, myMPC_bmy12, myMPC_yy12);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub14, myMPC_sub14, myMPC_ubIdx14, myMPC_ccrhsl14, myMPC_slb14, myMPC_lbIdx14, myMPC_rd14);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi14, myMPC_rd14, myMPC_Lbyrd14);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi13, myMPC_rd13, myMPC_Lbyrd13);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V13, myMPC_Lbyrd13, myMPC_W14, myMPC_Lbyrd14, myMPC_beta13);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd13, myMPC_yy12, myMPC_beta13, myMPC_bmy13);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld13, myMPC_bmy13, myMPC_yy13);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub15, myMPC_sub15, myMPC_ubIdx15, myMPC_ccrhsl15, myMPC_slb15, myMPC_lbIdx15, myMPC_rd15);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi15, myMPC_rd15, myMPC_Lbyrd15);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi14, myMPC_rd14, myMPC_Lbyrd14);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V14, myMPC_Lbyrd14, myMPC_W15, myMPC_Lbyrd15, myMPC_beta14);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd14, myMPC_yy13, myMPC_beta14, myMPC_bmy14);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld14, myMPC_bmy14, myMPC_yy14);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub16, myMPC_sub16, myMPC_ubIdx16, myMPC_ccrhsl16, myMPC_slb16, myMPC_lbIdx16, myMPC_rd16);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi16, myMPC_rd16, myMPC_Lbyrd16);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi15, myMPC_rd15, myMPC_Lbyrd15);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V15, myMPC_Lbyrd15, myMPC_W16, myMPC_Lbyrd16, myMPC_beta15);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd15, myMPC_yy14, myMPC_beta15, myMPC_bmy15);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld15, myMPC_bmy15, myMPC_yy15);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub17, myMPC_sub17, myMPC_ubIdx17, myMPC_ccrhsl17, myMPC_slb17, myMPC_lbIdx17, myMPC_rd17);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi17, myMPC_rd17, myMPC_Lbyrd17);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi16, myMPC_rd16, myMPC_Lbyrd16);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V16, myMPC_Lbyrd16, myMPC_W17, myMPC_Lbyrd17, myMPC_beta16);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd16, myMPC_yy15, myMPC_beta16, myMPC_bmy16);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld16, myMPC_bmy16, myMPC_yy16);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub18, myMPC_sub18, myMPC_ubIdx18, myMPC_ccrhsl18, myMPC_slb18, myMPC_lbIdx18, myMPC_rd18);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi18, myMPC_rd18, myMPC_Lbyrd18);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi17, myMPC_rd17, myMPC_Lbyrd17);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V17, myMPC_Lbyrd17, myMPC_W18, myMPC_Lbyrd18, myMPC_beta17);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd17, myMPC_yy16, myMPC_beta17, myMPC_bmy17);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld17, myMPC_bmy17, myMPC_yy17);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub19, myMPC_sub19, myMPC_ubIdx19, myMPC_ccrhsl19, myMPC_slb19, myMPC_lbIdx19, myMPC_rd19);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi19, myMPC_rd19, myMPC_Lbyrd19);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi18, myMPC_rd18, myMPC_Lbyrd18);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V18, myMPC_Lbyrd18, myMPC_W19, myMPC_Lbyrd19, myMPC_beta18);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd18, myMPC_yy17, myMPC_beta18, myMPC_bmy18);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld18, myMPC_bmy18, myMPC_yy18);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub20, myMPC_sub20, myMPC_ubIdx20, myMPC_ccrhsl20, myMPC_slb20, myMPC_lbIdx20, myMPC_rd20);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi20, myMPC_rd20, myMPC_Lbyrd20);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi19, myMPC_rd19, myMPC_Lbyrd19);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V19, myMPC_Lbyrd19, myMPC_W20, myMPC_Lbyrd20, myMPC_beta19);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd19, myMPC_yy18, myMPC_beta19, myMPC_bmy19);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld19, myMPC_bmy19, myMPC_yy19);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub21, myMPC_sub21, myMPC_ubIdx21, myMPC_ccrhsl21, myMPC_slb21, myMPC_lbIdx21, myMPC_rd21);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi21, myMPC_rd21, myMPC_Lbyrd21);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi20, myMPC_rd20, myMPC_Lbyrd20);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V20, myMPC_Lbyrd20, myMPC_W21, myMPC_Lbyrd21, myMPC_beta20);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd20, myMPC_yy19, myMPC_beta20, myMPC_bmy20);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld20, myMPC_bmy20, myMPC_yy20);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub22, myMPC_sub22, myMPC_ubIdx22, myMPC_ccrhsl22, myMPC_slb22, myMPC_lbIdx22, myMPC_rd22);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi22, myMPC_rd22, myMPC_Lbyrd22);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi21, myMPC_rd21, myMPC_Lbyrd21);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V21, myMPC_Lbyrd21, myMPC_W22, myMPC_Lbyrd22, myMPC_beta21);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd21, myMPC_yy20, myMPC_beta21, myMPC_bmy21);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld21, myMPC_bmy21, myMPC_yy21);
myMPC_LA_VSUB6_INDEXED_10_10_10(myMPC_ccrhsub23, myMPC_sub23, myMPC_ubIdx23, myMPC_ccrhsl23, myMPC_slb23, myMPC_lbIdx23, myMPC_rd23);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi23, myMPC_rd23, myMPC_Lbyrd23);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi22, myMPC_rd22, myMPC_Lbyrd22);
myMPC_LA_DENSE_2MVMADD_8_10_10(myMPC_V22, myMPC_Lbyrd22, myMPC_W23, myMPC_Lbyrd23, myMPC_beta22);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd22, myMPC_yy21, myMPC_beta22, myMPC_bmy22);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld22, myMPC_bmy22, myMPC_yy22);
myMPC_LA_VSUB6_INDEXED_8_8_8(myMPC_ccrhsub24, myMPC_sub24, myMPC_ubIdx24, myMPC_ccrhsl24, myMPC_slb24, myMPC_lbIdx24, myMPC_rd24);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Phi24, myMPC_rd24, myMPC_Lbyrd24);
myMPC_LA_DENSE_FORWARDSUB_10(myMPC_Phi23, myMPC_rd23, myMPC_Lbyrd23);
myMPC_LA_DENSE_2MVMADD_8_10_8(myMPC_V23, myMPC_Lbyrd23, myMPC_W24, myMPC_Lbyrd24, myMPC_beta23);
myMPC_LA_DENSE_MVMSUB1_8_8(myMPC_Lsd23, myMPC_yy22, myMPC_beta23, myMPC_bmy23);
myMPC_LA_DENSE_FORWARDSUB_8(myMPC_Ld23, myMPC_bmy23, myMPC_yy23);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld23, myMPC_yy23, myMPC_dvcc23);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd23, myMPC_dvcc23, myMPC_yy22, myMPC_bmy22);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld22, myMPC_bmy22, myMPC_dvcc22);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd22, myMPC_dvcc22, myMPC_yy21, myMPC_bmy21);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld21, myMPC_bmy21, myMPC_dvcc21);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd21, myMPC_dvcc21, myMPC_yy20, myMPC_bmy20);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld20, myMPC_bmy20, myMPC_dvcc20);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd20, myMPC_dvcc20, myMPC_yy19, myMPC_bmy19);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld19, myMPC_bmy19, myMPC_dvcc19);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd19, myMPC_dvcc19, myMPC_yy18, myMPC_bmy18);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld18, myMPC_bmy18, myMPC_dvcc18);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd18, myMPC_dvcc18, myMPC_yy17, myMPC_bmy17);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld17, myMPC_bmy17, myMPC_dvcc17);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd17, myMPC_dvcc17, myMPC_yy16, myMPC_bmy16);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld16, myMPC_bmy16, myMPC_dvcc16);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd16, myMPC_dvcc16, myMPC_yy15, myMPC_bmy15);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld15, myMPC_bmy15, myMPC_dvcc15);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd15, myMPC_dvcc15, myMPC_yy14, myMPC_bmy14);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld14, myMPC_bmy14, myMPC_dvcc14);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd14, myMPC_dvcc14, myMPC_yy13, myMPC_bmy13);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld13, myMPC_bmy13, myMPC_dvcc13);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd13, myMPC_dvcc13, myMPC_yy12, myMPC_bmy12);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld12, myMPC_bmy12, myMPC_dvcc12);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd12, myMPC_dvcc12, myMPC_yy11, myMPC_bmy11);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld11, myMPC_bmy11, myMPC_dvcc11);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd11, myMPC_dvcc11, myMPC_yy10, myMPC_bmy10);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld10, myMPC_bmy10, myMPC_dvcc10);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd10, myMPC_dvcc10, myMPC_yy09, myMPC_bmy09);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld09, myMPC_bmy09, myMPC_dvcc09);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd09, myMPC_dvcc09, myMPC_yy08, myMPC_bmy08);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld08, myMPC_bmy08, myMPC_dvcc08);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd08, myMPC_dvcc08, myMPC_yy07, myMPC_bmy07);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld07, myMPC_bmy07, myMPC_dvcc07);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd07, myMPC_dvcc07, myMPC_yy06, myMPC_bmy06);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld06, myMPC_bmy06, myMPC_dvcc06);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd06, myMPC_dvcc06, myMPC_yy05, myMPC_bmy05);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld05, myMPC_bmy05, myMPC_dvcc05);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd05, myMPC_dvcc05, myMPC_yy04, myMPC_bmy04);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld04, myMPC_bmy04, myMPC_dvcc04);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd04, myMPC_dvcc04, myMPC_yy03, myMPC_bmy03);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld03, myMPC_bmy03, myMPC_dvcc03);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd03, myMPC_dvcc03, myMPC_yy02, myMPC_bmy02);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld02, myMPC_bmy02, myMPC_dvcc02);
myMPC_LA_DENSE_MTVMSUB_8_8(myMPC_Lsd02, myMPC_dvcc02, myMPC_yy01, myMPC_bmy01);
myMPC_LA_DENSE_BACKWARDSUB_8(myMPC_Ld01, myMPC_bmy01, myMPC_dvcc01);
myMPC_LA_DENSE_MTVMSUB_8_16(myMPC_Lsd01, myMPC_dvcc01, myMPC_yy00, myMPC_bmy00);
myMPC_LA_DENSE_BACKWARDSUB_16(myMPC_Ld00, myMPC_bmy00, myMPC_dvcc00);
myMPC_LA_DENSE_MTVM_16_10(myMPC_C00, myMPC_dvcc00, myMPC_grad_eq00);
myMPC_LA_DENSE_MTVM2_8_10_16(myMPC_C01, myMPC_dvcc01, myMPC_D01, myMPC_dvcc00, myMPC_grad_eq01);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc02, myMPC_D02, myMPC_dvcc01, myMPC_grad_eq02);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc03, myMPC_D02, myMPC_dvcc02, myMPC_grad_eq03);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc04, myMPC_D02, myMPC_dvcc03, myMPC_grad_eq04);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc05, myMPC_D02, myMPC_dvcc04, myMPC_grad_eq05);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc06, myMPC_D02, myMPC_dvcc05, myMPC_grad_eq06);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc07, myMPC_D02, myMPC_dvcc06, myMPC_grad_eq07);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc08, myMPC_D02, myMPC_dvcc07, myMPC_grad_eq08);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc09, myMPC_D02, myMPC_dvcc08, myMPC_grad_eq09);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc10, myMPC_D02, myMPC_dvcc09, myMPC_grad_eq10);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc11, myMPC_D02, myMPC_dvcc10, myMPC_grad_eq11);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc12, myMPC_D02, myMPC_dvcc11, myMPC_grad_eq12);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc13, myMPC_D02, myMPC_dvcc12, myMPC_grad_eq13);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc14, myMPC_D02, myMPC_dvcc13, myMPC_grad_eq14);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc15, myMPC_D02, myMPC_dvcc14, myMPC_grad_eq15);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc16, myMPC_D02, myMPC_dvcc15, myMPC_grad_eq16);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc17, myMPC_D02, myMPC_dvcc16, myMPC_grad_eq17);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc18, myMPC_D02, myMPC_dvcc17, myMPC_grad_eq18);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc19, myMPC_D02, myMPC_dvcc18, myMPC_grad_eq19);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc20, myMPC_D02, myMPC_dvcc19, myMPC_grad_eq20);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc21, myMPC_D02, myMPC_dvcc20, myMPC_grad_eq21);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc22, myMPC_D02, myMPC_dvcc21, myMPC_grad_eq22);
myMPC_LA_DENSE_MTVM2_8_10_8(myMPC_C01, myMPC_dvcc23, myMPC_D02, myMPC_dvcc22, myMPC_grad_eq23);
myMPC_LA_DENSE_MTVM_8_8(myMPC_D24, myMPC_dvcc23, myMPC_grad_eq24);
myMPC_LA_VSUB_10(myMPC_rd00, myMPC_grad_eq00, myMPC_rd00);
myMPC_LA_VSUB_10(myMPC_rd01, myMPC_grad_eq01, myMPC_rd01);
myMPC_LA_VSUB_10(myMPC_rd02, myMPC_grad_eq02, myMPC_rd02);
myMPC_LA_VSUB_10(myMPC_rd03, myMPC_grad_eq03, myMPC_rd03);
myMPC_LA_VSUB_10(myMPC_rd04, myMPC_grad_eq04, myMPC_rd04);
myMPC_LA_VSUB_10(myMPC_rd05, myMPC_grad_eq05, myMPC_rd05);
myMPC_LA_VSUB_10(myMPC_rd06, myMPC_grad_eq06, myMPC_rd06);
myMPC_LA_VSUB_10(myMPC_rd07, myMPC_grad_eq07, myMPC_rd07);
myMPC_LA_VSUB_10(myMPC_rd08, myMPC_grad_eq08, myMPC_rd08);
myMPC_LA_VSUB_10(myMPC_rd09, myMPC_grad_eq09, myMPC_rd09);
myMPC_LA_VSUB_10(myMPC_rd10, myMPC_grad_eq10, myMPC_rd10);
myMPC_LA_VSUB_10(myMPC_rd11, myMPC_grad_eq11, myMPC_rd11);
myMPC_LA_VSUB_10(myMPC_rd12, myMPC_grad_eq12, myMPC_rd12);
myMPC_LA_VSUB_10(myMPC_rd13, myMPC_grad_eq13, myMPC_rd13);
myMPC_LA_VSUB_10(myMPC_rd14, myMPC_grad_eq14, myMPC_rd14);
myMPC_LA_VSUB_10(myMPC_rd15, myMPC_grad_eq15, myMPC_rd15);
myMPC_LA_VSUB_10(myMPC_rd16, myMPC_grad_eq16, myMPC_rd16);
myMPC_LA_VSUB_10(myMPC_rd17, myMPC_grad_eq17, myMPC_rd17);
myMPC_LA_VSUB_10(myMPC_rd18, myMPC_grad_eq18, myMPC_rd18);
myMPC_LA_VSUB_10(myMPC_rd19, myMPC_grad_eq19, myMPC_rd19);
myMPC_LA_VSUB_10(myMPC_rd20, myMPC_grad_eq20, myMPC_rd20);
myMPC_LA_VSUB_10(myMPC_rd21, myMPC_grad_eq21, myMPC_rd21);
myMPC_LA_VSUB_10(myMPC_rd22, myMPC_grad_eq22, myMPC_rd22);
myMPC_LA_VSUB_10(myMPC_rd23, myMPC_grad_eq23, myMPC_rd23);
myMPC_LA_VSUB_8(myMPC_rd24, myMPC_grad_eq24, myMPC_rd24);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi00, myMPC_rd00, myMPC_dzcc00);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi01, myMPC_rd01, myMPC_dzcc01);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi02, myMPC_rd02, myMPC_dzcc02);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi03, myMPC_rd03, myMPC_dzcc03);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi04, myMPC_rd04, myMPC_dzcc04);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi05, myMPC_rd05, myMPC_dzcc05);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi06, myMPC_rd06, myMPC_dzcc06);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi07, myMPC_rd07, myMPC_dzcc07);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi08, myMPC_rd08, myMPC_dzcc08);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi09, myMPC_rd09, myMPC_dzcc09);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi10, myMPC_rd10, myMPC_dzcc10);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi11, myMPC_rd11, myMPC_dzcc11);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi12, myMPC_rd12, myMPC_dzcc12);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi13, myMPC_rd13, myMPC_dzcc13);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi14, myMPC_rd14, myMPC_dzcc14);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi15, myMPC_rd15, myMPC_dzcc15);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi16, myMPC_rd16, myMPC_dzcc16);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi17, myMPC_rd17, myMPC_dzcc17);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi18, myMPC_rd18, myMPC_dzcc18);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi19, myMPC_rd19, myMPC_dzcc19);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi20, myMPC_rd20, myMPC_dzcc20);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi21, myMPC_rd21, myMPC_dzcc21);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi22, myMPC_rd22, myMPC_dzcc22);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_10(myMPC_Phi23, myMPC_rd23, myMPC_dzcc23);
myMPC_LA_DENSE_FORWARDBACKWARDSUB_8(myMPC_Phi24, myMPC_rd24, myMPC_dzcc24);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl00, myMPC_slb00, myMPC_llbbyslb00, myMPC_dzcc00, myMPC_lbIdx00, myMPC_dllbcc00);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub00, myMPC_sub00, myMPC_lubbysub00, myMPC_dzcc00, myMPC_ubIdx00, myMPC_dlubcc00);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl01, myMPC_slb01, myMPC_llbbyslb01, myMPC_dzcc01, myMPC_lbIdx01, myMPC_dllbcc01);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub01, myMPC_sub01, myMPC_lubbysub01, myMPC_dzcc01, myMPC_ubIdx01, myMPC_dlubcc01);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl02, myMPC_slb02, myMPC_llbbyslb02, myMPC_dzcc02, myMPC_lbIdx02, myMPC_dllbcc02);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub02, myMPC_sub02, myMPC_lubbysub02, myMPC_dzcc02, myMPC_ubIdx02, myMPC_dlubcc02);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl03, myMPC_slb03, myMPC_llbbyslb03, myMPC_dzcc03, myMPC_lbIdx03, myMPC_dllbcc03);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub03, myMPC_sub03, myMPC_lubbysub03, myMPC_dzcc03, myMPC_ubIdx03, myMPC_dlubcc03);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl04, myMPC_slb04, myMPC_llbbyslb04, myMPC_dzcc04, myMPC_lbIdx04, myMPC_dllbcc04);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub04, myMPC_sub04, myMPC_lubbysub04, myMPC_dzcc04, myMPC_ubIdx04, myMPC_dlubcc04);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl05, myMPC_slb05, myMPC_llbbyslb05, myMPC_dzcc05, myMPC_lbIdx05, myMPC_dllbcc05);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub05, myMPC_sub05, myMPC_lubbysub05, myMPC_dzcc05, myMPC_ubIdx05, myMPC_dlubcc05);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl06, myMPC_slb06, myMPC_llbbyslb06, myMPC_dzcc06, myMPC_lbIdx06, myMPC_dllbcc06);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub06, myMPC_sub06, myMPC_lubbysub06, myMPC_dzcc06, myMPC_ubIdx06, myMPC_dlubcc06);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl07, myMPC_slb07, myMPC_llbbyslb07, myMPC_dzcc07, myMPC_lbIdx07, myMPC_dllbcc07);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub07, myMPC_sub07, myMPC_lubbysub07, myMPC_dzcc07, myMPC_ubIdx07, myMPC_dlubcc07);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl08, myMPC_slb08, myMPC_llbbyslb08, myMPC_dzcc08, myMPC_lbIdx08, myMPC_dllbcc08);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub08, myMPC_sub08, myMPC_lubbysub08, myMPC_dzcc08, myMPC_ubIdx08, myMPC_dlubcc08);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl09, myMPC_slb09, myMPC_llbbyslb09, myMPC_dzcc09, myMPC_lbIdx09, myMPC_dllbcc09);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub09, myMPC_sub09, myMPC_lubbysub09, myMPC_dzcc09, myMPC_ubIdx09, myMPC_dlubcc09);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl10, myMPC_slb10, myMPC_llbbyslb10, myMPC_dzcc10, myMPC_lbIdx10, myMPC_dllbcc10);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub10, myMPC_sub10, myMPC_lubbysub10, myMPC_dzcc10, myMPC_ubIdx10, myMPC_dlubcc10);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl11, myMPC_slb11, myMPC_llbbyslb11, myMPC_dzcc11, myMPC_lbIdx11, myMPC_dllbcc11);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub11, myMPC_sub11, myMPC_lubbysub11, myMPC_dzcc11, myMPC_ubIdx11, myMPC_dlubcc11);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl12, myMPC_slb12, myMPC_llbbyslb12, myMPC_dzcc12, myMPC_lbIdx12, myMPC_dllbcc12);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub12, myMPC_sub12, myMPC_lubbysub12, myMPC_dzcc12, myMPC_ubIdx12, myMPC_dlubcc12);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl13, myMPC_slb13, myMPC_llbbyslb13, myMPC_dzcc13, myMPC_lbIdx13, myMPC_dllbcc13);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub13, myMPC_sub13, myMPC_lubbysub13, myMPC_dzcc13, myMPC_ubIdx13, myMPC_dlubcc13);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl14, myMPC_slb14, myMPC_llbbyslb14, myMPC_dzcc14, myMPC_lbIdx14, myMPC_dllbcc14);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub14, myMPC_sub14, myMPC_lubbysub14, myMPC_dzcc14, myMPC_ubIdx14, myMPC_dlubcc14);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl15, myMPC_slb15, myMPC_llbbyslb15, myMPC_dzcc15, myMPC_lbIdx15, myMPC_dllbcc15);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub15, myMPC_sub15, myMPC_lubbysub15, myMPC_dzcc15, myMPC_ubIdx15, myMPC_dlubcc15);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl16, myMPC_slb16, myMPC_llbbyslb16, myMPC_dzcc16, myMPC_lbIdx16, myMPC_dllbcc16);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub16, myMPC_sub16, myMPC_lubbysub16, myMPC_dzcc16, myMPC_ubIdx16, myMPC_dlubcc16);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl17, myMPC_slb17, myMPC_llbbyslb17, myMPC_dzcc17, myMPC_lbIdx17, myMPC_dllbcc17);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub17, myMPC_sub17, myMPC_lubbysub17, myMPC_dzcc17, myMPC_ubIdx17, myMPC_dlubcc17);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl18, myMPC_slb18, myMPC_llbbyslb18, myMPC_dzcc18, myMPC_lbIdx18, myMPC_dllbcc18);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub18, myMPC_sub18, myMPC_lubbysub18, myMPC_dzcc18, myMPC_ubIdx18, myMPC_dlubcc18);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl19, myMPC_slb19, myMPC_llbbyslb19, myMPC_dzcc19, myMPC_lbIdx19, myMPC_dllbcc19);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub19, myMPC_sub19, myMPC_lubbysub19, myMPC_dzcc19, myMPC_ubIdx19, myMPC_dlubcc19);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl20, myMPC_slb20, myMPC_llbbyslb20, myMPC_dzcc20, myMPC_lbIdx20, myMPC_dllbcc20);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub20, myMPC_sub20, myMPC_lubbysub20, myMPC_dzcc20, myMPC_ubIdx20, myMPC_dlubcc20);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl21, myMPC_slb21, myMPC_llbbyslb21, myMPC_dzcc21, myMPC_lbIdx21, myMPC_dllbcc21);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub21, myMPC_sub21, myMPC_lubbysub21, myMPC_dzcc21, myMPC_ubIdx21, myMPC_dlubcc21);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl22, myMPC_slb22, myMPC_llbbyslb22, myMPC_dzcc22, myMPC_lbIdx22, myMPC_dllbcc22);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub22, myMPC_sub22, myMPC_lubbysub22, myMPC_dzcc22, myMPC_ubIdx22, myMPC_dlubcc22);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_10(myMPC_ccrhsl23, myMPC_slb23, myMPC_llbbyslb23, myMPC_dzcc23, myMPC_lbIdx23, myMPC_dllbcc23);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_10(myMPC_ccrhsub23, myMPC_sub23, myMPC_lubbysub23, myMPC_dzcc23, myMPC_ubIdx23, myMPC_dlubcc23);
myMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_8(myMPC_ccrhsl24, myMPC_slb24, myMPC_llbbyslb24, myMPC_dzcc24, myMPC_lbIdx24, myMPC_dllbcc24);
myMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_8(myMPC_ccrhsub24, myMPC_sub24, myMPC_lubbysub24, myMPC_dzcc24, myMPC_ubIdx24, myMPC_dlubcc24);
myMPC_LA_VSUB7_496(myMPC_l, myMPC_ccrhs, myMPC_s, myMPC_dl_cc, myMPC_ds_cc);
myMPC_LA_VADD_248(myMPC_dz_cc, myMPC_dz_aff);
myMPC_LA_VADD_200(myMPC_dv_cc, myMPC_dv_aff);
myMPC_LA_VADD_496(myMPC_dl_cc, myMPC_dl_aff);
myMPC_LA_VADD_496(myMPC_ds_cc, myMPC_ds_aff);
info->lsit_cc = myMPC_LINESEARCH_BACKTRACKING_COMBINED(myMPC_z, myMPC_v, myMPC_l, myMPC_s, myMPC_dz_cc, myMPC_dv_cc, myMPC_dl_cc, myMPC_ds_cc, &info->step_cc, &info->mu);
if( info->lsit_cc == myMPC_NOPROGRESS ){
PRINTTEXT("Line search could not proceed at iteration %d, exiting.\n",info->it+1);
exitcode = myMPC_NOPROGRESS; break;
}
info->it++;
}
output->u1[0] = myMPC_z00[8];
output->u1[1] = myMPC_z00[9];

return exitcode;
}
