/**************************************************************************************************/
/* FILE : basic_calc.c                                                                            */
/* MEMO : Basic Calculation Functions not included in "math.h"                                    */
/*                                                                                                */
/* 2010/10/03 : Start to edit this file                                                           */
/*              (sign(), sat(), fact(), nCk(), b_spline_norm())                                   */
/* 2010/10/31 : Add "inv2()", "inv3()", "inv4()", "inv5()", "pseinv()"                            */
/* 2010/12/15 : Add "inv()" to generalize calculation of inverse of square matrix                 */
//* 2011/01/10 : Add "det()" function to calculate determinant n x n matrix                        */
/**************************************************************************************************/

/**************************************************************************************************/
/* Include Files                                                                                  */
/**************************************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "basic_calc.h"

//================================================================================================//
// Sign Function                                                                                  //
//================================================================================================//
double sign(double x)
{
	if(x>0)			return(1.0);
	else if(x<0)	return(-1.0);
	else			return(0.0);
}

//================================================================================================//
// Saturation Function                                                                            //
//================================================================================================//
double sat(double x, double width)
{
    if(x>width)			return(1.0);
    else if(x<-width)	return(-1.0);
    else				return(x/width);
}

//================================================================================================//
// Function to Calculate Factorial                                                                //
//================================================================================================//
int fact(int n)
{
	int tmp=1, i;
	for(i=1; i<=n; i++)	tmp*=i;
	return(tmp);
}

//================================================================================================//
// Function to Calculate Combination                                                              //
//================================================================================================//
int nCk(int n, int k)
{
	if(k==0)		return(1);
	if(k==1)		return(n);
	else if(n==k)	return(1);
	else			return(nCk(n-1, k)+nCk(n-1, k-1));
}

//================================================================================================//
// Normalized B-Spline Function                                                                   //
//================================================================================================//
double b_spline_norm(double z, double epsilon, int m)
{
	int i, n = m+1;
	double Nm_0 = 0.0, Nm_z = 0.0, delta = 2.0*epsilon/n;
	double tmp=0.0;
	
	for(i = 0; i <=n; i++){
		if(i & 1)	tmp = -1.0;
		else		tmp = 1.0;
		
	    if((z + (n/2.0 - i) * delta)>0)		Nm_z += tmp * nCk(n, i) * pow(  z + (n/2.0 - i) * delta, (double)m);
	    if((0.0 + (n/2.0 - i) * delta)>0)	Nm_0 += tmp * nCk(n, i) * pow(0.0 + (n/2.0 - i) * delta, (double)m);
	}
	
	return(Nm_z/Nm_0);
}

//================================================================================================//
// Calculate inverse matrix                                                                       //
// double *A     : pointer to the source matrix                                                   //
// double *A_inv : pointer to the target matrix                                                   //
// int deg       : size of the matrix                                                             //
//================================================================================================//
void inv(double *A, double *A_inv, int deg)
{
	int i, j, k, max_row;
	double inv_diag, tmp;
	double A_copy[deg][deg];
	
	// Copy source matrix
	for(i=0; i<deg; i++) for(j=0; j<deg; j++) A_copy[i][j]=*(A+deg*i+j);
	
	// Initialize target matrix with unit matrix
	for(i=0; i<deg; i++){
		for(j=0; j<deg; j++){
			if(i == j) *(A_inv+deg*i+j) = 1.0;
			else       *(A_inv+deg*i+j) = 0.0;
		}
	}
	
	// Procedure of the Gaussian Elimination
	for(i=0; i<deg; i++){
		// Search the biggest element among the "i"th column elements
		for(max_row=i, j=i+1; j<deg; j++) if(fabs(A_copy[j][i])>fabs(A_copy[max_row][i])) max_row = i;
		
		// Exchange "i"th row and "max_row"th row if "i" is not equal to the "max_row"
		if(i != max_row){
			for(j=0; j<deg; j++){
				tmp = A_copy[i][j];      A_copy[i][j] = A_copy[max_row][j];         A_copy[max_row][j] = tmp;
				tmp = *(A_inv+deg*i+j);  *(A_inv+deg*i+j) = *(A_inv+deg*max_row+j); *(A_inv+deg*max_row+j) = tmp;
			}
		}
		
		// Multiply the inverse of the diagonal element to each elemnt of the "i" row
		// to make the diagonal element of the source matrix to 0
		inv_diag = 1.0/A_copy[i][i];
		for(j=0; j<deg; j++){									
			A_copy[i][j] *= inv_diag;
			*(A_inv+deg*i+j) *= inv_diag;
		}
		
		// Eliminate "i"th column element of the source matrix except the diagonal one
		for(j=0; j<deg; j++){
			if(j != i){
				tmp = A_copy[j][i];
				for(k=0; k<deg; k++){
					A_copy[j][k]     -= tmp * A_copy[i][k];
					*(A_inv+deg*j+k) -= tmp * *(A_inv+deg*i+k);
				}
			}
		}
	}
}

//================================================================================================//
// Calculate pseudo inverse matrix                                                                //
// double A      : pointer to the source matrix                                                   //
// double A_inv  : pointer to the target matrix                                                   //
// int m         : row size of the matrix                                                         //
// int n         : column size of the matrix                                                      //
//================================================================================================//
void pseinv(double *A, double *A_pseinv, int m, int n)
{
	int i, j, k;
	double tmp[m][m], tmp_inv[m][m];
	
	for(i=0; i<m; i++) for(j=0; j<m; j++) for(k=0, tmp[i][j]=0.0; k<n; k++) tmp[i][j]+=*(A+n*i+k) * (*(A+n*j+k));						// Calculate A*AT
	inv(&tmp[0][0], &tmp_inv[0][0], sizeof(tmp[0])/sizeof(tmp[0][0]));																												// Calculate (A*AT)^(-1)
	for(i=0; i<n; i++) for(j=0; j<m; j++) for(k=0, *(A_pseinv+m*i+j)=0.0; k<m; k++) *(A_pseinv+m*i+j)+=*(A+n*k+i) * tmp_inv[k][j];	// Calculate AT*(A*AT)^(-1)
}


//================================================================================================//
// Calculate determinant of n x n matrix                                                          //
//================================================================================================//
double det(double *A, int n)
{
	int i, j, k;
	double ret, tmp;
	double A_tmp[n][n];
	
	for(i=0; i<n; i++) for(j=0; j<n; j++) A_tmp[i][j]=*(A+n*i+j);		// Copy source matrix
	for(i=0; i<n-1; i++){
		for(j=i; j<n-1; j++){
			tmp=A_tmp[j+1][i]/A_tmp[i][i];
			for(k=0; k<n; k++) A_tmp[j+1][k]-=A_tmp[i][k]*tmp;
		}
	}
	for(i=0, ret=1.0; i<n; i++) ret*=A_tmp[i][i];
	
	return(ret);
}
