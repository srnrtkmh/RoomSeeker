/**************************************************************************************************/
/*                                                                                                */
/* FILE : basic_calc.h                                                                            */
/* MEMO : Library to calculate basic mathematical functions                                       */
/*                                                                                                */
/* 2010/10/03 : Start to edit this file                                                           */
/*              (sign(), sat(), b_spline_norm())                                                  */
/* 2010/10/31 : Add "inv2()", "inv3()", "inv4()", "inv5()", "pseinv()"                            */
/* 2010/11/11 : Add "inv6()"                                                                      */
/* 2010/12/15 : Add "inv()" to generalize calculation of inverse of square matrix                 */
/* 2010/12/16 : Add "pse_inv()" to generalize calculation of pseudo inverse matrix                */
/*              Modify "inv()" as its argument has the pointer of the head element of each matrix */
/*              and the degree of the matrices                                                    */
/* 2011/01/10 : Add "det()" function to calculate determinant n x n matrix                        */
/* 2011/05/13 : Delete "inv2()", "inv3()", "inv4()", "inv5()", "inv6()" function                  */
/* 2011/09/02 : Add "#ifndef BASIC_CALC ..."                                                      */
/**************************************************************************************************/

#ifndef BASIC_CALC
#define BASIC_CALC

//================================================================================================//
// Function Prototypes                                                                            //
//================================================================================================//
double sign(double x);
double sat(double x, double width);
int fact(int n);
int nCk(int n, int k);
double b_spline_norm(double z, double epsilon, int m);

// Calculate Invertse Matrix
void inv(double *A, double *A_inv, int deg);

// Calculate Pseudo Inverse Matrix
// m : A rows, n : A columns
void pseinv(double *A, double *pseinvA, int m, int n);

double det(double *A, int n);	// Determinant of nxn matrix

#endif
