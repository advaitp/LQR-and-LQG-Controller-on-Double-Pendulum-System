
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output function
 *
 */
void Cart_Pendulum_Outputs_wrapper(const real_T *u,
			real_T *x,
			const real_T *xC)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
/* This sample sets the output equal to the input
      y0[0] = u0[0]; 
 For complex signals use: y0[0].re = u0[0].re; 
      y0[0].im = u0[0].im;
      y1[0].re = u1[0].re;
      y1[0].im = u1[0].im;
*/
y0[0] = xC[0];
y0[1] = xC[1];
y0[2] = xC[2];
y0[3] = xC[3];
y0[4] = xC[4];
y0[5] = xC[5];
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Derivatives function
 *
 */
void Cart_Pendulum_Derivatives_wrapper(const real_T *u,
			real_T *x,
			real_T *dx,
			real_T *xC)
{
/* %%%-SFUNWIZ_wrapper_Derivatives_Changes_BEGIN --- EDIT HERE TO _END */
// double m = 1;
// double M = 5;
// double L = 2;
// double g = -10;
// double d = 1;
double Sy;
double Cy;
Sy = sin(xC[2]);
Cy = cos(xC[2]);
dxdt[6] = {0, 0, 0, 0, 0, 0};
% y =[x x_dot theta1 theta1dot theta2 theta2dot]
% y_dot = [x_dot x_dot_dot theta1dot theta1dot_dot theta2dot theta2dot_dot]
dxdt[0] = x[1]; 
dxdt[1]=(F-(g/2)*(m1*sin(2*x[2])+m2*sin(2*x[4]))-(m1*l1*(x[3]^2)*sin(x[2]))-(m2*l2*(x[5]^2)*sin(x[4])))/(M+m1*((sin(x[2]))^2)+m2*((sin(x[4]))^2));
dxdt[2]= x[3]; 
dxdt[3]= (dxdt[1]*cos(x[2])-g*(sin(x[2])))/l1; 
dxdt[4]= x[5]; 
dxdt[5]= (dxdt[1]*cos(x[4])-g*(sin(x[4])))/l2;
/* %%%-SFUNWIZ_wrapper_Derivatives_Changes_END --- EDIT HERE TO _BEGIN */
}

