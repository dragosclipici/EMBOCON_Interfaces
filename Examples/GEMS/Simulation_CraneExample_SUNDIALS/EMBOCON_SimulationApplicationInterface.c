/*
 ******************************************************************************
 ***** EMBOCON SIMULATION APPLICATION INTERFACE *******************************
 ******************************************************************************
 * EMBOCON_SimulationApplicationInterface.c
 *
 *  Created on: 10.10.2011
 *      Author: schoppmeyerc
 *      E-Mail: christian.schoppmeyer@bci.tu-dortmund.de
 *      Version: 1.0
 ******************************************************************************
 ******************************************************************************
 ******************************************************************************
 */

#include "EMBOCON_SimulationApplicationInterface.h"
#include <math.h>
#include <stdio.h>

/* Header files with a description of contents used */

#include <cvode/cvode.h>             /* prototypes for CVODE fcts., consts. */
#include <nvector/nvector_serial.h>  /* serial N_Vector types, fcts., macros */
#include <cvode/cvode_dense.h>       /* prototype for CVDense */
#include <sundials/sundials_dense.h> /* definitions DlsMat DENSE_ELEM */
#include <sundials/sundials_types.h> /* definition of type realtype */

//parameters for the states
double Y[8];

int run_sundials(const double ucur[], double Y[]);

emb_simapp createSimApp()
{
	return 0;
}

void freeSimApp(emb_simapp simapp)
{
	return;
}

void initSimApp(emb_simapp simapp)
{
	Y[0] = 0.0; //x_C cart position (m)
	Y[1] = 0.75; //x_L cable length (m)
	Y[2] = 0; //v_C cart velocity (m/s)
	Y[3] = 0; //v_L cable velocity (m/s)
	Y[4] = 0; //theta cable deflection (rad)
	Y[5] = 3.14159265359 / 4.0; //omega cable deflection rate (rad/s) //pi/4
	Y[6] = 0; //u_C cart controller voltage (V)
	Y[7] = 0; //u_L cable controller voltage (V)

	return;
}

int makeSimStep(emb_simapp simapp, const double ucur[], double meascur[], double pcur[])
{
	int result = run_sundials(ucur, Y); //CALL SUNDIALS MAIN OF SERGIO

	int i;
	for(i = 0; i < 8; i++)
	{
		meascur[i] = Y[i];
	}

	return result;
}

emb_size_type getSimParameterCount(emb_simapp simapp)
{
	return 0;
}

emb_size_type getSimStateCount(emb_simapp simapp)
{
	return 8;
}

int getTrueStates(emb_simapp simapp, double xtrue[])
{
	xtrue = Y;

	return 0;
}

int getTrueParameters(emb_simapp simapp, double ptrue[])
{
	return 0;
}

/* User-defined vector and matrix accessor macros: Ith, IJth */

/* These macros are defined in order to write code which exactly matches
   the mathematical problem description given above.

   Ith(v,i) references the ith component of the vector v, where i is in
   the range [1..NEQ] and NEQ is defined below. The Ith macro is defined
   using the N_VIth macro in nvector.h. N_VIth numbers the components of
   a vector starting from 0.

   IJth(A,i,j) references the (i,j)th element of the dense matrix A, where
   i and j are in the range [1..NEQ]. The IJth macro is defined using the
   DENSE_ELEM macro in dense.h. DENSE_ELEM numbers rows and columns of a
   dense matrix starting from 0. */

#define Ith(v,i)    NV_Ith_S(v,i-1)       /* Ith numbers components 1..NEQ */
#define IJth(A,i,j) DENSE_ELEM(A,i-1,j-1) /* IJth numbers rows,cols 1..NEQ */


/* Initial problem values */
#define NEQ   		RCONST(8.0)      /* number of differential equations. Using RCONST does the code precision independent (double, float...)  */
#define RTOL  		RCONST(1.0e-8)   /* scalar relative tolerance */
#define ATOL  		RCONST(1.0e-15)  /* scalar absolute tolerance */
#define T0    		RCONST(0.0)      /* initial time           */
#define T1    		RCONST(0.01)      /* first output time      */
#define TMULT 		RCONST(0.01)      /* output time factor     */ //MODIFIED FROM 10.0
#define T_INCREMENT RCONST(0.01)	     /*Increments of 1 second for the results output*/
#define NOUT  		RCONST(1.0)      /* number of output times */ //MODIFIED FROM 100


/*Problem Parameters*/
#define m     RCONST(1318)              //mass (g)
#define g     RCONST(9.81)              //gravity (m/s^2)
#define c     RCONST(0)                 //damping (kgm^2/s)
#define A_C   RCONST(0.047418203070092) //Gain of cart vel. ctrl. (m/s/V)
#define tau_C RCONST(0.012790605943772) //time constant cart ctrl. (s)
#define A_L   RCONST(0.034087337273386) //Gain of winch vel. ctrl. (m/s/V)
#define tau_L RCONST(0.024695192379264) //time constant winch ctrl. (s)

/*Control inputs*/
/**
 * u_CR
 * u_LR
 */
realtype U1 = 0.0;
realtype U2 = 0.0;

/* Functions Called by the Solver */

static int f(realtype t, N_Vector y, N_Vector ydot, void *user_data);

/* Private functions to output results */

static void PrintOutput(realtype t, realtype y1, realtype y2, realtype y3, realtype y4, realtype y5, realtype y6, realtype y7, realtype y8);

/* Private function to print final statistics */

static void PrintFinalStats(void *cvode_mem);

/* Private function to check function return values */

static int check_flag(void *flagvalue, char *funcname, int opt);


/*
 *-------------------------------
 * Main loop for the integrator
 *-------------------------------
 */
int run_sundials(const double ucur[], double Y[])
{
	U1 = ucur[0];
	U2 = ucur[1];

	realtype reltol, abstol, t, tout;
	N_Vector y;
	void *cvode_mem;
	int flag, iout;

	y = NULL;
	cvode_mem = NULL;

	/* Create serial vector of length NEQ for I.C. and abstol */
	y = N_VNew_Serial(NEQ);
	if (check_flag((void *)y, "N_VNew_Serial", 0))
	{
		return(1);
	}

	/* Initialize y */
	int i;
	for(i = 0; i < NEQ; i++)
	{
		Ith(y,i+1) = Y[i];
	}

	/* Set the scalar relative tolerance */
	reltol = RTOL;
	/* Set the vector absolute tolerance */
	abstol = ATOL;

	/* Call CVodeCreate to create the solver memory and specify the
	 * Backward Differentiation Formula and the use of a Newton iteration */
	cvode_mem = CVodeCreate(CV_BDF, CV_NEWTON);
	if (check_flag((void *)cvode_mem, "CVodeCreate", 0))
	{
		return(1);
	}

	/* Call CVodeInit (==> changed to CVode() because version 2.3.0-1 used in ubuntu here) to initialize the integrator memory and specify the
	 * user's right hand side function in y'=f(t,y), the inital time T0, and
	 * the initial dependent variable vector y. */
	flag = CVodeInit(cvode_mem, f, T0, y);
	if (check_flag(&flag, "CVodeInit", 1))
	{
		return(1);
	}

	/* Call CVodeSVtolerances to specify the scalar relative tolerance
	 * and vector absolute tolerances */
	flag = CVodeSStolerances(cvode_mem, reltol, abstol);
	if (check_flag(&flag, "CVodeSVtolerances", 1))
	{
		return(1);
	}

	/* Call CVDense to specify the CVDENSE dense linear solver */
	flag = CVDense(cvode_mem, NEQ);
	if (check_flag(&flag, "CVDense", 1))
	{
		return(1);
	}

	/* In loop, call CVode, print results, and test for error.
     Break out of loop when NOUT preset output times have been reached.  */
	printf(" \nNon-linear Crane Example\n\n");

	iout = 0;  tout = T1;
	while(1)
	{
		flag = CVode(cvode_mem, tout, y, &t, CV_NORMAL);
		PrintOutput(t, Ith(y,1), Ith(y,2), Ith(y,3),Ith(y,4),Ith(y,5),Ith(y,6),Ith(y,7),Ith(y,8));

		if (check_flag(&flag, "CVode", 1))
		{
			break;
		}
		if (flag == CV_SUCCESS)
		{
			iout++;
			tout += T_INCREMENT;  // Integrate all the time the same interval
		}

		if (iout == NOUT)
		{
			break;
		}
	}

	/* Print some final statistics */
	PrintFinalStats(cvode_mem);

	for(i = 0; i < NEQ; i++)
	{
		Y[i] = Ith(y,i+1);
	}

	/* Free y vector */
	N_VDestroy_Serial(y);

	/* Free integrator memory */
	CVodeFree(&cvode_mem);

	return(0);
}


/*
 *-------------------------------
 * Functions called by the solver
 *-------------------------------
 */

/*
 * f routine. Compute function f(t,y).
 */

static int f(realtype t, N_Vector y, N_Vector ydot, void *user_data)
{
	realtype y1, y2, y3, y4, y5, y6, y7, y8;
	realtype yd1, yd2, yd3, yd4, yd5, yd6, yd7, yd8;
	realtype u1,u2;

	/*Auxiliary variables for writing ODE's easier*/
	y1 = Ith(y,1); y2 = Ith(y,2); y3 = Ith(y,3); y4 = Ith(y,4); y5 = Ith(y,5); y6 = Ith(y,6); y7 = Ith(y,7); y8 = Ith(y,8);

  	/*Set control inputs*/
	u1=U1;
	u2=U2;

	/**%working point
	1 x_C   = 1;     %cart position (m)
	2 x_L   = 1.5;   %cable length (m)
	3 v_C   = 0;     %cart velocity (ms)
	4 v_L   = 0;     %cable velocity (m/s)
	5 theta = 0;     %cable deflection (rad)
	6 omega = 0;     %cable deflection rate (rad/s)
	7 u_C   = 0;     %cart controller voltage (V)
	8 u_L   = 0;     %cable controller voltage (V)*/

	yd1 = Ith(ydot,1) = y3;
	yd2 = Ith(ydot,2) = y4;
	yd3 = Ith(ydot,3) = (-1/tau_C)*y3 + (A_C/tau_C)*y7;
	yd4 = Ith(ydot,4) = (-1/tau_L)*y4 + (A_L/tau_L)*y8;
	yd5 = Ith(ydot,5) = y6;
	yd6 = Ith(ydot,6) = -(1 / y2) * (g * sin(y5) + ((-1 / tau_C) * y3 + (A_C / tau_C) * y7) * cos(y5) + 2 * y4 * y6 + ((c * y6) / (m * y2)));
	yd7 = Ith(ydot,7) = u1;
	yd8 = Ith(ydot,8) = u2;

	return(0);
}


/*
 *-------------------------------
 * Private helper functions
 *-------------------------------
 */

static void PrintOutput(realtype t, realtype y1, realtype y2, realtype y3, realtype y4, realtype y5, realtype y6, realtype y7, realtype y8)
{
#if defined(SUNDIALS_EXTENDED_PRECISION)
  printf("At t = %0.4Le      y =%14.3Le  %14.3Le  %14.3Le %14.3Le %14.3Le %14.3Le %14.3Le %14.3Le \n", t, y1, y2, y3, y4, y5, y6, y7, y8);
#elif defined(SUNDIALS_DOUBLE_PRECISION)
  printf("At t = %0.2le      y =%1.3le  %1.3le  %1.3le %1.3le %1.3le %1.3le %1.3le %1.3le \n", t, y1, y2, y3, y4, y5, y6, y7, y8);
#else
  printf("At t = %0.4e      y =%14.6e  %14.6e  %14.6e %14.6e %14.6e %14.6e %14.6e %14.6e \n", t, y1, y2, y3, y4, y5, y6, y7, y8);
#endif

  return;
}



/*
 * Get and print some final statistics
 */

static void PrintFinalStats(void *cvode_mem)
{
  long int nst, nfe, nsetups, nje, nfeLS, nni, ncfn, netf, nge;
  int flag;

  flag = CVodeGetNumSteps(cvode_mem, &nst);
  check_flag(&flag, "CVodeGetNumSteps", 1);
  flag = CVodeGetNumRhsEvals(cvode_mem, &nfe);
  check_flag(&flag, "CVodeGetNumRhsEvals", 1);
  flag = CVodeGetNumLinSolvSetups(cvode_mem, &nsetups);
  check_flag(&flag, "CVodeGetNumLinSolvSetups", 1);
  flag = CVodeGetNumErrTestFails(cvode_mem, &netf);
  check_flag(&flag, "CVodeGetNumErrTestFails", 1);
  flag = CVodeGetNumNonlinSolvIters(cvode_mem, &nni);
  check_flag(&flag, "CVodeGetNumNonlinSolvIters", 1);
  flag = CVodeGetNumNonlinSolvConvFails(cvode_mem, &ncfn);
  check_flag(&flag, "CVodeGetNumNonlinSolvConvFails", 1);

  flag = CVDlsGetNumJacEvals(cvode_mem, &nje);
  check_flag(&flag, "CVDlsGetNumJacEvals", 1);
  flag = CVDlsGetNumRhsEvals(cvode_mem, &nfeLS);
  check_flag(&flag, "CVDlsGetNumRhsEvals", 1);

  flag = CVodeGetNumGEvals(cvode_mem, &nge);
  check_flag(&flag, "CVodeGetNumGEvals", 1);

  printf("\nFinal Statistics:\n");
  printf("nst = %-6ld nfe  = %-6ld nsetups = %-6ld nfeLS = %-6ld nje = %ld\n",
	 nst, nfe, nsetups, nfeLS, nje);
  printf("nni = %-6ld ncfn = %-6ld netf = %-6ld nge = %ld\n \n",
	 nni, ncfn, netf, nge);
}

/*
 * Check function return value...
 *   opt == 0 means SUNDIALS function allocates memory so check if
 *            returned NULL pointer
 *   opt == 1 means SUNDIALS function returns a flag so check if
 *            flag >= 0
 *   opt == 2 means function allocates memory so check if returned
 *            NULL pointer
 */

static int check_flag(void *flagvalue, char *funcname, int opt)
{
  int *errflag;

  /* Check if SUNDIALS function returned NULL pointer - no memory allocated */
  if (opt == 0 && flagvalue == NULL)
  {
    fprintf(stderr, "\nSUNDIALS_ERROR: %s() failed - returned NULL pointer\n\n",
	    funcname);
    return(1);
  }
  /* Check if flag < 0 */
  else if (opt == 1)
  {
    errflag = (int *) flagvalue;
    if (*errflag < 0)
    {
      fprintf(stderr, "\nSUNDIALS_ERROR: %s() failed with flag = %d\n\n",
	      funcname, *errflag);
      return(1);
    }
  }
  /* Check if function returned NULL pointer - no memory allocated */
  else if (opt == 2 && flagvalue == NULL)
  {
    fprintf(stderr, "\nMEMORY_ERROR: %s() failed - returned NULL pointer\n\n",
	    funcname);
    return(1);
  }

  return(0);
}



