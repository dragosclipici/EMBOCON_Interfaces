#include "EMBOCON_DataTypes.h"
#include "EMBOCON_OptimizerInterface.h"
 
#include "acado.h"

/* ACADO specific code START **************************************************/

#ifndef __MATLAB__
#include <stdio.h>
#define PRINTTEXT printf
#else
#include "mex.h"
#define PRINTTEXT mexPrintf
#endif

/* ACADO code generation related variables */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

/* qpOASES related structures */
Vars         vars;
Params       params;

/* Reference */
double xref[ ACADO_NX ];

/* ACADO specific code END ****************************************************/

emb_optimizer createOptimizer(emb_model model)
{
	return model;
}

void freeOptimizer(emb_optimizer optim)
{
}

emb_opt_context initOptContext(emb_optimizer optim)
{
	return optim;
}

emb_opt_context cloneOptContext(emb_optimizer optim, emb_opt_context sourceContext)
{
	return optim;
}

void freeOptContext(emb_optimizer optim, emb_opt_context optContext)
{
}

emb_size_type getOptModelParameterCount(emb_optimizer optim)
{
	return ACADO_NX;
}

emb_size_type getOptModelStateCount(emb_optimizer optim)
{
	return ACADO_NX;
}

emb_size_type getOptModelInputCount(emb_optimizer optim)
{
	return ACADO_NU;
}

emb_size_type getOptModelOutputCount(emb_optimizer optim)
{
	return 0;
}

int getOptModelStateType(emb_optimizer optim, int is_algebraic[])
{
	is_algebraic = 0;
	return 0;
}

int setOptModelParameter(emb_optimizer optim, emb_opt_context opt_context, const double p[], const int p_flag[])
{
	return 0; /*there are no model parameters to be set*/
}

int setOptModelParameterUncertainty(emb_optimizer optim, const int uncType[], const double lower[], const double upper[], const double cov[])
{
	return 1; /*always fail, no uncertainty is taken into account*/
}

emb_size_type getOptParameterCount(emb_optimizer optim)
{
	return ACADO_NX; /*xref*/ 
}

/* ACADO specific code START **************************************************/

int setOptParameter(emb_optimizer optim, emb_opt_context opt_context, const double optp[])
{
	unsigned int np = getOptModelParameterCount(optim);
	int i;
    xref[ 0 ] = optp[ 0 ];
	xref[ 1 ] = optp[ 2 ];
	xref[ 2 ] = optp[ 1 ];
	xref[ 3 ] = optp[ 3 ];
	xref[ 4 ] = optp[ 4 ];
	xref[ 5 ] = optp[ 5 ];
	xref[ 6 ] = optp[ 6 ];
	xref[ 7 ] = optp[ 7 ];
	
	return 0;
}

int makeOptStep(emb_optimizer optim, emb_opt_context opt_context, const double xcur[], const double uprev[], double ucur[])
{
	emb_size_type nx = getOptModelStateCount(optim);
	emb_size_type nu = getOptModelInputCount(optim);
	
	int i, j, status;
	
	static unsigned firstRun = 1;
	
	double feedback[ ACADO_NX ];
	
	/* Fill the reference for states */
	for (i = 0; i < ACADO_N; ++i)
		for (j = 0; j < ACADO_NX; ++j)
			acadoVariables.xRef[i * ACADO_NX + j] = xref[ j ];
	
	/* Fill the reference for controls -- always zero :) */
	for (i = 0; i < ACADO_N; ++i)
		for (j = 0; j < ACADO_NU; ++j)
			acadoVariables.uRef[i * ACADO_NU + j] = 0.0;
		
	if ( firstRun )
	{
		/* Print header */
		PRINTTEXT(	"\nACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.\n"
					"Copyright (C) 2008-2011 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.\n"
					"Developed within the Optimization in Engineering Center (OPTEC) under\n"
					"supervision of Moritz Diehl. All rights reserved.\n\n"
					"ACADO Toolkit is distributed under the terms of the GNU Lesser\n"
					"General Public License 3 in the hope that it will be useful,\n"
					"but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
					"MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the\n"
					"GNU Lesser General Public License for more details.\n\n" );
		
		/* Initialize the shooting nodes */
		
		for (i = 0; i < ACADO_N + 1; ++i)
		{
			acadoVariables.x[i * ACADO_NX + 0] = 0.1;
			acadoVariables.x[i * ACADO_NX + 1] = 0.0;
			
			acadoVariables.x[i * ACADO_NX + 2] = 0.7;
			acadoVariables.x[i * ACADO_NX + 3] = 0.0;
			
			acadoVariables.x[i * ACADO_NX + 4] = 0.0;
			acadoVariables.x[i * ACADO_NX + 5] = 0.0;
			
			acadoVariables.x[i * ACADO_NX + 6] = 0.0;
			acadoVariables.x[i * ACADO_NX + 7] = 0.0;
		}
			
		for (i = 0; i < ACADO_N; ++i)
			for (j = 0; j < ACADO_NU; ++j)
				acadoVariables.u[i * ACADO_NU + j] = 0.0;
		
		firstRun = 0;
	}
	else
	{
		/* Shift the states and the controls */
		
		shiftStates( 0 );
		shiftControls( 0 );
	}
	
	/* Execute the preparation phase */
	preparationStep();
	
	/* Prepare the feedback signal */
	feedback[ 0 ] = xcur[ 0 ];
	feedback[ 1 ] = xcur[ 2 ];
	feedback[ 2 ] = xcur[ 1 ];
	feedback[ 3 ] = xcur[ 3 ];
	feedback[ 4 ] = xcur[ 4 ];
	feedback[ 5 ] = xcur[ 5 ];
	feedback[ 6 ] = xcur[ 6 ];
	feedback[ 7 ] = xcur[ 7 ];
	
	/* Execute the feedback phase */
	status = feedbackStep( feedback );
	
	PRINTTEXT("KKT tolerance: %1.5e\n", getKKT());

	if ( status )
	{
		PRINTTEXT("Problem in the QP solver detected. QP error code is: %d\n", status);
		
		return 1;	
	}

	/* Assign to input */
	for( i = 0; i < ACADO_NU; i++ ) {
		ucur[ i ] = acadoVariables.u[ i ];
	}
	
	return 0;
}

/* ACADO specific code END ****************************************************/

int getOptStepDetails(emb_optimizer optim, emb_opt_context opt_context, emb_size_type predHor, emb_size_type *actHor, double uPredict[], double yPredict[], double xPredict[])
{
	return 1; /*no extra details available*/
}

int getOptModelParameterName(emb_optimizer optim, int name_idx, char par_name[], size_t maxlen, size_t *reqlen)
{
	return 1; /* no model parameters, so no names */
}

int getOptModelStateName(emb_optimizer optim, int name_idx, char x_name[], size_t maxlen, size_t *reqlen)
{
	char stateNames[ACADO_NX][6] = {"x_C", "x_L", "v_C", "v_L", "theta", "omega", "u_C", "u_L"};
	
	unsigned int i = 0;
	while ( stateNames[name_idx][i] != '\0' )
	{
		i++;
	}
	*reqlen = i+1; /*length of string, including '\0'*/


	if (maxlen >= *reqlen)
	{
		for ( i = 0; i < *reqlen; i++)
		{
			x_name[i] = stateNames[name_idx][i];
		}
	}
	else if ( maxlen != 0) /*maxlen < length*/
	{
		for ( i = 0; i < maxlen - 1; i++)
		{
			x_name[i] = stateNames[name_idx][i];
		}
		x_name[maxlen-1] = '\0';
	}
	return 0;
}


int getOptModelInputName(emb_optimizer optim, int name_idx, char u_name[], size_t maxlen, size_t *reqlen)
{
	char inputNames[ACADO_NU][5] = {"u_CR", "u_LR"};
	
	unsigned int i = 0;
	while ( inputNames[name_idx][i] != '\0' )
	{
		i++;
	}
	*reqlen = i+1; /*length of string, including '\0'*/


	if (maxlen >= *reqlen)
	{
		for ( i = 0; i < *reqlen; i++)
		{
			u_name[i] = inputNames[name_idx][i];
		}
	}
	else if (maxlen != 0) /*maxlen < length*/
	{
		for ( i = 0; i < maxlen - 1; i++)
		{
			u_name[i] = inputNames[name_idx][i];
		}
		u_name[maxlen-1] = '\0';
	}
	return 0;
}

int getOptModelOutputName(emb_optimizer optim, int name_idx, char y_name[], size_t maxlen, size_t *reqlen)
{
	char outputNames[ACADO_NX][6] = {"x_C", "x_L", "v_C", "v_L", "theta", "omega", "u_C", "u_L"};
	
	unsigned int i = 0;
	while ( outputNames[name_idx][i] != '\0' )
	{
		i++;
	}
	*reqlen = i+1; /*length of string, including '\0'*/


	if (maxlen >= *reqlen)
	{
		for ( i = 0; i < *reqlen; i++)
		{
			y_name[i] = outputNames[name_idx][i];
		}
	}
	else if (maxlen != 0) /*maxlen < length */
	{
		for ( i = 0; i < maxlen - 1; i++)
		{
			y_name[i] = outputNames[name_idx][i];
		}
		y_name[maxlen-1] = '\0';
	}
	return 0;
}
