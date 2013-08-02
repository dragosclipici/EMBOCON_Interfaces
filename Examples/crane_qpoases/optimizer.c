#include "EMBOCON_DataTypes.h"
#include "EMBOCON_OptimizerInterface.h"
 
#include "acado.h"

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
	return 8; /*xref*/ 
}

int setOptParameter(emb_optimizer optim, emb_opt_context opt_context, const double optp[])
{
	unsigned int np = getOptModelParameterCount(optim);
	
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
		/* Initialize the shooting nodes */
		
		for (i = 0; i < ACADO_N + 1; ++i)
		{
			/* Choose point for model linearization */
			acadoVariables.x[i * ACADO_NX + 0] = 0.0;
			acadoVariables.x[i * ACADO_NX + 1] = 0.0;
			
			acadoVariables.x[i * ACADO_NX + 2] = 0.75;
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
		
		/* Linearize the model */
		preparationStep();
	}
	
	
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

	if ( status )
	{
		PRINTTEXT("qpOASES problem detected. Error code %d\n", status);
		
		return 1;	
	}

	/* Assign to input */
 	ucur[ 0 ] = acadoVariables.u[ 0 ];
	ucur[ 1 ] = acadoVariables.u[ 1 ];
	
	return 0;
}


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
	char stateNames[8][6] = {"x_C", "x_L", "v_C", "v_L", "theta", "omega", "u_C", "u_L"};
	
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
	char inputNames[2][5] = {"u_CR", "u_LR"};
	
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
	char outputNames[8][6] = {"x_C", "x_L", "v_C", "v_L", "theta", "omega", "u_C", "u_L"};
	
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
