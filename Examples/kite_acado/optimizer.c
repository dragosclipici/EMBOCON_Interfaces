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

/* The text files used for reference and initialization */
FILE *initStates, *initControls, *ref;

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

int setOptParameter(emb_optimizer optim, emb_opt_context opt_context, const double optp[])
{
	unsigned int np = getOptModelParameterCount(optim);
	int i;
	
	/* the reference trajectory is read from a txt file */
	
	return 0;
}

int makeOptStep(emb_optimizer optim, emb_opt_context opt_context, const double xcur[], const double uprev[], double ucur[])
{
	emb_size_type nx = getOptModelStateCount(optim);
	emb_size_type nu = getOptModelInputCount(optim);
	
	int i, j, status, nil;
	
	static unsigned firstRun = 1;
	
	double feedback[ ACADO_NX ];
		
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
			
		/* INITIALIZE THE STATES AND CONTROLS: */
		/* ---------------------------------------- */
		initStates = fopen("TX.txt","r");
		for( i = 0; i < (ACADO_N+1); ++i )  {
			nil = fscanf( initStates, "%lf", &acadoVariables.x[i*ACADO_NX] ); /* ignore first column */
			for( j = 0; j < ACADO_NX; ++j ) nil = fscanf( initStates, "%lf", &acadoVariables.x[i*ACADO_NX+j] );
		}
	  
		fclose(initStates);
      
		initControls = fopen("TU.txt","r");
		for( i = 0; i < ACADO_N; ++i )  {
			nil = fscanf( initControls, "%lf", &acadoVariables.u[i*ACADO_NU] ); /* ignore first column */
			for( j = 0; j < ACADO_NU; ++j ) nil = fscanf( initControls, "%lf", &acadoVariables.u[i*ACADO_NU+j] );
		}
		fclose(initControls);
		
		
		/* INITIALIZE THE STATES AND CONTROL REFERENCE: */
		/* -------------------------------------------- */
		ref = fopen("RefTraj.txt","r");
		for( i = -1; i < ACADO_N; ++i )  {
			if( i >= 0 ) {
				nil = fscanf( ref, "%lf", &acadoVariables.xRef[i*ACADO_NX] ); /* ignore first column */
				for( j = 0; j < ACADO_NX; ++j ) nil = fscanf( ref, "%lf", &acadoVariables.xRef[i*ACADO_NX+j] );
				for( j = 0; j < ACADO_NU; ++j ) nil = fscanf( ref, "%lf", &acadoVariables.uRef[i*ACADO_NU+j] );
			}
			else {
				for( j = 0; j < (ACADO_NX+ACADO_NU+1); ++j ) nil = fscanf( ref, "%lf", &acadoVariables.xRef[j] );
			}
		}
		
		firstRun = 0;
	}
	else
	{
		/* Shift the states and the controls */
		shiftStates( 0 );
		shiftControls( 0 );
		
		/* SHIFT THE REFERENCE (ONLY STATES BECAUSE CONTROL REFERENCE CONSTANT HERE) */
        /* ----------------------------------- */
		for( i = 0; i < ACADO_N-1; i++ ) {
			for( j = 0; j < ACADO_NX; j++ ) {
				acadoVariables.xRef[i*ACADO_NX+j] = acadoVariables.xRef[(i+1)*ACADO_NX+j];
			}
		}
		for( i = 0; i < ACADO_N-1; i++ ) {
			for( j = 0; j < ACADO_NU; j++ ) {
				acadoVariables.uRef[i*ACADO_NU+j] = acadoVariables.uRef[(i+1)*ACADO_NU+j];
			}
		}
		nil = fscanf( ref, "%lf", &acadoVariables.xRef[(ACADO_N-1)*ACADO_NX] ); /* ignore first column */
		for( j = 0; j < ACADO_NX; ++j ) nil = fscanf( ref, "%lf", &acadoVariables.xRef[(ACADO_N-1)*ACADO_NX+j] );
		for( j = 0; j < ACADO_NU; ++j ) nil = fscanf( ref, "%lf", &acadoVariables.uRef[(ACADO_N-1)*ACADO_NU+j] );
		for( j = 0; j < ACADO_NX; j++ ) {
			acadoVariables.x[ACADO_N*ACADO_NX+j] = acadoVariables.xRef[(ACADO_N-1)*ACADO_NX+j];
		}
	}
	
	/* Execute the preparation phase */
	preparationStep();
	
	/* Prepare the feedback signal */
	for( i = 0; i < ACADO_NX; i++ ) {
		feedback[ i ] = xcur[ i ];
	}
	
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
	char stateNames[ACADO_NX][7] = {"x", "y", "z", "dx", "dy", "dz", "e11", "e12", "e13", "e21", "e22", "e23", "e31", "e32", "e33", "w1", "w2", "w3", "r", "dr", "delta", "ddelta"};
	
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
	char inputNames[ACADO_NU][8] = {"dddelta", "ddr", "ur", "up"};
	
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
	char outputNames[ACADO_NX][7] = {"x", "y", "z", "dx", "dy", "dz", "e11", "e12", "e13", "e21", "e22", "e23", "e31", "e32", "e33", "w1", "w2", "w3", "r", "dr", "delta", "ddelta"};
	
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
