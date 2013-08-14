/*
 ******************************************************************************
 ***** EMBOCON OPTIMIZER INTERFACE ********************************************
 ******************************************************************************
 * EMBOCON_OptimizerInterface.cpp
 *
 *  Created on: 27.02.2013
 *      Author: schoppmeyerc
 *      E-Mail: christian.schoppmeyer@bci.tu-dortmund.de
 *      Version: 1.0
 ******************************************************************************
 * This is a dummy implementation for testing the different function calls of
 * the EMBOCON OptimizerInterface.
 ******************************************************************************
 */

#include "EMBOCON_OptimizerInterface.h"

/**
 * STANDARD EMBOCON VARIABLES AND FUNCTION CALLS:
 */
emb_model myModel;

emb_optimizer createOptimizer(emb_model model)
{
	//inputs: 2
	//outputs: 8
	//states: 8
	//parameters: 0

	myModel = model;

	initialize();

	return 0;
}

void freeOptimizer(emb_optimizer optim)
{
	freeModel(myModel);
	return;
}

emb_opt_context initOptContext(emb_optimizer optim)
{
	return 0;
}

emb_opt_context cloneOptContext(emb_optimizer optim, emb_opt_context sourceContext)
{
	return 0;
}

void freeOptContext(emb_optimizer optim, emb_opt_context optContext)
{
	return;
}

emb_size_type getOptModelParameterCount(emb_optimizer optim)
{
	return getParameterCount(myModel);
}

emb_size_type getOptModelStateCount(emb_optimizer optim)
{
	return getStateCount(myModel);
}

emb_size_type getOptModelInputCount(emb_optimizer optim)
{
	return getInputCount(myModel);
}

emb_size_type getOptModelOutputCount(emb_optimizer optim)
{
	return getOutputCount(myModel);
}

int getOptModelStateType(emb_optimizer optim, int is_algebraic[])
{
	return getStateType(myModel, is_algebraic);
}

int setOptModelParameter(emb_optimizer optim, emb_opt_context opt_context, const double p[], const int p_flag[])
{
	return setParameter(myModel, p);
}

int setOptModelParameterUncertainty(emb_optimizer optim, const int uncType[], const double lower[], const double upper[], const double cov[])
{
	return 0;
}

emb_size_type getOptParameterCount(emb_optimizer optim)
{
	return 0;
}

int setOptParameter(emb_optimizer optim, emb_opt_context opt_context, const double optp[])
{
	return 0;
}

int makeOptStep(emb_optimizer optim, emb_opt_context opt_context, const double xcur[], const double uprev[], double ucur[])
{
	// Scale state
	double x_scaling[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
	double scaled_x[8];
	unsigned int i;
	for(i = 0; i < 8; i++)
	{
		scaled_x[i] = xcur[i] / x_scaling[i];
	}
	eval(scaled_x);

	getSolution(ucur);

	ucur[0] = ucur[0];
	ucur[1] = ucur[1];

	return 0;
}

int getOptStepDetails(emb_optimizer optim, emb_opt_context opt_context, emb_size_type predHor, emb_size_type *actHor, double uPredict[], double yPredict[], double xPredict[])
{
	return 0;
}

int getOptModelParameterName(emb_optimizer optim, int name_idx, char par_name[], size_t maxlen, size_t *reqlen)
{
	return getParameterName(myModel, name_idx, par_name, maxlen, reqlen);
}

int getOptModelStateName(emb_optimizer optim, int name_idx, char x_name[], size_t maxlen, size_t *reqlen)
{
	return getStateName(myModel, name_idx, x_name, maxlen, reqlen);
}

int getOptModelInputName(emb_optimizer optim, int name_idx, char u_name[], size_t maxlen, size_t *reqlen)
{
	return getInputName(myModel, name_idx, u_name, maxlen, reqlen);
}

int getOptModelOutputName(emb_optimizer optim, int name_idx, char y_name[], size_t maxlen, size_t *reqlen)
{
	return getOutputName(myModel, name_idx, y_name, maxlen, reqlen);
}

int getOptParameterName(emb_optimizer optim, int name_idx, char par_name[], size_t maxlen, size_t *reqlen)
{
	return -1;
}

