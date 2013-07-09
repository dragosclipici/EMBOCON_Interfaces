/*
 ******************************************************************************
 ***** EMBOCON MODEL INTERFACE ************************************************
 ******************************************************************************
 * EMBOCON_ModelInterface.c
 *
 *  Created on: 01.12.2011
 *      Author: schoppmeyerc
 *      E-Mail: christian.schoppmeyer@bci.tu-dortmund.de
 *      Version: 1.0
 ******************************************************************************
 * This is a dummy implementation for testing the different function calls of
 * the EMBOCON ModelInterface.
 ******************************************************************************
 */

#include "EMBOCON_ModelInterface.h"

emb_model createModel()
{
	return 0;
}

void freeModel(emb_model model)
{
	return;
}

emb_size_type getParameterCount(emb_model model)
{
	emb_size_type rtnVal = 0;

	return rtnVal;
}

emb_size_type getStateCount(emb_model model)
{
	emb_size_type rtnVal = 0;

	return rtnVal;
}

emb_size_type getInputCount(emb_model model)
{
	emb_size_type rtnVal = 0;

	return rtnVal;
}

emb_size_type getOutputCount(emb_model model)
{
	emb_size_type rtnVal = 0;

	return rtnVal;
}

int getStateType(emb_model model, int is_algebraic[])
{
	is_algebraic[0] = 0;

	return 0;
}

int setParameter(emb_model model, const double p[])
{
	return 0;
}

int getInitialInputs(emb_model model, double u0[])
{
	u0[0] = 0.0;

	return 0;
}

int getInitialState(emb_model model, const double u[], const double y[], double x0[])
{
	x0[0] = 0.0;

	return 0;
}

int getDerivatives(emb_model model, const double x[], const double u[], double deriv_resid[], double *objective)
{
	return 0;
}

int getParameterName(emb_model model, int name_idx, char par_name[], size_t maxlen, size_t *reqlen)
{
	return -1;
}

int getStateName(emb_model model, int name_idx, char x_name[], size_t maxlen, size_t *reqlen)
{
	return -1;
}

int getInputName(emb_model model, int name_idx, char u_name[], size_t maxlen, size_t *reqlen)
{
	return -1;
}

int getOutputName(emb_model model, int name_idx, char y_name[], size_t maxlen, size_t *reqlen)
{
	return -1;
}




