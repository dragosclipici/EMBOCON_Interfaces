/*
 ******************************************************************************
 ***** EMBOCON OBSERVER INTERFACE *********************************************
 ******************************************************************************
 * EMBOCON_OptimizerObserver.c
 *
 *  Created on: 10.10.2011
 *      Author: schoppmeyerc
 *      E-Mail: christian.schoppmeyer@bci.tu-dortmund.de
 *      Version: 1.0
 ******************************************************************************
 * The Observer Interface of the EMBOCON software platform connects the
 * supervisor with a selected observer package of an optimization module. Each
 * observer of an optimization package integrated in the EMBOCON platform has
 * to implement this interface to enable the connection to the supervisor.
 ******************************************************************************
 */

#include "EMBOCON_ObserverInterface.h"

emb_model myModel;

emb_observer createObserver(emb_model model)
{
	myModel = model;

	return 0;
}

void freeObserver(emb_observer obs)
{
	freeModel(myModel);

	return;
}

emb_obs_context initObsContext(emb_observer obs)
{
	return 0;
}

emb_obs_context cloneObsContext(emb_observer obs, emb_obs_context sourceContext)
{
	return 0;
}

void freeObsContext(emb_observer obs, emb_obs_context obsContext)
{
	return;
}

emb_size_type getObsModelParameterCount(emb_observer obs)
{
	return getParameterCount(myModel);
}

emb_size_type getObsModelStateCount(emb_observer obs)
{
	return getStateCount(myModel);
}

emb_size_type getObsModelInputCount(emb_observer obs)
{
	return getInputCount(myModel);
}

emb_size_type getObsModelOutputCount(emb_observer obs)
{
	return getOutputCount(myModel);
}

int getObsModelStateType(emb_observer obs, int is_algebraic[])
{
	return getStateType(myModel, is_algebraic);
}

int setObsModelParameter(emb_observer obs, emb_obs_context obs_context, const double p[], const int p_flag[])
{
	return setParameter(myModel, p);
}

int setObsModelParameterUncertainty(emb_observer obs, const int uncType[], const double lower[], const double upper[], const double cov[])
{
	return 0;
}

emb_size_type getObsParameterCount(emb_observer obs)
{
	return 0;
}

int setObsParameter(emb_observer obs, emb_obs_context obs_context, const double obsp[])
{
	return 0;
}

int makeObsStep(emb_observer obs, emb_obs_context obs_context, const double ucur[], const double ycur[], const double xprev[], double xcur[])
{
	xcur[0] = ycur[0];

	return 0;
}

int getObsStepDetails(emb_observer obs, emb_obs_context obs_context, emb_size_type pastHor, emb_size_type *actHor, double uPast[], double yPast[], double xPast[])
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

int getObsParameterName(emb_observer obs, int name_idx, char par_name[], size_t maxlen, size_t *reqlen)
{
	return -1;
}



