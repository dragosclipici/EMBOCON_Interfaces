#include "EMBOCON_DataTypes.h"
#include "EMBOCON_ObserverInterface.h"

#include "acado.h"

emb_observer createObserver(emb_model model)
{
	return model;
}

void freeObserver(emb_observer obs)
{
}

emb_obs_context initObsContext(emb_observer obs)
{
	return obs;
}

emb_obs_context cloneObsContext(emb_observer obs, emb_obs_context sourceContext)
{
	return obs;
}

void freeObsContext(emb_observer obs, emb_obs_context obsContext)
{
}

emb_size_type getObsModelParameterCount(emb_observer obs)
{
	return 0;
}

emb_size_type getObsModelStateCount(emb_observer obs)
{
	return ACADO_NX;
}

emb_size_type getObsModelInputCount(emb_observer obs)
{
	return ACADO_NU;
}

emb_size_type getObsModelOutputCount(emb_observer obs)
{
	return ACADO_NX;
}

int getObsModelStateType(emb_observer obs, int is_algebraic[])
{
	is_algebraic = 0;
	return 0;
}


 int setObsModelParameter(emb_observer obs, emb_obs_context obs_context, const double p[], const int p_flag[])
{
	return 1; /*always fail, there are no parameters to set */
}


int setObsModelParameterUncertainty(emb_observer obs, const int uncType[], const double lower[], const double        upper[], const double cov[])
{
	return 1; /*always fail, no uncertainty is taken into account*/
}

emb_size_type getObsParameterCount(emb_observer obs)
{
	return 0; /*no parameters in observer*/
}	

int setObsParameter(emb_observer obs, emb_obs_context obs_context, const double obsp[])
{
	return 1; /*there are no parameters to be set*/
}

/*set xcur = ycur*/
int makeObsStep(emb_observer obs, emb_obs_context obs_context, const double ucur[], const double ycur[], const double xprev[], double xcur[])
{
	emb_size_type nx =  getObsModelStateCount(obs);
	unsigned int i;
	for ( i = 0; i < nx; i++)
	{
		xcur[i] = ycur[i];
	}
	return 0;
}

 int getObsStepDetails(emb_observer obs, emb_obs_context obs_context, emb_size_type pastHor, emb_size_type *actHor, double uPast[], double yPast[], double xPast[])
{
	return 1; /*no extra details available*/
}

int getObsModelParameterName(emb_observer obs, int name_idx, char par_name[], size_t maxlen, size_t *reqlen)
{
	return 1; /* no model parameters, so no names */
}


int getObsModelStateName(emb_observer obs, int name_idx, char x_name[], size_t maxlen, size_t *reqlen)
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


int getObsModelInputName(emb_observer obs, int name_idx, char u_name[], size_t maxlen, size_t *reqlen)
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

int getObsModelOutputName(emb_observer obs, int name_idx, char y_name[], size_t maxlen, size_t *reqlen)
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
