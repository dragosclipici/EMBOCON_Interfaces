 #include "EMBOCON_DataTypes.h"
 #include "EMBOCON_OptimizerInterface.h"
 #include "myMPC.h"

double xref[8];

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
	return 8;
}

emb_size_type getOptModelStateCount(emb_optimizer optim)
{
	return 8;
}

emb_size_type getOptModelInputCount(emb_optimizer optim)
{
	return 2;
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
	unsigned int i;
	unsigned int np = getOptModelParameterCount(optim);
	for (i = 0; i < np;  i++)
	{
		xref[i] = optp[i];
	}
	return 0;
}

int makeOptStep(emb_optimizer optim, emb_opt_context opt_context, const double xcur[], const double uprev[], double ucur[])
{
	emb_size_type nx = getOptModelStateCount(optim);
	emb_size_type nu = getOptModelInputCount(optim);

	myMPC_params params;
	myMPC_output output;
	myMPC_info info;

	myMPC_FLOAT Qdiag[] = {100, 100, 100, 100, 100, 100, 1, 1};
	myMPC_FLOAT Rdiag[] = {1, 1};
	myMPC_FLOAT Hdiag[] = {2*Qdiag[0], 2*Qdiag[1], 2*Qdiag[2], 2*Qdiag[3], 2*Qdiag[4], 
			2*Qdiag[5], 2*Qdiag[6], 2*Qdiag[7], 2*Rdiag[0], 2*Rdiag[1]};
	/* P Martix is solution of ricatti equation related to the LQR problem */
	myMPC_FLOAT P[8][8] = {	{ 3.1737e+04,  4.6682e-10,  1.8752e+03,  8.7111e-12, -9.3106e+03,  2.0915e+03,  1.0000e+03,  1.3242e-11},
		   		{ 4.6682e-10,  3.9632e+04, -6.2554e-12,  9.7031e+02, -1.8209e-10, -2.2602e-11,  2.4313e-11,  1.0000e+03},
		      		{ 1.8752e+03, -6.2554e-12,  1.6687e+04,  6.4560e-12, -2.7441e+03,  2.4835e+04, -5.9160e+02, -1.5141e-11},
		         	{ 8.7111e-12,  9.7031e+02,  6.4560e-12,  2.0401e+02, -8.4896e-12,  9.3342e-12,  5.2174e-13,  2.8654e+01},
			   	{-9.3106e+03, -1.8209e-10, -2.7441e+03, -8.4896e-12,  2.5723e+05, -6.6525e+02, -1.3680e+03,  1.2303e-10},
			      	{ 2.0915e+03, -2.2602e-11,  2.4835e+04,  9.3342e-12, -6.6525e+02,  3.7426e+04, -9.3086e+02, -2.1550e-11},
			        { 1.0000e+03,  2.4313e-11, -5.9160e+02,  5.2174e-13, -1.3680e+03, -9.3086e+02,  1.7879e+02,  9.2775e-13},
				{ 1.3242e-11,  1.0000e+03, -1.5141e-11,  2.8654e+01,  1.2303e-10, -2.1550e-11,  9.2775e-13,  1.3445e+02}	};


/*	double xref[] = {1, 1.5, 0, 0, 0, 0, 0, 0};*/

	unsigned int i; unsigned int j;
	/*SET H*/
	/*initialize to zero*/
	for (i = 0; i < (nx+nu)*(nx+nu); i++)
	{
		params.H[i] = 0;
	}
	for (i = 0; i < nx + nu; i++)
	{
		params.H[i*(nx+nu)+i] = Hdiag[i];
	}

	/*SET f*/
	for (i = 0; i < nx; i++)
	{
		/* ONLY HOLDS IF Q = DIAG!!!! correct: [-(xref'*Q + xref'*Q'), -(uref'*R + uref'*R')]'*/
		params.f[i] = -2.0*xref[i]*Qdiag[i]; 
	}
	for (i = nx; i < nx+nu; i++)
	{
		params.f[i] = 0;
	}

	/*set H_N*/
	for (i = 0; i < nx; i++)
	{
		for (j = 0; j < nx; j++)
		{
			params.H_N[j*nx+i] = 2*P[i][j];
		}
	}

	/*set f_N*/
	double sum;
	for (i = 0; i < nx; i++)
	{
		sum = 0;
		for (j = 0; j < nx; j++)
		{
			/*correct: -(xref'*Q + xref'*Q')'*/
			sum = sum - (xref[j]*P[j][i] + xref[j]*P[i][j]);
		}
		params.f_N[i] = sum;
	}

	/*SET z1*/
	for (i = 0; i < nx; i++ )
	{
    		params.z1[i] = xcur[i];
	}
	for (i = nx; i < 2*nx; i++)
	{
    		params.z1[i] = 0;
	}
   
	/*Solve MPC*/
	int flag = myMPC_solve( &params, &output, &info);

	if (flag != 1)
	{
		return 1;	
	}

	/*assign to input*/
 	ucur[0] = output.u1[0];
	ucur[1] = output.u1[1];
	
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
