 #include "EMBOCON_DataTypes.h"
 #include "EMBOCON_OptimizerInterface.h"


/* Include socket libraries for different platforms */
#ifdef _WIN32
	#include<winsock.h>
	#include<time.h>
	#pragma comment(lib, "wsock32.lib")
#else
	#include<sys/socket.h>
	#include<arpa/inet.h>
	#include<netinet/in.h>
	#include<netdb.h>
	#include<sys/select.h>
	#include<sys/time.h>
    #include<unistd.h>
#endif

#include "FPGAcontroller_library.h"


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



int makeOptStep(emb_optimizer optim, emb_opt_context opt_context, const double xcur[], const double uprev[], double ucur[])
{
	int flag;
	unsigned int i;
	float 	input[NINPUTS]; //NINPUTS=2*nx
    float   output[NOUTPUTS]; //NOUTPUTS=nu
	double  *eTime;

	emb_size_type nx = getOptModelStateCount(optim);
	emb_size_type nu = getOptModelInputCount(optim);
	
	// Ethernet handle variables
	int 	socket_handle;
	struct 	sockaddr_in server_address;
	int 	saddr_len = sizeof(server_address);
	
	
	//Ethernet port initialization
	int port_number=2008;           	 // Ethernet port number
    char *hostname = "192.168.1.10";     // Ethernet IP

	//Open Ethernet socket
    flag = UDPclient_setup(hostname, port_number, &socket_handle, &server_address, &saddr_len);
    if (flag != 0) {
		UDPclient_error(flag);
		return 1;
	}

	//fill in the input vector to send to FPGA with x_hat and x_ref
	for ( i = 0; i < nx; i++) //x_hat
	{
		input[i] = (float)xcur[i];
	}
    for ( i = nx; i < 2*nx; i++) //x_ref
	{
		input[i] = (float)xref[i];
	}

	//Solve MPC on FPGA
    flag = UDPclient_call(&socket_handle, &server_address, &saddr_len, input, output, NINPUTS, NOUTPUTS, 4);
    if (flag != 0){
		UDPclient_error(flag);
        return 1;
    }
	
	//read back the control action computed by the FPGA
	for ( i = 0; i < NOUTPUTS; i++)
	{
		ucur[i] = (double)output[i];
	}

	
	//Free Ethernet socket
	UDPclient_close(&socket_handle);
	
	return 0;
	
}

