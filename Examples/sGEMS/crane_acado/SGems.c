/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif
#include "SGems_SFunc_bus.h"
 
#include "acado.h"

/*
 *Include files
 */
#include <math.h>
#include "EMBOCON_ObserverInterface.h"
#include "EMBOCON_OptimizerInterface.h"


/*
 *Defines
 */
#define u_width 1
#define y_width 1

/*
 * Create external references here.  
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output functions
 *
 */
void SGems(const xref_bus *xref,
                          const x_bus *x,
                          u_bus *u)
{

/*
 *#####OBSERVER##### 
 */
    
 /*
  *Initialize aguments observer
  */
void *obs;
void *obs_context;
double ucur[ACADO_NU];;
const double ycur[] = { (*x).x_C, (*x).x_L,  (*x).v_C, (*x).v_L, (*x).theta, (*x).omega, (*x).u_C, (*x).u_L };
const double xprev[ACADO_NX];
double xcur[ACADO_NX];

/*
 *Observe the state xcur = xprev (in this observer). xprev is state obtained from simulation
 */
int flagObs = makeObsStep( obs, obs_context, ucur, ycur, xprev, xcur);
                    
 
/*
 *#####OPTIMIZER#####
 */

 /*
  *Initialize aguments optimizer
  */
void *optim;
void *opt_context;
const double uprev[ACADO_NU];
const double optp[ACADO_NX] = { (*xref).x_C, (*xref).x_L,  (*xref).v_C, (*xref).v_L, (*xref).theta, (*xref).omega, (*xref).u_C, (*xref).u_L };       

/*
 *Optimize which just means u = -Kx here. xcur is x_diff here because of reference
 */

/*Use this to set your own xref signal*/
int flag = setOptParameter(optim,opt_context, optp);

int flagOpt = makeOptStep(optim, opt_context, xcur, uprev, ucur);

/*
 * Pass inputs to the model
 */
(*u).u_CR = ucur[0];
(*u).u_LR = ucur[1];

}



