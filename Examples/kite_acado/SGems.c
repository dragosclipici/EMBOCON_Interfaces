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

/*
 *Include files
 */
#include <math.h>
#include "EMBOCON_ObserverInterface.h"
#include "EMBOCON_OptimizerInterface.h"

#include "acado.h"

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
void SGems(const x_bus *x,
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
double ucur[ACADO_NU];
const double ycur[] = { (*x).x, (*x).y,  (*x).z, (*x).dx, (*x).dy, (*x).dz, (*x).e11, (*x).e12, (*x).e13, (*x).e21, (*x).e22, (*x).e23, (*x).e31, (*x).e32, (*x).e33, (*x).w1, (*x).w2, (*x).w3, (*x).r, (*x).dr, (*x).delta, (*x).ddelta };
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

/*
 *Optimize which just means u = -Kx here. xcur is x_diff here because of reference
 */

/*Use this to set your own xref signal
int flag = setOptParameter(optim,opt_context, optp);*/

int flagOpt = makeOptStep(optim, opt_context, xcur, uprev, ucur);

/*
 * Pass inputs to the model
 */
(*u).dddelta = ucur[0];
(*u).ddr = ucur[1];
(*u).ur = ucur[2];
(*u).up = ucur[3];

}



