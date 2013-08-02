
#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME SGems_SFunc

#include "acado.h"
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
/* %%%-SFUNWIZ_defines_Changes_BEGIN --- EDIT HERE TO _END */
#define NUM_INPUTS          2
/* Input Port  0 */
#define IN_PORT_0_NAME      xref
#define INPUT_0_WIDTH       1
#define INPUT_DIMS_0_COL    1
#define INPUT_0_DTYPE       real_T
#define INPUT_0_COMPLEX     COMPLEX_NO
#define IN_0_FRAME_BASED    FRAME_NO
#define IN_0_BUS_BASED      1
#define IN_0_BUS_NAME       xref_bus
#define IN_0_DIMS           1-D
#define INPUT_0_FEEDTHROUGH 1
#define IN_0_ISSIGNED        0
#define IN_0_WORDLENGTH      8
#define IN_0_FIXPOINTSCALING 1
#define IN_0_FRACTIONLENGTH  9
#define IN_0_BIAS            0
#define IN_0_SLOPE           0.125
/* Input Port  1 */
#define IN_PORT_1_NAME      x
#define INPUT_1_WIDTH       1
#define INPUT_DIMS_1_COL    1
#define INPUT_1_DTYPE       real_T
#define INPUT_1_COMPLEX     COMPLEX_NO
#define IN_1_FRAME_BASED    FRAME_NO
#define IN_1_BUS_BASED      1
#define IN_1_BUS_NAME       x_bus
#define IN_1_DIMS           1-D
#define INPUT_1_FEEDTHROUGH 1
#define IN_1_ISSIGNED        0
#define IN_1_WORDLENGTH      8
#define IN_1_FIXPOINTSCALING 1
#define IN_1_FRACTIONLENGTH  9
#define IN_1_BIAS            0
#define IN_1_SLOPE           0.125

#define NUM_OUTPUTS          1
/* Output Port  0 */
#define OUT_PORT_0_NAME      u
#define OUTPUT_0_WIDTH       1
#define OUTPUT_DIMS_0_COL    1
#define OUTPUT_0_DTYPE       real_T
#define OUTPUT_0_COMPLEX     COMPLEX_NO
#define OUT_0_FRAME_BASED    FRAME_NO
#define OUT_0_BUS_BASED      1
#define OUT_0_BUS_NAME       u_bus
#define OUT_0_DIMS           1-D
#define OUT_0_ISSIGNED        1
#define OUT_0_WORDLENGTH      8
#define OUT_0_FIXPOINTSCALING 1
#define OUT_0_FRACTIONLENGTH  3
#define OUT_0_BIAS            0
#define OUT_0_SLOPE           0.125

#define NPARAMS              0

#define SAMPLE_TIME_0        INHERITED_SAMPLE_TIME
#define NUM_DISC_STATES      0
#define DISC_STATES_IC       [0]
#define NUM_CONT_STATES      0
#define CONT_STATES_IC       [0]

#define SFUNWIZ_GENERATE_TLC 1
#define SOURCEFILES "__SFB__"
#define PANELINDEX           6
#define USE_SIMSTRUCT        0
#define SHOW_COMPILE_STEPS   0                   
#define CREATE_DEBUG_MEXFILE 0
#define SAVE_CODE_ONLY       0
#define SFUNWIZ_REVISION     3.0
/* %%%-SFUNWIZ_defines_Changes_END --- EDIT HERE TO _BEGIN */
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
#include "simstruc.h"
#include "SGems_SFunc_bus.h"
 
/*
* Code Generation Environment flag (simulation or standalone target).
 */
static int_T isSimulationTarget;
/*  Utility function prototypes. */
static int_T GetRTWEnvironmentMode(SimStruct *S);
/* Macro used to check if Simulation mode is set to accelerator */
#define isDWorkPresent !(ssRTWGenIsCodeGen(S) && !isSimulationTarget)

extern void SGems(const xref_bus *xref,
                          const x_bus *x,
                          u_bus *u);

/*====================*
 * S-function methods *
 *====================*/
/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{

    DECL_AND_INIT_DIMSINFO(inputDimsInfo);
    DECL_AND_INIT_DIMSINFO(outputDimsInfo);
    ssSetNumSFcnParams(S, NPARAMS);
     if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
	 return; /* Parameter mismatch will be reported by Simulink */
     }

    ssSetNumContStates(S, NUM_CONT_STATES);
    ssSetNumDiscStates(S, NUM_DISC_STATES);

    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;
    /*Input Port 0 */

  /* Register xref_bus datatype for Input port 0 */

    #if defined(MATLAB_MEX_FILE)
    if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY)
    {
      DTypeId dataTypeIdReg;
      ssRegisterTypeFromNamedObject(S, "xref_bus", &dataTypeIdReg);
      if(dataTypeIdReg == INVALID_DTYPE_ID) return;
      ssSetInputPortDataType(S,0, dataTypeIdReg);
    }
    #endif
    ssSetInputPortWidth(S, 0, INPUT_0_WIDTH);
    ssSetInputPortComplexSignal(S, 0, INPUT_0_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 0, INPUT_0_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/
    ssSetBusInputAsStruct(S, 0,IN_0_BUS_BASED);
    ssSetInputPortBusMode(S, 0, SL_BUS_MODE);    /*Input Port 1 */

  /* Register x_bus datatype for Input port 1 */

    #if defined(MATLAB_MEX_FILE)
    if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY)
    {
      DTypeId dataTypeIdReg;
      ssRegisterTypeFromNamedObject(S, "x_bus", &dataTypeIdReg);
      if(dataTypeIdReg == INVALID_DTYPE_ID) return;
      ssSetInputPortDataType(S,1, dataTypeIdReg);
    }
    #endif
    ssSetInputPortWidth(S, 1, INPUT_1_WIDTH);
    ssSetInputPortComplexSignal(S, 1, INPUT_1_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 1, INPUT_1_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/
    ssSetBusInputAsStruct(S, 1,IN_1_BUS_BASED);
    ssSetInputPortBusMode(S, 1, SL_BUS_MODE);
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;

  /* Register u_bus datatype for Output port 0 */

  #if defined(MATLAB_MEX_FILE)
    if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY)
    {
      DTypeId dataTypeIdReg;
      ssRegisterTypeFromNamedObject(S, "u_bus", &dataTypeIdReg);
      if(dataTypeIdReg == INVALID_DTYPE_ID) return;
        ssSetOutputPortDataType(S,0, dataTypeIdReg);
    }
    #endif

    ssSetBusOutputObjectName(S, 0, (void *) "u_bus");
    ssSetOutputPortWidth(S, 0, OUTPUT_0_WIDTH);
    ssSetOutputPortComplexSignal(S, 0, OUTPUT_0_COMPLEX);
    ssSetBusOutputAsStruct(S, 0,OUT_0_BUS_BASED);
    ssSetOutputPortBusMode(S, 0, SL_BUS_MODE);
    if (ssRTWGenIsCodeGen(S)) {
       isSimulationTarget = GetRTWEnvironmentMode(S);
    if (isSimulationTarget==-1) {
       ssSetErrorStatus(S, " Unable to determine a valid code generation environment mode");
       return;
     }
       isSimulationTarget |= ssRTWGenIsModelReferenceSimTarget(S);
    }
  
    /* Set the number of dworks */
    if (!isDWorkPresent) {
      if (!ssSetNumDWork(S, 0)) return;
    } else {
      if (!ssSetNumDWork(S, 3)) return;
    }


   if (isDWorkPresent) {
   
    /*
     * Configure the dwork 0 (u0."BUS")
     */
#if defined(MATLAB_MEX_FILE)

    if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {
      DTypeId dataTypeIdReg;
      ssRegisterTypeFromNamedObject(S, "xref_bus", &dataTypeIdReg);
      if (dataTypeIdReg == INVALID_DTYPE_ID) return;
      ssSetDWorkDataType(S, 0, dataTypeIdReg);
    }

#endif

    ssSetDWorkUsageType(S, 0, SS_DWORK_USED_AS_DWORK);
    ssSetDWorkName(S, 0, "u0BUS");
    ssSetDWorkWidth(S, 0, DYNAMICALLY_SIZED);
    ssSetDWorkComplexSignal(S, 0, COMPLEX_NO);

    /*
     * Configure the dwork 1 (u1."BUS")
     */
#if defined(MATLAB_MEX_FILE)

    if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {
      DTypeId dataTypeIdReg;
      ssRegisterTypeFromNamedObject(S, "x_bus", &dataTypeIdReg);
      if (dataTypeIdReg == INVALID_DTYPE_ID) return;
      ssSetDWorkDataType(S, 1, dataTypeIdReg);
    }

#endif

    ssSetDWorkUsageType(S, 1, SS_DWORK_USED_AS_DWORK);
    ssSetDWorkName(S, 1, "u1BUS");
    ssSetDWorkWidth(S, 1, DYNAMICALLY_SIZED);
    ssSetDWorkComplexSignal(S, 1, COMPLEX_NO);

    /*
     * Configure the dwork 2 (y0BUS)
     */
#if defined(MATLAB_MEX_FILE)

    if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {
      DTypeId dataTypeIdReg;
      ssRegisterTypeFromNamedObject(S, "u_bus", &dataTypeIdReg);
      if (dataTypeIdReg == INVALID_DTYPE_ID) return;
      ssSetDWorkDataType(S, 2, dataTypeIdReg);
    }

#endif

    ssSetDWorkUsageType(S, 2, SS_DWORK_USED_AS_DWORK);
    ssSetDWorkName(S, 2, "y0BUS");
    ssSetDWorkWidth(S, 2, DYNAMICALLY_SIZED);
    ssSetDWorkComplexSignal(S, 2, COMPLEX_NO);
}
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
                     SS_OPTION_USE_TLC_WITH_ACCELERATOR | 
		     SS_OPTION_WORKS_WITH_CODE_REUSE));
}

# define MDL_SET_INPUT_PORT_FRAME_DATA
static void mdlSetInputPortFrameData(SimStruct  *S, 
                                     int_T      port,
                                     Frame_T    frameData)
{
    ssSetInputPortFrameData(S, port, frameData);
}
/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy  the sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLE_TIME_0);
    ssSetOffsetTime(S, 0, 0.0);
}


#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START)
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
static void mdlStart(SimStruct *S)
{
    int i;
    /* Bus Information */
    slDataTypeAccess *dta = ssGetDataTypeAccess(S);
    const char *bpath = ssGetPath(S);
	DTypeId u_busId = ssGetDataTypeId(S, "u_bus");
	DTypeId x_busId = ssGetDataTypeId(S, "x_bus");
	DTypeId xref_busId = ssGetDataTypeId(S, "xref_bus");

	int_T *busInfo = (int_T *)malloc(36*sizeof(int_T));
	if(busInfo==NULL) {
        ssSetErrorStatus(S, "Memory allocation failure");
        return;
    }

      /* Calculate offsets of all primitive elements of the bus */
    for( i = 0; i < ACADO_NX; i++ ) {
        busInfo[2*i] = dtaGetDataTypeElementOffset(dta, bpath,xref_busId,i);
        busInfo[2*i+1] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
    }
    for( i = 0; i < ACADO_NX; i++ ) {
        busInfo[2*(ACADO_NX+i)] = dtaGetDataTypeElementOffset(dta, bpath,x_busId,i);
        busInfo[2*(ACADO_NX+i)+1] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
    }
    for( i = 0; i < ACADO_NU; i++ ) {
        busInfo[2*(2*ACADO_NX+i)] = dtaGetDataTypeElementOffset(dta, bpath,u_busId,i);
        busInfo[2*(2*ACADO_NX+i)+1] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
    }
	ssSetUserData(S, busInfo);
}
#endif /*  MDL_START */

#define MDL_SET_INPUT_PORT_DATA_TYPE
static void mdlSetInputPortDataType(SimStruct *S, int port, DTypeId dType)
{
    ssSetInputPortDataType( S, 0, dType);
}
#define MDL_SET_OUTPUT_PORT_DATA_TYPE
static void mdlSetOutputPortDataType(SimStruct *S, int port, DTypeId dType)
{
    ssSetOutputPortDataType(S, 0, dType);
}

#define MDL_SET_DEFAULT_PORT_DATA_TYPES
static void mdlSetDefaultPortDataTypes(SimStruct *S)
{
  ssSetInputPortDataType( S, 0, SS_DOUBLE);
 ssSetOutputPortDataType(S, 0, SS_DOUBLE);
}

#define MDL_SET_WORK_WIDTHS
#if defined(MDL_SET_WORK_WIDTHS) && defined(MATLAB_MEX_FILE)

static void mdlSetWorkWidths(SimStruct *S)
{
  /* Set the width of DWork(s) used for marshalling the IOs */
  if (isDWorkPresent) {

     /* Update dwork 0 */
     ssSetDWorkWidth(S, 0, ssGetInputPortWidth(S, 0));
       
     /* Update dwork 1 */
     ssSetDWorkWidth(S, 1, ssGetInputPortWidth(S, 1));
       
     /* Update dwork 2 */
     ssSetDWorkWidth(S, 2, ssGetOutputPortWidth(S, 0));
        
    }
}

#endif
/* Function: mdlOutputs =======================================================
 *
*/
static void mdlOutputs(SimStruct *S, int_T tid)
{
    const char *xref = (char *) ssGetInputPortSignal(S,0);
    const char *x = (char *) ssGetInputPortSignal(S,1);
    char *u = (char *) ssGetOutputPortSignal(S,0);

	int_T* busInfo = (int_T *) ssGetUserData(S);

	/* Temporary bus copy declarations */
	xref_bus _u0BUS;
	x_bus _u1BUS;
	u_bus _y0BUS;

	/*Copy input bus into temporary structure*/
	(void) memcpy(&_u0BUS.x_C,xref + busInfo[0], busInfo[1]);
	(void) memcpy(&_u0BUS.x_L,xref + busInfo[2], busInfo[3]);
	(void) memcpy(&_u0BUS.v_C,xref + busInfo[4], busInfo[5]);
	(void) memcpy(&_u0BUS.v_L,xref + busInfo[6], busInfo[7]);
	(void) memcpy(&_u0BUS.theta,xref + busInfo[8], busInfo[9]);
	(void) memcpy(&_u0BUS.omega,xref + busInfo[10], busInfo[11]);
	(void) memcpy(&_u0BUS.u_C,xref + busInfo[12], busInfo[13]);
	(void) memcpy(&_u0BUS.u_L,xref + busInfo[14], busInfo[15]);
    
	(void) memcpy(&_u1BUS.x_C,x + busInfo[16], busInfo[17]);
	(void) memcpy(&_u1BUS.x_L,x + busInfo[18], busInfo[19]);
	(void) memcpy(&_u1BUS.v_C,x + busInfo[20], busInfo[21]);
	(void) memcpy(&_u1BUS.v_L,x + busInfo[22], busInfo[23]);
	(void) memcpy(&_u1BUS.theta,x + busInfo[24], busInfo[25]);
	(void) memcpy(&_u1BUS.omega,x + busInfo[26], busInfo[27]);
	(void) memcpy(&_u1BUS.u_C,x + busInfo[28], busInfo[29]);
	(void) memcpy(&_u1BUS.u_L,x + busInfo[30], busInfo[31]);
	
	SGems(&_u0BUS,&_u1BUS, &_y0BUS);

	/*Copy temporary structure into output bus*/
	(void) memcpy(u+ busInfo[32], &_y0BUS.u_CR, busInfo[33]);
	(void) memcpy(u+ busInfo[34], &_y0BUS.u_LR, busInfo[35]);
}



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    /*Free stored bus information*/
    int_T *busInfo = (int_T *) ssGetUserData(S);
    if(busInfo!=NULL) {
      free(busInfo);
    }
}


static int_T GetRTWEnvironmentMode(SimStruct *S)
{
    int_T status;
    mxArray *plhs[1];
    mxArray *prhs[1];
    int_T err;
    
    /*
      * Get the name of the Simulink block diagram
    */
    prhs[0] = mxCreateString(ssGetModelName(ssGetRootSS(S)));
    plhs[0] = NULL;
    
    /*
      * Call "isSimulationTarget = rtwenvironmentmode(modelName)" in MATLAB
    */
    mexSetTrapFlag(1);
    err = mexCallMATLAB(1, plhs, 1, prhs, "rtwenvironmentmode");
    mexSetTrapFlag(0);
    mxDestroyArray(prhs[0]);
    
    /*
     * Set the error status if an error occurred
    */
    if (err) {
        if (plhs[0]) {
            mxDestroyArray(plhs[0]);
            plhs[0] = NULL;
        }
        ssSetErrorStatus(S, "Unknow error during call to 'rtwenvironmentmode'.");
        return -1;
    }
    
    /*
      * Get the value returned by rtwenvironmentmode(modelName)
    */
   if (plhs[0]) {
       status = (int_T) (mxGetScalar(plhs[0]) != 0);
       mxDestroyArray(plhs[0]);
       plhs[0] = NULL;
   }
    
    return (status);
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif


