/*
 ******************************************************************************
 ***** EMBOCON MODEL INTERFACE ************************************************
 ******************************************************************************
 * EMBOCON_ModelInterface.h
 *
 *  Created on: 08.11.2012
 *      Author: schoppmeyerc
 *      E-Mail: christian.schoppmeyer@bci.tu-dortmund.de
 *      Version: 1.1
 ******************************************************************************
 * The Model Interface of the software platform defines common functions
 * for a system model and optimization problem in C code. Usually, the C
 * code is generated by the Modelica-Optimica-based online part of the
 * platform, but any other set of functions that implement the proper
 * semantics can be used as well. The Model Interface connects the C code
 * representation with an optimizer and an observer and offers both to
 * receive information of the model via calling the defined functions. The
 * Model Interface can be used to connect the same C code representation
 * to both the optimizer and the observer or two different C code
 * representations, one to the optimizer and one to the observer. The Model
 * Interface is not used for the connection between a system model and an
 * optimizer generated by the direct path by-pass; here the information
 * about the system and the optimization problem at hand is incorporated
 * in the optimizer leading to a better performance and resulting in a
 * combined callable C code representation.
 ******************************************************************************
 */

#ifndef EMBOCON_MODELINTERFACE_H_
#define EMBOCON_MODELINTERFACE_H_

#include "EMBOCON_DataTypes.h"
#include "stdlib.h"

/*
 * The emb_model handle is used in all subsequent calls to identify
 * an working instance of the model.
 */
typedef emb_handle emb_model;

/*
 * A model is created by the createModel() call. This call returns a
 * handle of type emb_model that will be used in subsequent calls to
 * identify this instance of the model. All other calls defined in the
 * ModelInterface use this handle to identify the model concerned. This
 * makes it possible to use the same code but different parameters for
 * the model in the optimizer and the observer.
 */
emb_model createModel();

/*
 * If a model is no longer needed, the model should be freed by calling
 * freeModel(). This call will free any resources associated with the model
 * instance. After the call, the model handle can no longer be used.
 */
void freeModel(emb_model model);

/*
 * The getParameterCount() call returns the number of parameters of the model.
 * These numbers will remain the same throughout the lifetime of the model instance.
 * The return value of this function will be denoted np in the descriptions below.
 * The return value is undefined if an invalid model handle is passed for the model argument.
 */
emb_size_type getParameterCount(emb_model model);

/*
 *  The getStateCount() call returns the number of states of the model. The state count includes
 *  the differential states as well as the algebraic states. This number will remain the same
 *  throughout the lifetime of the model instance. The return value of this function will be denoted
 *  nx in the descriptions below. The return value is undefined if an invalid model handle is passed
 *  for the model argument.
 */
emb_size_type getStateCount(emb_model model);

/*
 *  The getInputCount() call returns the number of inputs of the model. This number will remain the
 *  same throughout the lifetime of the model instance. The return value of this function will be
 *  denoted nu in the descriptions below. The return value is undefined if an invalid model handle
 *  is passed for the model argument.
 */
emb_size_type getInputCount(emb_model model);

/*
 *  The getOutputCount() call returns the number of outputs of the model. This number will remain the
 *  same throughout the lifetime of the model instance. The return value of this function will be
 *  denoted ny in the descriptions below. The return value is undefined if an invalid model handle
 *  is passed for the model argument.
 */
emb_size_type getOutputCount(emb_model model);

/**
 * The getStateType() function can be used to distinguish algebraic states from differential states.
 * The caller should provide an array of int containing at least nx elements in the argument is algebraic.
 * The function will write 0 to the ith element of this array if the ith state element is not an algebraic
 * state and 1 if it is an algebraic state.
 */
int getStateType(emb_model model, int is_algebraic[]);

/**
 * The setParameter() function of the model interface updates the model data with new values for all
 * parameters. The input parameter p contains the values of the parameter in a vector of the type double.
 * The function returns 0 upon success. If the function fails, an implementation-defined, non-zero error
 * code is returned.
 */
int setParameter(emb_model model, const double p[]);

/**
 * The getInitialInputs() function of the model interface initializes the input vector of the model.
 * The actual initial input vector is written to the first nu elements of the output parameter u0.
 * The caller of this function must make sure that u0 is an array that can hold sufficiently many
 * values. The function returns 0 upon success. If the function fails, an implementation-defined,
 * non-zero error code is returned.
 *
 * It is up to the implementation whether or not the current values of the outputs are used to compute
 * the initial controls.
 *
 * It is up to the model implementation to decide what the ordering of the input elements is. The ordering
 * will be the same for all calls to the model interface and for all instances created by createModel().
 */
int getInitialInputs(emb_model model, double u0[]);

/**
 * The getInitialState() function of the model interface initializes the state vector of the model. The
 * input parameter u contains the initial control signals in an array of type double. This array must
 * contain nu elements. The current values of the output measurements are given in the input parameter
 * y. This array must contain ny elements. The actual initial state vector is written to the first nx
 * elements of the output parameter x0. The caller of this function must make sure that x0 is an array
 * that can hold sufficiently many values. The function returns 0 upon success. If the function fails, an
 * implementation-defined, non-zero error code is returned.
 *
 * It is up to the implementation whether or not the current values of the controls and/or the outputs
 * are used to compute the initial state. An implementation may try to compute a state vector that "fits"
 * these values in some sense, or it may return an initial state that is independent of these inputs.
 * In the latter case, it is entirely up to the observer to account for any mismatches between this state
 * and the actual process conditions.
 *
 * Note that when we refer to the `state' of a model, we mean a vector containing the differential and
 * algebraic states in the model equations. The state vector does not contain all `memory' that the model
 * has. For example, the parameters that are set by setParameter() and that influence the model equations
 * are stored in the model, but are not part of the model state. Also, the model state is not stored
 * inside the model, but is passed to the model interface functions when needed. This differs from what
 * is usually considered the state of an object in object-oriented programming.
 *
 * It is up to the model implementation to decide what the ordering of the state elements is. The ordering
 * will be the same for all calls to the model interface and for all instances created by createModel().
 * There is no requirement on the implementation to observe a specific ordering or to keep algebraic
 * states together.
 */
int getInitialState(emb_model model, const double u[], const double y[], double x0[]);

/**
 * The getDerivatives() function of the Model Interface triggers a calculation step of the model in which
 * new derivatives, constraint residuals and a new objective value are calculated based on the current
 * state and control inputs. The input parameter x contains the differential states and the algebraic
 * states for the model. The input parameter u contains the last control actions as an array of type
 * double. The output parameter derivs_resids contains the calculated derivatives for differential states
 * and the residuals for algebraic states. The objective value of the optimization problem stated in the
 * model is written to the output argument objective. This argument should point to a single double
 * value. The function returns 0 upon success. If the function fails, an implementation-defined, non-zero
 * error code is returned.
 */
int getDerivatives(emb_model model, const double x[], const double u[], double deriv_resid[], double *objective);

/**
 * An implementation is not required to provide meaningful and/or unique names for the parameters.
 * A function that returns only empty strings would be a trivial minimal implementation. Logging
 * and tracing functionality in the supervisor will likely benefit from getting meaningful parameter
 * names. It is up to the caller of the function to provide a buffer to receive the requested name.
 * The name is returned for a single parameter at a time. Which parameter's name is returned is
 * determined by the input argument name idx. This is a value between 0 and np (inclusive). The size
 * of the buffer is passed into the input argument maxlen. If maxlen is zero, the par name argument
 * is not touched. Otherwise, a string is written to par name of no more than maxlen characters,
 * including the terminating null character. If the actual parameter name is longer than maxlen
 * characters, the string is truncated. The truncated string will still be terminated by a null
 * character. The number of characters that are required to represent the complete name, including
 * the terminating null, is written to the output parameter reqlen. This can be used to detect whether
 * the parameter name was truncated. The function returns 0 upon success. If the function fails, an
 * implementation-defined, non-zero error code is returned. It is not considered a failure if the
 * parameter name was truncated.
 */
int getParameterName(emb_model model, int name_idx, char par_name[], size_t maxlen, size_t *reqlen);

/**
 * An implementation is not required to provide meaningful and/or unique names for the states.
 * A function that returns only empty strings would be a trivial minimal implementation. Logging
 * and tracing functionality in the supervisor will likely benefit from getting meaningful state
 * names. It is up to the caller of the function to provide a buffer to receive the requested name.
 * The name is returned for a single state at a time. Which state's name is returned is
 * determined by the input argument name idx. This is a value between 0 and nx (inclusive). The size
 * of the buffer is passed into the input argument maxlen. If maxlen is zero, the par name argument
 * is not touched. Otherwise, a string is written to par name of no more than maxlen characters,
 * including the terminating null character. If the actual state name is longer than maxlen
 * characters, the string is truncated. The truncated string will still be terminated by a null
 * character. The number of characters that are required to represent the complete name, including
 * the terminating null, is written to the output state reqlen. This can be used to detect whether
 * the state name was truncated. The function returns 0 upon success. If the function fails, an
 * implementation-defined, non-zero error code is returned. It is not considered a failure if the
 * state name was truncated.
 */
int getStateName(emb_model model, int name_idx, char x_name[], size_t maxlen, size_t *reqlen);

/**
 * An implementation is not required to provide meaningful and/or unique names for the inputs.
 * A function that returns only empty strings would be a trivial minimal implementation. Logging
 * and tracing functionality in the supervisor will likely benefit from getting meaningful input
 * names. It is up to the caller of the function to provide a buffer to receive the requested name.
 * The name is returned for a single input at a time. Which input's name is returned is
 * determined by the input argument name idx. This is a value between 0 and nu (inclusive). The size
 * of the buffer is passed into the input argument maxlen. If maxlen is zero, the par name argument
 * is not touched. Otherwise, a string is written to par name of no more than maxlen characters,
 * including the terminating null character. If the actual input name is longer than maxlen
 * characters, the string is truncated. The truncated string will still be terminated by a null
 * character. The number of characters that are required to represent the complete name, including
 * the terminating null, is written to the input input reqlen. This can be used to detect whether
 * the input name was truncated. The function returns 0 upon success. If the function fails, an
 * implementation-defined, non-zero error code is returned. It is not considered a failure if the
 * input name was truncated.
 */
int getInputName(emb_model model, int name_idx, char u_name[], size_t maxlen, size_t *reqlen);

/**
 * An implementation is not required to provide meaningful and/or unique names for the outputs.
 * A function that returns only empty strings would be a trivial minimal implementation. Logging
 * and tracing functionality in the supervisor will likely benefit from getting meaningful output
 * names. It is up to the caller of the function to provide a buffer to receive the requested name.
 * The name is returned for a single output at a time. Which output's name is returned is
 * determined by the output argument name idx. This is a value between 0 and ny (inclusive). The size
 * of the buffer is passed into the output argument maxlen. If maxlen is zero, the par name argument
 * is not touched. Otherwise, a string is written to par name of no more than maxlen characters,
 * including the terminating null character. If the actual output name is longer than maxlen
 * characters, the string is truncated. The truncated string will still be terminated by a null
 * character. The number of characters that are required to represent the complete name, including
 * the terminating null, is written to the output output reqlen. This can be used to detect whether
 * the output name was truncated. The function returns 0 upon success. If the function fails, an
 * implementation-defined, non-zero error code is returned. It is not considered a failure if the
 * output name was truncated.
 */
int getOutputName(emb_model model, int name_idx, char y_name[], size_t maxlen, size_t *reqlen);


#endif /* EMBOCON_MODELINTERFACE_H_ */
