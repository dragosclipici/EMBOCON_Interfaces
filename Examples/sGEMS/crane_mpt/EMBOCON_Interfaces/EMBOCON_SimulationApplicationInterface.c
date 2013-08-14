/*
 ******************************************************************************
 ***** EMBOCON SIMULATION APPLICATION INTERFACE *******************************
 ******************************************************************************
 * EMBOCON_SimulationApplicationInterface.c
 *
 *  Created on: 10.10.2011
 *      Author: schoppmeyerc
 *      E-Mail: christian.schoppmeyer@bci.tu-dortmund.de
 *      Version: 1.0
 ******************************************************************************
 * The SimulationApplication Interface of the EMBOCON software platform
 * connects the supervisor with an application, or simulation environment. The
 * interface is used to pass measurements and parameters of the current state
 * of the application or simulation to the observer to calculate the state of
 * the system. New control inputs calculated by the optimizer are passed back
 * through the interface to initiate the next steps of the controlled system.
 * The connection to an application can be highly specific in terms of
 * implementation language and information passing. Therefore the
 * SimulationApplication Interface specification consists of two layers, the
 * top layer defines the information which should be exchanged via the interface
 * by specifying function calls, the bottom layer contains the actual
 * implementation of the function calls which could involve an application
 * specific programming language as well as additional data transformation and
 * wrapper functions. Due to the demand for customized implementations for
 * particular applications, the bottom level of the SimulationApplication
 * Interface will not be specified in more details here.
 *
 * The SimulationApplication Interface offers additional functions for the
 * connection of simulation environments to the software platform to get values
 * for the real states and parameters calculated in the simulation. This additional
 * information enables a better performance evaluation of both, the observer and
 * the optimizer. The model of the system connected to the simulation environment
 * can be used to measure plant-model-mismatch or for evaluation the optimizer
 * and observer performance on slightly changed models.
 ******************************************************************************
 */

#include "EMBOCON_SimulationApplicationInterface.h"


