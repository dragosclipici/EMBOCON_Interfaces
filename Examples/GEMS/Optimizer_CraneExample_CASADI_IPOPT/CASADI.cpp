/*
 * CASADI.cpp
 *
 *  Created on: Feb 27, 2013
 *      Author: SchoppmeyerC
 */
#include "CASADI.h"
#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <symbolic/casadi.hpp>
#include <interfaces/ipopt/ipopt_solver.hpp>
#include <symbolic/stl_vector_tools.hpp>
#include <string>
#include <stdlib.h>

using namespace CasADi;
using namespace std;

void compile(const std::string& name);
void readTxtFile(char name[], double values[], int number);

/**
 * Local variable for the solver
 */
IpoptSolver solver[1];
double vars_lb[3411];
double vars_ub[3411];
double vars_init[3411];
double lbg[3200]; //zero!
double ubg[3200]; //zero!
ExternalFunction e_fct[5];
int iteration[1];

void compile(const std::string& name)
{
  // Compilation command
  string compile_command = "gcc -fPIC -shared -O3 " + name + ".c -o " + name + ".so";

  // Compile the c-code
  int flag = system(compile_command.c_str());
  casadi_assert_message(flag==0, "Compilation failed");
}


void readTxtFile(char name[], double values[], int number)
{
	//values = new double[number];
	int i = 0;

	ifstream infile;
	infile.open(name);
    while(!infile.eof()) // To get all lines.
    {
    	string line;

        getline(infile, line); // Saves the line in STRING.

        //cout << "LINE: " << line << "\r\n";

        if(line == "-inf")
        {
        	values[i] = -numeric_limits<double>::infinity();
			//values[i] = -10E18;
			//cout << values[i] << "\r\n";
		}
		else if(line == "inf")
		{
			//values[i] = 10E18;
			values[i] = numeric_limits<double>::infinity();
			//cout << values[i] << "\r\n";
		}
		else
		{
			double new_val = atof(line.c_str());
			values[i] = new_val;
			//cout << new_val << "\r\n";
		}

        i++;
    }
    infile.close();
}

void initialize()
{
	iteration[0] = 0;

	/**compile("./exclude/gfcn");
	compile("./exclude/grad_jfcn");
	compile("./exclude/jac_gfcn");
	compile("./exclude/jfcn");
	compile("./exclude/hess_lfcn");*/

	// Load the generated functions into CasADi
	/**ExternalFunction jfcn_e("./exclude/jfcn.so");
	ExternalFunction grad_jfcn_e("./exclude/grad_jfcn.so");
	ExternalFunction gfcn_e("./exclude/gfcn.so");
	ExternalFunction jac_gfcn_e("./exclude/jac_gfcn.so");
	ExternalFunction hess_lfcn_e("./exclude/hess_lfcn.so");*/
	e_fct[0] = ExternalFunction("./exclude/jfcn.so");
	e_fct[1] =ExternalFunction("./exclude/grad_jfcn.so");
	e_fct[2] =ExternalFunction("./exclude/gfcn.so");
	e_fct[3] =ExternalFunction("./exclude/jac_gfcn.so");
	e_fct[4] =ExternalFunction("./exclude/hess_lfcn.so");

	// Allocate an NLP solver
	solver[0] = IpoptSolver(e_fct[0], e_fct[2], e_fct[4], e_fct[3], e_fct[1]);

	solver[0].setOption("generate_hessian",true);
	solver[0].setOption("max_iter",200);
	solver[0].setOption("tol",1e-6);

	solver[0].init();

	readTxtFile((char*)"./exclude/vars_init.txt", vars_init, 3411);
	readTxtFile((char*)"./exclude/vars_lb.txt", vars_lb, 3411);
	readTxtFile((char*)"./exclude/vars_ub.txt", vars_ub, 3411);
	readTxtFile((char*)"./exclude/lbg.txt", lbg, 3200);
	readTxtFile((char*)"./exclude/ubg.txt", ubg, 3200);

	//###################################################
	//Something strange with the bounds --> please check!
	/**for(int i = 0; i < 2202; i++)
	{
		//if(vars_lb[i] > vars_ub[i])
		//{
		cout << "LB: " << vars_lb[i] << " UB: " << vars_ub[i] << "\r\n";
			//vars_lb[i] = vars_ub[i];
		//}
	}*/
	/**for(int i = 0; i < 2100; i++)
	{
		if(lbg[i] > ubg[i])
		{
			lbg[i] = ubg[i];
		}
	}*/
	//###################################################

	//Initial condition
	solver[0].setInput(vars_init, NLP_X_INIT);

	//Bounds on x
	solver[0].setInput(vars_lb, NLP_LBX);
	solver[0].setInput(vars_ub, NLP_UBX);

	//Bounds on g
	solver[0].setInput(lbg, NLP_LBG);
	solver[0].setInput(ubg, NLP_UBG);
}

void eval(const double xcur[])
{
	if(iteration[0] > 0)
	{
		//Set current solution as initial guess for next iteration
		Matrix<double>& v_opt = solver[0].output(NLP_X_OPT);
		solver[0].setInput(v_opt,NLP_X_INIT);
	}
	else
	{
		iteration[0]++;
	}

	cout << "Scaled-States :";
	for(unsigned int i = 0; i < 8; i++)
	{
		cout << xcur[i] << " ";
		vars_lb[3 + i] = xcur[i];
		vars_ub[3 + i] = xcur[i];
	}
	cout << "\r\n";
	//Set initial condition constraint for the next iteration
	solver[0].setInput(vars_lb, NLP_LBX);
	solver[0].setInput(vars_ub, NLP_UBX);

	// Solve the NLP
	solver[0].solve();
}

void getSolution(double ucur[])
{
	//Retrieve the solution
	Matrix<double>& v_opt = solver[0].output(NLP_X_OPT);
	//Extract the control input that will be injected to the plant
	ucur[0] = v_opt.getElement(35, 0);
	ucur[1] = v_opt.getElement(36, 0);

	//cout << "Inputs: " << ucur[0] << " " << ucur[1] << "\r\n";
}
