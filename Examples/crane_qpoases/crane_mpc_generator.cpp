/**
 *	\file   crane_nmpc_generator.cpp
 *	\author Milan Vukov, milan.vukov@esat.kuleuven.be
 *	\brief  Export of the crane NMPC
 * 	\note   EMBOCON
 */

#include <acado_toolkit.hpp>

USING_NAMESPACE_ACADO

/// Cart mechanism closed loop gain
#define SOM_A1		0.047418203070092
/// Cart mechanism closed loop time constant
#define SOM_TAU1	0.012790605943772

/// Hoisting mechanism closed loop gain
#define SOM_A2		0.034087337273386
/// Hoisting mechanism closed loop time constant
#define SOM_TAU2	0.024695192379264

/// A scaling macro
#define SCALE3( a ) ( 1.0 / (a) / (a) )

int main( void )
{
	// 
	// Define the dimensions
	//
	int nMS = 25;
	double dt = 0.1;
	
	//
    // Define the variables
	//
	DifferentialState   xC;		// the trolley position
	DifferentialState	vC;		// the trolley velocity
	IntermediateState	aC;		// the trolley acceleration
	DifferentialState	xL;		// the cable length
	DifferentialState	vL;		// the cable velocity
	IntermediateState	aL;		// the cable acceleration
	DifferentialState   theta;	// the excitation angle
	DifferentialState   omega;	// the angular velocity
	
	DifferentialState   uC;		// the input to the trolley velocity
								// controller
	DifferentialState	uL;		// the input to the cable velocity
								// controller

	Control				duC;
	Control				duL;

	DifferentialEquation	f;	// Differential Equation of the model

	//
	// Set up the MPC - Optimal control problem
	//
	OCP ocp(0.0, dt*nMS, nMS);

	//
	// Define the optimization problem
	// 
	
	//
	// Define the parameters
	//

	//
	// XT( s ) / UT( s ) = A1 / ( s * ( tau1 * s + 1 ) ) 
	//
	const double	tau1 = SOM_TAU1;	// time constant
	const double	a1   = SOM_A1;		// gain

	//
	// XL( s ) / UL( s ) = A2 / ( s * ( tau2 * s + 1 ) ) 
	//
	const double	tau2 = SOM_TAU2;	// time constant
	const double	a2   = SOM_A2;		// gain

	//
	// Additional parameters
	//
	const double	g = 9.81;	// the gravitational constant 
	const double	c = 0.0;	// damping constant for the motion of
								// the pendulum

	const double 	m = 1318.0;	// the mass of the pendulum	

	//
	// Define the model equations
	//

	aC = -1.0 / tau1 * vC + a1 / tau1 * uC;
	aL = -1.0 / tau2 * vL + a2 / tau2 * uL;

	f << dot( xC )    == vC;
	f << dot( vC )    == aC;
	f << dot( xL )    == vL;
	f << dot( vL )    == aL;
	f << dot( theta ) == omega;
	f << dot( omega ) == 1.0 / xL * (-g * sin( theta ) - aC * cos( theta )
		- 2 * vL * omega - c * omega / ( m * xL ) );

	f << dot( uC )    == duC;
	f << dot( uL )    == duL;

	ocp.subjectTo( f );

	ocp.subjectTo( -10.0 <= uC <= 10.0 );
	ocp.subjectTo( -10.0 <= uL <= 10.0 );

	ocp.subjectTo( -100.0 <= duC <= 100.0 );
	ocp.subjectTo( -100.0 <= duL <= 100.0 );

	// Define the weighting matrices
	Matrix Q = eye( 8 );
	Matrix R = eye( 2 );
	Matrix P = eye( 8 );
	
	P *= 1e-10;

// 	//trolley
// 	Q(0, 0) = SCALE3( SOM_A1 * 10.0 )	* 1.0;
// 	Q(1, 1) = SCALE3( SOM_A1 * 10.0 )	* 0.35;
// 
// 	//hoisting
// 	Q(2, 2) = SCALE3( SOM_A2 * 10.0 )	* 1.0;
// 	Q(3, 3) = SCALE3( SOM_A2 * 10.0 )	* 1e-4;
// 
// 	// angle
// 	Q(4, 4) = SCALE3( 0.1 )				* 0.01;
// 	Q(5, 5) = SCALE3( 0.3 )				* 0.5;
// 
// 	// controls
// 	Q(6, 6) = SCALE3( 10.0 )			* ;
// 	Q(7, 7) = SCALE3( 10.0 )			* 1e-10;
// 
// 	// control rates
// 	R(0, 0) = SCALE3( 100.0 )			* 0.1;
// 	R(1, 1) = SCALE3( 100.0 )			* 0.1;
	
	// NEW weights
	
	//trolley
	Q(0, 0) = 1e2;
	Q(1, 1) = 1e2;

	//hoisting
	Q(2, 2) = 1e2;
	Q(3, 3) = 1e2;

	// angle
	Q(4, 4) = 1e2;
	Q(5, 5) = 1e2;

	// controls
	Q(6, 6) = 1.;
	Q(7, 7) = 1.;
// 	Q(6, 6) = SCALE3( 10.0 )	* 1e-4;
// 	Q(7, 7) = SCALE3( 10.0 )	* 1e-4;

	// control rates
	R(0, 0) = 1.;
	R(1, 1) = 1.;
// 	R(0, 0) = SCALE3( 100.0 )	* 1e-2;
// 	R(1, 1) = SCALE3( 100.0 )	* 1e-2;

	// Define the LSQ
	ocp.minimizeLSQ( Q, R );
	ocp.minimizeLSQEndTerm( P );

	//
	// Define an MPC export module and generate the code
	//
	MPCexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
// 	mpc.set( DISCRETIZATION_TYPE,   MULTIPLE_SHOOTING );
	mpc.set( DISCRETIZATION_TYPE,   SINGLE_SHOOTING );
	mpc.set( INTEGRATOR_TYPE,       INT_RK4 );
	mpc.set( NUM_INTEGRATOR_STEPS,  nMS * 10 );
	mpc.set( QP_SOLVER,             QP_QPOASES );
	mpc.set( HOTSTART_QP,           YES );
	mpc.set( GENERATE_TEST_FILE,    NO );
	mpc.set( GENERATE_MAKE_FILE,    NO );

	mpc.exportCode( "./crane_mpc_export" );

    return 0;
}



