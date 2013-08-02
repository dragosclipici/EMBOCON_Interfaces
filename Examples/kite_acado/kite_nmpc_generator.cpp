/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC) under
 *    supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


 /**
 *    SINGLE POWER KITE START-UP WITH PROPELLER
 *    CARTESIAN COORDINATES (ODE FORMULATION)
 *    JULY 2011 SEBASTIEN GROS, HIGHWIND, OPTEC
 *    SEBASTIEN GROS
 */


#include <acado_code_generation.hpp>

USING_NAMESPACE_ACADO

int main( void )
{
	USING_NAMESPACE_ACADO


// DIFFERENTIAL STATES :
// -------------------------
  
   DifferentialState      x;      // Position
   DifferentialState	  y;      //  
   DifferentialState      z;      //  
// -------------------------      //  -------------------------------------------
  
   DifferentialState     dx;      //  Speed
   DifferentialState     dy;      //  
   DifferentialState     dz;      //  
	
/*	DifferentialState    q0;
	DifferentialState    q1;
	DifferentialState    q2;
	DifferentialState    q3;*/	
	
	DifferentialState    e11;
	DifferentialState    e12;
	DifferentialState    e13;
	DifferentialState    e21;
	DifferentialState    e22;
	DifferentialState    e23;
	DifferentialState    e31;
	DifferentialState    e32;
	DifferentialState    e33;
	
	
	DifferentialState    w1;
	DifferentialState    w2;
	DifferentialState    w3;	
// -------------------------      //  -------------------------------------------
   DifferentialState      r;      //  Kite distance
   DifferentialState     dr;      //  Kite distance / dt

//-------------------------      //  -------------------------------------------
 
	 DifferentialState  delta;      //  Carousel
	 DifferentialState ddelta;      //  
	
 	
// CONTROL :
// -------------------------
	Control             dddelta;  //  Carousel acceleration
	Control            ddr;
	Control             ur;
	Control             up;      //  Ailerons  
	
// PARAMETERS
// -----------------------
	

// 	double Nturn = 2;
// 	double Ncvp = 40;
// 	double Torbit = 2*Nturn;
	
	double Ncvp = 20;
	double Tc = 2.;
	
	
	// DEFINITION OF PI :
	// ------------------------
	
	double PI = 3.1415926535897932;
	
	
		
	//TAIL LENGTH
	double LT = 0.45;
	
	
	//ROLL DAMPING
	double RDfac = 1;
	double RD0 = 1e-2; 
	double RD = RDfac*RD0;    
	

// CONSTANTS :
// ------------------------
	
	//  PARAMETERS OF THE KITE :
	//  -----------------------------
	double mk =  0.463;      //  mass of the kite               //  [ kg    ]
	
	
	//   PHYSICAL CONSTANTS :
	//  -----------------------------
	double g =    9.81;      //  gravitational constant         //  [ m /s^2]
	double rho =    1.23;      //  density of the air             //  [ kg/m^3]
	
	//  PARAMETERS OF THE CABLE :
	//  -----------------------------
	double rhoc = 1450.00;      //  density of the cable           //  [ kg/m^3]
	double cc =   1.00;      //  frictional constant            //  [       ]
	double dc = 1e-3;      //  diameter                       //  [ m     ]
	
	
	
	double AQ      =  PI*dc*dc/4.0;
	
		
	
	//CAROUSEL ARM LENGTH
	double rA = 1.085; //(dixit Kurt)
	
	
	//INERTIA MATRIX (Kurt's direct measurements)
	// Note: low sensitivity to I1,2,3... high sensitivity to I31...
	double I1 =  0.0163;
	double I31 = 0.0006;
	double I2 =  0.0078;
	double I3 =  0.0229;
					 
									
					 
	//WIND-TUNNEL PARAMETERS
					 
	//Lift (report p. 67)
	//Sensitivity to CLA error low
	double CLA = 5.064;
	//Sensitivity to CLe error low
	double CLe = 0.318;
	//Sensitivity to CLr error low
	double CLr = 0.85; //?!?!?!?!?
	//HIGH sensitivity to CL0 !!
	double CL0 = 0.239;
					 
	//Drag (report p. 70)
	//Sensitivity to CDA error low
	double CDA = -0.195;
	double CDA2 = 4.268;
	double CDB2 = 0;
	//Sensitivity to CDe error low
	double CDe = 0.044;
	//Sensitivity to CDr error low
	double CDr = 0.111;
	//Sensitivity to CD0 error low
	double CD0 = 0.026;
					 
	//Roll (report p. 72)
	//HIGH sensitivity to CRB !!
	double CRB = -0.062;
	//HIGH sensitivity to CRAB !!
	double CRAB = -0.271;
	//Sensitivity to CRr error low
	double CRr = -0.244;
					 
	//Pitch (report p. 74)
	//HIGH sensitivity to CPA !!
	double CPA = 0.293;
	//Sensitivity to CPe error low
	double CPe = -0.821;
	//Sensitivity to CPr error low
	double CPr = -0.647; //?!?!?!?!?
	//HIGH sensitivity to CP0 !!
	double CP0 = 0.03;
					 
	//Yaw (report p. 76)
	//HIGH sensitivity to CYB !!
	double CYB = 0.05;
	//HIGH sensitivity to CYAB !!
	double CYAB = 0.229;
					 
	double SPAN = 0.96;
	double CHORD = 0.1;
		
		

// OTHER VARIABLES :
// ------------------------

   
	
   IntermediateState     mc;      //  mass of the cable
   IntermediateState     m ;      //  effective inertial mass
   IntermediateState  mgrav;      //  gravific mass
 //  IntermediateState     dmc;      //  first  derivative of m     with respect to t

	
	

// ORIENTATION AND FORCES :
// ------------------------
	
	
	IntermediateState wind               ;      //  the wind at altitude 	
	
	IntermediateState Cf              ;      //  cable drag
	IntermediateState CD              ;      //  the aerodynamic drag coefficient
	IntermediateState CL              ;      //  the aerodynamic lift coefficient
	IntermediateState CR              ;      //  the aerodynamic roll coefficient
	IntermediateState CP              ;      //  the aerodynamic pitch coefficient
	IntermediateState CY              ;      //  the aerodynamic yaw coefficient
	
	IntermediateState F           [ 3];      //  aero forces + gravity
	IntermediateState FL          [ 3];      //  the lift force
	IntermediateState FD          [ 3];      //  the drag force
	IntermediateState Ff          [ 3];      //  the frictional force
	IntermediateState Fcable             ;      //  force in the cable

	
	IntermediateState er          [ 3];      // X normed to 1
	IntermediateState eTe         [ 3];      //unrotated transversal vector (psi = 0)
	IntermediateState eLe         [ 3];      //unrotated lift vector (psi = 0)
	IntermediateState we          [ 3];      //  effective wind vector
	IntermediateState wE          [ 3];      //  effective wind vector
	IntermediateState wp              ;		
	IntermediateState wep         [ 3];		// effective wind projected in the plane orthogonal to X
	
	IntermediateState VKite           ;     //Kite (relative) speed
	IntermediateState VKite2          ;     //Squared (relative) kite speed
		
	IntermediateState Vp;
	IntermediateState VT			[3];
	IntermediateState alpha;
	IntermediateState beta;
	IntermediateState alphaTail;
	IntermediateState T			    [3];
	
	
	// TERMS ON RIGHT-HAND-SIDE
	// OF THE DIFFERENTIAL
	// EQUATIONS              :
	// ------------------------
	
// 	IntermediateState dq0;
// 	IntermediateState dq1;
// 	IntermediateState dq2; 
// 	IntermediateState dq3;      
	
	IntermediateState de11;
	IntermediateState de12;
	IntermediateState de13;
	IntermediateState de21;
	IntermediateState de22;
	IntermediateState de23;
	IntermediateState de31;
	IntermediateState de32;
	IntermediateState de33;
	
// ------------------------                 //  ----------------------------------------------
    IntermediateState dCost  ;      //  regularisation of controls




//                        MODEL EQUATIONS :
// ===============================================================

	// CROSS AREA OF THE CABLE :
	// ---------------------------------------------------------------
	
	AQ      =  PI*dc*dc/4.0                                       ;
	
	// THE EFECTIVE MASS' :
	// ---------------------------------------------------------------
	
	mc      =  rhoc*AQ*r  ;   // mass of the cable
	m       =  mk + mc/3.0;   // effective inertial mass
	mgrav   =  mk + mc/2.0;   // effective inertial mass
	
	// -----------------------------   // ----------------------------
	//   dm      =  (rhoc*AQ/ 3.0)*dr;   // time derivative of the mass
	
	
	// WIND SHEAR MODEL :
	// ---------------------------------------------------------------
	
	
	wind       =  0                        ;
	
	
	// EFFECTIVE WIND IN THE KITE`S SYSTEM :
	// ---------------------------------------------------------------
	
	we[0]   = -wind + dx;
	we[1]   =		  dy;
	we[2]   =		  dz;
	
	VKite2 = (we[0]*we[0] + we[1]*we[1] + we[2]*we[2]); 
	VKite = sqrt(VKite2); 
	
	// CALCULATION OF THE FORCES :
	// ---------------------------------------------------------------
	
	// er
    er[0] = x/r;
	er[1] = y/r;
	er[2] = z/r;
	
	//Velocity accross X (cable drag)
	wp = er[0]*we[0] + er[1]*we[1] + er[2]*we[2];
	wep[0] = we[0] - wp*er[0];
	wep[1] = we[1] - wp*er[1];
	wep[2] = we[2] - wp*er[2];
	
	//Aero coeff.
	
	
    // LIFT DIRECTION VECTOR
    // -------------------------
    
    //Relative wind speed in Airfoil's referential 'E'
    wE[0] = e11*we[0]  + e21*we[1]  + e31*we[2];
    wE[1] = e12*we[0]  + e22*we[1]  + e32*we[2];
    wE[2] = e13*we[0]  + e23*we[1]  + e33*we[2];
//     wE[0] = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*we[0]  +              (2*q0*q3 + 2*q1*q2)*we[1]  +              (2*q1*q3 - 2*q0*q2)*we[2];
//     wE[1] =             (2*q1*q2 - 2*q0*q3)*we[0]  +  (q0*q0 - q1*q1 + q2*q2 - q3*q3)*we[1]  +              (2*q0*q1 + 2*q2*q3)*we[2];
//     wE[2] =             (2*q0*q2 + 2*q1*q3)*we[0]  +              (2*q2*q3 - 2*q0*q1)*we[1]  +  (q0*q0 - q1*q1 - q2*q2 + q3*q3)*we[2];
    
    
    
    //Airfoil's transversal axis in fixed referential 'e'
    eTe[0] = e12;
    eTe[1] = e22;
    eTe[2] = e32;
//     eTe[0] =                2*q1*q2 - 2*q0*q3;        
//     eTe[1] =  q0*q0 - q1*q1 + q2*q2 -   q3*q3;
//     eTe[2] =                2*q0*q1 + 2*q2*q3;  
    
    
    // Lift axis ** Normed to we !! **
    eLe[0] = - eTe[1]*we[2] + eTe[2]*we[1];
	eLe[1] = - eTe[2]*we[0] + eTe[0]*we[2];
	eLe[2] = - eTe[0]*we[1] + eTe[1]*we[0];
	
    // AERODYNAMIC COEEFICIENTS
    // ----------------------------------
    //VT = cross([w1;w2;w3],[-LT;0;0]) + wE;
	
	VT[0] =          wE[0];
	VT[1] = -LT*w3 + wE[1];
	VT[2] =  LT*w2 + wE[2];
// 	VT[0] =  wE[0];
// 	VT[1] =  wE[1];
// 	VT[2] =  wE[2];
	
    alpha = -wE[2]/wE[0];
    
    //Note: beta & alphaTail are compensated for the tail motion induced by omega
	beta = VT[1]/sqrt(VT[0]*VT[0] + VT[2]*VT[2]);
    alphaTail = -VT[2]/VT[0];
    
	CL = CLA*alpha + CLe*up     + CLr*ur + CL0;
    CD = CDA*alpha + CDA2*alpha*alpha + CDB2*beta*beta + CDe*up + CDr*ur + CD0;
    CR = -RD*w1 + CRB*beta + CRAB*alphaTail*beta + CRr*ur;
    CP = CPA*alphaTail + CPe*up + CPr*ur + CP0;
    CY = CYB*beta + CYAB*alphaTail*beta;
	
	
	Cf = rho*dc*r*VKite/8.0;
    
	// THE FRICTION OF THE CABLE :
	// ---------------------------------------------------------------
	
	Ff[0] = -rho*dc*r*VKite*cc*wep[0]/8.0;
	Ff[1] = -rho*dc*r*VKite*cc*wep[1]/8.0;
	Ff[2] = -rho*dc*r*VKite*cc*wep[2]/8.0;
	
	// LIFT :
	// ---------------------------------------------------------------
	
	FL[0] =  rho*CL*eLe[0]*VKite/2.0;
    FL[1] =  rho*CL*eLe[1]*VKite/2.0;
    FL[2] =  rho*CL*eLe[2]*VKite/2.0;
	
	// DRAG :
	// ---------------------------------------------------------------
	
	FD[0] = -rho*VKite*CD*we[0]/2.0;
    FD[1] = -rho*VKite*CD*we[1]/2.0; 
    FD[2] = -rho*VKite*CD*we[2]/2.0; 
	
	
	// FORCES (AERO)
	// ---------------------------------------------------------------
	
	F[0] = FL[0] + FD[0] + Ff[0];
	F[1] = FL[1] + FD[1] + Ff[1];
	F[2] = FL[2] + FD[2] + Ff[2];
	
	// TORQUES (AERO)
	// ---------------------------------------------------------------
	
	T[0] =  0.5*rho*VKite2*SPAN*CR;
	T[1] =  0.5*rho*VKite2*CHORD*CP;
	T[2] =  0.5*rho*VKite2*SPAN*CY;
	
	
	
	// ATTITUDE DYNAMICS
	// -----------------------------------------------------------
	
// 	dq0 = (-q1*w1 - q2*w2 - q3*w3)/2;
// 	dq1 = ( q0*w1 - q3*w2 + q2*w3)/2;
// 	dq2 = ( q3*w1 + q0*w2 - q1*w3)/2;
// 	dq3 = (-q2*w1 + q1*w2 + q0*w3)/2;
	
	de11 =  e12*w3 - e13*w2;
	de12 =  e13*w1 - e11*w3;
	de13 =  e11*w2 - e12*w1;
	de21 =  e22*w3 - e23*w2;
	de22 =  e23*w1 - e21*w3;
	de23 =  e21*w2 - e22*w1;
	de31 =  e32*w3 - e33*w2;
	de32 =  e33*w1 - e31*w3;
	de33 =  e31*w2 - e32*w1;
	
	
	
	//////////////////////////////////////////////////////////////////////// 
	//                                                                    // 
	//  AUTO-GENERATED EQUATIONS (S. Gros, HIGHWIND, OPTEC, KU Leuven)    // 
	//                                                                    // 
	//////////////////////////////////////////////////////////////////////// 
	
	// Equations read: 
	// IMA = inv(MA) 
	// ddX = IMA*(Bx - CA*lambda) 
	// lambdaNum = CA^T*IMA*Bx - Blambda 
	// lambdaDen = CA^T*IMA*CA 
	// lambda = lambdaNum/lambdaDen 
	
	// Arm 
	IntermediateState xA; 
	IntermediateState dxA; 
	IntermediateState ddxA; 
	IntermediateState yA; 
	IntermediateState dyA; 
	IntermediateState ddyA; 
	
// 	xA = rA*sin(delta); 
// 	dxA = ddelta*rA*cos(delta); 
// 	ddxA = dddelta*rA*cos(delta) - ddelta*ddelta*rA*sin(delta); 
	xA = -rA*sin(delta); 
	dxA = -(ddelta*rA*cos(delta)); 
	ddxA = -(dddelta*rA*cos(delta) - ddelta*ddelta*rA*sin(delta)); 
	yA = rA*cos(delta); 
	dyA = -ddelta*rA*sin(delta); 
	ddyA = - rA*cos(delta)*ddelta*ddelta - dddelta*rA*sin(delta); 
	
	// BUILD DYNAMICS 
	IntermediateState lambdaNum; 
	lambdaNum = ddxA*xA - 2*dy*dyA - ddr*r - ddxA*x - 2*dx*dxA - ddyA*y + ddyA*yA - dr*dr + dx*dx + dxA*dxA + dy*dy + dyA*dyA + dz*dz + (F[0]*(x - xA))/m + (F[1]*(y - yA))/m + (z*(F[2] - g*mgrav))/m; 
	
	IntermediateState lambdaDen; 
	lambdaDen = x*x/m + xA*xA/m + y*y/m + yA*yA/m + z*z/m - (2*x*xA)/m - (2*y*yA)/m; 
	
	IntermediateState lambda; 
	lambda = lambdaNum/lambdaDen; 
	
	IntermediateState ddX(6,1); 
	ddX(0,0) = (F[0] - lambda*(x - xA))/m; 
	ddX(1,0) = (F[1] - lambda*(y - yA))/m; 
	ddX(2,0) = -(g*mgrav - F[2] + lambda*z)/m; 
	ddX(3,0) = (I31*(T[2] + w2*(I1*w1 + I31*w3) - I2*w1*w2))/(I31*I31 - I1*I3) - (I3*(T[0] - w2*(I31*w1 + I3*w3) + I2*w2*w3))/(I31*I31 - I1*I3); 
	ddX(4,0) = (T[1] + w1*(I31*w1 + I3*w3) - w3*(I1*w1 + I31*w3))/I2; 
	ddX(5,0) = (I31*(T[0] - w2*(I31*w1 + I3*w3) + I2*w2*w3))/(I31*I31 - I1*I3) - (I1*(T[2] + w2*(I1*w1 + I31*w3) - I2*w1*w2))/(I31*I31 - I1*I3); 
	
	// BUILD CONSTRAINTS 
	IntermediateState Const, dConst; 
	Const = - r*r/2 + x*x/2 - x*xA + xA*xA/2 + y*y/2 - y*yA + yA*yA/2 + z*z/2; 
	dConst = dx*x - dr*r - dxA*x - dx*xA + dxA*xA + dy*y - dyA*y - dy*yA + dyA*yA + dz*z; 
	
	/*
	// AIRCRAFT REF. FRAME ACCELERATION 
	IntermediateState ddxIMU; 
	IntermediateState ddyIMU; 
	IntermediateState ddzIMU; 
	IntermediateState w1IMU; 
	IntermediateState w2IMU; 
	IntermediateState w3IMU; 
	ddxIMU = ddX(0,0)*(q0*q0 + q1*q1 - q2*q2 - q3*q3) - (ddX(2,0) + g)*(2*q0*q2 - 2*q1*q3) + ddX(1,0)*(2*q0*q3 + 2*q1*q2); 
	ddyIMU = ddX(1,0)*(q0*q0 - q1*q1 + q2*q2 - q3*q3) + (ddX(2,0) + g)*(2*q0*q1 + 2*q2*q3) - ddX(0,0)*(2*q0*q3 - 2*q1*q2); 
	ddzIMU = ddX(0,0)*(2*q0*q2 + 2*q1*q3) - ddX(1,0)*(2*q0*q1 - 2*q2*q3) + (ddX(2,0) + g)*(q0*q0 - q1*q1 - q2*q2 + q3*q3); 
	w1IMU = w1; 
	w2IMU = w2; 
	w3IMU = w3; */
	
	///////////////////////////// END OF AUTO-GENERATED CODE ////////////////////////////////////////////////////// 
	
	
// 	IntermediateState ConstQ;
// 	ConstQ = q0*q0 + q1*q1 + q2*q2 + q3*q3 - 1;
	IntermediateState ConstR1;
	IntermediateState ConstR2;
	IntermediateState ConstR3;
	IntermediateState ConstR4;
	IntermediateState ConstR5;
	IntermediateState ConstR6;
	ConstR1 = e11*e11 + e12*e12 + e13*e13 - 1;
	ConstR2 = e11*e21 + e12*e22 + e13*e23;
	ConstR3 = e11*e31 + e12*e32 + e13*e33;
	ConstR4 = e21*e21 + e22*e22 + e23*e23 - 1;
	ConstR5 = e21*e31 + e22*e32 + e23*e33;
	ConstR6 = e31*e31 + e32*e32 + e33*e33 - 1;
	
	

	Fcable = lambda*r;

	
// THE "RIGHT-HAND-SIDE" OF THE ODE:
// ---------------------------------------------------------------
   DifferentialEquation f( 0.0, Tc );

  
   f  << dot(x)      ==  dx                             ;
   f  << dot(y)	     ==  dy                             ;
   f  << dot(z)		 ==  dz                             ;
   f  << dot(dx)     ==  ddX(0,0)                            ;
   f  << dot(dy)	 ==  ddX(1,0)                            ;
   f  << dot(dz)	 ==  ddX(2,0)                            ;
   
   f  << dot(e11)	 ==  de11;
   f  << dot(e12)	 ==  de12;
   f  << dot(e13)	 ==  de13;
   f  << dot(e21)	 ==  de21;
   f  << dot(e22)	 ==  de22;
   f  << dot(e23)	 ==  de23;
   f  << dot(e31)	 ==  de31;
   f  << dot(e32)	 ==  de32;
   f  << dot(e33)	 ==  de33;
//    f  << dot(q0)	 ==  dq0                            ;
//    f  << dot(q1)	 ==  dq1                           ;
//    f  << dot(q2)	 ==  dq2                            ;
//    f  << dot(q3)	 ==  dq3                            ;
	
   f  << dot(w1)	 ==  ddX(3,0)                            ;
   f  << dot(w2)	 ==  ddX(4,0)                            ;
   f  << dot(w3)	 ==  ddX(5,0)                            ;

   f  << dot(r)      ==  dr                             ;
   f  << dot(dr)     ==  ddr                            ;
   f  << dot(delta)  ==  ddelta                         ;
   f  << dot(ddelta) ==  dddelta                        ;


	// DEFINE THE WEIGHTING MATRICES:
	// ----------------------------------------------------------
	Matrix Q(22,22);
	Q.setZero();	
	Matrix R(4,4);
	R.setZero();
	
	double Qscale = 1e-2;
	double Sx = (2*2);
	double Sy = (2*2);
	double Sz = (2*2);
	double Sdx = (10*10);
	double Sdy = (10*10);
	double Sdz = (10*10);
	double Sw  = (10*10);
	double Sdr = (3*3);
	double Sup = (0.2*0.2);
	double Sur = (0.2*0.2);
	//--- POSITION  ---
	Q(0,0) = Qscale*1e2/Sx;
	Q(1,1) = Qscale*1e2/Sy;
	Q(2,2) = Qscale*1e2/Sz;
	//--- VELOCITY  ---
	Q(3,3) = Qscale*1e2/Sdx;
	Q(4,4) = Qscale*1e2/Sdy;
	Q(5,5) = Qscale*1e2/Sdz;
	//--- ORIENTATION  ---
	Q(6,6) = Qscale*1e2;
	Q(7,7) = Qscale*1e2;
	Q(8,8) = Qscale*1e2;
	Q(9,9) = Qscale*1e2;
	Q(10,10) = Qscale*1e2;
	Q(11,11) = Qscale*1e2;
	Q(12,12) = Qscale*1e2;
	Q(13,13) = Qscale*1e2;
	Q(14,14) = Qscale*1e2;
	//--- ANGULAR VELOCITY ---
	Q(15,15) = Qscale*1e2/Sw;
	Q(16,16) = Qscale*1e2/Sw;
	Q(17,17) = Qscale*1e2/Sw;
	//--- CABLE & CARROUSEL ---
	Q(18,18) = Qscale*1e4;
	Q(19,19) = Qscale*1e0/Sdr;
	Q(20,20) = Qscale*1e2;
	Q(21,21) = Qscale*1e2;
	
	//--- CONTROL ---
	R(0,0) = Qscale*1e2;
	R(1,1) = Qscale*1e2;
	R(2,2) = Qscale*1e2/Sup;
	R(3,3) = Qscale*1e2/Sur;

    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( 0.0, Tc, Ncvp );
    ocp.minimizeLSQ( Q,R );
	//ocp.minimizeLSQEndTerm( P );
	
//     ocp.minimizeLagrangeTerm( ddr*ddr + dddelta*dddelta + up*up + ur*ur ); // + T[0]*T[0] + T[1]*T[1] + T[2]*T[2] );

    ocp.subjectTo( f );


    // INITIAL VALUE CONSTRAINTS:
    // ---------------------------------

		
	////CONSISTENCY CONDITIONS
 	//ocp.subjectTo( AT_START, ConstR1 == 0.0 );
 	//ocp.subjectTo( AT_START, ConstR2 == 0.0 );
 	//ocp.subjectTo( AT_START, ConstR3 == 0.0 );
 	//ocp.subjectTo( AT_START, ConstR4 == 0.0 );
 	//ocp.subjectTo( AT_START, ConstR5 == 0.0 );
 	//ocp.subjectTo( AT_START, ConstR6 == 0.0 );
 	//ocp.subjectTo( AT_START,  Const == 0.0 );
 	//ocp.subjectTo( AT_START, dConst == 0.0 );
	
 	//ocp.subjectTo( AT_START,  r ==  1.0 );
 	//ocp.subjectTo( AT_START, dr ==  0.0 );
 	
 	//ocp.subjectTo( AT_START,  z == -0.2 );
 	//ocp.subjectTo( AT_END  ,  z ==  0.2 );
 	//ocp.subjectTo( AT_END  , dz ==  0.0 );
	
	//ocp.subjectTo( AT_END  ,  r ==  1.2 );
	//ocp.subjectTo( AT_END  , dr ==  0.0 );

 	//ocp.subjectTo(-1.0 <= CL <= 1.0);
	


	//// CONTROL BOUNDS
	double AccRate = 30*PI/180;
	ocp.subjectTo( -AccRate <= dddelta <= AccRate );
	ocp.subjectTo( -0.5 <= ddr <= 0.5 );
	ocp.subjectTo( -0.2 <= up <= 0.2 );
	ocp.subjectTo( -0.2 <= ur <= 0.2 );
	
	//// STATE BOUNDS
	ocp.subjectTo( -1 <= e11 <= 1 );
	ocp.subjectTo( -1 <= e12 <= 1 );
	ocp.subjectTo( -1 <= e13 <= 1 );
	ocp.subjectTo( -1 <= e21 <= 1 );
	ocp.subjectTo( -1 <= e22 <= 1 );
	ocp.subjectTo( -1 <= e23 <= 1 );
	ocp.subjectTo( -1 <= e31 <= 1 );
	ocp.subjectTo( -1 <= e32 <= 1 );
	ocp.subjectTo( -1 <= e33 <= 1 );


    //ocp.subjectTo(-0.0 <= CL <= 1.0);
	
	
	//ocp.subjectTo( 0 <= lambda/30 );
	//ocp.subjectTo( -10*PI/180 <= beta <= 10*PI/180 );
//// 	ocp.subjectTo( -10*PI/180 <= alpha <= 10*PI/180 );
	//ocp.subjectTo( -15*PI/180 <= alphaTail <= 15*PI/180 );

	// DEFINE AN MPC EXPORT MODULE AND GENERATE THE CODE:
	// ----------------------------------------------------------
	MPCexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
	mpc.set( DISCRETIZATION_TYPE,         SINGLE_SHOOTING );
	mpc.set( INTEGRATOR_TYPE,             INT_IRK_GL4     );
	mpc.set( NUM_INTEGRATOR_STEPS,        40              );
	mpc.set( IMPLICIT_INTEGRATOR_NUM_ITS, 2				  );
	mpc.set( IMPLICIT_INTEGRATOR_NUM_ITS_INIT, 0		  );
	mpc.set( LINEAR_ALGEBRA_SOLVER,		  GAUSS_LU		  );
	mpc.set( UNROLL_LINEAR_SOLVER,        BT_FALSE	      );
	mpc.set( QP_SOLVER,                   QP_QPOASES      );
	mpc.set( HOTSTART_QP,           	  YES 			  );
	mpc.set( GENERATE_TEST_FILE,          NO              );
	mpc.set( GENERATE_MAKE_FILE,          NO              );
	mpc.set( GENERATE_SIMULINK_INTERFACE, NO              );
	mpc.set( GENERATE_MATLAB_INTERFACE,   YES             );

	mpc.exportCode( "kite_nmpc_export" );
	// ----------------------------------------------------------

	// DEFINE A SIMULATION EXPORT MODULE AND GENERATE THE CODE:
	// ----------------------------------------------------------
	SIMexport sim( 1, 0.1 );
	
	sim.setModel( f );
	
	sim.set( INTEGRATOR_TYPE,             INT_IRK_GL4     );
	sim.set( NUM_INTEGRATOR_STEPS,        2              );
	sim.set( IMPLICIT_INTEGRATOR_NUM_ITS, 2				  );
	sim.set( IMPLICIT_INTEGRATOR_NUM_ITS_INIT, 0		  );
	sim.set( LINEAR_ALGEBRA_SOLVER,		  GAUSS_LU		  );
	sim.set( UNROLL_LINEAR_SOLVER,        BT_FALSE	      );
	sim.set( GENERATE_MATLAB_INTERFACE,   YES             );

	sim.exportCode( "kite_sim_export" );

    return 0;
}



