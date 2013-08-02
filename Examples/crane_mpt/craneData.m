function [data] = craneData

%Model data for the KUL crane experiment.
%
% Usage : data = craneData(), returns a big structure
% full of the simulation parameters


%------------------------------------------
% Modelling data
%------------------------------------------

%Model data used to simulate crane dynamics
data.simModel.m     = 1318;              %mass (g)
data.simModel.g     = 9.81;              %gravity (m/s^2)
data.simModel.c     = 0;                 %damping (kgm^2/s)
data.simModel.A_C   = 0.047418203070092; %Gain of cart vel. ctrl. (m/s/V)
data.simModel.tau_C = 0.012790605943772; %time constant cart ctrl. (s)
data.simModel.A_L   = 0.034087337273386; %Gain of winch vel. ctrl. (m/s/V)
data.simModel.tau_L = 0.024695192379264; %time constant winch ctrl. (s)

%Model data to be used by controller
data.ctrlModel = data.simModel; %no model error


%------------------------------------------
% Simulation initial state
%------------------------------------------

data.initialState.x_C   = 0;     %cart position (m)
data.initialState.x_L   = 0.75;  %cable length (m)
data.initialState.v_C   = 0;     %cart velocity (m/s)
data.initialState.v_L   = 0;     %cable velocity (m/s)
data.initialState.theta = 0;     %cable deflection (rad)
data.initialState.omega = pi/4;  %cable deflection rate (rad/s)
data.initialState.u_C   = 0;     %cart controller voltage (V)
data.initialState.u_L   = 0;     %cable controller voltage (V)





