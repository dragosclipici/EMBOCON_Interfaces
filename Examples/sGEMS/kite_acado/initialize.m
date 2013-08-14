%% Run this first to initialize variables for simulation
clear all;
clc;
close all;

T = 5;
Ts = 0.1;

mex COPTIMFLAGS='-DNDEBUG -O3' kite_sim_export/integrate.c kite_sim_export/integrator.c

%% initialize Bus (Do not change this)

% initialize x_bus
x_bus = Simulink.Bus;

x   = Simulink.BusElement; x.Name = 'x';
y   = Simulink.BusElement; y.Name = 'y';
z   = Simulink.BusElement; z.Name = 'z';
dx  = Simulink.BusElement; dx.Name = 'dx';
dy = Simulink.BusElement; dy.Name = 'dy';
dz = Simulink.BusElement; dz.Name = 'dz';
e11   = Simulink.BusElement; e11.Name = 'e11';
e12   = Simulink.BusElement; e12.Name = 'e12';
e13   = Simulink.BusElement; e13.Name = 'e13';
e21   = Simulink.BusElement; e21.Name = 'e21';
e22   = Simulink.BusElement; e22.Name = 'e22';
e23   = Simulink.BusElement; e23.Name = 'e23';
e31   = Simulink.BusElement; e31.Name = 'e31';
e32   = Simulink.BusElement; e32.Name = 'e32';
e33   = Simulink.BusElement; e33.Name = 'e33';
w1   = Simulink.BusElement; w1.Name = 'w1';
w2   = Simulink.BusElement; w2.Name = 'w2';
w3   = Simulink.BusElement; w3.Name = 'w3';
r   = Simulink.BusElement; r.Name = 'r';
dr   = Simulink.BusElement; dr.Name = 'dr';
delta   = Simulink.BusElement; delta.Name = 'delta';
ddelta   = Simulink.BusElement; ddelta.Name = 'ddelta';

x_bus.Elements = [x;y;z;dx;dy;dz;e11;e12;e13;e21;e22;e23;e31;e32;e33;w1;w2;w3;r;dr;delta;ddelta];


% initialize u_bus
u_bus = Simulink.Bus;

dddelta = Simulink.BusElement; dddelta.Name = 'dddelta';
ddr = Simulink.BusElement; ddr.Name = 'ddr';
ur = Simulink.BusElement; ur.Name = 'ur';
up = Simulink.BusElement; up.Name = 'up';

u_bus.Elements = [dddelta;ddr;ur;up];



