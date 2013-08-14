%% Run this first to initialize variables for simulation
clear all;

%% set xref (Use this to change x_ref)
xref.x_C   = 1;     %cart position (m)
xref.x_L   = 1.5;   %cable length (m)
xref.v_C   = 0;     %cart velocity (m/s)
xref.v_L   = 0;     %cable velocity (m/s)
xref.theta = 0;     %cable deflection (rad)
xref.omega = 0;     %cable deflection rate (rad/s)
xref.u_C   = 0;     %cart controller voltage (V)
xref.u_L   = 0;     %cable controller voltage (V)


%% initialize Bus (Do not change this)

% initialize x_bus
x_bus = Simulink.Bus;

x_C   = Simulink.BusElement; x_C.Name = 'x_C';
x_L   = Simulink.BusElement; x_L.Name = 'x_L';
v_C   = Simulink.BusElement; v_C.Name = 'v_C';
v_L   = Simulink.BusElement; v_L.Name = 'v_L';
theta = Simulink.BusElement; theta.Name = 'theta';
omega = Simulink.BusElement; omega.Name = 'omega';
u_C   = Simulink.BusElement; u_C.Name = 'u_C';
u_L   = Simulink.BusElement; u_L.Name = 'u_L';

x_bus.Elements = [x_C; x_L; v_C; v_L; theta; omega; u_C; u_L];

% initialize xref_bus
xref_bus = Simulink.Bus;

x_C   = Simulink.BusElement; x_C.Name = 'x_C';
x_L   = Simulink.BusElement; x_L.Name = 'x_L';
v_C   = Simulink.BusElement; v_C.Name = 'v_C';
v_L   = Simulink.BusElement; v_L.Name = 'v_L';
theta = Simulink.BusElement; theta.Name = 'theta';
omega = Simulink.BusElement; omega.Name = 'omega';
u_C   = Simulink.BusElement; u_C.Name = 'u_C';
u_L   = Simulink.BusElement; u_L.Name = 'u_L';

xref_bus.Elements = [x_C; x_L; v_C; v_L; theta; omega; u_C; u_L];

% initialize u_bus
u_bus = Simulink.Bus;

u_CR = Simulink.BusElement; u_CR.Name = 'u_CR';
u_LR = Simulink.BusElement; u_LR.Name = 'u_LR';

u_bus.Elements = [u_CR; u_LR];
