%% INFORMATION FILE, THIS IS NOT DIRECTLY USED FOR SIMULATION
% Non-linear Continuous model equations can be found in deliverable 8.2 and are the following

% x_C'  = v_C
% x_L'  = v_L
% v_C'  = -1/tau_C*v_C + A_C/tau_C*u_C
% v_L'  = -1/tau_L*v_L + A_L/tau_L*u_L
% theta'= omega 
% omega'= - 1/x_L ( g*sin(theta) + ( -1/tau_C*v_C + A_C/tau_C*u_C )*cos(theta) + 2*v_L*omega + (c*omega)(m*x_L) )
% u_C'  = u_CR
% u_L'  = u_LR 

% Outputs
% x_C, x_L, v_C, v_L, theta, omega, u_C, u_L

% Inputs
% u_CR, u_LR


%% system
nx = 8;
nu = 2;
ts = 0.01;

%constants
m     = 1318;              %mass (g)
g     = 9.81;              %gravity (m/s^2)
c     = 0;                 %damping (kgm^2/s)
A_C   = 0.047418203070092; %Gain of cart vel. ctrl. (m/s/V) 
tau_C = 0.012790605943772; %time constant cart ctrl. (s)
A_L   = 0.034087337273386; %Gain of winch vel. ctrl. (m/s/V)
tau_L = 0.024695192379264; %time constant winch ctrl. (s)

%working point
x_C   = 1;     %cart position (m)
x_L   = 1.5;   %cable length (m)
v_C   = 0;     %cart velocity (m/s)
v_L   = 0;     %cable velocity (m/s)
theta = 0;     %cable deflection (rad)
omega = 0;     %cable deflection rate (rad/s)
u_C   = 0;     %cart controller voltage (V)
u_L   = 0;     %cable controller voltage (V)

xlin = [x_C; x_L; v_C; v_C; theta; omega; u_C; u_L]; % point around which is linearized

% the reference state (setpoint) is the state around which is linearized
xref = xlin;
uref = zeros(2,1); % reference input are zeros

% Continuous Linearized system

%w=omega_dot and omega is just the normal omega
dwdx_L = (2*omega/x_L *c/m + 2*v_L*omega + g*sin(theta) + (A_C*u_C-v_C)/tau_C*cos(theta)) / x_L^2;
dwdv_C = cos(theta)/(tau_C*x_L);
dwdv_L = -2*omega/x_L;
dwdtheta = ( (A_C*u_C-v_C)*sin(theta)/tau_C - g*cos(theta) )/x_L;
dwdomega = -2*v_L/x_L;
dwdu_C = -A_C*cos(theta)/(tau_C*x_L);

A_c = [ 0       0       1       0       0       0       0       0;      % x_C
        0       0       0       1       0       0       0       0;      % x_L
        0       0   -1/tau_C    0       0       0   A_C/tau_C   0;      % v_L
        0       0       0    -1/tau_L   0       0       0    A_L/tau_L; % v_C
        0       0       0       0       0       1       0       0;      % theta
        0     dwdx_L dwdv_C   dwdv_L dwdtheta dwdomega dwdu_C   0;      % omega
        0       0       0       0       0       0       0       0;      % u_C
        0       0       0       0       0       0       0       0];     % u_L
        

B_c = [   0       0;  % x_C
          0       0;  % x_L
          0       0;  % v_C
          0       0;  % v_L
          0       0;  % theta
          0       0;  % omega
          1       0;  % u_C
          0       1]; % u_L
    
% Discrete Linearized system
M = [A_c*ts B_c*ts; zeros(nu,nx+nu)];
Ns = expm(M);
A_d = Ns(1:nx,1:nx);
B_d = Ns(1:nx, nx+1:end);

A = A_d;
B = B_d;

%% MPC setup
N = 25;             % Horizon with k=1,..N (note, starts at 1 as opposed to 0)
Q = 100*eye(nx);    % Weights on states
Q(end,end) = 1;
Q(end-1,end-1) = 1;
R = 1*eye(nu);      % Weights on inputs
% P is the terminal cost. It is the solution of the ricatti equations obtained by [~,P] =
% dlqr(A,B,Q,R) and trunctuating
P = [   3.1737e+04,  4.6682e-10,  1.8752e+03,  8.7111e-12, -9.3106e+03,  2.0915e+03,  1.0000e+03,  1.3242e-11;
        4.6682e-10,  3.9632e+04, -6.2554e-12,  9.7031e+02, -1.8209e-10, -2.2602e-11,  2.4313e-11,  1.0000e+03;
        1.8752e+03, -6.2554e-12,  1.6687e+04,  6.4560e-12, -2.7441e+03,  2.4835e+04, -5.9160e+02, -1.5141e-11;
        8.7111e-12,  9.7031e+02,  6.4560e-12,  2.0401e+02, -8.4896e-12,  9.3342e-12,  5.2174e-13,  2.8654e+01;
       -9.3106e+03, -1.8209e-10, -2.7441e+03, -8.4896e-12,  2.5723e+05, -6.6525e+02, -1.3680e+03,  1.2303e-10;
        2.0915e+03, -2.2602e-11,  2.4835e+04,  9.3342e-12, -6.6525e+02,  3.7426e+04, -9.3086e+02, -2.1550e-11;
        1.0000e+03,  2.4313e-11, -5.9160e+02,  5.2174e-13, -1.3680e+03, -9.3086e+02,  1.7879e+02,  9.2775e-13;
        1.3242e-11,  1.0000e+03, -1.5141e-11,  2.8654e+01,  1.2303e-10, -2.1550e-11,  9.2775e-13,  1.3445e+02];

% bounds
umin = -100*ones(2,1); umax = 100*ones(2,1);
xmin = -100*ones(8,1); xmax = 100*ones(8,1);

% costs matrices for k = 1..N-1
H = 2*blkdiag(Q,R);
f = [-(xref'*Q + xref'*Q'), -(uref'*R + uref'*R')]';

% terminal cost k = N
H_N = 2*P;
f_N = -(xref'*P + xref'*P')';