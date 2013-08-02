%% system
nx = 8;
nu = 2;

ts = 0.01;
xref = [1;1.5;zeros(6,1)];
uref = zeros(nu,1);

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
x_L   = 1.5;  %cable length (m)
v_C   = 0;     %cart velocity (m/s)
v_L   = 0;     %cable velocity (m/s)
theta = 0;     %cable deflection (rad)
omega = 0;  %cable deflection rate (rad/s)
u_C   = 0;     %cart controller voltage (V)
u_L   = 0;     %cable controller voltage (V)

xlin = [x_C; x_L; v_C; v_C; theta; omega; u_C; u_L];

% STIFF MODEL
%w=omega_dot
dwdx_L = (2*omega/x_L *c/m + 2*v_L*omega + g*sin(theta) + (A_C*u_C-v_C)/tau_C*cos(theta)) / x_L^2;
dwdv_C = cos(theta)/(tau_C*x_L);
dwdv_L = -2*omega/x_L;
dwdtheta = ( (A_C*u_C-v_C)*sin(theta)/tau_C - g*cos(theta) )/x_L;
dwdomega = -2*v_L/x_L;
dwdu_C = -A_C*cos(theta)/(tau_C*x_L);
%      x_C     x_L     v_C     v_L    theta   omega    u_C     u_L
A_c = [ 0       0       1       0       0       0       0       0;
        0       0       0       1       0       0       0       0;
        0       0   -1/tau_C    0       0       0   A_C/tau_C   0;
        0       0       0    -1/tau_L   0       0       0    A_L/tau_L;
        0       0       0       0       0       1       0       0;
        0     dwdx_L dwdv_C   dwdv_L dwdtheta dwdomega dwdu_C   0;
        0       0       0       0       0       0       0       0;
        0       0       0       0       0       0       0       0];
        
%      u_CR    u_LR
B_c = [   0       0;  % x_C
        0       0;  % x_L
        0       0;  % v_C
        0       0;  % v_L
        0       0;  % theta
        0       0;  % omega
        1       0;  % u_CR
        0       1]; % u_LR

    
% % REDUCED MODEL
% %w=omega_dot
% u_CR = 0;
% 
% dwdx_L = (A_C*u_CR*cos(theta) + g*sin(theta) + 2*v_L*omega) / x_L^2;
% dwdv_L = -2*omega/x_L;
% dwdtheta = (A_C*u_CR*sin(theta) - g*cos(theta))/x_L;
% dwdomega = -2*v_L/x_L;
% 
% %      x_C     x_L     v_C     v_L    theta   omega    u_C     u_L
% A_c = [   0       0       1       0       0       0       0       0;
%         0       0       0       1       0       0       0       0;
%         0       0       0       0       0       0       0       0;
%         0       0       0    -1/tau_L   0       0       0    A_L/tau_L;
%         0       0       0       0       0       1       0       0;
%         0     dwdx_L    0     dwdv_L dwdtheta dwdomega  0       0;
%         0       0       0       0       0       0       0       0;
%         0       0       0       0       0       0       0       0];
%         
% dwdu_CR = -A_C*cos(theta)/x_L;
% %      u_CR    u_LR
% B_c = [   0       0;  % x_C
%         0       0;  % x_L
%        A_C      0;  % v_C
%         0       0;  % v_L
%         0       0;  % theta
%      dwdu_CR    0;  % omega
%         1       0;  % u_CR
%         0       1]; % u_LR
 

M = [A_c*ts B_c*ts; zeros(nu,nx+nu)];
Ns = expm(M);
A_d = Ns(1:nx,1:nx);
B_d = Ns(1:nx, nx+1:end);


%% MPC setup
N = 2;
Q = 100*eye(nx);
Q(end,end) = 1;
Q(end-1,end-1) = 1;
R = 1*eye(nu);
%[~,P] = dlqr(A,B,Q,R);
umin = -100*ones(2,1); umax = 100*ones(2,1);
xmin = -100*ones(8,1); xmax = 100*ones(8,1);   

%% MPT setup

% create model
model = LTISystem('A',A_d,'B',B_d,'Ts',ts);
model.x.min = -100*ones(nx,1);
model.x.max = 100*ones(nx,1);
model.u.min = -5*ones(nu,1);
model.u.max = 5*ones(nu,1);

% add reference signals
model.x.with('reference');
model.x.reference = xref;
model.u.with('reference');
model.u.reference = uref;

% penalties
model.x.penalty = Penalty(Q,2);
model.u.penalty = Penalty(R,2);

% terminal penalty
model.x.with('terminalPenalty');
model.x.terminalPenalty = model.LQRPenalty;

% create online controller
ctrl = MPCController(model, N);

% generate explicit controller if it does not exist
if exist('ectrl.mat','file')~=2
    % create explicit controller
    ectrl = ctrl.toExplicit;

    % save the explicit controller
    save ectrl ectrl
else
    % if explicit controller exists, load it from the saved data
    if exist('ectrl','var')~=1
        load ectrl
    end
end

% generate C-code
ectrl.exportToC;


%% Simulate the closed loop
N_sim = 10/ts;

% Create the closed-loop system with the model used in prediction
loop = ClosedLoop(ctrl, model);

% simulate
x0 = [0; 0.75; 0; 0; 0; pi/4; 0; 0];

data = loop.simulate(x0, N_sim);


% plot the output trajectories
figure
plot(0:N_sim,data.X,'LineWidth',2)
xlabel('Simulation steps')
title('State trajectories')

% plot inputs
figure
stairs(0:N_sim-1,data.U','LineWidth',2)
xlabel('Simulation steps')
title('Control inputs')