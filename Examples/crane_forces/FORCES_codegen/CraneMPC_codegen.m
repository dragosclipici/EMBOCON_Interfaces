clear all;
close all;

addpath('~/Documents/FORCES');

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

A = A_d;
B = B_d;


%% MPC setup
N = 25;
Q = 100*eye(nx);
Q(end,end) = 1;
Q(end-1,end-1) = 1;
R = 1*eye(nu);
[~,P] = dlqr(A,B,Q,R);
% [~,P] = lqr(A,B,Q,R);
umin = -100*ones(2,1); umax = 100*ones(2,1);
xmin = -100*ones(8,1); xmax = 100*ones(8,1);   

%% FORCES multistage form - zi = [xi, ui] for i=1...N-1 and zN = xN

params(1) = newParam('H',1:N-1,'cost.H');
params(end+1) = newParam('f',1:N-1,'cost.f');
params(end+1) = newParam('H_N', N, 'cost.H');
params(end+1) = newParam('f_N', N, 'cost.f');

stages = MultistageProblem(N);

for i = 1:N
    % initial stage
    if( i==1 )
        
        % dimension
        stages(i).dims.n = nx+nu; % number of stage variables
        stages(i).dims.r = 2*nx;  % number of equality constraints        
        stages(i).dims.l = nx+nu; % number of lower bounds
        stages(i).dims.u = nx+nu; % number of upper bounds
        stages(i).dims.p = 0;     % number of polytopic constraints
        stages(i).dims.q = 0;     % number of quadratic constraints
        
        % cost
%         stages(i).cost.H = 2*blkdiag(Q,R);
%         stages(i).cost.f = [-(xref'*Q + xref'*Q'), -(uref'*R + uref'*R') ]';
%         stages(i).cost.f = zeros(stages(i).dims.n,1);
        
        % lower bounds
        stages(i).ineq.b.lbidx = 1:stages(i).dims.n; % lower bound acts on these indices
        stages(i).ineq.b.lb = [xmin; umin]; % lower bound for this stage variable
        
        % upper bounds
        stages(i).ineq.b.ubidx = 1:stages(i).dims.n; % upper bound acts on these indices
        stages(i).ineq.b.ub = [xmax; umax]; % upper bound for this stage variable
        
        % equality constraints
        stages(i).eq.C = [eye(nx), zeros(nx,nu); A, B];
        params(end+1) = newParam('z1',1,'eq.c'); % RHS of first eq. constr. is a parameter: [x0, 0]
        
    end
    
    % stages along horizon
    if( i>1 && i<N )       
        
        % dimension
        stages(i).dims.n = nx+nu; % number of stage variables
        stages(i).dims.r = nx;    % number of equality constraints        
        stages(i).dims.l = nx+nu; % number of lower bounds
        stages(i).dims.u = nx+nu; % number of upper bounds
        stages(i).dims.p = 0;     % number of polytopic constraints
        stages(i).dims.q = 0;     % number of quadratic constraints
        
        % cost
%         stages(i).cost.H = 2*blkdiag(Q,R);
%         stages(i).cost.f = [-(xref'*Q + xref'*Q'), -(uref'*R + uref'*R')]';
%         stages(i).cost.f = zeros(stages(i).dims.n,1);
        
        % lower bounds
        stages(i).ineq.b.lbidx = 1:stages(i).dims.n; % lower bound acts on these indices
        stages(i).ineq.b.lb = [xmin; umin]; % lower bound for this stage variable
        
        % upper bounds
        stages(i).ineq.b.ubidx = 1:stages(i).dims.n; % upper bound acts on these indices
        stages(i).ineq.b.ub = [xmax; umax]; % upper bound for this stage variable
        
        % equality constraints
        stages(i).eq.C = [A, B];
        stages(i).eq.c = zeros(nx,1);
        if( i==2 )
            stages(i).eq.D = [zeros(nx,nx+nu); -eye(nx), zeros(nx,nu)];
        else
            stages(i).eq.D = [-eye(nx), zeros(nx,nu)];
        end
        
    end
    
    % final stage
    if( i==N )
        
        % dimension
        stages(i).dims.n = nx;    % number of stage variables
        stages(i).dims.r = 0;     % number of equality constraints        
        stages(i).dims.l = nx;    % number of lower bounds
        stages(i).dims.u = nx;    % number of upper bounds
        stages(i).dims.p = 0;     % number of polytopic constraints
        stages(i).dims.q = 0;     % number of quadratic constraints
        
        % cost
%         stages(i).cost.H = 2*P;
%         stages(i).cost.f = -(xref'*P + xref'*P')';
%         stages(i).cost.f = zeros(stages(i).dims.n,1);
        
        % lower bounds
        stages(i).ineq.b.lbidx = 1:stages(i).dims.n; % lower bound acts on these indices
        stages(i).ineq.b.lb = xmin; % lower bound for this stage variable
        
        % upper bounds
        stages(i).ineq.b.ubidx = 1:stages(i).dims.n; % upper bound acts on these indices
        stages(i).ineq.b.ub = xmax; % upper bound for this stage variable
        
        % equality constraints        
        stages(i).eq.D = -eye(nx);
        
    end
end

%% define outputs of the solver
outputs = newOutput('u1',1,nx+1:nx+nu);

%% solver settings
codeoptions = getOptions('myMPC');

%% generate code
generateCode(stages,params,codeoptions,outputs);


%% simulate
% x1 = [0;0.75;zeros(6,1)];
x1 = xref;
kmax = 10/ts;
X = zeros(nx,kmax+1); X(:,1) = x1;
U = zeros(nu,kmax);
problem.z1 = zeros(2*nx,1);
for k = 1:kmax
    problem.z1(1:nx) = X(:,k);
    problem.H = 2*blkdiag(Q,R);
    problem.f = [-(xref'*Q + xref'*Q'), -(uref'*R + uref'*R')]';
    problem.H_N = 2*P;
    problem.f_N = -(xref'*P + xref'*P')';
    [solverout,exitflag,info] = myMPC(problem);
    if( exitflag == 1 )
        U(:,k) = solverout.u1;
    else
        info
        error('Some problem in solver');
    end
    X(:,k+1) = A*(X(:,k) - xref) + B*(U(:,k) - uref) + xref;
end

%% plot
figure(1); clf;
subplot(2,1,1); grid on; title('states'); hold on;
% plot([1 kmax], [xmax xmax]', 'r--'); plot([1 kmax], [xmin xmin]', 'r--');
% ylim(1.1*[min(xmin),max(xmax)]); 
stairs(1:kmax,X(:,1:kmax)');
subplot(2,1,2);  grid on; title('input'); hold on;
% plot([1 kmax], [umax umax]', 'r--'); plot([1 kmax], [umin umin]', 'r--');
% ylim(1.1*[min(umin),max(umax)]); 
stairs(1:kmax,U(:,1:kmax)');