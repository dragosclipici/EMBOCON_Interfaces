% plot the simulated data with explicit controller

if exist('states','var')~=1 || exist('inputs','var')~=1
    error('You must run the simulation "crane_embocon_sim.mdl" first.');
end

% plot the states
figure
hold on
grid on
plot(states.x_C.Time,states.x_C.Data,'r','LineWidth',2)
plot(states.x_L.Time,states.x_L.Data,'g','LineWidth',2)
plot(states.v_C.Time,states.v_C.Data,'b','LineWidth',2)
plot(states.v_L.Time,states.v_L.Data,'m','LineWidth',2)
plot(states.theta.Time,states.theta.Data,'y','LineWidth',2)
plot(states.omega.Time,states.omega.Data,'c','LineWidth',2)
plot(states.u_C.Time,states.u_C.Data,'r--','LineWidth',2)
plot(states.u_L.Time,states.u_L.Data,'g--','LineWidth',2)

hold off


% plot the inputs
figure
hold on
grid on
plot(inputs.u_CR.Time,inputs.u_CR.Data,'r','LineWidth',2)
plot(inputs.u_LR.Time,inputs.u_LR.Data,'g','LineWidth',2)

hold off