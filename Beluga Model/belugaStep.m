% Beluga parameters


m = 4.2173; % mass of IVER

% Gravity IVER
xg = 0; 
zg = 0;

% Moments of Inertia IVER
Ix = 0.05;
Iy = 2.05;
Iz = 2.06;

X_udot = -0.51334;
Y_vdot = -23.8314;
Z_wdot = -23.8314;

Y_rdot = -2.5656;
Z_qdot = 2.5656;

K_pdot = -0.0704;
M_wdot = 2.5656;

M_qdot = -3.2455;
N_vdot = -2.5656;

N_rdot = -3.2455;

% Run simulation
numSteps = 100;
timeStep = 0.1;
t = (timeStep*(0:numSteps))';
u = [ones(numel(t),1), ones(numel(t),1), ones(numel(t),1), ones(numel(t),1)];
[simOut, y] = sim('beluga_model.slx');


% Plot
figure(1)
subplot(2,1,1)
plot(simOut, y(:,1))
xlabel('Time (s)')
ylabel('X Velocity (m/s)')
title('Beluga Step Response: X Velocity vs. Time')

subplot(2,1,2)
plot(simOut, y(:,7))
xlabel('Time (s)')
ylabel('X Position (m)')
title('Beluga Step Response: X Position vs. Time')

