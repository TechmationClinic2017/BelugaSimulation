%% Calculate PD constants for yaw using phase margin

% Set desired crossover and phase
omega0 = 1;
phi_m = pi/3;

[Kp, Kd] = phasePD(phi_m, omega0)
