% Iver2 parameters
m = 4.2173; % mass of AUV

% Gravity
xg = 0;
zg = 0;

% Moments of Inertia
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
[simOut, y, z, q] = sim('beluga_model.slx');