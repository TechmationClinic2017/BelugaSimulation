% Iver2 parameters
m = 0; % mass of AUV

% Gravity
xg = 0;
zg = 0;

% Moments of Inertia
Ix = 0;
Iy = 0;
Iz = 0;

X_udot = 0;
Y_vdot = 0;
Z_wdot = 0;

Y_rdot = 0;
Z_qdot = 0;

K_pdot = 0;
M_wdot = 0;

M_qdot = 0;
N_vdot = 0;

N_rdot = 0;

% Run simulation
simOut = sim('iver_model.slx');