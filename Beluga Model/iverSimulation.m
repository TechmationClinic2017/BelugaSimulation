% AUV parameters
m = 0; % mass of AUV
X_udot = 0;
zg = 0;
Y_vdot = 0;
xg = 0;
Y_rdot = 0;
Z_wdot = 0;
Z_qdot = 0;
Ix = 0;
K_pdot = 0;
M_wdot = 0;
Iy = 0;
M_qdot = 0;
N_vdot = 0;
Iz = 0;
N_rdot = 0;

% Run simulation
simOut = sim('iver_model.slx');