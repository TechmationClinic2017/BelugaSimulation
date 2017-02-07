% linearize the Beluga dynamic model

% define constants
m = 20.2; %kg
Ix = 

%define mass matrix
syms m Ix Iy Iz xg zg X_udot Y_vdot Y_rdot Z_wdot Z_qdot K_pdot M_wdot M_qdot N_vdot N_rdot
M = [   m-X_udot    0           0               0           m*zg            0; 
        0           m-Y_vdot    0               -m*zg       0               m*xg-Y_rdot; 
        0           0           m-Z_wdot        0           -m*xg-Z_qdot    0; 
        0           -m*zg       0               Ix-K_pdot   0               0; 
        m*zg        0           -m*xg-M_wdot    0           Iy-M_qdot       0; 
        0           m*xg-N_vdot 0               0           0               Iz-N_rdot];


% define force vector
syms f1 f2 f3 f4 f5 f6
F = [f1 f2 f3 f4 f5 f6];

% determine acceleration vector
a = simplify(inv(M))*transpose(F);


