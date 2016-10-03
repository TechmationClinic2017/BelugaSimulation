function [ u, v, w, p, q, r, x, y, z, phi, theta, psi ] = iverModel( rudder, stern )
%IVERMODEL Iver Model in Code
%   a code implementation of the iver model

% The Mass Matrix constant
% Iver2 parameters
m = 5; % mass of AUV

% Gravity
xg = 1;
zg = -1;

% Moments of Inertia
Ix = 1;
Iy = 1;
Iz = 1;

X_udot = 1;
Y_vdot = 1;
Z_wdot = 1;

Y_rdot = 1;
Z_qdot = 1;

K_pdot = 1;
M_wdot = 1;

M_qdot = 1;
N_vdot = 1;

N_rdot = -1;

Mass_Matrix = [m-X_udot 0 0 0 m*zg 0; 
    0 m-Y_vdot 0 -m*zg 0 m*xg-Y_rdot; 
    0 0 m-Z_wdot 0 -m*xg-Z_qdot 0; 
    0 -m*zg 0 Ix-K_pdot 0 0; 
    m*zg 0 -m*xg-M_wdot 0 Iy-M_qdot 0;
    0 m*xg-N_vdot 0 0 0 Iz-N_rdot];

end

