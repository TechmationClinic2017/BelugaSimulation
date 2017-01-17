function newstate = beluga_dynamic_model( state, u, disturbance, Ts )
%SURFACE_VEHICLE_DYNAMIC_MODEL 
% Author: Vaibhav Viswanathan (vviswanathan@hmc.edu)
%             (email me for any questions!)
%                    ___
%  current state -> |   |
%                   | H | -> new state
% control inputs -> |___|

integration_steps = 5; %increase this number if you find the 
                       %simulator go into numerical instability

%% CONSTANTS
m = 4.2173; % mass of Beluga

% Gravity Beluga
xg = 0; 
zg = 0;

% Moments of Inertia Beluga
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

mass_matrix = [m-X_udot 0 0 0 m*zg 0;
               0 m-Y_vdot 0 -m*zg 0 m*xg-Y_rdot; 
               0 0 m-Z_wdot 0 -m*xg-Z_qdot 0; 
               0 -m*zg 0 Ix-K_pdot 0 0; 
               m*zg 0 -m*xg-M_wdot 0 Iy-M_qdot 0; 
               0 m*xg-N_vdot 0 0 0 Iz-N_rdot];

%% STATE
x       = state(1);
y       = state(2);
z       = state(3);
phi     = state(4);
theta   = state(5);
psi     = state(6);

x_d   = state(1);
y_d   = state(2);
z_d   = state(3);
phi_d = state(4);
theta_d = state(5);
psi_d = state(6);

x_disturbance     = disturbance(1);
y_disturbance     = disturbance(2);
theta_disturbance = disturbance(3);

%% CALCULATE FORCES TODO

F = ones(6,1); 

%% COMPUTE NEW STATE

% Convert forces into accelerations
accel = (mass_matrix)\F;
state_d = [state(1:6), accel'];

newstate = state(1:12);

for j=1:integration_steps
    
    Ts_int = Ts/integration_steps;

    % update state variables by integrating
    newstate = newstate + state_d*Ts_int;

end

% convert to global frame
cosphi = cos(phi);
sinphi = sin(phi);
costheta = cos(theta);
sintheta = sin(theta);
cospsi = cos(psi);
sinpsi = sin(psi);

R = zeros(6,6);
R(1,1) = cospsi.*cosphi-costheta.*sinphi.*sinpsi;
R(1,2) = sinphi.*cospsi+costheta.*cosphi.*sinpsi;
R(1,3) = sintheta.*sinpsi;

R(2,1) = -cosphi.*sinpsi-costheta.*cospsi.*sinphi;
R(2,2) = -sinpsi.*sinphi+costheta.*cosphi.*cospsi;
R(2,3) = sintheta.*cospsi;

R(3,1) = sinphi.*sintheta;
R(3,2) = -cosphi.*sintheta;
R(3,3) = costheta;

R(4,4) = 1;
R(5,5) = 1;
R(6,6) = 1;

%Rotate to global frame
newstate = [(R*newstate(1:6)')' newstate(7:12)];

newstate = [newstate, u(1), u(2), u(3), u(4)];

end

