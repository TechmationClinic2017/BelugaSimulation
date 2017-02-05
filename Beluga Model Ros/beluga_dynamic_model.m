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
m = 2; % mass of Beluga

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
x_d   = state(1);
y_d   = state(2);
z_d   = state(3);
phi_d = state(4);
theta_d = state(5);
psi_d = state(6);

x       = state(7);
y       = state(8);
z       = state(9);
phi     = state(10);
theta   = state(11);
psi     = state(12);

x_disturbance     = disturbance(1);
y_disturbance     = disturbance(2);
theta_disturbance = disturbance(3);

%% CALCULATE FORCES TODO

F = 10*ones(6,1); 

%% COMPUTE NEW STATE

% Convert forces into accelerations
accel = ((mass_matrix)\F)';

% Integrate acceleration to velocity
new_vel = state(1:6);
Ts_int = Ts/integration_steps;

for j=1:integration_steps
    
    % update state variables by integrating
    new_vel = new_vel(1:6) + accel*Ts_int;

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
global_vel = (R*new_vel')';

% Integrate velocity to position
new_pos = state(7:12);
Ts_int = Ts/integration_steps;

for j=1:integration_steps
    
    % update state variables by integrating
    new_pos = new_pos + global_vel*Ts_int;

end

newstate = [new_vel, new_pos, u(1), u(2), u(3), u(4)];

end

function F = f_control(state, inputs)
    
    %% control forces
    
    %% Thruster Model
    % Values from documentation stationary calibration
    thruster_pwm = [1100        1110        1120        1130        1140        1150        1160        1170 1180        1190        1200        1210        1220        1230        1240        1250        1260        1270        1280        1290        1300        1310        1320        1330        1340        1350        1360        1370        1380        1390        1400        1410         1420        1430        1440        1450        1460        1470        1480        1490 1500        1510        1520        1530        1540        1550        1560        1570 1580        1590        1600        1610        1620        1630        1640        1650 1660        1670        1680        1690        1700        1710        1720        1730 1740        1750        1760        1770        1780        1790        1800        1810 1820        1830        1840        1850        1860        1870        1880        1890 1900];
    thruster_force = 9.81*[-4.0823   -4.0823   -4.0869   -3.8283   -3.7331   -3.6197   -3.4428   -3.3022   -3.1842   -3.0572 -2.9075   -2.8168   -2.7079   -2.5265   -2.3632   -2.2589   -2.1772   -1.9913   -1.8824   -1.7690 -1.6420   -1.4923   -1.3336   -1.2428   -1.1204   -1.0251   -0.9072   -0.7893   -0.6940   -0.6169 -0.5216   -0.4354   -0.3538   -0.2767   -0.1905   -0.1134   -0.0544   -0.0272         0         0 0         0         0    0.0590    0.1134    0.1860    0.2631    0.3493    0.4173    0.5216 0.6350    0.7167    0.8437    0.9435    1.1022    1.2474    1.3925    1.5286    1.6783    1.8053 1.9595    2.0956    2.2181    2.4131    2.5220    2.6943    2.8259    3.0209    3.1933    3.3521 3.4473    3.4927    3.7784    3.9417    4.0052    4.1957    4.3772    4.5677    4.8035    4.8988 5.0938];
    
    
    % From Arduino implementation
    DEAD_POS = 16;
    DEAD_NEG = -35;
    PERIOD = 500; % [ms]
    LEAK_THRESH = 150;
    
    %Correct for deadzone
    for i = 1:numel(inputs)
        if inputs(i) > 0
            inputs(i) = inputs(i)*50+DEAD_POS;
        elseif inputs(i) < 0
            inputs(i) = inputs(i)*50+DEAD_NEG;            
        end
        
        inputs(i) = inputs(i) + 1500;
        inputs(i) = interp1(thruster_pwm, thruster_force, inputs(i)); % assuming static thrust
    end


    %% States and calculations
    % defining states
    u = state(1);
    v = state(2);
    w = state(3);
    p = state(4);
    q = state(5);
    r = state(6);
    
    x = state(7);
    y = state(8);
    z = state(9);
    phi = state(10);
    theta = state(11);
    psi = state(12);
    
    %approximate dynamic thrust
    d = 0.076; %m
    rho = 1000; %kg/m^3
    for i = 1:numel(inputs)
        if inputs(i)==0
            inputs(i) = 0.0001;
        end
        n = sqrt(abs(inputs(i))/(rho*d^4*0.1858));
        Kt = 0.1858*(1-(u/(n*d)));
        Kt(isnan(Kt))=0;
        inputs(i) = real(Kt*rho*n.^2*d^4);
    end
    
    % Control Forces
    X_c = sum(inputs); 
    Y_c = 0;
    Z_c = 0;
    K_c = inputs(1)-inputs(2)+inputs(3)-inputs(4); %TODO CHECK CONSTANTS AND POLARITY
    M_c = inputs(2)-inputs(4); %TODO CHECK
    N_c = inputs(1)-inputs(3); %TODO CHECK POLARITY
    
    % output force vector
    X = X_c;
    Y = Y_c;
    Z = Z_c;
    K = K_c;
    M = M_c;
    N = N_c;
    F = [X Y Z K M N]'; 
    %F = [0 0 0 0 0 0]';
end

