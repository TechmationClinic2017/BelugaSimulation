function newstate = surface_vehicle_dynamic_model( state, u, disturbance, Ts )
%SURFACE_VEHICLE_DYNAMIC_MODEL 
% Author: Benjamin Chasnov (bchasnov@gmail.com)
%             (email me for any questions!)
%                    ___
%  current state -> |   |
%                   | H | -> new state
% control inputs -> |___|

integration_steps = 5; %increase this number if you find the 
                       %simulator go into numerical instability

% constants
m = 1; % mass 1kg
I = 1;
k1 = 10; % 1/2*p*Cd*A; drag coefficient
k2 = 5; % rotational drag
r = 0.050; % thrusters r meters from the center of mass axis
a = 0.050; % center of drag a meters from center of mass along axis
max_thrust = 500; % Newtons

x       = state(1);
y       = state(2);
theta   = state(3);
x_d     = state(4);
y_d     = state(5);
theta_d = state(6);

x_disturbance     = disturbance(1);
y_disturbance     = disturbance(2);
theta_disturbance = disturbance(3);

Fl = max_thrust*u(1);
Fr = max_thrust*u(2);

for j=1:integration_steps
    
    Ts_int = Ts/integration_steps;
    % calculate velocity vector angle and magnitude
    phi = atan2(y_d, x_d); 
    vel = sqrt(x_d^2+y_d^2);

    % balance forces and moments
    x_dd = 1/m*(Fl+Fr)*cos(theta)-k1*vel^2*cos(phi) + x_disturbance;
    y_dd = 1/m*(Fl+Fr)*sin(theta)-k1*vel^2*sin(phi) + y_disturbance;
    theta_dd = 1/I* ((Fr-Fl)*r - k1*vel^2*a*sin(theta-phi) - k2*theta_d*abs(theta_d)) ...
        + theta_disturbance;

    % update state variables by integrating
    x_d = x_d + x_dd*Ts_int;
    y_d = y_d + y_dd*Ts_int;
    theta_d = theta_d + theta_dd*Ts_int;
    x = x + x_d*Ts_int;
    y = y + y_d*Ts_int;
    theta = theta + theta_d*Ts_int;
    
    theta = mod(theta, 2*pi);
end

newstate = [x, y, theta, x_d, y_d, theta_d,...
    u(1), u(2), x_disturbance, y_disturbance, theta_disturbance];

end

