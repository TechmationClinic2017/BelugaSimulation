% Load Data
load('forward050_2017-02-10-08-29-38_vel.mat');
u = 25;

% compute linear velocity
v_lin = sqrt(vx_step.^2+vy_step.^2);

% Compute first derivative
dv = diff(v_lin);
dt = t_step(1:end-1)+0.5*diff(t_step);

% Compute second derivative
d2v = diff(dv);
dt2 = dt(1:end-1)+0.5*diff(dt);

% Plot all 3
plot(t_step, v_lin, dt, dv, dt2, d2v);
xlabel('Time (s)');
ylabel('Linear Velocity (m/s)');
title('Step Response (u=0.5)');


% Compute steady-state velocity
% assume the system is underdamped and any "overshoot" is due to IMU drift
% TODO FIX
vss = max(v_lin);

% determine inflection point of rise
% assume inflection point is halfway
ind = ceil(find(vss == v_lin)/3); %TODO FIX CALCULATION
inf_t = t_step(ind);

% compute slope at inflection point
slope = (v_lin(ind+1)-v_lin(ind-1))/(t_step(ind+1)-t_step(ind-1));

% compute linear equation for inflection point slope
A = v_lin(ind)-slope*t_step(ind);
y = slope*t_step + A;

% Compute parameters for system step-response model
Ks = vss;
T = interp1(y, t_step, Ks);
L = interp1(y, t_step, 0);


% Compute PID gains
Kp = 1.2/A
Ti = 2*L
Td = L/2
Tp = 3.4*L

% Plot everything
plot(t_step, v_lin,t_step, y);
xlabel('Time (s)');
ylabel('Linear Velocity (m/s)');
title('Step Response (u=0.5)');
ylim([min([y; v_lin])*1.1 max(v_lin)*1.1]);

