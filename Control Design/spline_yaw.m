% Yaw spline fit - 5th order

% parameters
phi_o = 0;
phi_f = pi/2; %radians 
dt = 10; %s

to = 0;
tf = 0 + dt;

ro = 0;
rf = 0;
ao = 0;
af = 0;

% Compute spline fit parameters
A =     [   to^5,   to^4,  to^3,   to^2,   to, 1;
            tf^5,   tf^4,  tf^3,   tf^2,   tf, 1;
            5*to^4, 4*to^3,  3*to^2, 2*to,   1, 0;
            5*tf^4, 4*tf^3,  3*tf^2, 2*tf,   1, 0;
            20*to^3,  12*to^2, 6*to,   2, 0, 0;
            20*tf^3,  12*tf^2, 6*tf,   2, 0, 0];
B = [phi_o; phi_f; ro; rf; ao; af];

[X, R] = linsolve(A,B);

% Plot spline fit velocity
t = linspace(to, tf);
phi = X(1)*t.^5 + X(2)*t.^4 + X(3)*t.^3 + X(4)*t.^2 + X(5)*t + X(6);
r = 5*X(1)*t.^4 + 4*X(2)*t.^3 + 3*X(3)*t.^2 + 2*X(4)*t + X(5);
a = 20*X(1)*t.^3 + 12*X(2)*t.^2 + 6*X(3)*t + 2*X(4);

plot(t, phi, t, r, t, a)
title('Yaw Spline Fit')
xlabel('Time (s)')
legend('Yaw (rad)', 'Angular velocity (rad/s)', 'Angular Acceleration (rad/s^2)', 'location', 'best')
