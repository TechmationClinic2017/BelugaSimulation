% Forward velocity spline fit - 3rd order

% parameters
vo = 0;
vf = 0.5; %m/s
dt = 5; %s

to = 0;
tf = 0 + dt;

ao = 0;
af = 0;

% Compute spline fit parameters
A =     [   to^3,   to^2,   to, 1;
            tf^3,   tf^2,   tf, 1;
            3*to^2, 2*to,   1, 0;
            3*tf^2, 2*tf,   1, 0 ];
B = [vo; vf; ao; af];

[X, R] = linsolve(A,B);

% Plot spline fit velocity
t = linspace(to, tf);
v = X(1)*t.^3 + X(2)*t.^2 + X(3)*t + X(4);
a = 3*X(1)*t.^2 + 2*X(2)*t + X(3);

plot(t, v, t, a)
title('Forward Vel Spline Fit')
xlabel('Time (s)')
legend('Axial velocity (m/s)', 'Axial Acceleration (m/s^2)', 'location', 'best')
