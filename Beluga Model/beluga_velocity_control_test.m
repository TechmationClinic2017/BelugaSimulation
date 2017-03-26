clear all
%constants
A = 1.06;
tau = 1.75;
tau_d = 0.09;
Kp = 20.2;
Ki = 3.77;
Kd = 0.3;
desired_velocity = 0.5;



% Run simulation
tstop=10;
[t,x,y]=sim('beluga_controller',tstop);
subplot(2,1,1)
plot(t,y(:,1))
title('Step response (axial)')
ylabel('forward  velocity u (m/s)')
subplot(2,1,2)
plot(t,y(:,2))
ylabel('axial distance x (m)')
xlabel('time (s)')
