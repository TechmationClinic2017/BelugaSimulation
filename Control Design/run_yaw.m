clear all
close all

% set system constants
tdyaw = 0.11;
B = 0.55;

% set PID constants
Kp = 2.17;
Kd = 3.47;

% setpoint
r_set = pi/2;

%run simulation
[simout, y, z] = sim('beluga1_model_yaw.slx');

%Plot stuff
ref = ones(size(simout,1),1)*r_set;
plot(simout, z, simout, ref)    
xlabel('Time (s)');
ylabel('Angle (rads)');
title('Yaw Control(\pi/2)');