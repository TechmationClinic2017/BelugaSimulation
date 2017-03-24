clear all
close all

% set system constants
tdyaw = 0.11;
B = 0.55;

% Set desired crossover and phase
omegas = [0.5, 1, 2, 4];
phi_m = pi/4;

% setpoint
r_set = pi/2;

for omega0 = omegas
   
    [Kp, Kd] = phasePD(phi_m, omega0)

    %run simulation
    [simout, y, z] = sim('beluga1_model_yaw.slx');

    %Plot stuff
    hold on
    plot(simout, z)    
end

ref = ones(size(simout,1),1)*r_set;
plot(simout, ref); 
xlabel('Time (s)');
ylabel('Angle (rads)');
title('Yaw Control(\pi/2)');
legend(omegas);