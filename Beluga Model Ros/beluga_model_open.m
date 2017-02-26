%% Simulates the Beluga AUV with linear velocity and theta (yaw) setpoints
% 
% Author: Vai Viswanathan
% (vviswanathan@hmc.edu)
%             (email me for any questions!)
% Date: Dec 2016
%

clear all
close all
animationspeed = 4; % 4 gives approximately real time
Tend = 6;   % run simulation for Tend seconds. must be multiple of 3

Ts = 0.01; % Control loop at 100 Hz
state = [0 0 0 0 0 0 ...
         0 0 0 0 0 0 ...
         0 0 0 0]; 
% state = [x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot,
%          x, y, z, phi, theta, psi, 
%          u(1), u(2), u(3), u(4)];

N = Tend/Ts; % number of cycles
time = linspace(0,Tend,N);

% initialize states, control and disturbances inputs to zero
states = zeros(N, length(state));
u = ones(N,4);
disturbance = zeros(N,6);
%disturbance(:,3) = 9*ones(N,1);
disturbance = 1*(rand([N 3])-0.5);

% set points
%theta_desired = sin([1:N]/100)*pi/2+2;
%vel_desired = [0.3*ones(N/3,1); 0.8*ones(N/3,1); 1.2*ones(N/3,1)];
pos_des = [[1,1,0];
           [2,2,0]
           [3,1,0]];
point_num = 1;

pid_save = zeros(N,3);

figure(1);

%% Main loop
for i=1:N
    
    %% Run model.
    state = beluga_dynamic_model(state,u(i,:),disturbance(i,:),Ts);
    states(i,:) = state;
    
    if mod(i,animationspeed) == 1
        beluga_draw(states,i,Ts,1);
        drawnow;
    end
end

%% Plot
figure(2);

subplot(5,1,1);
plot(time, states(:,13)); hold on;
plot(time, states(:,14)); hold on;
plot(time, states(:,15)); hold on;
plot(time, states(:,16)); hold off;
legend({'left motor','right motor'});
title('Robot Thrust');

subplot(5,1,2);
plot(time, states(:,1)); hold on;
plot(time, states(:,2)); hold on;
plot(time, states(:, 3)); hold off;
legend({'u','v', 'w'});

subplot(5,1,3);
plot(time, states(:,4)); hold on;
plot(time, states(:,5)); 
plot(time, states(:,6)); 
hold off;
legend({'p','q','r'});

subplot(5,1,4);
plot(time, states(:,7)); hold on;
plot(time, states(:,8)); hold on;
plot(time, states(:,9)); hold off;
legend({'x','y','z'});

subplot(5,1,5);
plot(time, states(:,10)); hold on;
plot(time, states(:,11)); hold on;
plot(time, states(:,12)); hold off;
ylim([-3 3]);
legend('\phi','\theta','\psi');

xlabel('time (s)');