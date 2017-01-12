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
Tend = 60;   % run simulation for Tend seconds. must be multiple of 3

Ts = 0.01; % Control loop at 100 Hz
state = [0 0 0 0 0 0 ...
         0 0 0 0 0 0 ...
         0 0 0 0]; 
% state = [x, y, z, phi, theta, psi, 
%          x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot,
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
plot(time, states(:,7)); hold on;
plot(time, states(:,8)); hold off;
legend({'left motor','right motor'});
title('Robot state');

subplot(5,1,2);
plot(time, states(:,9)); hold on;
plot(time, states(:,10)); hold on;
plot(time, states(:, 11)); hold off;
legend({'x disturbance','y disturbance', 'theta disturbance'});

subplot(5,1,3);
plot(time, states(:,1)); hold on;
plot(time, states(:,2)); 
plot(time, states(:,3)); 
plot(time, theta_desired,'--'); hold off;
legend({'x','y','theta', 'theta desired'});

subplot(5,1,4);
plot(time, states(:,4)); hold on;
plot(time, states(:,5)); hold on;
plot(time, states(:,6)); hold off;
legend({'x_d','y_d','theta_d'});

subplot(5,1,5);
plot(time, pid_save(:,1)); hold on;
plot(time, pid_save(:,2)); hold on;
plot(time, pid_save(:,3)); hold off;
ylim([-3 3]);
legend('error','integrator','differentiator');

figure(3);
subplot(3,1,1);
theta = states(:,3);
plot(time, theta); hold on;
plot(time, theta_desired,'--','LineWidth',2); hold off;
legend('theta dot (rad/s)');
title('PID Controller performance');

subplot(3,1,2);
vel = sqrt(states(:,4).^2 + states(:,5).^2);
plot(time, vel); hold on;
plot(time, vel_desired,'--','LineWidth',2); hold off;
legend('speed (m/s)');

subplot(3,1,3);
plot(time, states(:,7)); hold on;
plot(time, states(:,8)); hold off;
legend({'left motor','right motor'});

xlabel('time (s)');



%% Animate

%max_lim = max(abs([states(:,1); states(:,2)]));
%if ~max_lim
%    max_lim = 1;
%end
%max_lim = max_lim*1.2;

%for i=1:4:N
%figure(2);
%beluga_draw(states,i,Ts,max_lim);
%drawnow;
%end