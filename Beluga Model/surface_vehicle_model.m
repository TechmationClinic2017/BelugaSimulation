%% Simulates a 2D surface vehicle with linear velocity and theta (yaw) setpoints
% 
% Author: Benjamin Chasnov (bchasnov@gmail.com) and Vai Viswanathan
% (vviswanathan@hmc.edu)
%             (email me for any questions!)
% Date: May 2016
%

clear all
close all
animationspeed = 4; % 4 gives approximately real time
Tend = 60;   % run simulation for Tend seconds. must be multiple of 3

Ts = 0.01; % Control loop at 100Hz
state = [0 0 0 ...
         0 0 0 ...
         0 0 ...
         0 0 0]; 
% state = [x, y, theta, x_dot, y_dot, theta_dot, 
%          u(1), u(2), disturbance(1), disturbance(2), disturbance(3)];

state_est = state;

N = Tend/Ts; % number of cycles
time = linspace(0,Tend,N);

% initialize states, control and disturbances inputs to zero
states = zeros(N, length(state));
states_est = zeros(N, length(state_est));
u = zeros(N,2);
disturbance = zeros(N,3);
%disturbance(:,3) = 9*ones(N,1);
disturbance = 1*(rand([N 3])-0.5);

% PID controller
integral1   = 0;
differentiator1 = 0;
prev_error1 = 0;

integral2   = 0;
differentiator2 = 0;
prev_error2 = 0;

prev_pid = [integral1 differentiator1 prev_error1;
            integral2 differentiator2 prev_error2];

% set points
%theta_desired = sin([1:N]/100)*pi/2+2;
%vel_desired = [0.3*ones(N/3,1); 0.8*ones(N/3,1); 1.2*ones(N/3,1)];
pos_des = [[1,1,0];
           [2,2,0]
           [3,1,0]];
point_num = 1;

pid_save = zeros(N,3);

%% keyboard control
% figure(2);
% 
% global buttons;
% global b_v;
% 
% b_v = {'leftarrow','rightarrow','pageup','pagedown','q'};
% 
% buttons = zeros(1,length(b_v));
% 
% set(gcf, 'KeyPressFcn',   @(h_obj,e) ...
%  eval('global buttons; global b_v; buttons = buttons + arrayfun(@(n) strcmp(e.Key,b_v{n}), [1:length(b_v)]);'));
% set(gcf, 'KeyReleaseFcn', @(h_obj,e)...
%  eval('global buttons; global b_v; buttons = buttons .* arrayfun(@(n) ~strcmp(e.Key,b_v{n}), [1:length(b_v)]);'));
% 
% mouse_xy = [0 0];

figure(1);

%% Main loop
for i=1:N

%     % manual keyboard control
%     if buttons(end) % quit
%        break
%     end
% 
%     u(i,1) = 0.05*(buttons(3)-buttons(1));
%     u(i,2) = 0.05*(buttons(4)-buttons(2));

    %% Point tracking controller
    scatter(pos_des(point_num,1),pos_des(point_num,2)); hold on;
    [vel_desired(i), theta_desired(i)] = point_tracking_controller(state_est, pos_des(point_num,1), pos_des(point_num,2), pos_des(point_num, 3));
    % next point if close
    goal_rad = 0.2;
    if((sqrt((state_est(1)-pos_des(point_num,1)).^2+(state_est(2)-pos_des(point_num,2)).^2) < goal_rad) && (point_num < (numel(pos_des)/3)+1))
        point_num = point_num+1;
    end
    if(point_num > (numel(pos_des)/3))
        break;
    end

    %% Velocity Controller
    [u(i, :), prev_pid] = vel_controller(state_est, vel_desired(i), theta_desired(i), Ts, prev_pid);

    %% Run model.
    state = surface_vehicle_dynamic_model(state,u(i,:),disturbance(i,:),Ts);
    states(i,:) = state;
    
    state_est = surface_vehicle_state_estimate(state_est,u(i,:),state,disturbance(i,:),Ts);
    states_est(i,:) = state_est;

    if mod(i,animationspeed) == 1
        surface_vehicle_draw(states,states_est,i,Ts,1);
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
%surface_vehicle_draw(states,i,Ts,max_lim);
%drawnow;
%end