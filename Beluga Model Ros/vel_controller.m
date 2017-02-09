function [ u, prev_pid ] = vel_controller( state_est, vel_desired, theta_desired, Ts, prev_PID)
% VEL_CONTROLLER
% Author: Vaibhav Viswanathan (vviswanathan@hmc.edu)
%             (email me for any questions!)
%                   
% Inputs: current state estimate (state_est), desired velocity
% (vel_desired), desired angular velocity(theta_desired), previous PID
% values(prev_PID)
% Output: control inputs (u)
%   current state estimate -> |   |
% desired angular velocity -> | H | -> control inputs
%         desired velocity -> |___|

% previous PID values
integral1   = prev_PID(1,1);
differentiator1 = prev_PID(1,2);
prev_error1 = prev_PID(1,3);

integral2   = prev_PID(2,1);
differentiator2 = prev_PID(2,2);
prev_error2 = prev_PID(2,3);

% PID constants for linear velocity
kp1 = 0.02; 
ki1 = 1;
kd1 = 0.002;
alpha1 = 0.2; % higher the number, the least low pass on the differentiator

% PID constants for yaw angle
kp2 = 0.6;
ki2 = 0.2;
kd2 = 0.5;


%% PID controllers
% linear velocity controller
vel = sqrt(state_est(4)^2+state_est(5)^2);
error1 = vel_desired - vel;
integral1 = integral1 + error1*Ts;
differentiator1_new = (error1 - prev_error1)/Ts;
prev_error1 = error1;

% differentiator needs low pass filter
differentiator1 = alpha1*differentiator1_new + (1-alpha1)*differentiator1;

linear = kp1*error1 + ki1*integral1 + kd1*differentiator1;

% yaw controller
theta = mod(state_est(3),2*pi);
error2 = theta_desired - theta;
error2 = mod(error2 + pi,2*pi)-pi;
integral2 = integral2 + error2*Ts;
differentiator2 = (error2 - prev_error2)/Ts;
prev_error2 = error2;

angular = kp2*error2 + ki2*integral2 + kd2*differentiator2;

% % save PID values for debugging
% pid_save(i,1) = error2;
% pid_save(i,2) = integral2;
% pid_save(i,3) = differentiator2;


% set motor speeds (normalized and saturated to 0..1)
u(1) = linear - angular;
u(2) = linear + angular;
u(1) = max(-1,min(1,u(1)));
u(2) = max(-1,min(1,u(2)));

% save PID values
prev_pid = [integral1 differentiator1 prev_error1;
            integral2 differentiator2 prev_error2];

end

