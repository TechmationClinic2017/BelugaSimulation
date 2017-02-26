function [ vel_des, theta_des ] = point_tracking_controller(state_est, x_des, y_des, theta_des)
% POINT_TRACKING_CONTROLLER
% Author: Vaibhav Viswanathan (vviswanathan@hmc.edu)
%             (email me for any questions!)
%                   
% Inputs: current state estimate (state_est), desired velocity
% (vel_desired), desired angular velocity(theta_desired), previous PID
% values(prev_PID)
% Output: control inputs (u)
% current state estimate -> |   | -> desired velocity
%                           | H | 
%       desired position -> |___| -> desired angular velocity

%% STATE SPACE CONSTANTS TODO tune
k_rho = 1;
k_beta = -1;
k_alpha = 2;

%% STATE SPACE CONTROLLER
dx = (state_est(1)-x_des);
dy = (state_est(2)-y_des);
theta = state_est(3);

rho = sqrt(dx.^2+dy.^2);
alpha = -theta + atan2(dy, dx);
beta = -theta - alpha; 

vel_des = rho*k_rho; 
theta_des = k_alpha*alpha+k_beta*beta; 

% saturate values
theta_des = mod(theta_des, 2*pi);
vel_des = min(vel_des, 5);

end

