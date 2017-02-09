function [ new_state_est ] = surface_vehicle_state_estimate( state_est, u, state, disturbance, Ts )
%SURFACE_VEHICLE_STATE_ESTIMATE
% Author: Vaibhav Viswanathan (vviswanathan@hmc.edu)
%             (email me for any questions!)
%                    ___
% Inputs: current state estimate (cur_state_est), control inputs (u),
% disturbance, Ts
% Output: new state estimate (new_state_est)
%  current state -> |   |
% estimate error -> | H | -> new state
% control inputs -> |___|

% Compute IMU state estimate
new_state_est = imu_state_est(state_est, u, disturbance, Ts);

end

