% run dynamic model of beluga


Ts = 0.01; % Control loop at 100 Hz
state = [0 0 0 0 0 0 ...
         0 0 0 0 0 0 ...
         0 0 0 0]; 

u = ones(1,4);

disturbance = zeros(1,3);
     
%% Run model.
state = beluga_dynamic_model(state,u,disturbance,Ts);