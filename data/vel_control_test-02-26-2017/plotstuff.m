
close all
load('step.mat')
xdes = linspace(0,8);
ydes = ones(numel(xdes),1)*0.5;

%simulation
figure(1)
plot(xsim, ysim, xdes, ydes,'--g')
xlabel('Time (s)')
ylabel('Forward Velocity (m/s)')
title('Closed Loop: 0.5 m/s')
legend('Simulation','Desired', 'location', 'southeast')
xlim([0 8])

% actual
figure(2)
plot(xsim, ysim, xdata, ydata, xdes, ydes,'--g')
xlabel('Time (s)')
ylabel('Forward Velocity (m/s)')
title('Closed Loop: 0.5 m/s')
legend('Simulation', 'Experimental', 'Desired', 'location', 'southeast')
xlim([0 8])