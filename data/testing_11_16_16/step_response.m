tests = ['025'; '050'; '075'; '100'];

for i = 1:size(tests,1)
   imu = strcat('step_test_', tests(i,:),'_imu.dat'); 
   input = strcat('step_test_',tests(i,:),'_bottom_input.dat');
   [t, v] = return_step(imu,input);
   analyzeStep(t(2:20),v(2:20));
%    figure(i)
%    plot(t,v)
%    grid on
%    xlabel('Time (s)')
%    ylabel('Forward Velocity (m/s)')
end

function [tau, yss] = analyzeStep(t,v)
    tStep = mean(diff(t));
    vd = diff(v)/tStep;
    lnV = log(vd);
    t= t(2:end);
    p = polyfit(t, lnV, 1);
    p = max(diff(lnV))/tStep;
    tau = -1/(p(1))
    yss = tau*vd(1)
    figure
    plot(t,lnV)
end
