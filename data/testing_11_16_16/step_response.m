tests = ['025'; '050'; '075'; '100'];

for i = 1:size(tests,1)
   imu = strcat('step_test_', tests(i,:),'_imu.dat'); 
   input = strcat('step_test_',tests(i,:),'_bottom_input.dat');
   [t, v] = return_step(imu,input);
   figure(i)
   plot(t,v)
   grid on
   xlabel('Time (s)')
   ylabel('Forward Velocity (m/s)')
end