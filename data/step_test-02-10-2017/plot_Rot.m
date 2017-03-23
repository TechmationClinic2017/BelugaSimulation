clear all

% Load files
load('rot_50.mat');

% normalize time and time1 (the step time)
init = time(1);
time = (time - init*ones(size(time,1),1))/10^10;
time1 = (time1 - init*ones(size(time1,1),1))/10^10;

% cut out stuff from end of step response
ind = find(time > time1(2),1);
time = time(1:ind-1);
fieldposeposeorientationw = fieldposeposeorientationw(1:ind-1);
fieldposeposeorientationx = fieldposeposeorientationx(1:ind-1);
fieldposeposeorientationy = fieldposeposeorientationy(1:ind-1);
fieldposeposeorientationz = fieldposeposeorientationz(1:ind-1);

% cut out stuff from beginning of step response
ind = find(time > time1(1),1);
time = time(ind-1:end);
fieldposeposeorientationw = fieldposeposeorientationw(ind-1:end);
fieldposeposeorientationx = fieldposeposeorientationx(ind-1:end);
fieldposeposeorientationy = fieldposeposeorientationy(ind-1:end);
fieldposeposeorientationz = fieldposeposeorientationz(ind-1:end);

% plot things
figure(1)
plot(time, [fieldposeposeorientationw, fieldposeposeorientationx, fieldposeposeorientationy, fieldposeposeorientationz])
title('Rotational Step Response Position (0.5)');
xlabel('Time (s)')
legend('w', 'x', 'y', 'z')

% save just the important z
t = (time - time(1)*ones(size(time,1),1));
yaw = fieldposeposeorientationz;
yaw = (yaw - yaw(1)*ones(size(yaw,1),1));
figure(2);
plot(t,yaw)
title('Rotational Step Response Position (0.5)');
xlabel('Time (s)')
ylabel('Yaw (rads?)')
