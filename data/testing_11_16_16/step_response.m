% load step test
load('step_100.mat');

% normalize time
time = time*10^-9;
time = time - ones(numel(time),1)*time(1);

% plot
figure(1)
hold on
plot(time, sqrt(fieldtwisttwistlinearx.^2+fieldtwisttwistlineary.^2+fieldtwisttwistlinearz.^2));
xlim([0 15]);
xlabel('Time(s)')
ylabel('Velocity(m/s)');

% load step test
clear all;
load('step_75.mat');

% normalize time
time = time*10^-9;
time = time - ones(numel(time),1)*time(1);

% plot
figure(1)
hold on
plot(time, sqrt(fieldtwisttwistlinearx.^2+fieldtwisttwistlineary.^2+fieldtwisttwistlinearz.^2));
xlim([0 15]);
xlabel('Time(s)')
ylabel('Velocity(m/s)');

% load step test
clear all;
load('step_50.mat');

% normalize time
time = time*10^-9;
time = time - ones(numel(time),1)*time(1);

% plot
figure(1)
hold on
plot(time, sqrt(fieldtwisttwistlinearx.^2+fieldtwisttwistlineary.^2+fieldtwisttwistlinearz.^2));
xlim([0 15]);
xlabel('Time(s)')
ylabel('Velocity(m/s)');

% load step test
clear all;
load('step_25.mat');

% normalize time\
time = time2;
time = time*10^-9;
time = time - ones(numel(time),1)*time(1);

% plot
figure(1)
hold on
plot(time, sqrt(fieldtwisttwistlinearx2.^2+fieldtwisttwistlineary2.^2+fieldtwisttwistlinearz2.^2));
xlim([0 15]);
xlabel('Time(s)')
ylabel('Velocity(m/s)');