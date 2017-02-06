% load step test
load('step_test_3.mat');

% normalize time
time = nsec*10^-9+sec1;
time = time - ones(numel(time),1)*time(1);

% plot
figure(2)
hold on
plot(time, -vy);
xlim([0 10]);