x = v25(:,1);
y = v25(:,2);
t = linspace(1,6,numel(x));

% find origin
x0 = x(1);
y0 = y(1);

% find displacement
x = x - x0*ones(numel(x),1);
y = y - y0*ones(numel(y),1);
d = sqrt(x.^2+y.^2);
d = d*0.3048;

% find velocity
dt = t(2) - t(1);
dd = diff(d)/dt;
td = t(1:end-1);

% find time scale
%t = 2*(1:numel(d));

% plot
figure(1)
plot(t,d)
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Step response v = 100%');
