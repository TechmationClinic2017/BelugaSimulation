function[t, v] = return_step( data_filename, input_filename)

raw = importdata(data_filename);
t_str = raw.textdata(2:end,1);
t0 = str2num(t_str{1});
t = (cellfun(@str2num,t_str)-t0)*1e-9;
p = raw.data(:,1:3);
v = raw.data(:,44:46);
vx = v(:,1);
vy = v(:,2);
vz = v(:,3);

s = sqrt(vx.^2+vy.^2+vz.^2);

in_raw = importdata(input_filename);

t_in_0 = (in_raw.data(1,1)-t0)*1e-9;
t_in_f = (in_raw.data(2,1)-t0)*1e-9;

ind = t>=t_in_0 & t<=t_in_f;

% normalize step
t = t(ind);
v = -vy(ind);
t = t - ones(numel(t),1)*t(1);
v = v - ones(numel(v),1)*v(1);

% Plot
%figure
%plot(t,-vy,'b')
%hold on
% plot(t,v)
% grid on
% xlabel('Time (s)')
% ylabel('Forward Velocity (m/s)')


end

