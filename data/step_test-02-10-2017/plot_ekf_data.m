function plot_ekf_data( data_filename, input_filename)

split_filename = regexp(input_filename,'\.','split');
file_prefix = split_filename(1);

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
if ~all(size(in_raw.data) == [2,2])
    fprintf('Bad input data: %s\n',input_filename)
    return
end
input_speed = -in_raw.data(1,2);
t_in_0 = (in_raw.data(1,1)-t0)*1e-9;
t_in_f = (in_raw.data(2,1)-t0)*1e-9;

ind = t>=t_in_0 & t<=t_in_f;

t_step = t(ind);
t0 = t_step(1);
t_step = t_step - t0;


vx_step = vx(ind);
vx_step = vx_step - vx_step(1);
vy_step = vy(ind);
vy_step = vy_step - vy_step(1);
vz_step = vz(ind);
vz_step = vz_step - vz_step(1);

s_2d_step = sqrt(vx_step.^2 + vy_step.^2);
s_step = sqrt(vx_step.^2 + vy_step.^2 + vz_step.^2);

figure
plot(t_step,s_2d_step)
grid on
xlabel('Time (s)')
ylabel('Speed (m/s)')
title(sprintf('Step Response -- Input: %g',input_speed))
print(file_prefix{1},'-dpng')

%{
figure
plot(t,s,'b')
hold on
plot(t(ind),s(ind),'r')
grid on


figure
plot(t,-vy,'b')
hold on
plot(t(ind),-vy(ind),'r')
grid on
xlabel('Time (s)')
ylabel('Forward Velocity (m/s)')


figure
subplot(3,1,1)
plot(t,vx,'b')
hold on
plot(t(ind),vx(ind),'r')
grid on

subplot(3,1,2)
plot(t,vy,'b')
hold on
plot(t(ind),vy(ind),'r')
grid on

subplot(3,1,3)
plot(t,vz,'b')
hold on
plot(t(ind),vz(ind),'r')
grid on

%}
end

