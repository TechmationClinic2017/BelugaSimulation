%% Process the stationary data and produce covariance matrices
stationary_raw = importdata('stationary_imu.csv');

s_magn  = stationary_raw.data(:, 1:3);
s_accel = stationary_raw.data(:, 4:6);
s_gyro  = stationary_raw.data(:, 7:9);

cov_m = cov(s_magn);
cov_a = cov(s_accel);
cov_g = cov(s_gyro);

%% Get the gps data for our path travelled
gps_raw  = importdata('path_gpsfix.csv');
gps_time = cellfun(@str2num,gps_raw.textdata(2:3,1));
lat  = gps_raw.data(:, 3);
long = gps_raw.data(:, 4);
alt  = gps_raw.data(:, 5);

%% Get the imu data for the path travelled
imu_raw = importdata('path_imu.csv');
imu_time = cellfun(@str2num,imu_raw.textdata(2:3,1));
magn  = imu_raw.data(:, 1:3);
accel = imu_raw.data(:, 4:6);
gyro  = imu_raw.data(:, 7:9);

%% Make some plots

figure
plot(long, lat);
hold on
plot(long(1),lat(1),'rx');
xlabel('Longitude')
ylabel('Latitude')
title('Reconstructed Path, GPS Only')