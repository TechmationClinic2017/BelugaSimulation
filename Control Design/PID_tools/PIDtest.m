%Test Data (openloop)
ControlInput_all = [.25 .5 .75 1.0];
Rps_all = [17.4 25.0 31.25 36.6];
mps_all = [.8 1.05 1.46 1.54];
tau_all = [1.35 1.13 .9 .87];

i=2;
%Inputs
A = mps_all(i)/Rps_all(i);
tau = tau_all(i);
td = .09;
w_co = 5;
phi_m = 63; %phase magin (adjust)
PoleRatio = 10;
step = 1;
CtoD = 1;
fs = 10; %controller frequency [Hz]

warning('off','all')
%PID phase margin design
result = PIDmarginDesign(A, tau, td, w_co, phi_m, PoleRatio, step, CtoD, fs);
Kp = result(1)
Ki = result(2)
Kd = result(3)
discrete_Kp = result(4)
discrete_Ki = result(5)
discrete_Kd = result(6)


%SIMULINK
%physical disturbance
d = 0; %distrubance magnitude
dfreq = 10; %disturbance freq (Hz)

%sensor noise
v = 0; %sensor noise variance
sensorrate = 1/10; %sampling time (s)
r = 1;
%Simulink
sim('PIDsim.slx');
plot(tout,yout);
grid('on');



