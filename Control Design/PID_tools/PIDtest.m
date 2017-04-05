
%Inputs
A = 1.06;
tau = 1.75;
td = .09;
w_co = 5;
phi_m = 63; %phase magin
PoleRatio = 10;
step = 1;
CtoD = 1;
fs = 10;

%PID phase margin design
result = PIDmarginDesign(A, tau, td, w_co, phi_m, PoleRatio, step, CtoD, fs);
Kp = result(1);
Ki = result(2);
Kd = result(3);

%physical disturbance
d = 0; %distrubance magnitude
dfreq = 10; %disturbance freq (Hz)

%sensor noise
v = .001; %sensor noise variance
sensorrate = 1/10; %sampling time (s)

%Simulink
sim('PIDsim.slx');
plot(tout,yout);
grid('on');



