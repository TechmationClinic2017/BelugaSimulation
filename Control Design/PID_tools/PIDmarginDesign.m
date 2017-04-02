function [constants] = PIDmarginDesign(A, tau, td, w_co, phi_m, PoleRatio, step)
%outputs PID constants in the form [Kp Ki Kd T1 T2].
%Takes in 1st order process TF, phase margin, crossover frequency, and pole ratio
%specifications.
%Assumes 1st order process (with delay) with PID controller in closed loop

%ex:
%PIDmarginDesign(1.06, 1.75, .09,5, 60, 10,1)

%Input:
% A: process magnitude - openloop
% tau: (s) process time constant - openloop
% td: (s) process time delay - openloop
%w_co: crossover frequency (rad/s)
%phi_m: phase margin (degrees)
%PoleRatio: T2/T1 speed ratio of poles (between 10 and 100)
%step: if 1, plot closed loop step response using constants

w = w_co;
pm = phi_m/(180/pi); %phase margin (rads)

syms T1 T2 Ki Kp Kd
%Phase Margin equations:
PoleEqn = T2/T1 == PoleRatio;
MagEqn = A*Ki*((1+(T1*w)^2)^.5)*((1+(T2*w)^2)^.5)/( w*(1+(tau*w)^2)^.5 ) == 1;
PhaseEqn = pm == pi/2 - w*td + atan2(T1*w,1) + atan2(T2*w,1) - atan2(tau*w,1);
Eqns = [PoleEqn MagEqn PhaseEqn];

%calculate Ki,T1,and T2
[T1 T2 Ki] = solve( Eqns, [T1 T2 Ki]); 
T1 = double(T1);
T2 = double(T2);
Ki = double(Ki);

%convert to Kp Ki Kd form
[Kp Kd] = solve([T1*T2 == Kd/Ki, T1+T2 == Kp/Ki], [Kp Kd]);
Kp = double(Kp);
Kd = double(Kd);

constants = [Kp Ki Kd T1 T2]; %output

%plot closed loop step response
s = tf('s');
H = (A*exp(-s*td))/(tau*s + 1); %1st order w/ delay
C = Kp + Ki/s + Kd*s;  %PID Controller
L = C*H; %openloop TF
F = C*H; %forward path TF
Q = (F)/(1+L); %closed loop TF

if step==1
    ltiview('step',Q);
else

end

