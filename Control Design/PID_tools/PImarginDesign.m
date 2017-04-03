function [constants] = PImarginDesign(A, tau, td, w_co, phi_m, step, CtoD, fs)
%Chris Kotcherha
%HMC Class of 2018 - Engineering
%
%outputs PI constants in the form [Kp Ki].
%Takes in 1st order process TF, phase margin, and crossover frequency
%specifications.
%Assumes 1st order process (with delay) with PID controller in closed loop

%ex:
%PImarginDesign(1.06, 1.75, .09,5, 60,1,1,20)

%Input:
% A: process magnitude - openloop
% tau: (s) process time constant - openloop
% td: (s) process time delay - openloop
%w_co: crossover frequency (rad/s)
%phi_m: phase margin (degrees)
%step: if 1, plot closed loop step response using constants
%CtoD: Convert from Continuous to discrete?
%sfreq: Sampling time for c2d (Hz)

Ts = 1/fs;
w = w_co;
pm = phi_m/(180/pi); %phase margin (rads)

syms Kp Ki
%Phase Margin equations:
MagEqn = A*((Ki^2+(Kp*w)^2)^.5)/( w*(1+(tau*w)^2)^.5 ) == 1;
PhaseEqn = pm == pi/2 - w*td + atan2(Kp*w,Ki) - atan2(tau*w,1);
Eqns = [MagEqn PhaseEqn];

%calculate Kp and Ki
[Kp Ki] = solve( Eqns, [Kp Ki]); 
Kp = double(Kp);
Ki = double(Ki);

constants = [Kp Ki]; %output

%plot closed loop step response
s = tf('s');
H = (A*exp(-s*td))/(tau*s + 1); %1st order w/ delay
C = Kp + Ki/s %PI Controller
L = C*H; %openloop TF
F = C*H; %forward path TF
Q = (F)/(1+L); %closed loop TF

%Discretize and Plot Closed Loop Step Response
if CtoD==1
    Cd = c2d(C,Ts, 'matched') %discretize Controller
    Hd = c2d(H,Ts, 'matched'); %discretize Process
    Ld = Cd*Hd; %discrete feedback TF
    Fd = Cd*Hd; %discrete forward path TF
    Qd = (Fd)/(1+Ld); %discrete closed loop TF
    if step==1
        ltiview('step',Q,Qd);%
    end

elseif step==1
    ltiview('step',Q);
end


end

