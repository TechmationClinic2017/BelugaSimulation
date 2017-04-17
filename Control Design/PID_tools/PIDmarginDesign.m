function [constants] = PIDmarginDesign(A, tau, td, w_co, phi_m, PoleRatio, step, CtoD, fs)
%Chris Kotcherha
%HMC Class of 2018 - Engineering
%
%outputs continuous PID constants in the form [Kp Ki Kd].
%Takes in 1st order process TF, phase margin, crossover frequency, and pole ratio
%specifications.
%Assumes 1st order process (with delay) with PID controller in closed loop
%
%ex:
%PIDmarginDesign(1.06, 1.75, .09,5, 70, 10,1,1,10)
%
%Input:
% A: process magnitude - openloop
% tau: (s) process time constant - openloop
% td: (s) process time delay - openloop
%w_co: crossover frequency (rad/s)
%phi_m: phase margin (degrees)
%PoleRatio: T2/T1 speed ratio of poles (between 10 and 100)
%step: if 1, plot closed loop step response using constants
%CtoD: Convert from Continuous to discrete?
%sfreq: Sampling time for c2d (Hz)
Ts = 1/fs;
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


%closed loop step response
s = tf('s');
H = (A*exp(-s*td))/(tau*s + 1); %1st order w/ delay
C = Kp + Ki/s + Kd*s  %PID Controller
L = C*H; %feedback TF
F = C*H; %forward path TF
Q = (F)/(1+L); %closed loop TF

%Discretize and Plot Closed Loop Step Responses
if CtoD==1
    Cd = c2d(C,Ts, 'matched') %discretize Controller
    Hd = c2d(H,Ts, 'matched'); %discretize Process
    Ld = Cd*Hd; %discrete feedback TF
    Fd = Cd*Hd; %discrete forward path TF
    Qd = (Fd)/(1+Ld);
    %discrete closed loop TF
    if step==1
        ltiview('step',Q,Qd);%plot continuous and discrete closed loop response
    end
    %convert to standard discrete PID form
    [num,den] = tfdata(Cd,'v');
    a = num(1);
    b = num(2);
    c = num(3);
    alpha = .2; %expoinential average factor (1=no effect)
    Kp_d = -b-(1+1/alpha)*c;
    Ki_d = (1/Ts)*(a+b+c);
    Kd_d = (Ts/alpha)*c;
elseif step==1
    ltiview('step',Q);
end

constants = [Kp Ki Kd Kp_d Ki_d Kd_d]; %output
end
