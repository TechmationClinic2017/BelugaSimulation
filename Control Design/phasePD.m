function [ Kp, Kd ] = phasePD( phi_m, omega0  )
%phasePD Returns the PID constants for a given phase margin and crossover
%freq.

    syms s B td Kp Kd  % omega0 phi_m

    % Define plant
    B = 0.55;
    td = 0.11;
    s = omega0*1i;
    G = B*exp(-s*td)/s^2;
    H = Kp + Kd*s;

    % Solve
    [Kp, Kd] = solve(G*H == -exp(1i*phi_m), [Kp, Kd]);
    Kp = double(real(Kp));
    [Kd] = solve(G*H == -exp(1i*phi_m), [Kd]);
    Kd = double(real(Kd.Kd));
end