% linearize the Beluga dynamic model

% TODO define constants
m = 20.2; %kg 

% create constant variables
syms m Ix Iy Iz xg zg W ...
    X_udot Y_vdot Y_rdot Z_wdot Z_qdot K_pdot M_wdot M_qdot N_vdot N_rdot ...
    X_u_abs_u K_p_abs_p M_w_abs_w M_q_abs_q N_v_abs_v N_r_abs_r ...
    K rho d L

% create state variables
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 n1 n2 n3 n4

% state vectors
f_state = [ X_u_abs_u/(m-X_udot)*(x1*abs(x1));
            0;
            0;
            1/(Ix-K_pdot)*(-zg*W*cos(x8)*sin(x7)+K_p_abs_p*x4*abs(x4));
            1/(Iy-M_qdot)*(-zg*W*sin(x8)+M_w_abs_w*x3*abs(x3)+M_q_abs_q*x5*abs(x5));
            1/(Iz-N_rdot)*(N_v_abs_v*x5*abs(x5)+N_r_abs_r*x6*abs(x6))];
f_control = [   K*rho*d^4/(m-X_udot)*(n1^2-x1*n1/d+n2^2-x1*n2/d+n3^2-x1*n3/d+n4^2-x1*n4/d);
                0;
                0;
                0;
                L*K*rho*d^4/(Iy-M_qdot)*(n1^2-x1*n1/d+n3^2-x1*n3/d);
                L*K*rho*d^4/(Iz-N_rdot)*(n2^2-x1*n2/d+n4^2-x1*n4/d);
                0;
                0;
                0;
                0;
                0;
                0];
 
            
            
f_state = [f_state(1) f_state(5)];
f_control = [f_control(1) f_control(5)];
% Now compute jacobian
dfdx = jacobian(f_state, [x1, x5])
dfdu = jacobian(f_control, [n1; n2; n3; n4])