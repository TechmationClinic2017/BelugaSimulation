    state = zeros(1,12);
    %% Constants
    %disp('Calc Nonlinear Coeffs')
    %Conversion Factors:
    i2m = 0.0254; %Conversion factor (inches to meters)
    lb2N = 4.44822162; %Conversion factor (lbs-force to Newtons)
    %Water Characteristics:
    %U = 1.5; %nominal velocity (m/s)
    U = 1; %2 knots
    rho = 1.03*10^(3); %seawater density (kg/m^3)
    %****Iver2 Hull Parameters*********************
    a = 0.152;
    a_offset = 0.000001;
    b = 1.088;
    c = 0.00001;
    c_offset = 0;
    d = 0.1397;
    l = a+b+c;
    Tail_angle = 0.1;
    n = 0.1;
    %****************************************
    %**********Iver2 Moments of Inertia *****
    I_x = 0.05; %kg*m^2
    Iyy = 2.05; %kg*m^2
    Izz = 2.06; %kg*m^2
    %***************************************
    %Measured Parameters:
    W = 41.33*lb2N;
    m = W/9.8;
    B = 41.77*lb2N;
    %Measured or Estimated Centers of Buoyancy and Gravity
    xcg = 0*i2m;
    ycg = 0*i2m;
    zcg = 0.089*i2m;
    xcb = -23.72*i2m;

    %***********************Calculate The Non-Linear Coefficients**************
    %**************************************************************************
    %*****************************Axial Drag***********************************
    %Theoretical Drag Coeffecient: ( REMUS Eq 4.7)
    css = 3.397*10^(-3); %Schoenherr's value for flat plate skin friction
    Ap = l*d; %vehicle plan area
    Af = pi*(d/2)^2; %vehicle frontal area
    Cd = css*pi*Ap/Af*(1+60*(d/l)^3+0.0025*(l/d)); %drag coefficient estimate
    X_u_abs_u = -1/2*rho*Cd*Af; %Linear axial drag coefficient (kg/s)
    %*****************************Crossflow Drag*******************************
    %Define integration step size and limits of integration
    dx = 0.01; %integration step size
    %****Iver2 Values*****
    xn = abs(xcb)-(a-a_offset);
    xn2 = xn + (a-a_offset);
    xt2 = -(b-xn);
    xt = xt2 - (c - c_offset);
    xf = xt;
    lf = a+b-a_offset; %vehicle forward length
    %*********************
    Cdc = 1.1; %crossflow drag coefficient of a cylinder(Hoerner)
    Cdf = 0.1; %control fin crossflow drag coefficient
    intg = cdrag(xt,xt2,xn,xn2,d,a,a_offset,n,c,Tail_angle,l,lf,dx,xcb);
    Z_w_abs_w = -1/2*rho*Cdc*intg;
    Y_v_abs_v = Z_w_abs_w;
    intg = cdrag(xt,xt2,xn,xn2,d,a,a_offset,n,c,Tail_angle,l,lf,dx,xcb);
    M_w_abs_w = -1/2*rho*Cdc*intg;
    N_v_abs_v = -M_w_abs_w;
    intg = cdrag(xt,xt2,xn,xn2,d,a,a_offset,n,c,Tail_angle,l,lf,dx,xcb);
    Zq_abs_q = 1/2*rho*Cdc*intg;
    Y_r_abs_r = -Zq_abs_q;
    intg = cdrag(xt,xt2,xn,xn2,d,a,a_offset,n,c,Tail_angle,l,lf,dx,xcb);
    M_q_abs_q = -1/2*rho*Cdc*intg;
    N_r_abs_r = M_q_abs_q;
    %Rolling Drag
    K_p_abs_p = 0; % assume zero  %-1.3*10^(-1); %Used REMUS VALUE
    
    %% States and calculations
    % defining states
    u = state(1);
    v = state(2);
    w = state(3);
    p = state(4);
    q = state(5);
    r = state(6);
    
    x = state(7);
    y = state(8);
    z = state(9);
    phi = state(10);
    theta = state(11);
    psi = state(12);
    
    % Experimental values
    A = 0.0467438; % cross section
    c_dx = 0.589;%1.87367;
    %X_u_abs_u = -0.5*rho*c_dx*A;
    
    % Drag forces
    X_drag = X_u_abs_u*u*abs(u);
    Y_drag = Y_v_abs_v*v*abs(v);
    Z_drag = Z_w_abs_w*w*abs(w);
    K_drag = K_p_abs_p*p*abs(p);
    M_drag = M_w_abs_w*w*abs(w)+M_q_abs_q*q*(abs(q));
    N_drag = N_v_abs_v*v*abs(v)+N_r_abs_r*r*abs(r);
    

    % output force vector
    X = X_drag;
    Y = Y_drag;
    Z = Z_drag;
    K = K_drag;
    M = M_drag;
    N = N_drag;
    F = [X Y Z K M N]'; 
    %F = [0 0 0 0 0 0]';

function intg = cdrag(xt,xt2,xn,xn2,d,a,a_offset,n,c,theta,l,lf,dx,xb)
%This function performs the integration for the first equation of 4.8 in
%the REMUS thesis
intg = 0;
xti = l;
for x = xt:dx:xt2
 %Need to adjust for the difference in origin definitions between the
 %radius functions and the body-coord with the following transform:
 Rx = ( 1/2*d - ( 3*d/(2*c^2) - tan(theta)/c )*(xti-lf)^2 + ...
 ( d/(c^3) - tan(theta)/(c^2) )*(xti - lf)^3 );
 xti = xti-dx;

 intg = intg + 2*Rx*dx;
end
for x = xt2:dx:xn
 Rx = d/2;
 intg = intg + 2*Rx*dx;
end
for x = xn:dx:xn2
 %Need to adjust for the difference in origin definitions betwee the
 %radius functions and the body-coord with the following transform:
 xi = -( x - abs(xb) + (a-a_offset) ) + a;
 Rx = ( 1/2*d*( 1 - ( (xi + a_offset-a)/a )^2 )^(1/n) );

 intg = intg + 2*Rx*dx;
end
end
%End of cdrag function
