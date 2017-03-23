function F = control(u, inputs)
    
    %% Constants TODO
    d = 0.076; %m
    rho = 1000; %kg/m^3
    
    %% Thruster Model
    % Values from documentation stationary calibration
    thruster_pwm = [1100        1110        1120        1130        1140        1150        1160        1170 1180        1190        1200        1210        1220        1230        1240        1250        1260        1270        1280        1290        1300        1310        1320        1330        1340        1350        1360        1370        1380        1390        1400        1410         1420        1430        1440        1450        1460        1470        1480        1490 1500        1510        1520        1530        1540        1550        1560        1570 1580        1590        1600        1610        1620        1630        1640        1650 1660        1670        1680        1690        1700        1710        1720        1730 1740        1750        1760        1770        1780        1790        1800        1810 1820        1830        1840        1850        1860        1870        1880        1890 1900];
    thruster_force = 9.81*[-4.0823   -4.0823   -4.0869   -3.8283   -3.7331   -3.6197   -3.4428   -3.3022   -3.1842   -3.0572 -2.9075   -2.8168   -2.7079   -2.5265   -2.3632   -2.2589   -2.1772   -1.9913   -1.8824   -1.7690 -1.6420   -1.4923   -1.3336   -1.2428   -1.1204   -1.0251   -0.9072   -0.7893   -0.6940   -0.6169 -0.5216   -0.4354   -0.3538   -0.2767   -0.1905   -0.1134   -0.0544   -0.0272         0         0 0         0         0    0.0590    0.1134    0.1860    0.2631    0.3493    0.4173    0.5216 0.6350    0.7167    0.8437    0.9435    1.1022    1.2474    1.3925    1.5286    1.6783    1.8053 1.9595    2.0956    2.2181    2.4131    2.5220    2.6943    2.8259    3.0209    3.1933    3.3521 3.4473    3.4927    3.7784    3.9417    4.0052    4.1957    4.3772    4.5677    4.8035    4.8988 5.0938];
    
    thruster_omega = sign(thruster_force).*sqrt(abs(thruster_force)./(rho*d^4*0.1858));
    
    % From Arduino implementation
    DEAD_POS = 16;
    DEAD_NEG = -35;
    PERIOD = 500; % [ms]
    LEAK_THRESH = 150;
    %Correct for deadzone
    for i = 1:numel(inputs)
        if inputs(i) > 0
            inputs(i) = inputs(i)*50+DEAD_POS;
        elseif inputs(i) < 0
            inputs(i) = inputs(i)*50+DEAD_NEG;            
        end
        
        inputs(i) = inputs(i) + 1500;
        inputs(i) = interp1(thruster_pwm, thruster_force, inputs(i)); % assuming static thrust
    end

 
    %approximate dynamic thrust

    for i = 1:numel(inputs)
        if inputs(i)==0
            inputs(i) = 0.0001;
        end
        n = sqrt(abs(inputs(i))/(rho*d^4*0.1858));
        Kt = 0.1858*(1-(u/(n*d)));
        Kt(isnan(Kt))=0;
        inputs(i) = real(Kt*rho*n.^2*d^4);
    end
    
    % Control Forces
    X_c = sum(inputs); 
    Y_c = 0;
    Z_c = 0;
    K_c = inputs(1)-inputs(2)+inputs(3)-inputs(4); %TODO CHECK CONSTANTS AND POLARITY
    M_c = inputs(2)-inputs(4); %TODO CHECK
    N_c = inputs(1)-inputs(3); %TODO CHECK POLARITY
    
    % output force vector
    X = X_c;
    Y = Y_c;
    Z = Z_c;
    K = K_c;
    M = M_c;
    N = N_c;
    F = [X Y Z K M N]'; 
    %F = [0 0 0 0 0 0]';
    
    %% fit stuff
    pos_omega = thruster_omega(44:81);
    pos_pwm = thruster_pwm(44:81);
%     plot(pos_pwm, pos_omega)
%     ylabel('Thruster angular velocity (deg/s)');
%     xlabel('PWM (ms)')
    
    % create fit
    %myfit = a*sqrt(b+c)+d;  
    myfit = fittype('a + b*log(x+d)',...
    'dependent',{'y'},'independent',{'x'},...
    'coefficients',{'a','b', 'd'});
   
    f = fit((pos_pwm)', pos_omega', 'poly1');
     
    
    plot(f, (pos_pwm), pos_omega)
    ylabel('Thruster angular velocity (deg/s)');
    xlabel('PWM (ms)')
    
    
    
    
end