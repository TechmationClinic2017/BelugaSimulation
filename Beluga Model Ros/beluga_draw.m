%% Draws the surface vehicle

function beluga_draw(states, i,Ts,max_lim)
    % Velocity and theta values
    vel   = sqrt(states(:,1).^2 + states(:,2).^2 + states(:,3).^2);
    theta = states(:,11);

    % Zoom out as the robot moves out of frame
    max_lim2 = max(max(abs(states(:,1:2))));
    max_lim = max(max_lim, max_lim2)*1.2;
    
    % Plot the track
    plot3(states(1:i,7),states(1:i,8),states(1:i,9),'-'); hold on;
    xlim([-max_lim max_lim]);
    ylim([-max_lim max_lim]);
    zlim([-max_lim max_lim]);
    hold on;
    
    % Draw the robot
    plot3(states(i,7),states(i,8),states(i,9),'ko');
    plot3([states(i,7), states(i,7)+0.1*max_lim*cos(states(i,10))*cos(states(i,11))],...
         [states(i,8), states(i,8)+0.1*max_lim*sin(states(i,10))*cos(states(i,11))], ...
         [states(i,9), states(i,9)+0.1*max_lim*sin(states(i,12))], ...
          'k','LineWidth',3);
    hold off;
    
    % Draw the title and labels
    title([num2str(i*Ts,'%.2f') ' s, '...
           num2str(vel(i),'%.2f') 'm/s, '...
           num2str(theta(i)/pi,'%.2f') ' \pi rad']);
    xlabel('x (m)');
    ylabel('y (m)');
    
    grid minor;
end