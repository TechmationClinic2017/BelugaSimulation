%% Draws the surface vehicle

function beluga_draw(states, i,Ts,max_lim)
    % Velocity and theta values
    vel   = sqrt(states(:,4).^2 + states(:,5).^2);
    theta = states(:,3);

    % Zoom out as the robot moves out of frame
    max_lim2 = max(max(abs(states(:,1:2))));
    max_lim = max(max_lim, max_lim2)*1.2;
    
    % Plot the track
    plot3(states(1:i,1),states(1:i,2),states(1:i,3),'-'); hold on;
    xlim([-max_lim max_lim]);
    ylim([-max_lim max_lim]);
    zlim([-max_lim max_lim]);
    hold on;
    
    % Draw the robot
    plot3(states(i,1),states(i,2),states(i,3),'ko');
    plot3([states(i,1), states(i,1)+0.1*max_lim*cos(states(i,4))*cos(states(i,5))],...
         [states(i,2), states(i,2)+0.1*max_lim*sin(states(i,4))*cos(states(i,5))], ...
         [states(i,3), states(i,3)+0.1*max_lim*sin(states(i,6))], ...
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