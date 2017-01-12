%% Draws the surface vehicle

function surface_vehicle_draw(states, states_est, i,Ts,max_lim)
    % Velocity and theta values
    vel   = sqrt(states(:,4).^2 + states(:,5).^2);
    theta = states(:,3);

    % Zoom out as the robot moves out of frame
    max_lim2 = max(max(abs(states(:,1:2))));
    max_lim = max(max_lim, max_lim2)*1.2;
    
    % Plot the track
    plot(states(1:i,1),states(1:i,2),'-'); hold on;
    plot(states_est(1:i,1),states_est(1:i,2),'g');
    xlim([-max_lim max_lim]);
    ylim([-max_lim max_lim]);
    hold on;
    
    % Draw the robot
    plot(states(i,1),states(i,2),'ko');
    plot([states(i,1), states(i,1)+0.1*max_lim*cos(states(i,3))],...
         [states(i,2), states(i,2)+0.1*max_lim*sin(states(i,3))],'k','LineWidth',3);
    hold off;
    
    % Draw the title and labels
    title([num2str(i*Ts,'%.2f') ' s, '...
           num2str(vel(i),'%.2f') 'm/s, '...
           num2str(theta(i)/pi,'%.2f') ' \pi rad']);
    xlabel('x (m)');
    ylabel('y (m)');
    
    grid minor;
end