% Given a sequence of waypoints totalTrajError computes the integral of the
% divergence between the robot trajectory and the lines defined by the
% waypoints.
%
% TotErr = totalTrajError(WP,t,traj,d_wp)
%
% Inputs
%   WP      - array of waypoints
%   t       - timestamps of the trajectory 
%   traj    - the history of poses (x,y,theta) of the robot
%   d_wp    - distance to waypoint for the waypoint to be considered
%             visited 
%
% Outputs
%   TotErr - the sum of the distances of the position of the robot along 
%       its trajectory from the lines which are defined by the waypoints

% (C) G. Fainekos - ASU

function  TotErr = totalTrajError(WP, t, traj, d_wp)

wp_id = 2;
TotErr = 0;

for i = 2:size(traj,1)
    if wp_id <= size(WP,1)

        % Is the WP line parallel to y axis?
        if abs(WP(wp_id,1)-WP(wp_id-1,1))>eps

            % Compute line parameters
            a = -(WP(wp_id,2)-WP(wp_id-1,2))/(WP(wp_id,1)-WP(wp_id-1,1));
            b = 1;
            c = -a*WP(wp_id-1,1)-WP(wp_id-1,2);
            L_par = [a b c];

        else

            % Line parameters
            a = 1;
            b = 0;
            c = -WP(wp_id-1,1);
            L_par = [a b c];

        end

        % compute divergence error 
        DivErr1 = abs(traj(i-1,1)*L_par(1)+traj(i-1,2)*L_par(2)+L_par(3))/sqrt(L_par(1)^2+L_par(2)^2);
        DivErr2 = abs(traj(i,1)*L_par(1)+traj(i,2)*L_par(2)+L_par(3))/sqrt(L_par(1)^2+L_par(2)^2);
        TotErr = TotErr+(DivErr2+DivErr1)/2*(t(i)-t(i-1));

        % did we visit the waypoint?
        if norm(WP(wp_id,:)-traj(i,1:2))<d_wp
            wp_id = wp_id+1;
        end

    end
end

if wp_id~=(size(WP,1)+1)
    TotErr = inf;
    warning('Not all waypoints were visited!')
end

end

