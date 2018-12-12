% GoTrhoughWPDriver Vehicle driver class
%
% Create a "driver" object capable of driving a Vehicle object through a 
% given sequence of waypoints.
%
% The driver object is attached to a Bicycle object by the latter's
% add_driver() method.
%
% Methods to modify::
%  demand     return speed and steer angle for the next time instant
%
% Methods to ignore (not related to motion planning)
%  init       does nothing 
%  display    display the state and parameters in human readable form
%  char       convert to string the display method
%      
% Properties::
%  veh           the Vehicle object being controlled
%  WP            an array of waypoint coordinates. Each row contains the [x y] 
%                coordinates of each waypoint.
%  d_wp          Radius of neighborhood around each waypoint (when inside the 
%                neighborhood the waypoint is considered visited) 
%  wp_nxt        the index of the next waypoint (the initial value is 1)
%
% Example::
%
%    WP = [0 0; -10 -10];
%    veh = Bicycle();
%    veh.add_driver(hw4_GoThroughWPDriver(WP, 0.5));
%    poses.veh.run(110)
%
% See also Vehicle, Bicycle.

% (C) G. Fainekos ASU

classdef hw4_GoThroughWPDriver < handle
    properties
        veh             % the vehicle we are driving
        WP              % An array of waypoint coordinates. Each row contains the [x y] coordinates of each waypoint.
        d_wp            % Radius of neighborhood around each waypoint (when inside the neighborhood the waypoint is considered visited) 
        wp_nxt          % index of next waypoint (the initial value is 1)
    end

    methods

        function driver = hw4_GoThroughWPDriver(WP, d_wp)
            driver.WP = WP;
            driver.d_wp = d_wp;
            driver.wp_nxt = 1; % First waypoint to visit
        end

        function [vf, gamma] = demand(driver)
        % Compute velocity (vf) and steering angle (gamma)
        %
        % See also Vehicle, Bicycle
        
            % Current robot pose [x,y,theta]
            pose = driver.veh.x;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Replace the code below this line with your code 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % For this template, we skip the 1st waypoint and go directly
            % to the 2nd waypoint
            if driver.wp_nxt==1
                driver.wp_nxt = 2;
            end
            
            % Get the coordinates of the next waypoint
            wp_xy = driver.WP(driver.wp_nxt,:);

            % set forward velocity
            vf = 50;
            a = -(wp_xy(2)-driver.WP(driver.wp_nxt-1,2))
            b = (wp_xy(1)-driver.WP(driver.wp_nxt-1,1))
            c = -b*driver.WP(driver.wp_nxt-1,2)+(-a)*driver.WP(driver.wp_nxt-1,1)
            L_par = [a b c];   % The line parameters [a b c] 
                                % (this is only for the template model to work; 
                                % it is not needed for the homework)

            % define gains
            K1 = 9.5; % gain for distance
            K2 = 5.385; % gain for orientation

            % compute proportional control 
            dist = (pose(1)*L_par(1)+pose(2)*L_par(2)+L_par(3))/sqrt(L_par(1)^2+L_par(2)^2);
            slope = atan2(-L_par(1),L_par(2));
            ang = angdiff(slope, pose(3));
            gamma = K2*ang - K1*dist;
            
            %next waypoint
            if norm(wp_xy'-driver.veh.x(1:2)) < driver.d_wp
                if driver.wp_nxt < size(driver.WP,1)
                    driver.wp_nxt = driver.wp_nxt + 1;
                end
            end 
            
            % stop
            stop_id = norm(wp_xy'-driver.veh.x(1:2)) < driver.d_wp;
            if driver.wp_nxt == size(driver.WP,1)
                if stop_id
                    % stay at the same place
                    vf = 0;
                end
            end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % End of your modifications
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Do not modify below this line
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % called by Vehicle superclass
        function plot(driver)
            clf
            xmax = max(driver.WP(:,1))+driver.d_wp;
            xmin = min(driver.WP(:,1))-driver.d_wp;
            ymax = max(driver.WP(:,2))+driver.d_wp;
            ymin = min(driver.WP(:,2))-driver.d_wp;
            axis([xmin xmax ymin ymax]);
            hold on
            xlabel('x');
            ylabel('y');
        end

        function display(driver)
        %RandomPath.display Display driver parameters and state
        %
        % R.display() displays driver parameters and state in compact
        % human readable form.
        %
        % Notes::
        % - This method is invoked implicitly at the command line when the result
        %   of an expression is a hw4_GoThroughWPDriver object and the command has no trailing
        %   semicolon.
        %
        % See also hw4_GoThroughWPDriver.char.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(driver) );
        end % display()

        function s = char(driver)
        %hw4_GoThroughWPDriver.char Convert to string
        %
        % s = hw4_GoThroughWPDriver.char() is a string showing driver parameters and state in in 
        % a compact human readable format. 
            s = 'hw4_GoThroughWPDriver driver object';
            s = char(s, sprintf('  '));
        end
        
        function init(driver)
        % Nothing to initialize
        end
        
    end % methods
end % classdef
