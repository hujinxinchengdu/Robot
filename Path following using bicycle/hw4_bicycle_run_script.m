% Script : hw4_bicycle_run_script
% Initialize the variables for the driver class for the bicycle in HW4
%
% The script includes 3 different sets of waypoints for your practice

clear

x0 = [0 1 pi/2];    % Initial pose for the bicycle 
                    % (this will be modified during auto grading)
d_wp = 1;           % Radius of neighborhood around each waypoint 
                    % (when inside the neighborhood the waypoint is
                    % considered visited). You can set the value to
                    % somewhere between 0.5 and 0.9 to make sure you can
                    % visit the waypoint irrespective of any integration
                    % errors.
                    
% Instantiate a bicycle vehicle without noise steering limit 1 and max speed 1
dt = 0.2; % Simulation step size
veh = Bicycle('L', 1,'dt',dt,'steermax',1,'speedmax',1,'x0',x0);

% Set some dummy waypoint list for the template to run
WP0 = [0 0; -10 -10];
% Practice waypoint lists
% Each raw contains the x y coordinates of each waypoint
% The initial waypoint is the initial position of the robot
WP1 = [x0(1), x0(2); 0, 10; 10, 10; 10, 0; 0, 0]; % The square environment in the HW instructions
WP2 = [x0(1), x0(2); 15, 15; 20, 0; 5, 15; 0, 0];
WP3 = [x0(1), x0(2); 15, 15; 10, 0; 5, 15; 0, 0];
% Select waypoint:
WP = WP0; % set to dummy waypoint list

% Set the driver method
% The driver class provided only drives the robot across a line with two waypoints
veh.add_driver(hw4_GoThroughWPDriver(WP, d_wp));

% Run vehicle for 50 steps with total simulation time 100*dt sec. The
% vehicle will reach the waypoint and stop. For HW4 you will have to 
% increase the simulation steps to 220 steps or more 
sampPoints = 100;
% sampPoints = 220;
poses = veh.run(sampPoints);
% Add to poses the initial pose (not include)
poses = [x0; poses];

% plot the waypoints
ha = gca; % get the handle for a new axis
% compute some bounds on the graph
xmin = min([WP(:,1); x0(1)]);
xmax = max([WP(:,1); x0(1)]);
ymin = min([WP(:,2); x0(2)]);
ymax = max([WP(:,2); x0(2)]);
scl = 0.2; % some scale factor 
x_offset = scl*(xmax-xmin);
y_offset = scl*(ymax-ymin);
axis([xmin-x_offset xmax+x_offset ymin-y_offset ymax+y_offset])
hold on
plotWP(WP,d_wp) % plot the waypoints
axis square
grid on

% Plot results
plot_vehicle(poses,'axis',ha,'trail','r--')

% Get path error
t = 0:dt:(sampPoints*dt); % create time stamps
err = totalTrajError(WP,t,poses,d_wp)


