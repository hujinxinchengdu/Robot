% This script produces the data for computing the probability of success of
% the RRT algorithm on a given problem instance. 
% 
% The data generated by this file will be modified by the autograder.

clear 

start = [6 5 0];        % Initial pose for the bicycle (Case 1)
% start = [15 10 0];    % Initial pose for the bicycle (Case 2)
goal = [23 14 pi/2];    % Goal pose for the bicycle 
dist = 1;               % Distance from goal for successful run

% Instantiate a bicycle vehicle without noise, with steering limit 1.2 and
% max speed 2.5
veh = Bicycle('L', 1,'steermax',1.2,'speedmax',2.5,'x0',start);

% Define a map with polygonal obstacles
V1 = [4,10; 10,10; 10,4; 12,4; 12,12; 2,12; 2,6; 4,4];
p1 = Polygon(V1');
V2 = [17,13; 19,7; 22,8; 20,23; 18,23];
p2 = Polygon(V2');
P = [p1 p2];
% Create an occupancy grid from the polygonal environment
maxRange = [25,25]; % maximum range for x and y position
map = Polygon2OccMap(P,maxRange);

% The bounding box over which RRT samples for adding new points:
% range = [xmin, xmax. ymin, ymax]
range = [0, maxRange(1), 0, maxRange(2)]; 

n_tests = 10;       % Number of times to run the RRT algorithm (you can modify for your testing)
n_nodes = 1000;     % Max number of nodes for the RRT 

%% For your own use (your submission should not plot anything)
% Plot the environment and an RRT tree to visualize how RRT works
rrt = RRTmap(veh,map,'root',start,'range',range,'npoints',floor(n_nodes));
rrt.plan(); % create the rrt
figure
rrt.plot;
hold on
plot(map)
plot(P) % to visualize the rough overapproximation of the obstacles using a grid world
figure
rrt.query(start,goal)
hold on 
plot(P)
%%

%% Call your function and return the rate of success
prob_success = hw6_prob_of_success(n_tests,n_nodes,map,veh,start,goal,dist,range)


