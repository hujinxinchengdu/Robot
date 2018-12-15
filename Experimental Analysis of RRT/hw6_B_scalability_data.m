% This script produces the data for studying the scalability of the RRT 
% algorithm. 
%
% The data generated by this file will be modified by the autograder.

clear 

dist = 1;           % Distance from goal for successful run

% Instantiate a bicycle vehicle without noise, with steering limit 1.2 and
% max speed 2.5
veh = Bicycle('L', 1,'steermax',1.2,'speedmax',2.5);

% The vertices of the basic polygon
V = [0,0; 0,3; 3,3; 3,0]';
% You can translate the x position by using the instruction V(1,:) = V(1,:)+x;
% You can translate the y position by using the instruction V(2,:) = V(2,:)+y;
% (Or you can use translations and rotations as we did in HW3 to both
% translate and rotate the obstacles)

% The bounding box over which RRT samples for adding new points:
% range = [xmin, xmax. ymin, ymax]
range = [0, 25, 0, 25]; 

n_tests = 10;       % Number of times to run the RRT algorithm (you can modify for your testing)
n_nodes = 500;      % Max number of nodes for the RRT (you can modify for your testing but computation time increases)

scenario_sizes = [0 2 4 6]; % The number of obstacles for each scenario (just an example)

%% Call your function and return the results of your analysis
[results,scenarios] = hw6_scalability(scenario_sizes,n_tests,n_nodes,V,veh,dist,range)

%% Run one of the scenarios for visualization
% This is not a required part of the homework assignment
k = 2;
rrt = RRTmap(veh,scenarios(k).map,'root',scenarios(k).start,'range',range,'npoints',floor(n_nodes));
rrt.plan(); % create the rrt
G = rrt.graph; % extract the graph 
v = G.closest(scenarios(2).goal); % find the closest node to the goal
%% Plot one scenario for debugging 
% For your own use (your submission should not plot anything)
figure
rrt.plot;
hold on
plot3(scenarios(k).goal(1),scenarios(k).goal(2),scenarios(k).goal(3),'ro')
%%