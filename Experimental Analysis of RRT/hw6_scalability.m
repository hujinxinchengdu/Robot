%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% "First Name", "Last Name", "ASUID #"
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add your personal information in the line above
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Comments:
% 
% HW 6 Statistical analysis 2
% 
% Inputs:
%   scenario_sizes  : A vector with the number of obstacles for each
%                     scenario 
%   n_tests         : The total number of RRT executions
%   n_nodes         : The maximum number of nodes in the tree
%   V               : The vertices of the polygon to be replicated,
%                     translated and/or rotated
%   veh             : A vehicle object
%   dist            : Maximum distance on the x-y plane from the goal for 
%                     establishing success of the RRT algorithm. In other
%                     words, RRT must generate a node within distance dist
%                     from the goal for the RRT run to be considered as 
%                     successful.
%   range           : The bounding box over which RRT samples for adding 
%                     new points: range = [xmin, xmax. ymin, ymax]
%
% Outputs:
%   results         : A vector holding the probability of success for each
%                     scenario
%   scenarios       : An array structure with the following fields:
%       map             : The occupancy map for each scenario
%       start           : The start position for the robot
%       goal            : The goal position for the robot
%%

function [results,scenarios] = hw6_scalability(scenario_sizes,n_tests,n_nodes,V,veh,dist,range)

% Create a structure to store the scenarios
scenarios.map = [];
scenarios.start = [];
scenarios.goal = [];

% Create an array to store the success rate for each scenario
results = zeros(1,length(scenario_sizes));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modify the code below this line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Example: of a random translation and rotation
% You need to change the rotation and translation to something meaningful
% that respects the search range
R = SE2(rand,10+2*rand,rand);
p_tmp = Polygon(R*V);

% This section of the code needs to be modified based on the input array scenario_sizes
% You can change the dimension of the array structure dynamically
% Some random scenarios:
% scenario 1
scenarios(1).map = Polygon2OccMap(Polygon(V),range([2,4]));
scenarios(1).start = [10 10 10];
scenarios(1).goal = [0 0 0];
% scenario 2
scenarios(2).map = Polygon2OccMap([Polygon(V) p_tmp],range([2,4]));
scenarios(2).start = 5+5*rand(1,3);
scenarios(2).goal = 10+rand(1,3);
% Remark: if the start or goal nodes are inside an obstacle, then an error
% will be issued. This is not a meaningful scenario to test since the
% algorithm will just time out with the maximum number of steps. You can
% try to capture these cases with try ... catch ... end and create a new
% scenario.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of modifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
