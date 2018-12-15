%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% "First Name", "Last Name", "ASUID #"
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add your personal information in the line above
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Comments:
% 
% HW 6 Statistical analysis 3
% 
% Inputs:
%   scenarios       : An array structure with the following fields:
%       map             : An occupancy map
%       start           : The start position for the robot
%       goal            : The goal position for the robot
%   n_tests         : The total number of RRT executions
%   n_nodes         : The maximum number of nodes in the tree
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
%   better_RRT      : Boolean variable
%                      0 - plain RRT is better
%                      1 - modified RRT is better
%   results         : A vector holding the probability of success for each
%                     scenario
%%

function [better_RRT,results] = hw6_comparison(scenarios,n_tests,n_nodes,veh,dist,range)

% Create an array to store the success rate for each scenario
results = zeros(1,length(scenarios));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modify the code below this line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

better_RRT = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of modifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
