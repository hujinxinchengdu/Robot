%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% "First Name", "Last Name", "ASUID #"
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add your personal information in the line above
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Comments:
% 
% HW 6 Statistical analysis 1
% 
% Inputs:
%   n_tests         : The total number of RRT executions
%   n_nodes         : The maximum number of nodes in the tree
%   map             : An occupancy map
%   veh             : A vehicle object
%   start           : The start position for the robot
%   goal            : The goal position for the robot
%   dist            : Maximum distance on the x-y plane from the goal for 
%                     establishing success of the RRT algorithm. In other
%                     words, RRT must generate a node within distance dist
%                     from the goal for the RRT run to be considered as 
%                     successful.
%   range           : The bounding box over which RRT samples for adding 
%                     new points: range = [xmin, xmax. ymin, ymax]
%
% Outputs:
%   prob_success    : Probability of success
%%

function prob_success = hw6_prob_of_success(n_tests,n_nodes,map,veh,start,goal,dist,range)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modify the code below this line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Sample code to use
% rrt = RRTmap(veh,map,'root',start,'range',range,'npoints',floor(n_nodes));
% rrt.plan(); % create the rrt
% G = rrt.graph; % extract the graph 
% % find the closest node to the goal, you will need this to compute the
% % probability of success
% v = G.closest(goal); 
prob_success = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of modifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
