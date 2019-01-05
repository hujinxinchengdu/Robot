%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% "Jinxin", "Hu", "1207744664"
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
prob = 0;
for i = 1:n_tests
    rrt = RRTmap(veh,map,'root',start,'range',range,'npoints',floor(n_nodes));
    rrt.plan(); % create the rrt
    rrt.query(start,goal)

    % find the closest node to the goal, you will need this to compute the
    % probability of success
    for v = 1 : size(rrt.graph.vertexlist,2)
        v_xytheta = rrt.graph.coord(v);
        d = norm(v_xytheta(1:2)'-goal(1:2));
        if d < dist
            prob = prob + 1;
            break;
        end
    end
    %[v,d] = rrt.graph.closest(goal);    % nearest vertex
    %xnear = rrt.graph.coord(v)     % coord of nearest vertex
    %d = rrt.graph.distance_metric(xnear, goal');
    %d = norm(xnear(1:2)-goal(1:2));
end
prob_success = prob/n_tests;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of modifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
