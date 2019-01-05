%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% "Jinxin", "Hu", "1207744664"
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
for k = 1:3
    prob1(k) = 0;
    prob2(k) = 0;
    for t = 1 : n_tests     
        rrt1 = RRTmap(veh,scenarios(k).map,'root',scenarios(k).start,'range',range,'npoints',floor(n_nodes));
        rrt1.plan(); % create the rrt
        G1 = rrt1.graph; % extract the graph
        %[v1,d1] = G1.closest(scenarios(k).goal); % find the closest node to the goal
        for v = 1 : size(G1.vertexlist,2)
            v_xytheta1 = G1.coord(v);
            d1 = norm(v_xytheta1(1:2)'-scenarios(k).goal(1:2));
            if d1 < dist
                prob1(k) = prob1(k) + 1;
                break;
            end
        end
        
        rrt2 = RRTgoal(veh,scenarios(k).map,'root',scenarios(k).start,'goal',scenarios(k).goal,'range',range,'npoints',floor(n_nodes));
        rrt2.plan(); % create the rrt
        G2 = rrt2.graph; % extract the graph
        %[v2,d2] = G2.closest(scenarios(k).goal); % find the closest node to the goal
        for v = 1 : size(G2.vertexlist,2)
            v_xytheta2 = G2.coord(v);
            d2 = norm(v_xytheta2(1:2)'-scenarios(k).goal(1:2));
            if d2 < dist
                prob2(k) = prob2(k) + 1;
                break;
            end
        end
    end
    if prob2(k) >= prob1(k)
        better_RRT(k) = 1;
    else 
        better_RRT(k) = 0;
    end
    results(1,k)= prob1(k)/n_tests;
    results(2,k)= prob2(k)/n_tests;
end
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of modifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
