
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% "Jinxin", "Hu", "1207744664"
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
%R = SE2(22*rand,22*rand,0);
%p_tmp = Polygon(R*V);

for k = 1:size(scenario_sizes,2)
    P=[];
    prob(k) = 0;
    for t = 1:n_tests
        for j = 1:scenario_sizes(k)
            R(j) = SE2(22*rand,22*rand,2*pi*rand);
            p_tmp(j)=Polygon(R(j)*V);
            if j==1
                P=p_tmp(j);
            else
                P=[P,p_tmp];
            end
        end
        scenarios(k).map = Polygon2OccMap(P,range([2,4]));
        scenarios(k).start = k+2*rand(1,3);
        scenarios(k).goal = 2*k+3*rand(1,3);
        for l = 1:size(P,2)
            while (inpolygon(scenarios(k).start(1),scenarios(k).start(2),P(l).x,P(l).y) == 1 || inpolygon(scenarios(k).goal(1),scenarios(k).goal(2),P(l).x,P(l).y) == 1 )
                scenarios(k).start = k+3*rand(1,3);
                scenarios(k).goal = 2*k+5*rand(1,3);
            end
        end
        rrt = RRTmap(veh,scenarios(k).map,'root',scenarios(k).start,'range',range,'npoints',floor(n_nodes));
        while( rrt.isoccupied(rrt.root(1:2)))
            rrt.root = 2+3*rand(1,3);
        end
        rrt.plan(); % create the rrt
        G = rrt.graph; % extract the graph 
        % find the closest node to the goal
        for v = 1 : size(G.vertexlist,2)
            v_xytheta = G.coord(v);
            d = norm(v_xytheta(1:2)'-scenarios(k).goal(1:2));
            if d < dist
                prob(k) = prob(k) + 1;
                break;
            end
        end
        %[v,d] = G.closest(scenarios(k).goal); % find the closest node to the goal
        %xnear = rrt.graph.coord(v);     % coord of nearest vertex
        %d = norm(xnear(1:2)-scenarios(k).goal(1:2)); 
    end
    results(k)= prob(k)/n_tests;
end

% Remark: if the start or goal nodes are inside an obstacle, then an error
% will be issued. This is not a meaningful scenario to test since the
% algorithm will just time out with the maximum number of steps. You can
% try to capture these cases with try ... catch ... end and create a new
% scenario.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of modifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
