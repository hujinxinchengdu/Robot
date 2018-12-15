% Script for running Homework 8 - Localization 
%
% The script creates the inputs for hw8_localization and plots the results.
% Data created:
%   G       - An object of class PGraph modeling the connectivity of the 
%             road network
%   ProbObs - The probabilistic sensor model
%   ObsAct  - The sequence of pairs of observations and actions
%%
% ASU CSE494 2018 (c) G. Fainekos

clear 

%% Data for practice

% Actions and observations

% Probabilistic sensing model
% ProbObs = [1 0 0; 0 1 0; 0 0 1]; % No sensing uncertainty  
ProbObs = [0.8 0.2 0; 0.3 0.6 0.1; 0.2 0.1 0.7]; % some sensing uncertainty 

% scenario 1 - Two possible solutions starting from edge (14,10)
ObsAct = {2 's'; 3 'r'; 1 'l'};

% scenario 2 - As scenario 1, but with a different sensor measurement at step 2 
% ObsAct = {2 's'; 2 'r'; 1 'l'};

% scenario 3 - As scenario 2, but with two more observations
% ObsAct = {2 's'; 2 'r'; 1 'l'; 3 'r'; 1 'r'};

% Map of colors of spheres at each intersection
ColorMap = [1 2 3 1; 2 3 1 2; 2 2 2 2; 1 2 3 2];
% ColorMap = [1 2 3 1 1; 2 3 1 2 2; 2 2 2 2 1; 1 2 3 2 3];
% ColorMap = [1 2 3 1; 2 3 1 2; 2 2 2 2; 1 2 3 2; 1 2 1 2];

%% Construct a PGraph from the color map
G = PGraph(2);
% Add nodes
for i = 1:size(ColorMap,1)
    for j = 1:size(ColorMap,2)
        v = G.add_node([j,size(ColorMap,1)-i+1]);
        % Each node is labeled by the intersection color
		% You can read the data by calling G.vdata(v)
		G.setvdata(v,ColorMap(i,j)); 
    end
end
% Add edges (quick and dirty; can be done better)
for i = 1:numel(ColorMap)
    for j = (i+1):numel(ColorMap)
        if G.distance(i,j)<=1 
            G.add_edge(i,j);
            G.add_edge(j,i);
        end
    end
end
% Assign the initial probability for each edge: each edge equally likely
for e = 1:G.ne
    G.setcost(e,1/G.ne);
end
% plot the graph
figure(1)
clf
plot(G,'labels')
% print the color label of each node 
for v = 1:G.n
    x = G.coord(v);
    text(x(1)+0.05,x(2)+0.15,['\bf ',num2str(G.vdata(v))],'Color','red','FontSize',14)
end
% plot the initial probability distribution
figure(2)
clf
plotProbGraph(G);
title('Initial Probability Distribution')

%%
% Run your code
hist = hw8_localization(G,ProbObs,ObsAct);

%% visualize probability distribution / results
for i = 1:size(hist,1)

    % set edge weights for visualization
    for e = 1:G.ne
        G.setcost(e,hist(i,e));
    end
    
    % visualize results
    figure(i+2)
    clf
    plotProbGraph(G);
    if mod(i,2)==1 
        title(['Step ',num2str(floor(i/2)),' observation'])
    else
        title(['Step ',num2str(i/2),' movement'])
    end

end

% highlight the most likely edges on the graph
figure(1)
[maxProb,whichEdge] = getMostLikelyEdge(G);
for i = 1:length(whichEdge)
    G.highlight_edge(whichEdge(i),'EdgeColor','r')
end

%% This function can only be called by hw8_runme_script.m
% Returns the most likely edge on the graph
function [maxProb,whichEdge] = getMostLikelyEdge(G)
    maxProb = 0;
    whichEdge = [];
    for j = 1:G.n
        % Get incoming edges to each node (can also work with outgoing)
        E = G.edges_in(j);
        % find most likely edge
        for k = 1:length(E)
            e = E(k);
            if G.cost(e)>=maxProb
                if G.cost(e)==maxProb
                    whichEdge = union(whichEdge,e);
                else
                    maxProb = G.cost(e);
                    whichEdge = e;
                end
            end
        end
    end
end


