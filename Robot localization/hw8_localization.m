%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% "Jinxin ", "Hu"
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add your personal information in the line above
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Comments:
% 
% HW 8 Localization 
% 
% Inputs:
%   G       - An object of class PGraph modeling the connectivity of the 
%             road network
%   ProbObs - The probabilistic sensor model
%   ObsAct  - The sequence of pairs of observations and actions
%
% Outputs:
%   hist    - The history of probability distributions over the edges of
%             the road network after each observation and each action.
%             This is a m x n array where m is the number of observations -
%             actions pairs and n is the number of edges.
%
%%


function hist = hw8_localization(G,ProbObs,ObsAct)

% Create a placeholder to store the probability distribution for each step
% of the algorithm - one for observation updates and one for action updates
hist = zeros(2*size(ObsAct,1), G.ne);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modify the code below this line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Go over each action 
for t = 1:size(ObsAct,1)
    
    %% Observation update
    
    % Your code here
    yita = 0;
    Bel_new = zeros(1,size(G.edgelen,2));
    for j = 1:G.ne
        Bel(j) = G.cost(j);
        if Bel(j) ~=0
            if isequal(ObsAct{t,1},1)
                if isequal(G.vertexdata{G.edgelist(2,j)},1)
                    Bel_new(j) = ProbObs(1,1) * Bel(j);
                elseif isequal(G.vertexdata{G.edgelist(2,j)},2)
                    Bel_new(j) = ProbObs(1,2) * Bel(j);
                elseif isequal(G.vertexdata{G.edgelist(2,j)},3)
                    Bel_new(j) = ProbObs(1,3) * Bel(j);
                end
            elseif isequal(ObsAct{t,1},2)
                if isequal(G.vertexdata{G.edgelist(2,j)},1)
                    Bel_new(j) = ProbObs(2,1) * Bel(j);
                elseif isequal(G.vertexdata{G.edgelist(2,j)},2)
                    Bel_new(j) = ProbObs(2,2) * Bel(j);
                elseif isequal(G.vertexdata{G.edgelist(2,j)},3)
                    Bel_new(j) = ProbObs(2,3) * Bel(j);
                end
            elseif isequal(ObsAct{t,1},3)
                if isequal(G.vertexdata{G.edgelist(2,j)},1)
                    Bel_new(j) = ProbObs(3,1) * Bel(j);
                elseif isequal(G.vertexdata{G.edgelist(2,j)},2)
                    Bel_new(j) = ProbObs(3,2) * Bel(j);
                elseif isequal(G.vertexdata{G.edgelist(2,j)},3)
                    Bel_new(j) = ProbObs(3,3) * Bel(j);
                end
            end
            yita = yita + Bel_new(j);
        end
    end
    for j = 1:G.ne
        Bel_new(j) = Bel_new(j) / yita;
        G.setcost(j,Bel_new(j));
    end   
    %% Save distribution after observation 
    for e = 1:G.ne
        hist(2*t-1,e) = G.cost(e);
    end
    
    %% Action updates
    % Working assumption: flat earth => if the action update leads to a 
    % new state outside the road network, then this prediction disappears
    % (falls off the edge of earth)
    
    % Your code here
    Bel_new = zeros(1,size(G.edgelen,2));
    for j = 1:G.ne
        for k = 1:G.ne
            if isequal(ObsAct{t,2},'s') && G.edgelist(1,j) == G.edgelist(2,k) && (abs(G.edgelist(2,j) - G.edgelist(1,k)) == 2 || abs(G.edgelist(2,j) - G.edgelist(1,k)) == 8)
                Bel_new(j) = Bel_new(j) + G.cost(k);
            elseif isequal(ObsAct{t,2},'l') && G.edgelist(1,j) == G.edgelist(2,k) && abs(G.edgelist(1,j) - G.edgelist(2,j)) == 1 && abs(G.edgelist(2,j) - G.edgelist(1,k)) == 5
                Bel_new(j) = Bel_new(j) + G.cost(k);
            elseif isequal(ObsAct{t,2},'l') && G.edgelist(1,j) == G.edgelist(2,k) && abs(G.edgelist(1,j) - G.edgelist(2,j)) == 4 && abs(G.edgelist(2,j) - G.edgelist(1,k)) == 3
                Bel_new(j) = Bel_new(j) + G.cost(k);
            elseif isequal(ObsAct{t,2},'r') && G.edgelist(1,j) == G.edgelist(2,k) && abs(G.edgelist(1,j) - G.edgelist(2,j)) == 1 && abs(G.edgelist(2,j) - G.edgelist(1,k)) == 3
                Bel_new(j) = Bel_new(j) + G.cost(k);
            elseif isequal(ObsAct{t,2},'r') && G.edgelist(1,j) == G.edgelist(2,k) && abs(G.edgelist(1,j) - G.edgelist(2,j)) == 4 && abs(G.edgelist(2,j) - G.edgelist(1,k)) == 5
                Bel_new(j) = Bel_new(j) + G.cost(k);
            end
        end
    end
    for j = 1:G.ne
        G.setcost(j,Bel_new(j));
    end
    %% Save distribution after action 
    for e = 1:G.ne
        hist(2*t,e) = G.cost(e);
    end
        
end

end

