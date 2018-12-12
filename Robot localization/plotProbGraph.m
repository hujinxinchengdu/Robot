% Plot probability distribution over the graph edges
% Input: 
%	G: A graph of class PGraph which represents a grid road network
% Output: 
%	Z: the values for the 3D bar 

function Z = plotProbGraph(G)
    % Pass once to get max coordinates
    maxX = -inf;
    maxY = -inf;
    for v = 1:G.n
        x = G.coord(v);
        if x(1)>maxX
            maxX = x(1);
        end
        if x(2)>maxY
            maxY = x(2);
        end
    end
    % initialize array for bars
    % Z = zeros(3*(maxX-1)+2,3*(maxY-1)+2);
    Z = -0.025*ones(3*(maxX-1)+2,3*(maxY-1)+2);
    % populate probabilities over road segments (edges)
    for v2 = 1:G.n
        x2 = G.coord(v2);
        % Get incoming edges to each node (can also work with outgoing)
        E = G.edges_in(v2);
        % find most likely edge
        for k = 1:length(E)
            e = E(k);
            v = G.vertices(e);
            v1 = v(1);
            x1 = G.coord(v1);
            if x1(2)-x2(2)==0 % Road segment parallel to the x axis?
                i = 3*min([x1(1) x2(1)]);
                if x1(1)<x2(1)
                    j_tmp = 1;
                else
                    j_tmp = 2;
                end
                j = 3*(min([x1(2) x2(2)])-1)+j_tmp;
            elseif x1(1)-x2(1)==0 % Road segment parallel to the y axis?
                j = 3*min([x1(2) x2(2)]);
                if x1(2)<x2(2)
                    i_tmp = 2;
                else
                    i_tmp = 1;
                end
                i = 3*(min([x1(1) x2(1)])-1)+i_tmp;
            else
                error('Case not possible: Diagonal movement')
            end
            Z(i,j) = G.cost(e);
        end
    end
    % plot bars 
    b = bar3(Z);
    % modify colors based on height
    for k = 1:length(b)
        zdata = b(k).ZData;
        b(k).CData = zdata;
        b(k).FaceColor = 'interp';
    end
end
