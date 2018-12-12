% This is the template for HW5. You need to modify the m-function so that
% it creates a visibility graph given the start point (s), goal (g) and 
% a vector of polygons P.
%
% You can assume that any two vertices are at least 1 unit far away from 
% each other.
%
% Output:
%   G - an object of class PGraph
%   vs - the node which corresponds to the start position
%   vg - the node which corresponds to the goal position
%
% Remark: Polygon class expects all the arrays containing m points on the
% plane to be 2 x m.
% E.g. A = [0 1 2; 0 1 0];
% The array A contains the points p1 = [0; 0], p2 = [1; 1] and p3 = [2; 0]
% where p = [x; y].

function [G,vs,vg] = HW5VisGraph_ASUID(P,s,g)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add your code below this line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create a trivial weighted graph with the start and goal nodes 
G = PGraph(2);

% Create nodes
vs = G.add_node(s);
num = 1;
for count_shap = 1:size(P,2)
    for count_shapPoint = 1:size(P(count_shap).vertices,2)
        num = num + 1;
        %Id for each vertices
        idNUM(num) = G.add_node(P(count_shap).vertices(:,count_shapPoint)');
    end
end
vg = G.add_node(g);

%start point to visible vertex
for i = 2:(size(G.vertexlist,2) - 1)
    flag = 0;
    for counter = 0.01:0.01:0.99
        if flag ~= 1
            xq = counter * G.vertexlist(1,1) + (1 - counter) * G.vertexlist(1,i);
            yq = counter * G.vertexlist(2,1) + (1 - counter) * G.vertexlist(2,i);
            for counter2 = 1:size(P,2)
                in = inpolygon(xq,yq,P(counter2).x,P(counter2).y);
                if in ~= 1 
                    continue;
                elseif in == 1
                    flag = 1;
                    break;
                else
                    continue;
                end
            end
        elseif flag == 1 
            break;
        else
            continue;
        end
     end
     if flag == 0
        G.add_edge(vs,idNUM(i));
     end
end

%visible vertex to goal point
for i = 2:(size(G.vertexlist,2) - 1)
    flag = 0;
    for counter = 0.01:0.01:0.99
        if flag ~= 1
            xq = counter * G.vertexlist(1,vg) + (1 - counter) * G.vertexlist(1,i);
            yq = counter * G.vertexlist(2,vg) + (1 - counter) * G.vertexlist(2,i);
            for counter2 = 1:size(P,2)
                in = inpolygon(xq,yq,P(counter2).x,P(counter2).y);
                if in ~= 1 
                    continue;
                elseif in == 1
                    flag = 1;
                    break;
                else
                    continue;
                end
            end
        elseif flag == 1 
            break;
        else
            continue;
        end
     end
     if flag == 0
        G.add_edge(idNUM(i),vg);
     end
end

% vertex to visible vertex
for i = 2:(size(G.vertexlist,2) - 1)
    for j = (i + 1):(size(G.vertexlist,2) - 1)
        flag = 0;
        for counter = 0.01:0.01:0.99
            if flag ~= 1
                xq = counter * G.vertexlist(1,i) + (1 - counter) * G.vertexlist(1,j);
                yq = counter * G.vertexlist(2,i) + (1 - counter) * G.vertexlist(2,j);
                for counter2 = 1:size(P,2)
                    [in,on] = inpolygon(xq,yq,P(counter2).x,P(counter2).y);
                    if in == 1 
                        if on == 0
                            flag = 1;
                        break;
                        end
                    end
                end
            elseif flag == 1 
                break;
            else
                continue;
            end
        end
       if flag == 0
        G.add_edge(idNUM(i),idNUM(j));
       end
    end
end

% start point to goal point
    flag = 0;
    for counter = 0.01:0.01:0.99
        if flag ~= 1
            xq = counter * G.vertexlist(1,1) + (1 - counter) * G.vertexlist(1,vg);
            yq = counter * G.vertexlist(2,1) + (1 - counter) * G.vertexlist(2,vg);
            for counter2 = 1:size(P,2)
                in = inpolygon(xq,yq,P(counter2).x,P(counter2).y);
                if in ~= 1 
                    continue;
                elseif in == 1
                    flag = 1;
                    break;
                else
                    continue;
                end
            end
        elseif flag == 1 
            break;
        else
            continue;
        end
    end
if flag == 0
    G.add_edge(vs,vg);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of your modifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
