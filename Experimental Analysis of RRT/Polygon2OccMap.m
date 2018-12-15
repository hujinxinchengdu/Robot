% Translate a Polygon object to an occupancy map within the specified range
%
% Inputs:
%   P - an array of polygons
%   Range - [x y] 
%           x and y are integers specifying the maximum x and y range 
%           for the occupancy map. An occupancy map always starts from
%           [1,1].

function map = Polygon2OccMap(P,Range)

map = zeros(Range);

for i = 1:length(P)
    for j = 1:Range(1)
        for k = 1:Range(2)
            p_tmp = P(i);
            if p_tmp.intersect(Polygon([j-0.5,k-0.5; j+0.5,k-0.5; j+0.5,k+0.5; j-0.5,k+0.5]')) || p_tmp.inside([j;k])
                map(j,k) = 1;
            end
        end
    end
end

map = map';

end

