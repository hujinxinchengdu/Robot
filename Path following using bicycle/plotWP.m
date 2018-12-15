% PlotWP
% Plot the sequence of waypoints and the local neighborhoods
%
% Inputs: 
%   WP - Sequence of waypoints; Each raw contains the x y coordinates of 
%        each waypoint
%   d_wp  - Radius of neighborhood around each waypoint 

% (C) G. Fainekos - ASU

function plotWP(WP,r)
grid on
hold on
plot(WP(:,1),WP(:,2))
for ii = 1:size(WP,1)
    plot(WP(ii,1),WP(ii,2),'r*')
    circle([WP(ii,1),WP(ii,2)],r,'b')
end
