% plot_vehicle_poses Plots a sequence of vehicle poses 
%
% plot_vehicle_poses(x,opt)
% 
% Inputs
%
%   X - An n by 3 array with the poses of the vehicle [x,y,theta].
%
%   opt - ID figure - if not set a new figure will be generated 
%
% See also plot_vehicle, animate_vehicle_poses

% (C) G. Fainekos ASU

function plot_vehicle_poses(x,opt)

if nargin==1 || isempty(opt)
    id_fig = figure;
    opt = id_fig.Number;
end

figure(opt)

hold on 

[m,~] = size(x);
xmax = max(x(:,1));
xmin = min(x(:,1));
ymax = max(x(:,2));
ymin = min(x(:,2));
if xmin==xmax
    xmin = -(ymax-ymin)/10;
    xmax = -xmin;
end
if ymin==ymax
    ymin = -(xmax-xmin)/10;
    ymax = -ymin;
end

scl_f = 0.2;
axis([xmin-(xmax-xmin)*scl_f xmax+(xmax-xmin)*scl_f ymin-(ymax-ymin)*scl_f ymax+(ymax-ymin)*scl_f])
axis equal

for i = 1:m
    plot_vehicle(x(i,:))
end

end

