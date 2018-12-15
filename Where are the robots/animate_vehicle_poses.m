% animate_vehicle_poses Animates a sequence of vehicle poses 
%
% M = animate_vehicle_poses(X,opt,filename) 
% 
% Inputs
% 
%   X - An n by 3 array with the poses of the vehicle [x,y,theta].
%
%   opt(1) - 1 keep or 0 do not keep the previous pose on the animation 
%   opt(2) - ID figure - if not set a new figure will be generated; Set to
%       zero if you would like a new figure to be created.
%
%   filename - If a filename is provided as a string, then the animation is 
%       saved as an MPEG file under the filename file.
%
% Outputs
%  
%   M - If no file name is provided, then M contains the frames for the
%       animation. You can use the m-function "movie" to animate.
%
% See also plot_vehicle, plot_vehicle_poses, movie

% (C) G. Fainekos ASU

function M = animate_vehicle_poses(x,opt,filename)

if nargin==1 || isempty(opt)
    opt(1) = 0;
    id_fig = figure;
    opt(2) = id_fig.Number;
end

if opt(2)==0
    id_fig = figure;
    opt(2) = id_fig.Number;
end

figure(opt(2))

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

if (nargin<=2 || isempty(filename))
    for i = 1:m
        plot_vehicle(x(i,:));
        M(i) = getframe;
        if ~opt(1)
            hold off
            clf
            axis([xmin-(xmax-xmin)*scl_f xmax+(xmax-xmin)*scl_f ymin-(ymax-ymin)*scl_f ymax+(ymax-ymin)*scl_f])
            axis equal
        end
    end
elseif (nargin==3)
    M = [];
    % Prepare the new file.
    vidObj = VideoWriter(filename);
    open(vidObj);
 
    % Create an animation.
    for i = 1:m
        plot_vehicle(x(i,:));
        currFrame  = getframe;
        writeVideo(vidObj,currFrame);
        if ~opt(1)
            hold off
            clf
            axis([xmin-(xmax-xmin)*scl_f xmax+(xmax-xmin)*scl_f ymin-(ymax-ymin)*scl_f ymax+(ymax-ymin)*scl_f])
            axis equal
        end
    end
  
    % Close the file.
    close(vidObj);
end

end