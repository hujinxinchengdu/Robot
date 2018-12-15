% HW3 template
%
% Inputs:
%
%   rel_rob1_pos : an m x 6 array where m is the number of points in the 
%       robot trajectory. Each row consists of x1, y1, z1, x2, y2, z2 where
%       (x1, y1, z1) are the coordinates of the first marker of robot 1 and
%       (x2, y2, z2) are the coordinates of the second marker of robot 1
%       with respect to the "camera" frame {C}. 
%
%   rel_rob2_pos : an m x 2 array where m is the number of points in the 
%       robot trajectory. Each row consists of (x, y) which is the position
%       of robot 2 relative to the position of robot 1.
%
% Outputs
%
%   gl_rob1_pos : an m by 3 array where m is the number of points in the 
%       robot trajectory. Each row consists of x, y and theta where x,y is
%       the position of robot 1 and theta is the orientation of the
%       robot 1 with respect to the world frame placed at (0,0,0)
%
%   g2_rob2_pos : an m by 2 array where m is the number of points in the 
%       robot trajectory. Each row consists of x and y where x,y is
%       the position of robot 2 with respect to the world frame placed 
%       at (0,0,0)
%   
% Both robots are operating at z = 0.
%
% Useful functions: 
%   transl, trotz, troty, trotx, atan2, SE2, SO2, SE3, SO3
% type "help <function>" where <function> is a function from the above list
% to read details on each function.
%
% If you would like to plot the output type the following in the command
% window:
% figure
% plot(gl_rob1_pos(:,1),gl_rob1_pos(:,2),gl_rob2_pos(:,1),gl_rob2_pos(:,2),'r')
% grid on 

function [gl_rob1_pos, gl_rob2_pos] = HW3GetRobotTrajectories(rel_rob1_pos, rel_rob2_pos)

[m,~] = size(rel_rob1_pos); % Get the number of trajectory points

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Replace with your code below
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % For robot1, just return the data of marker 1 and zeros for orientation
% gl_rob1_pos = [rel_rob1_pos(:,1:2) zeros(m,1)]; 
% % For robot2, just return the input data 
% gl_rob2_pos = rel_rob2_pos;
% For robot1

% the "camera" frame {C} with respect to the world frame
OriginalLocation = SE3(-5,-5,5)*SE3.Rz(pi/4)*SE3.Ry(pi/3);

for i = 1:m
    % change the coordinates(x,y,z) of first marker to the original x,y,z
    mw1(i,1:3) = (OriginalLocation*rel_rob1_pos(i,1:3)')';
    % change the coordinates(x,y,z) of second marker to the original x,y,z
    mw2(i,1:3) = (OriginalLocation*rel_rob1_pos(i,4:6)')';
    % find the distance of robot 1 respect original coordinates(x,y) position.
    gl_rob1_pos(i,1:2)=mw1(i,1:2);
    % find the angal of robot 1 respect original coordinates x position.
    gl_rob1_pos(i,3)=atan2(mw2(i,2)-mw1(i,2),mw2(i,1)-mw1(i,1));
end

% For robot2

for i =1:m
    %Find the angal of the coordinate change from robot 1. 
    xCoordinate = mw1(i,1);
    yCoordinate = mw1(i,2);
    A = atan2(mw2(i,2)-mw1(i,2),mw2(i,1)-mw1(i,1));
    %change the coordinate of robot 2 to original one.
    Xuanzhuan = SE2(xCoordinate,yCoordinate,0)*SE2(0,0,A);
    %find the position of robot 2 which in the original coordinate.
    gl_rob2_pos(i,1:2)=(Xuanzhuan*rel_rob2_pos(i,1:2)')';
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Replace with your code above
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end

