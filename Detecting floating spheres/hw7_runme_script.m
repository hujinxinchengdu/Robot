% This script produces some training data for color classification and 
% sphere identification for hw7_ball_identification
%
% Data created for hw7_ball_identification:
%   t_data : training data. An m x 4 array of m training tuples of RGB 
%            values and the corresponding numeric labels.
%            Possible label values: 
%               0 - none
%               1 - purple
%               2 - red
%               3 - yellow
%           Example of data:
%               t_data = [160 173 206 0; 238 31 31 2];
%  im_data : a cell array of image data. Each cell is an array as returned 
%            by the "imread" function.
%
% Data received from hw7_ball_identification:
%   label : A vector with the label of the sphere for each image; the 
%           cell entry should be 0 if no sphere was detected
%   bbox : A cell vector with the bounding box coordinates for the detected 
%          sphere in the image; format [umin umax vmin vmax], where
%               umin is the minimum horizontal coordinate
%               umax is the maximum horizontal coordinate
%               vmin is the minimum vertical coordinate
%               vmax is the maximum vertical coordinate
%          return empty ([]) if no sphere was detected
%  road : [for the bonus points] A cell vector with a binary mask for each 
%		  image identifying the empty space on the road.
% 
% 
% The data generated by this file will be modified by the autograder. You
% need to change the data in this file to fully try your code.

% ASU CSE494 2018 (c) G. Fainekos

clear 

%%
% Create training data just for red color - you need to add the other
% colors as well

% Images and identified regions to crop training data from
t_images = {'image_30_red.png',204:220,250:280;...
    'image_10_red.png',84:175,100:160};
% produce the array with training data
t_data = [];
for i = 1:size(t_images,1)
    im = iread(t_images{i,1});
    im = im(t_images{i,2},t_images{i,3},:); % crop image to the desired region
    m = length(t_images{i,2});
    n = length(t_images{i,3});
    % create a temp array to store the concatenated training data
    % note 1: the last column is 2 because the cropped images are for the red
    % color for this demo
    % note 2: you may want to add some noise on the data for a more robust
    % classifier
    tmp = [zeros(m*n,3) 2*ones(m*n,1)]; 
    for j = 1:n
        tmp(((j-1)*m+1):(j*m),1:3) = im(:,j,:); 
    end
    t_data = [t_data; tmp]; %#ok<AGROW>
end

%%
% load some images for detecting spheres
im_data{1} = imread('image_10_red.png');
im_data{2} = imread('image_10_yellow.png');

%% Call your function and return the detected spheres
[label,bbox] = hw7_ball_identification(t_data,im_data);

%% Plot the position of the sphere
figure
imshow(im_data{1})
hold on
plot([bbox{1}(1) bbox{1}(2)],[bbox{1}(3) bbox{1}(4)],'b')
plot([bbox{1}(1) bbox{1}(2)],[bbox{1}(4) bbox{1}(3)],'b')
