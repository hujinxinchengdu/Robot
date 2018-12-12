%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% "Jinxin", "Hu", "1207744664"
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add your personal information in the line above
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Comments:
% 
% HW 7 Sphere identification
% 
% Inputs:
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
% Outputs:
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
%%



function [label,bbox,road] = hw7_ball_identification(t_data,im_data)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modify the code below this line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

label = zeros(length(im_data),1);
% Create a empty cell vectors to store the data 
bbox = cell(length(im_data),1);
road = cell(length(im_data),1);
temp = 0;
a = 1;
noise = 0.005;

T_dataPurple = t_data(find(t_data(:,4) == 1),1:3);
T_dataRed = t_data(find(t_data(:,4) == 2),1:3);
T_dataYellow = t_data(find(t_data(:,4) == 3),1:3);
T_dataPurple = T_dataPurple + noise  * rand(size(T_dataPurple,1),size(T_dataPurple,2));
T_dataRed = T_dataRed + noise * rand(size(T_dataRed,1),size(T_dataRed,2));
T_dataYellow = T_dataYellow + noise * rand(size(T_dataYellow,1),size(T_dataYellow,2));

% train purple class
x = T_dataPurple(:,1);
y = T_dataPurple(:,2);
z = T_dataPurple(:,3);
b = 1;
MeanOfX = mean(x);
MeanOfY = mean(y);
MeanOfZ = mean(z);
X = [x,y,z];
MeanofXYZ = [MeanOfX,MeanOfY,MeanOfZ];
MeanOfXPurple = zeros(1,3);
if temp == 0
    if a == 1
        if b ==1
            for k = 1 : size(T_dataPurple,1)
                MeanOfXPurple = MeanOfXPurple + X(k,:); 
            end
            
        end
    end
end
MeanOfXPurple = MeanOfXPurple / size(T_dataPurple,1);
    
InversePurpleOfA = zeros(3,3);
if temp == 0
    if a == 1
        if b ==1
            for i = 1:3
                for j = 1:3
                    for k = 1 : size(T_dataPurple,1)
                        InversePurpleOfA(i,j) = InversePurpleOfA(i,j) + (X(k,i)-MeanofXYZ(i))*(X(k,j)-MeanofXYZ(j));
                    end
                    InversePurpleOfA(i,j) = InversePurpleOfA(i,j) / size(T_dataPurple,1);
                end
            end
         end
    end
end
PurpleA = inv(InversePurpleOfA);

% train red class
x = T_dataRed(:,1);
y = T_dataRed(:,2);
z = T_dataRed(:,3);
MeanOfX = mean(x);
MeanOfY = mean(y);
MeanOfZ = mean(z);
X=[x,y,z];
MeanofXYZ = [MeanOfX,MeanOfY,MeanOfZ];
XmeanOfRed = zeros(1,3);

if temp == 0
    if a == 1
        if b ==1
            for k = 1 : size(T_dataRed,1)
                XmeanOfRed = XmeanOfRed + X(k,:); 
            end
        end
    end
end

XmeanOfRed = XmeanOfRed / size(T_dataRed,1);
    
InverseOfRedA = zeros(3,3); 
if temp == 0
    if a == 1
        if b ==1
            for i = 1:3
                for j = 1:3
                    for k = 1 : size(T_dataRed,1)
                        InverseOfRedA(i,j) = InverseOfRedA(i,j) + (X(k,i)-MeanofXYZ(i))*(X(k,j)-MeanofXYZ(j));
                    end
                    InverseOfRedA(i,j) = InverseOfRedA(i,j) / size(T_dataRed,1);
                end
            end
        end
    end
end
RedA = inv(InverseOfRedA);

% train yellow class
x = T_dataYellow(:,1);
y = T_dataYellow(:,2);
z = T_dataYellow(:,3);
MeanOfX = mean(x);
MeanOfY = mean(y);
MeanOfZ = mean(z);
X=[x,y,z];
MeanofXYZ = [MeanOfX,MeanOfY,MeanOfZ];
XmeanOfYellow = zeros(1,3);

if temp == 0
    if a == 1
        if b ==1
            for k = 1 : size(T_dataYellow,1)
                XmeanOfYellow = XmeanOfYellow + X(k,:); 
            end
        end
    end
end

XmeanOfYellow = XmeanOfYellow / size(T_dataYellow,1);
    
InverseOfYellowA = zeros(3,3);

if temp == 0
    if a == 1
        if b ==1
            for i = 1:3
                for j = 1:3
                    for k = 1 : size(T_dataYellow,1)
                        InverseOfYellowA(i,j) = InverseOfYellowA(i,j) + (X(k,i)-MeanofXYZ(i))*(X(k,j)-MeanofXYZ(j));
                    end
                    InverseOfYellowA(i,j) = InverseOfYellowA(i,j) / size(T_dataYellow,1);
                end
            end
        end
    end
end

    
YellowA = inv(InverseOfYellowA);

if temp == 0
    if a == 1
        if b ==1
            for i = 1:length(im_data)
                % get the first image
                im = im_data{i};
                im = im(1:235,:,:);
                im = double(im);

                % Sample code to try to detect the sphere by using a threshold value as 
                % introduced on the slides. t_data is ignored.
                % Remark for arithmetic operations in the color spaces you may want to
                % cast the data to double since uint8 may saturate

                TestOfX=im(:,:,1:3);
                 if temp == 0
                     if a == b
                        for m = 1:size(TestOfX,1)
                            for n = 1:size(TestOfX,2)
                                p_purple(m,n) = sqrt(det(PurpleA)) * exp(-(([TestOfX(m,n,1),TestOfX(m,n,2),TestOfX(m,n,3)] - MeanOfXPurple) * PurpleA * ([TestOfX(m,n,1),TestOfX(m,n,2),TestOfX(m,n,3)] - MeanOfXPurple)') / 2) / sqrt((2 * pi)^3);
                                p_red(m,n) = sqrt(det(RedA))*exp(-(([TestOfX(m,n,1),TestOfX(m,n,2),TestOfX(m,n,3)] - XmeanOfRed) * RedA * ([TestOfX(m,n,1),TestOfX(m,n,2),TestOfX(m,n,3)] - XmeanOfRed)') / 2) / sqrt((2 * pi)^3);
                                p_yellow(m,n) = sqrt(det(YellowA))*exp(-(([TestOfX(m,n,1),TestOfX(m,n,2),TestOfX(m,n,3)] - XmeanOfYellow) * YellowA * ([TestOfX(m,n,1),TestOfX(m,n,2),TestOfX(m,n,3)] - XmeanOfYellow)') / 2) / sqrt((2 * pi)^3 );

                                if p_purple(m,n) >= 1.0e-10 && p_purple(m,n) > p_red(m,n) && p_purple(m,n) > p_yellow(m,n) && a == b
                                    p(m,n) = 1;
                                elseif p_red(m,n) >= 1.0e-10 && p_red(m,n) > p_purple(m,n) && p_red(m,n) > p_yellow(m,n)&& a == b
                                    p(m,n) = 2;
                                elseif p_yellow(m,n) >= 1.0e-10 && p_yellow(m,n) > p_purple(m,n) && p_yellow(m,n) > p_red(m,n)&& a == b
                                    p(m,n) = 3;
                                else 
                                    p(m,n) = 0;
                                end
                            end
                        end
                    end
                end
                %figure
                %imshow(p_purple,[0,1.0e-10])
                %figure
                %imshow(p_red,[0,1.0e-10])
                %figure
                %imshow(p_yellow,[0,1.0e-10])

                if length(find(p == 1)) - length(find(p == 2)) > 500 && length(find(p == 1)) - length(find(p == 3)) > 500 && a == b
                    label(i) = 1;
                    if temp == 0
                        if a == b
                            for m = 1:size(TestOfX,1)
                                for n = 1:size(TestOfX,2)
                                    if p(m,n)~= 1
                                        p(m,n) = 0;
                                    end
                                end
                            end
                        end
                    end
                elseif length(find(p == 2)) - length(find(p == 1)) > 500 && length(find(p == 2)) - length(find(p == 3))> 500 && a == b
                    label(i) = 2;
                    if temp == 0
                        if a == b
                            for m = 1:size(TestOfX,1)
                                for n = 1:size(TestOfX,2)
                                    if p(m,n) == 2
                                        p(m,n) = 1;
                                    else
                                        p(m,n) = 0;
                                    end
                                end
                            end
                        end
                    end
                elseif length(find(p == 3)) - length(find(p == 1)) > 500 && length(find(p == 3)) - length(find(p == 2)) > 500 && a == b
                    label(i) = 3;
                    if temp == 0
                        if a == b
                            for m = 1:size(TestOfX,1)
                                for n = 1:size(TestOfX,2)
                                    if p(m,n) == 3
                                        p(m,n) = 1;
                                    else
                                        p(m,n) = 0;
                                    end
                                end
                            end
                        end
                    end
                else
                    label(i) = 0;
                end

                bbox{i} = get_bounding_box(p);


            end
        end
    end
end
% Use of nested functions just for demonstration
function bbox = get_bounding_box(im_bw)
    % here I am using iblobs to retrieve the bounding box, but you can develop
    % your own method if you wish
    b_dat = iblobs(im_bw, 'area', [30 Inf], 'class', 1);
    if isempty(b_dat)
        bbox = [];
    else
        bbox = [b_dat.umin b_dat.umax b_dat.vmin b_dat.vmax];
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of modifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
