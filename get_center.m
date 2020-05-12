function [center] = get_center(im)

%%%%%%%%%%%%% FOR ROS
%billede = rossubscriber("/camera/rgb/image_raw");
%im = receive(billede);
%img = imread(im);
%test = get_center(img) % you might only need to use "im" and not "img"
%needs testing
%%%%%%%%%%%%%%%%%%



%% Color-based segmentation and finding center position
imR = im(:,:,1); % red channel

imG = im(:,:,2); % green channel

imB = im(:,:,3); % blue channel

ExR = 2*imG-imB-imR;
%%%%%%%%%%%
imggray = ExR;
threshold = graythresh(imggray);
bw = im2bw(imggray,threshold);
bw2 = bw;


myStats = regionprops(bw2,'all'); % finding properties of connected components..
center = myStats.Centroid; 
end

