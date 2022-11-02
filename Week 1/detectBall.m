% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
% mu = 
% sig = 
thre = 10^-7;
load('Model Parameters fit','mu','sig');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
%
% Change the standard deviation to covariance matrix:
cov = diag(sig.^2);
prob = zeros(size(I(:,:,1)));
for i = 1 : size(I,2)
    prob(:,i) = mvnpdf(double([I(:,i,1) I(:,i,2) I(:,i,3)]), mu, cov);
end

% Create the mask:
mask = prob > thre;

CC = bwconncomp(mask);
% Filter out the biggest section from the mask and push it to Segmented
% Image
numPixels = cellfun(@numel,CC.PixelIdxList);
[~,idx] = max(numPixels);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

segI =  false(size(mask));
segI(CC.PixelIdxList{idx}) = true;

s = regionprops(CC, 'Centroid');
loc = s(idx).Centroid;

% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
