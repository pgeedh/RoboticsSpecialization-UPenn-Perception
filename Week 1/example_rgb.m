% Robotics: Estimation and Learning 
% WEEK 3
% 
% This is an example code for collecting ball sample colors using roipoly
close all

imagepath = './train';
Samples = [];
for k=1:15
    % Load image
    I = imread(sprintf('%s/%03d.png',imagepath,k));
    
    % You may consider other color space than RGB
    R = I(:,:,1);
    G = I(:,:,2);
    B = I(:,:,3);
    
    % Collect samples 
    disp('');
    disp('INTRUCTION: Click along the boundary of the ball. Double-click when you get back to the initial point.')
    disp('INTRUCTION: You can maximize the window size of the figure for precise clicks.')
    figure(1), 
    mask = roipoly(I); 
    figure(2), imshow(mask); title('Mask');
    sample_ind = find(mask > 0);
    
    R = R(sample_ind);
    G = G(sample_ind);
    B = B(sample_ind);
    
    Samples = [Samples; [R G B]];
    
    disp('INTRUCTION: Press any key to continue. (Ctrl+c to exit)')
    pause
end

% visualize the sample distribution
figure, 
scatter3(Samples(:,1),Samples(:,2),Samples(:,3),'.');
title('Pixel Color Distribubtion');
xlabel('Red');
ylabel('Green');
zlabel('Blue');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [IMPORTANT]
%
% Now choose you model type and estimate the parameters (mu and Sigma) from
% the sample data.
%

% Converting data from uint8 to double
Samples = double(Samples);
% Chosen 3 Dimensions of Gaussian model, as we have 3 Values: RGB
D = 3;
% Mean Estimate: 2 Ways
muHat1 = zeros([3,1]);

% Standard Deviation: 2 Ways
sigHat1 = zeros([3,1]);

% Number of Samples:
N = size(Samples,1);
% Doing first way:
for i = 1 : D
    muHat1(i) = mean(Samples(:,i));
    sigHat1(i) = sqrt(sum((Samples(:,i) - muHat1(i)).^2)/N);
end

% Doing Second Way:
[mu, sig] = normfit(Samples);
save('Model Parameters fit','mu','sig')