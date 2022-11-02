% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
myResolution = param.resol;
% % the origin of the map in pixels
myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.



% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 500;                            % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
% 3-by-M Matrix:
P = repmat(myPose(:,1), [1, M]);
a = scanAngles;

xVar = 0.02;
yVar = 0.02;
tVar = 0.02;
w = ones([1 M])/M;

minScore = 1200;

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
    fprintf("timestep %d: \n", j);
    % 1) Propagate the particles
    P(1,:) = P(1,:) + randn(1, M) * xVar;
    P(2,:) = P(2,:) + randn(1, M) * yVar;
    P(3,:) = P(3,:) + randn(1, M) * tVar;
    
    % Get the range values from Lidar
    d = ranges(:,j);
    
    % Robot Poses in Discretized Map
    currLoc  = ceil(myResolution * P(1:2, :)) + myOrigin;
    % 2-by-M Matrix
    %     2) Measurement Update
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)
    RadarLoc = zeros([2, numel(a), M]);
    % Each Slice of 3D array stores the X any Y values of all the read
    % LIDAR values
    % 2-by-1081-by-M Matrix
    for i = 1 : numel(a)
        % Each Lidar Ray has to be converted from Local frame to Global
        % Frame
        % Get the Pose of all the sampled Points
        RadarYaw = a(i) + P(3,:);
        % 1-by-M Matrix = constant + 1-by-M Matrix
        RadarRange = ceil(myResolution * d(i));
        % Get the range of Lidar in Discretized Map
        % Negative Sign is used for sin, as Y axis direction is downwards.
        % Refer to Image in the Directory.
        GlobalLoc = [cos(RadarYaw); -sin(RadarYaw)] * RadarRange + currLoc;
        % 2-by-M = 2-by-M + 2-by-M
        
        % Save the Locations of mth Particle Ranges in discretized Cells
        RadarLoc(:,i,:) = floor(GlobalLoc);
    end
    
    for m = 1 : M
        % Create a Mask of Free and Occupied cells
        occ = RadarLoc(:,:,m);
        idx = ( occ(1,:) > 0);
        occ = occ(:,idx);
        idx = ( occ(1,:) < size(map, 1));
        occ = occ(:,idx);
        
        
        idx = ( occ(2,:) > 0);
        occ = occ(:,idx);
        idx = ( occ(1,:) < size(map, 2));
        occ = occ(:,idx);
        
        idx = sub2ind(size(map), occ(2,:)', occ(1,:)');
        w(m) = sum(map(idx));
    end
    %   2-2) For each particle, calculate the correlation scores of the particles

    %   2-3) Update the particle weights         
    [~, idx] = sort(w);
    m = w(idx(end));
    myPose(:,j) = P(:,idx(end));
    WorstPose1 = P(:,idx(1));
    WorstPose2 = P(:,idx(2));
    %   2-4) Choose the best particle to update the pose
    scoreFilter = min(m, minScore);
    idx = (w >= scoreFilter);
    P = P(:,idx);
    
    Dropped = M - size(P,2);
    idx = randi(size(P,2), Dropped, 1);
    P = [P P(:,idx)];
    % 3) Resample if the effective number of particles is smaller than a threshold

    % 4) Visualize the pose on the map as needed

    imagesc(map);
    hold on
    axis equal;
    plot(myPose(1,j)*param.resol+param.origin(1), ...
        myPose(2,j)*param.resol+param.origin(2), 'r.-', 'MarkerSize',20);
    plot(WorstPose1(1)*param.resol+param.origin(1), ...
        WorstPose1(2)*param.resol+param.origin(2), 'g.-', 'MarkerSize',20);
    plot(WorstPose2(1)*param.resol+param.origin(1), ...
        WorstPose2(2)*param.resol+param.origin(2), 'b.-', 'MarkerSize',20);
    drawnow limitrate
    if mod(j,10) == 0
        name = strcat('DemoImages/demo', num2str(j/10),'.png');
        saveas(gcf, name)
    end
end

end

