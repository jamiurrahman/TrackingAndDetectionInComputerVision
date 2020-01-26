clear
clc
close all
addpath('helper_functions')

%% Setup
% path to the images folder
path_img_dir = '../data/detection';
% path to object ply file
object_path = '../data/teabox.ply';

% Read the object's geometry 
% Here vertices correspond to object's corners and faces are triangles
[vertices, faces] = read_ply(object_path);

% Load the SIFT model from the previous task
load('sift_model.mat');


% TODO: setup camera intrinsic parameters using cameraParameters()
K = [2960.37845 0 0;
    0 2960.37845 0;
    1841.68855 1235.23369 1];

camera_params = cameraParameters('IntrinsicMatrix', K);

%% Get all filenames in images folder

FolderInfo = dir(fullfile(path_img_dir, '*.JPG'));
Filenames = fullfile(path_img_dir, {FolderInfo.name} );
num_files = length(Filenames);


%% Match SIFT features of new images to the SIFT model with features computed in the task 1
% You should use VLFeat function vl_ubcmatch()

% Place SIFT keypoints and descriptors of new images here
keypoints=cell(num_files,1);
descriptors=cell(num_files,1);
% Place matches between new SIFT features and SIFT features from the SIFT
% model here
sift_matches=cell(num_files,1);

% Default threshold for SIFT keypoints matching: 1.5 
% When taking higher value, match is only recognized if similarity is very high
threshold_ubcmatch = 1.5; 

% for i=1:num_files
%     fprintf('Calculating and matching sift features for image: %d \n', i)
%     
% %     TODO: Prepare the image (img) for vl_sift() function
%     img = single(rgb2gray(imread(char(Filenames(i)))));
%     [keypoints{i}, descriptors{i}] = vl_sift(img);
% %     Match features between SIFT model and SIFT features from new image
%     disp(class(double(descriptors{i})));
%     disp(class(model.descriptors));
%     sift_matches{i} = vl_ubcmatch(double(descriptors{i}), model.descriptors, threshold_ubcmatch); 
% end
% 
% 
% % Save sift features, descriptors and matches and load them when you rerun the code to save time
% save('sift_matches.mat', 'sift_matches');
% save('detection_keypoints.mat', 'keypoints')
% save('detection_descriptors.mat', 'descriptors')

load('sift_matches.mat')
load('detection_keypoints.mat')
load('detection_descriptors.mat')


%% PnP and RANSAC 
% Implement the RANSAC algorithm featuring also the following arguments:
% Reprojection error threshold for inlier selection - 'threshold_ransac'  
% Number of RANSAC iterations - 'ransac_iterations'

% Pseudocode
% i Randomly select a sample of 4 data points from S and estimate the pose using PnP.
% ii Determine the set of data points Si from all 2D-3D correspondences 
%   where the reprojection error (Euclidean distance) is below the threshold (threshold_ransac). 
%   The set Si is the consensus set of the sample and defines the inliers of S.
% iii If the number of inliers is greater than we have seen so far,
%   re-estimate the pose using Si and store it with the corresponding number of inliers.
% iv Repeat the above mentioned procedure for N iterations (ransac_iterations).

% For PnP you can use estimateWorldCameraPose() function
% but only use it with 4 points and set the 'MaxReprojectionError' to the
% value of 10000 so that all these 4 points are considered to be inliers

% Place camera orientations, locations and best inliers set for every image here
cam_in_world_orientations = zeros(3,3,num_files);
cam_in_world_locations = zeros(1,3,num_files);
best_inliers_set = cell(num_files, 1);

ransac_iterations = 50;
threshold_ransac = 4;

for i = 1:num_files
    fprintf('Running PnP+RANSAC for image: %d \n', i)

%     TODO: Implement the RANSAC algorithm here

    % Sometimes there are zero inliers and cause error
    best_inliers_size = 3;
    for j = 1:ransac_iterations
        
        % i Randomly select a sample of 4 data points from S
        % start
        index = randperm(size(sift_matches{i}, 2), 4);
        index = sift_matches{i}(:, index);
        
        image_index = index(1, :);
        sample_image = keypoints{i}(1:2, image_index)';
        
        model_index = index(2, :);
        sample_world = model.coord3d(model_index, :);
        % end

        % eventhoug MaxReprojectionError = 10000
        % sometimes got error in this reason try-catch block in here
        try
            % estimating pose
            [cam_in_world_orientation, cam_in_world_location] = estimateWorldCameraPose(sample_image, sample_world, camera_params, 'MaxReprojectionError', 10000);
        catch
            
            continue
        end
        
        % ii Determine the set of data points Si from all 2D-3D correspondences 
        %   where the reprojection error (Euclidean distance) is below the threshold (threshold_ransac). 
        %   The set Si is the consensus set of the sample and defines the inliers of S.

        % calculating rotation and translation parameters
        [R, t] = cameraPoseToExtrinsics(cam_in_world_orientation, cam_in_world_location);
        
        % Updating total transformation matrix, M = intrinsics * extrinsics
        M = cameraMatrix(camera_params, R, t);

        % Updating image matches from keypoints
        match_image_index = sift_matches{i}(1, :);
        match_image = keypoints{i}(1:2, match_image_index);
        
        % Updating world matches from model
        match_world_index = sift_matches{i}(2, :);
        match_world = model.coord3d(match_world_index, :);
        
        % till now wolrd matches number of column is 3
        % making the column 4 to match the row of M, for multiplication
        match_world(:, 4) = 1;
        
%         disp(match_image)
        
        % calculating re-projected point in image coordinate
        image_coord = match_world*M;
        
%         disp(image_coord)

        % Scaling is not correct for the re-projected point in image coordinate
        % Converting from image coordinate to pixel coordinate
        reproject = image_coord(:, 1:2)./image_coord(:, 3);

        % Calculating differences in pixel coordinate
        diff = match_image - reproject';
        
        % Calculating euclidian norm
%         diff = match_image - image_coord;
        reprojection_error = vecnorm(diff, 2, 1);

        % finding the inliers (which are below threshold_ransac)
        inliers = find(reprojection_error < threshold_ransac);
%         disp(numel(inliers))

        % iii If the number of inliers is greater than we have seen so far,
        if (numel(inliers) > best_inliers_size)
            best_inliers{i} = inliers; % Updating the inliers set
            best_inliers_size = numel(best_inliers{i}); % Updating the size of best inliers Sometimes it is more than 3 which is better
        end
    end

    %   re-estimate the pose using Si and store it with the corresponding number of inliers.
    [cam_in_world_orientations(:,:,i), cam_in_world_locations(:,:,i)] = estimateWorldCameraPose(match_image(:, best_inliers{i})', match_world(best_inliers{i}, 1:3), camera_params,'MaxReprojectionError', 4);
end


%% Visualize inliers and the bounding box

% You can use the visualizations below or create your own one
% But be sure to present the bounding boxes drawn on the image to verify
% the camera pose

edges = [[1, 1, 1, 2, 2, 3, 3, 4, 5, 5, 6, 7]
    [2, 4, 5, 3, 6, 4, 7, 8, 6, 8, 7, 8]];

for i=1:num_files
    
    figure()
    imshow(char(Filenames(i)), 'InitialMagnification', 'fit');
    title(sprintf('Image: %d', i))
    hold on
    
%   Plot inliers set
    PlotInlierOutlier(best_inliers_set{i}, camera_params, sift_matches{i}, model.coord3d, keypoints{i}, cam_in_world_orientations(:,:,i), cam_in_world_locations(:,:,i))
%   Plot bounding box
    points = project3d2image(vertices',camera_params, cam_in_world_orientations(:,:,i), cam_in_world_locations(:, :, i));
    for j=1:12
        plot(points(1, edges(:, j)), points(2, edges(:,j)), 'color', 'b');
    end
    hold off;
end