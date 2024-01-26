classdef ImageProcessing < handle

    properties
        cam
        robot_transformation
        unsorted_low_x = 225 % thresholds for determining whether a centroid is an unsorted ball
        unsorted_high_x = 1000
        unsorted_low_y = 150
        unsorted_high_y = 470
    end

    methods
        % generate a mask for HSV system with desired max and minimum
% assumption: all entered values are from 0 to 255
% however in matlab: HSV are within 0 to 1

function self = ImageProcessing(cam)
    self.cam = cam;
    self.robot_transformation = [0, 1, 0, 75;
                                 1, 0, 0, -100;
                                 0, 0, -1, 0;
                                 0, 0, 0, 1];
end
function mask = generateMask(self,img, minHue, maxHue,minSat, maxSat, minVal, maxVal)
    hue = img(:,:,1); % extracts the hue value of the image
    saturation = img(:,:, 2);
    value = img(:, :, 3);
    
    % we are setting the bounds of the threshold..
    % we find what values are inside of the threshold
    minHueThreshold = hue > (minHue / 255); % generates a logical array where the hue is greater than
                              % a given minimum.
                             
    maxHueThreshold = hue < (maxHue / 255);
    
    minSatThreshold = saturation > (minSat / 255);
    maxSatThreshold = saturation < (maxSat / 255);
    
    minValThreshold = value > (minVal / 255);
    maxValThreshold = value < (maxVal / 255);
    
    % set all the values thata are outside of the threshold = 0
    hue(~(minHueThreshold & maxHueThreshold)) = 0;
    
    % all saturations outside of the threshold = 0
    % the way that logical array operations work as as follows:
    % it checks if the cell (n,m) is true, if so, it does the assignment
    saturation(~(minSatThreshold & maxSatThreshold)) = 0;
    
    value(~(minValThreshold & maxValThreshold)) = 0;
    
    HSV_MASK = cat(3, hue, saturation, value);
    mask = all(HSV_MASK, 3); % logical array with array dimensions of values
                             % to include. all values inside of the threshold
    
    end
    
    function img_filtered = colorFilter(self, img, mask)
    hue = img(:,:,1); % extracts the hue value of the image
    saturation = img(:,:, 2);
    value = img(:, :, 3);
    
    % all values outside not in the mask are set to zero
    hue(~mask) = 0;
    
    saturation(~mask) = 0;
    value(~mask) = 0;
    
    img_filtered = cat(3, hue, saturation, value);
end

function centroids = findCentroids(self, mask)
    
    mask_label = logical(mask);
    stat = regionprops(mask_label,'area','centroid');
    imshow(mask); 
    hold on;
    centroids = zeros(100, 2);
    i = 1;
    for x = 1: numel(stat)
        if stat(x).Area > 3000 && stat(x).Area < 6000
    %         returns the x, y pixel coordinate for a ball 
          centroids(i,:) = [stat(x).Centroid(1),stat(x).Centroid(2)];
        %plot(stat(x).Centroid(1),stat(x).Centroid(2),'ro');
        i = i + 1 ;
        end
    end
    
    centroids = centroids(1:i-1,:);

end

function centroidsWorld = findCentroidsWorld(self, mask)

    centroidsPixel = self.getUnsortedCentroids(self.findCentroids(mask));  % n x 2 array containing [xpixel, ypixel]
    centroidsWorld = zeros(100, 3);  % contains the xyz coordinate of the centroid in task space
    i = 1;
    for x = 1: size(centroidsPixel)
       
    %         returns the x, y, z coordinate for centroid of a ball in task space
    
          px_centroid_x = centroidsPixel(i,1);
          px_centroid_y = centroidsPixel(i,2);
          
          % column vector containing the x, y points on checker [x;y]
          % .' is matlab syntax to take the transpose of a matrix
          xy_world = pointsToWorld(self.cam.cam_IS, self.cam.cam_R, ...
              self.cam.cam_T,[px_centroid_x, px_centroid_y]).';
    
           robot_to_camera = 200; % distance from center of robot to center of camera (mm)
           camera_height = 185;   % camera post height (mm)
           ball_height = 10;      % ball radius (mm)
    
            % output : 4x1
            % ???? should I include the z height of the centroid or is it not
            % necesarry yet? will need to test to figure out
            % 4x4 * 4x1 --> 4x1
          distortedPointRelativeToRobot = self.robot_transformation * [xy_world ; ball_height; 1];
    
    
          distored_centroid_x = distortedPointRelativeToRobot(1);  % store x coordinate relative to robot (mm)
          distored_centroid_y = distortedPointRelativeToRobot(2);  % store y coordinate relative to robot (mm)
    
    
          
          % find x offsets: projection onto the xz plane
         
    
          % returns distance from camera to point in the x direction
          point_to_camera = robot_to_camera - distored_centroid_x; 
    
          % x offset from camera: add this
          x_offset = (ball_height * point_to_camera) / camera_height;
    
    
          % find y offsets
          % y offset to the camera: subtract this
          y_offset = (x_offset * distored_centroid_y) / point_to_camera;
    
          % test out using angles instead of hypotenuse to see that we get the
          % same points
    
          actual_centroid_x = distored_centroid_x + x_offset;
          actual_centroid_y = distored_centroid_y - y_offset;
    
          pointToCentroid_relative_toRobot = [actual_centroid_x, actual_centroid_y, ball_height];
     
          centroidsWorld(i,:) = pointToCentroid_relative_toRobot;
        
        i = i + 1 ;
    end
            % output: n x 3
            centroidsWorld = centroidsWorld(1:i-1,:);  % returns array with the xyz position of 
                                                     % the centroid of the ball in the task
                                                     % space


end

function bool = isDataValid(self, mask)

    mask_label = logical(mask);
    stat = regionprops(mask_label,'area','centroid')
    i = 1;
    for x = 1: numel(stat)
        if stat(x).Area > 2000
    %         returns the x, y pixel coordinate for a ball 
        i = i + 1 ;
        end
    end
    
    if i == 1
        bool = false;
    else
        bool = true;
    end
end
        
        % takes in a list of centroids found on the image and returns ones
        % associated with unsorted balls
        % uses thresholds defined as properties
function unsorted_centroids = getUnsortedCentroids(self, centroids)
    num_centroids = size(centroids,1);
    unsorted_centroids = zeros(num_centroids,2); % stores pixel x y of centroids in bounded area
    index = 1;
    % for every centroid, if between the boundaries, add to list
    for i=1:num_centroids
        if centroids(i,1) > self.unsorted_low_x && centroids(i,1) < self.unsorted_high_x
            if centroids(i,2) > self.unsorted_low_y && centroids(i,2) < self.unsorted_high_y
                unsorted_centroids(index,1) = centroids(i,1);
                unsorted_centroids(index,2) = centroids(i,2);
                index = index+1;
            end
        end
    end
    unsorted_centroids = unsorted_centroids(1:index-1,:);
end
        
        
    end
end