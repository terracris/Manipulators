%%
% RBE 3001 Lab 5 example code!
%%
clc;
clear;
clear java;
format short

%% Flags
DEBUG = false;
STICKMODEL = false;

%% Setup
vid = hex2dec('16c0');
pid = hex2dec('0486');

if DEBUG
    disp(vid);
    disp(pid);
end


javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java;
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

robot = Robot(myHIDSimplePacketComs);

try
    load("camParams.mat");
    disp("Loaded Camera Parameters from camParams.mat");
catch exception
    disp("Could not find camParams.mat, creating new Camera object");
    cam = Camera();
    save("camParams.mat","cam");
    disp("Saved Camera Parameters to camParams.mat");
end

%% Main Loop
try
    % Set up camera
    if cam.params == 0
        error("No camera parameters found!");
    end
    imshow(cam.getImage());
    
    
catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end

pixel_x = 309;
pixel_y = 242;

pointInChecker = pointsToWorld(cam.cam_IS, cam.cam_R, cam.cam_T, [pixel_x, pixel_y]);
pointInChecker = [pointInChecker.'; 0; 1] %augment matrix with z coordinate and 1 for transformation

T_Robot_Checker = [0, 1, 0, 75;
                   1, 0, 0, -100;
                   0, 0, -1, 0;
                   0, 0, 0, 1];

pointRelativeToRobot = T_Robot_Checker * pointInChecker;
i = 1;

while true
    disp("waiting to take another image")
    pause;
    disp("Taking image of the space")
    img = cam.getImage(); % gets undistorted image
    %  figure; imshow(img, InitialMagnification= 25);
    %  title("undistorted image");
    
    % segment the balls
    
    % convert the image to the HSV color space.
    imgHSV = rgb2hsv(img);
    im = ImageProcessing(cam);
    
    centroids = zeros(10,3);
    ind = 1;
    
    % looping in the order of red (1), orange (2), yellow (3), green (4)
    for i=1:4
        mask = im.generateMask(imgHSV, color_masks(i,1), color_masks(i,1), 100 , 255, 100, 255);
        figure;
        imshow(mask, InitialMagnification = 25);
        title("segmented image");
        
        new_img = im.colorFilter(img, mask);
        figure;
        imshow(new_img, InitialMagnification = 25);
        title("segmented colored image");
        
        valid = im.isDataValid(mask); % if valid data, calculate centroid, 
                                      % else, dont
        
        if valid
            centroidsPixel = im.findCentroid(mask)
            for a=1:size(centroidsPixel,1)
                centroids(ind,:) = [centroidsPixel(a,1) centroidsPixel(a,2) i]; % i indicates which color
            end
        else
            continue; % loop again
        end
    end

    centroids


end

%% Shutdown Procedure
% robot.shutdown()
 cam.shutdown()
