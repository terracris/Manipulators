% Final Project State Machine Script

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
tp = Traj_Planner();
cam = Camera();

duration = 2;
roundsThroughNoBalls = 0;
z_of_ball = 13;

% drop off positions in task space (x, y, z)
red_dropoff_position = [-8.07, -125.55, 85.41];
orange_dropoff_position = [-8.50, 126.11, 85.41];
yellow_dropoff_position = [182.33, -76.64, 85.41];
green_dropoff_position = [174.66, 83.08, 85.41];


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

running = true;

% state machine time!!
state = 1
disp("awaiting starting command")

pause 
while running
    switch state
    
        case 1
            
             current_location_angs = robot.measured_js(true, false);
             cL = robot.fk3001(current_location_angs(1,:));
             %xyz of current location
             current_location = [cL(1,4), cL(2,4), cL(3,4)];
             location = [95,0,100]; % home position
                 
             traj_coeffs(1,:) = tp.quintic_traj(duration, 0, 0, current_location(1), location(1), 0, 0);
             traj_coeffs(2,:) = tp.quintic_traj(duration, 0, 0, current_location(2), location(2), 0, 0);
             traj_coeffs(3,:) = tp.quintic_traj(duration, 0, 0, current_location(3), location(3), 0, 0);
             
             robot.run_trajectory(traj_coeffs, duration);
             robot.openGripper(); 
            % identify objects
            % Determines the 3D position of an object with respect to the robot’s reference frame (you
            % should be able to calculate the position of an object in the robot frame based on camera
            % images from the output of Part 4)
            % identifyObjects (prob in camera class)
            disp("awaiting next command")
            
            pause(.5);
            
            disp("waiting to take another image")
            
            disp("Taking image of the space")
            img = cam.getImage(); % gets undistorted image
%             imshow(img)
%             title("raw image")
%              imgHSV = rgb2hsv(img);
%             
%              im = ImageProcessing(cam);
%              mask = im.generateMask(imgHSV, 0, 255, 100 , 255, 100, 255);
%              figure; 
%              imshow(mask, InitialMagnification = 25);
%              title("segmented image");
%               
%              new_img = im.colorFilter(img, mask);
%              figure;
%              imshow(new_img, InitialMagnification = 25);
%              title("segmented colored image");
%             
%              valid = im.isDataValid(mask); % if valid data, calculate centroid, 
%                                        % else, dont             
            
            % convert the image to the HSV color space.
            imgHSV = rgb2hsv(img);
            im = ImageProcessing(cam);
            
            centroids = zeros(10,4); % 10 should change if there's going to be more than 10 balls on the board at a time
            % ^ 4th column is color where red = 1, orange = 2, yellow = 3, green = 4
            ind = 1;
        
            % red ball recognition
            [bw_img rgb_img] = redMask(img); 
%             figure()
%             imshow(rgb_img)
%             title("red image with balls only")
            centroidsPixel = im.findCentroidsWorld(bw_img);
            a=1:size(centroidsPixel,1)
            for a=1:size(centroidsPixel,1)
                centroids(ind,:) = [centroidsPixel(a,1) centroidsPixel(a,2) z_of_ball 1]; % 1 = red
                ind = ind+1;
            end
            %figure;
%             imshow(bw_img, InitialMagnification = 25);
            %title("red binary (black and white) image");

%             you MIGHT have to remove ind = ind - 1?
        
            % orange ball recognition
            [bw_img rgb_img] = orangeMask(img);
%             figure()
%             imshow(rgb_img)
%             title("orange image with balls only")
            centroidsPixel = im.findCentroidsWorld(bw_img);
            for a=1:size(centroidsPixel,1)
                centroids(ind,:) = [centroidsPixel(a,1) centroidsPixel(a,2) z_of_ball 2]; % 2 = orange
                ind = ind+1;
            end
        
            % yellow ball recogition
            [bw_img rgb_img] = yellowMask(img);
%             figure()
%             imshow(rgb_img)
%             title("yellow image with balls only")
            centroidsPixel = im.findCentroidsWorld(bw_img);
            for a=1:size(centroidsPixel,1)
                centroids(ind,:) = [centroidsPixel(a,1) centroidsPixel(a,2) z_of_ball 3]; % 3 = yellow
                ind = ind+1;
            end
        
            % green ball recognition
            [bw_img rgb_img] = greenMask(img);
%             figure()
%             imshow(rgb_img)
%             title("green image with balls only")
            centroidsPixel = im.findCentroidsWorld(bw_img);
            for a=1:size(centroidsPixel,1)
                centroids(ind,:) = [centroidsPixel(a,1) centroidsPixel(a,2) z_of_ball 4]; % 4 = green
                ind = ind+1;
            end

            
            if centroids(1,4) > 0
                state = 2
                centroids = centroids(1:ind-1,:)
                roundsThroughNoBalls = 0;
            else
                if roundsThroughNoBalls == 5
                    state = 10;
                else
                roundsThroughNoBalls = roundsThroughNoBalls + 1;
                state = 1;
                end 
                
            end
        
            

%             if valid
%              centroidsWorld = im.findCentroidsWorld(mask)
%              state = 2

       
    
            % if more than one object identified, move to case 2
    
        case 2
            % choose balls
            current_location_angs = robot.measured_js(true, false);
             cL = robot.fk3001(current_location_angs(1,:));
             %xyz of current location
             currXYZ = [cL(1,4), cL(2,4), cL(3,4)]

% 
%             curr = robot.measured_js(true,false);
%             currTF = robot.fk3001(curr(1,:));
%             currXYZ = [currTF(1,4), currTF(2,4), currTF(3,4)]
            
            ballLocation = Camera.get_closest_ball(centroids, currXYZ)
            % current ball = closest ball
            % move to ball
            robot.moveToBall(tp, currXYZ, ballLocation)

            % if at ball location, move to case 3
            state = 3
    
        case 3
            % pick up ball
            robot.closeGripper();
            pause(.5);
            % sort and place ball
            % get location ball will be placed based on ball color
            

            current_location_angs = robot.measured_js(true, false);
             cL = robot.fk3001(current_location_angs(1,:));
             %xyz of current location
             current_location = [cL(1,4), cL(2,4), cL(3,4)];
             location = [95,0,195]; 
                 
             traj_coeffs(1,:) = tp.quintic_traj(duration, 0, 0, current_location(1), location(1), 0, 0);
             traj_coeffs(2,:) = tp.quintic_traj(duration, 0, 0, current_location(2), location(2), 0, 0);
             traj_coeffs(3,:) = tp.quintic_traj(duration, 0, 0, current_location(3), location(3), 0, 0);
             
             robot.run_trajectory(traj_coeffs, duration);
             pause(.5);

%             ballGoal = robot.ballLocation(ballColor)
%             moveToLocation(ballGoal)
             
             % state = 10;
             state = 4

        case 4
%        go to drop of location
        color = ballLocation(4);
        if color == 1
            destination = red_dropoff_position;
        elseif color == 2
            destination = orange_dropoff_position;
        elseif color == 3
            destination = yellow_dropoff_position;
        else 
            destination = green_dropoff_position;
        end

         current_location_angs = robot.measured_js(true, false);
             cL = robot.fk3001(current_location_angs(1,:));
             %xyz of current location
             current_location = [cL(1,4), cL(2,4), cL(3,4)];
             location = destination; 
                 
             traj_coeffs(1,:) = tp.quintic_traj(duration, 0, 0, current_location(1), location(1), 0, 0);
             traj_coeffs(2,:) = tp.quintic_traj(duration, 0, 0, current_location(2), location(2), 0, 0);
             traj_coeffs(3,:) = tp.quintic_traj(duration, 0, 0, current_location(3), location(3), 0, 0);
             
             robot.run_trajectory(traj_coeffs, duration);
             pause(.5);
             robot.openGripper(); 

             state = 1;

    
        case 5
            % check end aka check if there's still objects unsorted
            % if not ended, go back to case 1
            % else, end
    
        otherwise
            disp(state)
            running = false;
    end
end

disp('Shutting down!')
robot.shutdown()
cam.shutdown()
