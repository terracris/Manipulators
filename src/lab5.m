%%
% RBE 3001 Lab 5 example code!
% colors: red = 1, orange = 2, yellow = 3, green = 4
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

tp = Traj_Planner();
traj_coefficients = zeros(3,7);
duration = 3;
current_location = zeros(1,3);
location = zeros(1,3);


disp("STARTING WHILE LOOP")
pause(3)

while true
    robot.servo_jp([0 0 0]);
    pause(2);
    robot.openGripper();
    disp("waiting to take another image")
    pause;
    disp("Taking image of the space")
    img = cam.getImage(); % gets undistorted image
    %  figure; imshow(img, InitialMagnification= 25);
    %  title("undistorted image");
    
     % segment the balls

% 
%     % convert the image to the HSV color space.
%     imgHSV = rgb2hsv(img);
%     im = ImageProcessing(cam);
%     
%     centroids = zeros(10,3); % 10 should change if there's going to be more than 10 balls on the board at a time
%     % ^ 3rd column is color where red = 1, orange = 2, yellow = 3, green = 4
%     ind = 1;
% 
%     % red ball recognition
%     [bw_img rgb_img] = redMask(img);
% %     figure;
% %     imshow(rgb_img, InitialMagnification = 25);
% %     title("colored image");
%     centroidsPixel = im.findCentroids(bw_img);
%     for a=1:size(centroidsPixel,1)
%         centroids(ind,:) = [centroidsPixel(a,1) centroidsPixel(a,2) 1]; % 1 = red
%         ind = ind+1;
%     end
% 
%     % orange ball recognition
%     [bw_img rgb_img] = orangeMask(img);
% %     figure;
% %     imshow(rgb_img, InitialMagnification = 25);
% %     title("colored image");
%     centroidsPixel = im.findCentroids(bw_img);
%     for a=1:size(centroidsPixel,1)
%         centroids(ind,:) = [centroidsPixel(a,1) centroidsPixel(a,2) 2]; % 2 = orange
%         ind = ind+1;
%     end
% 
%     % yellow ball recogition
%     [bw_img rgb_img] = yellowMask(img);
% %     figure;
% %     imshow(rgb_img, InitialMagnification = 25);
% %     title("colored image");
%     centroidsPixel = im.findCentroids(bw_img);
%     for a=1:size(centroidsPixel,1)
%         centroids(ind,:) = [centroidsPixel(a,1) centroidsPixel(a,2) 3]; % 3 = yellow
%         ind = ind+1;
%     end
% 
%     % green ball recognition
%     [bw_img rgb_img] = greenMask(img);
% %     figure;
% %     imshow(bw_img, InitialMagnification = 25);
% %     title("colored image");
%     centroidsPixel = im.findCentroids(bw_img);
%     for a=1:size(centroidsPixel,1)
%         centroids(ind,:) = [centroidsPixel(a,1) centroidsPixel(a,2) 4]; % 4 = green
%         ind = ind+1;
%     end
% 
%     centroids
%     
%     % make the fancy picture
%     position = zeros(10,4);
%     num = 0;
%     for n=1:size(centroids,1)
%         if centroids(n,1) ~= 0
%             position(n,1) = centroids(n,1)-35;
%             position(n,2) = centroids(n,2)-35;
%             position(n,3) = 75;
%             position(n,4) = 75;
%             num = num+1;
%         end
%     end
%     position
% 
%     label_str = cell(num,1);
%     for ii=1:num
%         switch centroids(ii,3)
%             case 1
%                 label_str{ii} = ['Red'];
%             case 2
%                 label_str{ii} = ['Orange'];
%             case 3
%                 label_str{ii} = ['Yellow'];
%             otherwise
%                 label_str{ii} = ['Green'];
%         end
%     end
%     label_str
% 
%     figure;
%     I = insertObjectAnnotation(img,"rectangle",position(1:num,:),label_str,TextBoxOpacity=0.9,FontSize=18);
%     imshow(I);
%     title("Image with Color Labels");
    
     % convert the image to the HSV color space.
     imgHSV = rgb2hsv(img);
    
     im = ImageProcessing(cam);
     mask = im.generateMask(imgHSV, 0, 255, 100 , 255, 100, 255);
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
         centroidsWorld = im.findCentroidsWorld(mask)
         
         disp("command robot to move")
         pause;
         angles = robot.ik3001(centroidsWorld(1, :));
         %robot.servo_jp(angles)
         current_location_angs = robot.measured_js(true, false);
         cL = robot.fk3001(current_location_angs(1,:));
         %xyz of current location
         current_location = [cL(1,4), cL(2,4), cL(3,4)];
         offset = 0;

         if (angles(1) < 0)
             offset = 10;
         end


         location = [centroidsWorld(1,1),centroidsWorld(1,2),centroidsWorld(1,3)]

         traj_coeffs(1,:) = tp.quintic_traj(duration, 0, 0, current_location(1), location(1), 0, 0);
         traj_coeffs(2,:) = tp.quintic_traj(duration, 0, 0, current_location(2), location(2)-offset, 0, 0);
         traj_coeffs(3,:) = tp.quintic_traj(duration, 0, 0, current_location(3), location(3) + 40, 0, 0);
         
         robot.run_trajectory(traj_coeffs, duration);
         pause(.5)

         current_location_angs = robot.measured_js(true, false);
         cL = robot.fk3001(current_location_angs(1,:));
         current_location = [cL(1,4), cL(2,4), cL(3,4)];

         traj_coeffs(1,:) = tp.quintic_traj(duration, 0, 0, current_location(1), location(1), 0, 0);
         traj_coeffs(2,:) = tp.quintic_traj(duration, 0, 0, current_location(2), location(2)-offset, 0, 0);
         traj_coeffs(3,:) = tp.quintic_traj(duration, 0, 0, current_location(3), location(3), 0, 0);
         
         robot.run_trajectory(traj_coeffs, duration);

            pause(1);
         robot.closeGripper();
         pause(.5);

     else
         continue; % loop again
     end

end

%% Shutdown Procedure

disp('Shutting down!')
robot.shutdown()
cam.shutdown()

