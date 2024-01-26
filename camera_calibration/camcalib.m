% Auto-generated by cameraCalibrator app on 27-Feb-2023
%-------------------------------------------------------


% Define images to process
imageFileNames = {'/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image24.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image26.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image28.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image29.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image31.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image33.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image34.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image35.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image36.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image46.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image47.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image53.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image55.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image62.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image64.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image66.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image68.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image69.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image124.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image141.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image142.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image143.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image144.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image145.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image146.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image148.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image155.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image156.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image158.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image160.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image161.png',...
    '/home/anespeland/Documents/RBE3001_C23_Team07/camera_calibration/Image162.png',...
    };
% Detect calibration pattern in images
detector = vision.calibration.monocular.CheckerboardDetector();
[imagePoints, imagesUsed] = detectPatternPoints(detector, imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates for the planar pattern keypoints
squareSize = 25;  % in units of 'millimeters'
worldPoints = generateWorldPoints(detector, 'SquareSize', squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams);

% Visualize pattern locations
h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% For example, you can use the calibration data to remove effects of lens distortion.
undistortedImage = undistortImage(originalImage, cameraParams);

% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('MeasuringPlanarObjectsExample')
% showdemo('StructureFromMotionExample')