% Create a video input object for the inbuilt webcam
camera = videoinput('winvideo', 1); % Adjust 'winvideo' and 1 based on your camera type

% Set properties for the video input object
set(camera, 'ReturnedColorSpace', 'RGB');
camera.FrameGrabInterval = 5; % Adjust the frame grab interval as needed

% Start the camera preview (optional)
preview(camera);

% Capture a frame from the camera
frame = getsnapshot(camera);

% Stop the camera preview after capturing (optional)
closepreview(camera);

% Convert the captured frame to HSV color space
hsvFrame = rgb2hsv(frame);

% Define thresholds for orange color
orangeHueMin = 0.01; % Lower bound of hue for orange
orangeHueMax = 0.25; % Upper bound of hue for orange
satMin = 0.4; % Minimum saturation for all colors
satMax = 1.0; % Maximum saturation for all colors
valMin = 0.4; % Minimum value (brightness) for all colors
valMax = 1.0; % Maximum value (brightness) for all colors

% Create a binary mask for orange
orangeMask = (hsvFrame(:,:,1) >= orangeHueMin) & (hsvFrame(:,:,1) <= orangeHueMax) & ...
             (hsvFrame(:,:,2) >= satMin) & (hsvFrame(:,:,2) <= satMax) & ...
             (hsvFrame(:,:,3) >= valMin) & (hsvFrame(:,:,3) <= valMax);

% Define thresholds for green color
greenHueMin = 0.2; % Lower bound of hue for green
greenHueMax = 0.6; % Upper bound of hue for green
satMinGreen = 0.1; % Minimum saturation for all colors
satMaxGreen = 1.0; % Maximum saturation for all colors
valMinGreen = 0.1; % Minimum value (brightness) for all colors
valMaxGreen = 1.0; % Maximum value (brightness) for all colors

% Create a binary mask for green
greenMask = (hsvFrame(:,:,1) >= greenHueMin) & (hsvFrame(:,:,1) <= greenHueMax) & ...
            (hsvFrame(:,:,2) >= satMinGreen) & (hsvFrame(:,:,2) <= satMaxGreen) & ...
            (hsvFrame(:,:,3) >= valMinGreen) & (hsvFrame(:,:,3) <= valMaxGreen);

% Define thresholds for purple color
purpleHueMin = 0.73; % Lower bound of hue for purple
purpleHueMax = 0.95; % Upper bound of hue for purple

% Create a binary mask for purple
purpleMask = (hsvFrame(:,:,1) >= purpleHueMin) & (hsvFrame(:,:,1) <= purpleHueMax) & ...
             (hsvFrame(:,:,2) >= satMin) & (hsvFrame(:,:,2) <= satMax) & ...
             (hsvFrame(:,:,3) >= valMin) & (hsvFrame(:,:,3) <= valMax);

% Combine the masks to create a color segmented output
orangeSegmented = frame;
greenSegmented = frame;
purpleSegmented = frame;

% Apply each mask to the respective color segmentation
orangeSegmented(repmat(~orangeMask, [1, 1, 3])) = 0;
greenSegmented(repmat(~greenMask, [1, 1, 3])) = 0;
purpleSegmented(repmat(~purpleMask, [1, 1, 3])) = 0;

% Display the original frame and the segmented outputs
figure;
subplot(2, 2, 1);
imshow(frame);
title('Original Frame');

subplot(2, 2, 2);
imshow(orangeSegmented);
title('Orange Segmented');

subplot(2, 2, 3);
imshow(greenSegmented);
title('Green Segmented');

subplot(2, 2, 4);
imshow(purpleSegmented);
title('Purple Segmented');

% Release the camera when done
delete(camera);
clear camera;
