%%
% Create a video input object for the kinect and view outputs
imaqreset;
kinect = Kinect(1);
%%
rgbImage = getsnapshot(kinect.rgbVid);
imshow(rgbImage);
%%
preview(kinect.rgbVid);
preview(kinect.depthVid);

% Perform the calibratoin
if kinect.calibrated == 0
    kinect.performCalibration();
    kinect.calibrateDepth();
end
%%
relativeFruitPositions = detectFruit(kinect);