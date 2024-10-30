function relativeFruitPosition = detectFruit()
kinect = Kinect();
%%
preview(kinect.rgbVid);
%%
% preview(kinect.depthVid);
kinect.performCalibration();
kinect.calibrateDepth();
%%
rgbImage = getsnapshot(kinect.rgbVid);
imshow(rgbImage);
depthImage = getsnapshot(kinect.depthVid);

fruitArea = [110 50 420 275];
x = fruitArea(1); % Start column (horizontal position)
y = fruitArea(2); % Start row (vertical position)
width = fruitArea(3) - fruitArea(1);
height = fruitArea(4) - fruitArea(2);

% Extract the RGB region using proper indexing for all 3 color channels
croppedRgbImage = rgbImage(y:(y + height - 1), x:(x + width - 1), :);
rgbImage = croppedRgbImage;
cropping = true;
imshow(rgbImage);
title('Cropped RGB Image');
%%

% figure;
imshow(depthImage, []);

hsvFrame = rgb2hsv(rgbImage)


% Define thresholds for colour mapping
orangeHueMin = 0.02;
orangeHueMax = 0.18;
darkOrangeHueMin = 0.85;
darkOrangeHueMax = 0.99;
satMinO = 0.15;
satMaxO = 1.0;
valMinO = 0.15;
valMaxO = 1.0;

greenHueMin = 0.3;
greenHueMax = 0.6;
satMinG = 0.1;
satMaxG = 1.0;
valMinG = 0.1;
valMaxG = 1.0;

purpleHueMin = 0.7;
purpleHueMax = 0.85;
satMinP = 0.18;
satMaxP = 1.0;
valMinP = 0.18;
valMaxP = 1.0;

% Set the colour masks
orangeMask = (hsvFrame(:,:,1) >= orangeHueMin) & (hsvFrame(:,:,1) <= orangeHueMin) & ...
    (hsvFrame(:,:,2) >= satMinO) & (hsvFrame(:,:,2) <= satMaxO) & ...
    (hsvFrame(:,:,3) >= valMinO) & (hsvFrame(:,:,3) <= valMaxO);

darkOrangeMask = (hsvFrame(:,:,1) >= darkOrangeHueMin) & (hsvFrame(:,:,1) <= darkOrangeHueMin) & ...
    (hsvFrame(:,:,2) >= satMinO) & (hsvFrame(:,:,2) <= satMaxO) & ...
    (hsvFrame(:,:,3) >= valMinO) & (hsvFrame(:,:,3) <= valMaxO);

greenMask = (hsvFrame(:,:,1) >= greenHueMin) & (hsvFrame(:,:,1) <= greenHueMax) & ...
    (hsvFrame(:,:,2) >= satMinG) & (hsvFrame(:,:,2) <= satMaxG) & ...
    (hsvFrame(:,:,3) >= valMinG) & (hsvFrame(:,:,3) <= valMaxG);

purpleMask = (hsvFrame(:,:,1) >= purpleHueMin) & (hsvFrame(:,:,1) <= purpleHueMax) & ...
    (hsvFrame(:,:,2) >= satMinP) & (hsvFrame(:,:,2) <= satMaxP) & ...
    (hsvFrame(:,:,3) >= valMinP) & (hsvFrame(:,:,3) <= valMaxP);


orangeMask = bwareaopen(orangeMask,250);
darkOrangeMask = bwareaopen(darkOrangeMask,250);
orangeMask = orangeMask | darkOrangeMask;

greenMask = bwareaopen(greenMask,250);
purpleMask = bwareaopen(purpleMask,250);

orangeSegmented = rgbImage;
greenSegmented = rgbImage;
purpleSegmented = rgbImage;

% Apply each mask to the respective color segmentation
orangeSegmented(repmat(~orangeMask, [1, 1, 3])) = 0;
greenSegmented(repmat(~greenMask, [1, 1, 3])) = 0;
purpleSegmented(repmat(~purpleMask, [1, 1, 3])) = 0;

% Shape outlines
    function centres = plotOutlines(mask)
        centres = [];
        contours = bwboundaries(mask);
        stats = regionprops(mask, 'Centroid', 'MajorAxisLength', 'MinorAxisLength', 'Orientation');
        hold on;
        for k = 1:length(stats)
            % Get the centroid
            centroid = stats(k).Centroid;
            centres = [centres; centroid];
            % Get the size of the ellipse
            majorAxis = stats(k).MajorAxisLength / 2; % Approximate radius
            minorAxis = stats(k).MinorAxisLength / 2;
            orientation = stats(k).Orientation;

            % Draw the ellipse approximation
            ellipse(majorAxis, minorAxis, deg2rad(orientation), centroid(1), centroid(2), 'r');
            plot(centroid(1), centroid(2), 'b+', 'MarkerSize', 10, 'LineWidth', 2); % Mark the center
        end
        hold off;
    end

% Helper function to draw an ellipse
    function h = ellipse(ra,rb,ang,x0,y0,C)
        t = linspace(0,2*pi,100);
        X = ra*cos(t);
        Y = rb*sin(t);
        x = x0 + X*cos(ang) - Y*sin(ang);
        y = y0 + X*sin(ang) + Y*cos(ang);
        h = plot(x,y,C,'LineWidth',2);
    end

% Plot
figure;
subplot(1, 2, 1);
imshow(orangeSegmented);
title('Orange Segmented');

subplot(1, 2,2);
imshow(orangeSegmented);
title('Orange Outlines');
orangeCentroids = plotOutlines(orangeMask)

pause(1);
figure;
subplot(1, 2,1);
imshow(greenSegmented);
title('Green Segmented');

subplot(1, 2,2);
imshow(greenSegmented);
title('Green Outlines');
greenCentroids = plotOutlines(greenMask)

pause(1);
figure;
subplot(1, 2,1);
imshow(purpleSegmented);
title('Purple Segmented');

subplot(1, 2,2);
imshow(purpleSegmented);
title('Purple Outlines');
purpleCentroids = plotOutlines(purpleMask)

pause(1);
figure;
subplot(1, 2, 1);
imshow(rgbImage);
title('Original Frame');

subplot(1, 2, 2);
imshow(rgbImage);
title('Fruit Centres');
hold on;
numFruit = size(orangeCentroids) + size(greenCentroids) + size(purpleCentroids)
fruit = [orangeCentroids; greenCentroids; purpleCentroids]
for i=1:numFruit
    plot(fruit(i,1), fruit(i,2), 'b+', 'MarkerSize', 5, 'LineWidth', 1);
end
hold off;

pause(1);

centres = [orangeCentroids; greenCentroids; purpleCentroids]

% Assuming depthImage is your acquired depth image
filteredDepthImage = medfilt2(depthImage, [3 3]); % 3x3 median filter
relativeFruitPosition = cell(numFruit);
for i = 1:numFruit
    x = fruitArea(1) + round(centres(i,1)) - 1;
    y = fruitArea(2) + round(centres(i,2)) - 1;
    % x = round(centres(i,1));
    % y = round(centres(i,2));
    depthValue = depthImage(y, x);
    filteredDepthValue = filteredDepthImage(y,x);
    %disp(["X: ",x," Y: ",y," Depth: ",depthValue," and filtered Depth: ",filteredDepthValue]);
    depth = kinect.getPixelDepth(x,y);
    disp(["Calibrated depth = ",depth]);
    %adjust for table height and radius
    relativeFruitPosition{i} = kinect.getPixelCoordinates(x,y,depth);
end

end