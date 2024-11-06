classdef Kinect < handle

    properties
        cameraParams;
        rgbVid;
        rgbImage;
        depthVid;
        depthImage;
        calibrated;
        calibratedImage;
        worldPoints3D;
    end

    methods

        function self = Kinect(precalibrated)
            self.rgbVid = videoinput('kinect', 1); % RGB camera
            self.depthVid = videoinput('kinect', 2); % Depth camera+
            self.rgbImage = getsnapshot(self.rgbVid);
            self.depthImage = getsnapshot(self.depthVid);
            if nargin < 1 || precalibrated == 0
                self.calibrated = 0;
                disp("Please complete the camera calibration.");
            else
                self.calibrated = 1;
                disp("Camera will be calibrated with data from a previous calibration");
                self.performCalibration();
                self.getPrecalibratedData();
            end
        end

        function takeCalibrationImages(self)
            preview(self.rgbVid);
            %preview(self.depthVid);

            numImages = 3; % Number of images to take for calibration

            % Create a folder to save calibration images
            saveFolder = 'CalibrationImages';
            if ~exist(saveFolder, 'dir')
                mkdir(saveFolder);
            end

            % Instructions for the user
            disp('Move the Kinect camera to different angles around the checkerboard pattern.');
            disp('Enter "Y" to take each snapshot.');
            disp(['A total of ', num2str(numImages), ' images will be captured.']);

            % Loop to capture images
            for i = 1:numImages
                % Wait for the user to confirm readiness
                userInput = '';
                while ~strcmpi(userInput, 'Y')
                    userInput = input(['Type "Y" and press Enter for image ' num2str(i) ': '], 's'); % Use 's' to treat input as a string
                    if ~strcmpi(userInput, 'Y')
                        disp('Invalid input. Please type "Y".');
                    end
                end

                % Capture and save the RGB image
                rgbImage = getsnapshot(self.rgbVid);
                rgbFileName = fullfile(saveFolder, ['rgb_image_' num2str(i) '.png']);
                imwrite(rgbImage, rgbFileName);

                % Capture and save the depth image
                % depthImage = getsnapshot(self.depthVid);
                % depthFileName = fullfile(saveFolder, ['depth_image_' num2str(i) '.png']);
                % imwrite(depthImage, depthFileName);

                % Notify the user that the images have been saved
                disp(['Captured and saved images for position ' num2str(i)]);
            end

            % Clean up and release video input objects
            delete(self.rgbVid);
            % delete(self.depthVid);
            disp('Calibration images have been captured and saved. You can now proceed with calibration.');
        end

        function calibrateCamera(self)
            self.takeCalibrationImages();
            % Define input folder and create a subfolder for processed images
            inputFolder = 'CalibrationImages'; % Folder containing the images
            outputFolder = fullfile(inputFolder, 'ProcessedImages'); % Archive folder

            % Check if output folder exists, if not, create it
            if ~exist(outputFolder, 'dir')
                mkdir(outputFolder);
            end

            % Step 1: Get a list of all image files in the calibration image folder
            imageFiles = dir(fullfile(inputFolder, '*.png'))

            % Check if there are any images in the folder
            if isempty(imageFiles)
                disp('No images found in the specified folder.');
                return;
            end

            % Step 2: Detect checkerboard points
            imagePoints = {};
            validImages = 0;

            for i = 1:length(imageFiles)
                currentImageFile = fullfile(inputFolder, imageFiles(i).name);
                I = imread(currentImageFile);
                imshow(currentImageFile);

                % Detect checkerboard points in the image
                [currentImagePoints, boardSize] = detectCheckerboardPoints(I);

                % Append detected points to the list]
                if isequal(boardSize, [4, 5]) % Change as per your checkerboard
                    % Append detected points to the cell array
                    imagePoints{end + 1} = currentImagePoints
                    validImages = validImages + 1; % Increment valid image count
                    disp("Valid");
                end
                disp(['Image ', imageFiles(i).name, ' detected board size: ', num2str(boardSize)]);

            end
            % Step 3: Generate world points
            squareSize = 40; % Size of a square in mm
            worldPoints = generateCheckerboardPoints(boardSize, squareSize)

            % Step 4: Estimate camera parameters
            [self.cameraParams, imagesUsed] = estimateCameraParameters(imagePoints, worldPoints);

            % Step 5: Visualize the calibration results
            figure;
            showReprojectionErrors(cameraParams);
            title('Reprojection Errors');

            % Loop through each image and move to archive folder
            for i = 1:length(imageFiles)
                currentImageFile = fullfile(inputFolder, imageFiles(i).name);
                movefile(currentImageFile, outputFolder);
            end

            disp('Camera has been calibrated. All images have been processed and moved to the subfolder.');
        end

        function alignRGBDepth(self)
            % Ensure images are of the same size
            if size(self.rgbImage, 1) ~= size(self.depthImage, 1) || size(self.rgbImage, 2) ~= size(self.depthImage, 2)
                error('RGB and depth images must be of the same dimensions.');
            end

            % Convert depth image to meters (if it is in mm or another scale)
            depthImage = double(self.depthImage) / 1000;

            % Create an output image for aligned RGB data
            self.calibratedImage = zeros(size(self.rgbImage), 'like', self.rgbImage);

            % Loop through each pixel in the RGB image
            for y = 1:size(self.rgbImage, 1)
                for x = 1:size(self.rgbImage, 2)
                    % Get the depth value for the current pixel
                    z = depthImage(y, x);
                    if z > 0  % Ensure the depth is valid (not zero)
                        % Get the normalized coordinates from the RGB image
                        normalizedCoords = [x; y; 1];

                        % Calculate 3D point in the camera coordinate system
                        cameraCoords = self.cameraParams.IntrinsicMatrix \ normalizedCoords * z;

                        % Optionally, apply any transformation to align RGB and depth
                        % (This will depend on your specific setup and calibration)

                        % Re-project the 3D point back to the RGB image plane
                        projectedCoords = self.cameraParams.IntrinsicMatrix * cameraCoords;
                        projectedCoords = projectedCoords ./ projectedCoords(3); % Normalize

                        % Find the corresponding pixel in the RGB image
                        projectedX = round(projectedCoords(1));
                        projectedY = round(projectedCoords(2));

                        % Ensure projected coordinates are within bounds
                        if projectedX > 0 && projectedX <= size(self.rgbImage, 2) && ...
                                projectedY > 0 && projectedY <= size(self.rgbImage, 1)
                            self.calibratedImage(y, x, :) = self.rgbImage(projectedY, projectedX, :);
                        end
                    end
                end
            end

            % Convert alignedImage to uint8 if needed
            self.calibratedImage = uint8(self.calibratedImage);
        end

        function performCalibration(self)
            % Step 1: Get a list of all image files in the calibration image folder
            imageFiles = dir(fullfile('CalibrationImages', '*.png'));

            % Check if there are any images in the folder
            if isempty(imageFiles)
                disp('No images found in the specified folder.');
                return;
            end
            numImages = length(imageFiles); %using predefined date
            squareSize = 41; % Size of a square in mm
            imagePoints = cell(1, numImages);
            worldPoints3D = [];

            %allImagePoints = [];
            allWorldPoints = [];

            for i = 1:length(imageFiles)
                currentImageFile = fullfile('CalibrationImages', imageFiles(i).name);
                image = imread(currentImageFile);
                imshow(currentImageFile);

                % Detect checkerboard points in the Kinect snapshot
                [snapshotImagePoints, snapshotBoardSize] = detectCheckerboardPoints(image);
                %disp(['Image RGB ', num2str(i), ' detected board size: ', num2str(snapshotBoardSize)]);
                imagePoints{i} = snapshotImagePoints;

                % Generate world points (same for all images)
                worldPoints = generateCheckerboardPoints(snapshotBoardSize, squareSize);
                allWorldPoints = [allWorldPoints; worldPoints];
            end

            % Convert imagePoints to the required format
            allImagePoints = cat(3, imagePoints{:});


            % disp(size(allImagePoints));
            % disp(size(allWorldPoints));
            % Estimate camera parameters using the detected points
            [self.cameraParams, imagesUsed] = estimateCameraParameters(allImagePoints, worldPoints);

            % Visualize the calibration results
            % figure;
            % showReprojectionErrors(self.cameraParams);
            % title('Reprojection Errors');

            % Clean up
            % delete(rgbVid);
            % delete(depthVid);
            disp('RGB Camera calibration completed.');
            % disp(self.cameraParams);
        end

        function calibrateDepth(self)
            % Get camera params for calibration
            fx = self.cameraParams.FocalLength(1);
            fy = self.cameraParams.FocalLength(2);
            cx = self.cameraParams.PrincipalPoint(1);
            cy = self.cameraParams.PrincipalPoint(2);

            rgbImage = getsnapshot(self.rgbVid);
            [imagePoints, boardSize] = detectCheckerboardPoints(rgbImage);
            numPoints = size(imagePoints, 1);

            depthImage = getsnapshot(self.depthVid);
            depthImage = double(depthImage) / 1000; % convert to meters


            worldPoints3D = zeros(numPoints, 3);
            for i = 1:numPoints
                % Extract the 2D point coordinates
                u = imagePoints(i, 1); % x-coordinate in the image
                v = imagePoints(i, 2); % y-coordinate in the image

                % Get the corresponding depth value
                z = double(depthImage(round(v), round(u))); % Depth at (u, v)

                % Map 2D image points to 3D using intrinsic parameters and depth value
                x = (u - cx) * z / fx;
                y = (v - cy) * z / fy;

                % Store the 3D world point
                self.worldPoints3D(i, :) = [x, y, z];
            end

            self.calibrated = 1;
            % Assign the property to a temporary variable
            worldPoints3D = self.worldPoints3D;

            filePath = fullfile("Camera", 'depthCalibration.mat');
            
            % Save the temporary variable
            save(filePath, 'worldPoints3D');
            self.alignRGBDepth();

            disp('Depth calibration completed.');
        end

        function getPrecalibratedData(self)
            filePath = fullfile("Camera", 'depthCalibration.mat');
            data = load(filePath, 'worldPoints3D');
            fileInfo = dir(filePath);
            lastModified =  dateshift(datetime(fileInfo.date), 'start', 'day'); 
            today = dateshift(datetime("now"), 'start', 'day');
    
            if isempty(fieldnames(data)) 
                disp("No calibration data saved, please calibrate the camera.");
            elseif lastModified ~= today
                disp("Calibration data outdated, please recalibrate the camera.");
            else
                self.worldPoints3D = data.worldPoints3D;
                self.calibrated = 1;
                self.alignRGBDepth();
                disp("Successfully obtained precalibrated camera information.");
            end
        end

        function depth = getPixelDepth(self,x,y)
            % depth = zeros(1,2); %depth away from camera & z position?
            x = round(x);
            y = round(y);
            depth = 0;
            if x < 1 || x > size(self.calibratedImage, 2) || y < 1 || y > size(self.calibratedImage, 1)
                error('Pixel coordinates are out of bounds.');
            else
                 depth = double(self.depthImage(y, x)) / 1000;
            end
        end

        function coords = getRelativePosition(self,x,y,depth)
            tableHeight = 0.8;
            cameraHeight = 2.4;
            coords = zeros(1,3);
            [height, width, ~] = size(self.rgbImage);
    
            % Calculate the middle pixel coordinates
            cameraX = round(width / 2) / 1000;
            cameraY = round(height / 2) / 1000;
            referenceX = x / 1000;
            referenceY = y / 1000;

            coords(1) = cameraX - referenceX;
            coords(2) = cameraY - referenceY;
            coords(3) = cameraHeight - depth
        end
            
    end
end