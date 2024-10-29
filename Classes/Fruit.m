classdef Fruit < handle
    properties
        mode;
        type; %individual type of fruit for this object
        colourName; %individual colour of fruit for this object
        colourCode;
        colourCodes;
        sizes;
        size;
        radius;
        startPoint;
        midPoint;
        dropPoint;
        x;
        y;
        z;
        tag;
        pointCloud;
<<<<<<< HEAD
=======
        handle;
        % end
        %
        % properties (Access = protected)
>>>>>>> ca2a970044c1e26910cd9943bf03d1ff4f83e436
        types = ["Greengage","Apricot","Plum"];
        colourNames = ["Green","Orange","Purple"];
        green = [0 0.5 0];
        orange = [1 0.5 0];
        purple = [0.5 0 0.5];
        sizeOptions = ["s" "m" "l"];
        greengageSize = [0.015 0.025; 0.025 0.04; 0.04 0.07]; % 2.5-4 cm standard size
        apricotSize = [0.015 0.03; 0.03 0.05; 0.05 0.075]; % 3-5 cm standard size
        plumSize = [0.015 0.04; 0.04 0.065; 0.065 0.08]; % 4-7 cm standard

        % Camera detected
        centre;
    end
    methods
        function self = Fruit(mode,qty)
            % if empty(mode)
            %     mode = "manual";
            % end
            if mode == "camera"
                self.detectFruit();
            else
                if isempty(qty)
                    qty = 9;
                end
                self.colourCodes = {self.green; self.orange; self.purple};
                self.sizes = {self.greengageSize; self.apricotSize; self.plumSize};
                self.type = cell(1,qty);
                self.colourName = cell(1,qty);
                self.colourCode = cell(1,qty);
                self.size = cell(1,qty);
                self.radius = zeros(1,qty);
                self.startPoint = cell(1,qty);
                self.midPoint = cell(1,qty);
                self.dropPoint = cell(1,qty);
                self.x = zeros(1,qty);
                self.y = zeros(1,qty);
                self.z = zeros(1,qty);
                self.pointCloud = cell(1,qty);
                self.plotFruit(qty);
                self.centre = cell(1,qty);
            end
        end

        function delete(self)
            handles = findobj('Tag',self.tag);

            % Check if any handles were found
            if ~isempty(handles)
                disp(['Found ', num2str(length(handles)), ' object(s) with identifier: ', self.tag]);

                % Loop through the found handles and delete them
                if length(handles) > 1
                    for i = 1:length(handles)
                        if isvalid(handles(i)) % Ensure the handle is valid
                            delete(handles(i)); % Safely attempt to delete the object
                            disp("Object deleted.");
                        end
                    end
                end
            else
                disp('No existing objects found with the same identifier.');
            end
        end

        function plotFruit(self,qty)
            iterator = 1;
            for i=1:qty
                if iterator > 3
                    iterator = iterator-3;
                end
                self.type{i} = char(self.types(iterator));
                self.colourName{i} = char(self.colourNames(iterator));
                self.colourCode{i} = self.colourCodes{iterator};
                self.size{i} = char(self.sizeOptions(iterator));
                radii = self.randomSize(i);
                %self.radius{i} = [radii radii radii];
                self.radius(i) = radii;

                iterator = iterator + 1;
            end
            self.generateStartPoses(qty);
            for i=1:qty
                self.plotFruitPly(i);
            end
        end

        function plotted = plotFruitPly(self, index, mode, transform)
            % disp("Entered plotFruitPly");
            % nargin
            plotted = false;
            if nargin <3
                [faceData, vertexData] = plyread('sphere.ply', 'tri');
                transformedCoordinates = self.ApplyTransform(index, self.startPoint{index}.T);
                self.pointCloud{index} = transformedCoordinates(:,1:3);
                self.tag{index} = self.type{index}+" "+index;
                %disp(self.colourCode{index});
                self.handle{index} = trisurf(faceData,transformedCoordinates(:,1),transformedCoordinates(:,2),transformedCoordinates(:,3), ...
                    'FaceColor', self.colourCode{index},'EdgeColor', 'none', 'Tag',self.tag{index});
                plotted = true;
            elseif strcmp(mode,'moving')
                transformedCoordinates = self.ApplyTransform(index, transform);
                self.pointCloud{index} = transformedCoordinates;
                %disp(self.colourCode{index});
                set(self.handle{index}, 'Vertices', transformedCoordinates(:, 1:3));
                plotted = true;
            end
        end

        function transformedCoordinates = ApplyTransform(self, index, transform)
            [faceData, vertexData] = plyread('sphere.ply', 'tri');
            scaled = vertexData(:, 1:3) * self.radius(index) * 0.1;
            coordinates = [scaled, ones(size(scaled, 1), 1)]';
            transformedCoordinates = (transform * coordinates)';
        end

        function radii = randomSize(self,index)
            thisType = find(strcmp(self.types, self.type(index)));
            thisSize = find(strcmp(self.sizeOptions, self.size(index)));
            sizeLimits = self.sizes{thisType}(thisSize, :);
            lowerLimit = sizeLimits(1);
            upperLimit = sizeLimits(2);
            radii = lowerLimit + (upperLimit - lowerLimit) * rand();
        end

        function generateStartPoses(self,qty)
            positions = [0 0 0];
            xLimit = [-0.3 0.3];
            yLimit = [-1.15 -0.99];
            zLimit = [0.85 1.1];
            boxSize = {xLimit, yLimit, zLimit};
            minDist = max(self.radius)*2;
            for x = xLimit(1) : minDist : xLimit(2)
                for y = yLimit(1) : minDist : yLimit(2)
                    for z = zLimit(1) : minDist : zLimit(2)
                        positions = [positions; x y z];
                    end
                end
            end

            for i = 1:qty
                fruitRadius = self.radius(i);
                %block out middle where robot is
                isValidPosition = false;
                while ~isValidPosition
                    % Generate a random pose within the paddock size
                    xPos = xLimit(1) + (xLimit(2) - xLimit(1)) * rand();
                    yPos = yLimit(1) + (yLimit(2) - yLimit(1)) * rand();
                    zPos = zLimit(1) + (zLimit(2) - zLimit(1)) * rand();
                    position = [xPos, yPos, zPos];

                    % Check if the position is far enough from all other objects
                    if isempty(positions)
                        % If it's the first object, the position is always valid
                        isValidPosition = true;
                    else
                        % Calculate the Euclidean distances between the new position and all existing positions
                        distances = sqrt(sum((positions - position).^2, 2));

                        % Check if all distances are greater than the minimum threshold
                        if min(distances) > fruitRadius
                            isValidPosition = true;
                        end
                    end
                    if isValidPosition == true
                        self.startPoint{i} = SE3(position);
                        positions = [positions; position];
                    end
                end
            end
        end %%getPoses


        function detectFruit(self,rgbVid,imageArea)
            fullFrame = getsnapshot(rgbVid);
            if ~empty(imageArea)
                x = imageArea(1); % Start column (horizontal position)
                y = imageArea(2); % Start row (vertical position)
                width = imageArea(3) - imageArea(1);
                height = imageArea(4) - imageArea(2);

                % Extract the RGB region using proper indexing for all 3 color channels
                frame = fullFrame(y:(y + height - 1), x:(x + width - 1), :);
            else
                frame = fullFrame;
            end

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

            numFruit = 0;
            figure;
            subplot(1, 2, 1);
            imshow(orangeSegmented);
            title('Orange Segmented');

            subplot(1, 2,2);
            imshow(orangeSegmented);
            title('Orange Outlines');
            shapes = self.plotOutlines(self.orangeMask);
            numFruit = size(self.centres,1);
            for i = 1:size(numFruit)
                self.type{i} = "Apricot";
                self.colourName{i} = "Orange";
                self.centre(i) = shapes{i,1};
                self.radius(i) = shapes{i,2};
                assignSize(i,self.radius(i),apricotSizes);
            end

            figure;
            subplot(1, 2,1);
            imshow(greenSegmented);
            title('Green Segmented');

            subplot(1, 2,2);
            imshow(greenSegmented);
            title('Green Outlines');
            shapes = self.plotOutlines(self.greenMask);
            startFrom = numFruit;
            numFruit = numFruit + size(shapes,1);
            for i = startFrom:size(numFruit)
                self.type{i} = "Greengage";
                self.colourName{i} = "Green";
                self.centre(i) = shapes{i,1};
                self.radius(i) = shapes{i,2};
                assignSize(i,self.radius(i),greengageSizes);
            end

            figure;
            subplot(1, 2,1);
            imshow(purpleSegmented);
            title('Purple Segmented');

            subplot(1, 2,2);
            imshow(purpleSegmented);
            title('Purple Outlines');
            shapes = self.plotOutlines(self.purpleMask);
            startFrom = numFruit;
            numFruit = numFruit + size(shapes,1);
            for i = startFrom:size(numFruit)
                self.type{i} = "Plum";
                self.colourName{i} = "Purple";
                self.centre(i) = shapes{i,1};
                self.radius(i) = shapes{i,2};
                assignSize(i,self.radius(i),plumSizes);
            end

            figure;
            subplot(1, 2, 1);
            imshow(rgbImage);
            title('Original Frame');
        end
    end

    methods (Hidden)
        function assignSize(self,index,radius,ranges)
            for i=1:3
                if ranges(1,1) <= radius && radius >= ranges(1,2)
                    self{index} = "s";
                elseif ranges(2,1) <= radius && radius >= ranges(2,2)
                    self{index} = "m";
                elseif ranges(3,1) <= radius && radius >= ranges(3,2)
                    self{index} = "l";
                end
            end
        end

        function shape = plotOutlines(mask)
            contours = bwboundaries(mask);
            stats = regionprops(mask, 'Centroid', 'MajorAxisLength', 'MinorAxisLength', 'Orientation');
            shape = cell(1,length(stats));
            hold on;
            for k = 1:length(stats)
                % Get the centroid
                centroid = stats(k).Centroid;
                % Get the size of the ellipse
                majorAxis = stats(k).MajorAxisLength / 2; % Approximate radius
                minorAxis = stats(k).MinorAxisLength / 2;
                orientation = stats(k).Orientation;
                radius = (majorAxis + minorAxis) / 2; % effective radius

                shape{k} = [centroid radius];
                % Draw the ellipse approximation
                ellipse(majorAxis, minorAxis, deg2rad(orientation), centroid(1), centroid(2), 'r');
                plot(centroid(1), centroid(2), 'b+', 'MarkerSize', 10, 'LineWidth', 2); % Mark the center
            end
            hold off;
        end

        function h = ellipse(ra,rb,ang,x0,y0,C)
            t = linspace(0,2*pi,100);
            X = ra*cos(t);
            Y = rb*sin(t);
            x = x0 + X*cos(ang) - Y*sin(ang);
            y = y0 + X*sin(ang) + Y*cos(ang);
            h = plot(x,y,C,'LineWidth',2);
        end
    end
end