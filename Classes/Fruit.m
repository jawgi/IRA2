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
        % end
        %
        % properties (Access = protected)
        types = ["Greengage","Apricot","Plum"];
        colourNames = ["Green","Orange","Purple"];
        green = [0 0.5 0];
        orange = [1 0.5 0];
        purple = [0.5 0 0.5];
        sizeOptions = ["s" "m" "l"];
        greengageSize = [0.015 0.025; 0.025 0.04; 0.04 0.07]; % 2.5-4 cm standard size
        apricotSize = [0.015 0.03; 0.03 0.05; 0.05 0.075]; % 3-5 cm standard size
        plumSize = [0.015 0.04; 0.04 0.065; 0.065 0.08]; % 4-7 cm standard
    end
    methods
        function self = Fruit(mode,qty)
            if mode ~= "manual"
                mode = "manual";
            end
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
                self.plotFruitPly(index);
            end
        end

        function plotFruitPly(self, index)
            [faceData, vertexData] = plyread('sphere.ply', 'tri');
            scaled = vertexData(:,1:3) * self.radius(index) * 0.1;
            coordinates = [scaled, ones(size(scaled, 1), 1)]';
            transformedCoordinates = (self.startPoint{index}.T * coordinates)';
            self.pointCloud = transformedCoordinates;
            trisurf(faceData,transformedCoordinates(:,1),transformedCoordinates(:,2),transformedCoordinates(:,3), ...
                'FaceColor', self.colourCode{index},'EdgeColor', 'none', 'Tag',self.tag{index});
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
            xLimit = [-0.55 0.55];
            yLimit = [-1.33 -1.12];
            zLimit = [0.85 1.2];
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
                        self.startPoint{i} = position;
                        positions = [positions; position];
                    end
                end
            end
        end %%getPoses
    end
end