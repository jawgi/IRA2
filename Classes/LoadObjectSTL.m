classdef LoadObjectSTL < handle
    %% LoadPlyFile class handles the loading of ply files used for building the work environment
    %#ok<*TRYNC>

    properties (Abstract) % An inheriing class must implement and assign these properties
        %> The first few letters or the name of ply files in the same directory
        fileName;
        identifier;
        scaleX;
        scaleY;
        scaleZ;
        transformation;
        faceColour;
        edgeColour;
        pointCloud;
    end

    properties
        %> workspace (this changes based on the DH parameters)
        workspace = [-4 4 -5 5 -0.02 3];

        %> The home location in radians
        homeQ = [];

        name;
        sizeX;
        sizeY;
        sizeZ;
        objectVertices;

    end

    properties (Hidden)
        axis_h = [];
        figure_h = [];
        lightAdded = false;
        surfaceAdded = false;
    end

    methods

        %% General class for multiDOF robot simulation
        function self = LoadObjectSTL()
            % This is intentionally left empty. Implement the class
            % constructor in the inhereting class.
            pause(0.001);
            try
                self.name = [self.fileName,datestr(now,'yyyymmddTHHMMSSFFF')];
            catch
                self.name = ['LoadObjectSTL',datestr(now,'yyyymmddTHHMMSSFFF')];
            end

            if self.fileName ~= ""
                self.plotFile();
                self.getObjectSize();
            end
        end

        function delete(self)
            handles = findobj('Tag',self.identifier);

            % Check if any handles were found
            if ~isempty(handles)
                disp(['Found ', num2str(length(handles)), ' object(s) with identifier: ', self.identifier]);

                % Loop through the found handles and delete them
                if length(handles) > 1
                    for i = 2:length(handles)
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

        function deleteAll(self)
            handles = findobj('Tag',self.identifier);

            % Check if any handles were found
            if ~isempty(handles)
                disp(['Found ', num2str(length(handles)), ' object(s) with identifier: ', self.identifier]);

                for i = 1:length(handles)
                    if isvalid(handles(i)) % Ensure the handle is valid
                        delete(handles(i)); % Safely attempt to delete the object
                        disp("Object deleted.");
                    end
                end
            else
                disp('No existing objects found with the same identifier.');
            end
        end

        function plotFile(self)
            %filepath = "@CADFiles\" + self.fileName + ".stl";
            filepath = self.fileName + ".stl";
            object = stlread(filepath); %import ply file
            faces = object.ConnectivityList;
            vertices = object.Points;
            self.pointCloud = vertices;
            scaledX = vertices(:,1) * self.scaleX;
            scaledY = vertices(:,2) * self.scaleY;
            scaledZ = vertices(:,3) * self.scaleZ;

            %centre table to origin, x and y only
            centreX = mean(scaledX, 1);
            centerY = mean(scaledY, 1);
            finalVerticesX = scaledX - centreX;
            finalVerticesY = scaledY - centerY;
            self.objectVertices = [finalVerticesX,finalVerticesY,scaledZ];

            % Convert point cloud locations to homogeneous coordinates
            coordinates = [self.objectVertices, ones(size(self.objectVertices, 1), 1)]';
            transformedCoordinates = (self.transformation.T * coordinates)';
            newCoordinates = transformedCoordinates(:, 1:3);
            transparency = 1;
            if self.fileName == "enclosure"
                transparency = 0.1;
            elseif self.fileName == "door"
                transparency = 0.7;
            end
            hold on;
            axis(self.workspace);
            disp(self.identifier);
            xlabel('X Axis');
            ylabel('Y Axis');
            zlabel('Z Axis');

            trisurf(faces, newCoordinates(:, 1), newCoordinates(:, 2), newCoordinates(:,3),'FaceColor', self.faceColour, "EdgeColor",  self.edgeColour,  'FaceAlpha', transparency, "Tag", self.identifier);
        end

        function getObjectSize(self)
            %get table height from scaled STL
            self.sizeZ = max(self.objectVertices(:,3));
            self.sizeX = max(self.objectVertices(:,1));
            self.sizeY = max(self.objectVertices(:,2));
        end

    end
end