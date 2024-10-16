classdef LoadObjectPLY < handle
    %% LoadPlyFile class handles the loading of ply files used for building the work environment
    %#ok<*TRYNC>

    properties (Abstract) % An inheriing class must implement and assign these properties
        %> The first few letters or the name of ply files in the same directory
        fileName;
        scaleX;
        scaleY;
        scaleZ;
        transformation;
    end

    properties
        %> workspace (this changes based on the DH parameters)
        workspace = [-10 10 -10 10 -0.01 10];

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
        function self = LoadObjectPLY()
            % This is intentionally left empty. Implement the class
            % constructor in the inhereting class.
            pause(0.001);
            try
                self.name = [self.fileName,datestr(now,'yyyymmddTHHMMSSFFF')];
            catch
                self.name = ['LoadObjectPLY',datestr(now,'yyyymmddTHHMMSSFFF')];
            end
            if self.fileName ~= ""
                self.plotFile();
                self.getObjectSize();
            end
        end

        function delete(self)
            handles = findobj('Tag', self.name, 'ClassType','LoadObjectPLY'); % Find handles with matching tag

            if isempty(handles) % If no handles found, exit the function
                return;
            end
            disp(['Found ', num2str(length(handles)), ' handle(s) to delete.']);
            for i = 1:length(handles)-1
                if isvalid(handles(i)) % Ensure the handle is valid
                    h = get(handles(i), 'UserData'); % Get user data only for valid objects

                    try delete(handles(i)); end % Safely attempt to delete the object
                end
            end
        end

        function plotFile(self)
            filepath = "@CADFiles\" + self.fileName + ".ply"
            [faces,vertices] = plyread(filepath,'tri');
            scaledObject(:,1) = vertices(:,1) * self.scaleX;
            scaledObject(:,2) = vertices(:,2) * self.scaleY;
            scaledObject(:,3) = vertices(:,3) * self.scaleZ;

            homogeneousVertices = [scaledVertices, ones(size(scaledVertices, 1), 1)]';
            transformedCoordinates = self.transformation.T * homogeneousVertices;

            % Remove the homogeneous coordinate
            self.objectVertices = transformedCoordinates(1:3, :)';

            % Plot the transformed PLY file
            hold on;
            trisurf(faces, self.objectVertices(:, 1), self.objectVertices(:, 2), self.objectVertices(:, 3), ...
                'FaceColor', [0,0,0], 'EdgeColor', [0,0,0]);
            hold off;

            % Plot the translated PLY file
            hold on;
            trisurf(faces, translatedVertices(:, 1), translatedVertices(:, 2), translatedVertices(:, 3), ...
                'FaceColor', [0.804, 0.667, 0.490], 'EdgeColor', [0.545, 0.451, 0.333]);
            hold off;

        end

        function getObjectSize(self)
            %get table height from scaled PLY
            self.sizeZ = max(self.objectVertices(:,3));
            self.sizeX = max(self.objectVertices(:,1));
            self.sizeY = max(self.objectVertices(:,2));
        end

    end
end