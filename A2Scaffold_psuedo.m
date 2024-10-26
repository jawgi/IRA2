classdef A2Scaffold_psuedo < handle
    properties (Constant)

        dobotLinkDim = [ ];
        dobotFilename = 'dobot_qMatrix';
        rebelLinkDim = [ ];
        rebelFilename = 'rebel_qMatrix';

        %% number of fruits chosen        
        numFruits = 9;
        
        %% Setting drop off points for each bucket 
        greenGoalPos = [ -0.45, -0.15, 0.9 ];
        orangeGoalPos = [ 0, -0.15, 0.9 ];
        purpleGoalPos = [ 0.45, -0.15, 0.9 ];
        
        smallGreenGoalPos = [ -0.4, 0.85, 0.95 ];
        smallOrangeGoalPos = [ 0, 0.85, 0.95 ];
        smallPurpleGoalPos = [ 0.4, 0.85, 0.95 ];

        mediumGreenGoalPos = [ -0.4, 0.85, 1.25 ];
        mediumOrangeGoalPos = [ 0, 0.85, 1.25 ];
        mediumPurpleGoalPos = [ 0.4, 0.85, 1.25 ];

        largeGreenGoalPos = [ -0.4, 0.85, 1.55 ];
        largeOrangeGoalPos = [ 0, 0.85, 1.55 ];
        largePurpleGoalPos = [ 0.4, 0.85, 1.55 ];

        maxGoalDistError = 0.001;                   % Buffer specified between goalPos and robot end effector
    end

    properties
        dobotModel;
        dobotBase;
        dobotEllipsoidPts;
        dobotQMatrix;
        dobotGoalsCompleted;

        rebelModel;
        rebelBase;
        rebelEllipsoidPts;
        rebelQMatrix;
        rebelGoalsCompleted;
        
        allFruits;

        environmentCl;
        % handles
        mainFig_h;

        taskComplete;           % true - all fruits moved to final sorted buckets. false - fruits still in stage 0 or stage 1 of task completion.
        systemStatus;           % true - system running and robot moving. false - system on standby (either before starting or after an e-stop operation)
        stopStatus;                 % true - e-stop engaged. false - e-stop disengaged
    end
    
    methods
        function self = A2Scaffold_psuedo() %figuring out general flow of code
            tic;
            count = 1;
            %% Simulating the environment with robot models and initialising variables
            self.SimulateEnvironment(false);        
            
            self.dobotBase = SE3(0.4,-0.7,0.8).T;
            dobot = LinearUR3e(self.dobotBase);
            self.dobotModel = dobot.model;
            % self.dobotModel.teach(self.dobotModel.getpos());
            hold on;
            % input("checked reach of dobot?")

            self.rebelBase = SE3(0.4,0.4,0.8).T;
            rebel = LinearUR3e(self.rebelBase);
            self.rebelModel = rebel.model;
            % self.rebelModel.teach(self.rebelModel.getpos());
            hold on;
            % input("checked reach of rebel?");

            % self.CreateRotatedVideo([ -2.5, 2.5, -2.5, 2.5 ,0.01,2], 1.5, 95, 'rotated_video_environment');
            
            input("done loading environment?");

            self.taskComplete = false;
            self.systemStatus = false;
            self.stopStatus = false;
            self.dobotGoalsCompleted = 0;
            self.rebelGoalsCompleted = 0;
            while(~self.taskComplete)
                if ~self.systemStatus                       % System on standby
                    if self.stopStatus                          % E-stop engaged
                        input("Disengage e-stop?");
                        self.stopStatus = false;            % Disengage e-stop
                    else
                        if self.CheckStart                      % System start/resume selected - HAVE TO EDIT FUNCTION FUNCTIONALITY
                            self.systemStatus = true;       % Set system to running
                            self.dobotQMatrix = self.LoadQMatrix(self.dobotModel,self.dobotFilename);       % Load qMatrix for dobot
                            self.rebelQMatrix = self.LoadQMatrix(self.rebelModel,self.rebelFilename);           % Load qMatrix for rebel
                            input("stored qMatrix for both robots?");
                        end
                    end
                else
                    input("Starting operation of robot.");
                    
                    if self.EStopPressed(true) || ~self.safetyTest('sensor')                % Check if hardware or GUI e-stop is pressed or sensor senses object in workspace (unsafe environment)
                        self.stopStatus = true;                                                             % Change relevant variables to stop system operation
                        self.systemStatus = false;
                        self.SaveQMatrix(self.dobotQMatrix,self.dobotFilename);     % Save trajectories for both robots for restarting
                        self.SaveQMatrix(self.rebelQMatrix,self.rebelFilename);
                    end
                    


                    % check if each robotGoalsCompleted complete and only continue if incomplete - 
                    % so when dobot finished but rebel continues, dobot isn't being given any comands
                    % store currentGoalQ for each robot - i.e. # of goals completed+1
                    % create trajectory for each robot
                        % check collisions and store updated trajectory
                    % move robots by respective qMatrix - BUT ONLY THE FIRST VALUE OR FIRST 5 depending on how slow it is
                        % ensure code is also moving faces of fruit
                    % that way the robot doesn't complete entire trajectory without checking e-stop status or whatever
                    % next - CheckGoalCompleted for each robot
                        % if completed goal, then self.robotGoalsCompleted++
                        % initiate DropFruit where it'll simulate fruit dropping naturally

                    count = count +1;       %hypothetical run
                    if count > 2
                        input("Finish?");
                        self.dobotGoalsCompleted = 9;
                        self.rebelGoalsCompleted = 9;
                        if self.dobotGoalsCompleted == self.rebelGoalsCompleted ...
                                        && self.numFruits == self.dobotGoalsCompleted
                            self.taskComplete = true;
                            break;
                        end
                    end
                end

                        %%ASYNCHRONUS STOP/COLLISION TESTING
                        % using tic/tok - after certain random time
                            % loadHuman within safety barriers %can comment in and out in demo 
                            % loadObstacle between robots within robots workspace %can comment in and out in demo
                        %%ASYNCHRONUS STOP/COLLISION TESTING
            end
            delete(strcat(self.dobotFilename,'.xls'));
            pause(1);
            delete(strcat(self.rebelFilename,'.xls'));
            pause(1);
            toc;
            % code for task completion status or at least time taken for entire task
        end
        
        %% Simulates entire environment except the robot models and stores initial locations of fruits
        function SimulateEnvironment(self, pointCloudOn)            
            %% Load floor
            floor = Environment("floor","floor");

            %% Load the worktable
            table = Table();
            tableCl = table.pointCloud;
            
            %% Stage 1 - bucket with all fruits
            buckets = Buckets();
            bucketsCl = buckets.pointCloud;
            
            %% Buckets for collection, sorted by size and colour
            sortedBuckets = SortedBuckets();
            sortedBucketsCl = sortedBuckets.pointCloud;
            
            %% Display enclosure
            walls = Enclosure();
            door = Door();
            shoot = Shoot();
            shootCl = shoot.pointCloud;
            
            %% Display e-stops and sensors
            eStopDoor = EStopObject("doorstop");
            eStopRobot1 = EStopObject("robot1");
            eStopRebel = EStopObject("robot2");
            eStopCollection = EStopObject("collection");
            sensor = DoorSensor();
            camera = CameraObject();
            
            %% Get Point Cloud of all environment and plot if needed
            environmentCl = [tableCl; bucketsCl; sortedBucketsCl; shootCl];
            if pointCloudOn
                plot3(environmentCl(:,1),environmentCl(:,2),environmentCl(:,3),'r.');
            end;
            self.environmentCl = environmentCl;
            
            % load fruits and store locations
            self.allFruits = Fruit("manual",self.numFruits);
            
            for i =1:self.numFruits
                colour = self.allFruits.colourName{i};
                size = self.allFruits.size{i};

                switch colour
                    case 'Green'
                        self.allFruits.midPoint{i} = self.greenGoalPos;
                        switch size
                            case 's'
                                self.allFruits.dropPoint{i} = self.smallGreenGoalPos;
                            case 'm'
                                self.allFruits.dropPoint{i} = self.mediumGreenGoalPos;
                            case 'l'
                                self.allFruits.dropPoint{i} = self.largeGreenGoalPos;
                        end
                    case 'Orange'
                        self.allFruits.midPoint{i} = self.orangeGoalPos;
                        switch size
                            case 's'
                                self.allFruits.dropPoint{i} = self.smallOrangeGoalPos;
                            case 'm'
                                self.allFruits.dropPoint{i} = self.mediumOrangeGoalPos;
                            case 'l'
                                self.allFruits.dropPoint{i} = self.largeOrangeGoalPos;
                        end
                    case 'Purple'
                        self.allFruits.midPoint{i} = self.purpleGoalPos;
                        switch size
                            case 's'
                                self.allFruits.dropPoint{i} = self.smallPurpleGoalPos;
                            case 'm'
                                self.allFruits.dropPoint{i} = self.mediumPurpleGoalPos;
                            case 'l'
                                self.allFruits.dropPoint{i} = self.largePurpleGoalPos;
                        end
                end
            end

            % Populate additional safety features?
            self.PlaceSafety();
            
            % make view nicer - found by manually adjust figure
            set(gcf, 'Windowstyle', 'docked');
            camlight();
            optimalAzEl = [95.8730,17.8284];
            view(optimalAzEl);
            zoom(3);
            axis([ -1, 2.5, -2.5, 2.5 ,0.25,2]);
        end

        %% Creates link ellipsoids for specified robot model and returns point cloud?
        function  robotEllipsoids = CreateLinkEllipsoids(self, robotModel)
            
            centerPoint = [0,0,0];
            radii = [1,0.5,0.5];
            [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
            for i = 1:4
                robot.points{i} = [X(:),Y(:),Z(:)];
                warning off
                robot.faces{i} = delaunay(robot.points{i});    
                warning on;
            end
            robot.plot3d([0,0,0]);
            axis equal
            camlight
        end
        
        %% Checks if start/resume button has been pressed
        function status = CheckStart(self) % ADD FUNCTIONALITY TO CHECK IF RESUME/START PRESSED
            input("Press enter to start. (Will be replaced by actual checking)");
            status = true;
        end

        %% Checks if hardware or GUI e-stop is pressed and returns true or false
        function status = EStopPressed(self,press)
            status = press;
            if status
                input("Stop?");
                disp("E-Stop Engaged! Stopping system operation.");
            else
                disp("E-Stop Disengaged. System operation can continue ...");
            end
        end

        %% % Load qMatrix files for both robot models if available (previously stopped) or create a file (first started)
        function qMatrix = LoadQMatrix(self,robotModel,filename)
            newFilename = strcat(filename,'.xls'); % creates full filename
            if isfile(newFilename)
                 % qMatrix = importdata(strcat(filename,'.mat'))
                 qMatrix = readmatrix(newFilename); %stores previous qMatrix data into variable
            else
                 qMatrix = robotModel.getpos();
                 self.SaveQMatrix(qMatrix,filename);
            end
        end

        %% Checks if goal is reached by comparing current end effector position to goal within specified buffer
        function reached = CheckGoalComplete(self,robotModel,goalPos)
            currentEndEffectorTr = robotModel.fkine(robotModel.getpos()).T;
            dist = self.dist2pts(goalPos,currentEndEffectorTr(1:3,4)');
            if dist < self.maxGoalDistError
                reached = true;
            else
                reached = false;
            end
        end

%% ==============TO BE MODIFIED FOR A2 STILL IF NEEDED==================
        function qMatrix = CalcJointStates(self, endPos)
            % disp("Entered endPos: ");
            % disp(endPos);
            
            % get the current joint states of robot in workspace
            currentPos = self.robotModel.getpos();
            
            % calculate the goal joint states based on required endPos and
            % the current joint state for q0
            % newQ = self.robotModel.ikine(endPos, 'q0', currentPos, 'mask', [1,1,0,0,0,0]);
            newQ = self.robotModel.ikcon(endPos, currentPos);
            
            % use Quintic Polynomial method to generate and store our joint
            % state waypoints over 100 steps
            qMatrix = jtraj(currentPos, newQ, 100);
        end

        function MoveRobot(self,endPos, status)
            % calculate joint states from the desired end transform
            qMatrix = self.CalcJointStates(endPos);

            % display progress message showing which brick is being moved
            disp("Brick Status: (0 is no brick being moved)");
            disp(status);
            % endPos;
            
            %for each joint state, animate it, get the resultant transform then check if brick should
            % be moving too. If so, update brick mesh verties so location
            % updated - latter not really working
            for i =1:size(qMatrix,1)
                self.robotModel.animate(qMatrix(i,:));
                currentTr = self.robotModel.fkineUTS(qMatrix(i,:));
                    switch status
                        case 1-9 %brick moving too
                            verticies = get(self.brick_h(status), 'Verticies');
                            transformedVertices = [verticies, ones(size(verticies,1),1)] *currentTr;
                            set(self.brick_h(status), 'Verticies', transformedVertices(:,1:3));
                        otherwise
                            %nothing - only move brick when it's been picked up
                    end
                % update figure
                pause(0.01);
                drawnow();
            end
            % store final transform and compare to originally asked for position
            % - only outputs when it isn't within tolerance
            finalTr = currentTr;
            if ~isempty(find(0.005 < finalTr-endPos,1))
                warning('Desired EndEffector is different from calculated and animated endPos')
                finalTr
                endPos
            end
        end
   
        %% Sets up the specified safety tests for simulated sensor input and upcoming collision
        function status = safetyTest(self, type)
            switch type
                case {'sensor', 'Sensor'}
                    status = true;
                    % add simulated sensor readings to environment (like someone entered enclosure)
                    % e-stop function when this occurs
                case {'collision', 'Collision'}
                otherwise
                    disp('Invalid test. Specify type of test: sensor/collision');
                    return;
            end
        end
    end

    methods(Static)

        function SaveQMatrix(qMatrix,filename)
            newFilename = strcat(filename, '.xls'); % creates full filename with xls to save qMatrix to spreadsheet
            writematrix(qMatrix, newFilename);
        end

        function dropFruit(fruit)
            % add in gradual drop of fruit for better simulation (modify Z values in for loop?
        end
        
        %% Calculate distance (dist) between consecutive points
        function dist=dist2pts(pt1,pt2)
            % If 2D
            if size(pt1,2) == 2
                dist=sqrt((pt1(:,1)-pt2(:,1)).^2+...
                          (pt1(:,2)-pt2(:,2)).^2);
            % If 3D          
            elseif size(pt1,2) == 3
                dist=sqrt((pt1(:,1)-pt2(:,1)).^2+...
                          (pt1(:,2)-pt2(:,2)).^2+...
                          (pt1(:,3)-pt2(:,3)).^2);
            % If 6D like two poses
            elseif size(pt1,2) == 6
                dist=sqrt((pt1(:,1)-pt2(:,1)).^2+...
                          (pt1(:,2)-pt2(:,2)).^2+...
                          (pt1(:,3)-pt2(:,3)).^2+...
                          (pt1(:,4)-pt2(:,4)).^2+...
                          (pt1(:,5)-pt2(:,5)).^2+...
                          (pt1(:,6)-pt2(:,6)).^2);
            end
        end
        
        function PlaceSafety()
            
            
        end
        
        %% Plots a plane creates meshgrid of points on the plane - may be used for asynchronus safety
        function points = PlotPlane(axis,maxDim,gridInterval,planePoint)
            switch axis
                case {'X','x'}
                    [Y,Z] = meshgrid(-maxDim:gridInterval:maxDim,-maxDim:gridInterval:maxDim);
                    X = repmat(planePoint,size(Y,1),size(Y,2));
                case {'Y','y'}
                    [X,Z] = meshgrid(-maxDim:gridInterval:maxDim,-maxDim:gridInterval:maxDim);
                    Y = repmat(planePoint,size(X,1),size(X,2));
                case {'Z','z'}
                    [X,Y] = meshgrid(-maxDim:gridInterval:maxDim,-maxDim:gridInterval:maxDim);
                    Z = repmat(planePoint,size(X,1),size(X,2));
                otherwise
                    disp('axis entered invalid. Specify X, Y, or Z')
                    points = nan(1,3);
                    return;
            end
            points = [X(:),Y(:),Z(:)];
            surf(X,Y,Z);
        end

        %% Plots foreign object in current figure and obtains point cloud
        function points = PlotForeignObject(shape,position,colour,side1,side2,side3)
            center = position;
            alpha = 0.5;

            switch shape
                case 'sphere'
                    radius = side1;
                    % Create the sphere
                    [X, Y, Z] = sphere(30); % 30 specifies the resolution of the sphere
                
                    % Scale and shift the sphere to the desired position and size
                    X = radius * X + center(1);
                    Y = radius * Y + center(2);
                    Z = radius * Z + center(3);
                
                case 'rectangle'
                    if nargin < 5
                        side2 = side1;              % only given 1 side dimension - square
                    end
                    centerpnt = position;
                    [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side1/2, centerpnt+side2/2);
                    % need to figure out how to get point cloud

                case 'ellipsoid'
                    if nargin == 6
                        radii = [side1,side2,side3];
                    else
                        disp('3 radii not specified - resorting to default.');
                        radii = [0.1,0.2,0.3]; % Default radii if not fully specified
                    end
                    [X,Y,Z] = ellipsoid( center(1), center(2), center(3), radii(1), radii(2), radii(3));

                otherwise
                    disp('Specify shape of foreign object: (Sphere/Rectangle/Ellipsoid)');
                    points = nan(1,3);
                    return;
            end

            % Plot object
            points = [X(:),Y(:),Z(:)];
            surf(X, Y, Z, 'FaceAlpha', alpha, 'EdgeColor','none', 'FaceColor',colour);
        end

        %% Plots point cloud from matrix of points with modifiable parameters 
        function pointCloud_h = PlotPointCloud(points,colour,shape) 
            X = points(:,1);
            Y = points(:,2);
            Z = points(:,3);
        
            pointCloud_h = plot3(X,Y,Z,strcat(colour,shape));
        end
        
        %% Rotated Video Creation - sourced online and modified for use
        % https://au.mathworks.com/matlabcentral/answers/397810-creating-an-animated-video-by-rotating-a-surface-plot#answer_1532350
        % inputs - axis limits and zoom level for modifying current plot
        % view, final exported video quality and file name
        function CreateRotatedVideo(axisLim,zoomLvl,vidQuality,vidName)
            
            % update view of plot and use renderer for better image quality
            axis tight; % Keep axis limits stable
            zoom(zoomLvl);
            % [ -2.5, 2.5, -2.5, 2.5 ,0.01,2]
            axis(axisLim);
            
            input("check");

            %set up VideoWriter
            v = VideoWriter(strcat(vidName,'.avi'), 'Motion JPEG AVI');
            v.Quality = vidQuality; %set quality (70 default - up to 100)
            open(v);

            % Specify the number of frames for the animation
            numFrames = 360;

            % Rotate the surface plot and capture frames
            for k = 1:numFrames
                % Change the view angle
                view([k, 30]);
                
                % Capture the current frame
                frame = getframe(gcf);
                writeVideo(v, frame);
            end

            % Complete the video writing process
            close(v);
        end
    end
end