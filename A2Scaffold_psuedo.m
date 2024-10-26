classdef A2Scaffold_psuedo < handle
    properties (Constant)

        dobotLinkDim = [ ];
        dobotFilename = 'dobot_qMatrix';
        rebelLinkDim = [ ];
        rebelFilename = 'rebel_qMatrix';
        steps = 50;
        
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
        currentDobotGoal;
        nextDobotGoal;
        dobotStatus;

        rebelModel;
        rebelBase;
        rebelEllipsoidPts;
        rebelQMatrix;
        rebelGoalsCompleted;
        currentRebelGoal;
        nextRebelGoal;
        rebelStatus;
        
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

            self.taskComplete = false;                  % Initialise all variables
            self.systemStatus = false;
            self.stopStatus = false;
            self.dobotGoalsCompleted = 0;
            self.rebelGoalsCompleted = 0;
            self.dobotStatus = 0;                           % 0 is picking up, 0 is placing down.
            self.rebelStatus = 0;
            
            %% Start of loop - continues until task is complete
            while(~self.taskComplete)                   
                if ~self.systemStatus                       % System on standby
                    %% Checking system status and e-stop status to adhere to two action start/resume
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
                    
                    %% Checking if E-stop pressed or safetyTest found unsafe to continue
                    if self.EStopPressed(true) || ~self.safetyTest('sensor')                % Check if hardware or GUI e-stop is pressed or sensor senses object in workspace (unsafe environment)
                        self.stopStatus = true;                                                             % Change relevant variables to stop system operation
                        self.systemStatus = false;
                        self.SaveQMatrix(self.dobotQMatrix,self.dobotFilename);     % Save trajectories for both robots for restarting
                        self.SaveQMatrix(self.rebelQMatrix,self.rebelFilename);
                    end
                    
                    %% Checking if each robot has goals to complete and storing the picking and placing goals from next fruit to be picked
                    if self.dobotGoalsCompleted < self.numFruits
                        self.currentDobotGoal = self.allFruits.startPoint{self.dobotGoalsCompleted+1};
                        self.nextDobotGoal = self.allFruits.midPoint{self.dobotGoalsCompleted+1};
                        
                        % Checking if dobot is in the state of picking or placing and calculate joint states depending on that
                        if ~self.dobotStatus 
                            self.dobotQMatrix = self.CalcJointStates(self.dobotModel,self.currentDobotGoal,self.steps,'quintic');
                            dobotGoal = self.currentDobotGoal;
                        else
                            dobotGoal = self.nextDobotGoal;
                            self.dobotQMatrix = self.CalcJointStates(self.dobotModel,self.nextDobotGoal,self.steps,'quintic');
                        end
                        % check collisions and store updated trajectory
                    end
                    % Same as above for rebel
                    if self.rebelGoalsCompleted < self.numFruits && self.dobotGoalsCompleted > 0
                        self.currentRebelGoal = self.allFruits.startPoint{self.rebelGoalsCompleted+1};
                        self.nextRebelGoal = self.allFruits.midPoint{self.rebelGoalsCompleted+1};
                        if ~self.rebelStatus
                            rebelGoal = self.currentRebelGoal;
                            self.rebelQMatrix = self.CalcJointStates(self.rebelModel,self.currentRebelGoal,self.steps,'quintic');
                        else
                            rebelGoal = self.nextRebelGoal;
                            self.rebelQMatrix = self.CalcJointStates(self.rebelModel,self.nextRebelGoal,self.steps,'quintic');
                        end
                        % check collisions and store updated trajectory
                    end
                    
                    %% Moving robots by selected steps in trajectory - to give opportunity for e-stop/asynchronous safety test
                    if self.dobotGoalsCompleted ~= self.numFruits
                        self.MoveRobot('dobot',5);
                        dobotReached = CheckGoalComplete(self.dobotModel,dobotGoal);
                        if dobotReached
                            self.dobotGoalsCompleted = self.dobotGoalsCompleted + 1;
                            self.DropFruit(self.dobotGoalsCompleted);
                        end
                    end
                    self.MoveRobot('rebel',5);
                    rebelReached = CheckGoalComplete(self.rebelModel,rebelGoal);
                    if rebelReached
                        self.rebelGoalsCompleted = self.rebelGoalsCompleted + 1;
                        self.DropFruit(self.rebelGoalsCompleted);
                    end

                    %% Checking if all goals completed and setting taskComplete if so
                    if self.dobotGoalsCompleted == self.rebelGoalsCompleted ...
                                    && self.numFruits == self.dobotGoalsCompleted
                        self.taskComplete = true;
                        break;
                    end
                end

            %% ASYNCHRONUS STOP/COLLISION TESTING
                % using tic/tok - after certain random time
                % loadHuman within safety barriers %can comment in and out in demo 
                % loadObstacle between robots within robots workspace %can comment in and out in demo
            end
            %% Deleting created files after task completion
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
        function  robotEllipsoids = CreateLinkEllipsoids(self, robotModel)      % ADD FUNCTIONALITY 
            
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
        function status = CheckStart(self)                                      % ADD FUNCTIONALITY TO CHECK IF RESUME/START PRESSED
            input("Press enter to start. (Will be replaced by actual checking)");
            status = true;
        end

        %% Checks if hardware or GUI e-stop is pressed and returns true or false
        function status = EStopPressed(self,press)                          % ADD FUNCTIONALITY 
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

        %% Calculates qMatrix of joint states for specified robot end effector postion and with specified interpolation method
        function finalQMatrix = CalcJointStates(self,robotModel, endPos, steps, method)
            currentQ = robotModel.getpos();                                      % get the current joint states of robot in figure
            newQ = robotModel.ikcon(SE3(endPos), currentQ);        % calculate the goal joint states based on required endPos and the current joint state
        
            switch method
                case 'trapezoidal'                                                          % trapezoidal velocity  - minimum time
                    s = lspb(0,1,steps);                                                    % First, create the scalar function
                    qMatrix = nan(steps,robotModel.n);                           % Create memory allocation for variables
                    for i = 1:steps
                        qMatrix(i,:) = (1-s(i))*currentQ + s(i)*newQ;           % Generate interpolated joint angles
                    end
                case 'quintic' % quintic polynomial - minimum jerk
                    qMatrix = jtraj(currentQ,newQ,steps);
     
                case 'rmrc' % Resolved Motion Rate Control
                    error("Needs to be done still");
                otherwise
                   error("Specify method of calculating joint state: (trapezoidal/minimum jerk/RMRC)")
            end
            finalQMatrix = qMatrix;
        end

        %% 
        function MoveRobot(self,robotName,deltaQ)
            
            switch robotName
                case 'dobot'
                    % fruitNum = self.dobotGoalsCompleted+1;
                    % currentFruit = allFruits.handle{fruitNum};
                    assert(deltaQ<=length(self.dobotQMatrix),'deltaQ chosen is larger than number of steps in trajectory');
                    self.dobotModel.animate(self.dobotQMatrix(1:deltaQ,:));
                    %move fruit with end effector based on current fruit figure?
                case 'rebel'
                    % fruitNum = self.rebelGoalsCompleted+1;
                    % currentFruit = allFruits.handle{fruitNum};
                    self.rebelModel.animate(self.rebelQMatrix(1:deltaQ,:));
                    %move fruit with end effector based on current fruit figure?
                otherwise
                    error('Invalid robot chosen. Specify (dobot/rebel)');
            end
            
            % %for each joint state, animate it, get the resultant transform then check if brick should
            % % be moving too. If so, update brick mesh verties so location
            % % updated - latter not really working
            % for i =1:size(qMatrix,1)
            %     self.robotModel.animate(qMatrix(i,:));
            %     currentTr = self.robotModel.fkineUTS(qMatrix(i,:));
            %         switch status
            %             case 1-9 %brick moving too
            %                 verticies = get(self.brick_h(status), 'Verticies');
            %                 transformedVertices = [verticies, ones(size(verticies,1),1)] *currentTr;
            %                 set(self.brick_h(status), 'Verticies', transformedVertices(:,1:3));
            %             otherwise
            %                 %nothing - only move brick when it's been picked up
            %         end
            %     % update figure
            %     pause(0.01);
            %     drawnow();
            % end
        end
   
        %% Sets up the specified safety tests for simulated sensor input and upcoming collision
        function status = safetyTest(self, type)                                % ADD FUNCTIONALITY 
            switch type
                case {'sensor', 'Sensor'}
                    % add simulated sensor readings to environment (like someone entered enclosure) 
                    % and see if showing up as true i.e. unsafe to continue
                    status = true;
                    
                    self.stopStatus = true;
                    self.systemStatus = false;
                case {'collision', 'Collision'}
                otherwise
                    error('Invalid test. Specify type of test: (sensor/collision)');
            end
        end

       function DropFruit(self,fruitNum)
            % add in gradual drop of fruit for better simulation (modify Z values in for loop?
            disp("Fruit dropping yay.");
        end
    end

    methods(Static)

        function SaveQMatrix(qMatrix,filename)
            newFilename = strcat(filename, '.xls'); % creates full filename with xls to save qMatrix to spreadsheet
            writematrix(qMatrix, newFilename);
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
                    error('Axis entered invalid. Specify X, Y, or Z');
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
                    error('Specify shape of foreign object: (Sphere/Rectangle/Ellipsoid)');
                    
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
        % Example: self.CreateRotatedVideo([ -2.5, 2.5, -2.5, 2.5 ,0.01,2], 1.5, 95, 'rotated_video_environment');
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