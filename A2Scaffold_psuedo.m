classdef A2Scaffold_psuedo < handle
    properties (Constant)
        dobotFruitGoal = [0.6,0,0.0;        0.6,0.17,0;         0.6,-0.17,0; ...
                              0.6,0,0.03;       0.6,0.17,0.03;      0.6,-0.17,0.03; ...
                              0.6,0,0.06;       0.6,0.17,0.06;      0.6,-0.17,0.06;];

        rebelFruitPos = [0.6,0,0.0;        0.6,0.17,0;         0.6,-0.17,0; ...
                              0.6,0,0.03;       0.6,0.17,0.03;      0.6,-0.17,0.03; ...
                              0.6,0,0.06;       0.6,0.17,0.06;      0.6,-0.17,0.06;];

        rebelFruitGoal = [0.6,0,0.0;        0.6,0.17,0;         0.6,-0.17,0; ...
                              0.6,0,0.03;       0.6,0.17,0.03;      0.6,-0.17,0.03; ...
                              0.6,0,0.06;       0.6,0.17,0.06;      0.6,-0.17,0.06;];
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
    end

    properties
        dobotModel;
        dobotBase;
        rebelModel;
        rebelBase;
        
        testFruits;
        dobotFruitPos;

        environmentCl;
        % handles
        mainFig_h;
    end
    
    methods
        function self = A2Scaffold_psuedo() %figuring out general flow of code
            tic;
            %% Simulating the environment with robot models.

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

            %initialising fruit positions
            disp("These are fruit locations:");
            self.dobotFruitPos = zeros(self.numFruits,3);
            for i =1:self.numFruits
                self.dobotFruitPos(i,:) = self.testFruits.startPoint{i};
            end
            disp(self.dobotFruitPos);
            
            allGoalPos = [ self.greenGoalPos;
                                    self.orangeGoalPos;
                                    self.purpleGoalPos;
                                    self.smallGreenGoalPos;
                                    self.smallOrangeGoalPos;
                                    self.smallPurpleGoalPos;
                                    self.mediumGreenGoalPos;
                                    self.mediumOrangeGoalPos;
                                    self.mediumPurpleGoalPos;
                                    self.largeGreenGoalPos;
                                    self.largeOrangeGoalPos;
                                    self.largePurpleGoalPos]
            for i = 1:12
                size = 0.025;
                colour = 'r';
                self.PlotForeignObject('sphere',allGoalPos(i,:),size,size,colour);
            end

            input("done?");

            % taskComplete = false
            % systemStatus = standby
            % stopStatus = false;
                % while(~taskComplete)
                    % if resume/start button pressed || systemStatus == running
                        % stopStatus = false;
                        % if systemStatus == standby
                            % load qMatrix files for both robot models if available(previously stopped) or create a file (first started)
                            % systemStatus = running

                        % if eStop pressed || human detected within safety barriers
                            % store current qMatrix for both robots into files (replace old ones)
                            % stopStatus = true
                            % systemStatus = standby
                        % else
                            % check each robot taskcompletion i.e. dobotGoalsCompleted and rebelGoalsCompleted
                            % store next q value for both robots (for loop?)
                                % only store q values for rebel if dobotGoalsCompleted >0 and currently has one fruit in rebel container
                            % check both robots for collisions in that q position
    
                            % if (~(dobotCollision || rebelCollision)) - none colliding
                                % move both robots to stored q value
                                % if current q value is also goal/final q value for both robots 
                                    % update dobotGoalsCompleted and rebelGoalsCompleted
                                    % if both of = 9 (or however many fruits)
                                        % taskComplete = true (maybe vertex for each robot so [true,true] needed for task complete)
                            % else (colliding)
                                % find different q that doesn't collide and connects to next q
                                % update qMatrix and add in to replace current q

                        %%ASYNCHRONUS STOP/COLLISION TESTING
                        % using tic/tok - after certain random time
                            % loadHuman within safety barriers %can comment in and out in demo 
                            % loadObstacle between robots within robots workspace %can comment in and out in demo
                        %%ASYNCHRONUS STOP/COLLISION TESTING
                % end
            toc;
            % code for task completion status or at least time taken for entire task
        end

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
            self.testFruits = Fruit("manual",self.numFruits);

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

        function robotEllipsoids = CreateLinkEllipsoids(self, robotModel)
            
        end

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
        function self.safetyTest(self, type)
            switch type
                case {'sensor', 'Sensor'}
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

        function dropFruit(fruit)
            % add in gradual drop of fruit for better simulation (modify Z values in for loop?
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
         function points = PlotForeignObject(shape,position,minSide,maxSide,colour)
            switch shape
                case 'sphere'
                    radius = minSide
                    center = position
            
                    % Create the sphere
                    [X, Y, Z] = sphere(30); % 30 specifies the resolution of the sphere
            
                    % Scale and shift the sphere to the desired position and size
                    X = radius * X + center(1);
                    Y = radius * Y + center(2);
                    Z = radius * Z + center(3);
                case 'rectangle'
                centerpnt = position;
                [vertex,faces,faceNormals] = RectangularPrism(centerpnt-minSide/2, centerpnt+maxSide/2);
                otherwise
                    disp('Specify shape of foreign object: (Sphere/Rectangle)');
                    points = nan(1,3);
                    return;
            end
            %Plot object
            points = [X(:),Y(:),Z(:)];
            surf(X, Y, Z, 'FaceAlpha', 0.7, 'EdgeColor','none', 'FaceColor',colour);
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