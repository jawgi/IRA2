classdef A2Scaffold_psuedo < handle
    properties (Constant)

        dobotLinkDim = [ ];
        dobotFilename = 'dobot_qMatrix';
        rebelLinkDim = [ ];
        rebelFilename = 'rebel_qMatrix';
        defaultSteps = 10;
        defaultDeltaQ = 2;
        resetQ = [-0.1,zeros(1,6)];
        
        %% number of fruits chosen        
        numFruits = 3;
        
        %% Setting drop off points for each bucket 
        greenGoalPos = [ -0.3, -0.225, 1 ];
        orangeGoalPos = [ 0, -0.225, 1 ];
        purpleGoalPos = [ 0.3, -0.225, 1 ];
        
        smallGreenGoalPos = [ 0.3, 0.5, 1.3 ];
        smallOrangeGoalPos = [ 0, 0.5, 1.3 ];
        smallPurpleGoalPos = [ -0.3, 0.5, 1.3 ];

        mediumGreenGoalPos = [ 0.3, 0.47, 1.125 ];
        mediumOrangeGoalPos = [ 0, 0.47, 1.125 ];
        mediumPurpleGoalPos = [ -0.3, 0.47, 1.125 ];

        largeGreenGoalPos = [ 0.3, 0.47, 0.95 ];
        largeOrangeGoalPos = [ 0, 0.47, 0.95 ];
        largePurpleGoalPos = [ -0.3, 0.47, 0.95 ];

        maxGoalDistError = 0.01;                   % Buffer specified between goalPos and robot end effector
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

        taskComplete = false;           % true - all fruits moved to final sorted buckets. false - fruits still in stage 0 or stage 1 of task completion.
        systemStatus;                       % true - system running and robot moving. false - system on standby (either before starting or after an e-stop operation)
        stopStatus;                            % true - e-stop engaged. false - e-stop disengaged
        steps = 40;
        deltaQ = 10;
    end
    
    methods
        function self = A2Scaffold_psuedo() 
            tic;
            %% Simulating the environment with robot models and initialising variables
            self.SimulateEnvironment(false);        
            
            self.dobotBase = SE3(0.4,-0.625,0.8).T;
            dobot = LinearUR3e(self.dobotBase);
            self.dobotModel = dobot.model;
            hold on;

            self.rebelBase = SE3(0.4,0.2,0.8).T;
            rebel = LinearUR3e(self.rebelBase);
            self.rebelModel = rebel.model;
            hold on;

            input("done loading environment?");

            % self.CheckAllTaskLocations();
            % self.RunBasicMode();
            
            %% Initialising all variables for main loop
            self.taskComplete = false;                  
            self.systemStatus = false;
            self.stopStatus = false;
            self.dobotGoalsCompleted = 0;
            self.rebelGoalsCompleted = 0;
            self.dobotStatus = 0;                           % 0 is picking up, 1 is placing down.
            self.rebelStatus = 0;
            
            %% Start of loop - continues until task is complete
            while(1)                   
                if ~self.systemStatus                           % System on standby
                    %% Checking system status and e-stop status to adhere to two action start/resume
                    if self.stopStatus                              % E-stop engaged
                        input("Disengage e-stop?");
                        self.stopStatus = false;                % Disengage e-stop
                    elseif self.CheckStart()                      % System start/resume selected - HAVE TO EDIT FUNCTION FUNCTIONALITY
                            self.systemStatus = true;           % Set system to running
                            self.dobotQMatrix = self.LoadQMatrix(self.dobotModel,self.dobotFilename);       % Load qMatrix for dobot
                            self.rebelQMatrix = self.LoadQMatrix(self.rebelModel,self.rebelFilename);           % Load qMatrix for rebel
                            input("stored qMatrix for both robots?");
                    end
                else
                    disp("Operating robot still...");
                    
                    %% Checking if each robot has goals to complete and storing the picking and placing goals from next fruit to be picked
                    self.dobotGoalsCompleted
                    if self.dobotGoalsCompleted < self.numFruits
                        % Checking if dobot is in the state of picking or placing and calculate joint states depending on that
                        if ~self.dobotStatus % false = picking up, true = placing down
                            fruitStartPoints = self.FruitPointCloud(self.dobotGoalsCompleted+1,100);                                                                               %Getting sample of points on surface of fruit to give better pose options
                            self.currentDobotGoal = self.CheckReachable(self.dobotModel,fruitStartPoints,false);                                                            %Finding best pose to get within maxErrorDist
                            dobotGoalReachable = self.currentDobotGoal{1};  %for debugging
                            dobotGoalQ = self.currentDobotGoal{2};
                            dobotErrorDist = self.currentDobotGoal{3};
                            dobotGoalTr = self.currentDobotGoal{4};
                        else
                            self.nextDobotGoal = self.CheckReachable(self.dobotModel,self.allFruits.midPoint{self.dobotGoalsCompleted+1},false);     %Using set array of midPoints associated with bucket area to find best pose
                            dobotGoalReachable = self.nextDobotGoal{1};
                            dobotGoalQ = self.nextDobotGoal{2};
                            dobotErrorDist = self.nextDobotGoal{3};
                            dobotGoalTr = self.nextDobotGoal{4};
                        end
                        self.dobotQMatrix = self.CalcJointStates(self.dobotModel,dobotGoalQ,self.defaultSteps,'quintic','basic');   %basic mode - using Q pose rather than point
                        disp("dobot joint states calculated");

                        % check collisions and store updated trajectory - maybe within calc joint states
                    end

                    % Same as above for rebel but only start if dobot has completed at least 1 goal
                    if self.rebelGoalsCompleted < self.numFruits && self.dobotGoalsCompleted > 0
                        if ~self.rebelStatus
                            self.currentRebelGoal = self.CheckReachable(self.rebelModel,self.allFruits.midPoint{self.rebelGoalsCompleted+1},false);     %Finding best pose to pick up from stage 2
                            rebelGoalReachable = self.currentRebelGoal{1};
                            rebelGoalQ = self.currentRebelGoal{2};
                            rebelErrorDist = self.currentRebelGoal{3};
                            rebelGoalTr = self.currentRebelGoal{4};
                        else
                            self.nextRebelGoal = self.CheckReachable(self.rebelModel,self.allFruits.dropPoint{self.rebelGoalsCompleted+1},false);       %Finding best pose to drop to stage 3
                            rebelGoalReachable = self.nextRebelGoal{1};
                            rebelGoalQ = self.nextRebelGoal{2};
                            rebelErrorDist = self.nextRebelGoal{3};
                            rebelGoalTr = self.nextRebelGoal{4};
                        end
                        self.rebelQMatrix = self.CalcJointStates(self.rebelModel,rebelGoalQ,self.defaultSteps,'quintic','basic');   %basic mode - using Q pose rather than point
                        disp("rebel joint states calculated");

                        % check collisions and store updated trajectory - maybe within calc joint states
                    end
                    
                    %% Moving robots by number of steps in trajectory - to give opportunity for e-stop/asynchronous safety test
                    if self.dobotGoalsCompleted > 0
                        self.MoveRobot('rebel',self.defaultDeltaQ,self.rebelGoalsCompleted+1,self.rebelQMatrix,'advanced',self.rebelStatus);
                        disp('rebel moved');

                        [rebelReached,rebelDist]  = self.CheckGoalComplete(self.rebelModel,rebelGoalTr(1:3,4)');
                        disp(['rebel goals done before check = ', num2str(self.rebelGoalsCompleted)]);
                        
                        if rebelReached
                            switch self.rebelStatus
                                case 1
                                    self.rebelGoalsCompleted = self.rebelGoalsCompleted + 1;
                                    self.rebelStatus = 0; % Now picking up
                                    self.DropFruit(self.rebelGoalsCompleted,'drop');
                                    input('check rebel dropped');
                                case 0
                                    self.rebelStatus = 1; % Now placing down
                            end
                            if self.rebelGoalsCompleted == 9
                                reset = self.MoveRobot('rebel',self.defaultDeltaQ,self.dobotGoalsCompleted,self.resetQ,'basic',-1)
                            end
                        end
                        disp(['rebel goals done after check = ', num2str(self.rebelGoalsCompleted)]);
                    end

                    if self.dobotGoalsCompleted ~= self.numFruits                   %Until dobot picking and placing completed
                        moved = self.MoveRobot('dobot',self.defaultDeltaQ, self.dobotGoalsCompleted+1,self.dobotQMatrix,'advanced',self.dobotStatus)       %default mode just needs interval of steps in stored qMatrix
                        disp('dobot moved');

                        [dobotReached,dobotDist] = self.CheckGoalComplete(self.dobotModel,dobotGoalTr(1:3,4)');
                        disp(['dobot goals done before check = ', num2str(self.dobotGoalsCompleted)]);

                        if dobotReached
                            switch self.dobotStatus
                                case 1
                                    self.dobotGoalsCompleted = self.dobotGoalsCompleted + 1;
                                    self.dobotStatus = 0; % Now picking up
                                    self.DropFruit(self.dobotGoalsCompleted,'mid');
                                    input('check dobot dropped');
                                case 0
                                    self.dobotStatus = 1; % Now placing down
                            end
                        end
                        disp(['dobot goals done after check = ', num2str(self.dobotGoalsCompleted)]);

                    else            %Dobot job done - go home
                        reset = self.MoveRobot('dobot',self.defaultDeltaQ,self.dobotGoalsCompleted,self.resetQ,'basic',-1)
                    end
                    
                    
                    
                    %% Checking if E-stop pressed or safetyTest found unsafe to continue
                    if self.EStopPressed(false) || ~self.SafetyTest('sensor')             % Check if hardware or GUI e-stop is pressed or sensor senses object in workspace (unsafe environment)
                        self.stopStatus = true;                                                             % Change relevant variables to stop system operation
                        self.systemStatus = false;
                        self.SaveQMatrix(self.dobotQMatrix,self.dobotFilename);     % Save trajectories for both robots for restarting
                        self.SaveQMatrix(self.rebelQMatrix,self.rebelFilename);
                    end

                    %% Checking if all goals completed and setting taskComplete if so
                    if self.dobotGoalsCompleted == self.rebelGoalsCompleted ...
                                    && self.numFruits == self.dobotGoalsCompleted
                        self.taskComplete = true;
                    end
                end

                %% ASYNCHRONUS STOP/COLLISION TESTING

                if self.dobotGoalsCompleted == 1
                    self.SafetyTest('collision')                          % plots foreign object within robot workspace and is added to environmentPtCl - simulating sensor/laser
                elseif self.dobotGoalsCompleted == 2
                    location = [-3,3,0];
                    human = Human(location);                        % load random human in workspace or set location outside of it
                    axis equal;
                end
    
                if self.taskComplete
                    delete(strcat(self.dobotFilename,'.xls'));      % Deleting created files after task completion
                    pause(1);
                    delete(strcat(self.rebelFilename,'.xls'));
                    pause(1);
                    exit('Task Completed - yay');
                end
            end
            toc;
            % code for task completion status or at least time taken for entire task
        end
        
        %% Basic Mode with moving fruit one by one - exits code execution after completed
        function RunBasicMode(self)
            resetQ = self.resetQ;
            while(~self.taskComplete)
                for i = 2:self.numFruits                
                    if i == 4
                        location = [-3,3,0];
                        human = Human(location);            % replace with random time? or manual after # of fruit until demo day?
                        axis equal;
                    end

                    disp("Testing simulated sensor input - i.e. human in enclosure:");
                    safeToOperate = self.SafetyTest('sensor')
                    if ~safeToOperate
                        error("Human detected within enclosure - stopping until safe to continue...");
                    end
                    % input("check");

                    disp("Testing forced simulated upcoming collision");
                    safeToOperate = self.SafetyTest('collision')
                    if ~safeToOperate
                        error("System is colliding with object - stopping until safe to conitinue...");
                    end
                    % input("check");
                    
                    fruitStartTr = self.allFruits.startPoint{i}.T;
                    fruitStartPt = fruitStartTr(1:3,4)';
                    % input("check");

                    dobotStartQ = self.CalcJointStates(self.dobotModel,fruitStartPt,self.steps,'quintic','other');
                    dobotStage1 = self.MoveRobot('dobot',1,i,dobotStartQ,'basic',0);
                    disp(['Picked up fruit ', num2str(i), ' from 1st Stage - ', num2str(dobotStage1)]);
                    % input("check");
                    
                    fruitMidPose = self.CheckReachable(self.dobotModel,self.allFruits.midPoint{i},false)
                    fruitMidQ = fruitMidPose{2};

                    dobotMidQ = self.CalcJointStates(self.dobotModel,fruitMidQ,self.steps,'quintic','basic');
                    dobotStage2 = self.MoveRobot('dobot',1,i,dobotMidQ,'basic',1);
                    dobotDropped = self.DropFruit(i,'mid');
                    disp(['Dropped fruit ', num2str(i), ' to 2nd Stage - ',num2str(dobotDropped)]);
                    dobotResetQ = self.CalcJointStates(self.dobotModel,resetQ,self.steps,'quintic','basic');
                    dobotReset = self.MoveRobot('dobot',5,i,dobotResetQ,'basic',-1);
                    % input("check");
                    
                    fruitMidPose = self.CheckReachable(self.rebelModel,self.allFruits.midPoint{i},false);
                    fruitMidQ = fruitMidPose{2};

                    rebelMidQ = self.CalcJointStates(self.rebelModel,fruitMidQ,self.steps,'quintic','basic');
                    rebelStage2 = self.MoveRobot('rebel',1,i,rebelMidQ,'basic',0);
                    disp(['Picked up fruit ', num2str(i), ' from 2nd Stage - ', num2str(rebelStage2)]);
                    % input("check");

                    fruitDropPose = self.CheckReachable(self.rebelModel,self.allFruits.dropPoint{i},false);
                    fruitDropQ = fruitDropPose{2};

                    rebelDropQ = self.CalcJointStates(self.rebelModel,fruitDropQ,self.steps,'quintic','basic');
                    rebelStage3= self.MoveRobot('rebel',1,i,rebelDropQ,'basic',1);
                    rebelDropped = self.DropFruit(i,'drop');
                    disp(['Dropped fruit ', num2str(i), ' to 3rd Stage - ', num2str(rebelDropped)]);
                    rebelResetQ = self.CalcJointStates(self.rebelModel,resetQ,self.steps,'quintic','basic');
                    rebelReset = self.MoveRobot('rebel',5,i,rebelResetQ,'basic',-1);
                    % input("check");
                end
                self.taskComplete = true;
                disp("task is complete");
            end
            error("Reached end.");                                                                  %Temporary so we don't move into main while loop
        end

        %% Checks each goal location and animates end poses for fruits and robots
        function CheckAllTaskLocations(self)
            for i = 1:self.numFruits
                i
                for j = 1:3
                    j
                    switch j
                        case 1
                            disp("Start Fruit Positions:");
                            robotModel = self.dobotModel;
                            fruitSurfacePts = self.FruitPointCloud(i,100);
                        case 2
                            disp("Mid Fruit Positions:");
                            fruitSurfacePts = self.allFruits.midPoint{i};
                        case 3
                            disp("Drop Fruit Positions:");
                            fruitSurfacePts = self.allFruits.dropPoint{i};
                            robotModel = self.rebelModel;
                    end
                
                    animateOn = false;
                    if j == 2
                        reachablePose = self.CheckReachable(self.dobotModel,fruitSurfacePts,animateOn)
                        optimalQ = reachablePose{2};
                        optimalTr = reachablePose{4};
                        plotted = self.ReplotFruit(i,optimalTr);
                        self.dobotModel.animate(optimalQ);
                        drawnow();
                        input("checked optimal Q?");
                        self.dobotModel.animate([0,0,0,0,0,0,0]);
                        drawnow();
    
                        reachablePose = self.CheckReachable(self.rebelModel,fruitSurfacePts,animateOn)
                        optimalQ = reachablePose{2};
                        optimalTr = reachablePose{4};
                        plotted = self.ReplotFruit(i,optimalTr);
                        self.rebelModel.animate(optimalQ);
                        drawnow();
                        input("checked optimal Q?");
                        self.rebelModel.animate([0,0,0,0,0,0,0]);
                        drawnow();
                    else
                        reachablePose = self.CheckReachable(robotModel,fruitSurfacePts,animateOn)
                        reachable = reachablePose{1};
                        optimalQ = reachablePose{2};
                        errorDistance = reachablePose{3};
                        optimalTr = reachablePose{4};
                        plotted = self.ReplotFruit(i,optimalTr);
                        robotModel.animate(optimalQ);
                        drawnow();
                        input("check optimal Q?");
                    end
                end
            end
        end

        %% Simulates entire environment except the robot models and stores initial locations of fruits
        function SimulateEnvironment(self, pointCloudOn)            
            %% Load floor
            floor = Environment("floor","floor");

            %% Load the worktable
            table = Table();
            tableCl = table.pointCloud;
            
            %% All bucket stages
            buckets = AllBuckets();
            bucketsCl = buckets.pointCloud;
            
            %% Display enclosure
            walls = Enclosure();
            door = Door();
            % shoot = Shoot();
            % shootCl = shoot.pointCloud;
            
            %% Display e-stops and sensors
            eStopDoor = EStopObject("doorstop");
            eStopRobot1 = EStopObject("robot1");
            eStopRebel = EStopObject("robot2");
            eStopCollection = EStopObject("collection");
            sensor = DoorSensor();
            camera = CameraObject();
            
            %% Get Point Cloud of all environment and plot if needed
            % environmentCl = [tableCl; bucketsCl; sortedBucketsCl; shootCl]; %with shoot
            environmentCl = [tableCl; bucketsCl;]; %without shoot
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
                        self.allFruits.midPoint{i} = self.CreatePlane('Z',0.03,0.01,self.greenGoalPos);
                        self.allFruits.midPoint{i} = [self.greenGoalPos;self.allFruits.midPoint{i}];
                        switch size
                            case 's'
                                % self.allFruits.dropPoint{i} = self.CreatePlane('Z',0.03,0.01,self.smallGreenGoalPos);
                                 self.allFruits.dropPoint{i} = self.PlotForeignObject('rectangle',self.smallGreenGoalPos,'r',0.2,0.05,0.03,0);
                                 self.allFruits.dropPoint{i} = [self.smallGreenGoalPos;self.allFruits.dropPoint{i}];
                            case 'm'
                                self.allFruits.dropPoint{i} = self.CreatePlane('Z',0.03,0.01,self.mediumGreenGoalPos);
                                self.allFruits.dropPoint{i} = [self.mediumGreenGoalPos;self.allFruits.dropPoint{i}];
                            case 'l'
                                self.allFruits.dropPoint{i} = self.CreatePlane('Z',0.03,0.01,self.largeGreenGoalPos);
                                self.allFruits.dropPoint{i} = [self.largeGreenGoalPos;self.allFruits.dropPoint{i}];
                        end
                    case 'Orange'
                        self.allFruits.midPoint{i} = self.CreatePlane('Z',0.03,0.01,self.orangeGoalPos);
                        self.allFruits.midPoint{i} = [self.orangeGoalPos;self.allFruits.midPoint{i}];
                        switch size
                            case 's'
                                % self.allFruits.dropPoint{i} = self.CreatePlane('Z',0.03,0.01,self.smallOrangeGoalPos);
                                self.allFruits.dropPoint{i} = self.PlotForeignObject('rectangle',self.smallOrangeGoalPos,'r',0.2,0.05,0.03,0);
                                self.allFruits.dropPoint{i} = [self.smallOrangeGoalPos;self.allFruits.dropPoint{i}];
                            case 'm'
                                self.allFruits.dropPoint{i} = self.CreatePlane('Z',0.03,0.01,self.mediumOrangeGoalPos);
                                self.allFruits.dropPoint{i} = [self.mediumOrangeGoalPos;self.allFruits.dropPoint{i}];
                            case 'l'
                                self.allFruits.dropPoint{i} = self.CreatePlane('Z',0.03,0.01,self.largeOrangeGoalPos);
                                self.allFruits.dropPoint{i} = [self.largeOrangeGoalPos;self.allFruits.dropPoint{i}];
                        end
                    case 'Purple'
                        self.allFruits.midPoint{i} = self.CreatePlane('Z',0.03,0.01,self.purpleGoalPos);
                        self.allFruits.midPoint{i} = [self.purpleGoalPos;self.allFruits.midPoint{i}];
                        switch size
                            case 's'
                                % self.allFruits.dropPoint{i} = self.CreatePlane('Z',0.03,0.01,self.smallPurpleGoalPos);
                                self.allFruits.dropPoint{i} = self.PlotForeignObject('rectangle',self.smallPurpleGoalPos,'r',0.2,0.05,0.03,0);
                                self.allFruits.dropPoint{i} = [self.smallPurpleGoalPos;self.allFruits.dropPoint{i}];
                            case 'm'
                                self.allFruits.dropPoint{i} = self.CreatePlane('Z',0.03,0.01,self.mediumPurpleGoalPos);
                                self.allFruits.dropPoint{i} = [self.mediumPurpleGoalPos;self.allFruits.dropPoint{i}];
                            case 'l'
                                self.allFruits.dropPoint{i} = self.CreatePlane('Z',0.03,0.01,self.largePurpleGoalPos);
                                self.allFruits.dropPoint{i} = [self.largePurpleGoalPos;self.allFruits.dropPoint{i}];
                        end
                end
                self.PlotPointCloud(self.allFruits.dropPoint{i},'r','.');
                
                % input("check")
                midPoints = self.allFruits.midPoint{i};
            end

            
            % make view nicer - found by manually adjust figure
            set(gcf, 'Windowstyle', 'docked');
            camlight();
            % optimalAzEl = [95.8730,17.8284];
            optimalAzEl = [-104,22];
            % optimalAzEl = [-112,14];
            view(optimalAzEl);
            zoom(1);
            axis([ -1, 2.5, -2, 2 ,0.25,2.25]);
        end
    
        %% Outputs start point cloud for specified fruit at a sample size
        function fruitPtCl = FruitPointCloud(self, index, sampleQty)
            fruitIndex = index;

            surfacePtCl = self.allFruits.pointCloud{fruitIndex};
            ptEnd = surfacePtCl(end,:);
            sizePtCl = length(surfacePtCl);
            fruitPtCl = surfacePtCl(1,:);
            % fruitRadius = self.allFruits.radius(fruitIndex);

            interval = int32(sizePtCl/sampleQty);                   % Making iterator interval positive int

            for i = 2:interval:sizePtCl
                fruitPtCl = [fruitPtCl;surfacePtCl(i,:)];
            end

            fruitStartTr = self.allFruits.startPoint{fruitIndex}.T;
            % pickFruitPt = fruitStartTr(1:3,4)';
            % pickFruitPt(3,4) = pickFruitPt(3,4)+fruitRadius;
            fruitPtCl = [fruitPtCl;fruitStartTr(1:3,4)'];
            newPtEnd = fruitPtCl(end,:);

            sizeFinalPtCl = length(fruitPtCl);
        end
        
        %% Outputs whether point/s reachable as well as the poses associated with end effector closest distance to goal
        function reachablePoses = CheckReachable(self,robotModel,points,animateOn)
            numPoints = size(points);
            reachable = 0;
            optimalQ = zeros(1,7);
            errorDistance = 1;              % Default to large quantity
            
            q0 = [-0.1 zeros(1,6)];        % Default reset join state - for animating.
            minDistPose = {errorDistance,optimalQ};

            for i = 1:numPoints(1)
                checkPt = points(i,:);
                pointTr = SE3(checkPt).T;
                if animateOn                            %Reset position of the robot from last attempt
                    warning off
                    robotModel.animate(q0);
                    drawnow();
                    % pause(0.001);
                    warning on
                end
                currentQ = robotModel.getpos();
                endPtTr = {pointTr*trotx(pi);pointTr*troty(pi)};
                maxAttempt = length(endPtTr);
                % input("check");
                
                attempt = 0;
                while errorDistance > self.maxGoalDistError && attempt < maxAttempt
                    attempt = attempt + 1;
                    attemptTr = endPtTr{attempt};

                    try
                        [Q1,ERR1] = robotModel.ikcon(attemptTr, currentQ);
                        [Q2,ERR2] = robotModel.qmincon(Q1);

                        Q1Tr =  robotModel.fkineUTS(Q1);
                        distQ1 = self.dist2pts(Q1Tr(1:3,4)',checkPt);
                        Q2Tr = robotModel.fkineUTS(Q2);
                        distQ2 = self.dist2pts(Q2Tr(1:3,4)',checkPt);

                        if distQ2 > distQ1                  %i.e. qmincon didn't help for lower error
                            finalQ = Q1;
                        else
                            finalQ = Q2;
                        end
                    catch
                        disp('Ikcon failed to converge - defined pose not reachable by the manipulator');
                        finalQ = Q1;
                        disp(Q1);
                        disp(distQ1);
                    end

                    if animateOn
                        warning off
                        robotModel.animate(q0);
                        drawnow();
                        % pause(0.001);
                        robotModel.animate(finalQ);
                        drawnow();
                        warning on
                    end
                    finalTr = robotModel.fkineUTS(finalQ);
                    finalPoint = finalTr(1:3,4)';
                    errorDistance = self.dist2pts(finalPoint, checkPt);

                    minDist = minDistPose{1,1};                                             %store previous values before comparing currently found distance
                    if errorDistance < minDist                                                  %To find lowest from the 3 attempts
                        % disp(['smaller dist ', num2str(dist), ' - storing new values']);
                        minDistPose{1,1} = errorDistance;
                        minDistPose{1,2} = finalQ;
                    end
                    % input("check attempt");
                end

                errorDistance = minDistPose{1,1};
                optimalQ = minDistPose{1,2};
                if errorDistance < self.maxGoalDistError;
                    reachable = true;
                    break;                                                      % Stop for-loop and move on - we got best pose
                end
            end
            optimalTr = robotModel.fkineUTS(optimalQ);
            reachablePoses = {reachable,optimalQ,errorDistance,optimalTr};
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
            input("Press enter to start.");
            status = true;
        end

        %% Checks if hardware or GUI e-stop is pressed and returns true or false
        function status = EStopPressed(self,press)                          % ADD FUNCTIONALITY 
            status = press;
            if status
                input("Stop?");
                disp("E-Stop Engaged! Stopping system operation.");
            else
                % disp("E-Stop Disengaged. System operation can continue ...");
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
        function [reached,errorDist] = CheckGoalComplete(self,robotModel,goalPos)
            currentEndEffectorTr = robotModel.fkineUTS(robotModel.getpos())
            errorDist = self.dist2pts(goalPos,currentEndEffectorTr(1:3,4)')
            if errorDist < self.maxGoalDistError
                reached = true;
            else
                reached = false;
                disp(['Error distance to ',robotModel.name, ' goal is ', num2str(errorDist)]);
            end
        end

        %% Calculates qMatrix of joint states for specified robot end effector postion and with specified interpolation method
        function finalQMatrix = CalcJointStates(self,robotModel, endPos, steps, method, mode)
            currentQ = robotModel.getpos();                                      % get the current joint states of robot in figure

            if strcmp(mode, 'basic')
                newQ = endPos;
            else
                endTr = SE3(endPos).T*trotx(pi);                                                            %rotating by pi so can pick up from top
                newPose = self.CheckReachable(robotModel,endTr(1:3,4)',false);        % calculate optimal goal joint state based on required endPos
                newQ = newPose{2};
                % robotModel.teach(newQ); % to check if it's actually near fruit
                % input("done checking?");
            end
            
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
            finalQMatrix = qMatrix
            % input('check join states calc')
        end

        %% Replots specified fruit to a new point by deleting current handle and plotting again
        function [plyPlotted, nextFruitPt] = ReplotFruit(self, index,transform)
            
            fruitIndex = index;
            nextFruitTr = transform;

            fruitHandle = self.allFruits.handle{fruitIndex};              %getting handle of fruit

             if ~isempty(fruitHandle) && ishandle(fruitHandle)
                nextFruitPt = nextFruitTr(1:3,4)';
                plyPlotted =  self.allFruits.plotFruitPly(fruitIndex,'moving',nextFruitTr);
                drawnow limitrate;           
                pause(0.001);
             else
                plyPlotted = false;
                disp(['No handle for fruit ', num2str(fruitIndex)]);
             end
        end
        
        %% Moves robot for both basic and normal mode - former requires specific task stage and fruit index while latter increments semi-autonomously
        function moved = MoveRobot(self,robotName,deltaQ,index,qMatrix,mode,stage)                  %ugly function ew need to make more straightforward
            fruitIndex = index;
            moved = false;
            nargin
            switch robotName
                case 'dobot'
                    if strcmp(mode,'advanced')                                                                         % Within major while loop - incremental movement while checking safety
                        newQMatrix = self.dobotQMatrix;
                        % fruitNum = self.dobotGoalsCompleted+1;
                        % currentFruit = allFruits.handle{fruitNum};
                        assert(deltaQ<=length(newQMatrix),'deltaQ chosen is larger than number of steps in trajectory');
                        self.dobotModel.animate(newQMatrix(1:deltaQ,:));
                        % drawnow();
                        drawnow limitrate;          %superspeed
                        % pause(0.001);
                        moved = true;

                        currentTr = self.dobotModel.fkineUTS(self.dobotModel.getpos());
                        % disp("transform: ");
                        % disp(currentTr);
                        switch stage
                            case 1                      %fruit moving too
                                % disp("entered stage 1");
                                fruitTr = currentTr;
                                % currentZ = fruitTr(3,4)
                                radius = self.allFruits.radius(fruitIndex);
                                fruitTr(3,4) = fruitTr(3,4) - radius/2;
                                % disp('modified Tr');
                                % fruitTr
                                [fruitPlotted,~] = self.ReplotFruit(fruitIndex,fruitTr);
                            otherwise
                                %nothing- - only move fruit when it's been picked up
                        end
                    else
                        newQMatrix = qMatrix;
                        if strcmp(mode,'basic')
                            for i =1:deltaQ:size(newQMatrix,1)
                                try
                                    self.dobotModel.animate(newQMatrix(i,:));
                                    drawnow limitrate;
                                    % disp("animated successfully");
                                    moved = true;
                                    currentTr = self.dobotModel.fkineUTS(self.dobotModel.getpos());
                                    % disp("transform: ");
                                    % disp(currentTr);
                                    switch stage
                                        case 1                      %fruit moving too
                                            % disp("entered stage 1");
                                            fruitTr = currentTr;
                                            % currentZ = fruitTr(3,4)
                                            radius = self.allFruits.radius(fruitIndex);
                                            fruitTr(3,4) = fruitTr(3,4) - radius/2;
                                            % disp('modified Tr');
                                            % fruitTr
                                            [fruitPlotted,~] = self.ReplotFruit(fruitIndex,fruitTr);
                                        otherwise
                                        %nothing- - only move fruit when it's been picked up
                                    end
                                    % pause(0.001);
                                    
                                    % disp("drawn new joint state:")
                                catch
                                    error("pjoint error - skipping joint state or check transform");
                                end
                            end
                        end
                    end

                case 'rebel'
                    if strcmp(mode,'advanced')                                                                               % Within major while loop - incremental movement while checking safety
                        newQMatrix = self.rebelQMatrix;
                        % fruitNum = self.rebelGoalsCompleted+1;
                        % currentFruit = allFruits.handle{fruitNum};
                        assert(deltaQ<=length(newQMatrix),'deltaQ chosen is larger than number of steps in trajectory');
                        self.rebelModel.animate(newQMatrix(1:deltaQ,:));
                        % drawnow();
                        drawnow limitrate;
                        % pause(0.001);
                        moved = true;

                        currentTr = self.rebelModel.fkineUTS(self.rebelModel.getpos());
                        % disp("transform: ");
                        % disp(currentTr);
                        switch stage
                            case 1                      %fruit moving too
                                % disp("entered stage 1");
                                fruitTr = currentTr;
                                % currentZ = fruitTr(3,4)
                                radius = self.allFruits.radius(fruitIndex);
                                fruitTr(3,4) = fruitTr(3,4) - radius/2;
                                % disp('modified Tr');
                                % fruitTr
                                [fruitPlotted,~] = self.ReplotFruit(fruitIndex,fruitTr);
                            otherwise
                                %nothing- - only move fruit when it's been picked up
                        end
                    else
                        fruitIndex = index;
                        newQMatrix = qMatrix;
                        if strcmp(mode,'basic')
                            for i = 1:deltaQ:size(newQMatrix,1)
                                try
                                    self.rebelModel.animate(newQMatrix(i,:));
                                    % drawnow();
                                    drawnow limitrate;
                                    % pause(0.001);
                                    disp("animated successfully");
                                    moved = true;
                                    currentTr = self.rebelModel.fkineUTS(self.rebelModel.getpos());
                                    switch stage
                                        case 1                      %fruit moving too
                                            % disp("entered stage 1");
                                            fruitTr = currentTr;
                                            % currentZ = fruitTr(3,4)
                                            radius = self.allFruits.radius(fruitIndex);
                                            fruitTr(3,4) = fruitTr(3,4) - radius/2;
                                            % disp('modified Tr');
                                            % fruitTr
                                            [fruitPlotted,~] = self.ReplotFruit(fruitIndex,fruitTr);
                                        otherwise
                                        %nothing - only move fruit when it's been picked up
                                    end                                    
                                catch
                                    error("pjoint error - skipping joint state");
                                end
                            end
                        end
                    end
                otherwise
                    error('Invalid robot chosen. Specify (dobot/rebel)');
            end
        end
   
        %% Sets up the specified safety tests for simulated sensor input (someone entering enclosure) and upcoming collision
        function status = SafetyTest(self, type,repeat)       
            status = true;                          % Default to true until human sensed or colliding with foreign object
            switch type
                case {'sensor', 'Sensor'}
                    
                    self.stopStatus = false;
                    self.systemStatus = true;

                    handle = findobj('Tag', 'human');
                    if ~isempty(handle)

                        % Boundaries of enclosure
                        minX = -1.46;
                        maxX = 1.61;
                        minY = -1.67;
                        maxY = 0.72;
                        
                        % Get coordinates of human
                        humanLocation = [mean(handle.XData(:)),mean(handle.YData(:)), mean(handle.ZData(:))];
                        humanX = humanLocation(1);
                        humanY = humanLocation(2);
                        
                        % Check if they are within enclosure
                        withinEnclosure = (humanX >= minX) && (humanX <= maxX) && ...
                                                     (humanY >= minY) && (humanY <= maxY);
                        if withinEnclosure
                            status = false;                      % Human is in enclosure - not safe
                            self.stopStatus = true;
                            self.systemStatus = false;
                        end
                    end

                case {'collision', 'Collision'}
                    shape = 'rectangle';
                    position = [0.25,-0.4,1.4];
                    sides = [0.1, 0.1,1];

                    foreignObjPts = self.PlotForeignObject(shape,position,'b',sides(1),sides(2),sides(3),1); % Plot foreign object within workspace
                    self.PlotPointCloud(foreignObjPts,'r','.');                         % Plot points for visability

                    % Add to environment point cloud to take into consideration for collision checking
                    self.environmentCl = [self.environmentCl;foreignObjPts];

                    if self.InCollision()                        % Placeholder function until collision checking completed
                        status = false;
                        self.stopStatus = true;
                        self.systemStatus = false;
                    end
                otherwise
                    error('Invalid test. Specify type of test: (sensor/collision)');
            end
        end

        %% Drops fruit into specified task stage buckets (mid,drop) visually in the figure 
        function dropped = DropFruit(self,fruitIndex,stage)
            dropped = false;
            waitTime = 0.001; %s
            g = 9.81;
            fruitRadius = self.allFruits.radius(fruitIndex);
            switch stage
                case 'mid'
                    bucketHeight = 0.10; % height stage 1 bucket
                    % fruitDropPoint = self.allFruits.midPoint{fruitIndex};
                    fruitDropTr = self.dobotModel.fkineUTS(self.dobotModel.getpos())
                    fruitDropTr(3,4) = fruitDropTr(3,4) - fruitRadius;
                    fruitDropPoint = fruitDropTr(1:3,4)'
                    buffer = 0.05; % for this stage, robot drop point 0.05m about bucket top (for collision avoidance)
                    
                    dropHeight = fruitDropPoint(1,3) - buffer - bucketHeight + fruitRadius;
                    finalDropPoint = [fruitDropPoint(1,1),fruitDropPoint(1,2),dropHeight];
                case 'drop'
                    bucketHeight = 0.05; % height of stage 2 buckets
                    fruitDropTr = self.rebelModel.fkineUTS(self.rebelModel.getpos())
                    fruitDropTr(3,4) = fruitDropTr(3,4) - fruitRadius;
                    fruitDropPoint = fruitDropTr(1:3,4)'
                    dropHeight = fruitDropPoint(1,3) - bucketHeight + fruitRadius;
                    finalDropPoint = [fruitDropPoint(1,1),fruitDropPoint(1,2),dropHeight];
                otherwise
                    disp("Invalid drop stage chosen. Fruit point not updated.");
                    return;
            end

            stopHeight = finalDropPoint(1,3);
            currentHeight = fruitDropPoint(1,3);
            loop = 1;
            while currentHeight > stopHeight
                thisDrop = loop * g * waitTime;                                 % not linear - will increase to drop faster/larger distance each loop
                currentHeight = currentHeight - thisDrop;
                loop = loop + 1;
                nextFruitTr = SE3([fruitDropPoint(1,1),fruitDropPoint(1,2),currentHeight]).T;
                % disp('Next transform:');
                % disp(nextFruitTr);
                [fruitPlotted,newFruitPt] = self.ReplotFruit(fruitIndex,nextFruitTr);
                handle = findobj('Tag', self.allFruits.tag{fruitIndex});
                currentFruitLocation = [mean(handle.XData(:)),mean(handle.YData(:)),mean(handle.ZData(:))];
                dist = self.dist2pts(finalDropPoint,currentFruitLocation);
                dropped = (dist <= self.maxGoalDistError);
            end
            
            handle = findobj('Tag', self.allFruits.tag{fruitIndex});
            currentFruitLocation = [mean(handle.XData(:)),mean(handle.YData(:)),mean(handle.ZData(:))];
            switch stage
                case 'mid'
                    self.allFruits.midPoint{fruitIndex} = currentFruitLocation;
                case 'drop'
                    self.allFruits.dropPoint{fruitIndex} = currentFruitLocation;
                otherwise
                    disp("Invalid drop stage chosen. Fruit point not updated.");
            end
        end
    
        %% Checks if any robot is in collision %%%%%%%%%%NEED TO UPDATE TRUE OR FALSE
        function collision = InCollision(self)                  
            collision = false;
        end
    end

    methods(Static)

        %% Saves current qMatrix values into file to be used later - may not be needed if code works nicely with constant qMatrix updates
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
        function points = CreatePlane(axis, maxDim, gridInterval, planePoint)
            range = -maxDim:gridInterval:maxDim;
        
            switch axis
                case {'X', 'x'}
                    % Plane is parallel to YZ, centered at planePoint along X
                    [Y, Z] = meshgrid(range, range);
                    X = ones(size(Y)) * planePoint(1);  
                    Y = Y + planePoint(2);              
                    Z = Z + planePoint(3);              
        
                case {'Y', 'y'}
                    % Plane is parallel to XZ, centered at planePoint along Y
                    [X, Z] = meshgrid(range, range);
                    Y = ones(size(X)) * planePoint(2);  
                    X = X + planePoint(1);              
                    Z = Z + planePoint(3);              
        
                case {'Z', 'z'}
                    % Plane is parallel to XY, centered at planePoint along Z
                    [X, Y] = meshgrid(range, range);
                    Z = ones(size(X)) * planePoint(3);  
                    X = X + planePoint(1);              
                    Y = Y + planePoint(2);              
        
                otherwise
                    error('Axis invalid - choose X, Y, or Z.');
            end
        
            points = [X(:), Y(:), Z(:)];
        
            plot3(points(:, 1), points(:, 2), points(:, 3), 'r.');

            % input("check");
        end

        %% Plots foreign object in current figure and obtains point cloud
        function points = PlotForeignObject(shape,position,colour,side1,side2,side3,plotOn)
            center = position;
            alpha = 0.7;

            switch shape
                case 'sphere'
                    radius = side1;
                    % Create the sphere
                    [X, Y, Z] = sphere(30); % 30 specifies the resolution of the sphere
                
                    % Scale and shift the sphere to the desired position and size
                    X = radius * X + center(1)
                    Y = radius * Y + center(2)
                    Z = radius * Z + center(3)
                
                case 'rectangle'
                    if nargin < 5
                        side2 = side1;              % only given 1 side dimension - square
                        side3 = side2;
                    elseif nargin < 6
                        side3 = side2;
                    end
                    
                    [X, Y, Z] = RectangularPrism2(center,side1,side2,side3,20);

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
            if plotOn
                surf(X, Y, Z, 'FaceAlpha', alpha, 'EdgeColor','none', 'FaceColor',colour, 'Tag', 'foreignObj');
            end
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