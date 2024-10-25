%% Load floor
floor = Environment("floor","floor");
set(gcf, 'Windowstyle', 'docked');
camlight;
% wallRear = Environment("wall","rear");
% wallCollectionSide = Environment("wall","collection");
% wallShootSide = Environment("wall","shoot");
% roof = Environment("floor","ceiling");
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

%% Get Point Cloud of all environment and plot over
environmentCl = [tableCl; bucketsCl; sortedBucketsCl; shootCl];

plot3(environmentCl(:,1),environmentCl(:,2),environmentCl(:,3),'r.');
% fig_h = gcf;
% view(3);
% rotate3d(gcf);

% input("next");
%% Fruit plotting
fruit = Fruit("manual",9);

% %% Load robots
% 
% hold on;
% 
% 
% %% PointClouds
% testpoints = PlotPlane('Y',4,0.5,-3);
% 
% plot_h = PlotPointCloud(testpoints,'r','.');
% axis equal;
% hold on;
% % spheretest
% qty = 9;
% % fruit = Fruit("manual",qty);
% surfacePts = cell(1,qty);
% allPoints =  [ ];
%             for i=1:qty
%                 radius = 0.4;
%                 center = [1,1,5];
% 
%                 % Create the sphere
%                 [X, Y, Z] = sphere(30); % 30 specifies the resolution of the sphere
% 
%                 % Scale and shift the sphere to the desired position and size
%                 X = radius * X + center(1);
%                 Y = radius * Y + center(2);
%                 Z = radius * Z + center(3);
% 
%                 % Plot the sphere
%                 % disp(["i = "+i]);
%                 spherePts = [X(:),Y(:),Z(:)];
%                 % surfacePts = [surfacePts;spherePts]
%                 surfacePts{1,i} = [surfacePts{1,i};spherePts];
%                 allPoints = [allPoints;surfacePts{1,i}];
%             end
% 
%             plot3(allPoints(:,1),allPoints(:,2),allPoints(:,3), 'r.');
%             axis equal;




% scale = 0.5;
% robot1Base = SE3(0*scale,0*scale,0.8*scale).T;
% r = DobotMagician(robot1Base);
% % r = LinearUR3e(robot1Base);
% r.model.teach(zeros(1,r.model.n));

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

function pointCloud_h = PlotPointCloud(points,colour,shape) 
    X = points(:,1);
    Y = points(:,2);
    Z = points(:,3);

    pointCloud_h = plot3(X,Y,Z,strcat(colour,shape));
end