%% Load floor
floor = Environment("floor","floor");
% wallRear = Environment("wall","rear");
% wallCollectionSide = Environment("wall","collection");
% wallShootSide = Environment("wall","shoot");
% roof = Environment("floor","ceiling");
%% Load the worktable
table = Table();

%% Stage 1 - bucket with all fruits
buckets = Buckets();

%% Buckets for collection, sorted by size and colour
sortedBuckets = SortedBuckets();

%% Display enclosure
walls = Enclosure();
door = Door();
shoot = Shoot();

%% Display e-stops and sensors
eStopDoor = EStopObject("doorstop");
eStopRobot1 = EStopObject("robot1");
eStopRebel = EStopObject("robot2");
eStopCollection = EStopObject("collection");

sensor = DoorSensor();
camera = CameraObject();

%% Fruit plotting
% fruit = Fruit("manual",9);

%% Load robots
set(gcf, 'Windowstyle', 'docked');
hold on;
scale = 0.5;
robot1Base = SE3(0*scale,0*scale,0.8*scale).T;
r = DobotMagician(robot1Base);
% r = LinearUR3e(robot1Base);
r.model.teach(zeros(1,r.model.n));