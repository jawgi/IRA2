%% A2
r = LinearUR3e;
robot = r.model;
linkDiameter = 0.1; %obtained from UR3e spec sheet base flange diameter

q = zeros(1,7);
robot.plot3d(q, 'workspace', [-1 1, -1 1, 0.01 1]);
hold on;

tr = zeros(4,4,robot.n+1);
tr(:,:,1) = robot.base;
L = robot.links;
for i = 1 : robot.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

for i = 1:robot.n-1
    radii = [L(i).d ,linkDiameter,linkDiameter];
    currentTr = tr(:,:,i);
    nextTr = tr(:,:,i+1);

    centerPoint = (currentTr(1:3,4) + nextTr(1:3,4)).'/2;                      % Finding midpoint between transform of j-1 link and j link
    [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
    if i ~= 1
        robot.points{i} = [X(:),Y(:),Z(:)];
        % warning off
        % robot.faces{i} = delaunay(robot.points{i});    
        % warning on;
        
    end
    points = robot.points{i};
    plot3(points(:,1), points(:,2), points(:,3),'r.');
    hold on;
end

axis equal
camlight
%% test
r = UR3e;
robot = r.model;

% centerPoint = [0,0,0];
% radii = [0.5,0.1,0.1];
linkDiameter = 0.07;
L = robot.links;

for i = 1:robot.n
    centerPoint = [0,0,0];
    linkLength = abs(L(i).d)
    if L(i).d == 0
        % linkLength = 0.0315;
        linkLength = abs(L(i).a)
        % points = PlotForeignObject('sphere',centerPoint,'r',linkLength);
        radii = [linkLength,linkDiameter,linkDiameter] % a value so linkLength is in X axis
        % centerPoint(1) = centerPoint(1)+L(i).a;
        % centerPoint
    else
        radii = [linkDiameter,linkDiameter,linkLength] % d value so linkLength is in Z axis
    end
    [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
    robot.points{i} = [X(:),Y(:),Z(:)];
    warning off
    robot.faces{i} = delaunay(robot.points{i});    
    warning on;
end

q = zeros(1,6);
% q(1) = -0.25;
robot.plot3d(q, 'workspace', [-1 1, -1 1, 0.01 1]);
axis equal
camlight
robot.teach(q);

%% A2 + test
r = UR3e;
robot = r.model;
linkDiameter = 0.08; %obtained from UR3e spec sheet base flange diameter

q = zeros(1,6);
% q(1) = -0.1;
robot.plot3d(q, 'workspace', [-1 1, -1 1, 0.01 1]);
hold on;

tr = zeros(4,4,robot.n+1);
tr(:,:,1) = robot.base
L = robot.links;
for i = 1 : robot.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha)
end

for i = 1:robot.n
    currentTr = tr(:,:,i)
    nextTr = tr(:,:,i+1)
    centerPoint = (currentTr(1:3,4) + nextTr(1:3,4)).'/2                      % Finding midpoint between transform of j-1 link and j link
    % linkLength = abs(L(i).d + L(i).a)
    linkLength = dist2pts(currentTr(1:3,4)', nextTr(1:3,4)')
    input('check');

    if L(i).d == 0
        % linkLength = 0.0315;
        % linkLength = abs(L(i).a)
        % points = PlotForeignObject('sphere',centerPoint,'r',linkLength);
        radii = [linkLength,linkDiameter,linkDiameter] % a value so linkLength is in X axis
    else
        radii = [linkDiameter,linkDiameter,linkLength] % d value so linkLength is in Z axis
        [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
    end
    
    % if i ~= 1
        robot.points{i} = [X(:),Y(:),Z(:)];
        warning off
        robot.faces{i} = delaunay(robot.points{i});    
        warning on
    % end
end
points = robot.points{i};
plot3(points(:,1), points(:,2), points(:,3),'r.');
hold on;
axis equal
camlight

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
            % surf(X, Y, Z, 'FaceAlpha', alpha, 'EdgeColor','none', 'FaceColor',colour);
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