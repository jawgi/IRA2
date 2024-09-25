classdef LinearUR3e < ModifiedRobotBaseClass
    %% UR3e Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)
        plyFileNameStem = 'LinearUR3e';
    end

    methods
        %% Constructor
        function self = LinearUR3e(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);
                end
            else % All passed in
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end

            self.CreateModel();
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
            drawnow
        end

        %% CreateModel
        function CreateModel(self)
            % link(3) = Link('d',0.15185,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            % link(4) = Link('d',0,'a',-0.24355,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            % link(5) = Link('d',0,'a',-0.2132,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            % link(6) = Link('d',0.13105,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            % link(7) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            % link(8) = Link('d',	0.0921,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
            link(1) = Link('theta', 0, 'a', 0, 'alpha', -pi/2,'qlim',[-2 2]); % PRISMATIC Link
            link(2) = Link('d', 0.08, 'a', 0, 'alpha', 0,'qlim',deg2rad([-360,360]));
            link(3) = Link('d', 0.1519, 'a', 0, 'alpha', pi/2,'qlim',deg2rad([-360,360]));
            link(4) = Link('d', 0, 'a', -0.24365, 'alpha', 0,'qlim',deg2rad([-360,360]));
            link(5) = Link('d', 0, 'a', -0.21325, 'alpha', 0,'qlim',deg2rad([-360,360]));
            link(6) = Link('d', 0.11235, 'a', 0, 'alpha', pi/2,'qlim',deg2rad([-360,360]));
            link(7) = Link('d', 0.08535, 'a', 0, 'alpha', -pi/2,'qlim',deg2rad([-360,360]));
            link(8) = Link('d', 0.0819, 'a', 0, 'alpha', 0,'qlim',deg2rad([-360,360]));
            self.model = SerialLink(link,'name',self.name);

        end
    end
    methods (Static)

        %% Plot reach
        function plotReach(ur3eInstance,tableHeight)
            persistent currentReachRadius maxReachRadius;
            if ~isa(ur3eInstance, 'LinearUR3e')
                error('Input must be a UR3e object.');
            end
            if ~isempty(currentReachRadius) && isvalid(currentReachRadius)
                delete(currentReachRadius);
            end

            if ~isempty(maxReachRadius) && isvalid(maxReachRadius)
                delete(maxReachRadius);
            end
            jointAngles = ur3eInstance.model.getpos();
            jointAngles = [jointAngles(1) 0 0 0 0 0 0 0];
            joint2 = ur3eInstance.model.A(2,jointAngles);
            origin2 = joint2.t;
            joint9 = ur3eInstance.model.fkine(jointAngles);
            origin9 = joint9.t;
            reach(1) = sqrt((origin9(1) - origin2(1))^2 + (origin9(2)-origin2(2))^2);
            reach(2) = sqrt((origin9(1) - origin2(1))^2 + (origin9(2)-origin2(2))^2 + (origin9(3)-origin2(3))^2)/2;

            theta = linspace(0, 2*pi, 100);
            x = reach(1) * cos(theta)+origin2(1);
            xMax = reach(2) * cos(theta)+origin2(1);
            y = reach(1) * sin(theta)+origin2(2);
            yMax = reach(2) * sin(theta)+origin2(2);

            hold on;
            currentReachRadius = fill3(x,y,(tableHeight+0.001)*ones(size(x)),[0 0 1]);
            maxReachRadius = fill3(xMax,yMax,tableHeight*ones(size(x)),'g');
            hold off;
        end

        function animateAndPlot(q,ur3eInstance,tableHeight)
            if ~isa(ur3eInstance, 'LinearUR3e')
                error('Input must be a UR3e object.');
            end
            ur3eInstance.plotReach(ur3eInstance,tableHeight);
            ur3eInstance.model.animate(q);
        end

        function testJoints(ur3eInstance)
            ur3eInstance.TestMoveJoints();
        end
    end
end
