classdef LinearDobotMagician < RobotBaseClass
    %% DobotMagician
    % This class is based on the DobotMagician. 
    % URL: https://en.dobot.cn/products/education/magician.html
    % 
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access =public)   
        plyFileNameStem = 'LinearDobotMagician';

        %> defaultRealQ 
        defaultRealQ  = [-0.1,  0,  pi/4,  pi/8,  0,  0];
    end

    methods (Access = public) 
%% Constructor 
function self = LinearDobotMagician(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);
            end
            self.model.base = self.model.base.T*baseTr* trotx(pi/2)* troty(pi/2);
            self.homeQ = self.RealQToModelQ(self.defaultRealQ);
            self.PlotAndColourRobot();
        end

%% CreateModel
        function CreateModel(self)       
            link(1) = Link([pi   0       0       pi/2    1]); % PRISMATIC Link
            link(2) = Link('d',0.103+0.0362,    'a',0,      'alpha',-pi/2,  'offset',0, 'qlim',[deg2rad(-135),deg2rad(135)]);
            link(3) = Link('d',0,        'a',0.135,  'alpha',0,      'offset',-pi/2, 'qlim',[deg2rad(5),deg2rad(80)]);
            link(4) = Link('d',0,        'a',0.147,  'alpha',0,      'offset',0, 'qlim',[deg2rad(-5),deg2rad(85)]);
            link(5) = Link('d',0,        'a',0.06,      'alpha',pi/2,  'offset',-pi/2, 'qlim',[deg2rad(-180),deg2rad(180)]);
            link(6) = Link('d',-0.05,      'a',0,      'alpha',0,      'offset',pi, 'qlim',[deg2rad(-85),deg2rad(85)]);

            link(1).qlim = [-0.8 -0.01];

            self.model = SerialLink(link,'name',self.name);
        end   
    end
    
    methods(Hidden)
%% TestMoveJoints
        % Overriding the RobotBaseClass function, since it won't work properly
        % for this robot
        function TestMoveJoints(self)
            self.TestMoveDobotMagician();
        end

%% Test Move DobotMagician
    function TestMoveDobotMagician(self)
            qPath = jtraj(self.model.qlim(:,1)',self.model.qlim(:,2)',50);                       
            for i = 1:50                
                self.model.animate(self.RealQToModelQ(qPath(i,:)));
%                 hold on;
%                 trplot(self.model.fkine(self.RealQToModelQ(qPath(i,:))));
                pause(0.2);
            end
        end
    end

    methods(Static)
%% RealQToModelQ
        % Convert the real Q to the model Q
        function modelQ = RealQToModelQ(realQ)
            modelQ = realQ;
            modelQ(4) = LinearDobotMagician.ComputeModelQ4GivenRealQ3and4( realQ(3), realQ(4) );
            modelQ(5) = pi - realQ(3) - modelQ(4);    
        end
        
%% ModelQ4GivenRealQ3and4
        % Convert the real Q3 & Q4 into the model Q3
        function modelQ4 = ComputeModelQ4GivenRealQ3and4(realQ3,realQ4)
            modelQ4 = pi/2 - realQ3 + realQ4;
        end
        
%% ModelQToRealQ
        % Convert the model Q to the real Q
        function realQ = ModelQToRealQ( modelQ )
            realQ = modelQ;
            realQ(4) = LinearDobotMagician.ComputeRealQ4GivenModelQ3and4( modelQ(3), modelQ(4) );
        end
        
%% RealQ3GivenModelQ2and3
        % Convert the model Q2 & Q3 into the real Q3
        function realQ4 = ComputeRealQ4GivenModelQ3and4( modelQ3, modelQ4 )
            realQ4 = modelQ4 - pi/2 + modelQ3;
        end
    end
end