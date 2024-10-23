classdef A2Scaffold_psuedo < handle
    properties (Constant)
        dobotFruitPos = [0.6,0,0.0;        0.6,0.17,0;         0.6,-0.17,0; ...
                              0.6,0,0.03;       0.6,0.17,0.03;      0.6,-0.17,0.03; ...
                              0.6,0,0.06;       0.6,0.17,0.06;      0.6,-0.17,0.06;];

        dobotFruitGoal = [0.6,0,0.0;        0.6,0.17,0;         0.6,-0.17,0; ...
                              0.6,0,0.03;       0.6,0.17,0.03;      0.6,-0.17,0.03; ...
                              0.6,0,0.06;       0.6,0.17,0.06;      0.6,-0.17,0.06;];

        rebelFruitPos = [0.6,0,0.0;        0.6,0.17,0;         0.6,-0.17,0; ...
                              0.6,0,0.03;       0.6,0.17,0.03;      0.6,-0.17,0.03; ...
                              0.6,0,0.06;       0.6,0.17,0.06;      0.6,-0.17,0.06;];

        rebelFruitGoal = [0.6,0,0.0;        0.6,0.17,0;         0.6,-0.17,0; ...
                              0.6,0,0.03;       0.6,0.17,0.03;      0.6,-0.17,0.03; ...
                              0.6,0,0.06;       0.6,0.17,0.06;      0.6,-0.17,0.06;];
    end

    properties
        dobotModel;
        dobotBase;
        rebelModel;
        rebelBase;

        % handles
        mainFig_h;
    end
    
    methods
        function self = A2Scaffold_psuedo() %figuring out general flow of code
            tic;
            self.SimEnv();

            self.dobotBase = SE3(0.4,-0.6,0.8).T;
            dobot = LinearDobotMagician(self.dobotBase);
            self.dobotModel = dobot.model;
            self.dobotModel.teach(self.dobotModel.getpos());
            hold on;
            input("checked reach of dobot?")

            self.rebelBase = SE3(0.4,0.4,0.8).T;
            rebel = LinearUR3e(self.rebelBase);
            self.rebelModel = rebel.model;
            self.rebelModel.teach(self.rebelModel.getpos());
            hold on;
            input("checked reach of rebel?");

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
        end

        function SimEnv(self)            
            % Running workspace testing copy
            run("shahd.m");
            
            % Populate additional safety features?
            self.PlaceSafety();
            
            % make view nicer
            camlight();
            view(0,90);
            zoom(5);

            % self.mainFig_h = gcf;
            % input("check robots");
            % pan(fig,)

        end
    end

    methods(Static)

        function PlaceSafety()
            
            
        end
    end
end