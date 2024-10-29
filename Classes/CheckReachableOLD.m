%% Finds whether a point in matrix of desired positions for endEffector is reachable from start pose
        %% Outputs whether reachable as well as the poses associated with end effector closest distance to goal
        function verdict = CheckReachable(self,robotModel,stage,animateOn)
            reachable = ones(1,self.numFruits);
            poses = {zeros(1,6)};
            for i=1:self.numFruits
                dist = 1;
                q0 = [-0.1 zeros(1,6)];
                M = [1 1 zeros(1,4)];
                attempt = 0;
                minDistPose = {dist,q0};

                switch stage
                    case 'start'
                        % fruitTr = self.allFruits.startPoint{i}.T
                        fruitPtCl = self.allFruits.pointCloud{i};
                        fruitPtCl = fruitPtCl(:,1:3);
                        ptEnd = fruitPtCl(end,:);
                        Tr = self.allFruits.startPoint{i}.T;
                        interval = 100;
                        fruitPtCl = [fruitPtCl;Tr(1:3,4)'];
                        newPtEnd = fruitPtCl(end,:);
                        sizePC = length(fruitPtCl(:,1));
                    case 'mid'
                        Tr = SE3(self.allFruits.midPoint{i}).T;
                        fruitPtCl = Tr(1:3,4)';
                        interval = 1;
                        sizePC = 1;
                    case 'drop'
                        Tr = SE3(self.allFruits.dropPoint{i}).T;
                        fruitPtCl = Tr(1:3,4)';
                        interval = 1;
                        sizePC = 1;
                    otherwise
                        error("Specify fruit stage: (start,mid,drop)");
                end
                disp(['Stage: ', stage])
                input("check");
                
                for j = 1:interval:sizePC
                    fruitTr = SE3(fruitPtCl(j,:)).T;
                    currentQ = robotModel.getpos();                                      % get the current joint states of robot in figure
                    % endTr = SE3(fruitPos).T*trotx(pi) %rotating by pi so can pick up from top
                    
                    if strcmp(stage,'start')
                        endTr = {fruitTr*trotx(pi);fruitTr*troty(pi);fruitTr*trotz(pi)};
                    else
                        endTr = {fruitTr*trotx(pi)};
                    end

                    % input("check");
                    for k = 1:length(endTr)
                        attempt = 0;
                        while dist > self.maxGoalDistError
                            attempt = attempt +1;
                            if attempt > 1
                                % disp('Attempted 1 times - aborting.');
                                reachable(i) = false;
                                break;
                            end

                            try
                                Q1  = robotModel.ikcon(endTr{k}, currentQ);
                                Q2 = robotModel.qmincon(Q1);
                                pause(0.001);
                                if animateOn
                                    robotModel.animate(Q2);
                                end
                                Q = Q2;
                                try
                                    warning off
                                    Q = robotModel.ikine(endTr{k},'q0', Q2, 'mask', M);                    % Solve for joint angles
                                    warning on
                                catch
                                    disp("Disregard Ikine solution");
                                    Q = Q2;
                                end
                                Q;
                            catch
                                disp('Ikcon failed to converge - defined pose not reachable by the manipulator')
                                startPose = false;
                                % Q
                                % ERR
                                % EXITFLAG
                            end

                            try
                                if animateOn
                                    warning off
                                    robotModel.animate(Q);
                                    warning on
                                end
                            catch
                                disp("ikine Q didn't work - sticking with Q2");
                                % error("ikine Q didn't work - sticking with Q2");
                                if animateOn
                                    warning off
                                    robotModel.animate(Q2);
                                    warning on
                                end
                            end

                            if animateOn
                                finalQ = robotModel.getpos();
                            else
                                finalQ = Q;
                            end
                            finalTr = robotModel.fkineUTS(finalQ);
                            % dist = self.dist2pts(finalTr(1:3,4)', self.allFruits.startPoint{i})
                            finalPoint = finalTr(1:3,4)';
                            
                            fruitPoint = fruitTr(1:3,4)';
                            dist = self.dist2pts(finalPoint, fruitPoint);

                            %% RMRC attempt
                            % input("now try RMRC");
                        % M = [1 1 zeros(1,4)];                                                       % Masking Matrix
                        % deltaT = 0.05;                                                              % Discrete time step
                        % 
                        % minManipMeasure = 0.1;
                        % steps = 100;
                        % deltaTheta = 2*pi/steps;
                        % x = [];
                        % 
                        % T = eye(4);
                        % 
                        % m = zeros(1,self.steps);
                        % error = nan(2,self.steps);
                        % for i = 1:self.steps-1
                        %     xdot = (x(:,i+1) - x(:,i))/deltaT;                                      % Calculate velocity at discrete time step
                        %     J = p2.jacob0(qMatrix(i,:));                                            % Get the Jacobian at the current state
                        %     J = J(1:2,:);                                                                   % Take only first 2 rows
                        %     m(:,i)= sqrt(det(J*J'));                                                % Measure of Manipulability
                        %     if m(:,i) < minManipMeasure
                        %         qdot = inv(J'*J + 0.01*eye(2))*J'*xdot;
                        %     else
                        %         qdot = inv(J) * xdot;                                               % Solve velocitities via RMRC
                        %     end
                        %     error(:,i) = xdot - J*qdot;
                        %     qMatrix(i+1,:) = qMatrix(i,:) + deltaT * qdot';                         % Update next joint state
                        % end
    
                        end
                        minDist = minDistPose{1,1};
                        minDistQ = minDistPose{1,2};
                        if dist < minDist
                            % disp(['smaller dist ', num2str(dist), ' - storing new values']);
                            minDistPose{1,1} = dist;
                            minDistPose{1,2} = finalQ;
                            finalQ;
                            % minDistPose
                        end
                        % input("check");
                    end

                    if minDistPose{1,1} < self.maxGoalDistError
                        reachable(i) = true;
                        poses(:,i) = {minDistPose{1,2}};
                        % disp(['reachable with endTr']);
                        % k
                        break;
                    end
                end

                minDistQ = minDistPose{1,2};
                minDist = minDistPose{1,1};
                if animateOn
                    robotModel.animate(q0);
                    input("ready to see minDist pose?");
                    robotModel.animate(minDistQ);
                    input("next fruit?");
                    % input("check");
                end
            end
            reachable;
            verdict = {reachable,minDistQ};
        end