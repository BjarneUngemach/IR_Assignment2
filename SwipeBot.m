classdef SwipeBot < UR3 & MECA500 & Table & Calculations
    
    properties
        base = eye(4);
        workspace = [-1 1 -1 1 0 1]
    end
    
    methods
    %% %%%%% Methods to generate the Swipebot %%%%% %%
    
        %% Swipebot object structor
        function self = SwipeBot()
            % because this function inherits the models from the other
            % classes (mentioned above), no "getSwipebot" function is
            % needed
            
            self.UpdatePosition();  %update the position of the new generated elements
            
        end
        
        %% Update all elements to the desired pose of the SwipeBot
        function UpdatePosition(self)
            self.table.base = self.base;                                %move the table to desired pose
            self.table.animate(0);                                      %plot table
            self.ur3.base = self.base * transl(0.1124, 0.1124, 0.42);   %put the UR3 on top of the table
            q = self.ur3.getpos;                                        %get current joint configuration
            self.ur3.animate(q);                                        %plot the same joint configuration at new pose
            self.meca500.base = self.base * transl(-0.1, 0, 0.05);      %put the Meca500 in the bottom shelf
            q = self.meca500.getpos;                                    %get current joint configuration
            self.meca500.animate(q);                                    %plot the same joint configuration at new pose
        end
    end
    
    methods
    %% %%%%% Methods to simulate the Swipebot %%%%% %%
    % refer to Calculations.m to find the big calculation programms
    
        %% function homes all robots
        function MoveTo(self, choice)
            % predefine jointstates
            % UR3
            qUR3 = self.ur3.getpos;
            qUR3Home = [-1.5708 -2.0429 -2.4267 -0.2428 1.5708 0];
            qUR3Store = deg2rad([-90 -200 -70 -90 180 0]);
            qUR3PreStore = deg2rad([-90 -135 -110 -115 180 0]);
            
            % move to home position
            if choice == "home"
                if 0.0001 > self.ur3.getpos - qUR3Store                         % if robot is in "store" position...
                    qMatrixUR3 = jtraj(qUR3Store,qUR3PreStore,50);              % move robot to waypoint
                    qMatrixUR3 = [qMatrixUR3; jtraj(qUR3PreStore,qUR3Home,50)]; % then move to "home" position
                else
                    qMatrixUR3 = jtraj(qUR3,qUR3Home,50);                       % else move directly to home position
                end
            end
            if choice == "store"
                if 0.0001 < self.ur3.getpos - qUR3Home                          % if robot is NOT at home position...
                    self.MoveTo("home");                                        % move robot there first
                end
                qMatrixUR3 = jtraj(qUR3,qUR3PreStore,50);                       % move robot to waypoint
                qMatrixUR3 = [qMatrixUR3; jtraj(qUR3PreStore,qUR3Store,50)];    % then to "store" position
            end
            
            for step = 1:size(qMatrixUR3,1)                                     % animate robot movement
                    self.ur3.animate(qMatrixUR3(step,:));
                    if self.CheckCollision("table", self.ur3) > 0
                        disp("Damn, I hit something! Press enter to continue...")
%                         pause
                    end
                    pause(0.01);
            end
        end
        
        %% get a joint state Matrix with a Resolved Motion Rate Control
        function [qMatrix,error] = SolveRMRC(~, robot, trajectory, stepsize, speed)
            %%% CONSTANTS %%%
            WEIGHT_MATRIX = diag([1 1 1 0.1 0.1 0.1]);   % Matrix of gains for RMRC
            THRESHOLD = 0.001;                           % threshold for DLS use
            MAX_DAMPING = 0.01;                          % max damping factor of DLS
            
            
            deltaT = stepsize/speed;                    % get timestep for wanted velocity
            steps = size(trajectory,3);                 % get number of steps in trajectory
            
            % allocate arrays
            qMatrix = nan(steps,6);                     % get empty matrix
            error = nan(steps,1);                       % get empty position error list
            
            qGuess = deg2rad([0,-70,-120,10,90,0]);
            
            
            qMatrix(1,:) = robot.ikine(trajectory(:,:,1),qGuess);                               % solve first qMatrix with inverse kinematics
            actualTransform = robot.fkine(qMatrix(1,:));                                        % get actual pose of endeffector
            trdiff = trajectory(:,:,1) - robot.fkine(qMatrix(1,:));                             % calculate error between poses
            error(1) = sqrt(sum((trdiff(1:3,4)).^2));                                          
            
            for i = 1:steps-1
                posdot = (trajectory(1:3,4,i+1) - actualTransform(1:3,4))/deltaT;               % Calculate linear velocities (x, y, z) to next step
                skewOmega = (trajectory(1:3,1:3,i+1)*actualTransform(1:3,1:3)' - eye(3))/deltaT;% Calculate skew-symmetric matrix of omega to get angular velocities
                rotdot = [skewOmega(3,2);skewOmega(1,3);skewOmega(2,1)];                        % Extract angular velocities (roll, pitch, yaw) between steps
                xdot = WEIGHT_MATRIX * [posdot;rotdot];                                         % concatinate linear and angular velocity and implement a gain factor
                
                J = robot.jacob0(qMatrix(i,:));                                                 % Get the Jacobian at the current state
                manipulability = sqrt(det(J*J'));                                               % calculate manipulability
                if manipulability < THRESHOLD                                                   % If manipulability is less than given threshold
                    dampingFactor = (1 - (manipulability/THRESHOLD)^2) * MAX_DAMPING;           % determine damping factor of DLS
                    qdot = ((J'*J + dampingFactor*eye(6))\J')*xdot;                             % Get joint velocities with Damped Least Squares Optimization
                    disp("DLS used");
                else
                    qdot = J\xdot;                                                              % Solve for joint velocitities via RMRC
                end
                
                for j = 1:6                                                                     % Loop through joints 1 to 6
                    if qMatrix(i+1,j) + deltaT*qdot(j) < robot.qlim(j,1)                        % If next joint angle is lower than joint limit...
                        qdot(j) = 0;                                                            % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(j) > robot.qlim(j,2)                      % If next joint angle is greater than joint limit ...
                        qdot(j) = 0;                                                            % Stop the motor
                    end
                end
                qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';                                  % Get next joint state
                
                actualTransform = robot.fkine(qMatrix(i+1,:));                                  % get actual pose of endeffector
                trdiff = trajectory(:,:,i+1) - actualTransform;                                 % calculate error between poses
                error(i+1) = sqrt(sum((trdiff(1:3,4)).^2));
            end
        end
    end
end