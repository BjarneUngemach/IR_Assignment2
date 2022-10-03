classdef SwipeBot < UR3 & Table & Calculations
    
    properties
        base = eye(4);
        workspace = [-1 1 -1 1 0 1]
        
% for coding purposes only
gripper3FOffset = 0.1;
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
            self.table.base = self.base;                            %move the table to desired pose
            self.table.animate(0);                                  %plot table
            self.ur3.base = self.base * transl(0.1124, 0.1124, 0.42);       %put the UR3 on top of the table
            q = self.ur3.getpos;                                    %get current joint configuration
            self.ur3.animate(q);                                    %plot the same joint configuration at new pose
%             self.pf3400.base = self.base * transl(-0.2, -0.3, 0.07);%put the PF3400 in the bottom shelf
%             q = self.pf3400.getpos;                                 %get current joint configuration
%             self.pf3400.animate(q);                                 
        end
    end
    
    methods
    %% %%%%% Methods to simulate the Swipebot %%%%% %%
    % refer to Calculations.m to find the big calculation programms
    
        %% function homes all robots
        function MoveTo(self, choice)
            %%% UR3 %%%
            qUR3 = self.ur3.getpos;
            qUR3Home = [-1.5708 -2.0429 -2.4267 -0.2428 1.5708 0];
            qUR3Store = deg2rad([-90 -200 -70 -90 180 0]);
            qUR3PreStore = deg2rad([-90 -135 -110 -115 180 0]);
            % home position
            if choice == "home"
                if 0.0001 > self.ur3.getpos - qUR3Store
                    qMatrixUR3 = jtraj(qUR3Store,qUR3PreStore,50);
                    qMatrixUR3 = [qMatrixUR3; jtraj(qUR3PreStore,qUR3Home,50)];
                else
                    qMatrixUR3 = jtraj(qUR3,qUR3Home,50);
                end
            end
            if choice == "store"
                if 0.0001 < self.ur3.getpos - qUR3Home
                    self.MoveTo("home");
                end
                qMatrixUR3 = jtraj(qUR3,qUR3PreStore,50);
                qMatrixUR3 = [qMatrixUR3; jtraj(qUR3PreStore,qUR3Store,50)];
            end
            
            for step = 1:size(qMatrixUR3,1)
                    self.ur3.animate(qMatrixUR3(step,:));
                    pause(0.1);
            end
        end
        
        %% get a joint state Matrix with a Resolved Motion Rate Control
        function [qMatrix,diff,m,Jacob] = SolveUR3RMRC(self, trajectory, offset, stepsize, speed)
            %trajectory = pagemtimes(trajectory, transl(0,0,-offset));   %offset every trajectory transform by offset value
            
            deltaT = stepsize/speed;
            steps = size(trajectory,3);
            
%             for i=1:steps
%                 trajectory(:,:,i)=trajectory(:,:,i)*transl(0,0,-offset);
%             end
            
            qGuess = deg2rad([0,-70,-120,10,90,0]);
            
            qMatrix = nan(steps,6);
            qMatrix(1,:) = self.ur3.ikine(trajectory(:,:,1),qGuess);
trdiff(:,:,1) = trajectory(:,:,1) - self.ur3.fkine(qMatrix(1,:));
diff(1,1) = sqrt(sum((trdiff(1:3,4,1)).^2));
            
            for i = 1:steps-1
                posdot(:,:,i) = (trajectory(1:3,4,i+1) - trajectory(1:3,4,i))/deltaT;           % Calculate linear velocities (x, y, z) between steps
                skewOmega = (trajectory(1:3,1:3,i+1)*trajectory(1:3,1:3,i)' - eye(3))/deltaT;   % Calculate skew-symmetric matrix of omega to get angular velocities
                rotdot = [skewOmega(3,2);skewOmega(1,3);skewOmega(2,1)];                        % Extract angular velocities (roll, pitch, yaw) between steps
                J = self.ur3.jacob0(qMatrix(i,:));                                              % Get the Jacobian at the current state
Jacob(:,:,i)=J;
                qdot = inv(J)*[posdot(:,:,i);rotdot];                                           % Solve velocitities via RMRC
                qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';                                  % Update next joint state
                m(:,i)= sqrt(det(J*J'));
trdiff(:,:,i+1) = trajectory(:,:,i+1) - self.ur3.fkine(qMatrix(i+1,:));
diff(i+1,1) = sqrt(sum((trdiff(1:3,4,i+1)).^2));
            end
        end
    end
end