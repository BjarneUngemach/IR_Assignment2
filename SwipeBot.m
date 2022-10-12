classdef SwipeBot < UR3 & MECA500 & Table & Calculations
    
    properties
        base = eye(4);
        workspace = [-0.5 0.5 -0.5 0.5 0 1]
        toolChangeTr = transl(0,-0.15,0.4) * troty(-pi);
    end

    properties (Hidden)
        % UR3 %
        qUR3Home = [0 -1.5539 -2.2649 -0.8937 1.5708 0];
        trUR3Home = [-1 0  0  0;
                      0 1  0 -0.135;
                      0 0 -1  0.6;
                      0 0  0  1];
        waypointUR3 = transl(0.2353,0,0.830) * trotz(-pi/2) * trotx(-pi/2);
        gripperUR3offset = transl(0,0,-0.08);
        toolUR3offset = eye(4)
        
        % Meca 500 %
        qMeca500Home = [0 0 0 0 0 0];
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
            self.table.base = self.base;                                % move the table to desired pose
            self.table.animate(0);                                      % plot table
            self.ur3.base = self.base * transl(0.1124, 0.1124, 0.42)*trotz(-pi/2);   % put the UR3 on top of the table
            q = self.qUR3Home;%self.ur3.getpos;                                        % get current joint configuration
            self.ur3.animate(q);                                        % plot the same joint configuration at new pose
            self.meca500.base = self.base * transl(-0.1, 0, 0.05);      % put the Meca500 in the bottom shelf
            q = self.meca500.getpos;                                    % get current joint configuration
            self.meca500.animate(q);                                    % plot the same joint configuration at new pose
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
            qUR3Home = self.qUR3Home;
            qUR3Store = deg2rad([0 -200 -70 -90 180 0]);
            qUR3PreStore = deg2rad([-14 -135 -110 -115 166 0]);
            % Meca500
            qMeca500 = self.meca500.getpos;
            qMeca500Home = self.qMeca500Home;
            qMeca500Store = deg2rad([0 -70 -45 0 115 0]);
            
            % move to home position
            if choice == "home"
                % UR3
                if 0.0001 > self.ur3.getpos - qUR3Store                         % if robot is in "store" position...
                    qMatrixUR3 = jtraj(qUR3Store,qUR3PreStore,50);              % move robot to waypoint
                    qMatrixUR3 = [qMatrixUR3; jtraj(qUR3PreStore,qUR3Home,50)]; % then move to "home" position
                else
                    qMatrixUR3 = jtraj(qUR3,qUR3Home,100);                      % else move directly to home position
                end
                % Meca 500
                qMatrixMeca500 = jtraj(qMeca500,qMeca500Home,100);              % move directly to home
            end
            
            if choice == "store"
                % UR3
                if (0.0001 < sum(self.ur3.getpos - qUR3Home)) || (0.0001 < sum(self.meca500.getpos - qMeca500Home)) % if robot is NOT at home position...
                    self.MoveTo("home");                                        % move robot there first
                end
                qMatrixUR3 = jtraj(qUR3,qUR3PreStore,50);                       % move robot to waypoint
                qMatrixUR3 = [qMatrixUR3; jtraj(qUR3PreStore,qUR3Store,50)];    % then to "store" position
                % Meca 500
                qMatrixMeca500 = jtraj(qMeca500,qMeca500Store, 100);            % move Meca to "store" position as well
            end
            
            for step = 1:size(qMatrixUR3,1)                                     % animate robot movement
                    self.ur3.animate(qMatrixUR3(step,:));
                    self.meca500.animate(qMatrixMeca500(step,:));
                    if (self.CheckCollision("table", self.ur3) > 0) || (self.CheckCollision("table", self.meca500) > 0)% check for collision
                        disp("Damn, I hit something! Press enter to continue...")
                        pause
                    end
                    pause(0.01);
            end
        end
        
        %% play the window cleaning program
        function CleanWindow(self)
            fprintf("\nLet me calculate the trajectories...\n\n");
            
            % get transform trajectory of the cleaning path at the window
            % input: stepsize
            [spongePath,squeegeePath] = self.CalculateCleaningPaths(0.01);
            
            % calculate qMatrix for every movement            
            [qUR3Matrix1, qUR3Steps1] = self.TravelUR3("toWindow", spongePath, self.qUR3Home, 100);
            [qUR3Matrix2, qUR3Steps2] = self.SolveRMRC(self.ur3, spongePath, qUR3Matrix1(end,:), 0.01, 1);
            [qUR3Matrix3, qUR3Steps3] = self.TravelUR3("fromWindow", spongePath, qUR3Matrix2(end,:), 100);
            [qUR3Matrix4, qUR3Steps4] = self.TravelUR3("toWindow", squeegeePath, qUR3Matrix3(end,:), 100);
            [qUR3Matrix5, qUR3Steps5] = self.SolveRMRC(self.ur3, squeegeePath, qUR3Matrix4(end,:), 0.01, 1);
            [qUR3Matrix6, qUR3Steps6] = self.TravelUR3("fromWindow", squeegeePath, qUR3Matrix5(end,:), 100);
            
            % concatenate qMatrices 
            qUR3Matrix = [qUR3Matrix1;
                          qUR3Matrix2;
                          qUR3Matrix3;
                          qUR3Matrix4;
                          qUR3Matrix5;
                          qUR3Matrix6];
            
            % check for collisions in advance
            for i=1:10:size(qUR3Matrix,1)
                if self.CheckCollision("table", self.ur3, qUR3Matrix(i,:))
                    fprintf(["\nWhile working I'm going to collide with something! :-( \n",...
                             "Please remove the obstacles, so I can do my job. \n",...
                             "If everything is clear press any key and I will start cleaning :-)\n"]);
                    pause;
                    break;
                end
            end
            
            fprintf("\nLet's get started!\n");
            
            % home both robots if not already done
            if (0.0001 < sum(self.ur3.getpos - self.qUR3Home)) || (0.0001 < sum(self.meca500.getpos - self.qMeca500Home))
                self.MoveTo("home");
            end
                      
            for i = 1:size(qUR3Matrix,1)
                self.ur3.animate(qUR3Matrix(i,:));
                drawnow;
%                 pause(0.001);
            end
            
            fprintf("\nFinished! The window is clean.\n");
        end
        
         %% move robot through waypoints
%         function CleanWindow(self)
%             [spongePath,squeegeePath]=self.CalculateCleaningPaths(0.01);
%             
%             waypoints(:,:,1) = self.ur3.fkine([-1.5708 -2.0429 -2.4267 -0.2428 1.5708 0]);
%             waypoints(:,:,2) = self.ur3.fkine([-1.5708 -2.0429 -2.4267 -0.2428 1.5708 0]) * transl(0,0,-0.1);
%             waypoints(:,:,3) = self.waypointUR3;
%             waypoints(:,:,4) = spongePath(:,:,1);
%             path1 = self.GetWaypointTrajectory(waypoints, 100);
%             
%             waypoints(:,:,1) = spongePath(:,:,end);
%             waypoints(:,:,2) = self.waypointUR3 * transl(0,0,0.1);
%             waypoints(:,:,3) = self.ur3.fkine([-1.5708 -2.0429 -2.4267 -0.2428 1.5708 0]) * transl(0,0,-0.1);
%             waypoints(:,:,4) = self.ur3.fkine([-1.5708 -2.0429 -2.4267 -0.2428 1.5708 0]);
%             path2 = self.GetWaypointTrajectory(waypoints, 100);
%             
%             
%             qMatrix = self.SolveRMRC(self.ur3, path1, 0.01, 1);
%             for i=1:size(qMatrix,1)
%                 self.ur3.animate(qMatrix(i,:));
%                 pause(0.001);
%             end
%             pause;
%             
%             qMatrix = self.SolveRMRC(self.ur3, spongePath, 0.01, 1);
%             for i=1:size(qMatrix,1)
%                 self.ur3.animate(qMatrix(i,:));
%                 pause(0.001);
%             end
%             pause;
%             
%             qMatrix = self.SolveRMRC(self.ur3, path2, 0.01, 1);
%             for i=1:size(qMatrix,1)
%                 self.ur3.animate(qMatrix(i,:));
%                 pause(0.001);
%             end
%             pause;
%         end
%         
        
    end
end