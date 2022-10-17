
classdef SwipeBot < Calculations & UR3 & MECA500 & Gripper3F & Gripper2F & Table & Sponge & Squeegee & Hand

    
    properties
        base = eye(4);
        workspace = [-0.5 0.5 -0.5 0.5 0 1.1]
        toolChangeTr = transl(0,-0.12,0.4) * troty(-pi);
    end

    properties (Hidden)
        % UR3 %
        qUR3Home = [0 -1.4900 -2.3247 -0.8976 1.5708 0];
        trUR3Home = [-1 0  0  0;
                      0 1  0 -0.12;
                      0 0 -1  0.6;
                      0 0  0  1];
        waypointUR3 = transl(0.2353,0,0.830) * trotz(-pi/2) * trotx(-pi/2);
        gripperUR3offset = transl(0,0,-0.0590);
        toolUR3offset = eye(4)
        
        % Meca 500 %
        qMeca500Home = [0 0 0 0 -pi/2 0];
        gripperMeca500offset = transl(0, 0.0685, 0.051) * trotx(pi/2) * trotz(pi/2);
        
        % Hand %
        handPos1 = transl(0.15, -0.2, 0.6) * trotz(pi) * trotx(pi/2);
    end
    
    methods
    %% %%%%% Methods to generate the Swipebot %%%%% %%
    
        %% Swipebot object structor
        function self = SwipeBot()
            % because this function inherits the models from the other
            % classes (mentioned above), no "getSwipebot" function is
            % needed
            
            self.UpdatePosition();  %update the position of the new generated elements
            
            % open App if not already open
            if isempty(findall(0,'Name','SwipeBotApp'))
                app = SwipeBotApp;
                assignin('base', "app", app);
            end
            % print Info
            app.TextArea.Value = "SwipeBot-System ist ready to use.";
        end
        
        %% Update all elements to the desired pose of the SwipeBot
        function UpdatePosition(self)
            self.table.base = self.base;                                % move the table to desired pose
            self.table.animate(0);                                      % plot table
            self.ur3.base = self.base * transl(0.1124, 0.1124, 0.42)*trotz(-pi/2);   % put the UR3 on top of the table
            q = self.qUR3Home;%self.ur3.getpos;                                      % get current joint configuration
            self.ur3.animate(q);                                        % plot the same joint configuration at new pose
            self.meca500.base = self.base * transl(-0, 0.15, 0.05) * trotz(-pi/2);   % put the Meca500 in the bottom shelf
            q = self.qMeca500Home;%self.meca500.getpos;                                    % get current joint configuration
            self.meca500.animate(q);                                    % plot the same joint configuration at new pose
            self.UpdateGripper3F;
            self.UpdateGripper2F;

            self.UpdateSponge(self.spongeHome);
            self.UpdateSqueegee(self.squeegeeHome);

            self.hand.base = self.handPos1;
            %self.hand.animate(0);
        end
    end
    
    methods
    %% %%%%% Methods to simulate the Swipebot %%%%% %%
    % refer to Calculations.m to find the big calculation programms
    
        %% function homes all robots
        function MoveTo(self, choice, app)
            % predefine jointstates
            % UR3
            qUR3 = self.ur3.getpos;
            qUR3Home = self.qUR3Home;
            qUR3Store = deg2rad([0 -190 -80 -90 180 0]);
            qUR3PreStore = deg2rad([-14 -135 -110 -115 166 0]);
            % Meca500
            qMeca500 = self.meca500.getpos;
            qMeca500Home = self.qMeca500Home;
            qMeca500Store = deg2rad([0 -70 -45 0 115 0]);
            
            % move to home position
            if choice == "home"
                app.TextArea.Value = "Move both robots to home position.";
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
                app.TextArea.Value = "Move both robots to store position.";
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
                    self.UpdateGripper3F;
                    self.UpdateGripper2F;
                    if (self.CheckCollision("table", self.ur3) > 0) || (self.CheckCollision("table", self.meca500) > 0)% check for collision
                        disp("Damn, I hit something! Press enter to continue...")
                        pause
                    end
                    pause(0.001);
            end
        end
        
        %% play the window cleaning program
        function CleanWindow(self,app)
            fprintf("\nLet me calculate the trajectories...\n\n");
            app.TextArea.Value = "Calculating trajectories...";
            drawnow;
            
            % get transform trajectory of the cleaning path at the window
            % input: stepsize
            [spongePath,squeegeePath] = self.CalculateCleaningPaths(app, 0.01);
            
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
                    app.TextArea.Value = [app.TextArea.Value;
                                          " ";
                                          "A collision was detected that can't be avoided.";
                                          "Please remove the obstacles from the workspace.";
                                          "If everything is clear press any key and the cleaning process will start."];
                    pause;
                    break;
                end
            end
                                                                                                          
            % home both robots if not already done
            if (0.0001 < sum((self.ur3.getpos - self.qUR3Home).^2)) || (0.0001 < sum((self.meca500.getpos - self.qMeca500Home).^2))
                fprintf("\nI'll home everything first to be sure everything is ready.\n")
                app.TextArea.Value = [app.TextArea.Value;
                                      " ";
                                      "Home all systems."];
                self.MoveTo("home");
            end
            
            fprintf("\nLet's get started!\n");
            app.TextArea.Value = [app.TextArea.Value;
                                  " ";
                                  "Cleaning Process started."];
            
            slow = 0;   % flag for reduced speed
            for i = 1:size(qUR3Matrix,1)
                if self.CheckWorkspace
                    if slow == 0
                        fprintf("Something is close. I go slower now");
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "Close obstacle detected.";
                                              "Going on with reduced speed"];
                        slow = 1;
                    end
                    pause(0.05);
                elseif slow == 1
                    fprintf("Everything clear again. Let's speed up.");
                    app.TextArea.Value = [app.TextArea.Value;
                                          " ";
                                          "Workspace is clear again.";
                                          "Going on with full speed"];
                    slow = 0;
                end
                
                self.ur3.animate(qUR3Matrix(i,:));
                self.UpdateGripper3F;
                if app.EmergencyStopButton.Value
                    if ~app.EmergencyStopButton.Value
                    end
                end
                drawnow;
%                 pause(0.001);
            end
            
            fprintf("\nFinished! The window is clean.\n");
            app.TextArea.Value = [app.TextArea.Value;
                                  " ";
                                  "Cleaning-Process finished.";
                                  "Enjoy your clean windows."];
        end
        
        %% change tools
        function ChangeTools(self, mode)
            if mode == "getSponge"
                % move Meca500 to grab sponge
                [qMatrix1, steps1] = self.MoveMeca500("sponge", "toTool");
                for i = 1:steps1
                    self.meca500.animate(qMatrix1(i,:));
                    self.UpdateGripper2F;
                    pause(0.001);
                end
                
                % close 2 Finger Gripper
                self.g2FStatus = "close";
                self.UpdateGripper2F;

                % lift sponge up
                [qMatrix2, steps2] = self.MoveMeca500("sponge", "toUR3");
                for i = 1:steps2
                    self.meca500.animate(qMatrix2(i,:));
                    self.UpdateGripper2F;
                    self.UpdateSponge(self.meca500.fkine(self.meca500.getpos)/(self.gripperMeca500offset));
                    pause(0.001);
                end
                
                % open 2 Finger Gripper
                self.g2FStatus = "open";
                self.UpdateGripper2F;
                
                % retreat Meca500
                [qMatrix3, steps3] = self.MoveMeca500("sponge", "toWait");
                for i = 1:steps3
                    self.meca500.animate(qMatrix3(i,:));
                    self.UpdateGripper2F;
                    pause(0.001);
                end
            end
            
            if mode == "switchTools"
                % grab sponge again
                [qMatrix4, steps4] = self.MoveMeca500("sponge", "fromWait");
                for i = 1:steps4
                    self.meca500.animate(qMatrix4(i,:));
                    self.UpdateGripper2F;
                    pause(0.001);
                end
                
                % close 2 Finger Gripper
                self.g2FStatus = "close";
                self.UpdateGripper2F;
                
                % bring sponge back
                [qMatrix5, steps5] = self.MoveMeca500("sponge", "fromUR3");
                for i = 1:steps5
                    self.meca500.animate(qMatrix5(i,:));
                    self.UpdateGripper2F;
                    self.UpdateSponge(self.meca500.fkine(self.meca500.getpos)/(self.gripperMeca500offset));
                    pause(0.001);
                end
                
                % open 2 Finger Gripper
                self.g2FStatus = "open";
                self.UpdateGripper2F;
                
                % retreat from sponge
                [qMatrix6, steps6] = self.MoveMeca500("sponge", "fromTool");
                for i = 1:steps6
                    self.meca500.animate(qMatrix6(i,:));
                    self.UpdateGripper2F;
                    pause(0.001);
                end
                
                % move to squeegee
                [qMatrix7, steps7] = self.MoveMeca500("squeegee", "toTool");
                for i = 1:steps7
                    self.meca500.animate(qMatrix7(i,:));
                    self.UpdateGripper2F;
                    pause(0.001);
                end
                
                % close 2 Finger Gripper
                self.g2FStatus = "close";
                self.UpdateGripper2F;

                % lift squeegee up
                [qMatrix8, steps8] = self.MoveMeca500("squeegee", "toUR3");
                for i = 1:steps8
                    self.meca500.animate(qMatrix8(i,:));
                    self.UpdateGripper2F;
                    self.UpdateSqueegee(self.meca500.fkine(self.meca500.getpos)/(self.gripperMeca500offset));
                    pause(0.001);
                end
                
                % open 2 Finger Gripper
                self.g2FStatus = "open";
                self.UpdateGripper2F;
                
                % retreat Meca500
                [qMatrix9, steps9] = self.MoveMeca500("squeegee", "toWait");
                for i = 1:steps9
                    self.meca500.animate(qMatrix9(i,:));
                    self.UpdateGripper2F;
                    pause(0.001);
                end
            end
            
            if mode == "removeSqueegee"
                % grab squeegee again
                [qMatrix10, steps10] = self.MoveMeca500("squeegee", "fromWait");
                for i = 1:steps10
                    self.meca500.animate(qMatrix10(i,:));
                    self.UpdateGripper2F;
                    pause(0.001);
                end
                
                % close 2 Finger Gripper
                self.g2FStatus = "close";
                self.UpdateGripper2F;
                
                % bring squeegee back
                [qMatrix11, steps11] = self.MoveMeca500("squeegee", "fromUR3");
                for i = 1:steps11
                    self.meca500.animate(qMatrix11(i,:));
                    self.UpdateGripper2F;
                    self.UpdateSqueegee(self.meca500.fkine(self.meca500.getpos)/(self.gripperMeca500offset));
                    pause(0.001);
                end
                
                % open 2 Finger Gripper
                self.g2FStatus = "open";
                self.UpdateGripper2F;
                
                % retreat from sponge
                [qMatrix12, steps12] = self.MoveMeca500("squeegee", "fromTool");
                for i = 1:steps12
                    self.meca500.animate(qMatrix12(i,:));
                    self.UpdateGripper2F;
                    pause(0.001);
                end
            end
        end
        %% test 
        function Test(self, app)
            app.TextArea.Value = ["Test";"another Test"];
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