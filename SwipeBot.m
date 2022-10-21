classdef SwipeBot < Calculations & UR3 & MECA500 & Gripper3F & Gripper2F & Table & Sponge & Squeegee & Hand
    
    properties
        base = eye(4);
        workspace = [-0.31 0.8 -0.5 0.5 -0.21 1.1]
        toolChangeTr = transl(0,-0.12,0.4) * troty(-pi);
        environment;
        floor;
    end

    properties (Hidden)
        % UR3 %
        qUR3Home = [0 -1.4900 -2.3247 -0.8976 1.5708 0];
        trUR3Home = [-1 0  0  0.0001;
                      0 1  0 -0.12;
                      0 0 -1  0.5999;
                      0 0  0  1];
        waypointUR3 = transl(0.2353,0,0.830) * trotz(-pi/2) * trotx(-pi/2);
        gripperUR3offset = transl(0,0,-0.0590) * trotz(-pi/2);
        toolUR3offset = transl(0,0,-0.12);
        
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
            axis equal
            hold on
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
            self.environment = PlaceObject("Environment.ply", [0,0,-0.2]);
            self.floor = surf([-4,-4;4.3,4.3],[-1.6,3.6;-1.6,3.6],[-0.2,-0.2;-0.2,-0.2],...
                         'CData',imread('Grass.jpg'),'FaceColor','texturemap');
            view(135,170);
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
                app.TextArea.Value = [app.TextArea.Value;
                                      " ";
                                      "Move both robots to home position."];
                drawnow;
                scroll(app.TextArea, 'bottom');
                
                if 0.0001 > self.ur3.getpos - qUR3Store                         % if robot is in "store" position...
                    % UR3
                    qMatrixUR3 = jtraj(qUR3Store,qUR3PreStore,25);              % move robot to waypoint
                    qMatrixUR3 = [qMatrixUR3; jtraj(qUR3PreStore,qUR3Home,25)]; % then move to "home" position
                    qMatrixMeca500 = jtraj(qMeca500,qMeca500Home,50);
                else
                    qMatrixUR3 = jtraj(qUR3,qUR3Home,50);                      % else move directly to home position
                    qMatrixMeca500 = jtraj(qMeca500,qMeca500Home,50);
                end
                self.sponge.base = self.spongeHome;
                self.sponge.animate(0);
                self.squeegee.base = self.squeegeeHome;
                self.squeegee.animate(0);
            end
            
            % move to store position
            if choice == "store"
                app.TextArea.Value = [app.TextArea.Value;
                                      " ";
                                      "Move both robots to store position."];
                drawnow;
                scroll(app.TextArea, 'bottom');
                
                % UR3
                if (0.0001 < sum(self.ur3.getpos - qUR3Home)) || (0.0001 < sum(self.meca500.getpos - qMeca500Home)) % if robot is NOT at home position...
                    self.MoveTo("home");                                        % move robot there first
                end
                qMatrixUR3 = jtraj(qUR3,qUR3PreStore,25);                       % move robot to waypoint
                qMatrixUR3 = [qMatrixUR3; jtraj(qUR3PreStore,qUR3Store,25)];    % then to "store" position
                % Meca 500
                qMatrixMeca500 = jtraj(qMeca500,qMeca500Store, 50);            % move Meca to "store" position as well
            end
            
            % animate movement
            for step = 1:size(qMatrixUR3,1)
                self.ur3.animate(qMatrixUR3(step,:));
                self.meca500.animate(qMatrixMeca500(step,:));
                self.UpdateGripper3F;
                self.UpdateGripper2F;
                if (self.CheckCollision(["table","hand"], self.ur3) > 0)% check for collision
                    app.TextArea.Value = [app.TextArea.Value;
                                          " ";
                                          "COLLISION DETECTED!";
                                          "E-Stop triggered"];
                    drawnow;
                    scroll(app.TextArea, 'bottom');
                    app.EMERGENCYSTOPButton.Value = 1;
                end
                drawnow;
                % E-Stop-Loop
                if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                    estopApp = EStopApp;                    % open E-Stop-App
                    while estopApp.estop == 0               % run empty loop to wait for user decision
                        pause(0.5);
                    end
                    if estopApp.estop == 1                  % user chose continue
                        estopApp.delete;                    % close app
                        app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                        continue;
                    elseif estopApp.estop == 2              % user chose exit
                        estopApp.delete;                    % close app
                        return;
                    end
                end
            end
        end
        
        %% play the window cleaning program
        function CleanWindow(self,app)
            app.TextArea.Value = [app.TextArea.Value;
                                  " ";
                                  "Calculating trajectories..."];
            drawnow;
            scroll(app.TextArea, 'bottom');

            
            % get transform trajectory of the cleaning path at the window
            % input: stepsize
            [spongePath,squeegeePath] = self.CalculateCleaningPaths(app, 0.03);
            
            % calculate qMatrix for every movement            
            [qUR3Matrix1, qUR3Steps1] = self.TravelUR3("toWindow", spongePath, self.qUR3Home, 50);
            [qUR3Matrix2, qUR3Steps2] = self.SolveRMRC(self.ur3, spongePath, qUR3Matrix1(end,:), 0.03, 1, (self.gripperUR3offset * self.toolUR3offset));
            [qUR3Matrix3, qUR3Steps3] = self.TravelUR3("fromWindow", spongePath, qUR3Matrix2(end,:), 50);
            [qUR3Matrix4, qUR3Steps4] = self.TravelUR3("toWindow", squeegeePath, qUR3Matrix3(end,:), 50);
            [qUR3Matrix5, qUR3Steps5] = self.SolveRMRC(self.ur3, squeegeePath, qUR3Matrix4(end,:), 0.03, 1, (self.gripperUR3offset * self.toolUR3offset));
            [qUR3Matrix6, qUR3Steps6] = self.TravelUR3("fromWindow", squeegeePath, qUR3Matrix5(end,:), 50);
            
            % concatenate qMatrices 
            qUR3MatrixA = [qUR3Matrix1;
                           qUR3Matrix2;
                           qUR3Matrix3];
            qUR3MatrixB = [qUR3Matrix4;
                           qUR3Matrix5;
                           qUR3Matrix6];
            
            % check for collisions in advance
            qUR3Matrix = [qUR3MatrixA; qUR3MatrixB];
            for i=1:10:size(qUR3Matrix,1)
                if self.CheckCollision("table", self.ur3, qUR3Matrix(i,:))
                    app.TextArea.Value = [app.TextArea.Value;
                                          " ";
                                          "A collision was detected that can't be avoided.";
                                          "Please remove the obstacles from the workspace.";
                                          "If everything is clear press any key and the cleaning process will start."];
                    drawnow;
                    scroll(app.TextArea, 'bottom');
                    pause;
                    break;
                end
            end
                                                                                                          
            % home both robots if not already done
            if (0.0001 < sum((self.ur3.getpos - self.qUR3Home).^2)) || (0.0001 < sum((self.meca500.getpos - self.qMeca500Home).^2))
                app.TextArea.Value = [app.TextArea.Value;
                                      " ";
                                      "Home all systems."];
                drawnow;
                scroll(app.TextArea, 'bottom');
                self.MoveTo("home", app);
                if app.EMERGENCYSTOPButton.Value == 1
                    return;
                end
            end
            
            app.TextArea.Value = [app.TextArea.Value;
                                  " ";
                                  "Cleaning Process started..."];
            drawnow;
            scroll(app.TextArea, 'bottom');
            
            %%%%%%%%%% ANIMATION %%%%%%%%%%
            %%% GET SPONGE %%%
            self.ChangeTools("getSponge", app);
            if app.EMERGENCYSTOPButton.Value == 1
                    return;
            end
            
            %%% SPONGE PATH %%%
            app.TextArea.Value = [app.TextArea.Value;
                                  " ";
                                  "Cleaning the window..."];
            drawnow;
            scroll(app.TextArea, 'bottom');
            
            slow = 0;   % flag for reduced speed
            for i = 1:size(qUR3MatrixA,1)
                % workspace checking
                if (i ~= 1) & (self.CheckWorkspace ~= 0)
                    if slow == 0
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "Close obstacle detected.";
                                              "Going on with reduced speed"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        slow = 1;
                    end
                    % make steps in between
                    s = 0.2:0.2:1;        %linear segment (10 steps)
                    for j = 1:3
                        self.ur3.animate((1-s(j))*qUR3MatrixA(i-1,:) + s(j)*qUR3MatrixA(i,:));
                        self.UpdateGripper3F;
                        self.UpdateSponge(self.ur3.fkine(self.ur3.getpos)/(self.gripperUR3offset));
                        drawnow;
                    end
                elseif slow == 1
                    app.TextArea.Value = [app.TextArea.Value;
                                          " ";
                                          "Workspace is clear again.";
                                          "Going on with full speed"];
                    drawnow;
                    scroll(app.TextArea, 'bottom');
                    slow = 0;
                end
                
                % animation
                self.ur3.animate(qUR3MatrixA(i,:));
                self.UpdateGripper3F;
                self.UpdateSponge(self.ur3.fkine(self.ur3.getpos)/(self.gripperUR3offset));
                drawnow;
                if (self.CheckCollision("table", self.ur3) > 0)% check for collision
                    app.TextArea.Value = [app.TextArea.Value;
                                          " ";
                                          "COLLISION DETECTED!";
                                          "E-Stop triggered"];
                    drawnow;
                    scroll(app.TextArea, 'bottom');
                    app.EMERGENCYSTOPButton.Value = 1;
                end
                % E-Stop-Loop
                if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                    estopApp = EStopApp;                    % open E-Stop-App
                    while estopApp.estop == 0               % run empty loop to wait for user decision
                        pause(0.5);
                    end
                    if estopApp.estop == 1                  % user chose continue
                        estopApp.delete;                    % close app
                        app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                        continue;
                    elseif estopApp.estop == 2              % user chose exit
                        estopApp.delete;                    % close app
                        return;
                    end
                end
            end
            
            %%% GET SQUEEGEE %%%
            app.TextArea.Value = [app.TextArea.Value;
                                  " ";
                                  "Changing tools..."];
            drawnow;
            scroll(app.TextArea, 'bottom');
            
            self.ChangeTools("switchTools", app);
            if app.EMERGENCYSTOPButton.Value == 1
                    return;
            end
            
            %%% SQUEEGEE PATH %%%
            app.TextArea.Value = [app.TextArea.Value;
                                  " ";
                                  "Drying the window..."];
            drawnow;
            scroll(app.TextArea, 'bottom');
            
            slow = 0;   % flag for reduced speed
            for i = 1:size(qUR3MatrixB,1)
                % workspace checking
                if (i ~= 1) & (self.CheckWorkspace ~= 0)
                    if slow == 0
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "Close obstacle detected.";
                                              "Going on with reduced speed"];
                        scroll(app.TextArea, 'bottom');
                        drawnow;
                        slow = 1;
                    end
                    % make steps in between
                    s = 0.2:0.2:1;        %linear segment (10 steps)
                    for j = 1:3
                        self.ur3.animate((1-s(j))*qUR3MatrixB(i-1,:) + s(j)*qUR3MatrixB(i,:));
                        self.UpdateGripper3F;
                        self.UpdateSqueegee(self.ur3.fkine(self.ur3.getpos)/(self.gripperUR3offset));
                        drawnow;
                    end
                elseif slow == 1
                    app.TextArea.Value = [app.TextArea.Value;
                                          " ";
                                          "Workspace is clear again.";
                                          "Going on with full speed"];
                    drawnow;
                    scroll(app.TextArea, 'bottom');
                    slow = 0;
                end
                
                % animation
                self.ur3.animate(qUR3MatrixB(i,:));
                self.UpdateGripper3F;
                self.UpdateSqueegee(self.ur3.fkine(self.ur3.getpos)/(self.gripperUR3offset));
                drawnow;
                if (self.CheckCollision("table", self.ur3) > 0)% check for collision
                    app.TextArea.Value = [app.TextArea.Value;
                                          " ";
                                          "COLLISION DETECTED!";
                                          "E-Stop triggered"];
                    drawnow;
                    scroll(app.TextArea, 'bottom');
                    app.EMERGENCYSTOPButton.Value = 1;
                end
                % E-Stop-Loop
                if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                    estopApp = EStopApp;                    % open E-Stop-App
                    while estopApp.estop == 0               % run empty loop to wait for user decision
                        pause(0.5);
                    end
                    if estopApp.estop == 1                  % user chose continue
                        estopApp.delete;                    % close app
                        app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                        continue;
                    elseif estopApp.estop == 2              % user chose exit
                        estopApp.delete;                    % close app
                        return;
                    end
                end
            end
            
            %%% REMOVE SQUEEGEE %%%
            self.ChangeTools("removeSqueegee", app);
            if app.EMERGENCYSTOPButton.Value == 1
                    return;
            end
            
            %%% FINISHED %%%
            app.TextArea.Value = [app.TextArea.Value;
                                  " ";
                                  "Cleaning-Process finished.";
                                  "Enjoy your clean windows."];
            drawnow;
            scroll(app.TextArea, 'bottom');
        end
        
        %% change tools
        function ChangeTools(self, mode, app)
            if mode == "getSponge"
                % move Meca500 to grab sponge
                [qMatrix1, steps1] = self.MoveMeca500("sponge", "toTool");
                for i = 1:steps1
                    self.meca500.animate(qMatrix1(i,:));
                    self.UpdateGripper2F;
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.meca500) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
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
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.meca500) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
                end
                
                % grab with UR3
                [qMatrixA, stepsA] = MoveUR3(self, "grabTool");
                for i = 1:stepsA
                    self.ur3.animate(qMatrixA(i,:));
                    self.UpdateGripper3F;
                    pause(0.001);
                    drawnow;
                    if (self.CheckCollision("table", self.ur3) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
                end
                
                % close 3 Finger Gripper
                self.g3FStatus = "close";
                self.UpdateGripper3F;
                
                % open 2 Finger Gripper
                self.g2FStatus = "open";
                self.UpdateGripper2F;
                
                % retreat Meca500
                [qMatrix3, steps3] = self.MoveMeca500("sponge", "toWait");
                for i = 1:steps3
                    self.meca500.animate(qMatrix3(i,:));
                    self.UpdateGripper2F;
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.meca500) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
                end

                % pull UR3 out
                [qMatrixB, stepsB] = MoveUR3(self, "getTool");
                for i = 1:stepsB
                    self.ur3.animate(qMatrixB(i,:));
                    self.UpdateGripper3F;
                    self.UpdateSponge(self.ur3.fkine(self.ur3.getpos)/(self.gripperUR3offset));
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.ur3) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
                end
            end
            
            if mode == "switchTools"
                % bring UR3 in
                [qMatrixC, stepsC] = MoveUR3(self, "bringTool");
                for i = 1:stepsC
                    self.ur3.animate(qMatrixC(i,:));
                    self.UpdateGripper3F;
                    self.UpdateSponge(self.ur3.fkine(self.ur3.getpos)/(self.gripperUR3offset));
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.ur3) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
                end
                
                % grab sponge again
                [qMatrix4, steps4] = self.MoveMeca500("sponge", "fromWait");
                for i = 1:steps4
                    self.meca500.animate(qMatrix4(i,:));
                    self.UpdateGripper2F;
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.meca500) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
                end
                
                % close 2 Finger Gripper
                self.g2FStatus = "close";
                self.UpdateGripper2F;
                
                % open 3 Finger Gripper
                self.g3FStatus = "open";
                self.UpdateGripper3F;
                
                % UR3 release tool
                [qMatrixD, stepsD] = MoveUR3(self, "releaseTool");
                for i = 1:stepsD
                    self.ur3.animate(qMatrixD(i,:));
                    self.UpdateGripper3F;
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.ur3) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
                end
                
                % bring sponge back
                [qMatrix5, steps5] = self.MoveMeca500("sponge", "fromUR3");
                for i = 1:steps5
                    self.meca500.animate(qMatrix5(i,:));
                    self.UpdateGripper2F;
                    self.UpdateSponge(self.meca500.fkine(self.meca500.getpos)/(self.gripperMeca500offset));
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.meca500) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
                end
                
                % open 2 Finger Gripper
                self.g2FStatus = "open";
                self.UpdateGripper2F;
                
                % retreat from sponge
                [qMatrix6, steps6] = self.MoveMeca500("sponge", "fromTool");
                for i = 1:steps6
                    self.meca500.animate(qMatrix6(i,:));
                    self.UpdateGripper2F;
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.meca500) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
                end
                
                % move to squeegee
                [qMatrix7, steps7] = self.MoveMeca500("squeegee", "toTool");
                for i = 1:steps7
                    self.meca500.animate(qMatrix7(i,:));
                    self.UpdateGripper2F;
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.meca500) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
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
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.meca500) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
                end
                
                % grab with UR3
                [qMatrixE, stepsE] = MoveUR3(self, "grabTool");
                for i = 1:stepsE
                    self.ur3.animate(qMatrixE(i,:));
                    self.UpdateGripper3F;
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.ur3) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
                end
                
                % close 3 Finger Gripper
                self.g3FStatus = "close";
                self.UpdateGripper3F; 
                
                % open 2 Finger Gripper
                self.g2FStatus = "open";
                self.UpdateGripper2F;
                
                % retreat Meca500
                [qMatrix9, steps9] = self.MoveMeca500("squeegee", "toWait");
                for i = 1:steps9
                    self.meca500.animate(qMatrix9(i,:));
                    self.UpdateGripper2F;
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.meca500) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
                end
                
                % pull UR3 out
                [qMatrixF, stepsF] = MoveUR3(self, "getTool");
                for i = 1:stepsF
                    self.ur3.animate(qMatrixF(i,:));
                    self.UpdateGripper3F;
                    self.UpdateSqueegee(self.ur3.fkine(self.ur3.getpos)/(self.gripperUR3offset));
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.ur3) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
                end
            end
            
            if mode == "removeSqueegee"
                % bring UR3 in
                [qMatrixG, stepsG] = MoveUR3(self, "bringTool");
                for i = 1:stepsG
                    self.ur3.animate(qMatrixG(i,:));
                    self.UpdateGripper3F;
                    self.UpdateSqueegee(self.ur3.fkine(self.ur3.getpos)/(self.gripperUR3offset));
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.ur3) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
                end
                
                % grab squeegee again
                [qMatrix10, steps10] = self.MoveMeca500("squeegee", "fromWait");
                for i = 1:steps10
                    self.meca500.animate(qMatrix10(i,:));
                    self.UpdateGripper2F;
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.meca500) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
                end
                
                % close 2 Finger Gripper
                self.g2FStatus = "close";
                self.UpdateGripper2F;
                
                % open 3 Finger Gripper
                self.g3FStatus = "open";
                self.UpdateGripper3F;
                
                % UR3 release tool
                [qMatrixH, stepsH] = MoveUR3(self, "releaseTool");
                for i = 1:stepsH
                    self.ur3.animate(qMatrixH(i,:));
                    self.UpdateGripper3F;
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.ur3) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
                end
                
                % bring squeegee back
                [qMatrix11, steps11] = self.MoveMeca500("squeegee", "fromUR3");
                for i = 1:steps11
                    self.meca500.animate(qMatrix11(i,:));
                    self.UpdateGripper2F;
                    self.UpdateSqueegee(self.meca500.fkine(self.meca500.getpos)/(self.gripperMeca500offset));
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.meca500) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
                end
                
                % open 2 Finger Gripper
                self.g2FStatus = "open";
                self.UpdateGripper2F;
                
                % retreat from sponge
                [qMatrix12, steps12] = self.MoveMeca500("squeegee", "fromTool");
                for i = 1:steps12
                    self.meca500.animate(qMatrix12(i,:));
                    self.UpdateGripper2F;
                    drawnow;
                    pause(0.001);
                    if (self.CheckCollision("table", self.meca500) > 0)% check for collision
                        app.TextArea.Value = [app.TextArea.Value;
                                              " ";
                                              "COLLISION DETECTED!";
                                              "E-Stop triggered"];
                        drawnow;
                        scroll(app.TextArea, 'bottom');
                        app.EMERGENCYSTOPButton.Value = 1;
                    end
                    % E-Stop-Loop
                    if app.EMERGENCYSTOPButton.Value == 1       % if E-Stop triggerd
                        estopApp = EStopApp;                    % open E-Stop-App
                        while estopApp.estop == 0               % run empty loop to wait for user decision
                            pause(0.5);
                        end
                        if estopApp.estop == 1                  % user chose continue
                            estopApp.delete;                    % close app
                            app.EMERGENCYSTOPButton.Value = 0;  % reset E-Stop
                            continue;
                        elseif estopApp.estop == 2              % user chose exit
                            estopApp.delete;                    % close app
                            return;
                        end
                    end
                end
            end
        end
        
        %% animate a IBVS
        function RunIBVS(self, app)
            % simulate meca 500 to bring the sign
            tr = self.toolChangeTr * self.gripperMeca500offset;
            qTr = self.meca500.ikcon(tr,deg2rad([0 -18 18 0 0 180]));
            self.meca500.animate(qTr);
            self.UpdateGripper2F;
            
            
            % gain
            lambda = 1;
            
            % servoing speed
            fps = 25;
            
            % Image goal points
            goalPoints = [626, 398, 398, 626;        % joint state [-0.0000   -1.2852   -1.7299   -1.6972    1.5708   -0.0000]
                          389, 389, 617, 617];
%             PStar = [657, 399, 394, 655;        % joint state [0.8330   -1.3682   -2.2793   -1.5167    1.1626    0.9093]
%                      381, 340, 699, 668];

            % corner points of sign in local coordinate frame
            cornerPointsLocal = [ 0.05,-0.05,-0.05, 0.05;
                                 -0.05,-0.05, 0.05, 0.05;
                                  0   , 0   , 0   , 0   ];
           
            %camera
            cam = CentralCamera('focal', 0.10, 'pixel', 10e-5, ...
                                'resolution', [1024 1024], 'centre', [512 512],...
                                'name', 'IBVS Camera');   
            cam.plot(goalPoints, '*');
            cam.hold;
            
            % initial values
            qUR3 = self.ur3.getpos;
            qRef = zeros(1,6);
%             cornerPoints = zeros(4,3);
%             middlePoint = zeros(3,1);
            
            while app.RunIBVSButton.Value == 1
                eePos = self.ur3.fkine(qUR3);                % get current position
                cam.T = eePos;
                
                qMeca500 = self.meca500.getpos;
                if any(abs(qRef - qMeca500) > 0.001)
                    qRef = qMeca500;
                    signPos = self.meca500.fkine(qMeca500)/self.gripperMeca500offset; 
                    middlePoint = signPos(1:3,4);
                    cornerPoints = [signPos * [cornerPointsLocal',ones(size(cornerPointsLocal',1),1)]']';
                end

                picturePoints = cam.plot(cornerPoints(:,1:3)', 'Tcam', eePos, 'o');        % calculate error
                pointError = goalPoints - picturePoints;
                pointError = pointError(:);
                
                depth = sqrt(sum((middlePoint - eePos(1:3,4)).^2));        % calculate camera distance
                
                IJ = cam.visjac_p(picturePoints, depth);                % get Image Jacobian
                
                camVel = lambda * pinv(IJ) * pointError;                % solve for camera velocity
                
                J = self.ur3.jacobn(qUR3);                                 % Robot Jacobian in tool frame
                
                qdot = pinv(J) * camVel;                                % solve RMRC
                qUR3 = qUR3 + (1/fps)*qdot';
                
                self.ur3.animate(qUR3);                                    % update robot
                self.UpdateGripper3F;
                
                drawnow;
                pause(1/fps);
            end
            
            % simulate meca 500 to bring back the sign
            self.meca500.animate(qTr);
            self.UpdateGripper2F;
            
        end
        
        %% test 
        function Test(self, app)
            app.EMERGENCYSTOPButton.Value = 0;
        end
              
    end
end