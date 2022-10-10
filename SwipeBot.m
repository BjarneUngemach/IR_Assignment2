classdef SwipeBot < UR3 & MECA500 & Table & Calculations
    
    properties
        base = eye(4);
        workspace = [-0.5 0.5 -0.5 0.5 0 1]
%         eeOffsMeca500 = troty(-pi/2);
        toolChangeTr = transl(0,-0.15,0.4) * troty(-pi);


        %UR3%
        qUR3Home = [-1.5708 -1.5539 -2.2649 -0.8937 1.5708 0];
        trUR3Home = [-1 0  0  0;
                      0 1  0 -0.135;
                      0 0 -1  0.6;
                      0 0  0  1];
        waypointUR3 = transl(0.2353,0,0.830) * trotz(-pi/2) * trotx(-pi/2);
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
            self.ur3.base = self.base * transl(0.1124, 0.1124, 0.42);   % put the UR3 on top of the table
            q = [-1.5708 -2.0429 -2.4267 -0.2428 1.5708 0];%self.ur3.getpos;                                        % get current joint configuration
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
        
        %% move robot through waypoints
        function CleanWindow(self)
            [spongePath,squeegeePath]=self.CalculateCleaningPaths(0.01);
            
            waypoints(:,:,1) = self.ur3.fkine([-1.5708 -2.0429 -2.4267 -0.2428 1.5708 0]);
            waypoints(:,:,2) = self.ur3.fkine([-1.5708 -2.0429 -2.4267 -0.2428 1.5708 0]) * transl(0,0,-0.1);
            waypoints(:,:,3) = self.waypointUR3;
            waypoints(:,:,4) = spongePath(:,:,1);
            path1 = self.GetWaypointTrajectory(waypoints, 100);
            
            waypoints(:,:,1) = spongePath(:,:,end);
            waypoints(:,:,2) = self.waypointUR3 * transl(0,0,0.1);
            waypoints(:,:,3) = self.ur3.fkine([-1.5708 -2.0429 -2.4267 -0.2428 1.5708 0]) * transl(0,0,-0.1);
            waypoints(:,:,4) = self.ur3.fkine([-1.5708 -2.0429 -2.4267 -0.2428 1.5708 0]);
            path2 = self.GetWaypointTrajectory(waypoints, 100);
            
            
            qMatrix = self.SolveRMRC(self.ur3, path1, 0.01, 1);
            for i=1:size(qMatrix,1)
                self.ur3.animate(qMatrix(i,:));
                pause(0.001);
            end
            pause;
            
            qMatrix = self.SolveRMRC(self.ur3, spongePath, 0.01, 1);
            for i=1:size(qMatrix,1)
                self.ur3.animate(qMatrix(i,:));
                pause(0.001);
            end
            pause;
            
            qMatrix = self.SolveRMRC(self.ur3, path2, 0.01, 1);
            for i=1:size(qMatrix,1)
                self.ur3.animate(qMatrix(i,:));
                pause(0.001);
            end
            pause;
        end
        
        %% move UR3 to the starting position at the window
        function qMatrix = MoveUR3toWindow(self,startPos,steps)
            waypoints(:,:,1) = self.trUR3Home;
            waypoints(:,:,2) = self.waypointUR3;
            waypoints(:,:,3) = startPos;
            
            qGuess = self.qUR3Home;
            qMatrix = nan(2*steps,6);
            
            for i=1:size(waypoints,3)-1
                q1 = self.ur3.ikcon(waypoints(:,:,i),qGuess);
                q2 = self.ur3.ikcon(waypoints(:,:,i+1),qGuess);
                
                qMatrix(((i-1)*steps+1):(i*steps),:) = jtraj(q1,q2,steps);
                
                qGuess = qMatrix((i*steps),:);
            end
        end
    end
end