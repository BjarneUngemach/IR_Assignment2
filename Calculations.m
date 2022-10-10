classdef Calculations < handle
    % class contains the big calculations for SwipeBot.m
    
    
    methods
    %%% RMRC SOLVER %%%
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
            
            qGuess = robot.getpos;
            
            
            qMatrix(1,:) = robot.getpos;%robot.ikine(trajectory(:,:,1),qGuess);                               % solve first qMatrix with inverse kinematics
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
        
    %%% COLLISION CHECKING %%%
        %% collision checking function
        function [numberCollidingPoints,collisionInfo] = CheckCollision(self, checkMatrix, robot, q)
            if nargin < 3
                robot = [self.ur3];                 % pack all robots in an array
            end    
            if nargin < 4
                for r = 1:size(robot,1)
                    q(r,:) = robot(r).getpos;       % get current joint state for every robot
                end
            end
            
            %%% CONSTANTS %%%
            SAFETYDISTANCE = 0.01;
            % offset of center of link from joint transform "[x y z r p y]"
            OFFSET(:,:,1) = [0      -0.0266 0       -pi/2   0       0;      % offsets of UR3
                             0.1220 0       0.1199  -pi/2   -pi/2   0;
                             0.1077 0       0.0208  -pi/2   -pi/2   0;
                             0      0       0.0032  0       0       -pi;
                             0      0       0.0027  0       0       -pi;
                             0      0       -0.0160 0       0       -pi];
            % shape matrix defines geometry of collision checking cage (first collumn defines shape: "1" for cylinder, "2" for prism) 
            SHAPE(:,:,1) =  [1      0.0450  0.0450  0.2058;                 % shapes of UR3
                             1      0.0549  0.0549  0.3265;
                             1      0.0487  0.0487  0.2818;
                             1      0.0315  0.0315  0.0972;
                             1      0.0315  0.0315  0.0947;
                             1      0.0315  0.0315  0.0320];
            
            
                         
            % get list of points that need to be checked (chosen by checkMatrix)
            points = [];
            if ismember("table", checkMatrix)
                points = [points; self.ExtractRobotpoints(self.table)];
            end   
            if ismember("ur3", checkMatrix)
                points = [points; self.ExtractRobotpoints(self.ur3)];
            end       
            
            
            % code to check collisions
            mask = nan(size(points,1),6,size(robot,1));
            for r = 1:size(robot,1)                                         % iterate through every robot
                linkTransform = robot(r).base;                              % initial linkTransform is base of robot
                for l = 1:robot(r).n                                        % iterate through every link of robot
                    linkTransform = linkTransform * trotz(q(r,l)+robot(r).links(l).offset) * transl(0,0,robot(r).links(l).d) * transl(robot(r).links(l).a,0,0) * trotx(robot(r).links(l).alpha);    % get link transform
                    centerTransform(:,:) = linkTransform * transl(OFFSET(l,1,r),OFFSET(l,2,r),OFFSET(l,3,r)) * trotx(OFFSET(l,4,r)) * troty(OFFSET(l,5,r)) * trotz(OFFSET(l,6,r));                  % correct link transforms to center of link
%                     c_h = trplot(centerTransform, 'color', 'r')
                    if SHAPE(l,1,r) == 1                                    % if collision checking is cylindric
                        mask(:,l,r) = self.IsInCylinder(centerTransform,SHAPE(l,2,r),SHAPE(l,4,r),SAFETYDISTANCE,points);   % check collision of points
                    elseif SHAPE(l,1,r) == 2                                % if collision checking is prismatic
                        mask(:,l,r) = self.IsInRectangularPrism(centerTransform,SHAPE(l,2,r),SHAPE(l,3,r),SHAPE(l,4,r),SAFETYDISTANCE,points);  % check collision of points
                    end
                end
            end
            
            % if there is a collision (mask contains "1"), get index of colliding point, index of colliding link and index of colliding robot
            [collidingPoint,collidingLink,collidingRobot] = ind2sub(size(mask),find(mask == 1));
            collisionInfo = [collidingPoint,collidingLink,collidingRobot];  % concatinate indices to matrix
            numberCollidingPoints = size(unique(collisionInfo(:,1)),1);     % count number of colliding points
        end
        
        %% check if any point is inside a cylinder with given centerpose
        function mask = IsInCylinder(~,pose,radius,height,safetyDistance,points)
            pointsInPose = [pose\[points,ones(size(points,1),1)]']';        % update global points to local coordinate frame
            mask = (abs(pointsInPose(:,3)) <= (height/2 + safetyDistance)) & (sqrt(pointsInPose(:,1).^2 + pointsInPose(:,2).^2) <= (radius + safetyDistance)); % conditions for collision
        end
        
        %% check if any point is inside a rectangular prism with given centerpose
        function mask = IsInRectangularPrism(~,pose,x,y,z,safetyDistance,points)
            pointsInPose = [pose\[points,ones(size(points,1),1)]']';        % update global points to local coordinate frame
            mask = (abs(pointsInPose(:,1)) <= (x/2 + safetyDistance)) & (abs(pointsInPose(1,2)) <= (y/2 + safetyDistance)) & (abs(pointsInPose(1,3)) <= (z/2 + safetyDistance));    % conditions for collision
        end
        
        %% extract global points of robot classes
        function points = ExtractRobotpoints(~, robot)
            q = robot.getpos;                                               % get current joint state
            points = [];                                                    % define empty matrix
            for i = 1:robot.n+1                                             % iterate through every robotlink + base
                if i == 1
                    linkTransform = robot.base;                             % use robotbase in first iteration
                else                                                        % calculate joint transforms
                    linkTransform = linkTransform * trotz(q(i-1)+robot.links(i-1).offset) * transl(0,0,robot.links(i-1).d) * transl(robot.links(i-1).a,0,0) * trotx(robot.links(i-1).alpha);
                end

                if ~isempty(robot.points{1,i})                              % if points exist for that joint
                    globalPoints = [linkTransform * [robot.points{1,i},ones(size(robot.points{1,i},1),1)]']';   % determine points in global coordinate frame
                    points = [points; globalPoints(:,1:3)];                 % concatinate calculated points
                end
            end
        end
        
        
    %%% GENERATING TRAJECTORIES %%%
        %% calculate transform trajectory between two poses with lspb
        % this function calculates a lspb trajectory between two poses with a maximum stepsize of the input parameter "stepsize"
        function [transformTrajectory,lspbSteps] = GetLspbTrajectory(~,transform1,transform2,stepsize)
            distance = sqrt(sum((transform1(1:3,4) - transform2(1:3,4)).^2));          % get travel distance
            lspbStepsize = stepsize/distance;                                          % convert stepsize to range [0 1]
            lspbSteps = ceil(1.5/lspbStepsize);                                        % get number of steps for lspb trajectory (inverted speed calculation of lspb function)
            lspbFractionalDistance = lspb(0,1,lspbSteps,lspbStepsize);                 % create lspb trajectory in range [0 1]
            transformTrajectory = ctraj(transform1,transform2,lspbFractionalDistance); % convert lspb to transform trajectory
        end
        
        %% calculate transform trajectory between two poses with quintic polynomial
        % this function calculates a quintic polynomial trajectory between two poses
        function [transformTrajectory,qpSteps] = GetQpTrajectory(~,transform1,transform2,stepsize,speedFactor)
            if nargin < 5
                speedFactor = 1;    % speedFactor enables reduced speeds to approach an object
            end
            distance = sqrt(sum((transform1(1:3,4) - transform2(1:3,4)).^2)); % get travel distance
            qpSteps = ceil(speedFactor * distance/stepsize);                  % get number of steps for quintic polynomial trajectory
            qpFractionalDistance = tpoly(0,1,qpSteps);               % create quintic polynomial trajectory in range [0 1]
            transformTrajectory = ctraj(transform1,transform2,qpFractionalDistance); % convert quintic polynomial to transform trajectory
        end
        
        %% given three corner points of the window (top left, top right, bottom left) calculate the trajectory at the window
        function [spongePath,squeegeePath]=CalculateCleaningPaths(self, stepsize, windowpoints)
            %default window
            if nargin < 3
                windowpoints = [0.5, 0.40,1.0;
                                0.5,-0.20,1.0;
                                0.5, 0.40,0.4];
            end
            %default stepsize
            if nargin < 2
                stepsize = 0.005;    % distance between two transforms in trajectory
            end
            
            
            safetyDistance =  0.05;  % distance between travelling paths and window
            
            
            %%%%% get pose of window %%%%%
            
            width  = sqrt(sum((windowpoints(2,:)' - windowpoints(1,:)').^2));   % width of window
            height = sqrt(sum((windowpoints(3,:)' - windowpoints(1,:)').^2));   % height of window
            disp(['The width of the window  is ', num2str(width), ' m']);
            disp(['The height of the window is ', num2str(height), ' m']);
            
            widthVector  = unit(windowpoints(2,:)' - windowpoints(1,:)');       % unit vector along window width
            heightVector = unit(windowpoints(3,:)' - windowpoints(1,:)');       % unit vector along window height
            approachVector = cross(widthVector, heightVector);                  % plane normal at window
            
            windowCorner(:,:,1) = [widthVector,heightVector,approachVector,windowpoints(1,:)';... % top left corner transform
                                   0          ,0           ,0             ,1];
            windowCorner(:,:,2) = [widthVector,heightVector,approachVector,windowpoints(2,:)';... % top right corner transform
                                   0          ,0           ,0             ,1];
            windowCorner(:,:,3) = [widthVector,heightVector,approachVector,windowpoints(3,:)';... % bottom left corner transform
                                   0          ,0           ,0             ,1];
            windowCorner(:,:,4) = windowCorner(:,:,3) * transl(width,0,0);                        % bottom right corner transform
            
            %plot window corners
            hold on;
            for i=1:4
                trplot(windowCorner(:,:,i), 'color', 'k', 'length', 0.1);
            end
            axis([0.4 0.6 -0.1 1.1 0.4 2.1]);
            axis equal;
            
               
            %%%%% path of sponge %%%%%
            
            spongeWidth    =  0.25;  % measurements of sponge
            spongeLength   =  0.1;
                                    
            % 1. approach window
            step=1;
            startPos = windowCorner(:,:,1) * transl(spongeLength/2, spongeWidth/2, -safetyDistance); % starting position of cleaning path
            endPos = startPos * transl(0, 0, safetyDistance); % first position at window
                        
            spongePath(:,:,step) = startPos;    % first pose of trajectory equals starting position
            step = step+1;                      % increment step counter
            
            [transformTrajectory,qpSteps] = self.GetQpTrajectory(startPos,endPos,stepsize,4); % get trajectory to approach window
            spongePath(:,:,step:(qpSteps+step-1)) = transformTrajectory;
            step=qpSteps+step;            % update step counter
                        
            % 2. cleaning path in s-shape
            % acceleration
            s = spongeWidth-spongeLength/2;                       % distance to accelerate
            v = stepsize/1;                                       % speed to accelerate to (it's the unit stepsize speed from path)
            a = 0.5 * (v^2)/s;                                    % acceleration needed
            timeSteps = sqrt(2*(s/a));                            % number of steps to accelerate
            for t = 1:timeSteps
                distance = 0.5*a*(t^2 - (t-1)^2);   % acceleration distances
                spongePath(:,:,step) = spongePath(:,:,step-1) * transl(distance, 0, 0); % trajectory of acceleration
                step = step+1;              % increment step count
            end            
            
            % s-shaped path
            widthSteps = floor((width-2*spongeWidth)/stepsize);   % number of straight increments between turns (width)
            heightSteps= floor((height-spongeWidth)/spongeWidth); % number of lines (height)
            supposedAngle = 2*asin((stepsize/2)/(spongeWidth/2)); % optimal angle the sponge has to rotate in turns (constant velocity)
            angleSteps = floor(pi/supposedAngle);                 % number of turn steps per turn (rounded down to whole number)
            angle = pi/angleSteps;                                % "real" turn angle
            turn = 1;                                             % direction of first turn ("+1" for right turn, "-1" for left turn)
            
            for i = 1:heightSteps           % iterate through lines
                for j = 1:widthSteps        % iterate through straight parts
                    spongePath(:,:,step) = spongePath(:,:,step-1) * transl(stepsize, 0, 0); % move transform one stepsize further straight
                    step = step+1;          % increment step count
                end
                                               
                for k = 1:angleSteps        % iterate through turns
                    spongePath(:,:,step) = spongePath(:,:,step-1) * trotz(turn * angle/2) * transl(stepsize, 0, 0) * trotz(turn * angle/2); 
                                                                                            % move transform one segment along the turn (symmetric)
                    step = step+1;          % increment step count
                end
                turn = turn * -1;           % change direction of next turn
            end
            
            for j = 1:widthSteps            % repeat straight part iteration for final straight
                spongePath(:,:,step) = spongePath(:,:,step-1) * transl(stepsize, 0, 0); % move transform one stepsize further
                step = step+1;              % increment step count
            end
            
            % deceleration
            for t = 1:timeSteps
                distance = stepsize - 0.5*a*(t^2 - (t-1)^2);    % deceleration fractional steps
                spongePath(:,:,step) = spongePath(:,:,step-1) * transl(distance, 0, 0); % deceleration trajectory
                step = step+1;              % increment step count
            end            
            
            % 3. lift sponge from window
            startPos = spongePath(:,:,step-1);
            endPos = spongePath(:,:,step-1) * transl(0,0,-safetyDistance);
            [transformTrajectory,qpSteps] = self.GetQpTrajectory(startPos,endPos,stepsize,4); % get trajectory to lift off window
            spongePath(:,:,step:(qpSteps+step-1)) = transformTrajectory;
            step=qpSteps+step;            % update step counter
            
            % 4. move to top right corner
            startPos = spongePath(:,:,step-1);  % get previous transform as reference
            endPos = windowCorner(:,:,2) * transl(-spongeWidth/2,spongeLength/2,-safetyDistance) * trotz(pi/2); % new pose (sponge horizontal in top right corner)
            [transformTrajectory,lspbSteps] = self.GetLspbTrajectory(startPos,endPos,stepsize); % get lspb trajectory
            spongePath(:,:,step:(lspbSteps+step-1)) = transformTrajectory;
            step = lspbSteps+step;
            
            % 5. clean edges
            for i = 1:3
                switch i
                    case 1
                        cornerPos = windowCorner(:,:,4) * transl(-spongeWidth/2,-spongeLength/2,0) * trotz(pi/2);
                        finalPos  = windowCorner(:,:,4) * transl(-spongeLength/2,-spongeWidth/2,-safetyDistance) * trotz(pi);
                    case 2
                        cornerPos = windowCorner(:,:,3) * transl(spongeLength/2,-spongeWidth/2,0) * trotz(pi);
                        finalPos  = windowCorner(:,:,3) * transl(spongeWidth/2,-spongeLength/2,-safetyDistance) * trotz(3*pi/2);
                    case 3
                        cornerPos = windowCorner(:,:,1) * transl(spongeWidth/2,spongeLength/2,0) * trotz(3*pi/2);
                end
                
                % approach to window
                endPos = spongePath(:,:,step-1)*transl(0,0,safetyDistance);
                [transformTrajectory,qpSteps] = self.GetQpTrajectory(spongePath(:,:,step-1),endPos,stepsize,4); % get quintic polynomial trajectory
                spongePath(:,:,step:(qpSteps+step-1)) = transformTrajectory;
                step=qpSteps+step;            % update step counter
                % move to bottom right edge
                endPos = cornerPos;
                [transformTrajectory,lspbSteps] = self.GetLspbTrajectory(spongePath(:,:,step-1),endPos,stepsize); % get lspb trajectory
                spongePath(:,:,step:(lspbSteps+step-1)) = transformTrajectory;
                step = lspbSteps+step;        % update step counter
                % lift off window
                endPos = endPos * transl(0,0,-safetyDistance);
                [transformTrajectory,qpSteps] = self.GetQpTrajectory(spongePath(:,:,step-1),endPos,stepsize,4); % get quintic polynomial trajectory
                spongePath(:,:,step:(qpSteps+step-1)) = transformTrajectory;
                step=qpSteps+step;            % update step counter
                % turn for next edge (after third edge no turn needed)
                if i ~= 3
                    endPos = finalPos;
                    [transformTrajectory,qpSteps] = self.GetQpTrajectory(spongePath(:,:,step-1),endPos,stepsize,4); % get quintic polynomial trajectory
                    spongePath(:,:,step:(qpSteps+step-1)) = transformTrajectory;
                    step=qpSteps+step;            % update step counter
                end
            end
            
            % plot trajectory
            plot = 0;
            if plot==1
                for i = 1:size(spongePath,3)
                    %trplot(spongePath(:,:,i),'length',0.05, 'color', 'b');
                    plot3(spongePath(1,4,i),spongePath(2,4,i),spongePath(3,4,i),'b.');
%                     pause(0.0001);
                end
            end
            
            
            %%%%% path of squeegee %%%%%
            
            squeegeeWidth    =  0.10;  % measurements of squeegee
            squeegeeLength   =  0.06;
            
            step=1;
            startPos = windowCorner(:,:,1) * transl(squeegeeWidth/2, squeegeeLength/2, -safetyDistance) * trotz(pi/2);    % first pose of trajectory at top left corner
            squeegeePath(:,:,step) = startPos;
            step = step+1;                      % increment step counter
            
            widthSteps = floor(width/squeegeeWidth);
            for i = 1:widthSteps+1
                % approach window
                endPos = squeegeePath(:,:,step-1) * transl(0,0,safetyDistance);
                [transformTrajectory,qpSteps] = self.GetQpTrajectory(squeegeePath(:,:,step-1),endPos,stepsize,4);    % get quintic polynomial trajectory
                squeegeePath(:,:,step:(qpSteps+step-1)) = transformTrajectory;
                step=qpSteps+step;            % update step counter
                % move down
                endPos = squeegeePath(:,:,step-1) * transl(height-squeegeeLength,0,0);
                [transformTrajectory,lspbSteps] = self.GetQpTrajectory(squeegeePath(:,:,step-1),endPos,stepsize,1); % get lspb trajectory     %here
                squeegeePath(:,:,step:(lspbSteps+step-1)) = transformTrajectory;
                step = lspbSteps+step;        % update step counter
                % lift off window
                endPos = squeegeePath(:,:,step-1) * transl(0,0,-safetyDistance);
                [transformTrajectory,qpSteps] = self.GetQpTrajectory(squeegeePath(:,:,step-1),endPos,stepsize,4);    % get quintic polynomial trajectory
                squeegeePath(:,:,step:(qpSteps+step-1)) = transformTrajectory;
                step=qpSteps+step;            % update step counter
                % move to next line
                if i < widthSteps % move next to previos line
                    endPos = startPos * transl(0,-(i*squeegeeWidth),0);
                    [transformTrajectory,lspbSteps] = self.GetQpTrajectory(squeegeePath(:,:,step-1),endPos,stepsize,1); % get lspb trajectory   %here
                    squeegeePath(:,:,step:(lspbSteps+step-1)) = transformTrajectory;
                    step = lspbSteps+step;        % update step counter
                elseif i < widthSteps+1           % move to right edge if not already done
                    if width/squeegeeWidth > widthSteps
                        endPos = windowCorner(:,:,2) * transl(-squeegeeWidth/2, squeegeeLength/2, -safetyDistance) * trotz(pi/2);
                        [transformTrajectory,lspbSteps] = self.GetQpTrajectory(squeegeePath(:,:,step-1),endPos,stepsize,1); % get lspb trajectory   %here
                        squeegeePath(:,:,step:(lspbSteps+step-1)) = transformTrajectory;
                        step = lspbSteps+step;        % update step counter
                    end
                end
            end
            
            % plot trajectory
            plot = 0;
            if plot == 1
                for i = 1:size(squeegeePath,3)
                    %trplot(squeegeePath(:,:,i),'length',0.05, 'color', 'r');
                    plot3(squeegeePath(1,4,i),squeegeePath(2,4,i),squeegeePath(3,4,i),'r.');
%                     pause(0.0001);
                end
            end
        end
        
        %% generate a transform trajectory from given waypoints with lspb
%         function transformTrajectory = GetWaypointTrajectory(~, transforms, steps)
%             waypoints = nan(6,size(transforms,3));
%             for i = 1:size(transforms,3)
%                 waypoints(:,i) = [transforms(1:3,4,i); tr2rpy(transforms(1:3,1:3,i))'];
%             end 
%             
%             travelpoints = trapveltraj(waypoints,steps);
%             
%             transformTrajectory = nan(4,4,steps);
%             for i = 1:steps
%                 transformTrajectory(:,:,i) = [rpy2r(travelpoints(4:6,i)'), travelpoints(1:3,i);
%                                               0, 0, 0, 1];
%             end
%         end
    end
end

