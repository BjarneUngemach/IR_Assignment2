classdef Calculations < handle
    % class contains the big calculations for SwipeBot.m
    
    
    methods
    %%% RMRC SOLVER %%%
        %% get a joint state Matrix with a Resolved Motion Rate Control
        function [qMatrix,steps,error] = SolveRMRC(self, robot, trajectory, qStart, stepsize, speed, offset)
            if nargin < 7
                offset = eye(4);
            end
            
            %%% CONSTANTS %%%
            WEIGHT_MATRIX = diag([1 1 1 1 1 1]);   % Matrix of gains for RMRC
            THRESHOLD = 0.001;                           % threshold for DLS use
            MAX_DAMPING = 0.01;                          % max damping factor of DLS
            
            
            deltaT = stepsize/speed;                    % get timestep for wanted velocity
            steps = size(trajectory,3);                 % get number of steps in trajectory
            
            % allocate arrays
            qMatrix = nan(steps,6);                     % get empty matrix
            error = nan(steps,1);                       % get empty position error list
            
            % offset trajectory with gripper- and tool-length
            trajectory = pagemtimes(trajectory, offset);
            
            
            qMatrix(1,:) = qStart;                                                              % solve first qMatrix with inverse kinematics
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
                robot = [self.ur3; self.meca500];                 % pack all robots in an array
            end    
            if nargin < 4
                for r = 1:size(robot,1)
                    q(r,:) = robot(r).getpos;       % get current joint state for every robot
                end
            end
            
            %%% CONSTANTS %%%
            SAFETYDISTANCE = 0.01;
            % offset of center of link from joint transform "[x y z r p y]"
            OFFSET_UR3 =    [0      -0.0266  0      -pi/2    0       0;      % offsets of UR3
                             0.1220  0       0.1199 -pi/2   -pi/2    0;
                             0.1077  0       0.0208 -pi/2   -pi/2    0;
                             0       0       0.0032  0       0      -pi;
                             0       0       0.0027  0       0      -pi;
                             0       0      -0.0358  0       0      -pi];
            OFFSET_MECA500= [0      -0.0518  0      -pi/2    0       0;     % offsets of Meca500
                            -0.0698  0       0       0       pi/2    0;
                             0       0       0.0168  0       0       0;
                             0       0.0288  0       pi/2    0       0;
                             0       0       0.0186  0       0       0;
                             0       0      -0.0053  0       0       0];
            % shape matrix defines geometry of collision checking cage (first collumn defines shape: "1" for cylinder, "2" for prism) 
            SHAPE_UR3    =  [1      0.0450  0.0450  0.2058;                 % shapes of UR3
                             1      0.0549  0.0549  0.3265;
                             1      0.0487  0.0487  0.2818;
                             1      0.0315  0.0315  0.0972;
                             1      0.0315  0.0315  0.0947;
                             1      0.0315  0.0315  0.1395];
            SHAPE_MECA500=  [1      0.1000  0.1000  0.1675;
                             2      0.0999  0.0685  0.1945;
                             2      0.0640  0.0450  0.0885;
                             2      0.0450  0.0660  0.0800;
                             2      0.0450  0.0460  0.0810;
                             1      0.0305  0.0305  0.0105];
            % copy constants into offset
            if nargin < 3
                OFFSET(:,:,1) = OFFSET_UR3;
                OFFSET(:,:,2) = OFFSET_MECA500;
                SHAPE(:,:,1) = SHAPE_UR3;
                SHAPE(:,:,2) = SHAPE_MECA500;
            elseif robot == self.ur3
                OFFSET(:,:,1) = OFFSET_UR3;
                SHAPE(:,:,1) = SHAPE_UR3;
            elseif robot == self.meca500
                OFFSET(:,:,1) = OFFSET_MECA500;
                SHAPE(:,:,1) = SHAPE_MECA500;
            end
                             
                                   
            % get list of points that need to be checked (chosen by checkMatrix)
            points = [];
            if ismember("table", checkMatrix)
                points = [points; self.ExtractRobotpoints(self.table)];
            end   
            if ismember("ur3", checkMatrix)
                points = [points; self.ExtractRobotpoints(self.ur3)];
            end
            if ismember("meca500", checkMatrix)
                points = [points; self.ExtractRobotpoints(self.meca500)];
            end
            if ismember("hand", checkMatrix)
                if ~isempty(findobj('Tag', self.hand.name))
                    points = [points; self.ExtractRobotpoints(self.hand)];
                else
                    points = [points; [10 0 0]];
                end
            end
            
            
            % code to check collisions
            mask = nan(size(points,1),6,size(robot,1));
            for r = 1:size(robot,1)                                         % iterate through every robot
                linkTransform = robot(r).base;                              % initial linkTransform is base of robot
                for l = 1:robot(r).n                                        % iterate through every link of robot
                    linkTransform = linkTransform * trotz(q(r,l)+robot(r).links(l).offset) * transl(0,0,robot(r).links(l).d) * transl(robot(r).links(l).a,0,0) * trotx(robot(r).links(l).alpha);    % get link transform
                    %c_h = trplot(linkTransform, 'color', 'b')
                    centerTransform(:,:) = linkTransform * transl(OFFSET(l,1,r),OFFSET(l,2,r),OFFSET(l,3,r)) * trotx(OFFSET(l,4,r)) * troty(OFFSET(l,5,r)) * trotz(OFFSET(l,6,r));                  % correct link transforms to center of link
                    %c_h = trplot(centerTransform, 'color', 'r')
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
        
        %% check if something is near the swipebot system
        function numberCollidingPoints = CheckWorkspace(self)
            sphereCenter = self.table.base * transl(0,0,0.42);
            radius = 0.8;
            safetyDistance = 0;
            
            if ~isempty(findobj('Tag', self.hand.name))
                points = self.ExtractRobotpoints(self.hand);
            else
                points = [10 0 0];
            end
            
            numberCollidingPoints = self.IsInEllipsoid(sphereCenter,radius,radius,radius,safetyDistance,points);
        end
        
        %% check if any point is inside a cylinder with given centerpose
        function mask = IsInCylinder(~,pose,radius,height,safetyDistance,points)
            pointsInPose = [pose\[points,ones(size(points,1),1)]']';        % update global points to local coordinate frame
            mask = (abs(pointsInPose(:,3)) <= (height/2 + safetyDistance)) & (sqrt(pointsInPose(:,1).^2 + pointsInPose(:,2).^2) <= (radius + safetyDistance)); % conditions for collision
        end
        
        %% check if any point is inside a rectangular prism with given centerpose
        function mask = IsInRectangularPrism(~,pose,x,y,z,safetyDistance,points)
            pointsInPose = [pose\[points,ones(size(points,1),1)]']';        % update global points to local coordinate frame
            mask = (abs(pointsInPose(:,1)) <= (x/2 + safetyDistance)) & (abs(pointsInPose(:,2)) <= (y/2 + safetyDistance)) & (abs(pointsInPose(:,3)) <= (z/2 + safetyDistance));    % conditions for collision
        end
        
        %% check if any point is inside a ellipsoid
        function mask = IsInEllipsoid(~,pose,rx,ry,rz,safetyDistance,points)
            pointsInPose = [pose\[points,ones(size(points,1),1)]']';
            mask = (((pointsInPose(:,1)+safetyDistance)/rx).^2 + ((pointsInPose(:,2)+safetyDistance)/ry).^2 + ((pointsInPose(:,3)+safetyDistance)/rz).^2) <= 1;
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
        function [spongePath,squeegeePath] = CalculateCleaningPaths(self, app, stepsize, windowpoints)
            %default window
            if nargin < 4
                windowpoints = [0.65, 0.45,0.9;
                                0.65,-0.20,0.9;
                                0.65, 0.45,0.4];
            end
            %default stepsize
            if nargin < 3
                stepsize = 0.005;    % distance between two transforms in trajectory
            end
            
            
            safetyDistance =  0.05;  % distance between travelling paths and window
            
            
            %%%%% get pose of window %%%%%
            
            width  = sqrt(sum((windowpoints(2,:)' - windowpoints(1,:)').^2));   % width of window
            height = sqrt(sum((windowpoints(3,:)' - windowpoints(1,:)').^2));   % height of window
            disp(['The width of the window  is ', num2str(width), ' m']);
            disp(['The height of the window is ', num2str(height), ' m']);
            str1 = sprintf('The width of the window is %.3f m',width);
            str2 = sprintf('The height of the window is %.3f m',height);
            app.TextArea.Value = [app.TextArea.Value;
                                  " ";
                                  str1;
                                  str2];
            drawnow;
            
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
            window = [windowpoints(1,:);
                      windowpoints(2,:);
                      windowpoints(2,1:2), windowpoints(3,3);
                      windowpoints(3,:);
                      windowpoints(1,:)];
%             for i=1:4
                plot3(window(:,1),window(:,2),window(:,3), '-*r');
%             end
            axis equal;
            
               
            %%%%% path of sponge %%%%%
            
            spongeWidth    =  0.2;  % measurements of sponge
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
            heightSteps= floor((height)/spongeWidth); % number of lines (height)
            supposedAngle = 2*asin((stepsize/2)/(spongeWidth/2)); % optimal angle the sponge has to rotate in turns (constant velocity)
            angleSteps = floor(pi/supposedAngle);                 % number of turn steps per turn (rounded down to whole number)
            angle = pi/angleSteps;                                % "real" turn angle
            turn = 1;                                             % direction of first turn ("+1" for right turn, "-1" for left turn)
            
            for i = 1:heightSteps-1           % iterate through lines
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
            
            squeegeeWidth    =  0.2;  % measurements of squeegee
            squeegeeLength   =  0.03;
            
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
                [transformTrajectory,lspbSteps] = self.GetLspbTrajectory(squeegeePath(:,:,step-1),endPos,stepsize);  % get lspb trajectory
                squeegeePath(:,:,step:(lspbSteps+step-1)) = transformTrajectory;
                step = lspbSteps+step;        % update step counter
                % lift off window
                endPos = squeegeePath(:,:,step-1) * transl(0,0,-safetyDistance);
                [transformTrajectory,qpSteps] = self.GetQpTrajectory(squeegeePath(:,:,step-1),endPos,stepsize,4);    % get quintic polynomial trajectory
                squeegeePath(:,:,step:(qpSteps+step-1)) = transformTrajectory;
                step=qpSteps+step;            % update step counter
                % move to next line
                if i < widthSteps % move to next line
                    endPos = startPos * transl(0,-(i*squeegeeWidth),0);
                    [transformTrajectory,lspbSteps] = self.GetLspbTrajectory(squeegeePath(:,:,step-1),endPos,stepsize);% get lspb trajectory
                    squeegeePath(:,:,step:(lspbSteps+step-1)) = transformTrajectory;
                    step = lspbSteps+step;        % update step counter
                elseif i < widthSteps+1           % move to right edge if not already done
                    if width/squeegeeWidth > widthSteps
                        endPos = windowCorner(:,:,2) * transl(-squeegeeWidth/2, squeegeeLength/2, -safetyDistance) * trotz(pi/2);
                        [transformTrajectory,lspbSteps] = self.GetLspbTrajectory(squeegeePath(:,:,step-1),endPos,stepsize); % get lspb trajectory
                        squeegeePath(:,:,step:(lspbSteps+step-1)) = transformTrajectory;
                        step = lspbSteps+step;        % update step counter
                    end
                end
            end
%             % move back to top left corner
%             endPos = startPos;
%             [transformTrajectory,lspbSteps] = self.GetLspbTrajectory(squeegeePath(:,:,step-1),endPos,stepsize);% get lspb trajectory
%             squeegeePath(:,:,step:(lspbSteps+step-1)) = transformTrajectory;
%             step = lspbSteps+step;        % update step counter
                
                
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
        
        %% move UR3 to the starting position at the window
        function [qMatrix1, steps] = TravelUR3(self,mode,path,qStart,steps,cycle)
            if nargin < 6
                cycle = 1;
            end
            % the variable "cycle" decides which waypoints are used. If you
            % call the function you should always use cycle = 1 or don't input any value. 
            % If the system recognizes a collision, it runs the same function
            % again with the alternative waypoints in cycle2
            
            % waypoint for trajectory
            waypoint = self.waypointUR3;
                
            if mode == "toWindow"
                if cycle == 1   % robot can reach its goal traditionally
                    % waypoints as joint states
                    q(1,:) = qStart;
                    q(2,:) = self.ur3.ikcon(waypoint,self.qUR3Home);
                    q(3,:) = self.ur3.ikcon(path(:,:,1)*(self.gripperUR3offset * self.toolUR3offset),q(2,:));

                elseif cycle == 2 % robot must avoid an obstacle
                    % waypoints as joint states
                    q(1,:) = qStart;
                    q(2,:) = deg2rad([-270 -90 -140 50 90 0]);
                    q(3,:) = self.ur3.ikcon(path(:,:,1)*(self.gripperUR3offset * self.toolUR3offset),q(2,:));
                end
            elseif mode == "fromWindow"
                if cycle == 1   % robot can reach its goal traditionally
                    % waypoints as joint states
                    q(1,:) = qStart;
                    q(2,:) = self.ur3.ikcon(waypoint,self.qUR3Home);
                    q(3,:) = self.qUR3Home;

                elseif cycle == 2 % robot must avoid an obstacle
                    % waypoints as joint states
                    q(1,:) = qStart;
                    q(2,:) = deg2rad([-270 -90 -140 50 90 0]);
                    q(3,:) = self.qUR3Home;
                end
            end
            
            partSteps = floor(steps/(size(q,1)-1));         % split number of steps between waypoints
            qMatrix1 = nan(partSteps*(size(q,1)-1),6);     % allocate empty array
            
            
            % iterate through waypoints and get lspb trajectory
            for i=1:size(q,1)-1
                s = lspb(0,1,partSteps);                    % linear segment wit parabolic blend
                for j = 1:partSteps
                    qMatrix1(((i-1)*partSteps+j),:) = (1-s(j))*q(i,:) + s(j)*q(i+1,:);  % get qMatrix with lspb
                end
            end
            
            % check for collisions
            if cycle == 1   % just used for initial cycle
                if ~isempty(findobj('Tag', self.hand.name))
                    for i = 1:10:(size(q,1)-1)*partSteps    % take every 10th step
                        numberOfCollidingPoints = CheckCollision(self, "hand", self.ur3, qMatrix1(i,:));   % call collision checking function
                        if numberOfCollidingPoints > 0
                            qMatrix1 = self.TravelUR3(mode,path,qStart,steps,2);   % if collision recognized call function again with cycle = 2
                            return;
                        end
                    end
                end
            end
        end
        
        %% move Meca500 to bring the tools to the UR3
        function [qMatrix, steps] = MoveMeca500(self, tool, mode)
            toolChange = self.toolChangeTr * self.gripperMeca500offset;
            waitingPos = toolChange * transl(0,0,-0.1);
            % waypoints for sponge
            if tool == "sponge"
                spongePickup = self.spongeHome * self.gripperMeca500offset;
                spongePrePickup = spongePickup * transl(0, 0, -0.04);
                if mode == "toTool"
                    % waypoints
                    q(1,:) = self.qMeca500Home;
                    q(2,:) = deg2rad([0 -60 -60 0 110 0]);
                    q(3,:) = self.meca500.ikcon(spongePrePickup,q(2,:));
                    q(4,:) = self.meca500.ikcon(spongePickup,q(3,:));
                    % steps
                    step(1) = 50;
                    step(2) = 20;
                    step(3) = 20;
                end
                if mode == "fromTool"
                    % waypoints
                    q(1,:) = self.meca500.getpos;
                    q(2,:) = self.meca500.ikcon(spongePrePickup,q(1,:));
                    % steps
                    step(1) = 20;
                end
                if mode == "toUR3"
                    % waypoints
                    q(1,:) = self.meca500.getpos;
                    q(2,:) = deg2rad([0 16 -40 0 26 180]);
                    q(3,:) = self.meca500.ikcon(toolChange,deg2rad([0 -18 18 0 0 180]));
                    % steps
                    step(1) = 30;
                    step(2) = 10;
                end
                if mode == "fromUR3"
                    % waypoints
                    q(1,:) = self.meca500.getpos;
                    q(2,:) = deg2rad([0 16 -40 0 26 180]);
                    q(3,:) = self.meca500.ikcon(spongePickup,[0.8297 -1.2568 -0.5634 0.8452 1.7382 0.1853]);
                    % steps
                    step(1) = 30;
                    step(2) = 30;
                end
                if mode == "toWait"
                    % waypoints
                    q(1,:) = self.meca500.getpos;
                    q(2,:) = self.meca500.ikcon(waitingPos,deg2rad([0 0 0 0 0 180]));
                    % steps 
                    step(1) = 30;
                end
                if mode == "fromWait"
                    % waypoints
                    q(1,:) = self.meca500.getpos;
                    q(2,:) = self.meca500.ikcon(toolChange,deg2rad([0 -18 18 0 0 180]));
                    % steps 
                    step(1) = 30;
                end
            end
            
            % waypoints for squeegee
            if tool == "squeegee"
                squeegeePickup = self.squeegeeHome * self.gripperMeca500offset;
                squeegeePrePickup = squeegeePickup * transl(0, 0, -0.04);
                if mode == "toTool"
                    % waypoints
                    q(1,:) = self.meca500.getpos;
                    q(2,:) = self.meca500.ikcon(squeegeePrePickup,deg2rad([0 -60 -60 0 110 0]));
                    q(3,:) = self.meca500.ikcon(squeegeePickup,q(2,:));
                    % steps
                    step(1) = 30;
                    step(2) = 20;
                end
                if mode == "fromTool"
                    % waypoints
                    q(1,:) = self.meca500.getpos;
                    q(2,:) = self.meca500.ikcon(squeegeePrePickup,q(1,:));
                    q(3,:) = deg2rad([0 -60 -60 0 110 0]);
                    q(4,:) = self.qMeca500Home;
                    
                    % steps
                    step(1) = 20;
                    step(2) = 20;
                    step(3) = 40;
                end
                if mode == "toUR3"
                    % waypoints
                    q(1,:) = self.meca500.getpos;
                    q(2,:) = deg2rad([0 16 -40 0 26 -180]);
                    q(3,:) = self.meca500.ikcon(toolChange,deg2rad([0 -18 18 0 0 -180]));
                    % steps
                    step(1) = 30;
                    step(2) = 10;
                end
                if mode == "fromUR3"
                    % waypoints
                    q(1,:) = self.meca500.getpos;
                    q(2,:) = deg2rad([0 16 -40 0 26 -180]);
                    q(3,:) = self.meca500.ikcon(squeegeePickup,[-0.8297 -1.2568 -0.5634 -0.8452 1.7382 -0.1853]);
                    % steps
                    step(1) = 30;
                    step(2) = 30;
                end
                if mode == "toWait"
                    % waypoints
                    q(1,:) = self.meca500.getpos;
                    q(2,:) = self.meca500.ikcon(waitingPos,deg2rad([0 0 0 0 0 -180]));
                    % steps 
                    step(1) = 30;
                end
                if mode == "fromWait"
                    % waypoints
                    q(1,:) = self.meca500.getpos;
                    q(2,:) = self.meca500.ikcon(toolChange,deg2rad([0 -18 18 0 0 -180]));
                    % steps 
                    step(1) = 30;
                end
            end
            
            qMatrix = [];
            steps = 0;
            
            % itareate trough waypoints
            for i = 1:size(q,1)-1
                qMatrix = [qMatrix; jtraj(q(i,:),q(i+1,:),step(i))];
                steps = steps + step(i);
            end
        end
        
        %% move UR3 to get the tools from Meca500
        function [qMatrix, steps] = MoveUR3(self, mode)
            if mode == "grabTool"
                steps1 = 10;
                qMatrix1 = jtraj(self.qUR3Home, [self.qUR3Home(1,1:5), -pi/2], steps1);
                trTrajToTool = ctraj(self.trUR3Home*trotz(-pi/2), self.toolChangeTr*self.gripperUR3offset, 30);
                [qMatrix2,steps2] = self.SolveRMRC(self.ur3, trTrajToTool, [self.qUR3Home(1,1:5), -pi/2], 0.01, 1);
                
                qMatrix = [qMatrix1; qMatrix2];
                steps = steps1+steps2;
            end
            if mode == "getTool"
                steps1 = 10;
                q = self.ur3.getpos;
                qMatrix1 = jtraj(q, [q(1,1:5), 0], steps1);
                trTrajToTool = ctraj(self.toolChangeTr*self.gripperUR3offset*trotz(pi/2), self.trUR3Home,  30);
                [qMatrix2,steps2] = self.SolveRMRC(self.ur3, trTrajToTool, [q(1,1:5), 0], 0.01, 1);
                
               
                qMatrix = [qMatrix1; qMatrix2];
                steps = steps1+steps2;
            end     
            if mode == "bringTool"
                trTrajToTool = ctraj(self.trUR3Home,  self.toolChangeTr*self.gripperUR3offset*trotz(pi/2), 30);
                [qMatrix1,steps1] = self.SolveRMRC(self.ur3, trTrajToTool, self.qUR3Home, 0.01, 1);
                steps2 = 10;
                q = self.ur3.ikcon(self.toolChangeTr*self.gripperUR3offset*trotz(pi/2), self.qUR3Home);
                qMatrix2 = jtraj(q, [q(1,1:5), -pi/2], steps2);
                
                
               
                qMatrix = [qMatrix1; qMatrix2];
                steps = steps1+steps2;
            end
            if mode == "releaseTool"
                trTrajToTool = ctraj(self.toolChangeTr*self.gripperUR3offset, self.trUR3Home*trotz(-pi/2), 30);
                [qMatrix1,steps1] = self.SolveRMRC(self.ur3, trTrajToTool, self.ur3.getpos, 0.01, 1);
                steps2 = 10;
                qMatrix2 = jtraj([self.qUR3Home(1,1:5), -pi/2], self.qUR3Home, steps2);
                
                
                qMatrix = [qMatrix1; qMatrix2];
                steps = steps1+steps2;
            end     
        end
    end
end

