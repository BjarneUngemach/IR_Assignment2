classdef Calculations < handle
    % class contains the big calculations for SwipeBot.m
    
    
    methods
        %% collision checking
        function CheckCollision(self)
            robot = [self.ur3];
            offset(:,:,1) = [0      -0.0266 0       -pi/2   0       0;
                             0.1220 0       0.1199  -pi/2   -pi/2   0;
                             0.1077 0       0.0208  -pi/2   -pi/2   0;
                             0      0       0.0032  0       0       -pi;
                             0      0       0.0027  0       0       -pi;
                             0      0       -0.0160 0       0       -pi];
            shape(:,:,1) = ["c"     0.0450  0.0450  0.2058;
                            "c"     0.0549  0.0549  0.3265;
                            "c"     0.0487  0.0487  0.2818;
                            "c"     0.0315  0.0315  0.0972;
                            "c"     0.0315  0.0315  0.0947;
                            "c"     0.0315  0.0315  0.0320];
            
            safetyDistance = 0.01;
            
points = [1 1 1;
          0 1 0];
            
            
            for r = 1:size(robot,1) %iterate through every robot
                q = robot(r).getpos;
                linkTransform = robot(r).base;
                for l = 1:robot(r).n %iterate through every link
                    linkTransform = linkTransform * trotz(q(l)+robot(r).links(l).offset) * transl(0,0,robot(r).links(l).d) * transl(robot(r).links(l).a,0,0) * trotx(robot(r).links(l).alpha);
                    centerTransform(:,:) = linkTransform * transl(offset(l,1,r),offset(l,2,r),offset(l,3,r)) * trotx(offset(l,4,r)) * troty(offset(l,5,r)) * trotz(offset(l,6,r));
%                     c_h = trplot(centerTransform, 'color', 'r')
                    if shape(l,1,r) == "c"
                        mask = self.IsInCylinder(centerTransform,shape(l,2,r),shape(l,4,r),safetyDistance,points)
                    elseif shape(l,1,r) == "p"
                        mask = self.IsInCylinder(centerTransform,shape(l,2,r),shape(l,3,r),shape(l,4,r),safetyDistance,points)
                    end
                end
            end
        end
        
        % get transforms of robot links
        function linkTransforms = getLinkTransforms(~, robot, q)
            linkTransforms = zeros(4,4,robot.n+1);

            linkTransforms(:,:,1) = robot.base;
            for i = 2:robot.n+1
                linkTransforms(:,:,i) = linkTransforms(:,:,i-1) * trotz(q(i-1)+robot.links(i-1).offset) * transl(0,0,robot.links(i-1).d) * transl(robot.links(i-1).a,0,0) * trotx(robot.links(i-1).alpha);
            end
        end
        
        % check if any point is inside a cylinder
        function mask = IsInCylinder(~,pose,radius,height,safetyDistance,points)
            pointsInPose = zeros(size(points,1),3);
            mask = zeros(size(points,1),1);
            for i = 1:size(points,1)
                 transform = inv(pose) * transl(points(i,1),points(i,2),points(i,3));
                 pointsInPose(i,:) = transform(1:3,4)';
            end
            for i = 1:size(pointsInPose,1)
                if (abs(pointsInPose(1,3)) <= (height/2 + safetyDistance))...
                && (sqrt(pointsInPose(i,1)^2 + pointsInPose(i,2)^2) <= (radius + safetyDistance))
                    mask(i,1) = 1;
                end                    
            end
        end
        
        % check if any point is inside a rectangular prism
        function mask = IsInRectangularPrism(~,pose,x,y,z,safetyDistance,points)
            pointsInPose = zeros(size(points,1),3);
            mask = zeros(size(points,1),1);
            for i = 1:size(points,1)
                 transform = inv(pose) * transl(points(i,1),points(i,2),points(i,3));
                 pointsInPose(i,:) = transform(1:3,4)';
            end
            for i = 1:size(pointsInPose,1)
                if (abs(pointsInPose(1,1)) <= (x/2 + safetyDistance))...
                && (abs(pointsInPose(1,2)) <= (y/2 + safetyDistance))...
                && (abs(pointsInPose(1,3)) <= (z/2 + safetyDistance))
                    mask(i,1) = 1;
                end                    
            end
        end
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
                windowpoints = [0.5, 0.40,0.9;...
                                0.5,-0.20,0.9;...
                                0.5, 0.40,0.6];
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
            
            spongeWidth    =  0.1;  % measurements of sponge
            spongeLength   =  0.03;
                                    
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
            plot = 1;
            if plot==1
                for i = 1:size(spongePath,3)
                    %trplot(spongePath(:,:,i),'length',0.05, 'color', 'b');
                    plot3(spongePath(1,4,i),spongePath(2,4,i),spongePath(3,4,i),'b.');
%                     pause(0.0001);
                end
            end
            
            
            %%%%% path of squeegee %%%%%
            
            squeegeeWidth    =  0.2;  % measurements of squeegee
            squeegeeLength   =  0.01;
            
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
    end
end
