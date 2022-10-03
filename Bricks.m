classdef Bricks
    % generates bricks and determins the brick position in the wall
    %% Information about the brick
    % loaded model: x max = 0.0319  x min = -0.0348  x_length = 0.0667
    %               y max = 0.0710  y min = -0.0624  y_length = 0.1334
    %               z max = 0.0366  z min =  0.0033  z_length = 0.0333
    
    properties
        startPos;
        wallPos;
        brick;
        
        % resulting dimensions
        width = 0.0667;
        length = 0.1334;
        hight = 0.0333;
        
        % offset of brick center point (middle bottom)
        xoff = 0.0014;
        yoff = -0.0043;
        zoff = -0.0033;
    end
    
    methods
        %% object structors
        function self = Bricks(robotBase,wallPosition,Log)
            disp('Calculating start and end pose of bricks.');
            Log.mlog = {Log.DEBUG,'Bricks','Calculating start and end pose of bricks.'};
            
            % calculate the position where the bricks get generated and where they belong in the wall
            self.startPos = self.GetBrickStartPositions(robotBase);
            self.wallPos = self.GetBrickWallPositions(robotBase,wallPosition);
            
            disp('Load brick models...');
            Log.mlog = {Log.DEBUG,'Bricks','Load brick models...'};
            % load every brick at starting pose
            for i = 1:9
                self.brick{i} = self.GetBrickModel(['brick',num2str(i)]);
                self.brick{i}.base = self.startPos{i};
                self.PlotAndColourBrick(i);
                drawnow();
            end
            
            disp('All bricks loaded.');
            Log.mlog = {Log.DEBUG,'Bricks','All bricks loaded.'};
        end
        
        
        
        %% get position where the bricks get generated
        function startPos = GetBrickStartPositions(self,robotBase)
            % position of middle brick
            middle = robotBase * transl(0, -0.41, 0); %-0.41 is a good y position
            % determines the transform of the middle brick (brick{5}) dependend on the position of the robot

            % get transform of six bricks equally distributed in front of the robot with space of 0.03m between each other
            space = 0.03;
            index = 1;
            for i = 1:3
                for j = 1:3
                    startPos{index} = middle * transl((i-2)*(self.length+space),(j-2)*(self.width+space),0);
                    index = index + 1;
                end
            end
        end
    
        %% get position where the bricks belong in the wall
        function wallPos = GetBrickWallPositions(self,robotBase,wallPosition)
            % pose of wall (wallPosition) represents the first brick of the wall. 
            % the wall will be made out of 3 by 3 bricks along the positive x-axis of wallPosition
            wallPos{1} = robotBase * wallPosition;
            index = 2;
            for i = 1:3
                for j = 2:3
                    wallPos{index} = wallPos{index-1} * transl(self.length,0,0);
                    index = index + 1;
                end
                if i < 3
                    wallPos{index} = wallPos{index-3} * transl(0,0,self.hight);
                    index = index + 1;
                end
            end
        end
        
        %% PlotAndColourBrick
        % Given a brick index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourBrick(self, i)
            % load data from .ply file
            [ faceData, vertexData, plyData] = plyread('HalfSizedRedGreenBrick.ply','tri');                
            self.brick{i}.faces = {faceData, []};
            self.brick{i}.points = {[vertexData(:,1)+self.xoff,vertexData(:,2)+self.yoff,vertexData(:,3)+self.zoff] * rotz(pi/2),[]};
            
            % Display bricks
            plot3d(self.brick{i},0,'delay',0,'view', [125 15]);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.brick{i}.delay = 0;

            % Try to correctly colour the bricks (if colours are in ply file data)
            handles = findobj('Tag', self.brick{i}.name);
            h = get(handles,'UserData');
            try 
            h.link(1).Children.FaceVertexCData = [plyData.vertex.red ...
                                                , plyData.vertex.green ...
                                                , plyData.vertex.blue]/255;
            h.link(1).Children.FaceColor = 'interp';
            catch ME_1
               disp(ME_1);
            end
        end
    end
    
    methods (Static)
        %% get brick models
        function model = GetBrickModel(name)
            % generate brick models
            L1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            model = SerialLink(L1,'name',name);
        end
    end
end
