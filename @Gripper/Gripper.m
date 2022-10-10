classdef Gripper < handle
    % class builds a gripper with two fingers equal to the 2F-85 from
    % RobotIQ
    % source: https://robotiq.com/products/2f85-140-adaptive-robot-gripper
    
    properties
        % cell structure of two fingers
        finger;

        %structure for base/core
        core

        % gripper status (open/grab/close)
        status;
        
        % base of gripper
        base = eye(4);
    end
    
    methods
        %% object structors
        function self = Gripper(roomSize,Log)
            disp('Loading gripper model...');
            Log.mlog = {Log.DEBUG,'Gripper','Loading gripper model...'}; 
            
            % create two fingers 
            for i = 1:1
                self.finger{i} = self.GetFingerModel(['finger',num2str(i)]);
            end
            
            % get right position for fingers
            % takes transform of gripperbase and translate it to the desired position (second finger flipped around)
            % the following troty and trotx gets the right orientation of the fingers
            self.finger{1}.base = self.base * trotz(+pi/2) * transl(0.012700, 0, 0.061178) * troty(-pi/2) * trotx(pi/2);
            self.finger{2}.base = self.base * trotz(-pi/2) * transl(0.012700, 0, 0.061178) * troty(-pi/2) * trotx(pi/2);
            
            % plot 3D-Model
            hold on;
            self.PlotAndColourGripper(roomSize);
            
            disp('Gripper loaded.');
            Log.mlog = {Log.DEBUG,'Gripper','Gripper loaded.'};
        end
        
        %% close gripper
        function CloseGripper(self, steps, Log)
            disp('Closing gripper.');
            Log.mlog = {Log.DEBUG,'Gripper','Closing gripper.'};
            %declare joint angle (absolute values are the same for both joints because we want the fingertips to stay parallel)
            qc = deg2rad([5.16 -5.16]);  % angles in closed position accordingly to data sheet (5.16°)
                        
            qclose = jtraj(self.finger{1}.getpos(), qc, steps); % calculate joint state matrix
            for i = 1:steps                                     % play joint state matrix step by step to simulate simultaneous movement
                self.finger{1}.animate(qclose(i,:));
                self.finger{2}.animate(qclose(i,:));
                drawnow();
                pause(0.0001);
            end
            self.status = 0;    % set gripper status to "close"
        end
        
        %% open gripper
        function OpenGripper(self, steps, Log)
            disp('Opening gripper.');
            Log.mlog = {Log.DEBUG,'Gripper','Opening gripper.'};
            %declare joint angle (absolute values are the same for both joints because we want the fingertips to stay parallel)
            qo = deg2rad([-40.7 40.7]);  % angles in opened position accordingly to data sheet (40.7°)
            
            qclose = jtraj(self.finger{1}.getpos(), qo, steps); % calculate joint state matrix
            for i = 1:steps                                     % play joint state matrix step by step to simulate simultaneous movement
                self.finger{1}.animate(qclose(i,:));
                self.finger{2}.animate(qclose(i,:));
                drawnow();
                pause(0.0001);
            end
            self.status = 2;    % set gripper status to "open"
        end
        
        %% gripper in grab position
        function GrabGripper(self, steps, Log)
            disp('Grabbing brick.');
            Log.mlog = {Log.DEBUG,'Gripper','Grabbing brick.'};
            %declare joint angle (absolute values are the same for both joints because we want the fingertips to stay parallel)
            % calculation of needed angle to grab (measurements from CAD-model)
            calcQ = asind(((0.0667-0.0254)/2 +0.007385)/0.057281);
            qg = deg2rad([-calcQ calcQ]);  % angles in grab position
                        
            qgrab = jtraj(self.finger{1}.getpos(), qg, steps);  % calculate joint state matrix
            for i = 1:steps                                     % play joint state matrix step by step to simulate simultaneous movement
                self.finger{1}.animate(qgrab(i,:));
                self.finger{2}.animate(qgrab(i,:));
                drawnow();
                pause(0.0001);
            end
            self.status = 1;    % set gripper status to "grab"
        end
        
        %% update position of gripper
        function UpdateGripperPosition(self, pose)
            
            % change base of gripper to given pose
            self.base = pose;
                        
            % get current joint angles of fingers
            q{1} = self.finger{1}.getpos();
            q{2} = self.finger{2}.getpos();
            
            % update base of every finger
            self.finger{1}.base = self.base * trotz(+pi/2) * transl(0.012700, 0, 0.061178) * troty(-pi/2) * trotx(pi/2);
            self.finger{2}.base = self.base * trotz(-pi/2) * transl(0.012700, 0, 0.061178) * troty(-pi/2) * trotx(pi/2);
            
            % plot updated gripper
            for i = 1:2
                self.finger{i}.animate(q{i})
            end
        end
            
        %% PlotAndColourGripper
        % Given a gripper index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourGripper(self, roomSize)
            % finger 1 and finger 2
            for i = 1:1
                % load data from .ply file
                for linkIndex = (i-1):self.finger{1}.n
                    [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['gripper0.ply'],'tri');              
                    self.finger{i}.faces{linkIndex + 1} = faceData;
                    self.finger{i}.points{linkIndex + 1} = vertexData;
                end

                % Display gripper with 3D shape
                self.finger{i}.plot3d(deg2rad([-40.7 40.7]),'noarrow','workspace',roomSize,'view', [125 15]);
                if isempty(findobj(get(gca,'Children'),'Type','Light'))
                    camlight
                end
                self.finger{i}.delay = 0;

                % Try to correctly colour the fingers (if colours are in ply file data)
                for linkIndex = (i-1):self.finger{i}.n
                    handles = findobj('Tag', self.finger{i}.name);
                    h = get(handles,'UserData');
                    try 
                        h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                      , plyData{linkIndex+1}.vertex.green ...
                                                                      , plyData{linkIndex+1}.vertex.blue]/255;
                        h.link(linkIndex+1).Children.FaceColor = 'interp';
                    catch ME_1
                        disp(ME_1);
                        continue;
                    end
                end
            end
        end
    end
    
    
    methods (Static)
        %% GetFingerModel
        function model = GetFingerModel(name)
            
            % declare the links and make a model out of it for one finger
            L1 = Link('d',0,'a',0.050,'alpha',0,'qlim',[-0.8 0],'offset',1);
            
            model = SerialLink(L1,'name',name);
        end        
        function model = GetBaseModel(name)
            
            % declare the links and make a model out of it for one finger
            L1 = Link('d',0.5,'a',0,'alpha',0,'qlim',[0 0],'offset',0);
            
            model = SerialLink(L1,'name',name);
        end 
    end
    
end

