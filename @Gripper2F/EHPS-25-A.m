classdef Gripper < handle
    % class builds a gripper with two fingers equal to the EHPS-25-A from
    % Festo
    % source: https://www.festo.com/gb/en/a/8070830/?q=~:sortByCoreRangeAndSp2020
    
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
        function self = Gripper(roomSize)

            % create core
            self.core = self.GetBaseModel('Base');
            self.core.plot(0);
            
            % create three fingers 
            for i = 1:3
                self.finger{i} = self.GetFingerModel(['finger',num2str(i)]);
                   
            end
            
            % get right position for fingers
            % takes transform of gripperbase and translate it to the desired position (second finger flipped around)
            % the following troty and trotx gets the right orientation of the fingers
            self.finger{1}.base = self.base 
            self.finger{2}.base = self.base 
            
            % plot 3D-Model
            hold on;
%             self.PlotAndColourGripper(roomSize);
            self.finger{1}.plot(-0.01);
        end

            methods (Static)
        %% GetFingerModel
        function model = GetFingerModel(name)
            
            % declare the links and make a model out of it for one finger
            L1 = Link('d',0,'a',0.049,'alpha',0,'qlim',[-0.8 0],'offset',1);
            
            model = SerialLink(L1,'name',name);
        end        
        function model = GetBaseModel(name)
            
            % declare the links and make a model out of it for one finger
            L1 = Link('d',0.005,'a',0,'alpha',0,'qlim',[0 0],'offset',0);
            
            model = SerialLink(L1,'name',name);
            
        end 
    end