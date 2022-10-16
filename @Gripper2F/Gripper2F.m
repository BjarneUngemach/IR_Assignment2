classdef Gripper2F < handle
    % class builds a gripper with three fingers similar to the HGPL-25-20-A-B from Festo
    % source: https://www.festo.com/gb/en/a/3361483/?q=~:sortByCoreRangeAndSp2020~:CC_Stroke_per_gripper_jaw_C_FP_GLOBAL~:%255B20%252C%255D
    
    properties
        % cell structure of three fingers
        g2Finger;
    end
    
    properties (Hidden)
        % base of gripper
        g2FBase = eye(4);
        
        %status of gripper
        g2FStatus = "open";
        workspace = [-0.1 0.1 -0.1 0.1 -0.2 0.2]
        dx = 0.019;
        dz = 0.03;
    end
    
    methods
        %% object structors
        function self = Gripper2F()
            
            self.GetGripper2F;
            self.PlotAndColourGripper2F;

        end
        
        
        %% Get Gripper3F Model
        function GetGripper2F(self)
            L1 = Link('theta',0,'a',0,'alpha',0,'prismatic','qlim',[-0.02 0],'offset',0); % PRISMATIC Link
            
            for i = 1:2
                name = ['g2Finger',num2str(i)];
                self.g2Finger{i} = SerialLink(L1,'name',name);
            end
            
            self.g2Finger{1}.base = self.g2FBase * trotz(180,'deg')*transl(self.dx, 0, self.dz) ;
            self.g2Finger{2}.base = self.g2FBase * transl(-self.dx, 0, self.dz);
        end
        
        %% Plot and Colour Gripper3F
        % Given a finger index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourGripper2F(self)
            % iterate through fingers
            for i = 1:2
                % load data from .ply file
                if i == 1
                    for linkIndex = 0:self.g2Finger{1}.n
                        [faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Gripper2F',num2str(linkIndex),'.ply'],'tri');              
                        self.g2Finger{i}.faces{linkIndex + 1} = faceData;
                        self.g2Finger{i}.points{linkIndex + 1} = vertexData;
                    end
                
                    % Display gripper with 3D shape
                    self.g2Finger{i}.plot3d(0,'noarrow','workspace',self.workspace);
                    if isempty(findobj(get(gca,'Children'),'Type','Light'))
                        camlight
                    end
                    self.g2Finger{i}.delay = 0;


                    % Try to correctly colour the fingers (if colours are in ply file data)
                    for linkIndex = 0:self.g2Finger{i}.n
                        handles = findobj('Tag', self.g2Finger{i}.name);
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
                else
                    for linkIndex = 1:self.g2Finger{1}.n
                        [faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Gripper2F',num2str(linkIndex),'.ply'],'tri');              
                        self.g2Finger{i}.faces{linkIndex + 1} = faceData;
                        self.g2Finger{i}.points{linkIndex + 1} = vertexData;
                    end
                    
                    % Display gripper with 3D shape
                    self.g2Finger{i}.plot3d(0,'noarrow','workspace',self.workspace);
                    if isempty(findobj(get(gca,'Children'),'Type','Light'))
                        camlight
                    end
                    self.g2Finger{i}.delay = 0;


                    % Try to correctly colour the fingers (if colours are in ply file data)
                    for linkIndex = 1:self.g2Finger{i}.n
                        handles = findobj('Tag', self.g2Finger{i}.name);
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
        
        %% update position of gripper
        function UpdateGripper2F(self)
            
            % change base of gripper to given pose
            self.g2FBase = self.ur3.fkine(self.ur3.getpos);
                                    
            % update base of every finger
            self.g2Finger{1}.base = self.g2FBase * transl(self.dx, 0, self.dz) ;
            self.g2Finger{2}.base = self.g2FBase * transl(-self.dx, 0, self.dz);
            
            if self.g2FStatus == "close"
                q = 0;
            elseif self.g2FStatus == "open"
                q = -0.02;
            end
            
            % plot updated gripper
            for i = 1:2
                self.g2Finger{i}.animate(q)
            end
        end
    end
end