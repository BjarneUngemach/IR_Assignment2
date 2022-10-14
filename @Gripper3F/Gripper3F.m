classdef Gripper3F < handle
    % class builds a gripper with three fingers similar to the DHDS-50-A from Festo
    % source: https://www.festo.com/au/en/a/1259495/?q=~:sortByCoreRangeAndSp2020
    
    properties
        % cell structure of three fingers
        g3Finger;
    end
    
    properties (Hidden)
        % base of gripper
        g3FBase = eye(4);
        
        %status of gripper
        g3FStatus = "open";
    end
    
    methods
        %% object structors
        function self = Gripper3F()
            
            self.GetGripper3F;
            self.PlotAndColourGripper3F;
           
        end
        
        
        %% Get Gripper3F Model
        function GetGripper3F(self)
            L1 = Link('theta',0,'a',0,'alpha',0,'prismatic','qlim',[-0.006 0],'offset',0); % PRISMATIC Link
            
            for i = 1:3
                name = ['g3Finger',num2str(i)];
                self.g3Finger{i} = SerialLink(L1,'name',name);
            end
            
            self.g3Finger{1}.base = self.g3FBase * trotz(pi/2 + 0     ) * transl(-0.025, 0, 0.0705) * troty(pi/2);
            self.g3Finger{2}.base = self.g3FBase * trotz(pi/2 + 2*pi/3) * transl(-0.025, 0, 0.0705) * troty(pi/2);
            self.g3Finger{3}.base = self.g3FBase * trotz(pi/2 + 4*pi/3) * transl(-0.025, 0, 0.0705) * troty(pi/2);
        end
        
        %% Plot and Colour Gripper3F
        % Given a finger index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourGripper3F(self)
            % iterate through fingers
            for i = 1:3
                % load data from .ply file
                if i == 1
                    for linkIndex = 0:self.g3Finger{1}.n
                        [faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Gripper3F',num2str(linkIndex),'.ply'],'tri');              
                        self.g3Finger{i}.faces{linkIndex + 1} = faceData;
                        self.g3Finger{i}.points{linkIndex + 1} = vertexData;
                    end
                
                    % Display gripper with 3D shape
                    self.g3Finger{i}.plot3d(0,'noarrow','workspace',self.workspace);
                    if isempty(findobj(get(gca,'Children'),'Type','Light'))
                        camlight
                    end
                    self.g3Finger{i}.delay = 0;


                    % Try to correctly colour the fingers (if colours are in ply file data)
                    for linkIndex = 0:self.g3Finger{i}.n
                        handles = findobj('Tag', self.g3Finger{i}.name);
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
                    for linkIndex = 1:self.g3Finger{1}.n
                        [faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Gripper3F',num2str(linkIndex),'.ply'],'tri');              
                        self.g3Finger{i}.faces{linkIndex + 1} = faceData;
                        self.g3Finger{i}.points{linkIndex + 1} = vertexData;
                    end
                    
                    % Display gripper with 3D shape
                    self.g3Finger{i}.plot3d(0,'noarrow','workspace',self.workspace);
                    if isempty(findobj(get(gca,'Children'),'Type','Light'))
                        camlight
                    end
                    self.g3Finger{i}.delay = 0;


                    % Try to correctly colour the fingers (if colours are in ply file data)
                    for linkIndex = 1:self.g3Finger{i}.n
                        handles = findobj('Tag', self.g3Finger{i}.name);
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
        function UpdateGripper3F(self)
            
            % change base of gripper to given pose
            self.g3FBase = self.ur3.fkine(self.ur3.getpos);
                                    
            % update base of every finger
            self.g3Finger{1}.base = self.g3FBase * trotz(pi/2 + 0     ) * transl(-0.025, 0, 0.0705) * troty(pi/2);
            self.g3Finger{2}.base = self.g3FBase * trotz(pi/2 + 2*pi/3) * transl(-0.025, 0, 0.0705) * troty(pi/2);
            self.g3Finger{3}.base = self.g3FBase * trotz(pi/2 + 4*pi/3) * transl(-0.025, 0, 0.0705) * troty(pi/2);
            
            if self.g3FStatus == "close"
                q = 0;
            elseif self.g3FStatus == "open"
                q = -0.006;
            end
            
            % plot updated gripper
            for i = 1:3
                self.g3Finger{i}.animate(q)
            end
        end
    end
end