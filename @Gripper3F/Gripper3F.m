classdef Gripper3F < handle
    % class builds a gripper with three fingers similar to the DHDS-50-A from Festo
    % source: https://www.festo.com/au/en/a/1259495/?q=~:sortByCoreRangeAndSp2020
    
    properties
        % cell structure of two fingers
        G3Finger;
        
        % base of gripper
        base = eye(4);
        workspace = [-0.2 0.2 -0.2 0.2 -0.2 0.2];
    end
    
    methods
        %% object structors
        function self = Gripper3F()
            
            self.GetGripper3F;
            self.PlotAndColourGripper3F;
%             for i=1:3
%             self.G3Finger{i}.plot(0, 'workspace', self.workspace);
%             hold on
%             pause
%             end
            
        end
        
        
        %% Get Gripper3F Model
        function GetGripper3F(self)
            L1 = Link('theta',0,'a',0,'alpha',0,'prismatic','qlim',[-0.006 0],'offset',0); % PRISMATIC Link
            
            for i = 1:3
                name = ['G3Finger',num2str(i)];
                self.G3Finger{i} = SerialLink(L1,'name',name);
            end
            
            self.G3Finger{1}.base = self.base * trotz(0) * transl(-0.025, 0, 0.0705) * troty(pi/2);
            self.G3Finger{2}.base = self.base * trotz(2*pi/3) * transl(-0.025, 0, 0.0705) * troty(pi/2);
            self.G3Finger{3}.base = self.base * trotz(4*pi/3) * transl(-0.025, 0, 0.0705) * troty(pi/2);
        end
        
        %% Plot and Colour Gripper3F
        % Given a finger index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourGripper3F(self)
            % iterate through fingers
            for i = 1:3
                % load data from .ply file
                if i == 1
                    for linkIndex = 0:self.G3Finger{1}.n
                        [faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Gripper3F',num2str(linkIndex),'.ply'],'tri');              
                        self.G3Finger{i}.faces{linkIndex + 1} = faceData;
                        self.G3Finger{i}.points{linkIndex + 1} = vertexData;
                    end
                
                    % Display gripper with 3D shape
                    self.G3Finger{i}.plot3d(0,'noarrow','workspace',self.workspace);
                    if isempty(findobj(get(gca,'Children'),'Type','Light'))
                        camlight
                    end
                    self.G3Finger{i}.delay = 0;


                    % Try to correctly colour the fingers (if colours are in ply file data)
                    for linkIndex = 0:self.G3Finger{i}.n
                        handles = findobj('Tag', self.G3Finger{i}.name);
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
                    for linkIndex = 1:self.G3Finger{1}.n
                        [faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Gripper3F',num2str(linkIndex),'.ply'],'tri');              
                        self.G3Finger{i}.faces{linkIndex + 1} = faceData;
                        self.G3Finger{i}.points{linkIndex + 1} = vertexData;
                    end
                    
                    % Display gripper with 3D shape
                    self.G3Finger{i}.plot3d(0,'noarrow','workspace',self.workspace);
                    if isempty(findobj(get(gca,'Children'),'Type','Light'))
                        camlight
                    end
                    self.G3Finger{i}.delay = 0;


                    % Try to correctly colour the fingers (if colours are in ply file data)
                    for linkIndex = 1:self.G3Finger{i}.n
                        handles = findobj('Tag', self.G3Finger{i}.name);
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
    end
end