classdef Meca500 < handle
    properties
        %> Robot model
        model;
        
        %> workspace
        workspace = [-0.6 0.6 -0.6 0.6 -0.2 1.1];   
      
    end
    
    methods%% Class for Meca500 robot simulation
        function self = Meca500(toolModelAndTCPFilenames)
            if 0 < nargin
                if length(toolModelAndTCPFilenames) ~= 2
                    error('Please pass a cell with two strings, toolModelFilename and toolCenterPointFilename');
                end
                self.toolModelFilename = toolModelAndTCPFilenames{1};
                self.toolParametersFilenamure = toolModelAndTCPFilenames{2};
            end
            
            self.GetMeca500Robot();
            self.PlotAndColourRobot();%robot,workspace);

            drawnow
        end

        %% GetMeca500Robot
        % Given a name (optional), create and return a Meca500 robot model
        function GetMeca500Robot(self)
            pause(0.001);
            name = ['Meca500_',datestr(now,'yyyymmddTHHMMSSFFF')];
            L1 = Link('d',0.135,'a',0,'alpha',pi/2,'qlim',deg2rad([-175 175]), 'offset',0);
            L2 = Link('d',0,'a',0.135,'alpha',0,'qlim', deg2rad([-70 90]), 'offset',0);
            L3 = Link('d',0,'a',0.038'alpha',0,'qlim', deg2rad([-135 70]), 'offset', 0);
            L4 = Link('d',0.12,'a',0,'alpha',pi/2,'qlim',deg2rad([-170 170]),'offset', 0);
            L5 = Link('d',0,'a',0.07,'alpha',-pi/2,'qlim',deg2rad([-115 115]), 'offset',0);
            L6 = Link('d',0,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
             
            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Meca500link_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
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
