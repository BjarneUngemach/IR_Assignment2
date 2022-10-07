classdef MECA500 < handle
    properties
        %> Robot model
        meca500;
    end
    
    methods%% Class for Meca500 robot simulation
        function self = MECA500()
                        
            self.GetMeca500Robot();
            self.PlotAndColourMeca500();

            drawnow
        end

        %% GetMeca500Robot
        % Given a name (optional), create and return a Meca500 robot model
        function GetMeca500Robot(self)
            name = 'Meca500';
            L1 = Link('d',0.135,'a',0,'alpha',pi/2,'qlim',deg2rad([-175 175]), 'offset',0);
            L2 = Link('d',0,'a',0.135,'alpha',0,'qlim', deg2rad([-70 90]), 'offset',pi/2);
            L3 = Link('d',0,'a',0.038,'alpha',pi/2,'qlim', deg2rad([-135 70]), 'offset', 0);
            L4 = Link('d',0.12,'a',0,'alpha',-pi/2,'qlim',deg2rad([-170 170]),'offset', 0);
            L5 = Link('d',0,'a',0,'alpha',pi/2,'qlim',deg2rad([-115 115]), 'offset',0);
            L6 = Link('d',0.07,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
             
            self.meca500 = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
%             self.meca500.plot(zeros(1,6));
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourMeca500(self)
            for linkIndex = 0:self.meca500.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Meca500link_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                self.meca500.faces{linkIndex + 1} = faceData;
                self.meca500.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.meca500.plot3d(zeros(1,self.meca500.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.meca500.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.meca500.n
                handles = findobj('Tag', self.meca500.name);
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
