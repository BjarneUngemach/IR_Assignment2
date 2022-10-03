classdef PF3400 < handle
    properties
        %> Robot model
        pf3400;
        
        %> workspace
        pf3400Workspace = [-1 1 -1 1 0 1.5];   
      
    end
    
    methods%% Class for PF3400 robot simulation
        function self = PF3400()
                        
            self.GetPF3400Robot();
            self.PlotAndColourRobotPF3400();%robot,workspace);

            drawnow
        end

        %% GetPF3400Robot
        % Given a name (optional), create and return a PF3400 robot model
        function GetPF3400Robot(self)
            name = 'PF3400';
            L1 = Link('theta',0,'a',0.098450,'alpha',pi,'prismatic','qlim',[0.173853 0.923853],'offset',0); % PRISMATIC Link
            L2 = Link('d',0.065675,'a',0.302,'alpha',0,'qlim', deg2rad([-90 90]), 'offset',0);
            L3 = Link('d',0.040175,'a',0.289,'alpha',0,'qlim', deg2rad([-167 167]), 'offset', 0);
            L4 = Link('d',0.032003,'a',0,'alpha',0,'qlim',deg2rad([-110 470]),'offset', 0);
            
            self.pf3400 = SerialLink([L1 L2 L3 L4],'name',name);
            
%             self.pf3400.plot(zeros(1,self.pf3400.n),'noarrow','workspace',self.workspace);
            
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobotPF3400(self)
            for linkIndex = 0:self.pf3400.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['pf3400link_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                self.pf3400.faces{linkIndex + 1} = faceData;
                self.pf3400.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.pf3400.plot3d([0.173853, zeros(1,self.pf3400.n-1)],'noarrow','workspace',self.pf3400Workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.pf3400.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.pf3400.n
                handles = findobj('Tag', self.pf3400.name);
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

