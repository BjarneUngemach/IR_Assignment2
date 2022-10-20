classdef Squeegee < handle
    
    properties
        squeegee;
    end
    
    properties (Hidden)
        squeegeeHome = transl(-0.1, -0.12, 0.05)
    end
    
    methods
        %% ...structors
        function self = Squeegee
            
            self.GetSqueegeeModel;
            self.PlotAndColourSqueegee;
                 
        end
        
        %% GetModel
        function GetSqueegeeModel(self)
            name = 'Squeegee';
            L1 = Link('alpha',0,'a',1,'d',0,'offset',0);
            self.squeegee = SerialLink(L1,'name',name);
        end
        
        %% PlotAndColourSqueegee
        % Given a squeegee, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourSqueegee(self)
            % load data from .ply file
            [faceData,vertexData,plyData] = plyread('Squegee.ply','tri');
            self.squeegee.faces = {faceData,[]};
            self.squeegee.points = {vertexData,[]};
            
            % Display squeegee
            self.squeegee.plot3d(0,'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.squeegee.delay = 0;
            
            % Try to correctly colour the sponge (if colours are in ply file data)
            handles = findobj('Tag', self.squeegee.name);
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
        
        %% update position of gripper
        function UpdateSqueegee(self, pose)
            
            % change base of sponge to given pose
            self.squeegee.base = pose;
            
            % plot updated gripper
            self.squeegee.animate(0)
        end
    end
end
