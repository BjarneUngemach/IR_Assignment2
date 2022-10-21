classdef Sponge < handle
        
    properties
        sponge;
    end
    
    properties (Hidden)
        spongeHome = transl(0.1, -0.12, 0.05)
    end
    
    methods
        %% ...structors
        function self = Sponge
            
            self.GetSpongeModel;
            self.PlotAndColourSponge;
            
        end
        
        %% GetModel
        function GetSpongeModel(self)
            name = 'Sponge';
            L1 = Link('alpha',0,'a',1,'d',0,'offset',0);
            self.sponge = SerialLink(L1,'name',name);
        end
        
        %% PlotAndColourSponge
        % Given a sponge, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourSponge(self)
            % load data from .ply file
            [faceData,vertexData,plyData] = plyread('Sponge.ply','tri');
            self.sponge.faces = {faceData,[]};
            self.sponge.points = {vertexData,[]};
            
            % Display sponge
            self.sponge.plot3d(0,'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.sponge.delay = 0;
            
            % Try to correctly colour the sponge (if colours are in ply file data)
            handles = findobj('Tag', self.sponge.name);
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
        function UpdateSponge(self, pose)
            
            % change base of sponge to given pose
            self.sponge.base = pose;
            
            % plot updated gripper
            self.sponge.animate(0)
        end
    end
end