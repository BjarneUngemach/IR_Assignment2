classdef Sponge < handle
    %UFO A way of creating a group of UFOs
    %
    
    properties
        
        sponge;
        
        
        %> Ply file data about the model
        faceData = [];
        vertexData = [];
        plyData = [];
    end
    
    methods
        %% ...structors
        function self = Sponge
            
            self.GetModel('Sponge');
            
            self.sponge.base = transl(0.2, -0.18, 0.18)*troty(pi)*trotz(pi/2);
            
            self.sponge.animate(0);
            
        end
        
        
        
        
        
        
        %% GetModel
        function GetModel(self,name)
            if nargin < 1
                name = 'Sponge';
            end

            [faceData,vertexData,plyData] = plyread('Sponge.ply','tri');
            
            L1 = Link('alpha',0,'a',1,'d',0,'offset',0);
            self.sponge = SerialLink(L1,'name',name);
            self.sponge.faces = {faceData,[]};
            self.sponge.points = {vertexData,[]};
            
            plot3d(self.sponge,0);
            handles = findobj('Tag', self.sponge.name);
            h = get(handles,'UserData');
            
          
%             h.link(1).Children.FaceVertexCData = [plyData.vertex.red, plyData.vertex.green, plyData.vertex.blue]/255;
%            
%             h.link(1).Children.FaceColor = 'interp';
        end
    end
end


