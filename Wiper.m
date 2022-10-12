classdef Wiper < handle
    %UFO A way of creating a group of UFOs
    %
    
    properties
        
        wiper;
        
        
        %> Ply file data about the model
        faceData = [];
        vertexData = [];
        plyData = [];
    end
    
    methods
        %% ...structors
        function self = Wiper
            
            
            
            self.wiper = self.GetModel('Wiper');
            
            self.wiper.base = transl(0.2, -0.12, 0.18)*troty(pi)*trotz(pi/2);
            
            self.wiper.animate(0);
                 
        end
        
        
        
        
        
        
        %% GetModel
        function wiper = GetModel(self,name)
            if nargin < 1
                name = 'Wiper';
            end
            if isempty(self.faceData) || isempty(self.vertexData) || isempty(self.plyData)
                [self.faceData,self.vertexData,self.plyData] = plyread('Squegee.ply','tri');
            end
            L1 = Link('alpha',0,'a',1,'d',0,'offset',0);
            wiper = SerialLink(L1,'name',name);
            wiper.faces = {self.faceData,[]};
            wiper.points = {self.vertexData,[]};
            
            plot3d(wiper,0);
            handles = findobj('Tag', wiper.name);
            h = get(handles,'UserData');
            
            
%             h.link(1).Children.FaceVertexCData = [self.plyData.vertex.red ...
%                 ,self.plyData.vertex.green ...
%                 ,self.plyData.vertex.blue]/255;
%             
%             h.link(1).Children.FaceColor = 'interp';
        end
    end
end


