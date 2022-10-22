classdef Sign < handle
    %UFO A way of creating a group of UFOs
    %
    
    properties
        
        sign;
        
        
        %> Ply file data about the model
        faceDataSign = [];
        vertexDataSign = [];
        plyDataSign = [];
    end
    
    methods
        %% ...structors
        function self = Sign
            
            
            
            self.sign = self.GetModelSign('Sign');
            
            self.sign.base = transl(-0.1, -0.18, 0.18)*troty(pi);
            
            self.sign.animate(0);
                 
        end
        
        
        
        
        
        
        %% GetModel
        function sign = GetModelSign(self,name)
            if nargin < 1
                name = 'Sign';
            end
            if isempty(self.faceDataSign) || isempty(self.vertexDataSign) || isempty(self.plyDataSign)
                [self.faceDataSign,self.vertexDataSign,self.plyDataSign] = plyread('Sign.ply','tri');
            end
            L1 = Link('alpha',0,'a',1,'d',0,'offset',0);
            sign = SerialLink(L1,'name',name);
            sign.faces = {self.faceDataSign,[]};
            sign.points = {self.vertexDataSign,[]};
            
            plot3d(sign,0);
            handles = findobj('Tag', sign.name);
            h = get(handles,'UserData');
            
            
%             h.link(1).Children.FaceVertexCData = [self.plyDataSign.vertex.red ...
%                 ,self.plyDataSign.vertex.green ...
%                 ,self.plyDataSign.vertex.blue]/255;
%             
%             h.link(1).Children.FaceColor = 'interp';
        end
    end
end


