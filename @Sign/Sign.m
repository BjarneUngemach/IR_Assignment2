classdef Sign < handle
    
    properties
        sign;
    end
    
     properties (Hidden)
        signHome = transl(0.15,0.15,0.1)*trotz(-pi/2)*trotx(pi/2)
    end
    
    methods
        %% ...structors
        function self = Sign
            
            self.GetSignModel;
            self.PlotAndColourSign;
            
        end
                
        %% GetModel
        function GetSignModel(self)
            name = 'Sign';
            L1 = Link('alpha',0,'a',1,'d',0,'offset',0);
            self.sign = SerialLink(L1,'name',name);
        end
        
        %% PlotAndColourSign
        function PlotAndColourSign(self)
            % load data from .ply file
            [faceData,vertexData,plyData] = plyread('Sign.ply','tri');
            self.sign.faces = {faceData,[]};
            self.sign.points = {vertexData,[]};
            
            % Display sign
            self.sign.plot3d(0,'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.sign.delay = 0;
            
            % Try to correctly colour the sponge (if colours are in ply file data)
            handles = findobj('Tag', self.sign.name);
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
        
        %% update position of sign
        function UpdateSign(self, pose)

            % change base of sign to given pose
            self.sign.base = pose;

            % plot updated sign
            self.sign.animate(0)
        end
    end
end
