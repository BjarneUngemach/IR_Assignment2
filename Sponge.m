classdef Sponge 
    
        properties
            
        model;
        
        % resulting dimensions
        width = 0.2;
        length = 0.5;
        height = 0.15;
        
        % Offset of centre point
%         xoff = 0.0014;
%         yoff = -0.0043;
%         zoff = -0.0033;
    end
    
    methods
        %% PlotAndColourSponge
        % Given a sponge index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourSponge(self, i)
            % load data from .ply file
            [ faceData, vertexData, plyData] = plyread('sponge.ply','tri');                
            self.sponge{i}.faces = {faceData, []};
            self.sponge{i}.points = {[vertexData(:,1)+self.xoff,vertexData(:,2)+self.yoff,vertexData(:,3)+self.zoff] * rotz(pi/2),[]};
            
            % Display sponge
            plot3d(self.sponge{i},0,'delay',0,'view', [125 15]);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.sponge{i}.delay = 0;

            % Try to correctly colour the bricks (if colours are in ply file data)
            handles = findobj('Tag', self.sponge{i}.name);
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
    end
    
    methods (Static)
        %% Get Sponge model
        function model = GetSpongeModel(name)
            % generate Sponge models
            L1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            model = SerialLink(L1,'name',name);
        end
    end
end
