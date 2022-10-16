classdef Hand < handle
       
    properties
        % robot models
        hand;
    end
    
    methods
        %% object structors
        function self = Hand()
                        
            self.GetHandModel();
            %self.PlotAndColourHand();
            
        end
        
        %% get table model
        function GetHandModel(self)
            name = 'Hand';
            L1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            self.hand = SerialLink(L1,'name',name);
        end
        
        %% PlotAndColourTable
        % Given a hand, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourHand(self)
            % load data from .ply file
            [ faceData, vertexData, plyData] = plyread('Hand.ply','tri');                
            self.hand.faces = {faceData, []};
            self.hand.points = {[vertexData(:,1),vertexData(:,2),vertexData(:,3)], []};
            
            % Display hand
            self.hand.plot3d(0,'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.hand.delay = 0;

            % Try to correctly colour the hand (if colours are in ply file data)
            handles = findobj('Tag', self.hand.name);
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
end