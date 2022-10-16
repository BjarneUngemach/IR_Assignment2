classdef Table < handle
       
    properties
        % robot models
        table;
    end
    
    methods
        %% object structors
        function self = Table()
                        
            self.GetTableModel();
            self.PlotAndColourTable();
            
        end
        
        %% get table model
        function GetTableModel(self)
            name = 'SwipeBot';
            L1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            self.table = SerialLink(L1,'name',name);
        end
        
        %% PlotAndColourTable
        % Given a table, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourTable(self)
            % load data from .ply file
            [faceData, vertexData, plyData] = plyread('Table.ply','tri');                
            self.table.faces = {faceData, []};
            self.table.points = {[vertexData(:,1),vertexData(:,2),vertexData(:,3)], []};
            
            % Display table
            self.table.plot3d(0,'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.table.delay = 0;

            % Try to correctly colour the table (if colours are in ply file data)
            handles = findobj('Tag', self.table.name);
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
            
               