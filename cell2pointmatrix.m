function matrix = cell2pointmatrix(cellarray)
    matrix = [];
    for i = 1:size(cellarray,2)
        matrix = [matrix;
                  cellarray{1,i}];
    end
end

