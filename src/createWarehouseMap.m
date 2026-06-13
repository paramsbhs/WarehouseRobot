function map = createWarehouseMap(resolution)
%CREATEWAREHOUSEMAP Binary occupancy map of a 50 m x 30 m warehouse.
%   Outer walls and three shelf racks with aisles between them.
%   resolution is in cells per meter (default 2).

if nargin < 1
    resolution = 2;
end

widthMeters  = 50;
heightMeters = 30;
nRows = heightMeters * resolution;
nCols = widthMeters * resolution;
grid = false(nRows, nCols);

% Outer walls
grid([1 end], :) = true;
grid(:, [1 end]) = true;

% Shelf racks: 2 m deep, x = 10..42 m, cross-aisles at both ends
shelfRowsMeters = [7 14 21];
for y = shelfRowsMeters
    rows = (y*resolution) : ((y+2)*resolution);
    cols = (10*resolution) : (42*resolution);
    grid(rows, cols) = true;
end

map = binaryOccupancyMap(grid, resolution);
end
