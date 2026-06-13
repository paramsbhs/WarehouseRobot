function waypoints = gridPathToWaypoints(path, map)
%GRIDPATHTOWAYPOINTS Convert a [row col] grid path to world xy waypoints.
%   Drops repeated cells (wait steps), which a pure pursuit controller
%   cannot use. path is K x 2 [row col]; map is the binaryOccupancyMap the
%   path was planned on.

xy = grid2world(map, path);
keep = [true; any(diff(xy) ~= 0, 2)];
waypoints = xy(keep, :);
end
