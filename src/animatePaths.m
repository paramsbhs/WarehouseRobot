function animatePaths(map, paths, titleText)
%ANIMATEPATHS Animate a set of grid paths on the warehouse map.
%   paths is an N x 1 cell array of K x 2 [row col] paths, one row per
%   timestep, planned on the grid of map.

n = numel(paths);
colors = lines(n);

figure;
show(map); hold on;
hRobots = gobjects(n, 1);
for i = 1:n
    xy = grid2world(map, paths{i}(1, :));
    goalXY = grid2world(map, paths{i}(end, :));
    plot(goalXY(1), goalXY(2), "p", MarkerSize = 12, ...
        MarkerEdgeColor = "k", MarkerFaceColor = colors(i, :));
    hRobots(i) = plot(xy(1), xy(2), "o", MarkerSize = 10, ...
        MarkerFaceColor = colors(i, :), MarkerEdgeColor = "k");
end
if nargin > 2
    title(titleText);
end

len = max(cellfun(@(p) size(p, 1), paths));
for t = 1:len
    for i = 1:n
        p = paths{i};
        xy = grid2world(map, p(min(t, size(p, 1)), :));
        set(hRobots(i), XData = xy(1), YData = xy(2));
    end
    drawnow;
    pause(0.05);
end
end
