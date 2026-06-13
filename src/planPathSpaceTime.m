function path = planPathSpaceTime(occ, start, goal, reserved)
%PLANPATHSPACETIME Grid A* in (row, col, time) honoring reservations.
%   occ       logical occupancy grid (true = obstacle)
%   start     [row col]
%   goal      [row col]
%   reserved  nRows x nCols x maxTime logical; true = cell taken at time t
%
%   The robot moves one 4-connected cell per timestep or waits in place.
%   Returns a K x 2 path of [row col], one row per timestep, or [] if no
%   path exists within the time horizon.

[nRows, nCols, maxT] = size(reserved);
moves = [0 0; 1 0; -1 0; 0 1; 0 -1]; % wait, down, up, right, left

key = @(r, c, t) sub2ind([nRows nCols maxT], r, c, t);

nodes = [start(1) start(2) 1 0]; % [r c t parentIdx]
hStart = abs(start(1)-goal(1)) + abs(start(2)-goal(2));
open = [hStart 1];               % [f nodeIdx]

gScore = inf(nRows*nCols*maxT, 1);
gScore(key(start(1), start(2), 1)) = 0;
closed = false(nRows*nCols*maxT, 1);

path = [];
while ~isempty(open)
    [~, i] = min(open(:, 1));
    nodeIdx = open(i, 2);
    open(i, :) = [];

    r = nodes(nodeIdx, 1);
    c = nodes(nodeIdx, 2);
    t = nodes(nodeIdx, 3);
    k = key(r, c, t);
    if closed(k)
        continue
    end
    closed(k) = true;

    if r == goal(1) && c == goal(2)
        path = backtrack(nodes, nodeIdx);
        return
    end
    if t == maxT
        continue
    end

    for m = 1:size(moves, 1)
        nr = r + moves(m, 1);
        nc = c + moves(m, 2);
        nt = t + 1;
        if nr < 1 || nr > nRows || nc < 1 || nc > nCols
            continue
        end
        if occ(nr, nc) || reserved(nr, nc, nt)
            continue
        end
        nk = key(nr, nc, nt);
        ng = gScore(k) + 1;
        if ng < gScore(nk)
            gScore(nk) = ng;
            nodes(end+1, :) = [nr nc nt nodeIdx];                  %#ok<AGROW>
            h = abs(nr-goal(1)) + abs(nc-goal(2));
            open(end+1, :) = [ng + h, size(nodes, 1)];             %#ok<AGROW>
        end
    end
end
end

function path = backtrack(nodes, idx)
path = zeros(0, 2);
while idx > 0
    path = [nodes(idx, 1:2); path]; %#ok<AGROW>
    idx = nodes(idx, 4);
end
end
