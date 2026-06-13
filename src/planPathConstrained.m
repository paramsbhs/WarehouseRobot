function path = planPathConstrained(occ, start, goal, vertexCons, edgeCons, maxT)
%PLANPATHCONSTRAINED Space-time A* under vertex and edge constraints.
%   Low-level search for planMAPF_CBS.
%   vertexCons  K x 3 rows [r c t]         — may not occupy (r,c) at time t
%   edgeCons    K x 5 rows [r1 c1 r2 c2 t] — may not move (r1,c1)->(r2,c2)
%                                            arriving at time t
%   The robot parks on its goal after arriving, so arrival is only
%   accepted after the last vertex constraint on the goal cell. Returns a
%   K x 2 path of [row col] per timestep, or [] if none exists within maxT.

[nRows, nCols] = size(occ);
nCells = nRows * nCols;
moves = [0 0; 1 0; -1 0; 0 1; 0 -1]; % wait, down, up, right, left

% Vertex constraints as a lookup table
vtab = false(nRows, nCols, maxT);
for k = 1:size(vertexCons, 1)
    if vertexCons(k, 3) <= maxT
        vtab(vertexCons(k, 1), vertexCons(k, 2), vertexCons(k, 3)) = true;
    end
end

% Edge constraints encoded as scalars for fast membership tests
ecodes = zeros(size(edgeCons, 1), 1);
for k = 1:size(edgeCons, 1)
    c1 = sub2ind([nRows nCols], edgeCons(k, 1), edgeCons(k, 2));
    c2 = sub2ind([nRows nCols], edgeCons(k, 3), edgeCons(k, 4));
    ecodes(k) = (edgeCons(k, 5) * nCells + c1) * (nCells + 1) + c2;
end

goalConsT = vertexCons(vertexCons(:, 1) == goal(1) & ...
                       vertexCons(:, 2) == goal(2), 3);
latestGoalT = max([0; goalConsT(:)]);

key = @(r, c, t) sub2ind([nRows nCols maxT], r, c, t);

nodes = [start(1) start(2) 1 0]; % [r c t parentIdx]
open = [abs(start(1)-goal(1)) + abs(start(2)-goal(2)), 1]; % [f nodeIdx]
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

    if r == goal(1) && c == goal(2) && t > latestGoalT
        path = backtrack(nodes, nodeIdx);
        return
    end
    if t == maxT
        continue
    end

    cellFrom = sub2ind([nRows nCols], r, c);
    for m = 1:size(moves, 1)
        nr = r + moves(m, 1);
        nc = c + moves(m, 2);
        nt = t + 1;
        if nr < 1 || nr > nRows || nc < 1 || nc > nCols
            continue
        end
        if occ(nr, nc) || vtab(nr, nc, nt)
            continue
        end
        if ~isempty(ecodes)
            cellTo = sub2ind([nRows nCols], nr, nc);
            code = (nt * nCells + cellFrom) * (nCells + 1) + cellTo;
            if any(ecodes == code)
                continue
            end
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
