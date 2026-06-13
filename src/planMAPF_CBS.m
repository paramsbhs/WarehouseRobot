function [paths, info] = planMAPF_CBS(occ, starts, goals, maxT, maxIter)
%PLANMAPF_CBS Conflict-Based Search multi-agent path finding.
%   Complete and optimal for sum of costs (Sharon et al., 2015). Searches
%   a constraint tree: when two paths collide, the node branches into two
%   children, each forbidding the collision for one robot, and only that
%   robot replans. starts/goals are N x 2 [row col]; maxIter (default
%   2000) caps high-level expansions. Returns paths as an N x 1 cell array
%   and info with fields success, cost, makespan, expansions.

if nargin < 5
    maxIter = 2000;
end
n = size(starts, 1);

% Root node: every robot plans unconstrained
node.vcons = repmat({zeros(0, 3)}, n, 1);
node.econs = repmat({zeros(0, 5)}, n, 1);
node.paths = cell(n, 1);
for i = 1:n
    p = planPathConstrained(occ, starts(i,:), goals(i,:), ...
        node.vcons{i}, node.econs{i}, maxT);
    if isempty(p)
        paths = {};
        info = struct('success', false, 'cost', inf, 'makespan', inf, ...
            'expansions', 0);
        return
    end
    node.paths{i} = p;
end
node.cost = sumOfCosts(node.paths);

openNodes = {node};
openCosts = node.cost;

for iter = 1:maxIter
    if isempty(openNodes)
        break
    end
    [~, idx] = min(openCosts);
    node = openNodes{idx};
    openNodes(idx) = [];
    openCosts(idx) = [];

    conflict = findFirstConflict(node.paths);
    if isempty(conflict)
        paths = node.paths;
        info = struct('success', true, 'cost', node.cost, ...
            'makespan', max(cellfun(@(p) size(p,1), paths)) - 1, ...
            'expansions', iter);
        return
    end

    % Branch: forbid the conflict for each involved robot in turn
    for who = [conflict.i, conflict.j]
        child = node;
        if strcmp(conflict.type, 'vertex')
            child.vcons{who} = [child.vcons{who}; conflict.cell, conflict.t];
        else % edge (swap) conflict, oriented per robot
            if who == conflict.i
                child.econs{who} = [child.econs{who}; ...
                    conflict.fromI, conflict.toI, conflict.t];
            else
                child.econs{who} = [child.econs{who}; ...
                    conflict.toI, conflict.fromI, conflict.t];
            end
        end
        p = planPathConstrained(occ, starts(who,:), goals(who,:), ...
            child.vcons{who}, child.econs{who}, maxT);
        if ~isempty(p)
            child.paths{who} = p;
            child.cost = sumOfCosts(child.paths);
            openNodes{end+1} = child;       %#ok<AGROW>
            openCosts(end+1) = child.cost;  %#ok<AGROW>
        end
    end
end

paths = {};
info = struct('success', false, 'cost', inf, 'makespan', inf, ...
    'expansions', maxIter);
end

function c = sumOfCosts(paths)
c = sum(cellfun(@(p) size(p, 1) - 1, paths));
end

function conflict = findFirstConflict(paths)
% Earliest vertex or edge conflict among the paths. Paths are padded with
% their goal cell: a parked robot still occupies it.
n = numel(paths);
len = max(cellfun(@(p) size(p, 1), paths));
P = zeros(len, 2, n);
for i = 1:n
    p = paths{i};
    P(1:size(p,1), :, i) = p;
    P(size(p,1)+1:end, :, i) = repmat(p(end,:), len - size(p,1), 1);
end

conflict = [];
for t = 1:len
    for i = 1:n-1
        for j = i+1:n
            if all(P(t,:,i) == P(t,:,j))
                conflict = struct('type', 'vertex', 'i', i, 'j', j, ...
                    't', t, 'cell', P(t,:,i));
                return
            end
            if t > 1 && all(P(t,:,i) == P(t-1,:,j)) ...
                     && all(P(t-1,:,i) == P(t,:,j))
                conflict = struct('type', 'edge', 'i', i, 'j', j, ...
                    't', t, 'fromI', P(t-1,:,i), 'toI', P(t,:,i));
                return
            end
        end
    end
end
end
