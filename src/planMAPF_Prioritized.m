function [paths, info] = planMAPF_Prioritized(occ, starts, goals, maxT)
%PLANMAPF_PRIORITIZED Prioritized multi-robot planning.
%   Robot 1 plans first; each later robot avoids all earlier reservations.
%   Fast, but neither complete nor optimal. Returns paths as an N x 1 cell
%   array and info with fields success, cost, makespan, failedRobot.

[nRows, nCols] = size(occ);
n = size(starts, 1);
reserved = false(nRows, nCols, maxT);

% Unplanned robots must still be avoided at their start cells
for i = 1:n
    reserved(starts(i,1), starts(i,2), 1:2) = true;
end

paths = cell(n, 1);
for i = 1:n
    % Release this robot's own start cell so it may wait there
    reserved(starts(i,1), starts(i,2), 1:2) = false;
    p = planPathSpaceTime(occ, starts(i,:), goals(i,:), reserved);
    if isempty(p)
        paths = {};
        info = struct('success', false, 'cost', inf, 'makespan', inf, ...
            'failedRobot', i);
        return
    end
    paths{i} = p;

    % Reserve at t and t+1 (rules out swaps), then park at the final cell
    for t = 1:size(p, 1)
        reserved(p(t,1), p(t,2), t) = true;
        if t + 1 <= maxT
            reserved(p(t,1), p(t,2), t+1) = true;
        end
    end
    reserved(p(end,1), p(end,2), size(p,1):maxT) = true;
end

info = struct('success', true, ...
    'cost', sum(cellfun(@(p) size(p,1) - 1, paths)), ...
    'makespan', max(cellfun(@(p) size(p,1), paths)) - 1, ...
    'failedRobot', []);
end
