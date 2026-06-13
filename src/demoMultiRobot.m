%% Multiple robots: coordinated planning + task scheduling
% Four robots work through a pool of pick locations. Each robot plans with
% space-time A* against a shared reservation table (prioritized planning),
% so robots wait for and route around each other. Tasks are assigned
% greedily by distance. The simulation runs in rounds: assign tasks, plan
% all paths, animate, repeat until the task pool is empty.

clear; clc;

%% Map at planner resolution (1 cell = 1 m)
map = createWarehouseMap(1);
inflate(map, 0.6);
occ = occupancyMatrix(map);
[nRows, nCols] = size(occ);

displayMap = createWarehouseMap(2); % finer map for drawing

%% Robots and tasks ([row col] grid coordinates)
starts = [ 4  4;
          11  4;
          18  4;
          26  4];
nRobots = size(starts, 1);

taskList = [ 5 15;
            11 25;
            12 35;
            18 20;
            19 40;
            26 30;
             5 38;
            26 12];
scheduler = TaskScheduler(taskList);

maxT = 300;
positions = starts;
colors = lines(nRobots);

%% Figure
figure(Name = "Coordinated multi-robot");
show(displayMap); hold on;
taskXY = grid2world(map, taskList);
plot(taskXY(:,1), taskXY(:,2), "ks", MarkerSize = 10, LineWidth = 1.5);
hRobots = gobjects(nRobots, 1);
for i = 1:nRobots
    xy = grid2world(map, positions(i, :));
    hRobots(i) = plot(xy(1), xy(2), "o", MarkerSize = 10, ...
        MarkerFaceColor = colors(i, :), MarkerEdgeColor = "k");
end
title("Multi-robot prioritized planning with task scheduling");

%% Rounds: assign tasks, plan with shared reservations, animate
roundNum = 0;
while scheduler.hasTasks()
    roundNum = roundNum + 1;
    reserved = false(nRows, nCols, maxT);
    paths = cell(nRobots, 1);

    % Unplanned robots must still be avoided at their start cells
    for i = 1:nRobots
        reserved(positions(i,1), positions(i,2), 1:2) = true;
    end

    for i = 1:nRobots
        if scheduler.hasTasks()
            goal = scheduler.assignNearest(positions(i, :));
        else
            goal = positions(i, :);
        end
        % Release this robot's own start cell so it may wait there
        reserved(positions(i,1), positions(i,2), 1:2) = false;
        p = planPathSpaceTime(occ, positions(i, :), goal, reserved);
        if isempty(p)
            warning("Robot %d found no path in round %d; it waits this round.", ...
                i, roundNum);
            p = positions(i, :);
        end
        paths{i} = p;
        reserved = addReservations(reserved, p);
    end

    % Animate the round
    roundLength = max(cellfun(@(p) size(p, 1), paths));
    for t = 1:roundLength
        for i = 1:nRobots
            p = paths{i};
            positions(i, :) = p(min(t, size(p, 1)), :);
            xy = grid2world(map, positions(i, :));
            set(hRobots(i), XData = xy(1), YData = xy(2));
        end
        drawnow;
        pause(0.05);
    end
    fprintf("Round %d complete. %d task(s) remaining.\n", ...
        roundNum, size(scheduler.Tasks, 1));
end
disp("All tasks completed.");

%% Helpers
function reserved = addReservations(reserved, path)
% Reserve each cell at t and t+1 (also rules out head-on swaps), then
% keep the final cell reserved while the robot parks on it.
maxT = size(reserved, 3);
for t = 1:size(path, 1)
    r = path(t, 1);
    c = path(t, 2);
    reserved(r, c, t) = true;
    if t + 1 <= maxT
        reserved(r, c, t + 1) = true;
    end
end
reserved(path(end, 1), path(end, 2), size(path, 1):maxT) = true;
end
