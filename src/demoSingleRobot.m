%% Single robot: A* path planning and path following
% Plans a path through the warehouse with grid A* and follows it with a
% pure pursuit controller on a unicycle model.

clear; clc;

%% Map and planner
map = createWarehouseMap();

robotRadius = 0.4;
inflatedMap = copy(map);
inflate(inflatedMap, robotRadius);

planner = plannerAStarGrid(inflatedMap);

startXY = [4 4];
goalXY  = [46 26];

pathGrid  = plan(planner, world2grid(inflatedMap, startXY), ...
                          world2grid(inflatedMap, goalXY));
pathWorld = grid2world(inflatedMap, pathGrid);

%% Path follower
controller = controllerPurePursuit( ...
    Waypoints = pathWorld, ...
    DesiredLinearVelocity = 1.0, ...
    MaxAngularVelocity = 2.0, ...
    LookaheadDistance = 0.8);

%% Simulate
pose = [startXY 0];
dt = 0.1;
goalRadius = 0.5;

figure(Name = "Single-robot A*");
show(map); hold on;
plot(pathWorld(:,1), pathWorld(:,2), "b-", LineWidth = 1.5);
plot(goalXY(1), goalXY(2), "gp", MarkerSize = 12, MarkerFaceColor = "g");
hRobot = plot(pose(1), pose(2), "ro", MarkerSize = 8, MarkerFaceColor = "r");
title("Single robot following an A* path");

while norm(pose(1:2) - goalXY) > goalRadius
    [v, w] = controller(pose');
    pose = pose + dt * [v*cos(pose(3)), v*sin(pose(3)), w];
    set(hRobot, XData = pose(1), YData = pose(2));
    drawnow limitrate;
end
disp("Goal reached.");
