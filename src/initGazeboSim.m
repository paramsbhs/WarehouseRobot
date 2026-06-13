%% Initialize the Gazebo co-simulation: parameters, map, planned path
% Run this before simulating the Simulink model. It populates the base
% workspace with everything the model's blocks reference, so no coordinates
% are hand-typed in the GUI. Everything is anchored to the Gazebo world
% frame (x in [0 50], y in [0 30] meters), the same frame
% createWarehouseMap and exportGazeboWorld use.

clear; clc;

%% Gazebo connection
gazeboIP   = "127.0.0.1";   % Docker container reachable on the host
gazeboPort = 14581;

%% Robot model (must match exportGazeboWorld.m)
robotName        = "warehouse_robot";
chassisLink      = "chassis";
leftJoint        = "left_wheel_joint";
rightJoint       = "right_wheel_joint";
lidarTopic       = "/gazebo/default/warehouse_robot/lidar/lidar/scan";
wheelRadius      = 0.1;     % m
wheelSeparation  = 0.34;    % m (wheels at y = +/-0.17)

%% Map (1 cell = 1 m, matching the Gazebo world geometry)
map = createWarehouseMap(1);
inflate(map, 0.6);          % robot radius + margin
occ = occupancyMatrix(map);

%% Stations in WORLD meters (must match the Gazebo robot start pose)
robotStartWorld = [4 4];    % exportGazeboWorld places the robot here
chargingStn     = [4 4];
loadingStn      = [26 4];
unloadingStn    = [40 20];

%% Plan one delivery path: dock -> unloading station
planner   = plannerAStarGrid(binaryOccupancyMap(occ, 1));
startGrid = world2grid(map, robotStartWorld);
goalGrid  = world2grid(map, unloadingStn);
pathGrid  = plan(planner, startGrid, goalGrid);
waypoints = gridPathToWaypoints(pathGrid, map);   % N x 2 world [x y]

%% Pure pursuit / control tuning
desiredLinearVelocity = 0.6;    % m/s
maxAngularVelocity    = 1.5;    % rad/s
lookaheadDistance     = 0.8;    % m
goalRadius            = 0.4;    % m, stop when this close to the goal

%% Obstacle stop (matches the warehouse example's logic)
avoidCollisionDistance = 2.0;   % m

fprintf("Init done. %d waypoints from [%.1f %.1f] to [%.1f %.1f].\n", ...
    size(waypoints, 1), robotStartWorld, unloadingStn);
figure; show(map); hold on;
plot(waypoints(:,1), waypoints(:,2), "b.-");
title("Planned path fed to Gazebo co-simulation");
