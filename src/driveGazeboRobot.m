function driveGazeboRobot(waypoints, opts)
%DRIVEGAZEBOROBOT Drive the Gazebo robot along a planned path from MATLAB.
%   driveGazeboRobot(waypoints) moves warehouse_robot along the N x 2 world
%   [x y] waypoints by setting its chassis pose over the Gazebo
%   co-simulation interface. No Simulink required. If waypoints is omitted,
%   it plans dock -> unloading station with initGazeboSim's settings.
%
%   This is kinematic playback: it places the robot along the path to
%   visualize planner output in the 3D simulator. Dynamic wheel/torque
%   control is the Simulink path (see docs/gazebo-setup.md).
%
%   opts fields (all optional): ip, port, speed (m/s), step (m), z.

if nargin < 2, opts = struct; end
if ~isfield(opts, "ip"),    opts.ip = "127.0.0.1"; end
if ~isfield(opts, "port"),  opts.port = 14581; end
if ~isfield(opts, "speed"), opts.speed = 0.6; end
if ~isfield(opts, "step"),  opts.step = 0.1; end
if ~isfield(opts, "z"),     opts.z = 0.1; end

if nargin < 1 || isempty(waypoints)
    map = createWarehouseMap(1);
    inflate(map, 0.6);
    planner = plannerAStarGrid(binaryOccupancyMap(occupancyMatrix(map), 1));
    pathGrid = plan(planner, world2grid(map, [4 4]), world2grid(map, [40 20]));
    waypoints = gridPathToWaypoints(pathGrid, map);
end

gzinit(opts.ip, opts.port);

% Resample the path to evenly spaced points for smooth motion
d = [0; cumsum(vecnorm(diff(waypoints), 2, 2))];
s = (0:opts.step:d(end))';
xy = interp1(d, waypoints, s);

dt = opts.step / opts.speed;
for i = 1:size(xy, 1)
    if i < size(xy, 1)
        yaw = atan2(xy(i+1,2) - xy(i,2), xy(i+1,1) - xy(i,1));
    end
    quat = [cos(yaw/2) 0 0 sin(yaw/2)];   % [w x y z], rotation about z
    [~, ~] = gzlink("set", "warehouse_robot", "chassis", ...
        "Position", [xy(i,1) xy(i,2) opts.z], "Orientation", quat);
    pause(dt);
end
fprintf("Reached goal at [%.1f %.1f].\n", xy(end,1), xy(end,2));
end
