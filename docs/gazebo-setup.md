# Gazebo co-simulation setup (Apple Silicon)

The MathWorks ROS/Gazebo virtual machine is an x86 VMware image and does not
run on Apple Silicon Macs. Instead, Gazebo 11 runs natively in an arm64 Docker
container and MATLAB/Simulink on macOS connects to it over TCP (port 14581),
which is all the co-simulation plugin needs.

```
+----------------------------+         TCP 14581         +---------------------------+
| macOS: MATLAB + Simulink   | <-----------------------> | Docker: Gazebo 11 (arm64) |
| Gazebo Pacer, planners     |                           | co-sim plugin + world     |
+----------------------------+                           +---------------------------+
```

## 1. Install desktop MATLAB

MATLAB Online cannot reach a local container, so install desktop MATLAB
(Apple Silicon version, R2023b or newer) through the UVic campus license,
with: Simulink, Robotics System Toolbox, Navigation Toolbox.

## 2. Package the co-simulation plugin source  (done — regenerate if MATLAB updates)

The plugin is not shipped as a ready-made zip; MATLAB generates it from
source with `packageGazeboPlugin`, which writes `GazeboPlugin.zip` to the
current directory. From the `gazebo/` folder:

```matlab
cd gazebo
packageGazeboPlugin
```

This was already run for R2026a; `gazebo/GazeboPlugin.zip` exists (and is
git-ignored, since the plugin source is MathWorks-copyrighted). Regenerate it
if you upgrade MATLAB, since the plugin must match the desktop install that
drives the co-simulation.

## 3. Generate the warehouse world  (done — regenerate if the map changes)

From `src/`:

```matlab
exportGazeboWorld
```

This writes `gazebo/worlds/warehouse.world` with the same geometry as
`createWarehouseMap` (so paths planned on the occupancy map are valid in
Gazebo), an inline sun and ground plane (no online model-database fetch), the
co-sim plugin on port 14581, and a differential-drive robot (`warehouse_robot`
with `left_wheel_joint` / `right_wheel_joint` and a 640-sample lidar).
Re-run it whenever you change `createWarehouseMap`.

## 4. Start Gazebo  (image already built and verified)

```bash
./gazebo/run_gazebo.sh
```

The image is already built and the stack verified: the plugin compiles for
arm64, the world loads with all models, and the co-sim server listens on
`localhost:14581` (reachable from the macOS host). The build takes a few
minutes the first time; if you ever rebuild from scratch, note:

- The plugin must build single-threaded (`make -j1`, already set in the
  Dockerfile). Parallel `make` exhausts 16 GB of RAM and Docker kills a
  compiler, surfacing as a misleading "make Error 2".
- To re-verify the running container end-to-end:
  ```bash
  docker logs warehouse-gazebo 2>&1 | grep -i error      # expect none (ignore render warnings)
  nc -z localhost 14581 && echo reachable                # co-sim port up
  docker exec warehouse-gazebo gz model --model-name warehouse_robot --info | head
  ```
- `libGazeboCoSimPlugin.so` lives at `/opt/GazeboPlugin/export/lib/` inside
  the image.

The "Can't open display / render engine NONE" lines in the log are expected:
`gzserver` is headless and co-simulation needs no rendering.

The container is headless. To watch the simulation, either rely on the
MATLAB-side plots, or install XQuartz, enable "Allow connections from network
clients", run `xhost +localhost`, then:

```bash
docker exec -it warehouse-gazebo bash -c \
  "DISPLAY=host.docker.internal:0 LIBGL_ALWAYS_SOFTWARE=1 gzclient"
```

Note: while the world is loading, Gazebo in co-simulation mode **pauses and
waits for a co-sim client to connect** before advancing. So a log that stops
at `Loading world file [...]` with no further output is normal — it means
"loaded and ready", not "hung". Confirm with `nc -z localhost 14581`.

## 5. Recommended path: drive the robot from MATLAB (no Simulink)

The Robotics System Toolbox Gazebo co-simulation **MATLAB API** (`gzinit`,
`gzlink`, `gzjoint`, `gzmodel`) connects to the same port 14581 and needs no
Simulink at all. This is the quickest way to see the robot move in the
warehouse. Verified working against the Docker container.

```matlab
cd src
driveGazeboRobot          % plans dock -> unloading and drives the robot there
```

`driveGazeboRobot(waypoints, opts)` connects with `gzinit`, then steps the
robot along the planned path by setting its chassis pose with `gzlink("set",
...)`. Pass `waypoints` from any planner (e.g.
`gridPathToWaypoints(paths{1}, map)` for a CBS solution) to drive whatever
path you like.

This is **kinematic playback**: it places the robot along the path to
visualize planner output in the 3D simulator. It does not simulate wheel
torque or contact dynamics — that is the Simulink path in section 7, which is
optional and only needed if dynamic control is a project goal.

### Watching it (the container is headless)

To see the 3D view, render it with software OpenGL inside the container and
stream it over VNC. **Do not use XQuartz** — Gazebo's OGRE renderer needs a
GLX context that XQuartz's indirect GLX cannot provide on Apple Silicon
(fails with `GLXBadContext`). Software-GL-over-VNC is the route that works.

The run script already maps port 5900. With the container running:

```bash
./gazebo/view_gazebo.sh        # starts Xvfb + gzclient (software GL) + x11vnc
open vnc://localhost:5900      # macOS built-in VNC viewer; password: warehouse
```

macOS Screen Sharing refuses no-auth VNC servers, so the view is
password-protected (password `warehouse`, set by `view_gazebo.sh`).

Then run `driveGazeboRobot` in MATLAB and watch the robot move in the VNC
window. (The default camera may not frame the robot; drag in the view or use
the Gazebo camera controls. The gzclient window may not fill the VNC screen —
resize it in the viewer.)

If you skip the viewer, `driveGazeboRobot` still runs; verify motion by
reading the pose back with `gzlink("get", "warehouse_robot", "chassis",
"Position")` or by the 2D path plot from `initGazeboSim`.

## 5b. (Optional) Smoke-test the connection from Simulink

Only needed if you take the Simulink path in section 7. Following the
"Perform Co-Simulation Between Simulink and Gazebo" example: add a **Gazebo
Pacer** block to an empty model, click "Configure Gazebo network and
simulation settings", set Network Address to Custom, Hostname/IP to
`127.0.0.1`, port `14581`, and Test.

## 6. Initialize the workspace

Run `initGazeboSim` (from `src/`) before opening or simulating the model. It
populates the base workspace with every value the model references, so no
coordinate is typed into the GUI:

- connection: `gazeboIP`, `gazeboPort`
- robot: `robotName`, `chassisLink`, `leftJoint`, `rightJoint`, `lidarTopic`,
  `wheelRadius`, `wheelSeparation`
- planned path: `waypoints` (N x 2 world [x y]), plus `map`, `occ`
- tuning: `desiredLinearVelocity`, `maxAngularVelocity`, `lookaheadDistance`,
  `goalRadius`, `avoidCollisionDistance`

Everything is anchored to the Gazebo world frame (x in [0 50], y in [0 30] m),
the same frame `createWarehouseMap` and `exportGazeboWorld` use, so a path
planned on the map is valid in Gazebo without any offset. The robot's start
pose in `exportGazeboWorld` (4, 4) matches `robotStartWorld`.

## 7. (Optional) Dynamic control loop in Simulink

Only needed if simulating wheel/contact dynamics is a project goal; the
section 5 MATLAB path already gives a moving robot in the warehouse. This
builds a physics control loop where Simulink's Gazebo Pacer steps the
simulation and reads/writes the robot each step.

Recommended control strategy: **velocity control**, not the torque control the
documentation example uses. The example's Apply Joint Torque + Pioneer Wheel
Control needs a PID tuned to the Pioneer's mass; our 10 kg robot with simple
inertias is far easier to drive by commanding wheel velocities directly. The
co-sim plugin supports SetJointVelocity (verified built into the image).

Build this loop in a new model (blocks from the *Robotics System Toolbox >
Gazebo Co-Simulation* and *Navigation Toolbox* libraries). Exact dialog labels
vary slightly by release; use each block's Select/Configure dialog.

1. **Gazebo Pacer** — one per model. Configure network: Custom, `gazeboIP`,
   `gazeboPort`. This steps Gazebo in lockstep with Simulink. Set a fixed-step
   solver; match the sample time to the Pacer step (e.g. 0.01 s).

2. **Sense — robot pose.** *Gazebo Read* configured to read the `chassisLink`
   link state of `robotName`. Take the position (x, y) and orientation
   quaternion; convert the quaternion to yaw with a *Quaternion to Euler*
   block. Assemble `pose = [x; y; yaw]`.

3. **Sense — lidar.** *Gazebo Read* (or Read Lidar Scan) subscribed to
   `lidarTopic`. Output is 640 ranges over [-2.356, 2.356] rad.

4. **Control — Pure Pursuit.** *Pure Pursuit* block (Navigation Toolbox).
   Inputs: `pose` and `waypoints`. Parameters: `desiredLinearVelocity`,
   `maxAngularVelocity`, `lookaheadDistance`. Outputs `v` and `omega`.

5. **Obstacle stop.** Take the lidar ranges in a forward window
   (about [-pi/10, pi/10], the center ~64 samples). If any finite range is
   below `avoidCollisionDistance`, force `v = 0` (a Switch block gated by a
   MATLAB Function or `min(...) < avoidCollisionDistance` comparison). This is
   the example's "Stop Robot On Sensing Obstacles" logic.

6. **Convert to wheel speeds.** A *MATLAB Function* block calling
   `diffDriveWheelSpeeds(v, omega, wheelRadius, wheelSeparation)`, returning
   `wLeft`, `wRight`. If the robot drives backward, negate both (the wheel
   joint axis sign depends on the SDF).

7. **Actuate.** Two *Gazebo Apply Command* blocks configured as
   **SetJointVelocity**: one for `leftJoint` (= `wLeft`), one for `rightJoint`
   (= `wRight`), both on model `robotName`. Use *Gazebo Blank Message* to
   create each command struct and *Gazebo Select Entity* to pick the joint.

8. **Stop at goal.** Compute `norm(pose(1:2) - waypoints(end,:))`; when it
   drops below `goalRadius`, stop the sim (a *Stop Simulation* block) or zero
   the velocities.

## 8. Run and verify

1. Start the container: `./gazebo/run_gazebo.sh`
2. In MATLAB: `cd src; initGazeboSim`
3. Open the model and press Run. The robot should drive along `waypoints`
   from (4, 4) toward the unloading station, stopping if something enters the
   2 m forward window.

If the robot drives the wrong way, negate the wheel speeds (step 6). If it
spins in place, swap `wLeft`/`wRight`. If it never moves, check the Pacer
test passed and that Apply Command targets the right joint names.

## 9. Multi-robot extension

Once one robot works end-to-end:

- In `exportGazeboWorld`, emit several `<model>` blocks with distinct names
  (`warehouse_robot_1`, ...) and start poses, by calling `robotSDF` in a loop.
- In Simulink, replicate the Sense -> Control -> Actuate chain per robot (or
  make it a subsystem and use a For Each / Model Reference per robot).
- Feed each robot the corresponding `paths{i}` from `planMAPF_CBS`, converted
  with `gridPathToWaypoints`:

  ```matlab
  [paths, info] = planMAPF_CBS(occ, starts, goals, 300);
  wp1 = gridPathToWaypoints(paths{1}, map);
  ```

Get one robot fully working before adding the rest.

## Fallback

If the Docker route stalls, the official VM runs fine on any x86 Windows/Linux
machine (e.g., a UVic lab PC) with VMware Workstation Player, with MATLAB on
the same machine following the example as written.
