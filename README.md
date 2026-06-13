# Warehouse Robotics Simulation

Solution to MATLAB and Simulink Challenge project 212, Warehouse Robotics Simulation.

[Program link](https://github.com/mathworks/MATLAB-Simulink-Challenge-Project-Hub)

[Project description link](https://github.com/mathworks/MATLAB-Simulink-Challenge-Project-Hub/tree/main/projects/Warehouse%20Robotics%20Simulation)

## Project details

This project simulates multi-robot warehouse operations in MATLAB: path planning, multi-robot coordination, and task scheduling on an occupancy map of a 50 m x 30 m warehouse with shelf racks and aisles.

The work is built up in three stages:

1. **Single-robot planning.** A path is planned on the inflated occupancy map with grid A* (`plannerAStarGrid`) and followed by a unicycle-model robot using a pure pursuit controller, following the approach of the MathWorks warehouse path planning example.

2. **Multi-robot coordination with task scheduling.** Multiple robots serve a pool of pick locations. Coordination uses prioritized planning: each robot plans with A* in space-time (x, y, time), where waiting is a legal move, and treats the paths of robots that planned before it as moving obstacles via a shared reservation table. Cells are reserved for two consecutive timesteps to rule out head-on swaps. Tasks are assigned greedily to the nearest robot.

3. **Optimal multi-agent path finding.** Prioritized planning is fast but incomplete and suboptimal: a fixed priority order can block a solvable instance. Conflict-Based Search (CBS) [2] is implemented as a complete, optimal alternative: a high-level search over a constraint tree branches whenever two paths collide, and only the constrained robot replans at the low level (space-time A* under vertex and edge constraints). A benchmark script compares both planners on identical random instances for 2 to 8 robots, measuring success rate, sum of costs, makespan, and planning time.

Source files (`src/`):

| File | Purpose |
|---|---|
| `createWarehouseMap.m` | Occupancy map of the warehouse (walls, shelf racks, aisles) |
| `demoSingleRobot.m` | Stage 1: A* planning + pure pursuit path following |
| `planPathSpaceTime.m` | Space-time A* against a reservation table |
| `TaskScheduler.m` | Greedy nearest-task assignment |
| `demoMultiRobot.m` | Stage 2: coordinated robots serving a task pool |
| `planMAPF_Prioritized.m` | Prioritized planner as a function (benchmark baseline) |
| `planPathConstrained.m` | Space-time A* under vertex/edge constraints (CBS low level) |
| `planMAPF_CBS.m` | Conflict-Based Search (complete, optimal) |
| `animatePaths.m` | Animate planned paths on the map |
| `benchmarkPlanners.m` | Stage 3: prioritized vs CBS comparison |

## How to run

Requirements:

- MATLAB R2023a or newer
- Navigation Toolbox
- Robotics System Toolbox

Steps:

```matlab
cd src
demoSingleRobot     % single robot: A* + pure pursuit
demoMultiRobot      % four robots, coordinated planning + task scheduling
benchmarkPlanners   % prioritized vs CBS benchmark (takes a few minutes)
```

Each script is self-contained and uses relative paths only. `demoSingleRobot` and `demoMultiRobot` open an animated figure; `benchmarkPlanners` prints a summary table, produces comparison plots, and ends with an animated CBS solution.

## Demo/Results

<!-- Record short screen captures of demoSingleRobot, demoMultiRobot, and the
     benchmark plots, save them under docs/, and embed them here, e.g.:
     ![Multi-robot demo](docs/multirobot.gif) -->

Expected results:

- `demoSingleRobot`: the robot follows a collision-free A* path from the dock to the far corner of the warehouse.
- `demoMultiRobot`: four robots clear eight pick tasks over several rounds, waiting for and routing around each other in the aisles; the command window reports tasks remaining per round.
- `benchmarkPlanners`: CBS matches or beats the prioritized planner on sum of costs on every instance (it is optimal), while its planning time grows much faster with robot count; the prioritized planner occasionally fails on instances CBS solves.

## Reference

[1] G. Wagner and H. Choset, "M*: A complete multirobot path planning algorithm with performance bounds," IROS 2011.

[2] G. Sharon, R. Stern, A. Felner, and N. Sturtevant, "Conflict-based search for optimal multi-agent pathfinding," Artificial Intelligence, vol. 219, 2015.

[3] MathWorks, "Path Planning and Obstacle Avoidance in a Warehouse," Navigation Toolbox example.

## Contact

Param Singh — params@uvic.ca
