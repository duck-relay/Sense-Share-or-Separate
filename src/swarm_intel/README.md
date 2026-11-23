# swarm_intel — Drone navigation study (Gazebo Fortress + ROS 2 Humble)

This is the only package in the workspace.

| Folder | Contents |
|---|---|
| `src/swarm_intel` | the code |
| `results/` | runs and studies (see `results/README.md`) |
| `build/`, `install/`, `log/` | made by colcon |

## The scenario

Five quadrotors (1.5 kg, four 0.135 m rotors, 0.52 m from the centre to the rotor tips) fly through a 20 × 20 m forest.

- **Five equal drones, one system:** each flies from its own pad to its own goal pad at the same cruise speed (1.2 m/s), and every route crosses others. All five use **the same navigation configuration**, none has priority over the others, and **every metric covers the whole fleet** (energy and flight time summed over the five drones, errors averaged over them, separation and downwash of the closest pair; section 6). The reports also list every drone.
- **Colours** only tell the drones apart, in Gazebo, the dashboard and the plots: drone1 orange, drone2 violet, drone3 aqua, drone4 plum, drone5 cerulean (a palette checked for colour-vision deficiencies).
- **Every route is computed on the drones (edge computing):** each drone runs as its own ROS 2 node (its on-board computer) and plans every route itself; nothing is pre-computed. In Gazebo, for **every drone**:
  - **green** = its **reference route**: A*, planned on board on the pad before take-off (the shortest route on its map, with the known trees), drawn as the whole flight: up from the take-off pad to cruise altitude, along the route, down to the landing pad;
  - **its own colour** (the colour of its body and landing pad) = the **route it has actually flown**: its trail, drawn as it flies, from the take-off pad on the ground up to where it is now, and in the end down to its landing pad (Gazebo's true position, a point every 0.15 m). The route it follows is planned on board in flight with RRT* (or A*, `planner:=astar`) and **re-planned every second**; the trail shows where that took it, next to the green route it is measured against.
  - Each drone's computer runs its planner as a **separate process next to its flight loop** (like a companion computer next to the autopilot): while a route is computed, the drone keeps flying its current route at the full control rate, and the real planning time of the drone's computer shows in the flight.
- **Trees:** the 20 landmark trees are known and carry ArUco markers on all 4 sides. The 6 dark pines are **unknown** obstacles that have to be discovered in flight.
- **Forests:** the Gazebo forest (layout 0) plus 9 random forests with the same counts and spacing as layout 0 (headless studies, `--layouts 10`): three of the unknown pines on the straight routes of three different drones (drawn at random), every pad at least 1.4 m from a tree and 2.2 m from a pine, and every drone's shortest route at most 1.6 × its straight line.

The study compares **12 configurations** with the **baseline: 3-point camera + VIO (C3+V)** on energy, accuracy, localisation, time, safety and fleet metrics.

| `nav:=` | Code | Configuration | Camera fix | VIO | Coordination |
|---|---|---|---|---|---|
| `cam3_sensor` | **C3+V** | **Baseline: 3-point camera + VIO** | 3 markers in one frame | ✓ | no radio |
| `cam3` | C3 | 3-point camera | 3 | – | no radio |
| `cam2` | C2 | 2-point camera | 2 | – | no radio |
| `cam2_sensor` | C2+V | 2-point camera + VIO | 2 | ✓ | no radio |
| `sensor` | V | VIO only | – | ✓ | no radio |
| `cam3_sensor_layers` | C3+V+L | Baseline + fixed altitude layers | 3 | ✓ | fixed layers, no radio |
| `cam3_sensor_swarm` | C3+V+W | 3-point camera + VIO + swarm | 3 | ✓ | swarm radio |
| `cam3_swarm` | C3+W | 3-point camera + swarm | 3 | – | swarm radio |
| `cam2_swarm` | C2+W | 2-point camera + swarm | 2 | – | swarm radio |
| `all` | C2+V+W | All-in-one: 2-point camera + VIO + swarm | 2 | ✓ | swarm radio |
| `sensor_swarm` | V+W | VIO + swarm | – | ✓ | swarm radio |
| `swarm` | W | Swarm only | – | – | swarm radio |

**V** is the drone's velocity from visual-inertial odometry (VIO). The `nav:=` keys keep their original names (`sensor` = V), and study reports made before version 1.3.2 call it *sensor data*.

The headless study can also fly **mixed fleets** (`--mixed`): *k* of the 5 drones carry the 3-point camera + VIO and the others VIO only, with or without swarm radio.

## How a drone navigates: every measurement goes through its EKF first

### Everything runs on the drone (edge computing)

In Gazebo every drone is a separate ROS 2 node (`drone_node.py`, one per drone): its on-board computer. It runs the drone's EKF, its map, all of its path planning (the take-off reference route and every in-flight plan), its part of the turn trading and its control, and nothing else computes routes for it. A **world node** (`world_node.py`) plays the part of the physical world: it applies the drones' commands with the true heading, wind and downwash, simulates their sensors from the true state, scores the run and draws the routes the drones send. The drones talk to each other only over the simulated radio (`/swarm/radio`).

| Topic (std_msgs/String, JSON) | From → to | Content |
|---|---|---|
| `/swarm/truth` | world → every drone | true state of every drone, each tick (feeds only the simulated sensors and the scoring) |
| `/swarm/radio` | drone → the other drones | radio broadcast: EKF position, uncertainty, intents, new depth scans (shared map), turn-trading messages |
| `/<drone>/cmd` | drone → world | air velocity in the drone's own frame, yaw rate, state |
| `/<drone>/plans` | drone → world (latched) | its reference route and its current route, each time it plans |
| `/<drone>/status` | drone → world (latched) | telemetry, summary and log, 5 Hz and on landing |

A drone acts on the radio messages of the previous tick (a 0.05 s link delay). Planning takes real time on the drone's computer: while it plans, the world moves on and the drone keeps flying its last command; when the computer is back, its filter and its energy count cover the ticks it missed. (The headless study steps drones and world together, so planning takes no simulated time there; the measured planning time is reported.) Each drone's energy and flight time run from take-off to touchdown: the reference route is planned on the pad with the motors off and is in neither. The headless study runs the same per-drone code (`mission.DroneRuntime`) and the same world model (`mission.WorldSim`) in one process.

### The EKF

Every drone runs one **extended Kalman filter** (`estimator.py`). Each measurement is fused by the EKF first. Guidance, control, mapping and collision avoidance only ever use the EKF's outputs. Ground truth from Gazebo is used only to *simulate* the sensors and to *score* the run.

| | |
|---|---|
| **EKF state** | position x, y, z · heading · wind (2-D) · sensor-data bias (2-D) |
| **Prediction** | the drone's own air-velocity command (with the airframe's 0.06 s response lag), plus the estimated wind once airborne; heading from the gyro |
| **Always on, every configuration** | IMU (gyro, attitude, magnetometer heading), ToF altimeter, forward depth camera (90°, 4.5 m), a short-range ToF ring that sees obstacles all around (360°, 2 m), and proximity sensing of other drones (3 m around, ±45° up/down) |
| **N-point camera** | ArUco markers on the known trees, seen by a **360° navigation camera ring** (4 × 90°, range 6 m, line of sight checked in 3-D). A fix needs N markers in the same frame; the EKF fuses each marker's body-frame bearing and PnP range. After a long gap without a fix (EKF 1σ > 0.3 m) the whole frame is fused as **one joint iterated update**, so the filter cannot lock onto a wrong, over-confident position. |
| **VIO** | IMU + visual-inertial odometry measure the drone's real velocity (with a slowly drifting bias). On the pad the drone calibrates the bias; in flight the EKF uses it to estimate the wind. If the measured velocity keeps disagreeing with the model for 0.2 s (the drone is pushed by downwash or held by an obstacle), the wind state takes the difference up instead of the position estimate drifting on. |
| **Swarm radio** | see below |

The **swarm radio** combines:

- **UWB peer-to-peer ranges**, fused at the peer's broadcast EKF estimate, with that peer's uncertainty added. A drone only uses peers that are clearly better localised than itself, and inflates their noise for correlation.
- **A shared obstacle map.** Every drone keeps its own copy and broadcasts its newest depth scans; the others add them to their copies. A drone only shares scans while its EKF says it is well localised.
- **Give-way by turn trading (TTC, `ttc.py`), for the lowest energy of the whole fleet; no drone has priority.** Every drone broadcasts its *intent* (where it will be over the next 10 s at its current speed) and its *route intent* (where it would be if it simply flew on along its route). Every 0.5 s the drones whose route intents would come closer than the separation standard (below) + 0.5 m form a conflict cluster and settle who passes first:
  1. **Endowment:** the passage turns (1st, 2nd, …) are owned first-come-first-served (a cluster keeps its last order once it has traded). Drones that would reach the conflict in the same 0.5 s are ordered by a draw that every drone computes alike from the round number, so no drone number is favoured.
  2. **Prices:** each drone prices each turn it could hold: the extra energy of its cheapest way to stay clear of the drones that would pass before it — **hold or slow down** (the shortest sufficient delay), **step left or right** (0.6–2.4 m off its route, where its map is free), or **climb / descend** 0.6 m — as flight time lost × its power (+ the climb). It prices on board, from its own route, map and energy model.
  3. **Trading:** the drones trade turns along cycles (each takes the turn of the next drone in its cycle) whenever that lowers the cluster's total energy by more than 100 J. With at most five drones every order is evaluated, so the cluster flies its lowest-energy order. Any drone may pass first; a drone that cannot get out of the way is very expensive behind others and so passes first. The trades follow trading cycles as in top trading cycles (Shapley & Scarf), but the objective differs: classic TTC never leaves a trader worse off than its own turn, while here a drone gives up its turn whenever that saves the fleet energy (the fleet's total, not each drone's own, is what is minimised).

  Every drone runs the same computation on the same broadcast and reaches the same order. Each drone then gives way only to the drones before it, every tick choosing the manoeuvre of least energy (continue, slow, hold, left, right, up, down) that keeps the separation standard + 0.5 m to their intents and does not lead into an obstacle; it never cuts into the drones after it (1.5 s look-ahead). Under the downwash-aware standard climbing or descending never resolves a conflict (the required distance only grows with the height difference), so the drones give way in the plane. A drone that cannot get out of the way reports itself *boxed in*, and the others go round it.

  **On a real link** (package 1.8.5): every message carries its send time. Each drone measures the delay of the messages it receives (the largest over the last 2 s) and reports it; all drones space the three phases of a round by the largest delay any drone reports plus one step (at least 0.1 s; a round lasts at least 0.5 s). A drone whose round message from another drone did not arrive in time avoids that drone reactively, with the 0.5 m buffer, until a round with it succeeds; positions in late messages are moved on by their age, and messages older than 1 s are not used. On an ideal link this changes nothing (the missions are bit-identical; `test/test_radio.py`). `study --ttc-fixed` flies the original fixed schedule without the fallback.
- **Fixed-priority give-way (comparison):** `study --give-way priority` / `launch … give_way:=priority`: the earlier rule, in which the drones give way in a fixed order (drone1 first, then drone2, …) (waiting first, stepping aside only when they must).
- **Power cost:** +2.5 W per drone.

**Without radio,** a drone notices other drones only with its depth camera / proximity sensing (within 3 m, not straight above or below) and tracks each one with a small Kalman filter. It avoids them reactively (repulsion + sideways curl) and with braking barriers.

**Fixed altitude layers (C3+V+L, no radio):** every drone cruises on its own layer (drone1 at 1.5 m, drones 2–5 at 2.3, 2.9, 3.5, 4.1 m). A simple form of layered airspace, with one layer per drone so that no radio is needed. With downwash modelled it does **not** separate them (see below); with drones that see each other up to 90° above and below (`study --drone-view-deg 90`) it is as safe as flying at one height, and no safer.

The camera option `--cam-fov 80` (or `cam_fov_deg = 80` in `config.py`) models only your single forward 80° camera: 3 markers are visible together in about 1 % of the frames.

### Path planning: RRT* (A* for comparison)

Every drone plans its own route with **RRT*** (`rrt_star.py`) on its occupancy map (0.2 m cells; obstacles inflated by the drone's radius + 0.18 m, and by another 0.15 m when a route is first planned, so small map updates do not cause endless re-planning). The tree grows from the drone's position by sampling the free space uniformly (a point of the map, redrawn while it falls on an obstacle; 10 % of the samples at the goal), connects each new node to the neighbour that gives it the shortest route, and rewires the neighbours through it when that is shorter (radius min(1.5 m, γ√(ln n / n)), γ above the bound for asymptotic optimality). Once a route to the goal exists, samples come only from the ellipse of points that can still shorten it (informed sampling). The budget is 1 000 samples per 400 m² of map (the 20 × 20 m field: 1 000), up to 6 000 per 400 m² while no route has been found. Sampling alone cannot tell that there is no route: a start or goal walled into a small pocket is recognised before planning (a bounded flood fill, the same for both planners), and when RRT*'s regular budget ends without a route it checks once whether one exists at all (two flood fills that stop where they meet) and, if not, reports so at once. The route is then pruned with line-of-sight shortcuts. A segment counts as free only if every grid cell it touches is free, and where it passes exactly through a cell corner both cells beside it must be free, so no route slips between two diagonally touching obstacles (the self-test checks this against an exact reference; the answer is the same in both directions). Nothing in RRT* visits every grid cell, so its planning time does not grow with finer grid cells the way A*'s does (see *Planner scaling* below).

**Rolling re-planning.** In flight every drone plans again **every second** from where it is to its pad, on its current map (`REPLAN_PERIOD`), and at once when a newly mapped obstacle blocks its route or it is pushed far off its route and cannot fly straight back. It switches to the new route only if that is shorter than what is left of its current route by 3 % (at least 0.25 m), or the current route is blocked, so RRT*'s random variation never makes a drone wander; A* runs under the same rule. Every route computed counts as on-board planning (planning time, routes computed); a switch to a new route counts as a re-plan. In the headless twin a route is ready in the control step that asks for it (no simulated time passes, so every mission is deterministic; its planning time is still measured). In the ROS 2 drone node (Gazebo) a separate planner process computes it while the flight loop keeps running, and the drone takes it when it is ready (`onboard_planner.py`). Each drone draws its samples from its own random stream, seeded by the mission, the forest and the drone: a mission replays exactly, and the sensor noise does not depend on how often a drone plans.

**A*** on the 8-connected grid (octile heuristic, no corner cutting, the same pruning) is kept for comparison: `study --planner astar`, `launch … planner:=astar`, `gazebo_study --planner astar`, or `p` in the menu.

**Comparing the planners fairly.** Both planners plan on the same map with the same inflation, from the same start, and get the same line-of-sight pruning. Path deviation is measured from the A* reference route below, and A*'s first route in flight is nearly that route, so path deviation favours A* by construction; energy, flight time and flown/reference length compare the planners on equal terms and lead the planner comparison. A two-tree RRT* (B-RRT* / RRT*-Connect) was also tried for large maps; with the same sample budget it was slower here, so the drones do not use it (`planner_bench --two-trees` measures it).

**Planner scaling** (`ros2 run swarm_intel planner_bench`, or `reproduce --only scaling`). One corner-to-corner plan on forests with the study's density and spacing, on 20, 40 and 80 m fields with 0.2, 0.1 and 0.05 m grid cells, 10 forests each; both planners get the same map, start, goal and pruning. The comparison rules were fixed before it was run (`planner_bench.py`), and all of it is reported in `results/planner_bench_<time>/report.txt`: for every field and cell size the planning time, route length and peak memory of RRT* relative to A* (geometric mean over the forests, with a sign-flip test), and how long each planner takes to report that a field cut in two has no route. Times are wall clock and depend on the CPU. A*'s work grows with the number of grid cells, RRT*'s with its sample budget, which is why the comparison spans coarse and fine grids. In the final run (i7-10750H, `results/planner_bench_20261120_084418/`) A* found the shorter route in all 9 settings (RRT* 3.5–13 % longer) and planned faster on 0.2 m cells of the 40 and 80 m fields (2.7× and 13.6×); RRT* planned about 4× faster on 0.05 m cells (time ratio 0.23–0.28) and needed 3–20× less memory in 8 of 9 settings. In flight, with every drone re-planning every second, the two planners use the same fleet energy (planner comparison).

**Reference route.** Before take-off, on its pad, every drone also plans an A* route on its own map (the known trees; the hidden pines are still unknown). It is never flown: path deviation is scored against it, so RRT* and A* missions are scored against the same reference, and it is computed on the drone like every other route (no pre-computed paths).

### Physics (identical in Gazebo and the headless study)

Gazebo's VelocityControl ignores forces, so the world node adds these to every drone's command (`mission.WorldSim`), the same way in both simulators:

- **Wind:** 0.2 m/s plus gusts. A drone holds its track by flying against its EKF's wind estimate. The guidance changes the ground-velocity setpoint by at most 2.2 m/s²; the wind compensation follows the estimate with the airframe's full tilt authority (6 m/s²), as a real attitude loop does.
- **Rotor downwash** (`downwash.py`): every flying drone pushes a column of air down. The wake is modelled as a round turbulent jet whose momentum flux equals the drone's thrust (Gaussian profile, 1/e radius 0.52 m, the vehicle radius, at the rotors, growing 0.15 m per metre; the downdraft is ramped up from zero over the first 0.25 m below the rotors, where the four slipstreams merge). Below a hovering drone the downdraft is **4.5 m/s at 0.6 m, 3.7 m/s at 1.5 m and 2.9 m/s at 3 m**: the wake stays strong for metres. It trails behind a moving drone and drifts with the wind. A drone in another drone's wake is pushed down (its altitude loop must climb against it, which costs power) and shaken sideways (turbulence, 25 % of the downdraft).
- **Heading error:** a small heading error rotates every velocity command.
- **Collisions:** the same shapes as the Gazebo collision model: a 0.5 m rotor disc (±0.05 m) and the four feet 0.127 m below it. A drone that hits a tree or another drone is pushed out and stopped; contacts are counted.
- **Energy:** a momentum-theory rotor model on the drone's **air** velocity (headwinds and climbing against downwash cost energy), about 157 W in hover, plus 6.5–12.5 W electronics (2.5 W camera processing, 1.0 W sensor-data processing, 2.5 W swarm radio).

### Separation standard (downwash-aware)

Every drone keeps, and every report scores, a minimum **horizontal** distance to every other drone that depends on their height difference Δz:

| Δz | 0 m | 0.6 m | 1.5 m | 3 m |
|---|---|---|---|---|
| required horizontal distance (centre to centre) | 1.29 m | 1.52 m | 1.62 m | 1.76 m |

It is the larger of rotor-disc contact (2 × 0.52 m, when |Δz| < 0.6 m) and the wake footprint (where the downdraft is still above 1 m/s, the largest the altitude loop can cancel, plus the lower drone's radius), plus a 0.25 m margin. Passing over another drone a few decimetres higher is therefore **not** separation. The **separation margin** metric is the smallest (distance − required distance) of any pair while both are en route: negative means closer than allowed.

### Safety layer (on the EKF estimate)

- **Braking barriers** against the map (only while the EKF uncertainty is small), against tracked drones at any height (closing speed limited to keep the separation standard), and against the last 1.5 s of depth-camera and ToF-ring hits (works even when the position estimate has drifted). The ring matters when a drone flies round a tree it discovered: the forward camera only ever saw the near side. The barriers of all nearby obstacles are enforced together, so backing off from one tree never drives the drone into the next.
- **Emergency brake** when another drone comes within 0.4 m of the rotor / wake footprint.
- **Speed limit** in directions the depth camera cannot see.
- **Vertical clearance:** the depth camera and the ToF ring report every obstacle from 0.25 m below the feet to 0.25 m above the rotors, so a drone never skims over a pine top (the 3.5 m layer would otherwise clip the 3.4 m pines with its feet).
- **Stall watchdog** and **rejoin-or-replan** when the drone is pushed off its path (not while it is giving way).
- **Landing:** a drone pushed more than 0.25 m off its pad on the way down (another drone passing close) stops descending and re-centres first (above 0.4 m, for at most 10 s).

---

## 1. One-time setup
```bash
pip3 install "numpy==1.26.4"             # the numpy of the published results
sudo apt install -y python3-matplotlib xterm
cd /media/sf_swarm_ws && colcon build
grep -q sf_swarm_ws/install ~/.bashrc || echo "source /media/sf_swarm_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
**Exact reproduction.** Every mission is deterministic for a given seed: on the same computer and setup it repeats exactly, whatever ran before it (the self-test replays a mission to check it). The reference machine is an Intel Core i7-10750H in a VirtualBox VM with Python 3.10.12 and numpy 1.26.4, where OpenBLAS runs its generic Prescott kernel. That kernel runs on every x86-64 CPU, so the package selects it itself before numpy loads (`swarm_intel/__init__.py`; every study prints which kernel it uses, and every results folder records its computer and software in `meta.json`). numpy's own vectorised functions and the C maths library still depend on the CPU and the system, and last-digit rounding can grow into a different trajectory, most of all with the swarm radio: on another computer compare the statistics, not single missions. To choose a kernel yourself set `OPENBLAS_CORETYPE`; `SWARM_INTEL_NO_BLAS_PIN=1` turns the automatic choice off. The planning-time metrics (`plan_time_ms_*`) and `planner_bench` are wall-clock measurements: they depend on the computer and its load and are the only numbers that never repeat exactly.

After any code change: `cd /media/sf_swarm_ws && colcon build && source install/setup.bash`.

## 2. Easiest: the menu
```bash
ros2 run swarm_intel menu
```
| Option | What it does |
|---|---|
| **1–12** | Watch one configuration in Gazebo with the GUI, listed with its code (C3+V, …) and `nav:=` key. Run option 1 (the baseline) first. Every drone's green reference route and the trail of the route it has flown (in its own colour) are drawn live; the live dashboard opens in an xterm, and the full report prints when all drones have landed. |
| **13** | All studies: every headless study of the paper (main study, mixed fleets, 80° camera, wind, robustness, speed, planner comparison with A*, give-way comparison with fixed priorities, robustness controls, planner scaling), then the figures, tables and numbers (~16 h on 2 cores, ~6.5 h on 8; `ros2 run swarm_intel reproduce`). |
| **14** | All in Gazebo: the 12 configurations × 3 seeds in Gazebo without the GUI, then the validation (Gazebo vs headless) and the paper figures (`ros2 run swarm_intel gazebo_study --seeds 3 --validate`). Finished runs are skipped, so it can be stopped and restarted. Only these study runs enter the Gazebo study and the validation; runs you watch with 1–12 stay separate. |
| **p** | Planner in Gazebo (options 1–12 and 14): RRT* or A*. |
| **g** | Give-way in Gazebo (options 1–12 and 14): turn trading or fixed priorities. |

Every option runs its tool in a separate process, so one run's settings can never carry over into the next. Each study and tool can also be run on its own (sections 4 and 5), for example `ros2 run swarm_intel study --layouts 10 --seeds 10` (main study), `planner_bench` (planner scaling), `compare --a all --b cam3_sensor` (two Gazebo runs side by side), `paper_figures`, or `selftest` (checks the planners, the turn trading and the mission logic, ~3 min).

## 3. Each simulation separately (GUI)
```bash
ros2 launch swarm_intel swarm_mission.launch.py nav:=cam3_sensor   # run the BASELINE first
ros2 run swarm_intel dashboard                                  # 2nd terminal: live states
```
Gazebo starts **paused**. Press ▶ Play: the drones compute their green reference routes on the pads (A*, about 5 s before take-off), then take off; each drone's trail, in its own colour, grows behind it from its take-off pad as it flies (its computer re-plans its route every second) and ends on the ground at its landing pad.

The **dashboard** shows each drone's position, height, speed, climb rate, power, energy, battery, position error, EKF uncertainty, markers in view, path deviation, distance to goal, re-plans, the **downdraft it feels** (DOWN) and its state (▼G = giving way, ~A = avoiding).

When every drone has landed, the terminal prints **every fleet metric** with this run's value, the baseline value (your latest `cam3_sensor` run flown with the same planner and give-way), the change and a verdict (▲ better, ▼ worse, = no meaningful difference), and a per-drone table (energy, distance, path, landing, position error, give-way time, time in downwash, contacts).

Each run is saved to `results/<nav>_<date>_<time>/` as `summary.json` and `telemetry.csv`.

| Launch argument | Default | Meaning |
|---|---|---|
| `nav` | `cam3_sensor` | navigation configuration (table above) |
| `gui` | `true` | `false` = no Gazebo window (fastest) |
| `paused` | `auto` | start paused; auto = paused only with the GUI |
| `lines` | `auto` | draw every drone's green reference route and the trail of its flown route (in its own colour); auto = only with the GUI |
| `dashboard` | `none` | `xterm` opens the live dashboard in its own window |
| `seed` | `0` | random seed (wind direction and gusts, sensor noise, the RRT* samples) |
| `planner` | `rrtstar` | path planner of every drone: `rrtstar` (RRT*) or `astar` (A*, for comparison) |
| `give_way` | `ttc` | swarm give-way: `ttc` (turn trading, no priorities) or `priority` (fixed priorities, for comparison) |
| `speed` | `1.0` | cruise-speed multiplier |
| `exit_when_done` | `false` | shut everything down after the report |
| `results_dir` | auto | where runs are saved |
| `shadows` | `false` | nicer shadows (slower) |
| `software_gl` | `true` | software rendering (needed in VirtualBox) |
| `kill_stale` | `true` | first stop any simulation processes left over from an earlier run |
| `study_run` | `false` | set by the Gazebo study: only such runs enter the Gazebo study and the validation (a run you launch to watch stays separate) |

The launch starts one world node (`swarm_world`) and one node per drone (`drone1` … `drone5`, the drones' on-board computers). The routes are drawn through the Gazebo GUI's marker service with the `ign service` tool. If they stay missing, check `ign service -l | grep marker` while Gazebo runs.

## 4. All results in one go (no GUI)
| Command | What it does | Time (2 CPU cores) |
|---|---|---|
| `ros2 run swarm_intel study` | All 12 configs × 10 seeds on the Gazebo forest, with the same mission code and a kinematic stand-in for Gazebo. | ~11 min |
| `ros2 run swarm_intel study --layouts 10` | The same on the Gazebo forest **plus 9 random forests** (100 missions per config). | ~2 h |
| `ros2 run swarm_intel gazebo_study --seeds 3` | All 12 configs in **real Gazebo without GUI**, seeds 0–2. Prints each run's report, then the combined table. Finished runs are skipped, so you can stop and restart it. | ~1–4 min per run |
| `ros2 run swarm_intel study --planner astar --layouts 10 --seeds 4` | The planner comparison: the same missions with A* instead of RRT*. | ~35 min (8 configs) |
| `ros2 run swarm_intel study --give-way priority --layouts 10 --seeds 4` | The give-way comparison: the same missions with fixed priorities (in drone order) instead of turn trading. | ~35 min (7 configs) |
| `ros2 run swarm_intel study --gazebo` | No new simulation: compiles your Gazebo runs (all seeds of this package version; `--planner astar` / `--give-way priority` for those runs). | seconds |
| `ros2 run swarm_intel validate` | Flies every Gazebo run again without Gazebo (same configuration, seed, forest, speed) and reports how well the two simulators agree. | ~10 s per run |

Each study prints four tables:

1. key metrics per configuration, with the change vs the baseline (fleet energy, flight time, mean path and position error, separation margin, mission time, goals, contacts, time in downwash)
2. **all metrics** side by side
3. a summary: how many metrics are better or worse, plus the Pareto-optimal configurations
4. the statistics behind the verdicts: mean paired difference ± 95 % CI, corrected p-value and effect size

**Verdict rule.** Every configuration flies exactly the same missions as the baseline (same forest and seed: same wind gusts, same downwash turbulence and, sensor by sensor, the same noise), so it is compared **pair by pair**. Different missions are independent: the noise depends on the seed *and* the forest. A configuration is *better* or *worse* on a metric only if both hold:

- its mean paired difference is significant in a sign-flip permutation test, Holm-corrected across the configurations for that metric (p < 0.05)
- the difference is more than 2 % of the baseline (0.05 m for the separation margin, which crosses zero)

Anything else is shown as = (no meaningful difference). With too few paired runs for a test (a few Gazebo seeds), the rule falls back to: more than that **and** more than 2 standard errors.

The study writes `results/study_<…>/` with `report.txt`, `runs.csv` (every mission), `summary.csv` (means, SDs, paired differences, CIs, p-values, verdicts) and `meta.json` (model and package version, planner, give-way, OpenBLAS kernel, numpy, Python and CPU; every results folder has one), plus figures:

| Figure | What it shows |
|---|---|
| **`study_comparison.png`** | **The detailed final comparison:** a heat-map of every metric of every configuration vs the baseline (blue = better, red = worse, grey = no meaningful difference), plus dot-plots of the key metrics |
| `study_overview.png` | energy vs accuracy, and how many metrics are better or worse |
| `study_changes.png` | change vs the baseline for the key metrics |
| `study_paths.png` | every configuration's flown paths (each drone in its colour) vs the drones' reference routes (green), on the Gazebo forest |

Folder names say what was run: `study_headless_<time>` (one forest), `study_headless_L10_<time>` (10 forests), `study_headless_cam80_L10_<time>`, `study_headless_wind1_L5_<time>`, `study_headless_mixed_L10_<time>`, `study_headless_dw0.5_L10_<time>`, `study_headless_gap1_L10_<time>`, `study_headless_astar_L10_<time>` (A*), `study_headless_priority_L10_<time>` (fixed-priority give-way), `study_gazebo_<time>`.

**Study options** (combine freely): `--layouts 10` (Gazebo forest + 9 random forests), `--seeds 20`, `--configs cam3_sensor,cam2,all`, `--mixed` (mixed fleets), `--cam-fov 80`, `--wind 1.0` (`--no-wind`: calm air), `--speed 1.3`, `--planner astar` (A* instead of RRT*), `--give-way priority` (fixed priorities instead of turn trading), two sensitivity checks of the model: `--downwash-scale 0.5` (downdraft half as strong) and `--layer-gap 1.0` (fixed altitude layers 1.0 m apart instead of 0.6 m; the lowest stays at 2.3 m, 0.8 m above drone1), and the control experiments of the robustness checks: `--radio-loss 0.3` (each message misses each receiver with this probability), `--radio-delay-ms 100-250` (extra delay drawn per message, in 50 ms steps), `--radio-range 8` (receivers farther away hear nothing), `--drone-view-deg 90` (drones see each other up to this far above and below; default 45), `--reactive-buffer 0.5` (drones without the radio keep this much more than the safe distance), `--ttc-fixed` (turn trading on its fixed 0.5 s / 0.1 s schedule without the reactive fallback, as in package 1.8.4). The radio options act in the headless simulator only (Gazebo uses ROS 2 topics); the extra configuration `cam3_sensor_layers_swarm` (layers + radio) is flown only when named in `--configs`.

**Other tools:** `ros2 run swarm_intel compare --a all --b cam3_sensor` (two Gazebo runs in detail), `ros2 run swarm_intel batch --configs cam3_sensor,all` (cruise-speed sweep), `ros2 run swarm_intel paper_figures` (the paper's figures and tables, next section). `ros2 run swarm_intel selftest` checks the installation: the planners (collision-free routes, against an exact reference), the turn trading, and that every drone plans its own routes with the reference route planned on the pad at no flight energy (`--quick` skips the mission checks).

## 5. For the paper

**Everything headless in one command** (~16 h on 2 CPU cores, ~6.5 h on 8; with rolling re-planning every drone computes about 20 routes per mission; each step writes its own folder):

```bash
ros2 run swarm_intel reproduce            # main, mixed, cam80, wind, robust, speed, planner, giveway, controls, scaling, then figures
ros2 run swarm_intel reproduce --list     # the exact commands of each step
ros2 run swarm_intel reproduce --only robust,figures
```

| Step | Command it runs | Missions |
|---|---|---|
| `main` | `study --layouts 10 --seeds 10` (12 configurations) | 1 200 |
| `mixed` | `study --mixed --layouts 10 --seeds 6` | 720 |
| `cam80` | `study --cam-fov 80 --layouts 10 --seeds 8` (the 9 camera configurations) | 720 |
| `wind` | `study --wind 0 / 0.5 / 1.0 / 1.5 --layouts 5 --seeds 4` (8 configurations) | 4 × 160 |
| `robust` | `study --downwash-scale 0.5`, `--layer-gap 1.0`, `--layer-gap 1.5` (`--layouts 10 --seeds 6`; baseline, fixed layers, give-way) | 3 × 180 |
| `speed` | `batch --seeds 4 --speeds 0.7,1.0,1.3,1.6 --configs cam3_sensor,cam2_sensor,cam3_sensor_swarm,all,cam3_sensor_layers` (C3+V, C2+V, C3+V+W, C2+V+W, C3+V+L) | 80 |
| `planner` | `study --planner astar --layouts 10 --seeds 4` (the 8 configurations of the wind study): the main study's missions with A* | 320 |
| `giveway` | `study --give-way priority --layouts 10 --seeds 4` (the baseline and the 6 radio configurations): the main study's missions with fixed priorities | 280 |
| `controls` | robustness checks (`--layouts 10 --seeds 4` each): a reference study with the layers + radio (`cam3_sensor_layers_swarm`); the radio with `--radio-loss 0.1`, `0.3`, `--radio-delay-ms 100-250`, and `--radio-loss 0.2 --radio-delay-ms 50-200 --radio-range 8`, each with the turn trading timed to the link and with `--ttc-fixed`; the layers with `--drone-view-deg 90`, with `--reactive-buffer 0.5`, and with both; then `controls` pairs each with its reference (`results/controls_<time>/report.txt`, `paper/table_controls.tex`, `paper/table_giveway.tex`) | 160 + 11 × 80 |
| `scaling` | `planner_bench`: RRT* vs A* on 20 / 40 / 80 m fields × 0.2 / 0.1 / 0.05 m cells (planning only; times depend on the CPU) | 180 plans per planner |
| `figures` | `paper_figures` → `results/paper/` | – |

The 80° study leaves out the three configurations without a camera: the camera does not affect them.

**The Gazebo check (on your VM):**

```bash
ros2 run swarm_intel gazebo_study --seeds 3 --validate    # 36 Gazebo runs (1-2 h), then the comparison
ros2 run swarm_intel reproduce --only figures              # adds fig_validation.pdf and the numbers
```

`validate` reports, per metric, the mean of both simulators, their difference and correlation, the same without configurations that lose their position in both (swarm only), and — what the conclusions rest on — **each configuration's change vs the baseline in both simulators**.

`results/paper/` then holds PDF figures for the IEEE two-column format (embedded fonts, no Type 3) with PNG previews, the LaTeX tables `table_configs.tex` and `table_main.tex`, `numbers.md` (every comparison with its 95 % CI, p-value and effect size). The manuscript (`paper/` in the workspace, not part of the repository) takes its figures and tables from here. Only studies of the current model version are used (their `meta.json`); earlier ones are left out and listed, `paper/meta.json` records which folders were used, and the first line of `numbers.md` names the model version and the computer.

## 6. Metrics (the whole fleet)

Every metric covers all five drones (`metrics.py`): **summed** (energy, flight time, distance, re-plans, contacts, time in downwash, …), **averaged over the drones** (path deviation, tracking and position error, landing error, …) or **the worst drone / closest pair** (largest deviation, separation margin, strongest downdraft, …), as each label says. The per-drone values are in every report and `summary.json`.

| Group | Metrics |
|---|---|
| **Accuracy** | drones at their goal; path deviation from each drone's reference route (A* on board before take-off; mean RMS, worst max); **tracking error** from each drone's own current plan; % of cruise within ±0.5 m of the reference route; landing error |
| **Localisation** | position error (mean RMS, worst max, at touchdown), position error of the drones **without** a camera (mixed fleets), EKF uncertainty, EKF consistency (NEES/2: 1 = honest, above 1 = over-confident), % of camera frames with an N-point fix, markers visible, longest gap without a fix, peer updates, outliers rejected |
| **Energy** | flight energy of all drones, energy per metre, average and peak power, navigation-electronics power, battery left (lowest drone) |
| **Time & path** | flight time of all drones, **mission time** (until the last drone landed), distance flown, flown/reference length, max speed, routes planned and planning time (mean, max) |
| **Safety & autonomy** | re-plans, stuck events, fail-safe escapes, reactive avoidance time, safety-filter interventions, obstacle contacts, drone contacts, closest pair (same altitude, 3-D), **separation margin**, **time in another drone's downwash** (downdraft > 1 m/s), **strongest downdraft felt**, hidden pines in each drone's map |
| **Coordination** | **time giving way** and how (holding, slowing, left, right, up, down), turn-trading cycles executed |

## 7. Results

The final run of package 1.8.4 (model `2026.11-fleet.3`) is in `results/` (**`results/README.md`** lists every folder); the findings are summarised in the top-level README, and every number of the paper, with its 95 % CI, Holm-corrected p-value and effect size, is in `results/paper/numbers.md`.

## 8. Notes
- **Configurations without an absolute reference drift by design:** `sensor` drifts with the sensor-data bias; `swarm` has no absolute reference at all, so its drones end metres away, brush trees and meet each other (counted as contacts).
- **Path deviation vs tracking error:** a drone that discovers a pine late may go round it on the other side from its reference route. That is a real detour, but not a tracking fault, so both are reported. With RRT*, a drone also often takes a route of about the same length on the other side of a tree (RRT* finds near-shortest routes, not always the same one), which shows up as path deviation too; the flown/reference length is the measure of route quality that does not depend on the side.
- **Everything runs on simulation time.** A slow VirtualBox real-time factor changes only how long you wait, not the numbers. `gui:=false` is much faster.
- **The headless study and Gazebo use the same mission code** (including wind and downwash): the same per-drone code and world model, in one process instead of one node per drone. Only the plant differs (Gazebo's physics vs a kinematic stand-in with the same collision model), and in Gazebo a drone acts on the radio messages of the previous tick.
- **Reports flag mismatches** that make a comparison unfair: runs of an older package version (this one is `2026.11-fleet.3`) or of another planner or give-way, a baseline flown at a different speed, or a baseline or fleet that did not finish. Run those again.
- **Troubleshooting:**
  - "Package 'swarm_intel' not found": source `install/setup.bash` in that terminal.
  - Dashboard says paused: press ▶ in Gazebo.
  - "Odometry has no altitude": your Fortress is old; the node falls back automatically.
  - **"THIS RUN IS INVALID … a second Gazebo simulation"**: a Gazebo server from an earlier run was still running. Such runs are left out of every study. Every launch stops leftover simulation processes first (`kill_stale:=true`) and runs in its own Gazebo transport partition. By hand: `pkill -f "ign gazebo"; pkill -f parameter_bridge; pkill -f swarm_intel`.
  - No green routes or trails: see section 3.

## 9. Code map
| File | Role |
|---|---|
| `config.py` | scenario, sensors, wind, downwash, the 12 configurations and the mixed fleets (edit here) |
| `downwash.py` | rotor wake model and the downwash-aware separation standard |
| `layouts.py` | the Gazebo forest and the random forests |
| `estimator.py` | the 8-state EKF |
| `agent.py` | one drone: sensing → EKF → planning → give-way / avoidance → control → safety layer |
| `mission.py` | the mission shared by Gazebo and the headless study: one drone's on-board runtime (`DroneRuntime`), the world (`WorldSim`: physics, wind, downwash, scoring), the radio and the report |
| `sensors.py`, `energy.py` | sensor models; rotor and electronics power model |
| `rrt_star.py` | the drones' path planner (RRT* with informed sampling) |
| `ttc.py` | the swarm give-way: conflict clusters, passage turns traded along trading cycles into the fleet's lowest-energy order |
| `grid_planner.py`, `geometry.py` | occupancy grid, A* (comparison and take-off reference route), line-of-sight pruning, obstacle geometry |
| `drone_node.py` | ROS 2 node: one drone's on-board computer (EKF, map, A* / RRT*, turn trading, control); one per drone |
| `world_node.py` | ROS 2 node: the world in Gazebo (physics, wind, downwash, scoring, report, route drawing) |
| `world_gen.py` | generates the Gazebo world (trees, markers, pads) |
| `gz_lines.py` | draws every drone's lines in Gazebo in 3-D (green reference flight; the trail of its flown route, in its own colour) |
| `onboard_planner.py` | a drone's route planner: inline (headless twin) or a separate process next to the flight loop (ROS drone node) |
| `runinfo.py` | `meta.json` of every results folder: model and package version, settings, OpenBLAS kernel, numpy, CPU |
| `planner_bench.py` | planner scaling benchmark: RRT* vs A* on larger fields and finer grids |
| `selftest.py`, `test/` | self-test of the planners, the turn trading and the mission logic (also runs under pytest) |
| `plant.py` | the kinematic stand-in for Gazebo (headless study) |
| `dashboard.py`, `report.py`, `metrics.py` | live dashboard, end-of-run report, metric definitions and verdicts |
| `study.py`, `study_plots.py`, `stats.py` | the all-configuration comparison, its figures and its paired statistics |
| `gazebo_study.py`, `validate.py` | the study in real Gazebo; Gazebo vs headless on the same missions |
| `paper_figures.py` | figures, tables and numbers for the paper |
| `reproduce.py` | runs every headless study of the paper, then `paper_figures` |
| `controls.py` | robustness checks from finished studies: lossy radio, layer controls, reactive buffer, give-way per configuration, forest-level tests, planner equivalence, payload mass |
| `compare.py`, `batch.py` | two-run comparison; speed sweep |
| `menu.py` | the menu |

## License

Copyright (C) 2026 Rajat Verma. This package is free software under the GNU General Public License, version 3 or (at your option) any later version; see `LICENSE`.
