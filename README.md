# Sense, Share, or Separate

A ROS 2 / Gazebo benchmark for small drone fleets in cluttered, GPS-denied airspace. It measures what sensing and coordination choices cost in energy, and what they buy in accuracy and safety, when the drones fly within reach of each other's rotor downwash.

Five quadrotors cross a partially mapped forest on intersecting routes. Every drone is its own edge node: it fuses all of its measurements in its own extended Kalman filter, plans every route on board with RRT* (A* is kept for comparison) with nothing pre-computed, and its guidance, mapping and safety use only that estimate. Twelve configurations combine

- **sensing:** an ArUco-marker camera that needs three or two markers in one frame for a fix, visual-inertial odometry (VIO), both, or neither;
- **coordination:** reactive avoidance at a common height, fixed altitude layers, or a swarm radio with UWB ranging, a shared map and a give-way in which the drones trade passage turns, along trading cycles in the spirit of top trading cycles, into the order of lowest energy for the whole fleet, with no drone given priority.

All five drones are equal, and every result covers the whole fleet (energy and time summed over the drones, errors averaged, separation of the closest pair).

The downwash of every drone is modeled as a turbulent jet, and a downwash-aware separation standard is used both by the drones and by the safety metrics. The same mission code runs in Gazebo Fortress (one ROS 2 node per drone plus a world node) and in a headless kinematic twin, which flies the large paired studies.

## Findings

From the final run of package 1.8.4 (model 2026.11-fleet.3, [`results/`](results/README.md); the radio controls with 1.8.5): paired by forest and seed, Holm-corrected sign-flip tests, energy summed over the five drones (baseline: 3-point camera + VIO, 23.7 kJ); main study 1 200 missions. Every number with its CI and p-value is in `results/paper/numbers.md`.

- **Share:** with the swarm radio the closest pair stays outside the downwash-aware separation standard on average (margin +0.29 to +0.31 m, against −0.14 m without coordination; p < 0.001). Most of that comes from the 0.5 m buffer with which the turn trading plans: given the same buffer, drones without the radio keep +0.27 m (39 of 40 missions outside, +1.7 % energy; robustness checks, `results/controls_20261122_122630/`). It costs fleet energy: +4.1 % (C2+V+W), +5.2 % (C3+V+W) and +5.7 % (C2+W) where the drones localise well, more with weaker localisation (+8.0 % C3+W, +10.7 % V+W); all p < 0.001.
- **Separate:** fixed altitude layers cost 13.3 % more energy and are the least safe option: margin −0.86 m and 0.44 s per mission in another drone's downwash. Most of the safety loss comes from the drones' blind spot above and below (they see each other up to 45°): seeing up to 90°, the layers keep the same margin as flying at one height (−0.15 m vs −0.14 m) and no longer meet downwash, but still cost 13 %; with the radio as well (C3+V+L+W) they are as safe as the radio alone but cost 21 %. Wider layer gaps make it worse (+16.3 % at 1 m, +21.6 % at 1.5 m) and halving the downwash strength still costs +10.9 %, while the radio's cost on the same missions stays at +3.0–3.4 %.
- **Sense:** VIO keeps camera drones on target (3-point camera alone: 4.0 of 5 drones at their goal; with VIO: 5 of 5). VIO alone drifts (0.42 m RMS, 2.25 of 5 at their goal), and the radio alone cannot localise the fleet (0 of 5, +147 % energy). With a single forward 80° camera a 3-point fix is rare: the baseline reaches 3.4 of 5, the 2-point camera + VIO 4.6 of 5 at the same energy. In mixed fleets (camera + VIO drones among VIO-only drones), with the radio two camera drones bring 4.8 of 5 drones to their goal; without it, four camera drones bring 4.4.
- **Give-way:** turn trading uses 10.2–13.9 % less fleet energy than fixed priorities in every radio configuration that localises (p < 0.001) and cuts the time spent giving way by 40–45 %. The closest-pair margin shrinks by 0.10–0.16 m but stays positive (+0.26 to +0.33 m).
- **Radio link:** on its original fixed schedule (0.1 s per step of a round), the turn trading shrugs off 10 % message loss, costs +4.2 % energy with 30 % (margin +0.17 m, 26 of 40 missions outside), and stalls when messages arrive 100–250 ms late: the drones never give way and the margin falls to −0.20 m, like no coordination. With the rounds timed to the measured link delay and a buffered reactive fallback (package 1.8.5, identical on an ideal link), the margin stays at +0.19 to +0.32 m on all four links tested (29–34 of 40 missions outside; 32 with the ideal radio) for 1–6 % more energy than the ideal radio. No contacts on any link. A message holds about 214 numbers (0.9 kB binary, 4.9 kB JSON): 17–99 kB/s per drone.
- **Planner in flight:** with every drone re-planning every second, RRT* and A* use the same fleet energy (differences −0.4 to +4.1 %, none significant in 8 configurations) and fly equally long routes; RRT* needs 87–109 ms per route against A*'s 6–9 ms and switches routes more often.
- **Planner scaling** (planning only): A* finds 3.5–13 % shorter routes everywhere and plans faster on 0.2 m cells of the 40 and 80 m fields; RRT* plans about 4× faster on 0.05 m cells and needs 3–20× less memory in 8 of 9 settings.
- **Wind and speed:** up to 1.5 m/s of wind the radio configurations keep their energy (about 24 kJ) and positive margins, while VIO alone loses more drones (1.9 of 5 at 1.5 m/s). Flying faster saves energy (baseline 27.5 kJ at 0.84 m/s, 19.2 kJ at 1.92 m/s): hovering dominates the power.
- **Gazebo check** (12 configurations × 3 seeds, the same missions as the headless twin): fleet energy agrees within 0.7 % and the separation margin within 0.01 m; the effects of the configurations have the same sign in both simulators in 9 of 10 cases for position error, 8 of 10 for tracking error and 5 of 8 for energy (most energy effects are within ±5 %). In Gazebo the radio configurations kept +0.32 to +0.51 m and the layers cost 28 % more energy; without coordination two drones touched once in the baseline's three runs.

## Repository layout

| Path | Contents |
|---|---|
| [`src/swarm_intel/`](src/swarm_intel/README.md) | The ROS 2 package: per-drone on-board nodes and a world node, mission code, per-drone EKF, downwash and energy models, Gazebo world and launch files, the headless twin, and the study, validation and plotting tools |
| [`results/`](results/README.md) | All studies: per-mission data (`runs.csv`), paired statistics (`summary.csv`), reports and plots, the Gazebo runs and the Gazebo-vs-headless validation |
| `results/paper/` | Figures, LaTeX tables and `numbers.md`, generated from the studies |

## Getting started

Requirements: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress with `ros_gz`, Python 3.10.

```bash
git clone https://github.com/<user>/sense-share-separate.git ~/swarm_ws
cd ~/swarm_ws
pip3 install "numpy==1.26.4"   # the version the published results used
sudo apt install -y python3-matplotlib xterm
colcon build && source install/setup.bash
ros2 run swarm_intel menu
```

The menu runs any configuration in Gazebo and starts the headless studies. The same can be done directly:

```bash
ros2 launch swarm_intel swarm_mission.launch.py nav:=cam3_sensor   # one Gazebo mission (baseline)
ros2 run swarm_intel study --layouts 10 --seeds 10                 # main headless study, about 4 h on 2 cores
ros2 run swarm_intel reproduce                                     # every headless study, figures and tables, about 6.5-16 h
```

The [package README](src/swarm_intel/README.md) documents the scenario, the models, all configurations and metrics, and every command and option.

## Reproducibility

Every mission is deterministic for a given seed: on the same computer and setup it repeats exactly. The package selects OpenBLAS's generic Prescott kernel on every x86-64 CPU (the kernel of the reference machine, an Intel Core i7-10750H VirtualBox VM with Python 3.10.12 and numpy 1.26.4), and every results folder records the computer and software it was computed with (`meta.json`). On another computer, numpy's vectorised functions and the system's maths library can still round differently, so individual missions can differ in detail, most of all those with the swarm radio: compare the statistics, not single missions. Planning times are wall-clock measurements and never repeat exactly.

## License

Copyright (C) 2026 Rajat Verma.

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version. See [`LICENSE`](LICENSE).

## Contact

Rajat Verma — r010702v@gmail.com
