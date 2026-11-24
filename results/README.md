# Results — model 2026.11-fleet.3

Everything here is computed with the current version of the package: every drone plans on board (RRT\*; A\* for the planner comparison), the swarm gives way by turn trading (fixed priorities for the give-way comparison), the five drones are equal, and every metric covers the whole fleet.

The flight studies are run with the headless twin: the same mission code as Gazebo (including wind and rotor downwash), with a kinematic stand-in for Gazebo's physics.

## What is in this folder

The final run of package 1.8.4 (model 2026.11-fleet.3), on an Intel Core i7-10750H (VirtualBox VM, Python 3.10.12, numpy 1.26.4, OpenBLAS kernel Prescott; every folder records this in `meta.json`). Every drone plans with RRT\* and re-plans every second, and the swarm gives way by turn trading, except in the two comparison studies. The control studies of the robustness checks were flown later with package 1.8.5 (the reference, view and buffer studies with 1.8.4, which flies identically with them) on a second computer (Intel Xeon @ 2.10 GHz, Python 3.13, numpy 2.5.3, OpenBLAS kernel Prescott), where single missions can round differently; they are therefore compared only with the reference study flown there, on the same 40 missions.

| Folder | What | Missions |
|---|---|---|
| `study_headless_L10_20261120_040523/` | **Main study:** 12 configurations × 10 forests × 10 seeds, 360° camera ring, wind 0.2 m/s | 1 200 |
| `study_headless_mixed_L10_20261120_050259/` | **Mixed fleets:** k of 5 drones with camera + VIO, the others VIO only, with and without the swarm radio; 10 forests × 6 seeds | 720 |
| `study_headless_cam80_L10_20261120_061214/` | **Single forward 80° camera:** the 9 camera configurations; 10 forests × 8 seeds | 720 |
| `study_headless_wind0_L5_…`, `wind0.5_L5_…`, `wind1_L5_…`, `wind1.5_L5_…` | **Wind sweep** 0 / 0.5 / 1.0 / 1.5 m/s: 8 configurations, 5 forests × 4 seeds each | 4 × 160 |
| `study_headless_dw0.5_L10_…`, `gap1_L10_…`, `gap1.5_L10_…` | **Robustness:** downwash half as strong; altitude layers 1.0 m and 1.5 m apart (baseline, layers, swarm; 10 forests × 6 seeds each) | 3 × 180 |
| `speed_sweep/` | **Speed sweep:** 5 configurations at 0.84 / 1.2 / 1.56 / 1.92 m/s, 4 seeds | 80 |
| `study_headless_astar_L10_20261120_081411/` | **Planner comparison:** A\* instead of RRT\* (8 configurations, 10 forests × 4 seeds, the main study's missions) | 320 |
| `study_headless_priority_L10_20261120_084413/` | **Give-way comparison:** fixed priorities instead of turn trading (baseline + 6 radio configurations, 10 forests × 4 seeds, the main study's missions) | 280 |
| `give_way_ablation_20261124/` | **Give-way ablation:** fixed priorities with turn trading's energy-priced manoeuvres (the patch is in the folder), and fixed priorities and turn trading flown again; baseline + 5 radio configurations, 10 forests × 4 seeds, the give-way comparison's missions, on the second computer (above). See its `README.md` | 3 × 240 |
| `planner_bench_20261120_084418/` | **Planner scaling:** RRT\* vs A\* on 20 / 40 / 80 m fields with 0.2 / 0.1 / 0.05 m cells, 10 forests each, plus a field cut in two; planning only | 180 + 18 plans per planner |
| `<configuration>_20261120_*/` (36 folders), `gazebo_logs/` | **Gazebo runs:** the 12 configurations × seeds 0–2 in Gazebo Fortress without the GUI (Gazebo-study runs), with each launch's log | 36 |
| `study_gazebo_20261120_113722/` | **Gazebo study:** the 36 runs, every configuration vs the baseline (3 seeds: ▲ / ▼ = more than 2 % and 2 standard errors) | 36 |
| `validation_20261120_114019/` | **Gazebo vs headless:** the same 36 missions flown by the headless twin, run by run and as effects vs the baseline | 36 + 36 |
| `study_headless_L10_20261120_134533/` | **Controls, reference:** C3+V, C3+V+W, C3+V+L and C3+V+L+W (layers + radio), 10 forests × 4 seeds, flown on the second computer (below) | 160 |
| `study_headless_loss0.1_L10_…`, `loss0.3_L10_…`, `delay2-5_L10_…`, `loss0.2_delay1-4_range8_L10_…` | **Controls, radio realism:** C3+V+W with 10 % / 30 % of the messages lost, 100–250 ms extra delay, and 20 % loss + 50–200 ms + 8 m range together, turn trading timed to the link (package 1.8.5; each with the baseline; 10 forests × 4 seeds) | 4 × 80 |
| `study_headless_loss0.1_ttcfixed_L10_…` and the other three `…_ttcfixed_…` | **The same links, original protocol:** turn trading on its fixed schedule without the reactive fallback (`--ttc-fixed`; identical, mission for mission, to the first runs with package 1.8.4) | 4 × 80 |
| `study_headless_view90_L10_…`, `rbuf0.5_L10_…`, `view90_rbuf0.5_L10_…` | **Controls, separation:** C3+V and C3+V+L with drones seen up to 90° above/below, with a 0.5 m buffer on the reactive avoidance, and with both (10 forests × 4 seeds) | 3 × 80 |
| `controls_20261122_122630/` | **Robustness checks** (`ros2 run swarm_intel controls`): the controls paired with their references, the radio's message size and agreement under loss, give-way per configuration, forest-level tests of the main study, planner equivalence (TOST), payload mass, energy facts and the count of distinct missions | 1 + 50 flown |
| `paper/` | Figures, LaTeX tables and `numbers.md` (every comparison with its 95 % CI, Holm-corrected p-value and effect size); `paper/meta.json` lists the folders used | |

Each study folder has `meta.json` (model and package version, planner, give-way, OpenBLAS kernel, numpy, Python, CPU), `report.txt` (the four terminal tables, including the statistics), `runs.csv` (every mission), `summary.csv` (per metric: mean, SD, paired difference, 95 % CI, corrected p-value, verdict) and `study_comparison.png` plus three more figures.

**How to read the verdicts.** Every configuration flies the same missions as the baseline (same forest, same seed), so it is compared pair by pair. ▲ / ▼ in the reports means the mean paired difference is significant in a sign-flip permutation test, Holm-corrected across the configurations for that metric (p < 0.05), **and** more than 2 % of the baseline (0.05 m for the separation margin, which crosses zero).

**Codes:** C2/C3 = 2-/3-point camera, V = VIO velocity (IMU + visual-inertial odometry; the configuration keys call it `sensor`: `cam3_sensor` = C3+V), W = swarm radio (shared map, UWB ranging, give-way by turn trading), L = fixed altitude layers without radio. The baseline is C3+V. **Sep. margin:** smallest (horizontal distance − downwash-aware separation standard) of any pair while en route; negative = closer than allowed. **Downwash:** seconds per mission any drone spends in another drone's downwash (> 1 m/s), summed over the five drones.
