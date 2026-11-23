# Numbers for the paper

Paired differences vs the baseline (3-point camera + VIO): mean [95 % CI], Holm-corrected sign-flip permutation p (Holm across configurations, per metric), effect size d_z. Verdict = significant AND > 2 %.

Model 2026.11-fleet.3, package 1.8.4; computed on Intel(R) Core(TM) i7-10750H CPU @ 2.60GHz (Python 3.10.12, numpy 1.26.4, OpenBLAS kernel Prescott). Sources: paper/meta.json.

## Main study (360° camera ring)

1200 missions; 10 forests x 10 seeds x 12 configurations; 296 paired tests (Holm-corrected per metric across the configurations).

| configuration | metric | baseline | config | change | 95 % CI of change | p (Holm) | d_z | verdict |
|---|---|---|---|---|---|---|---|---|
| cam3 | Energy | 23.660 | 24.957 | +5.5 % | ±2.5 % | <0.001 | +0.44 | worse |
| cam3 | Flight time | 139.6 | 147.7 | +5.8 % | ±2.5 % | <0.001 | +0.47 | worse |
| cam3 | Mission time | 40.1 | 43.6 | +8.7 % | ±4.1 % | <0.001 | +0.42 | worse |
| cam3 | Position error | 0.042 | 0.252 | +494.2 % | ±168.9 % | <0.001 | +0.58 | worse |
| cam3 | Tracking error | 0.147 | 0.251 | +70.4 % | ±30.3 % | <0.001 | +0.46 | worse |
| cam3 | Path deviation | 1.205 | 1.325 | +10.0 % | ±4.8 % | <0.001 | +0.41 | worse |
| cam3 | Separation margin (fleet) | -0.14 | -0.19 | -0.05 m | ±0.06 m | 0.302 | -0.18 | same |
| cam3 | Time in downwash (fleet) | 0.0 | 0.0 | +0.03 s | ±0.04 s | 1.000 | +0.18 | same |
| cam3 | Drone-drone contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| cam3 | Obstacle contacts (fleet) | 0 | 0.05 | +0.05 # | ±0.05 # | 0.879 | +0.19 | same |
| cam3 | Landing error | 0.068 | 0.327 | +378.5 % | ±134.0 % | <0.001 | +0.56 | worse |
| cam3 | Goals | 5 | 4.0 | -19.8 % | ±4.2 % | <0.001 | -0.93 | worse |
| cam2 | Energy | 23.660 | 24.085 | +1.8 % | ±1.9 % | 0.123 | +0.19 | same |
| cam2 | Flight time | 139.6 | 142.4 | +2.0 % | ±1.8 % | 0.060 | +0.22 | same |
| cam2 | Mission time | 40.1 | 41.3 | +3.0 % | ±2.7 % | 0.057 | +0.22 | same |
| cam2 | Position error | 0.042 | 0.074 | +73.8 % | ±13.5 % | <0.001 | +1.08 | worse |
| cam2 | Tracking error | 0.147 | 0.162 | +9.9 % | ±5.7 % | 0.003 | +0.35 | worse |
| cam2 | Path deviation | 1.205 | 1.233 | +2.3 % | ±4.4 % | 0.584 | +0.11 | same |
| cam2 | Separation margin (fleet) | -0.14 | -0.15 | -0.01 m | ±0.03 m | 1.000 | -0.06 | same |
| cam2 | Time in downwash (fleet) | 0.0 | 0.0 | +0.01 s | ±0.02 s | 1.000 | +0.10 | same |
| cam2 | Drone-drone contacts (fleet) | 0 | 0.02 | +0.02 # | ±0.03 # | 1.000 | +0.14 | same |
| cam2 | Obstacle contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| cam2 | Landing error | 0.068 | 0.112 | +64.0 % | ±16.9 % | <0.001 | +0.75 | worse |
| cam2 | Goals | 5 | 4.7 | -5.2 % | ±1.7 % | <0.001 | -0.59 | worse |
| cam2_sensor | Energy | 23.660 | 23.985 | +1.4 % | ±1.9 % | 0.159 | +0.14 | same |
| cam2_sensor | Flight time | 139.6 | 141.3 | +1.3 % | ±1.8 % | 0.182 | +0.13 | same |
| cam2_sensor | Mission time | 40.1 | 40.8 | +1.7 % | ±2.5 % | 0.171 | +0.14 | same |
| cam2_sensor | Position error | 0.042 | 0.038 | -10.5 % | ±4.2 % | <0.001 | -0.49 | better |
| cam2_sensor | Tracking error | 0.147 | 0.151 | +2.6 % | ±5.8 % | 0.755 | +0.09 | same |
| cam2_sensor | Path deviation | 1.205 | 1.198 | -0.6 % | ±3.6 % | 0.753 | -0.03 | same |
| cam2_sensor | Separation margin (fleet) | -0.14 | -0.14 | -0.00 m | ±0.01 m | 1.000 | -0.05 | same |
| cam2_sensor | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| cam2_sensor | Drone-drone contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| cam2_sensor | Obstacle contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| cam2_sensor | Landing error | 0.068 | 0.047 | -30.4 % | ±8.6 % | <0.001 | -0.70 | better |
| cam2_sensor | Goals | 5 | 5 | +0.0 % | ±0.0 % | 1.000 | +0.00 | same |
| sensor | Energy | 23.660 | 25.878 | +9.4 % | ±4.9 % | <0.001 | +0.38 | worse |
| sensor | Flight time | 139.6 | 155.1 | +11.1 % | ±5.1 % | <0.001 | +0.44 | worse |
| sensor | Mission time | 40.1 | 51.7 | +28.9 % | ±15.4 % | <0.001 | +0.37 | worse |
| sensor | Position error | 0.042 | 0.415 | +880.1 % | ±119.6 % | <0.001 | +1.46 | worse |
| sensor | Tracking error | 0.147 | 0.290 | +97.1 % | ±14.5 % | <0.001 | +1.33 | worse |
| sensor | Path deviation | 1.205 | 1.278 | +6.1 % | ±4.6 % | 0.031 | +0.26 | worse |
| sensor | Separation margin (fleet) | -0.14 | -0.16 | -0.02 m | ±0.03 m | 0.302 | -0.17 | same |
| sensor | Time in downwash (fleet) | 0.0 | 0.0 | +0.01 s | ±0.01 s | 1.000 | +0.10 | same |
| sensor | Drone-drone contacts (fleet) | 0 | 0.02 | +0.02 # | ±0.03 # | 1.000 | +0.14 | same |
| sensor | Obstacle contacts (fleet) | 0 | 0.11 | +0.11 # | ±0.09 # | 0.139 | +0.25 | same |
| sensor | Landing error | 0.068 | 0.746 | +993.2 % | ±118.6 % | <0.001 | +1.66 | worse |
| sensor | Goals | 5 | 2.2 | -55.0 % | ±4.0 % | <0.001 | -2.73 | worse |
| cam3_sensor_layers | Energy | 23.660 | 26.809 | +13.3 % | ±1.5 % | <0.001 | +1.74 | worse |
| cam3_sensor_layers | Flight time | 139.6 | 157.4 | +12.8 % | ±1.5 % | <0.001 | +1.72 | worse |
| cam3_sensor_layers | Mission time | 40.1 | 41.4 | +3.2 % | ±2.1 % | 0.008 | +0.30 | worse |
| cam3_sensor_layers | Position error | 0.042 | 0.050 | +19.0 % | ±4.4 % | <0.001 | +0.85 | worse |
| cam3_sensor_layers | Tracking error | 0.147 | 0.148 | +0.2 % | ±6.5 % | 0.940 | +0.01 | same |
| cam3_sensor_layers | Path deviation | 1.205 | 0.871 | -27.7 % | ±5.4 % | <0.001 | -1.02 | better |
| cam3_sensor_layers | Separation margin (fleet) | -0.14 | -0.86 | -0.72 m | ±0.12 m | <0.001 | -1.24 | worse |
| cam3_sensor_layers | Time in downwash (fleet) | 0.0 | 0.4 | +0.44 s | ±0.10 s | <0.001 | +0.85 | worse |
| cam3_sensor_layers | Drone-drone contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| cam3_sensor_layers | Obstacle contacts (fleet) | 0 | 0.01 | +0.01 # | ±0.02 # | 1.000 | +0.10 | same |
| cam3_sensor_layers | Landing error | 0.068 | 0.083 | +21.5 % | ±8.8 % | <0.001 | +0.49 | worse |
| cam3_sensor_layers | Goals | 5 | 5.0 | -0.2 % | ±0.4 % | 1.000 | -0.10 | same |
| cam3_sensor_swarm | Energy | 23.660 | 24.883 | +5.2 % | ±2.4 % | <0.001 | +0.42 | worse |
| cam3_sensor_swarm | Flight time | 139.6 | 145.1 | +3.9 % | ±2.4 % | 0.006 | +0.33 | worse |
| cam3_sensor_swarm | Mission time | 40.1 | 43.9 | +9.6 % | ±3.3 % | <0.001 | +0.58 | worse |
| cam3_sensor_swarm | Position error | 0.042 | 0.045 | +5.3 % | ±4.3 % | 0.014 | +0.25 | worse |
| cam3_sensor_swarm | Tracking error | 0.147 | 0.199 | +34.9 % | ±11.8 % | <0.001 | +0.58 | worse |
| cam3_sensor_swarm | Path deviation | 1.205 | 1.333 | +10.6 % | ±6.6 % | 0.011 | +0.32 | worse |
| cam3_sensor_swarm | Separation margin (fleet) | -0.14 | 0.30 | +0.44 m | ±0.05 m | <0.001 | +1.87 | better |
| cam3_sensor_swarm | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| cam3_sensor_swarm | Drone-drone contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| cam3_sensor_swarm | Obstacle contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| cam3_sensor_swarm | Landing error | 0.068 | 0.069 | +1.0 % | ±12.2 % | 0.934 | +0.02 | same |
| cam3_sensor_swarm | Goals | 5 | 5.0 | -0.2 % | ±0.4 % | 1.000 | -0.10 | same |
| cam3_swarm | Energy | 23.660 | 25.546 | +8.0 % | ±2.3 % | <0.001 | +0.68 | worse |
| cam3_swarm | Flight time | 139.6 | 149.4 | +7.1 % | ±2.3 % | <0.001 | +0.62 | worse |
| cam3_swarm | Mission time | 40.1 | 47.6 | +18.6 % | ±4.8 % | <0.001 | +0.77 | worse |
| cam3_swarm | Position error | 0.042 | 0.219 | +416.3 % | ±144.8 % | <0.001 | +0.57 | worse |
| cam3_swarm | Tracking error | 0.147 | 0.280 | +89.8 % | ±35.4 % | <0.001 | +0.50 | worse |
| cam3_swarm | Path deviation | 1.205 | 1.407 | +16.7 % | ±6.7 % | <0.001 | +0.50 | worse |
| cam3_swarm | Separation margin (fleet) | -0.14 | 0.29 | +0.43 m | ±0.05 m | <0.001 | +1.90 | better |
| cam3_swarm | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| cam3_swarm | Drone-drone contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| cam3_swarm | Obstacle contacts (fleet) | 0 | 0.18 | +0.18 # | ±0.17 # | 0.081 | +0.22 | same |
| cam3_swarm | Landing error | 0.068 | 0.300 | +339.7 % | ±128.0 % | <0.001 | +0.53 | worse |
| cam3_swarm | Goals | 5 | 4 | -20.0 % | ±3.9 % | <0.001 | -1.03 | worse |
| cam2_swarm | Energy | 23.660 | 25.001 | +5.7 % | ±2.3 % | <0.001 | +0.48 | worse |
| cam2_swarm | Flight time | 139.6 | 146.2 | +4.7 % | ±2.3 % | <0.001 | +0.41 | worse |
| cam2_swarm | Mission time | 40.1 | 44.5 | +11.1 % | ±3.4 % | <0.001 | +0.65 | worse |
| cam2_swarm | Position error | 0.042 | 0.076 | +78.8 % | ±17.9 % | <0.001 | +0.87 | worse |
| cam2_swarm | Tracking error | 0.147 | 0.198 | +34.5 % | ±10.9 % | <0.001 | +0.63 | worse |
| cam2_swarm | Path deviation | 1.205 | 1.335 | +10.7 % | ±5.6 % | 0.002 | +0.38 | worse |
| cam2_swarm | Separation margin (fleet) | -0.14 | 0.31 | +0.45 m | ±0.05 m | <0.001 | +1.75 | better |
| cam2_swarm | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| cam2_swarm | Drone-drone contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| cam2_swarm | Obstacle contacts (fleet) | 0 | 0.01 | +0.01 # | ±0.02 # | 1.000 | +0.10 | same |
| cam2_swarm | Landing error | 0.068 | 0.115 | +69.0 % | ±20.7 % | <0.001 | +0.66 | worse |
| cam2_swarm | Goals | 5 | 4.8 | -4.4 % | ±1.8 % | <0.001 | -0.48 | worse |
| all | Energy | 23.660 | 24.630 | +4.1 % | ±2.1 % | <0.001 | +0.39 | worse |
| all | Flight time | 139.6 | 143.6 | +2.9 % | ±2.1 % | 0.020 | +0.28 | worse |
| all | Mission time | 40.1 | 43.8 | +9.2 % | ±3.0 % | <0.001 | +0.61 | worse |
| all | Position error | 0.042 | 0.037 | -11.8 % | ±4.1 % | <0.001 | -0.57 | better |
| all | Tracking error | 0.147 | 0.181 | +23.0 % | ±10.8 % | <0.001 | +0.42 | worse |
| all | Path deviation | 1.205 | 1.308 | +8.5 % | ±5.6 % | 0.016 | +0.30 | worse |
| all | Separation margin (fleet) | -0.14 | 0.30 | +0.44 m | ±0.04 m | <0.001 | +2.06 | better |
| all | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| all | Drone-drone contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| all | Obstacle contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| all | Landing error | 0.068 | 0.049 | -28.4 % | ±11.5 % | <0.001 | -0.49 | better |
| all | Goals | 5 | 5.0 | -0.4 % | ±0.6 % | 1.000 | -0.14 | same |
| sensor_swarm | Energy | 23.660 | 26.198 | +10.7 % | ±5.8 % | <0.001 | +0.36 | worse |
| sensor_swarm | Flight time | 139.6 | 155.1 | +11.1 % | ±6.0 % | <0.001 | +0.37 | worse |
| sensor_swarm | Mission time | 40.1 | 53.3 | +33.0 % | ±16.1 % | <0.001 | +0.41 | worse |
| sensor_swarm | Position error | 0.042 | 0.398 | +840.6 % | ±117.3 % | <0.001 | +1.42 | worse |
| sensor_swarm | Tracking error | 0.147 | 0.326 | +121.4 % | ±18.8 % | <0.001 | +1.28 | worse |
| sensor_swarm | Path deviation | 1.205 | 1.386 | +15.0 % | ±5.5 % | <0.001 | +0.54 | worse |
| sensor_swarm | Separation margin (fleet) | -0.14 | 0.29 | +0.43 m | ±0.05 m | <0.001 | +1.87 | better |
| sensor_swarm | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| sensor_swarm | Drone-drone contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| sensor_swarm | Obstacle contacts (fleet) | 0 | 0.08 | +0.08 # | ±0.06 # | 0.139 | +0.26 | same |
| sensor_swarm | Landing error | 0.068 | 0.703 | +930.4 % | ±100.1 % | <0.001 | +1.84 | worse |
| sensor_swarm | Goals | 5 | 2.2 | -55.0 % | ±4.2 % | <0.001 | -2.62 | worse |
| swarm | Energy | 23.660 | 58.524 | +147.4 % | ±13.4 % | <0.001 | +2.18 | worse |
| swarm | Flight time | 139.6 | 344.0 | +146.5 % | ±13.3 % | <0.001 | +2.19 | worse |
| swarm | Mission time | 40.1 | 119.4 | +197.9 % | ±23.2 % | <0.001 | +1.70 | worse |
| swarm | Position error | 0.042 | 6.228 | +14611.9 % | ±704.2 % | <0.001 | +4.12 | worse |
| swarm | Tracking error | 0.147 | 4.553 | +2990.4 % | ±169.3 % | <0.001 | +3.50 | worse |
| swarm | Path deviation | 1.205 | 3.546 | +194.2 % | ±15.0 % | <0.001 | +2.57 | worse |
| swarm | Separation margin (fleet) | -0.14 | -0.47 | -0.34 m | ±0.12 m | <0.001 | -0.55 | worse |
| swarm | Time in downwash (fleet) | 0.0 | 0.3 | +0.28 s | ±0.13 s | <0.001 | +0.42 | worse |
| swarm | Drone-drone contacts (fleet) | 0 | 1.1 | +1.14 # | ±0.27 # | <0.001 | +0.85 | worse |
| swarm | Obstacle contacts (fleet) | 0 | 2.7 | +2.66 # | ±0.52 # | <0.001 | +1.02 | worse |
| swarm | Landing error | 0.068 | 7.887 | +11456.0 % | ±556.0 % | <0.001 | +4.09 | worse |
| swarm | Goals | 5 | 0 | -100.0 % | ±0.0 % | <0.001 | - | worse |

## Single forward 80° camera

720 missions; 10 forests x 8 seeds x 9 configurations; 215 paired tests (Holm-corrected per metric across the configurations).

| configuration | metric | baseline | config | change | 95 % CI of change | p (Holm) | d_z | verdict |
|---|---|---|---|---|---|---|---|---|
| cam3 | Energy | 24.560 | 38.235 | +55.7 % | ±7.1 % | <0.001 | +1.75 | worse |
| cam3 | Flight time | 144.8 | 225.9 | +56.0 % | ±7.1 % | <0.001 | +1.76 | worse |
| cam3 | Mission time | 43.5 | 74.3 | +70.8 % | ±11.1 % | <0.001 | +1.41 | worse |
| cam3 | Position error | 0.271 | 2.515 | +829.6 % | ±69.7 % | <0.001 | +2.65 | worse |
| cam3 | Tracking error | 0.236 | 1.693 | +616.0 % | ±55.7 % | <0.001 | +2.46 | worse |
| cam3 | Path deviation | 1.206 | 2.522 | +109.2 % | ±14.8 % | <0.001 | +1.64 | worse |
| cam3 | Separation margin (fleet) | -0.16 | -0.49 | -0.33 m | ±0.11 m | <0.001 | -0.64 | worse |
| cam3 | Time in downwash (fleet) | 0.0 | 0.2 | +0.17 s | ±0.09 s | <0.001 | +0.42 | worse |
| cam3 | Drone-drone contacts (fleet) | 0.03 | 1.2 | +4700.0 % | ±1183.1 % | <0.001 | +0.88 | worse |
| cam3 | Obstacle contacts (fleet) | 0.05 | 0.93 | +1750.0 % | ±568.2 % | <0.001 | +0.69 | worse |
| cam3 | Landing error | 0.467 | 3.076 | +558.0 % | ±67.2 % | <0.001 | +1.85 | worse |
| cam3 | Goals | 3.4 | 0.47 | -85.8 % | ±7.8 % | <0.001 | -2.45 | worse |
| cam2 | Energy | 24.560 | 27.536 | +12.1 % | ±5.0 % | <0.001 | +0.54 | worse |
| cam2 | Flight time | 144.8 | 163.1 | +12.6 % | ±4.9 % | <0.001 | +0.58 | worse |
| cam2 | Mission time | 43.5 | 50.2 | +15.4 % | ±8.0 % | 0.001 | +0.43 | worse |
| cam2 | Position error | 0.271 | 0.749 | +177.0 % | ±32.2 % | <0.001 | +1.22 | worse |
| cam2 | Tracking error | 0.236 | 0.530 | +124.1 % | ±28.9 % | <0.001 | +0.96 | worse |
| cam2 | Path deviation | 1.206 | 1.471 | +22.0 % | ±7.2 % | <0.001 | +0.68 | worse |
| cam2 | Separation margin (fleet) | -0.16 | -0.19 | -0.03 m | ±0.05 m | 0.624 | -0.12 | same |
| cam2 | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.01 s | 1.000 | +0.11 | same |
| cam2 | Drone-drone contacts (fleet) | 0.03 | 0.19 | +650.0 % | ±640.4 % | 0.252 | +0.23 | same |
| cam2 | Obstacle contacts (fleet) | 0.05 | 0.33 | +550.0 % | ±332.0 % | 0.007 | +0.37 | worse |
| cam2 | Landing error | 0.467 | 1.134 | +142.5 % | ±27.2 % | <0.001 | +1.16 | worse |
| cam2 | Goals | 3.4 | 1.1 | -67.5 % | ±8.6 % | <0.001 | -1.75 | worse |
| cam2_sensor | Energy | 24.560 | 24.332 | -0.9 % | ±3.0 % | 0.547 | -0.07 | same |
| cam2_sensor | Flight time | 144.8 | 143.4 | -1.0 % | ±2.9 % | 0.797 | -0.07 | same |
| cam2_sensor | Mission time | 43.5 | 41.4 | -4.8 % | ±4.7 % | 0.165 | -0.23 | same |
| cam2_sensor | Position error | 0.271 | 0.142 | -47.4 % | ±8.0 % | <0.001 | -1.32 | better |
| cam2_sensor | Tracking error | 0.236 | 0.181 | -23.4 % | ±6.5 % | <0.001 | -0.79 | better |
| cam2_sensor | Path deviation | 1.206 | 1.200 | -0.5 % | ±4.5 % | 0.843 | -0.02 | same |
| cam2_sensor | Separation margin (fleet) | -0.16 | -0.18 | -0.02 m | ±0.04 m | 0.624 | -0.10 | same |
| cam2_sensor | Time in downwash (fleet) | 0.0 | 0.0 | +0.03 s | ±0.04 s | 1.000 | +0.16 | same |
| cam2_sensor | Drone-drone contacts (fleet) | 0.03 | 0.01 | -50.0 % | ±173.1 % | 1.000 | -0.06 | same |
| cam2_sensor | Obstacle contacts (fleet) | 0.05 | 0 | -100.0 % | ±139.9 % | 1.000 | -0.16 | same |
| cam2_sensor | Landing error | 0.467 | 0.251 | -46.4 % | ±8.8 % | <0.001 | -1.18 | better |
| cam2_sensor | Goals | 3.4 | 4.6 | +38.1 % | ±6.8 % | <0.001 | +1.25 | better |
| cam3_sensor_layers | Energy | 24.560 | 26.900 | +9.5 % | ±2.7 % | <0.001 | +0.78 | worse |
| cam3_sensor_layers | Flight time | 144.8 | 158.1 | +9.1 % | ±2.7 % | <0.001 | +0.77 | worse |
| cam3_sensor_layers | Mission time | 43.5 | 41.6 | -4.5 % | ±4.6 % | 0.172 | -0.21 | same |
| cam3_sensor_layers | Position error | 0.271 | 0.318 | +17.4 % | ±8.3 % | <0.001 | +0.47 | worse |
| cam3_sensor_layers | Tracking error | 0.236 | 0.244 | +3.0 % | ±7.0 % | 0.484 | +0.10 | same |
| cam3_sensor_layers | Path deviation | 1.206 | 0.916 | -24.0 % | ±4.8 % | <0.001 | -1.12 | better |
| cam3_sensor_layers | Separation margin (fleet) | -0.16 | -1.04 | -0.88 m | ±0.12 m | <0.001 | -1.70 | worse |
| cam3_sensor_layers | Time in downwash (fleet) | 0.0 | 0.5 | +0.55 s | ±0.11 s | <0.001 | +1.13 | worse |
| cam3_sensor_layers | Drone-drone contacts (fleet) | 0.03 | 0 | -100.0 % | ±139.9 % | 1.000 | -0.16 | same |
| cam3_sensor_layers | Obstacle contacts (fleet) | 0.05 | 0.04 | -25.0 % | ±86.6 % | 1.000 | -0.06 | same |
| cam3_sensor_layers | Landing error | 0.467 | 0.562 | +20.2 % | ±9.4 % | <0.001 | +0.48 | worse |
| cam3_sensor_layers | Goals | 3.4 | 2.6 | -21.3 % | ±6.4 % | <0.001 | -0.73 | worse |
| cam3_sensor_swarm | Energy | 24.560 | 25.269 | +2.9 % | ±3.4 % | 0.200 | +0.19 | same |
| cam3_sensor_swarm | Flight time | 144.8 | 147.3 | +1.7 % | ±3.4 % | 0.797 | +0.11 | same |
| cam3_sensor_swarm | Mission time | 43.5 | 45.2 | +3.9 % | ±4.8 % | 0.218 | +0.18 | same |
| cam3_sensor_swarm | Position error | 0.271 | 0.230 | -14.9 % | ±8.2 % | <0.001 | -0.40 | better |
| cam3_sensor_swarm | Tracking error | 0.236 | 0.248 | +4.7 % | ±8.0 % | 0.484 | +0.13 | same |
| cam3_sensor_swarm | Path deviation | 1.206 | 1.350 | +12.0 % | ±6.6 % | 0.001 | +0.40 | worse |
| cam3_sensor_swarm | Separation margin (fleet) | -0.16 | 0.28 | +0.44 m | ±0.06 m | <0.001 | +1.75 | better |
| cam3_sensor_swarm | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| cam3_sensor_swarm | Drone-drone contacts (fleet) | 0.03 | 0 | -100.0 % | ±139.9 % | 1.000 | -0.16 | same |
| cam3_sensor_swarm | Obstacle contacts (fleet) | 0.05 | 0 | -100.0 % | ±139.9 % | 1.000 | -0.16 | same |
| cam3_sensor_swarm | Landing error | 0.467 | 0.387 | -17.3 % | ±9.4 % | <0.001 | -0.41 | better |
| cam3_sensor_swarm | Goals | 3.4 | 3.6 | +9.0 % | ±7.6 % | 0.027 | +0.26 | better |
| cam3_swarm | Energy | 24.560 | 42.251 | +72.0 % | ±8.8 % | <0.001 | +1.82 | worse |
| cam3_swarm | Flight time | 144.8 | 246.0 | +69.8 % | ±8.7 % | <0.001 | +1.79 | worse |
| cam3_swarm | Mission time | 43.5 | 85.6 | +96.8 % | ±17.6 % | <0.001 | +1.22 | worse |
| cam3_swarm | Position error | 0.271 | 2.221 | +721.1 % | ±87.6 % | <0.001 | +1.83 | worse |
| cam3_swarm | Tracking error | 0.236 | 1.670 | +606.2 % | ±74.7 % | <0.001 | +1.81 | worse |
| cam3_swarm | Path deviation | 1.206 | 2.027 | +68.2 % | ±11.3 % | <0.001 | +1.34 | worse |
| cam3_swarm | Separation margin (fleet) | -0.16 | -0.08 | +0.08 m | ±0.07 m | 0.065 | +0.25 | same |
| cam3_swarm | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| cam3_swarm | Drone-drone contacts (fleet) | 0.03 | 0.24 | +850.0 % | ±542.9 % | 0.020 | +0.35 | worse |
| cam3_swarm | Obstacle contacts (fleet) | 0.05 | 1.3 | +2500.0 % | ±761.1 % | <0.001 | +0.73 | worse |
| cam3_swarm | Landing error | 0.467 | 2.463 | +426.9 % | ±60.8 % | <0.001 | +1.56 | worse |
| cam3_swarm | Goals | 3.4 | 0.42 | -87.3 % | ±8.3 % | <0.001 | -2.34 | worse |
| cam2_swarm | Energy | 24.560 | 28.047 | +14.2 % | ±3.5 % | <0.001 | +0.89 | worse |
| cam2_swarm | Flight time | 144.8 | 164.0 | +13.2 % | ±3.5 % | <0.001 | +0.85 | worse |
| cam2_swarm | Mission time | 43.5 | 51.3 | +18.0 % | ±6.9 % | <0.001 | +0.58 | worse |
| cam2_swarm | Position error | 0.271 | 0.545 | +101.4 % | ±18.9 % | <0.001 | +1.19 | worse |
| cam2_swarm | Tracking error | 0.236 | 0.430 | +82.0 % | ±15.3 % | <0.001 | +1.19 | worse |
| cam2_swarm | Path deviation | 1.206 | 1.524 | +26.4 % | ±8.2 % | <0.001 | +0.71 | worse |
| cam2_swarm | Separation margin (fleet) | -0.16 | 0.21 | +0.37 m | ±0.06 m | <0.001 | +1.32 | better |
| cam2_swarm | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| cam2_swarm | Drone-drone contacts (fleet) | 0.03 | 0.05 | +100.0 % | ±346.2 % | 1.000 | +0.06 | same |
| cam2_swarm | Obstacle contacts (fleet) | 0.05 | 0.25 | +400.0 % | ±268.7 % | 0.030 | +0.33 | worse |
| cam2_swarm | Landing error | 0.467 | 0.940 | +101.1 % | ±19.1 % | <0.001 | +1.18 | worse |
| cam2_swarm | Goals | 3.4 | 1.4 | -57.8 % | ±9.5 % | <0.001 | -1.36 | worse |
| all | Energy | 24.560 | 25.303 | +3.0 % | ±3.2 % | 0.199 | +0.21 | same |
| all | Flight time | 144.8 | 147.4 | +1.8 % | ±3.1 % | 0.797 | +0.13 | same |
| all | Mission time | 43.5 | 45.4 | +4.4 % | ±5.5 % | 0.218 | +0.18 | same |
| all | Position error | 0.271 | 0.127 | -52.9 % | ±7.8 % | <0.001 | -1.52 | better |
| all | Tracking error | 0.236 | 0.211 | -10.6 % | ±8.0 % | 0.029 | -0.29 | better |
| all | Path deviation | 1.206 | 1.343 | +11.4 % | ±6.6 % | 0.002 | +0.38 | worse |
| all | Separation margin (fleet) | -0.16 | 0.28 | +0.44 m | ±0.05 m | <0.001 | +1.96 | better |
| all | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| all | Drone-drone contacts (fleet) | 0.03 | 0 | -100.0 % | ±139.9 % | 1.000 | -0.16 | same |
| all | Obstacle contacts (fleet) | 0.05 | 0 | -100.0 % | ±139.9 % | 1.000 | -0.16 | same |
| all | Landing error | 0.467 | 0.223 | -52.3 % | ±9.3 % | <0.001 | -1.25 | better |
| all | Goals | 3.4 | 4.7 | +39.9 % | ±7.3 % | <0.001 | +1.21 | better |

## Mixed fleets

720 missions; 10 forests x 6 seeds x 12 configurations; 297 paired tests (Holm-corrected per metric across the configurations).

| configuration | metric | baseline | config | change | 95 % CI of change | p (Holm) | d_z | verdict |
|---|---|---|---|---|---|---|---|---|
| k0 | Energy | 23.756 | 25.847 | +8.8 % | ±7.6 % | 0.070 | +0.30 | same |
| k0 | Flight time | 140.1 | 155.1 | +10.7 % | ±7.9 % | 0.006 | +0.35 | worse |
| k0 | Mission time | 40.7 | 53.0 | +30.4 % | ±24.3 % | 0.001 | +0.32 | worse |
| k0 | Position error | 0.042 | 0.417 | +895.4 % | ±184.1 % | <0.001 | +1.26 | worse |
| k0 | Tracking error | 0.150 | 0.278 | +85.4 % | ±17.4 % | <0.001 | +1.27 | worse |
| k0 | Path deviation | 1.187 | 1.256 | +5.8 % | ±5.4 % | 0.176 | +0.28 | same |
| k0 | Separation margin (fleet) | -0.14 | -0.16 | -0.02 m | ±0.02 m | 0.239 | -0.26 | same |
| k0 | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| k0 | Drone-drone contacts (fleet) | 0 | 0.03 | +0.03 # | ±0.05 # | 1.000 | +0.18 | same |
| k0 | Obstacle contacts (fleet) | 0 | 0.08 | +0.08 # | ±0.10 # | 1.000 | +0.22 | same |
| k0 | Landing error | 0.067 | 0.733 | +989.9 % | ±173.2 % | <0.001 | +1.48 | worse |
| k0 | Goals | 5 | 2.3 | -53.3 % | ±5.6 % | <0.001 | -2.46 | worse |
| k1 | Energy | 23.756 | 24.385 | +2.6 % | ±3.1 % | 0.257 | +0.22 | same |
| k1 | Flight time | 140.1 | 145.5 | +3.9 % | ±3.0 % | 0.074 | +0.33 | same |
| k1 | Mission time | 40.7 | 44.4 | +9.2 % | ±6.3 % | 0.012 | +0.38 | worse |
| k1 | Position error | 0.042 | 0.287 | +585.9 % | ±73.6 % | <0.001 | +2.06 | worse |
| k1 | Tracking error | 0.150 | 0.240 | +59.7 % | ±12.7 % | <0.001 | +1.21 | worse |
| k1 | Path deviation | 1.187 | 1.203 | +1.4 % | ±4.5 % | 1.000 | +0.08 | same |
| k1 | Separation margin (fleet) | -0.14 | -0.16 | -0.02 m | ±0.03 m | 1.000 | -0.12 | same |
| k1 | Time in downwash (fleet) | 0.0 | 0.0 | +0.02 s | ±0.04 s | 1.000 | +0.13 | same |
| k1 | Drone-drone contacts (fleet) | 0 | 0.03 | +0.03 # | ±0.05 # | 1.000 | +0.18 | same |
| k1 | Obstacle contacts (fleet) | 0 | 0.08 | +0.08 # | ±0.09 # | 1.000 | +0.25 | same |
| k1 | Landing error | 0.067 | 0.517 | +668.8 % | ±86.2 % | <0.001 | +2.00 | worse |
| k1 | Goals | 5 | 3.1 | -38.7 % | ±5.7 % | <0.001 | -1.75 | worse |
| k2 | Energy | 23.756 | 25.081 | +5.6 % | ±6.0 % | 0.120 | +0.24 | same |
| k2 | Flight time | 140.1 | 149.4 | +6.6 % | ±6.2 % | 0.040 | +0.27 | worse |
| k2 | Mission time | 40.7 | 47.7 | +17.3 % | ±18.9 % | 0.011 | +0.24 | worse |
| k2 | Position error | 0.042 | 0.261 | +524.7 % | ±170.0 % | <0.001 | +0.80 | worse |
| k2 | Tracking error | 0.150 | 0.226 | +50.3 % | ±12.7 % | <0.001 | +1.02 | worse |
| k2 | Path deviation | 1.187 | 1.213 | +2.2 % | ±5.9 % | 1.000 | +0.10 | same |
| k2 | Separation margin (fleet) | -0.14 | -0.14 | +0.00 m | ±0.01 m | 1.000 | +0.04 | same |
| k2 | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| k2 | Drone-drone contacts (fleet) | 0 | 0.02 | +0.02 # | ±0.03 # | 1.000 | +0.13 | same |
| k2 | Obstacle contacts (fleet) | 0 | 0.13 | +0.13 # | ±0.15 # | 0.676 | +0.24 | same |
| k2 | Landing error | 0.067 | 0.451 | +570.7 % | ±129.2 % | <0.001 | +1.14 | worse |
| k2 | Goals | 5 | 3.6 | -28.3 % | ±4.4 % | <0.001 | -1.67 | worse |
| k3 | Energy | 23.756 | 23.897 | +0.6 % | ±2.4 % | 1.000 | +0.06 | same |
| k3 | Flight time | 140.1 | 141.8 | +1.2 % | ±2.4 % | 0.620 | +0.13 | same |
| k3 | Mission time | 40.7 | 41.7 | +2.6 % | ±4.2 % | 0.475 | +0.16 | same |
| k3 | Position error | 0.042 | 0.160 | +283.2 % | ±50.6 % | <0.001 | +1.45 | worse |
| k3 | Tracking error | 0.150 | 0.194 | +29.3 % | ±8.5 % | <0.001 | +0.89 | worse |
| k3 | Path deviation | 1.187 | 1.186 | -0.1 % | ±3.5 % | 1.000 | -0.01 | same |
| k3 | Separation margin (fleet) | -0.14 | -0.13 | +0.01 m | ±0.01 m | 0.343 | +0.23 | same |
| k3 | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| k3 | Drone-drone contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| k3 | Obstacle contacts (fleet) | 0 | 0.02 | +0.02 # | ±0.03 # | 1.000 | +0.13 | same |
| k3 | Landing error | 0.067 | 0.283 | +320.8 % | ±56.6 % | <0.001 | +1.46 | worse |
| k3 | Goals | 5 | 4.1 | -18.7 % | ±3.7 % | <0.001 | -1.31 | worse |
| k4 | Energy | 23.756 | 24.094 | +1.4 % | ±5.8 % | 1.000 | +0.06 | same |
| k4 | Flight time | 140.1 | 142.8 | +2.0 % | ±6.1 % | 0.935 | +0.08 | same |
| k4 | Mission time | 40.7 | 43.9 | +8.1 % | ±18.9 % | 0.822 | +0.11 | same |
| k4 | Position error | 0.042 | 0.143 | +240.9 % | ±163.0 % | <0.001 | +0.38 | worse |
| k4 | Tracking error | 0.150 | 0.176 | +17.2 % | ±8.8 % | <0.001 | +0.51 | worse |
| k4 | Path deviation | 1.187 | 1.199 | +1.0 % | ±5.2 % | 1.000 | +0.05 | same |
| k4 | Separation margin (fleet) | -0.14 | -0.14 | +0.00 m | ±0.01 m | 1.000 | +0.06 | same |
| k4 | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| k4 | Drone-drone contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| k4 | Obstacle contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| k4 | Landing error | 0.067 | 0.226 | +236.8 % | ±112.0 % | <0.001 | +0.55 | worse |
| k4 | Goals | 5 | 4.4 | -12.3 % | ±2.5 % | <0.001 | -1.26 | worse |
| k0_swarm | Energy | 23.756 | 26.125 | +10.0 % | ±8.8 % | 0.067 | +0.29 | same |
| k0_swarm | Flight time | 140.1 | 154.8 | +10.5 % | ±9.0 % | 0.040 | +0.30 | worse |
| k0_swarm | Mission time | 40.7 | 53.6 | +31.7 % | ±24.6 % | <0.001 | +0.33 | worse |
| k0_swarm | Position error | 0.042 | 0.391 | +833.7 % | ±168.9 % | <0.001 | +1.27 | worse |
| k0_swarm | Tracking error | 0.150 | 0.311 | +107.5 % | ±20.7 % | <0.001 | +1.34 | worse |
| k0_swarm | Path deviation | 1.187 | 1.405 | +18.3 % | ±7.0 % | <0.001 | +0.68 | worse |
| k0_swarm | Separation margin (fleet) | -0.14 | 0.32 | +0.46 m | ±0.06 m | <0.001 | +1.94 | better |
| k0_swarm | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| k0_swarm | Drone-drone contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| k0_swarm | Obstacle contacts (fleet) | 0 | 0.08 | +0.08 # | ±0.09 # | 1.000 | +0.25 | same |
| k0_swarm | Landing error | 0.067 | 0.676 | +904.9 % | ±116.6 % | <0.001 | +2.01 | worse |
| k0_swarm | Goals | 5 | 2.4 | -53.0 % | ±5.9 % | <0.001 | -2.31 | worse |
| k1_swarm | Energy | 23.756 | 24.890 | +4.8 % | ±3.4 % | 0.067 | +0.36 | same |
| k1_swarm | Flight time | 140.1 | 146.7 | +4.7 % | ±3.4 % | 0.053 | +0.36 | same |
| k1_swarm | Mission time | 40.7 | 45.1 | +10.9 % | ±4.7 % | <0.001 | +0.61 | worse |
| k1_swarm | Position error | 0.042 | 0.199 | +376.8 % | ±40.5 % | <0.001 | +2.40 | worse |
| k1_swarm | Tracking error | 0.150 | 0.251 | +67.5 % | ±13.6 % | <0.001 | +1.28 | worse |
| k1_swarm | Path deviation | 1.187 | 1.387 | +16.8 % | ±7.3 % | <0.001 | +0.59 | worse |
| k1_swarm | Separation margin (fleet) | -0.14 | 0.32 | +0.46 m | ±0.07 m | <0.001 | +1.80 | better |
| k1_swarm | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| k1_swarm | Drone-drone contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| k1_swarm | Obstacle contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| k1_swarm | Landing error | 0.067 | 0.288 | +328.5 % | ±43.5 % | <0.001 | +1.95 | worse |
| k1_swarm | Goals | 5 | 4.2 | -17.0 % | ±4.5 % | <0.001 | -0.97 | worse |
| k2_swarm | Energy | 23.756 | 24.756 | +4.2 % | ±2.6 % | 0.024 | +0.41 | worse |
| k2_swarm | Flight time | 140.1 | 145.4 | +3.8 % | ±2.6 % | 0.040 | +0.38 | worse |
| k2_swarm | Mission time | 40.7 | 44.5 | +9.4 % | ±4.2 % | <0.001 | +0.57 | worse |
| k2_swarm | Position error | 0.042 | 0.122 | +190.5 % | ±22.8 % | <0.001 | +2.16 | worse |
| k2_swarm | Tracking error | 0.150 | 0.220 | +46.6 % | ±12.6 % | <0.001 | +0.96 | worse |
| k2_swarm | Path deviation | 1.187 | 1.358 | +14.4 % | ±7.8 % | 0.004 | +0.47 | worse |
| k2_swarm | Separation margin (fleet) | -0.14 | 0.32 | +0.46 m | ±0.06 m | <0.001 | +2.09 | better |
| k2_swarm | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| k2_swarm | Drone-drone contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| k2_swarm | Obstacle contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| k2_swarm | Landing error | 0.067 | 0.177 | +163.7 % | ±25.6 % | <0.001 | +1.65 | worse |
| k2_swarm | Goals | 5 | 4.8 | -5.0 % | ±2.6 % | 0.001 | -0.49 | worse |
| k3_swarm | Energy | 23.756 | 24.673 | +3.9 % | ±2.8 % | 0.070 | +0.35 | same |
| k3_swarm | Flight time | 140.1 | 144.6 | +3.2 % | ±2.8 % | 0.123 | +0.30 | same |
| k3_swarm | Mission time | 40.7 | 43.8 | +7.8 % | ±4.0 % | 0.002 | +0.50 | worse |
| k3_swarm | Position error | 0.042 | 0.087 | +107.0 % | ±14.5 % | <0.001 | +1.91 | worse |
| k3_swarm | Tracking error | 0.150 | 0.201 | +33.9 % | ±14.6 % | <0.001 | +0.60 | worse |
| k3_swarm | Path deviation | 1.187 | 1.358 | +14.4 % | ±7.8 % | 0.004 | +0.48 | worse |
| k3_swarm | Separation margin (fleet) | -0.14 | 0.31 | +0.45 m | ±0.06 m | <0.001 | +1.92 | better |
| k3_swarm | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| k3_swarm | Drone-drone contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| k3_swarm | Obstacle contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| k3_swarm | Landing error | 0.067 | 0.123 | +83.6 % | ±21.4 % | <0.001 | +1.01 | worse |
| k3_swarm | Goals | 5 | 4.9 | -1.7 % | ±1.4 % | 0.127 | -0.30 | same |
| k4_swarm | Energy | 23.756 | 24.713 | +4.0 % | ±3.3 % | 0.101 | +0.31 | same |
| k4_swarm | Flight time | 140.1 | 144.4 | +3.1 % | ±3.3 % | 0.243 | +0.25 | same |
| k4_swarm | Mission time | 40.7 | 43.8 | +7.8 % | ±4.3 % | 0.003 | +0.47 | worse |
| k4_swarm | Position error | 0.042 | 0.066 | +58.5 % | ±8.6 % | <0.001 | +1.76 | worse |
| k4_swarm | Tracking error | 0.150 | 0.196 | +30.7 % | ±13.9 % | <0.001 | +0.57 | worse |
| k4_swarm | Path deviation | 1.187 | 1.354 | +14.1 % | ±7.6 % | 0.004 | +0.48 | worse |
| k4_swarm | Separation margin (fleet) | -0.14 | 0.32 | +0.46 m | ±0.06 m | <0.001 | +2.03 | better |
| k4_swarm | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| k4_swarm | Drone-drone contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| k4_swarm | Obstacle contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| k4_swarm | Landing error | 0.067 | 0.106 | +57.1 % | ±22.8 % | <0.001 | +0.65 | worse |
| k4_swarm | Goals | 5 | 4.9 | -2.0 % | ±1.6 % | 0.091 | -0.33 | same |
| k5_swarm | Energy | 23.756 | 24.476 | +3.0 % | ±2.8 % | 0.144 | +0.28 | same |
| k5_swarm | Flight time | 140.1 | 142.7 | +1.9 % | ±2.8 % | 0.547 | +0.17 | same |
| k5_swarm | Mission time | 40.7 | 43.4 | +6.6 % | ±4.1 % | 0.011 | +0.42 | worse |
| k5_swarm | Position error | 0.042 | 0.044 | +4.9 % | ±5.8 % | 0.091 | +0.22 | same |
| k5_swarm | Tracking error | 0.150 | 0.187 | +24.3 % | ±14.2 % | <0.001 | +0.44 | worse |
| k5_swarm | Path deviation | 1.187 | 1.318 | +11.0 % | ±8.0 % | 0.048 | +0.35 | worse |
| k5_swarm | Separation margin (fleet) | -0.14 | 0.31 | +0.46 m | ±0.06 m | <0.001 | +1.93 | better |
| k5_swarm | Time in downwash (fleet) | 0.0 | 0.0 | +0.00 s | ±0.00 s | 1.000 | +0.00 | same |
| k5_swarm | Drone-drone contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| k5_swarm | Obstacle contacts (fleet) | 0 | 0 | +0.00 # | ±0.00 # | 1.000 | +0.00 | same |
| k5_swarm | Landing error | 0.067 | 0.067 | -0.0 % | ±19.8 % | 0.999 | -0.00 | same |
| k5_swarm | Goals | 5 | 5.0 | -0.3 % | ±0.7 % | 1.000 | -0.13 | same |

- cam3_sensor: average power 169.5 W, of which navigation electronics 3.5 W and other electronics 6.5 W (5.9 % of the total); hover rotor power 157 W.
- cam3_sensor_swarm: average power 171.5 W, of which navigation electronics 6.0 W and other electronics 6.5 W (7.3 % of the total); hover rotor power 157 W.

## Robustness of the coordination findings (downwash strength, layer spacing)

Same forests and seeds in every row. Changes are paired vs the baseline of the same setting; ▲/▼ = significant (Holm) and > 2 %.

| setting | configuration | fleet energy | fleet flight time | sep. margin (m) | downwash (s) | strongest downdraft (m/s) | drone contacts |
|---|---|---|---|---|---|---|---|
| default model (downwash x1, layer gap 0.6 m) (60 missions) | Baseline | 23.8 kJ | 140 s | −0.14 | 0.00 | 0.00 | 0.00 |
| default model (downwash x1, layer gap 0.6 m) (60 missions) | C3+V+L | 26.9 kJ (+13.1 % ▼) | 158 s (+12.6 % ▼) | −0.89 | 0.47 | 1.46 | 0.00 |
| default model (downwash x1, layer gap 0.6 m) (60 missions) | C3+V+W | 24.5 kJ (+3.0 % ▼) | 143 s (+1.9 %) | +0.31 | 0.00 | 0.00 | 0.00 |
| downwash x0.5 (60 missions) | Baseline | 23.7 kJ | 140 s | −0.14 | 0.00 | 0.00 | 0.00 |
| downwash x0.5 (60 missions) | C3+V+L | 26.3 kJ (+10.9 % ▼) | 155 s (+10.6 % ▼) | −0.66 | 0.37 | 0.98 | 0.00 |
| downwash x0.5 (60 missions) | C3+V+W | 24.6 kJ (+3.4 % ▼) | 143 s (+2.2 %) | +0.31 | 0.00 | 0.00 | 0.00 |
| layer gap 1 m (60 missions) | Baseline | 23.8 kJ | 140 s | −0.14 | 0.00 | 0.00 | 0.00 |
| layer gap 1 m (60 missions) | C3+V+L | 27.6 kJ (+16.3 % ▼) | 162 s (+15.5 % ▼) | −1.34 | 1.22 | 2.04 | 0.00 |
| layer gap 1 m (60 missions) | C3+V+W | 24.5 kJ (+3.0 % ▼) | 143 s (+1.9 %) | +0.31 | 0.00 | 0.00 | 0.00 |
| layer gap 1.5 m (60 missions) | Baseline | 23.8 kJ | 140 s | −0.14 | 0.00 | 0.00 | 0.00 |
| layer gap 1.5 m (60 missions) | C3+V+L | 28.9 kJ (+21.6 % ▼) | 169 s (+20.5 % ▼) | −1.45 | 2.10 | 2.03 | 0.00 |
| layer gap 1.5 m (60 missions) | C3+V+W | 24.5 kJ (+3.0 % ▼) | 143 s (+1.9 %) | +0.31 | 0.00 | 0.00 | 0.00 |


## Path planner: RRT* (the drones' planner) vs A* (comparison)

The same missions (40 per configuration: the forests and seeds of study_headless_astar_L10_20261120_081411, also flown by the main study). Change = RRT* - A*, paired by mission, with its 95 % CI; p = sign-flip permutation test, Holm-corrected across the configurations per metric. Path deviation is measured from each drone's reference route (A*, planned on board before take-off) for both planners. A*'s first route in flight is nearly that same route, so path deviation favours A* by construction; energy, flight time and flown/reference length compare the planners on equal terms. Planning times are wall clock and depend on the computer.

| configuration | metric | A* | RRT* | change | 95 % CI | p (Holm) |
|---|---|---|---|---|---|---|
| Baseline | Energy, fleet (kJ) | 23.5 | 23.6 | +0.3 % | ±2.5 % | 1.000 |
| Baseline | Flight time, fleet (s) | 139 | 139 | +0.2 % | ±2.4 % | 1.000 |
| Baseline | Mission time (s) | 39.7 | 40.2 | +1.3 % | ±3.7 % | 1.000 |
| Baseline | Flown / reference length, mean | 1.110 | 1.109 | -0.1 % | ±1.8 % | 1.000 |
| Baseline | Planning time per route (ms) | 7 | 100 | +1378.3 % | ±113.0 % | <0.001 |
| Baseline | Route switches, fleet | 17.7 | 20.9 | +18.4 % | ±10.6 % | 0.002 |
| Baseline | Goals (of 5) | 4.97 | 5.00 | +0.5 % | ±1.0 % | 1.000 |
| Baseline | Sep. margin (m) | -0.13 | -0.14 | -0.00886 | ±0.0275 | 1.000 |
| Baseline | Downwash (s) | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| Baseline | Obstacle contacts | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| Baseline | Drone contacts | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| Baseline | Position error (m) | 0.043 | 0.041 | -4.5 % | ±7.4 % | 1.000 |
| Baseline | Tracking error (m) | 0.145 | 0.146 | +0.7 % | ±7.7 % | 1.000 |
| Baseline | Path deviation from the A* reference (m; favours A*) | 1.082 | 1.215 | +12.4 % | ±8.8 % | 0.043 |
| C2+V | Energy, fleet (kJ) | 23.4 | 24.0 | +2.6 % | ±3.3 % | 0.970 |
| C2+V | Flight time, fleet (s) | 138 | 142 | +2.6 % | ±3.2 % | 0.870 |
| C2+V | Mission time (s) | 39.8 | 40.6 | +1.9 % | ±3.8 % | 1.000 |
| C2+V | Flown / reference length, mean | 1.106 | 1.121 | +1.4 % | ±2.4 % | 1.000 |
| C2+V | Planning time per route (ms) | 7 | 96 | +1289.0 % | ±117.3 % | <0.001 |
| C2+V | Route switches, fleet | 16.5 | 22.8 | +38.1 % | ±10.5 % | <0.001 |
| C2+V | Goals (of 5) | 5.00 | 5.00 | +0.0 % | ±0.0 % | 1.000 |
| C2+V | Sep. margin (m) | -0.14 | -0.14 | +0.00478 | ±0.022 | 1.000 |
| C2+V | Downwash (s) | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C2+V | Obstacle contacts | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C2+V | Drone contacts | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C2+V | Position error (m) | 0.038 | 0.037 | -0.6 % | ±2.1 % | 1.000 |
| C2+V | Tracking error (m) | 0.148 | 0.146 | -1.2 % | ±8.0 % | 1.000 |
| C2+V | Path deviation from the A* reference (m; favours A*) | 1.114 | 1.214 | +9.0 % | ±7.0 % | 0.063 |
| C3+V+W | Energy, fleet (kJ) | 24.4 | 24.6 | +0.8 % | ±3.2 % | 1.000 |
| C3+V+W | Flight time, fleet (s) | 142 | 143 | +0.8 % | ±3.2 % | 1.000 |
| C3+V+W | Mission time (s) | 43.7 | 43.3 | -1.0 % | ±4.2 % | 1.000 |
| C3+V+W | Flown / reference length, mean | 1.094 | 1.103 | +0.8 % | ±2.5 % | 1.000 |
| C3+V+W | Planning time per route (ms) | 9 | 103 | +1110.2 % | ±106.8 % | <0.001 |
| C3+V+W | Route switches, fleet | 9.9 | 15.5 | +56.4 % | ±17.0 % | <0.001 |
| C3+V+W | Goals (of 5) | 4.97 | 4.97 | +0.0 % | ±1.5 % | 1.000 |
| C3+V+W | Sep. margin (m) | +0.35 | +0.30 | -0.0461 | ±0.0918 | 1.000 |
| C3+V+W | Downwash (s) | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C3+V+W | Obstacle contacts | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C3+V+W | Drone contacts | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C3+V+W | Position error (m) | 0.044 | 0.043 | -3.8 % | ±4.9 % | 1.000 |
| C3+V+W | Tracking error (m) | 0.190 | 0.177 | -6.9 % | ±16.8 % | 1.000 |
| C3+V+W | Path deviation from the A* reference (m; favours A*) | 1.248 | 1.281 | +2.6 % | ±7.9 % | 0.536 |
| C2+V+W | Energy, fleet (kJ) | 24.6 | 24.7 | +0.3 % | ±2.8 % | 1.000 |
| C2+V+W | Flight time, fleet (s) | 143 | 144 | +0.3 % | ±2.8 % | 1.000 |
| C2+V+W | Mission time (s) | 44.6 | 43.6 | -2.3 % | ±4.3 % | 1.000 |
| C2+V+W | Flown / reference length, mean | 1.100 | 1.116 | +1.4 % | ±2.6 % | 1.000 |
| C2+V+W | Planning time per route (ms) | 9 | 107 | +1077.1 % | ±173.5 % | <0.001 |
| C2+V+W | Route switches, fleet | 10.3 | 15.0 | +44.9 % | ±14.3 % | <0.001 |
| C2+V+W | Goals (of 5) | 5.00 | 4.95 | -1.0 % | ±1.4 % | 1.000 |
| C2+V+W | Sep. margin (m) | +0.38 | +0.26 | -0.118 | ±0.0848 | 0.056 |
| C2+V+W | Downwash (s) | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C2+V+W | Obstacle contacts | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C2+V+W | Drone contacts | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C2+V+W | Position error (m) | 0.038 | 0.037 | -1.0 % | ±2.8 % | 1.000 |
| C2+V+W | Tracking error (m) | 0.197 | 0.182 | -7.2 % | ±14.5 % | 1.000 |
| C2+V+W | Path deviation from the A* reference (m; favours A*) | 1.212 | 1.338 | +10.4 % | ±8.6 % | 0.067 |
| V | Energy, fleet (kJ) | 24.0 | 25.0 | +4.1 % | ±6.3 % | 1.000 |
| V | Flight time, fleet (s) | 144 | 150 | +4.1 % | ±6.4 % | 1.000 |
| V | Mission time (s) | 45.1 | 47.8 | +5.9 % | ±14.1 % | 1.000 |
| V | Flown / reference length, mean | 1.117 | 1.137 | +1.8 % | ±2.4 % | 1.000 |
| V | Planning time per route (ms) | 7 | 95 | +1252.3 % | ±99.0 % | <0.001 |
| V | Route switches, fleet | 20.2 | 24.4 | +20.8 % | ±11.2 % | <0.001 |
| V | Goals (of 5) | 2.52 | 2.38 | -5.9 % | ±7.9 % | 1.000 |
| V | Sep. margin (m) | -0.14 | -0.16 | -0.013 | ±0.019 | 1.000 |
| V | Downwash (s) | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| V | Obstacle contacts | 0.15 | 0.07 | -50.0 % | ±56.9 % | 1.000 |
| V | Drone contacts | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| V | Position error (m) | 0.350 | 0.373 | +6.5 % | ±13.0 % | 1.000 |
| V | Tracking error (m) | 0.258 | 0.272 | +5.4 % | ±6.0 % | 0.590 |
| V | Path deviation from the A* reference (m; favours A*) | 1.141 | 1.257 | +10.2 % | ±7.5 % | 0.050 |
| V+W | Energy, fleet (kJ) | 25.0 | 24.9 | -0.4 % | ±2.7 % | 1.000 |
| V+W | Flight time, fleet (s) | 148 | 147 | -0.3 % | ±2.7 % | 1.000 |
| V+W | Mission time (s) | 48.0 | 47.5 | -1.0 % | ±5.1 % | 1.000 |
| V+W | Flown / reference length, mean | 1.111 | 1.121 | +0.9 % | ±2.1 % | 1.000 |
| V+W | Planning time per route (ms) | 9 | 109 | +1150.8 % | ±125.6 % | <0.001 |
| V+W | Route switches, fleet | 14.6 | 18.9 | +29.5 % | ±14.1 % | <0.001 |
| V+W | Goals (of 5) | 2.25 | 2.40 | +6.7 % | ±8.8 % | 1.000 |
| V+W | Sep. margin (m) | +0.33 | +0.30 | -0.0222 | ±0.0958 | 1.000 |
| V+W | Downwash (s) | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| V+W | Obstacle contacts | 0.05 | 0.10 | +100.0 % | ±141.2 % | 1.000 |
| V+W | Drone contacts | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| V+W | Position error (m) | 0.350 | 0.348 | -0.5 % | ±4.6 % | 1.000 |
| V+W | Tracking error (m) | 0.294 | 0.301 | +2.6 % | ±8.6 % | 1.000 |
| V+W | Path deviation from the A* reference (m; favours A*) | 1.313 | 1.413 | +7.6 % | ±8.5 % | 0.234 |
| W | Energy, fleet (kJ) | 60.1 | 60.5 | +0.7 % | ±6.7 % | 1.000 |
| W | Flight time, fleet (s) | 353 | 355 | +0.6 % | ±6.6 % | 1.000 |
| W | Mission time (s) | 129.0 | 129.1 | +0.1 % | ±9.4 % | 1.000 |
| W | Flown / reference length, mean | 1.441 | 1.443 | +0.2 % | ±5.7 % | 1.000 |
| W | Planning time per route (ms) | 7 | 87 | +1145.7 % | ±121.8 % | <0.001 |
| W | Route switches, fleet | 71.3 | 71.6 | +0.4 % | ±15.1 % | 0.961 |
| W | Goals (of 5) | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| W | Sep. margin (m) | -0.40 | -0.44 | -0.0351 | ±0.214 | 1.000 |
| W | Downwash (s) | 0.25 | 0.22 | -0.0287 | ±0.215 | 1.000 |
| W | Obstacle contacts | 3.40 | 2.90 | -14.7 % | ±30.6 % | 1.000 |
| W | Drone contacts | 1.55 | 1.02 | -33.9 % | ±52.2 % | 1.000 |
| W | Position error (m) | 6.257 | 6.303 | +0.7 % | ±4.4 % | 1.000 |
| W | Tracking error (m) | 4.456 | 4.500 | +1.0 % | ±5.3 % | 1.000 |
| W | Path deviation from the A* reference (m; favours A*) | 3.445 | 3.529 | +2.4 % | ±4.3 % | 0.536 |
| C3+V+L | Energy, fleet (kJ) | 26.9 | 27.0 | +0.5 % | ±2.5 % | 1.000 |
| C3+V+L | Flight time, fleet (s) | 157 | 158 | +0.5 % | ±2.3 % | 1.000 |
| C3+V+L | Mission time (s) | 40.8 | 41.6 | +2.0 % | ±3.5 % | 1.000 |
| C3+V+L | Flown / reference length, mean | 1.072 | 1.079 | +0.6 % | ±2.2 % | 1.000 |
| C3+V+L | Planning time per route (ms) | 6 | 95 | +1384.3 % | ±104.6 % | <0.001 |
| C3+V+L | Route switches, fleet | 11.6 | 14.5 | +24.5 % | ±12.7 % | <0.001 |
| C3+V+L | Goals (of 5) | 4.97 | 4.97 | +0.0 % | ±0.0 % | 1.000 |
| C3+V+L | Sep. margin (m) | -1.06 | -0.92 | +0.145 | ±0.211 | 1.000 |
| C3+V+L | Downwash (s) | 0.60 | 0.48 | -0.126 | ±0.229 | 1.000 |
| C3+V+L | Obstacle contacts | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C3+V+L | Drone contacts | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C3+V+L | Position error (m) | 0.050 | 0.050 | -0.8 % | ±3.5 % | 1.000 |
| C3+V+L | Tracking error (m) | 0.150 | 0.148 | -1.4 % | ±10.7 % | 1.000 |
| C3+V+L | Path deviation from the A* reference (m; favours A*) | 0.760 | 0.882 | +16.0 % | ±9.7 % | 0.004 |


## Swarm give-way: turn trading (TTC, no priorities) vs fixed priorities (comparison)

The same missions (40 per configuration: the forests and seeds of study_headless_priority_L10_20261120_084413, also flown by the main study). Change = TTC - priority, paired by mission, with its 95 % CI; p = sign-flip permutation test, Holm-corrected across the configurations per metric. Configurations without the radio do not give way and fly the same missions in both studies.

| configuration | metric | priority | TTC | change | 95 % CI | p (Holm) |
|---|---|---|---|---|---|---|
| Baseline | Energy, fleet (kJ) | 23.6 | 23.6 | +0.0 % | ±0.0 % | 1.000 |
| Baseline | Flight time, fleet (s) | 139 | 139 | +0.0 % | ±0.0 % | 1.000 |
| Baseline | Mission time (s) | 40.2 | 40.2 | +0.0 % | ±0.0 % | 1.000 |
| Baseline | Time giving way, fleet (s) | 0.0 | 0.0 | +0 | ±0 | 1.000 |
| Baseline | Path deviation, fleet mean (m) | 1.215 | 1.215 | +0.0 % | ±0.0 % | 1.000 |
| Baseline | Goals (of 5) | 5.00 | 5.00 | +0.0 % | ±0.0 % | 1.000 |
| Baseline | Sep. margin (m) | -0.14 | -0.14 | +0 | ±0 | 1.000 |
| Baseline | Downwash (s) | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| Baseline | Drone contacts | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C3+V+W | Energy, fleet (kJ) | 27.6 | 24.6 | -10.8 % | ±2.8 % | <0.001 |
| C3+V+W | Flight time, fleet (s) | 161 | 143 | -10.8 % | ±2.8 % | <0.001 |
| C3+V+W | Mission time (s) | 43.3 | 43.3 | +0.1 % | ±4.1 % | 1.000 |
| C3+V+W | Time giving way, fleet (s) | 42.5 | 25.3 | -40.4 % | ±10.1 % | <0.001 |
| C3+V+W | Path deviation, fleet mean (m) | 1.473 | 1.281 | -13.0 % | ±5.6 % | <0.001 |
| C3+V+W | Goals (of 5) | 5.00 | 4.97 | -0.5 % | ±1.0 % | 1.000 |
| C3+V+W | Sep. margin (m) | +0.42 | +0.30 | -0.117 | ±0.0917 | 0.056 |
| C3+V+W | Downwash (s) | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C3+V+W | Drone contacts | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C2+V+W | Energy, fleet (kJ) | 27.5 | 24.7 | -10.3 % | ±2.3 % | <0.001 |
| C2+V+W | Flight time, fleet (s) | 160 | 144 | -10.3 % | ±2.2 % | <0.001 |
| C2+V+W | Mission time (s) | 42.2 | 43.6 | +3.2 % | ±3.5 % | 0.472 |
| C2+V+W | Time giving way, fleet (s) | 42.1 | 24.7 | -41.4 % | ±8.4 % | <0.001 |
| C2+V+W | Path deviation, fleet mean (m) | 1.435 | 1.338 | -6.7 % | ±5.4 % | 0.061 |
| C2+V+W | Goals (of 5) | 5.00 | 4.95 | -1.0 % | ±1.4 % | 1.000 |
| C2+V+W | Sep. margin (m) | +0.42 | +0.26 | -0.163 | ±0.098 | 0.012 |
| C2+V+W | Downwash (s) | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C2+V+W | Drone contacts | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C3+W | Energy, fleet (kJ) | 28.8 | 25.2 | -12.2 % | ±2.7 % | <0.001 |
| C3+W | Flight time, fleet (s) | 168 | 148 | -12.2 % | ±2.7 % | <0.001 |
| C3+W | Mission time (s) | 45.9 | 46.8 | +2.0 % | ±4.8 % | 1.000 |
| C3+W | Time giving way, fleet (s) | 45.4 | 26.0 | -42.8 % | ±7.8 % | <0.001 |
| C3+W | Path deviation, fleet mean (m) | 1.541 | 1.364 | -11.5 % | ±5.1 % | <0.001 |
| C3+W | Goals (of 5) | 3.95 | 4.03 | +1.9 % | ±8.1 % | 1.000 |
| C3+W | Sep. margin (m) | +0.43 | +0.33 | -0.0952 | ±0.0619 | 0.016 |
| C3+W | Downwash (s) | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C3+W | Drone contacts | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C2+W | Energy, fleet (kJ) | 27.5 | 24.7 | -10.2 % | ±2.7 % | <0.001 |
| C2+W | Flight time, fleet (s) | 161 | 144 | -10.2 % | ±2.7 % | <0.001 |
| C2+W | Mission time (s) | 42.4 | 44.0 | +3.9 % | ±4.3 % | 0.472 |
| C2+W | Time giving way, fleet (s) | 42.7 | 23.8 | -44.3 % | ±9.7 % | <0.001 |
| C2+W | Path deviation, fleet mean (m) | 1.485 | 1.321 | -11.0 % | ±7.4 % | 0.013 |
| C2+W | Goals (of 5) | 4.80 | 4.78 | -0.5 % | ±4.1 % | 1.000 |
| C2+W | Sep. margin (m) | +0.45 | +0.29 | -0.161 | ±0.1 | 0.016 |
| C2+W | Downwash (s) | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| C2+W | Drone contacts | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| V+W | Energy, fleet (kJ) | 28.9 | 24.9 | -13.9 % | ±4.5 % | <0.001 |
| V+W | Flight time, fleet (s) | 171 | 147 | -13.8 % | ±4.4 % | <0.001 |
| V+W | Mission time (s) | 51.1 | 47.5 | -7.0 % | ±12.9 % | 1.000 |
| V+W | Time giving way, fleet (s) | 41.7 | 23.0 | -44.9 % | ±8.4 % | <0.001 |
| V+W | Path deviation, fleet mean (m) | 1.543 | 1.413 | -8.4 % | ±6.8 % | 0.061 |
| V+W | Goals (of 5) | 1.85 | 2.40 | +29.7 % | ±14.6 % | 0.003 |
| V+W | Sep. margin (m) | +0.41 | +0.30 | -0.1 | ±0.0954 | 0.121 |
| V+W | Downwash (s) | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| V+W | Drone contacts | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| W | Energy, fleet (kJ) | 64.3 | 60.5 | -5.8 % | ±9.7 % | 0.472 |
| W | Flight time, fleet (s) | 379 | 355 | -6.1 % | ±9.8 % | 0.421 |
| W | Mission time (s) | 133.2 | 129.1 | -3.0 % | ±13.8 % | 1.000 |
| W | Time giving way, fleet (s) | 109.4 | 77.9 | -28.8 % | ±17.4 % | 0.001 |
| W | Path deviation, fleet mean (m) | 3.632 | 3.529 | -2.8 % | ±6.0 % | 0.684 |
| W | Goals (of 5) | 0.00 | 0.00 | +0 | ±0 | 1.000 |
| W | Sep. margin (m) | -0.45 | -0.44 | +0.00801 | ±0.275 | 1.000 |
| W | Downwash (s) | 0.28 | 0.22 | -0.0562 | ±0.222 | 1.000 |
| W | Drone contacts | 0.55 | 1.02 | +86.4 % | ±73.3 % | 0.213 |


## Gazebo validation (same missions flown in Gazebo and headless)

36 paired missions: 12 configurations x seeds 0, 1, 2, from validation_20261120_114019. Lost in both simulators (mean position error > 1 m), left out below: swarm.

| metric | Gazebo | headless | mean diff | median paired diff | Spearman r (runs) | rho (config means) |
|---|---|---|---|---|---|---|
| Flight energy (kJ) | 24.751 | 23.816 | +3.9 % | +3.4 % | 0.16 | 0.65 |
| Flight time (s) | 145.201 | 140.115 | +3.6 % | +3.4 % | 0.21 | 0.53 |
| Mission time (s) | 41.150 | 39.598 | +3.9 % | +2.4 % | 0.22 | 0.55 |
| Path deviation RMS, mean (m) | 1.257 | 1.155 | +8.9 % | +3.3 % | 0.75 | 0.39 |
| Tracking error RMS, mean (m) | 0.226 | 0.222 | +2.0 % | +1.1 % | 0.62 | 0.77 |
| Position error RMS, mean (m) | 0.105 | 0.097 | +7.9 % | +4.1 % | 0.94 | 0.96 |
| Landing error, mean (m) | 0.158 | 0.152 | +4.2 % | +8.1 % | 0.69 | 0.95 |
| Closest pair, 3-D (m) | 1.396 | 1.416 | -1.4 % | -2.4 % | 0.83 | 0.87 |
| Separation margin (m) | 0.031 | 0.067 | -0.036 (abs.) | - | 0.84 | 0.87 |
| Time in downwash (s) | 0.027 | 0.009 | +0.018 (abs.) | - | 0.67 | 1.00 |

Change vs the baseline (paired by seed), Gazebo | headless:

| configuration | Energy | Flight time | Tracking error | Position error |
|---|---|---|---|---|
| cam3 | -2.1 % \| -5.1 % | -1.3 % \| -4.5 % | +12.9 % \| -17.6 % | ×2.9 \| +72.1 % |
| cam2 | +4.8 % \| +2.4 % | +5.0 % \| +2.3 % | +13.8 % \| +20.5 % | +52.4 % \| +53.5 % |
| cam2_sensor | +2.6 % \| -0.8 % | +2.7 % \| -0.7 % | -4.2 % \| +0.7 % | +4.1 % \| +1.5 % |
| sensor | -3.0 % \| -4.8 % | -1.1 % \| -3.1 % | +64.1 % \| +78.4 % | ×10.2 \| ×9.5 |
| cam3_sensor_layers | +27.7 % \| +19.0 % | +26.5 % \| +18.4 % | +29.7 % \| +23.2 % | +10.9 % \| +1.5 % |
| cam3_sensor_swarm | -2.9 % \| +0.0 % | -3.4 % \| -0.9 % | +12.0 % \| +23.0 % | -2.7 % \| -8.6 % |
| cam3_swarm | -1.2 % \| +1.7 % | -1.3 % \| +1.1 % | +40.3 % \| +52.2 % | +51.3 % \| +51.4 % |
| cam2_swarm | -2.1 % \| -1.2 % | -2.2 % \| -1.8 % | +11.2 % \| +43.3 % | +56.6 % \| +51.1 % |
| all | -1.0 % \| +1.7 % | -1.5 % \| +0.8 % | +21.6 % \| +58.1 % | +2.4 % \| -3.9 % |
| sensor_swarm | -1.5 % \| +2.2 % | -0.5 % \| +2.8 % | +92.2 % \| ×2.2 | ×10.3 \| ×10.1 |
| swarm (lost) | +76.5 % \| ×2.2 | +78.2 % \| ×2.2 | ×24.3 \| ×24.1 | ×172.5 \| ×156.0 |
| agreement (rho; median abs. diff; same sign of effects > 2 %) | 0.73; 2.9 pp; 5/8 | 0.55; 2.6 pp; 6/8 | 0.76; 13.1 pp; 8/10 | 0.96; 6.1 pp; 9/10 |

