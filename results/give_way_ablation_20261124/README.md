# Give-way ablation: who yields vs how the manoeuvre is picked

**Question.** The paper credits turn trading's 10–14 % saving over fixed priorities to the traded passing order. In the code, the two rules also pick manoeuvres differently. Fixed priority (`_give_way_priority`) takes the safe velocity closest to the wanted one, with horizontal moves only. Turn trading (`_give_way_ttc`) takes the cheapest manoeuvre by energy (`_gw_energy`), with climb/descend also allowed.

**Ablation.** The new give-way mode `priority_energy` keeps fixed priorities (lower drone number passes first) but picks the manoeuvre with turn trading's own code (`_give_way_ttc`, with `ahead` = lower-numbered radio drones). Compared with `priority`, only the manoeuvre choice differs. Compared with `ttc`, only who yields differs. The patch is `priority_energy_ablation.patch` (17 lines added, 6 removed; applies to commit 7d707dd).

**Runs.** All three rules flew the same 40 missions (10 forests × 4 seeds: the forests and seeds of the paper's give-way comparison), for C3+V+W, C2+V+W, C3+W, C2+W and V+W, on one machine (Xeon 2.1 GHz, Python 3.13.18, numpy 2.5.3). This is not the reference VM, so individual missions differ in detail from the published runs.

**Machine.** These three studies were flown on the second computer of the robustness checks (Intel Xeon @ 2.10 GHz, Python 3.13.18, numpy 2.5.3, OpenBLAS kernel Prescott), not on the reference VM (Intel Core i7-10750H, Python 3.10.12, numpy 1.26.4) that flew the main studies. Each rule is therefore compared only with the other rules flown here, on the same 40 missions. The patch is not part of the package: apply it to fly the ablation (`--give-way priority_energy`). Re-running on the reference VM can shift single results by a point or two.

## Result (same machine; paired by mission; Holm-corrected sign-flip tests)

| Setup | Priced manoeuvres vs fixed | Trading vs priced manoeuvres | Trading vs fixed |
|---|---|---|---|
| C3+V+W | +0.3 ± 2.3 % (p = 1.00) | −10.3 ± 2.7 % (p < 0.001) | −10.1 ± 2.8 % |
| C2+V+W | +0.2 ± 1.9 % (p = 1.00) | −10.9 ± 2.8 % (p < 0.001) | −10.6 ± 2.4 % |
| C3+W | +2.0 ± 3.1 % (p = 0.80) | −10.0 ± 6.9 % (p = 0.001) | −8.2 ± 5.6 % |
| C2+W | −0.4 ± 2.6 % (p = 1.00) | −9.2 ± 2.6 % (p < 0.001) | −9.6 ± 2.6 % |
| V+W | +10.4 ± 12.2 % (p = 0.40)* | −21.9 ± 10.9 % (p < 0.001)* | −13.8 ± 4.4 % |

Per forest (10 forest means, exact sign-flip, Holm): trading beats priced manoeuvres in 10 of 10 forests for four setups and 9 of 10 for C3+W (all p ≤ 0.035). Priced manoeuvres vs fixed: none significant.

\* V+W: four missions in which drifting drones took the escape climb dominate the mean. The median per-mission difference (priced vs fixed) is +0.4 kJ.

Safety follows the order too. With priced manoeuvres the margin stays at fixed-priority levels (+0.36 to +0.43 m; 38–39 of 40 missions outside the safe distance). With trading it falls to +0.26 to +0.31 m (31–35 of 40). There were no contacts in any run.

**So the saving, and the smaller margin, come from trading who yields, not from pricing the manoeuvre.**

## Reproducibility across machines (a by-product)

The re-runs of `priority` and `ttc` here, against the published runs on the same missions: energy differs by −1.2 to +0.0 % (fixed) and −0.4 to +3.3 % (trading), none significant. Margins agree within 0.03 m. On this machine, trading vs fixed is 8–14 % (the paper: 10–14 %). The difference is C3+W, whose CI is ±5.6 % here.

## Files

- `priority_energy_ablation.patch`: the code change.
- `analyze.py`: the paired comparison. It reproduces the paper's Table III give-way rows exactly from the published runs.
- `results_same_machine.md`: all comparison tables.
- `runs/`: `report.txt`, `runs.csv`, `summary.csv` and `meta.json` of the three studies.

## To reproduce on the reference VM (for the paper's own numbers)

```bash
git apply priority_energy_ablation.patch && colcon build
ros2 run swarm_intel study --layouts 10 --seeds 4 \
  --configs cam3_sensor_swarm,all,cam3_swarm,cam2_swarm,sensor_swarm --give-way priority_energy
python3 analyze.py fixed=<priority study>/runs.csv ablation=<new study>/runs.csv \
  "trading=<main study>/runs.csv@4" --pairs=fixed:ablation,ablation:trading,fixed:trading
```
