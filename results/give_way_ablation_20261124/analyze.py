"""Paired comparison of give-way rules on the same missions.

usage: python3 -I analyze.py NAME=runs.csv [NAME=runs.csv ...] --pairs A:B[,A:B...]
Each pair A:B reports B relative to A, paired by (config, layout, seed).
"""
import csv
import math
import sys
import random
from collections import defaultdict

LABEL = {'cam3_sensor_swarm': 'C3+V+W', 'all': 'C2+V+W', 'cam3_swarm': 'C3+W',
         'cam2_swarm': 'C2+W', 'sensor_swarm': 'V+W', 'swarm': 'W', 'cam3_sensor': 'C3+V'}
ORDER = ['cam3_sensor_swarm', 'all', 'cam3_swarm', 'cam2_swarm', 'sensor_swarm']


def load(path, seeds=None):
    out = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            if seeds is not None and int(r['seed']) not in seeds:
                continue
            out[(r['config'], int(r['layout']), int(r['seed']))] = r
    return out


def f(r, k):
    v = r.get(k)
    return float(v) if v not in (None, '') else 0.0


def t975(n):
    table = {39: 2.023, 38: 2.024, 35: 2.030, 30: 2.042}
    return table.get(n - 1, 2.02)


def signflip(d, n_perm=20000, rng=random.Random(7)):
    obs = abs(sum(d))
    if obs == 0:
        return 1.0
    hits = 0
    for _ in range(n_perm):
        s = sum(x if rng.random() < 0.5 else -x for x in d)
        if abs(s) >= obs - 1e-12:
            hits += 1
    return (hits + 1) / (n_perm + 1)


def holm(ps):
    idx = sorted(range(len(ps)), key=lambda i: ps[i])
    adj, run = [0.0] * len(ps), 0.0
    for rank, i in enumerate(idx):
        run = max(run, min(1.0, (len(ps) - rank) * ps[i]))
        adj[i] = run
    return adj


def compare(A, B, nameA, nameB):
    rows, ps = [], []
    for cfg in ORDER:
        keys = sorted(k for k in A if k[0] == cfg and k in B)
        if not keys:
            continue
        ea = [f(A[k], 'fleet_energy_kj') for k in keys]
        eb = [f(B[k], 'fleet_energy_kj') for k in keys]
        d = [b - a for a, b in zip(ea, eb)]
        n = len(d)
        ma = sum(ea) / n
        md = sum(d) / n
        sd = math.sqrt(sum((x - md) ** 2 for x in d) / (n - 1))
        pct, ci = 100 * md / ma, 100 * t975(n) * sd / math.sqrt(n) / ma
        p = signflip(d)
        ps.append(p)
        mar_a = [f(A[k], 'fleet_min_sep_margin_m') for k in keys]
        mar_b = [f(B[k], 'fleet_min_sep_margin_m') for k in keys]
        gw_a = sum(f(A[k], 'fleet_yield_time_s') for k in keys) / n
        gw_b = sum(f(B[k], 'fleet_yield_time_s') for k in keys) / n
        rows.append(dict(cfg=LABEL[cfg], n=n, ea=ma, eb=sum(eb) / n, pct=pct, ci=ci, p=p,
                         ma=sum(mar_a) / n, mb=sum(mar_b) / n,
                         sa=sum(m > 0 for m in mar_a), sb=sum(m > 0 for m in mar_b),
                         ca=sum(f(A[k], 'fleet_drone_contacts') for k in keys),
                         cb=sum(f(B[k], 'fleet_drone_contacts') for k in keys),
                         gwa=gw_a, gwb=gw_b))
    for r, ph in zip(rows, holm(ps)):
        r['ph'] = ph
    print(f'\n### {nameB} vs {nameA} (paired by mission; energy change = {nameB} - {nameA}, % of {nameA})')
    print(f'| config | n | E {nameA} kJ | E {nameB} kJ | change | 95% CI | p (Holm) | margin {nameA}/{nameB} m '
          f'| safe {nameA}/{nameB} | contacts | give-way s {nameA}/{nameB} |')
    print('|---|---|---|---|---|---|---|---|---|---|---|')
    for r in rows:
        p = '<0.001' if r['ph'] < 0.001 else f"{r['ph']:.3f}"
        print(f"| {r['cfg']} | {r['n']} | {r['ea']:.2f} | {r['eb']:.2f} | {r['pct']:+.1f} % | ±{r['ci']:.1f} % | {p} "
              f"| {r['ma']:+.2f}/{r['mb']:+.2f} | {r['sa']}/{r['sb']} | {r['ca']:.0f}/{r['cb']:.0f} "
              f"| {r['gwa']:.1f}/{r['gwb']:.1f} |")
    return rows


if __name__ == '__main__':
    data, pairs = {}, []
    args = sys.argv[1:]
    for a in args:
        if a.startswith('--pairs='):
            pairs = [tuple(p.split(':')) for p in a.split('=', 1)[1].split(',')]
        elif '=' in a:
            name, rest = a.split('=', 1)
            seeds = None
            if '@' in rest:
                rest, s = rest.split('@')
                seeds = set(range(int(s)))
            data[name] = load(rest, seeds)
    for a, b in pairs:
        compare(data[a], data[b], a, b)
