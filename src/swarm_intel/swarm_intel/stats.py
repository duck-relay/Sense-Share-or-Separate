"""Paired statistics for the study (numpy only, no scipy needed).

Every configuration is compared with the baseline on the SAME missions: the same forest
layout and the same seed (seed = wind direction, gusts and sensor-noise stream), so the
comparison is paired. Estimate, interval and test all concern the MEAN paired difference:

* paired sign-flip permutation test of the mean difference, two-sided (exact for up to
  16 pairs, otherwise 40 000 random sign flips with a fixed seed, so it is reproducible)
* Holm-Bonferroni correction across the configurations, separately for each metric
  (family-wise error 5 % per metric)
* 95 % confidence interval of the mean paired difference (Student t)
* effect size d_z = mean difference / SD of the differences
(an exact Wilcoxon signed-rank test is also provided, for reference)
"""
import math

import numpy as np

ALPHA = 0.05
N_PERM = 40000
EXACT_MAX = 16


def signflip_p(d, n_perm=N_PERM, seed=12345):
    """Two-sided p-value of the paired sign-flip permutation test of the mean of d."""
    d = np.asarray(d, float)
    n = len(d)
    if n == 0:
        return 1.0
    obs = abs(float(d.sum()))
    tol = 1e-9 * max(1.0, float(np.abs(d).sum()))
    if obs <= tol:
        return 1.0
    if n <= EXACT_MAX:
        signs = ((np.arange(2 ** n)[:, None] >> np.arange(n)) & 1) * 2 - 1
        sums = np.abs(signs @ d)
        return float(np.mean(sums >= obs - tol))
    rng = np.random.default_rng(seed)
    hits = 0
    left = n_perm
    while left > 0:
        b = min(left, 5000)
        signs = rng.integers(0, 2, size=(b, n)) * 2 - 1
        hits += int(np.sum(np.abs(signs @ d) >= obs - tol))
        left -= b
    return (hits + 1.0) / (n_perm + 1.0)


def min_p(n):
    """Smallest p-value the sign-flip test can give with n pairs."""
    return 2.0 ** (1 - n) if n <= EXACT_MAX else 1.0 / (N_PERM + 1.0)


def testable(n, m, alpha=ALPHA):
    """Can a comparison with n pairs be significant after Holm over m comparisons?"""
    return n >= 2 and min_p(n) * m < alpha


def wilcoxon_p(d, tol=0.0):
    """Two-sided exact p-value of the Wilcoxon signed-rank test for paired differences d."""
    d = np.asarray(d, float)
    d = d[np.abs(d) > tol]
    n = len(d)
    if n == 0:
        return 1.0
    a = np.abs(d)
    order = np.argsort(a, kind='mergesort')
    ranks = np.empty(n)
    i = 0
    while i < n:                                   # average ranks for ties
        j = i
        while j + 1 < n and a[order[j + 1]] == a[order[i]]:
            j += 1
        ranks[order[i:j + 1]] = 0.5 * (i + j) + 1.0
        i = j + 1
    r2 = np.rint(2 * ranks).astype(int)            # doubled ranks are integers
    w = int(r2[d > 0].sum())
    total = int(r2.sum())
    if n > 400:                                    # normal approximation (with tie correction)
        mu = total / 2.0
        var = float(np.sum(r2.astype(float) ** 2)) / 4.0
        z = (abs(w - mu) - 0.5) / math.sqrt(var) if var > 0 else 0.0
        return float(min(1.0, math.erfc(max(z, 0.0) / math.sqrt(2.0))))
    dist = np.zeros(total + 1)
    dist[0] = 1.0
    for r in r2:                                   # each rank enters with + or - sign, p = 1/2
        nxt = 0.5 * dist
        nxt[r:] += 0.5 * dist[:total + 1 - r]
        dist = nxt
    lo = float(dist[:w + 1].sum())
    hi = float(dist[w:].sum())
    return float(min(1.0, 2.0 * min(lo, hi)))


def holm(pvals):
    """Holm-Bonferroni adjusted p-values (same order as given)."""
    p = np.asarray(pvals, float)
    m = len(p)
    out = np.empty(m)
    run = 0.0
    for k, i in enumerate(np.argsort(p, kind='mergesort')):
        run = max(run, min(1.0, (m - k) * p[i]))
        out[i] = run
    return out


def t975(df):
    """0.975 quantile of Student's t (Cornish-Fisher expansion; exact enough for df >= 3)."""
    if df <= 0:
        return float('nan')
    table = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571}
    if df in table:
        return table[df]
    z = 1.959963985
    g1 = (z ** 3 + z) / 4
    g2 = (5 * z ** 5 + 16 * z ** 3 + 3 * z) / 96
    g3 = (3 * z ** 7 + 19 * z ** 5 + 17 * z ** 3 - 15 * z) / 384
    g4 = (79 * z ** 9 + 776 * z ** 7 + 1482 * z ** 5 - 1920 * z ** 3 - 945 * z) / 92160
    return z + g1 / df + g2 / df ** 2 + g3 / df ** 3 + g4 / df ** 4


def paired(x, y):
    """x (configuration) vs y (baseline), paired arrays -> dict of statistics of d = x - y."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    d = x - y
    n = len(d)
    out = {'n': n, 'diff': float(d.mean()) if n else None, 'ci': None, 'p': 1.0, 'dz': None}
    if n == 0:
        return out
    scale = max(1.0, float(np.max(np.abs(np.concatenate([x, y])))))
    if n > 1:
        sd = float(d.std(ddof=1))
        out['ci'] = t975(n - 1) * sd / math.sqrt(n)
        out['dz'] = (out['diff'] / sd) if sd > 1e-12 else (0.0 if abs(out['diff']) < 1e-12 else math.copysign(math.inf, out['diff']))
    out['p'] = signflip_p(d)
    out['p_wilcoxon'] = wilcoxon_p(d, tol=1e-9 * scale)
    return out
