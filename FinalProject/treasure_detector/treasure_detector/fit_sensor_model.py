#!/usr/bin/env python3
"""Fit the parametric sensor model  s = min(1, A exp(-d^2 / (2 sigma^2)) + U(0, w)).

Usage: ros2 run treasure_detector fit_sensor_model calib.csv [--plot out.png]

The CSV comes from treasure_eval with csv_file:=... It prints the
model.* parameters to pass to the detector.
"""
import argparse
import csv
import math

import numpy as np
from scipy.optimize import least_squares


def fit(d, s, far_distance=7.0):
    # Use readings far enough from saturation to be unclipped.
    ok = s < 0.98
    d, s = d[ok], s[ok]

    # With no treasure in range the reading is U(floor, floor + w): the
    # simulator caps the distance, so the floor is not zero.
    far = d > far_distance
    if far.sum() > 50:
        lo, hi = np.percentile(s[far], [0.25, 99.75])
        floor, w = float(lo), float(hi - lo)
    else:
        floor, w = 0.0, 0.1

    # mean reading: E[s] = max(f(d), floor) + w / 2, fitted on informative readings
    near = d < far_distance

    def residual(theta):
        A, sigma = theta
        f = np.maximum(A * np.exp(-0.5 * (d[near] / sigma) ** 2), floor)
        return f + 0.5 * w - s[near]

    res = least_squares(residual, x0=[1.0, 1.5], bounds=([0.1, 0.1], [2.0, 10.0]))
    A, sigma = res.x
    r = residual(res.x)
    d_cap = sigma * math.sqrt(-2.0 * math.log(floor / A)) if 0 < floor < A else math.inf
    return A, sigma, w, floor, d_cap, float(np.std(r)), int(near.sum())


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('csv_file')
    ap.add_argument('--plot', default='')
    args = ap.parse_args()

    with open(args.csv_file) as f:
        rows = list(csv.DictReader(f))
    d = np.array([float(r['true_distance']) for r in rows])
    s = np.array([float(r['signal']) for r in rows])
    A, sigma, w, floor, d_cap, std, n = fit(d, s)
    print('Fitted on %d informative readings (d in [%.2f, %.2f] m)' % (n, d.min(), d.max()))
    print('  model.amplitude   = %.3f' % A)
    print('  model.sigma       = %.3f m' % sigma)
    print('  model.noise_width = %.3f   (background reads U(%.3f, %.3f))'
          % (w, floor, floor + w))
    print('  model.d_cap       = %.2f m  (distance at which f(d) = background floor)' % d_cap)
    print('  residual std %.3f (uniform noise alone gives %.3f)' % (std, w / math.sqrt(12)))

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        dd = np.linspace(0, max(8.0, d.max()), 300)
        f = np.maximum(A * np.exp(-0.5 * (dd / sigma) ** 2), floor)
        fig, ax = plt.subplots(figsize=(7, 4))
        ax.scatter(d, s, s=3, alpha=0.3, label='readings')
        ax.plot(dd, np.minimum(1, f + w / 2), 'r', label='fitted mean')
        ax.fill_between(dd, np.minimum(1, f), np.minimum(1, f + w), color='r', alpha=0.2,
                        label='noise band')
        ax.set_xlabel('true distance tool -> treasure [m]')
        ax.set_ylabel('/vrep/metalDetector')
        ax.legend()
        fig.tight_layout()
        fig.savefig(args.plot, dpi=120)
        print('Plot saved to ' + args.plot)


if __name__ == '__main__':
    main()
