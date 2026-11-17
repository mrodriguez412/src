"""Offline checks of the voting grid against the simulator's sensor function."""
import math

import numpy as np

from treasure_detector.treasure_grid import MetalDetectorModel, TreasureGrid

TREASURE = (1.3, -0.7)
TOOL_Z = 0.5


def simulate_signal(rng, x, y, z):
    """Same formula as the cabin script in treasure_search.ttt."""
    d = math.sqrt((x - TREASURE[0]) ** 2 + (y - TREASURE[1]) ** 2 + z * z)
    d = min(d, 5.0)  # the script starts its search at dmin = 5.0
    return min(1.0, math.exp(-0.5 * (d / 2.0) ** 2) + rng.random() * 0.1)


def run(path, seed=0):
    rng = np.random.default_rng(seed)
    grid = TreasureGrid(MetalDetectorModel(), -10, 10, -10, 10, 0.05)
    for (x, y) in path:
        grid.add(x, y, TOOL_Z, simulate_signal(rng, x, y, TOOL_Z))
    return grid


def straight_path(offset, n=480):
    # tool moving at 0.5 m/s, 20 Hz, passing `offset` m beside the treasure
    xs = np.linspace(-6, 6, n) + TREASURE[0]
    return [(x, TREASURE[1] + offset) for x in xs]


def test_inverse_model_brackets_truth():
    m = MetalDetectorModel()
    rng = np.random.default_rng(1)
    for d in np.linspace(0.2, 4.0, 40):
        s = min(1.0, float(m.expected(d)) + rng.random() * 0.1)
        d_min, _, d_max = m.inverse(s)
        assert d_min - 1e-9 <= d <= d_max + 1e-9


def test_curved_path_localises_precisely():
    # the arm sways sideways while the truck drives, which breaks the symmetry
    xs = np.linspace(-6, 6, 480) + TREASURE[0]
    path = [(x, TREASURE[1] + 1.0 + 0.8 * math.sin(x / 1.5)) for x in xs]
    grid = run(path)
    peak = grid.global_peak()
    est = grid.estimate(peak[1], peak[2])
    err = math.hypot(est['x'] - TREASURE[0], est['y'] - TREASURE[1])
    assert err < 0.15, err
    assert not est['ambiguous']


def test_straight_pass_is_flagged_ambiguous():
    # readings are symmetric about the path: the mirror image must be reported
    grid = run(straight_path(offset=1.5))
    peak = grid.global_peak()
    est = grid.estimate(peak[1], peak[2])
    assert abs(est['x'] - TREASURE[0]) < 0.15
    assert est['ambiguous']
    rx, ry, _ = est['rival']
    # the two hypotheses are mirror images about the path line
    assert abs((est['y'] + ry) / 2 - (TREASURE[1] + 1.5)) < 0.2


def test_no_treasure_no_detection():
    rng = np.random.default_rng(3)
    grid = TreasureGrid(MetalDetectorModel(), -10, 10, -10, 10, 0.05)
    floor = math.exp(-0.5 * 2.5 ** 2)
    for x in np.linspace(-6, 6, 480):
        grid.add(x, 0.0, TOOL_Z, floor + rng.random() * 0.1)  # background only
    assert grid.global_peak()[0] < 5.0


def test_parking_at_range_does_not_drift():
    # Reproduces the drift seen in the sim: after the pass, the truck sits
    # ~5 m away and keeps reading background. The estimate must not move.
    xs = np.linspace(-6, 6, 480) + TREASURE[0]
    path = [(x, TREASURE[1] + 1.0 + 0.8 * math.sin(x / 1.5)) for x in xs]
    path += [(TREASURE[0], TREASURE[1] + 5.0)] * 3000
    grid = run(path)
    peak = grid.global_peak()
    est = grid.estimate(peak[1], peak[2])
    err = math.hypot(est['x'] - TREASURE[0], est['y'] - TREASURE[1])
    assert err < 0.15, err
