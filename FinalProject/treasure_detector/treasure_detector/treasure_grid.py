"""Metal detector sensor model and Bayesian voting grid (no ROS dependency).

Sensor model (from the treasure_search.ttt cabin script):

    d      = min(d_cap, 3D distance between the Tool origin and the closest treasure)
    signal = min(1, A * exp(-0.5 * (d / sigma)^2) + U(0, w))

with A = 1, sigma = 2.0 m, w = 0.1, d_cap = 5.0 m. The script starts its
minimum search at dmin = 5.0, so far from any treasure the reading is
U(0.044, 0.144), not U(0, 0.1). Calibration data from treasure_eval confirms
this.

Each measurement (tool position p, signal s) votes on a 2D grid of candidate
treasure positions c lying on the ground plane z = treasure_z. Each cell
accumulates the log-likelihood ratio

    LLR(c) += log p(s | f(|p - c|)) - log p(s | f = 0)

i.e. "treasure at c" against "no treasure within d_cap". Strong readings add
positive votes on a ring around the tool (the inverse model d = f^-1(s)).
Weak readings add negative votes close to the tool. Rings from several tool
positions intersect at the treasure, which fuses all measurements into one
estimate.
"""
import math

import numpy as np
from scipy.ndimage import maximum_filter
from scipy.special import ndtr

_INV_SQRT_2PI = 1.0 / math.sqrt(2.0 * math.pi)


class MetalDetectorModel:
    """Forward, inverse and likelihood model of the simulated metal detector."""

    def __init__(self, sigma=2.0, amplitude=1.0, noise_width=0.1, d_cap=5.0,
                 blur=0.02, outlier=0.01, saturation=1.0):
        self.sigma = sigma            # spatial spread of the response [m]
        self.amplitude = amplitude    # response at d = 0 (before noise)
        self.noise_width = noise_width  # additive noise is U(0, noise_width)
        self.d_cap = d_cap            # distances beyond this read as d_cap
        self.blur = blur              # extra Gaussian noise for model mismatch
        self.outlier = outlier        # probability that a reading is garbage
        self.saturation = saturation  # readings are clipped at this value
        self.f_floor = float(self.expected(d_cap))  # response with no treasure

    def expected(self, d):
        """Noise-free response f(d)."""
        d = np.minimum(np.asarray(d), self.d_cap)
        return self.amplitude * np.exp(-0.5 * (d / self.sigma) ** 2)

    def inverse(self, s):
        """Inverse model: return (d_min, d_mid, d_max) consistent with reading s.

        s - noise_width <= f(d) <= s, so the distance lies in an interval.
        d_max is inf when the reading can be explained by noise alone.
        """
        def f_inv(f):
            if f <= self.f_floor:
                return math.inf  # explained by background noise alone
            if f >= self.amplitude:
                return 0.0
            return self.sigma * math.sqrt(-2.0 * math.log(f / self.amplitude))

        f_hi = min(s, self.amplitude)
        f_lo = s - self.noise_width
        f_mid = s - 0.5 * self.noise_width
        if s >= self.saturation - 1e-6:
            f_hi = self.amplitude
        return f_inv(f_hi), f_inv(f_mid), f_inv(f_lo)

    def max_range(self):
        """Distance beyond which a treasure has no effect on the reading."""
        return self.d_cap

    def likelihood(self, s, f):
        """p(s | f) for a reading s, vectorised over expected values f."""
        f = np.asarray(f, dtype=np.float64)
        w, b = self.noise_width, self.blur
        if s >= self.saturation - 1e-4:
            # P(f + u + n >= sat), u ~ U(0, w), n ~ N(0, b^2).
            # integral of Phi has antiderivative G(x) = x Phi(x) + phi(x)
            def G(x):
                return x * ndtr(x) + _INV_SQRT_2PI * np.exp(-0.5 * x * x)
            p = (b / w) * (G((f + w - self.saturation) / b)
                           - G((f - self.saturation) / b))
        else:
            # density of f + u + n at s
            p = (ndtr((s - f) / b) - ndtr((s - f - w) / b)) / w
        return (1.0 - self.outlier) * p + self.outlier

    def log_likelihood_ratio(self, s, f):
        null = float(self.likelihood(s, self.f_floor))
        return np.log(self.likelihood(s, f)) - math.log(null)


class TreasureGrid:
    """Accumulates LLR votes from metal detector readings on a ground grid."""

    def __init__(self, model, x_min=-60.0, x_max=60.0, y_min=-60.0, y_max=60.0,
                 resolution=0.05, treasure_z=0.0, update_radius=None,
                 keep_threshold=0.2):
        self.model = model
        self.res = resolution
        self.x_min, self.y_min = x_min, y_min
        self.nx = int(math.ceil((x_max - x_min) / resolution))
        self.ny = int(math.ceil((y_max - y_min) / resolution))
        self.xs = x_min + (np.arange(self.nx) + 0.5) * resolution
        self.ys = y_min + (np.arange(self.ny) + 0.5) * resolution
        self.llr = np.zeros((self.ny, self.nx), dtype=np.float32)
        self.treasure_z = treasure_z
        self.update_radius = update_radius or model.max_range()
        self.keep_threshold = keep_threshold
        # Readings strong enough to matter. Used to check for a high peak.
        self.strong = []  # list of (x, y, z, s)
        self.n_updates = 0
        # bounding box (in cells) of everything touched so far
        self.bbox = None

    # ------------------------------------------------------------------ utils
    def _window(self, x, y, r):
        """Cell index window [i0, i1) x [j0, j1) covering a disk."""
        j0 = max(0, int((x - r - self.x_min) / self.res))
        j1 = min(self.nx, int((x + r - self.x_min) / self.res) + 1)
        i0 = max(0, int((y - r - self.y_min) / self.res))
        i1 = min(self.ny, int((y + r - self.y_min) / self.res) + 1)
        return i0, i1, j0, j1

    def cell_center(self, i, j):
        return self.xs[j], self.ys[i]

    # ----------------------------------------------------------------- update
    def add(self, x, y, z, s, weight=1.0):
        """Fuse one reading s taken with the tool at (x, y, z)."""
        h = z - self.treasure_z
        r2 = self.update_radius ** 2 - h * h
        if r2 <= 0.0:
            return False  # tool too high: reading carries no information
        i0, i1, j0, j1 = self._window(x, y, math.sqrt(r2))
        if i0 >= i1 or j0 >= j1:
            return False  # outside the map
        dx = self.xs[j0:j1] - x
        dy = self.ys[i0:i1] - y
        d = np.sqrt(dx[None, :] ** 2 + dy[:, None] ** 2 + h * h)
        llr = self.model.log_likelihood_ratio(s, self.model.expected(d))
        self.llr[i0:i1, j0:j1] += (weight * llr).astype(np.float32)
        self.n_updates += 1
        if s >= self.keep_threshold:
            self.strong.append((x, y, z, s))
        if self.bbox is None:
            self.bbox = [i0, i1, j0, j1]
        else:
            b = self.bbox
            self.bbox = [min(b[0], i0), max(b[1], i1), min(b[2], j0), max(b[3], j1)]
        return True

    # ------------------------------------------------------------------ query
    def max_signal_near(self, x, y, radius):
        best = 0.0
        r2 = radius * radius
        for (mx, my, _, s) in self.strong:
            if (mx - x) ** 2 + (my - y) ** 2 <= r2 and s > best:
                best = s
        return best

    def global_peak(self, exclude=()):
        """Best cell over the touched area, ignoring disks (x, y, r) in exclude."""
        if self.bbox is None:
            return None
        i0, i1, j0, j1 = self.bbox
        sub = self.llr[i0:i1, j0:j1]
        if exclude:
            sub = sub.copy()
            for (ex, ey, er) in exclude:
                a0, a1, b0, b1 = self._window(ex, ey, er)
                a0, a1 = max(a0, i0) - i0, min(a1, i1) - i0
                b0, b1 = max(b0, j0) - j0, min(b1, j1) - j0
                if a0 < a1 and b0 < b1:
                    dx = self.xs[j0 + b0:j0 + b1] - ex
                    dy = self.ys[i0 + a0:i0 + a1] - ey
                    mask = (dx[None, :] ** 2 + dy[:, None] ** 2) <= er * er
                    sub[a0:a1, b0:b1][mask] = -np.inf
        k = int(np.argmax(sub))
        i, j = np.unravel_index(k, sub.shape)
        v = float(sub[i, j])
        if not np.isfinite(v):
            return None
        x, y = self.cell_center(i0 + i, j0 + j)
        return v, x, y

    def estimate(self, x0, y0, search_radius=5.0, mode_radius=0.75,
                 ambiguity_gap=3.0):
        """Refine a treasure estimate around (x0, y0).

        Returns a dict with the posterior mean and covariance of the main mode,
        its peak LLR, and an ambiguity flag. The flag is set when another local
        maximum within search_radius is within ambiguity_gap nats of the best
        one, e.g. the mirror image produced by a straight-line pass. A mirror
        image lies 2 x (lateral offset of the path) away, so search_radius must
        be about twice the largest useful sensing offset.
        """
        i0, i1, j0, j1 = self._window(x0, y0, search_radius)
        sub = self.llr[i0:i1, j0:j1].astype(np.float64)
        xs, ys = self.xs[j0:j1], self.ys[i0:i1]
        k = int(np.argmax(sub))
        bi, bj = np.unravel_index(k, sub.shape)
        best = sub[bi, bj]
        px, py = xs[bj], ys[bi]

        # posterior weights of the main mode only
        dx = xs[None, :] - px
        dy = ys[:, None] - py
        in_mode = (dx * dx + dy * dy) <= mode_radius ** 2
        w = np.where(in_mode, np.exp(sub - best), 0.0)
        wsum = w.sum()
        X = np.broadcast_to(xs[None, :], sub.shape)
        Y = np.broadcast_to(ys[:, None], sub.shape)
        mx = float((w * X).sum() / wsum)
        my = float((w * Y).sum() / wsum)
        cxx = float((w * (X - mx) ** 2).sum() / wsum)
        cyy = float((w * (Y - my) ** 2).sum() / wsum)
        cxy = float((w * (X - mx) * (Y - my)).sum() / wsum)
        floor = self.res ** 2 / 12.0  # quantisation limit of the grid
        cov = np.array([[cxx + floor, cxy], [cxy, cyy + floor]])

        # competing modes: local maxima outside the main mode
        local_max = (sub == maximum_filter(sub, size=5, mode='nearest'))
        rivals = local_max & ~in_mode & (sub > best - ambiguity_gap)
        rival = None
        if rivals.any():
            ri, rj = np.unravel_index(int(np.argmax(np.where(rivals, sub, -np.inf))),
                                      sub.shape)
            rival = (float(xs[rj]), float(ys[ri]), float(sub[ri, rj]))

        return {
            'x': mx, 'y': my, 'cov': cov, 'llr': float(best),
            'ambiguous': rival is not None, 'rival': rival,
        }

    def window_values(self, x, y, half_size):
        """Return (llr sub-array, x of first column, y of first row)."""
        i0, i1, j0, j1 = self._window(x, y, half_size)
        return (self.llr[i0:i1, j0:j1],
                self.x_min + j0 * self.res, self.y_min + i0 * self.res)
