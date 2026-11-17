# treasure_detector — Objective 1

Fuses every `/vrep/metalDetector` reading into one position estimate per
treasure, and publishes the estimates as RViz markers.

## Sensor model

The cabin script in `treasure_search.ttt` computes:

```
d      = min(5.0, |Tool - closest Treasure|)      # 3D distance, capped at 5 m
signal = min(1, exp(-0.5 (d / 2.0)^2) + U(0, 0.1))
```

Because of the cap, background readings (no treasure within 5 m) are
U(0.044, 0.144), not U(0, 0.1). We checked this empirically: `treasure_eval`
logs (true distance, reading) pairs, and `fit_sensor_model` recovers
A = 0.996, sigma = 2.002 m, w = 0.100, d_cap = 5.00 m.

Inverse model: a reading s means `s - w <= f(d) <= s`. The treasure is
therefore on a ring around the tool, with radius in
`[f^-1(s), f^-1(s - w)]`.

## Fusion: Bayesian voting grid

A 5 cm grid over the dry lake stores, for each cell c, the log-likelihood
ratio "treasure at c" against "no treasure within range":

```
LLR(c) += log p(s | f(|tool - c|)) - log p(s | f_background)
```

- `p(s|f)` is the uniform noise convolved with a small Gaussian (to absorb TF
  lag), mixed with 1 % outliers. The clipped value s = 1 gets its own term.
- High readings vote **for** a ring around the tool. Low readings vote
  **against** the cells close to the tool. Rings taken from different tool
  positions intersect at the treasure.
- A treasure is declared when the best cell exceeds `llr_threshold` **and**
  a detector reading of at least `peak_signal_threshold` was seen within 3 m
  (the "high peak").
- Each estimate is the posterior mean of its mode, with a covariance (drawn as
  an ellipse). It is refined every 0.5 s as new readings arrive.

### Limitation: mirror ambiguity

If the tool moves in a straight line, the readings are identical for a
treasure on either side of the line. The detector looks for a rival local
maximum within 5 m. If the rival scores within `ambiguity_gap` nats of the
best mode, the marker turns **orange** and the label ends with `?`. Any
lateral motion of the tool removes the ambiguity: an arm sweep, a turn, or
following a curved shore.

## Run

```bash
ros2 launch vrep_vsv_driver vsv_geom_mux.launch.py      # robot + joystick
ros2 launch treasure_detector treasure_detector.launch.py   # detector + eval
# optional calibration log:  ... calib_csv:=/tmp/calib.csv
ros2 run treasure_detector fit_sensor_model /tmp/calib.csv --plot calib.png
```

In RViz (fixed frame `world`), add:

- **MarkerArray** `/treasure_detector/markers`: green = confirmed, orange = ambiguous.
- **Map** `/treasure_detector/heatmap`: votes over everything explored so far,
  max-pooled to 25 cm cells (`heatmap_resolution`). 100 = detection threshold,
  unknown = never sensed. Use the `costmap` color scheme.

`/treasure_detector/detections` (PoseArray) carries the estimates for other
nodes. `treasure_eval` compares them with the ground-truth `Treasure<i>` TF
frames that the scene publishes. These frames are used for evaluation only;
the detector never reads them.

## Tests

```bash
cd FinalProject/treasure_detector && python3 -m pytest test/
```

The tests run offline against the simulator's exact formula. They cover a
curved pass (error < 15 cm), a straight pass (flagged ambiguous), background
only (no detection), and a truck parked at 5 m after the pass (no drift).


ideas

slow down the car when the arm tip wants to move a big distance