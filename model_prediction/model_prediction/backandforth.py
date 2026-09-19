#!/usr/bin/env python3
"""Excitation signal generator for system identification.

The original version sent a +-0.5 square wave, which only visits two
amplitudes and so cannot reveal whether the plant is amplitude-linear.
This version sweeps a scripted sequence that satisfies the Step 1
requirements: many levels across [-1, 1] including zero, long holds
(steady-state gain), short bursts (fast dynamics), slow ramps and fast
pseudo-random steps (broadband excitation), plus optional noise.

Parameters (ROS):
    rate      float, publish rate in Hz (default 20.0)
    amplitude float, scales the whole signal (default 1.0)
    noise     float, std-dev of additive gaussian noise (default 0.0)
    seed      int,   RNG seed for the pseudo-random section (default 42)
    loop      bool,  restart the schedule when it ends (default True)
"""
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def build_schedule(rng):
    """Return a list of (duration, kind, a, b) segments.

    kind 'hold' holds value a; kind 'ramp' interpolates linearly a -> b.
    """
    seg = []

    def hold(v, d):
        seg.append((d, 'hold', v, v))

    def ramp(v0, v1, d):
        seg.append((d, 'ramp', v0, v1))

    # 1) Long holds at several amplitudes, always returning to zero.
    #    These pin down the steady-state gain and expose any saturation
    #    or amplitude-dependent behaviour.
    for v in (0.5, -0.5, 1.0, -1.0, 0.25, -0.25):
        hold(0.0, 1.5)
        hold(v, 3.0)
    hold(0.0, 2.0)

    # 2) Intermediate levels, shorter holds.
    for v in (0.75, -0.75, 0.35, -0.6, 0.9, -0.15):
        hold(v, 1.5)
    hold(0.0, 1.5)

    # 3) Short bursts: the system never settles, so these carry the
    #    high-frequency content that fixes the fast poles.
    for v in (1.0, -1.0, 0.6, -0.6, 0.3, -0.3):
        hold(v, 0.3)
        hold(0.0, 0.7)
    hold(0.0, 1.0)

    # 4) Slow transitions: low-frequency content, and a check that the
    #    model tracks gradual changes and not just steps.
    ramp(0.0, 1.0, 4.0)
    ramp(1.0, -1.0, 6.0)
    ramp(-1.0, 0.0, 3.0)
    hold(0.0, 1.5)

    # 5) Pseudo-random multi-level steps: broadband, the workhorse of the
    #    identification data set.
    for _ in range(40):
        seg.append((rng.uniform(0.2, 0.8), 'hold',
                    rng.uniform(-1.0, 1.0), 0.0))

    # 6) Small-amplitude random steps: same dynamics at low amplitude, so
    #    comparing a model fitted here against one fitted on section 5
    #    tells you whether the plant is really linear.
    for _ in range(25):
        seg.append((rng.uniform(0.2, 0.6), 'hold',
                    rng.uniform(-0.25, 0.25), 0.0))

    hold(0.0, 2.0)
    return seg


class BackAndForth(Node):
    def __init__(self):
        super().__init__("back_and_forth")
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("amplitude", 1.0)
        self.declare_parameter("noise", 0.0)
        self.declare_parameter("seed", 42)
        self.declare_parameter("loop", True)

        self.rate = self.get_parameter("rate").value
        self.amplitude = self.get_parameter("amplitude").value
        self.noise = self.get_parameter("noise").value
        self.loop = self.get_parameter("loop").value
        seed = self.get_parameter("seed").value

        self.rng = random.Random(seed)
        self.schedule = build_schedule(random.Random(seed))
        self.total = sum(s[0] for s in self.schedule)

        self.pub = self.create_publisher(Twist, "/vrep/twistCommand", 1)
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.done = False
        self.timer = self.create_timer(1.0 / self.rate, self.timer_cb)

        self.get_logger().info(
            "excitation schedule: %d segments, %.1f s per cycle, %.1f Hz"
            % (len(self.schedule), self.total, self.rate))

    def value_at(self, t):
        """Evaluate the schedule at elapsed time t."""
        if t >= self.total:
            if not self.loop:
                return None
            t = t % self.total
        for dur, kind, a, b in self.schedule:
            if t < dur:
                if kind == 'ramp':
                    return a + (b - a) * (t / dur)
                return a
            t -= dur
        return 0.0

    def timer_cb(self):
        t = self.get_clock().now().nanoseconds / 1e9 - self.t0
        v = self.value_at(t)

        if v is None:
            if not self.done:
                self.get_logger().info("schedule complete, holding zero")
                self.done = True
            v = 0.0

        v *= self.amplitude
        if self.noise > 0.0:
            v += self.rng.gauss(0.0, self.noise)
        v = max(-1.0, min(1.0, v))

        twist = Twist()
        twist.linear.x = v
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    driver = BackAndForth()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
