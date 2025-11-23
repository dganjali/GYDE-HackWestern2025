"""
Pure Pursuit controller for following a single moving target.

This module implements a small, self-contained Pure Pursuit controller that
accepts a target bearing (degrees) and distance (meters) measured in the
robot's coordinate frame (bearing: 0 = straight ahead, positive -> right), and
produces steering commands suitable for mixing into a differential-drive
motor command scheme (left = v + turn, right = v - turn).

The implementation is intentionally lightweight and does not assume any
particular motor/PWM scaling. It returns a normalized linear velocity scale
(-1..1) and a turn effort (-1..1). You can multiply those by your robot's
maximum motor PWM to produce actual motor commands.

API:
    pp = PurePursuit(lookahead=0.5, wheelbase=0.13)
    v_scale, turn_scale = pp.update(bearing_deg, distance_m, desired_speed_scale=0.6)

Notes on conventions (matches the existing codebase in this repo):
- bearing_deg: degrees, 0 forward, positive to the RIGHT of the robot
- distance_m: meters, range >= 0
- desired_speed_scale: -1..1 fraction of maximum forward speed

The controller computes the curvature kappa needed to drive to the target
point and converts that to a turn scale value in [-1,1]. The mapping uses a
simple gain and clamping; tune `turn_gain` and `max_turn` for your robot.

The file also includes a small CLI demo when run directly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class PurePursuit:
    lookahead: float = 0.5          # meters; nominal lookahead distance (used as smoothing)
    wheelbase: float = 0.13         # meters; distance between wheels (affects curvature->turn mapping)
    turn_gain: float = 1.0          # unitless gain mapping curvature -> turn effort
    max_turn: float = 1.0           # clamp turn effort to [-max_turn, max_turn]
    min_lookahead: float = 0.05     # avoid division by zero when target is very close

    def update(self, bearing_deg: float, distance_m: float, desired_speed_scale: float = 0.5):
        """Compute linear and turn scales for a differential-drive robot.

        Inputs:
            bearing_deg: target bearing in degrees (0 forward, + right)
            distance_m: target distance in meters
            desired_speed_scale: requested forward speed as fraction of max ([-1,1])

        Returns:
            (v_scale, turn_scale, info)
              v_scale: float in [-1,1] -- forward command scale
              turn_scale: float in [-1,1] -- turn command scale; positive -> turn RIGHT
              info: dict with internal values (curvature, lookahead used, x,y coords)
        """
        # Normalize inputs
        theta = math.radians(bearing_deg)
        d = max(0.0, float(distance_m))

        # If target is extremely close, return zero turn and small forward to avoid divide by zero
        if d < 1e-6:
            return 0.0, 0.0, {"reason": "no_target"}

        # Use the measured distance as the lookahead length if it's reasonable,
        # otherwise use configured lookahead. This gives natural smoothing.
        L = max(self.min_lookahead, min(self.lookahead, d))

        # Coordinates of the target in robot frame assuming 0 deg forward, +theta to the right
        # x forward, y right
        x = d * math.cos(theta)
        y = d * math.sin(theta)

        # Pure Pursuit curvature kappa = 2*y / (L^2)
        # (using the target point as the lookahead point)
        # If L is very small, clamp to avoid huge curvature
        L_eff = max(self.min_lookahead, math.hypot(x, y))
        kappa = 0.0
        try:
            kappa = (2.0 * y) / (L_eff * L_eff)
        except Exception:
            kappa = 0.0

        # Map curvature to a normalized turn effort.
        # For differential-drive, approximate turn rate ~ kappa * v; we create a turn_scale
        # that is independent of v_scale by scaling with turn_gain.
        turn = self.turn_gain * kappa

        # Clamp turn to reasonable range
        turn = max(-self.max_turn, min(self.max_turn, turn))

        # v_scale logic: we can reduce forward speed when curvature is large to encourage
        # cleaner turns. Use a simple heuristic: scale forward by 1/(1+abs(turn)*factor)
        # so tight turns slow the forward command.
        turn_slow_factor = 2.0
        v_scale = float(desired_speed_scale)
        v_scale = max(-1.0, min(1.0, v_scale))
        v_scale = v_scale / (1.0 + abs(turn) * turn_slow_factor)

        info = {
            "x": x,
            "y": y,
            "L_eff": L_eff,
            "kappa": kappa,
            "turn_raw": turn,
            "v_scale": v_scale,
        }

        return v_scale, turn, info


# Utility: convert v_scale/turn_scale -> left/right mixing (PWM-like fractions)
def mix_to_wheels(v_scale: float, turn_scale: float, max_pwm: float = 255.0):
    """Convert normalized v and turn into left/right PWM values.

    left = v + turn
    right = v - turn

    Inputs in [-1,1]; outputs scaled to integers in [-max_pwm,max_pwm].
    """
    v = max(-1.0, min(1.0, v_scale))
    t = max(-1.0, min(1.0, turn_scale))

    left = v + t
    right = v - t

    # Normalize if either exceeds 1.0 in magnitude
    m = max(1.0, abs(left), abs(right))
    left /= m
    right /= m

    left_pwm = int(round(left * max_pwm))
    right_pwm = int(round(right * max_pwm))
    return left_pwm, right_pwm


# Small CLI demo
if __name__ == "__main__":
    import time
    pp = PurePursuit(lookahead=0.6, wheelbase=0.13, turn_gain=1.2, max_turn=1.5)

    # Simple interactive demo: enter bearing and distance
    print("Pure Pursuit demo. Enter 'bearing distance [speed]' (deg m [scale]) or Ctrl-C to quit")
    try:
        while True:
            s = input("> ")
            if not s:
                continue
            parts = s.split()
            try:
                b = float(parts[0])
                d = float(parts[1]) if len(parts) > 1 else 0.5
                sp = float(parts[2]) if len(parts) > 2 else 0.6
            except Exception:
                print("invalid input")
                continue
            v, t, info = pp.update(b, d, sp)
            left_pwm, right_pwm = mix_to_wheels(v, t, max_pwm=200)
            print(f"bearing={b:.1f}°, dist={d:.2f}m -> v={v:.2f}, turn={t:.2f}, L={info['L_eff']:.2f}, k={info['kappa']:.3f}")
            print(f"  -> left_pwm={left_pwm}, right_pwm={right_pwm}")
    except KeyboardInterrupt:
        print("demo exit")
