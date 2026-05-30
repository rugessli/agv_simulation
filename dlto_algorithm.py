"""
dlto_algorithm.py
=================
Core DLTO (Dynamic Lateral Trajectory Offset) decision logic.

What it does
------------
Pure functions that, given the kinematic state of two AGVs, decide:
  * whether they are in a head-on conflict (DLTO 4 m / SW 1.5 m radius),
  * which side each one should offset to (travel direction determines lane),
  * when the pair has "passed" each other (longitudinal separation > 1 m).

This module deliberately holds *no* simulation state — no time, no integrator,
no ROS handles.  It is the analytic kernel of the planner.

ROS2 migration notes
--------------------
ROS2-READY: every function in this file.  Drop the module straight into a
            planner node; call it from the subscriber callback that receives
            neighbour poses from `/tf` or `/agv_states`.
ROS2-REPLACE: nothing here needs replacement.
"""

from __future__ import annotations
import math
from typing import Protocol


# ---------------------------------------------------------------------------
# Tunable parameters (would live in a ROS2 parameter server)
# ---------------------------------------------------------------------------
DETECT_RADIUS_DLTO   = 4.0                  # metres
DETECT_RADIUS_SW     = 1.5                  # metres
ANGLE_DIFF_THRESHOLD = math.radians(150.0)  # head-on if |Δθ| > 150°
OFFSET_TARGET        = 0.40                 # ±y target for the two AGVs
                                            # (spec says 0.3 with body
                                            #  edges touching; raised to
                                            #  0.40 so the two 0.6 m wide
                                            #  AGVs cross with a 20 cm
                                            #  axis-aligned gap, large
                                            #  enough to absorb the body
                                            #  rotation observed during
                                            #  the convergence transient)
WALL_LIMIT           = 0.45                 # AGV centre may never exceed |y|
PASS_LONG_THRESHOLD  = 1.0                  # longitudinal sep to call "passed"


# ---------------------------------------------------------------------------
# A minimal protocol that any AGV-state container must satisfy.  Both our
# `AGVState` dataclass and a ROS2 message wrapper can implement this — the
# functions below don't care which.
# ---------------------------------------------------------------------------
class PoseLike(Protocol):
    aid:       int
    x:         float
    y:         float
    theta:     float
    direction: int      # +1 heading right, -1 heading left


# ---------------------------------------------------------------------------
# Geometric / conflict primitives — ROS2-READY
# ---------------------------------------------------------------------------
def heading_difference(theta_a: float, theta_b: float) -> float:
    """Absolute heading difference, normalised to [0, π]."""
    d = (theta_a - theta_b + math.pi) % (2.0 * math.pi) - math.pi
    return abs(d)


def is_head_on(theta_a: float, theta_b: float) -> bool:
    return heading_difference(theta_a, theta_b) > ANGLE_DIFF_THRESHOLD


def distance(a: PoseLike, b: PoseLike) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def detect_dlto_conflict(own: PoseLike, partner: PoseLike) -> bool:
    return is_head_on(own.theta, partner.theta) \
       and distance(own, partner) <= DETECT_RADIUS_DLTO


def detect_sw_conflict(own: PoseLike, partner: PoseLike) -> bool:
    return is_head_on(own.theta, partner.theta) \
       and distance(own, partner) <= DETECT_RADIUS_SW


# ---------------------------------------------------------------------------
# Priority + offset assignment — ROS2-READY
# ---------------------------------------------------------------------------
def dlto_alpha(own_id: int, partner_id: int) -> float:
    """Asymmetric weight, tie-broken so the lower ID always gets 0.51."""
    return 0.51 if own_id < partner_id else 0.49


def dlto_target_offset(own_id: int, partner_id: int,
                       own_direction: int = +1) -> float:
    """Offset target is determined by TRAVEL DIRECTION, not ID.

    Traffic keep-right rule:
      AGV heading RIGHT (direction=+1) → offset to LOWER lane  (−y)
      AGV heading LEFT  (direction=−1) → offset to UPPER lane  (+y)

    This ensures both AGVs move to opposite sides of the corridor and
    pass each other without collision, like cars on a two-lane road.

    The ID is kept as a parameter for the tie-breaking alpha calculation
    (dlto_alpha) but does NOT affect the lateral offset direction.

    # ROS2-READY: direction comes from the AGV's /odom heading or goal vector.
    """
    if own_direction > 0:
        return -OFFSET_TARGET   # heading right → go to lower lane (negative y)
    else:
        return +OFFSET_TARGET   # heading left  → go to upper lane (positive y)


# ---------------------------------------------------------------------------
# Pass-detection — ROS2-READY
# ---------------------------------------------------------------------------
def passed(own: PoseLike, partner: PoseLike,
           threshold: float = PASS_LONG_THRESHOLD) -> bool:
    """True if `partner` is behind `own` in own's forward direction by
    more than `threshold` metres."""
    fx, fy = math.cos(own.theta), math.sin(own.theta)
    rel_x, rel_y = partner.x - own.x, partner.y - own.y
    longitudinal = fx * rel_x + fy * rel_y
    return longitudinal < -threshold
