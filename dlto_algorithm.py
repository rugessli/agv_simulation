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

from agv_model import AGV_WIDTH


# ---------------------------------------------------------------------------
# Tunable parameters (would live in a ROS2 parameter server)
# ---------------------------------------------------------------------------
DETECT_RADIUS_DLTO   = 7.0                  # metres — must be large enough for
                                            # AGVs starting at y=0 to complete
                                            # the lateral offset manoeuvre
                                            # before they are physically adjacent
DETECT_RADIUS_SW     = 1.5                  # metres
ANGLE_DIFF_THRESHOLD = math.radians(150.0)  # head-on if |Δθ| > 150°
WALL_CLEARANCE       = 0.08                 # minimum AGV-body-to-wall gap, m
OFFSET_TARGET        = 0.40                 # kept for backward compatibility;
                                            # dlto_target_offset now derives
                                            # the offset dynamically
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
    """Priority weight used in the dynamic offset formula.

    Higher priority (lower ID) → alpha = 0.10 → less lateral yield
                                          (offset = 0.90 × available_half_space)
    Lower priority (higher ID) → alpha = 0.00 → full lateral yield
                                          (offset = 1.00 × available_half_space)

    Values are chosen so that (1 − alpha) × available_half_space ≥ AGV_WIDTH / 2
    in any corridor ≥ 1.4 m, guaranteeing the two body envelopes clear each other.
    """
    return 0.10 if own_id < partner_id else 0.00


def dlto_target_offset(own_id: int, partner_id: int,
                       own_direction: int = +1,
                       corridor_width: float = 1.5,
                       wall_clearance: float = WALL_CLEARANCE) -> float:
    """Dynamic lateral target derived from corridor geometry and priority weight.

    Formula (self-derived, adapts when corridor width changes):
        available_half_space = corridor_width/2 − AGV_WIDTH/2 − wall_clearance
        offset               = available_half_space × (1 − alpha_i)
        target_y             = −offset  (heading right)
                             = +offset  (heading left)

    Higher-priority AGV (lower ID, alpha=0.10) offsets to 90 % of available
    half-space — keeps a slight centre-lane advantage.
    Lower-priority AGV (higher ID, alpha=0.00) offsets to the full available
    half-space — yields maximally while still respecting the wall clearance.

    # ROS2-READY: corridor_width and wall_clearance come from /map metadata.
    """
    alpha                = dlto_alpha(own_id, partner_id)
    available_half_space = corridor_width / 2 - AGV_WIDTH / 2 - wall_clearance
    offset               = available_half_space * (1 - alpha)
    sign = -1 if own_direction > 0 else +1
    return sign * offset


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
