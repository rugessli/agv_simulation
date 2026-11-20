"""ORCA for non-holonomic AGVs in a corridor.

Standard ORCA computes collision-free 2D velocities. Applied to a unicycle
model in a narrow corridor, large heading deviations (>45 deg) cause the rotated
AGV body to block itself. This implementation therefore keeps only ORCA's
responsibility-allocation principle: the fixed reciprocal coefficient 0.5
(each vehicle takes half of the avoidance) is mapped to a lateral offset in the
same offset framework, controller and safety layer as DLTO. No velocity-obstacle
half-planes are computed.

`max_offset_step` is an additional reference baseline (not ORCA) in which both
vehicles take the maximum admissible offset (coefficient 1.0).

Reference: van den Berg et al. (2011).
"""
from __future__ import annotations
import math

from config import WAIT_OFFSET_TOL, hold_distance
from config import (
    AGV_WIDTH, AGV_LENGTH, CRUISE_SPEED, DETECT_RANGE, DLTO_PLAN_RANGE,
    WALL_CLEARANCE, D_MIN, D_CLEAR,
)

_AGENT_RADIUS = math.hypot(AGV_LENGTH / 2.0, AGV_WIDTH / 2.0)
ANGLE_DIFF_THRESHOLD = math.radians(150.0)


def _is_head_on(theta_a: float, theta_b: float) -> bool:
    d = abs((theta_a - theta_b + math.pi) % (2 * math.pi) - math.pi)
    return d > ANGLE_DIFF_THRESHOLD


def _lateral_target(own, partner, corridor_width: float,
                    responsibility: float) -> float:
    """Compute lateral y-target using ORCA responsibility coefficient.

    Magnitude is bounded by offset_bounds() so the body keeps TILT_RESERVE of
    wall clearance during the transient arc (Problem 2).
    """
    from dlto_algorithm import offset_bounds
    min_off, max_off, available = offset_bounds(corridor_width)
    delta = min_off + responsibility * max(0.0, max_off - min_off)
    delta = max(min_off, min(delta, available))

    sign = -1 if own.direction > 0 else +1
    return sign * delta


def _passed(own, partner, threshold: float = 1.0) -> bool:
    fx, fy = math.cos(own.theta), math.sin(own.theta)
    return fx * (partner.x - own.x) + fy * (partner.y - own.y) < -threshold


def _rear_clear_for_offset(agv, agvs: list) -> bool:
    """Return True if same-direction followers are far enough for a lateral offset (Fix 3)."""
    min_gap = 2.0 * AGV_LENGTH
    for b in agvs:
        if b is agv or b.arrived or b.direction != agv.direction:
            continue
        x_rel = (b.x - agv.x) * agv.direction
        if x_rel < 0 and abs(x_rel) < min_gap:
            return False
    return True


def orca_step(agvs: list, corridor_half_w: float,
              obstacles: list = None, responsibility: float = 0.5) -> None:
    corridor_width = 2.0 * corridor_half_w
    live = [a for a in agvs if not a.arrived]

    from dlto_algorithm import (obstacle_plan, obstacle_traverse_state,
                                slower_state)

    # ORCA has no priority model — break the bottleneck tie by id (lower id
    # goes first).
    prio = lambda x: -x.aid

    for a in live:
        partner = None
        best_d = math.inf
        for b in live:
            if b is a:
                continue
            d = math.hypot(a.x - b.x, a.y - b.y)
            # Use direction attribute (not heading angle) for robust detection
            # during tilted lateral offset execution.
            if d <= DLTO_PLAN_RANGE and b.direction != a.direction \
                    and not _passed(a, b):
                if d < best_d:
                    partner, best_d = b, d

        op = obstacle_plan(a, live, obstacles, corridor_width, prio) \
            if obstacles else None

        if op is not None:
            tgt, wait, dist_obs = op
            a.target_y = tgt
            if wait:
                a.state = "WAIT"
            else:
                if best_d <= D_CLEAR:
                    partner_state = "CREEP"
                elif best_d <= DETECT_RANGE:
                    partner_state = "SLOW"
                else:
                    partner_state = "APPROACH"
                a.state = slower_state(partner_state,
                                       obstacle_traverse_state(a, dist_obs))

        elif partner is not None:
            a.target_y = _lateral_target(a, partner, corridor_width,
                                         responsibility=responsibility)

            # Fix 1: four-speed state ladder
            if best_d <= D_CLEAR:
                a.state = "CREEP"
            elif best_d <= DETECT_RANGE:
                a.state = "SLOW"
            else:
                a.state = "APPROACH"

            # Fix 3: wait-for-offset rule
            own_err = abs(a.target_y - a.y)
            partner_err = abs(partner.target_y - partner.y)
            if best_d <= hold_distance(partner_err):
                if partner_err > WAIT_OFFSET_TOL and own_err < partner_err - 0.05:
                    a.state = "WAIT"

        else:
            a.state = "RUN"
            a.target_y = 0.0


def max_offset_step(agvs: list, corridor_half_w: float,
                    obstacles: list = None) -> None:
    """Reference baseline: both vehicles take the maximum admissible offset."""
    orca_step(agvs, corridor_half_w, obstacles=obstacles, responsibility=1.0)
