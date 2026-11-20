"""FLC-ORCA: fuzzy responsibility coefficient for corridor AGV avoidance.

Responsibility-allocation principle of FLC-ORCA (Mohd Romlay et al., 2023):
a fuzzy system replaces the fixed 0.5 coefficient of standard ORCA, using the
normalised distance to the partner and the closing speed (sum of the two
speeds in a head-on encounter) as inputs. Higher responsibility -> the vehicle
deviates more from its preferred path. The coefficient is mapped to a lateral
offset in the same offset framework, controller and safety layer as DLTO and
ORCA; no velocity-obstacle half-planes are computed.
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


def _trimf(x: float, a: float, b: float, c: float) -> float:
    if x <= a or x >= c:
        return 0.0
    if x <= b:
        return (x - a) / (b - a) if b > a else 1.0
    return (c - x) / (c - b) if c > b else 1.0


def _fuzzy_responsibility(rel_dist_norm: float, rel_vel_norm: float) -> float:
    """Fuzzy inference: inputs → responsibility coefficient ∈ [0, 1].

    rel_dist_norm: dist / DLTO_PLAN_RANGE   (0 = very close, 1 = far)
    rel_vel_norm:  closing_speed / CRUISE_SPEED (0 = slow, 1 = fast)
    """
    d_near   = _trimf(rel_dist_norm, 0.0, 0.0, 0.5)
    d_medium = _trimf(rel_dist_norm, 0.0, 0.5, 1.0)
    d_far    = _trimf(rel_dist_norm, 0.5, 1.0, 1.0)

    v_slow   = _trimf(rel_vel_norm, 0.0, 0.0, 0.5)
    v_medium = _trimf(rel_vel_norm, 0.0, 0.5, 1.0)
    v_fast   = _trimf(rel_vel_norm, 0.5, 1.0, 1.0)

    rules = [
        (min(d_near,   v_fast),   0.80),
        (min(d_near,   v_medium), 0.65),
        (min(d_near,   v_slow),   0.50),
        (min(d_medium, v_fast),   0.50),
        (min(d_medium, v_medium), 0.50),
        (min(d_medium, v_slow),   0.40),
        (min(d_far,    v_fast),   0.30),
        (min(d_far,    v_medium), 0.25),
        (min(d_far,    v_slow),   0.20),
    ]
    num = sum(w * c for w, c in rules)
    den = sum(w for w, _ in rules)
    return num / den if den > 1e-9 else 0.5


def _lateral_target(own, partner, corridor_width: float,
                    responsibility: float) -> float:
    """Lateral y-target from the fuzzy responsibility coefficient.

    Magnitude bounded by offset_bounds() to reserve wall clearance for the
    transient body tilt during the arc (Problem 2).
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


def flc_orca_step(agvs: list, corridor_half_w: float,
                  obstacles: list = None) -> None:
    corridor_width = 2.0 * corridor_half_w
    live = [a for a in agvs if not a.arrived]

    from dlto_algorithm import (obstacle_plan, obstacle_traverse_state,
                                slower_state)

    prio = lambda x: -x.aid          # lower id goes first at a bottleneck

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
            # head-on encounter: the two vehicles move towards each other, so their speeds add
            closing_speed = max(1e-6, a.v + partner.v)
            rel_dist_norm = min(1.0, best_d / DLTO_PLAN_RANGE)
            rel_vel_norm  = min(1.0, closing_speed / CRUISE_SPEED)
            resp = _fuzzy_responsibility(rel_dist_norm, rel_vel_norm)

            a.target_y = _lateral_target(a, partner, corridor_width, resp)

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
