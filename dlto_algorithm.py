from __future__ import annotations
import math
from typing import Protocol

from config import (
    AGV_WIDTH, WALL_CLEARANCE, DETECT_RANGE, DLTO_PLAN_RANGE, CRUISE_SPEED,
    PRIORITY_ALPHA, PRIORITY_BETA, PRIORITY_GAMMA,
    PRIORITY_DELTA, PRIORITY_EPSILON, AGV_MAX_PAYLOAD,
)

ANGLE_DIFF_THRESHOLD = math.radians(150.0)
PASS_LONG_THRESHOLD  = 1.0


class PoseLike(Protocol):
    aid:             int
    x:               float
    y:               float
    theta:           float
    direction:       int
    cargo_state:     bool
    payload_mass:    float
    v:               float
    corridor_length: float


def heading_difference(theta_a: float, theta_b: float) -> float:
    d = (theta_a - theta_b + math.pi) % (2.0 * math.pi) - math.pi
    return abs(d)


def is_head_on(theta_a: float, theta_b: float) -> bool:
    return heading_difference(theta_a, theta_b) > ANGLE_DIFF_THRESHOLD


def distance(a: PoseLike, b: PoseLike) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def detect_dlto_conflict(own: PoseLike, partner: PoseLike) -> bool:
    # Use direction attribute (not heading angle) so detection works correctly
    # even when vehicles are tilted during lateral offset execution.
    return (own.direction != partner.direction) \
       and distance(own, partner) <= DLTO_PLAN_RANGE


def detect_sw_conflict(own: PoseLike, partner: PoseLike) -> bool:
    return (own.direction != partner.direction) \
       and distance(own, partner) <= DETECT_RANGE


def _priority_score(agv: PoseLike, all_ids) -> float:
    """5-term priority: cargo, payload, proximity-to-goal, speed, ID tie-breaker."""
    c_i    = 1.0 if agv.cargo_state else 0.0
    m_norm = agv.payload_mass / AGV_MAX_PAYLOAD
    goal_x = (agv.corridor_length - 0.5) if agv.direction > 0 else 0.5
    r_norm = 1.0 - min(1.0, abs(agv.x - goal_x) / max(agv.corridor_length, 1.0))
    v_norm = min(1.0, abs(agv.v) / CRUISE_SPEED) if CRUISE_SPEED > 0 else 0.0
    id_norm = agv.aid / max(all_ids) if max(all_ids) > 0 else 0.0
    return (PRIORITY_ALPHA   * c_i
          + PRIORITY_BETA    * m_norm
          + PRIORITY_GAMMA   * r_norm
          + PRIORITY_DELTA   * v_norm
          + PRIORITY_EPSILON * (1.0 - id_norm))


def dlto_target_offset(own: PoseLike, partner: PoseLike,
                       corridor_width: float,
                       all_ids=None,
                       forced_sign: int = None) -> float:
    """Compute lateral target y for `own` when negotiating with `partner`.

    Layer 1: priority-based raw split — high priority gets small offset.
    Layer 2: safety compensation — guarantee projected body gap >= D_MIN.
    """
    from config import D_MIN

    if all_ids is None:
        all_ids = [own.aid, partner.aid]

    p_i = _priority_score(own, all_ids)
    p_j = _priority_score(partner, all_ids)
    total = p_i + p_j

    # offset_coeff in [0,1]: 0 → high priority (small offset), 1 → low priority
    offset_coeff_i = p_j / total if total >= 1e-9 else 0.5

    available_space = max(0.0, corridor_width / 2.0 - AGV_WIDTH / 2.0 - WALL_CLEARANCE)

    # Layer 1: guaranteed minimum base + priority-scaled extra space.
    # min_off ensures each vehicle provides at least half the required gap.
    # sym_delta is the symmetric (ORCA-equivalent) baseline that guarantees
    # safe dynamic clearance during the lateral approach; priority can only
    # ADD extra offset for low-priority vehicles, never remove from baseline.
    min_off    = min((AGV_WIDTH + D_MIN) / 2.0, available_space)
    range_space = max(0.0, available_space - min_off)
    sym_delta  = min_off + 0.5 * range_space        # symmetric baseline

    if range_space < D_MIN:
        # Corridor too narrow for meaningful priority differentiation:
        # asymmetric targets cause timing divergence that can violate D_MIN.
        # Fall back to symmetric baseline so both vehicles behave like ORCA.
        delta_i = sym_delta
        delta_j = sym_delta
    else:
        raw_i = min_off + offset_coeff_i * range_space
        raw_j = min_off + (1.0 - offset_coeff_i) * range_space
        delta_i = max(sym_delta, raw_i)   # floor at sym; low-prio can go higher
        delta_j = max(sym_delta, raw_j)

    # Layer 2: safety check — catches extreme narrow corridors where
    # available_space < min_off so Layer 1 alone may leave gap < D_MIN.
    projected_gap = (delta_i + delta_j) - AGV_WIDTH
    if projected_gap < D_MIN:
        shortage = D_MIN - projected_gap
        remaining_i = available_space - delta_i
        remaining_j = available_space - delta_j
        total_remaining = remaining_i + remaining_j
        if total_remaining > 1e-9:
            delta_i += shortage * (remaining_i / total_remaining)
            delta_j += shortage * (remaining_j / total_remaining)
        else:
            delta_i = available_space
            delta_j = available_space
        delta_i = min(delta_i, available_space)

    # Direction: forced_sign overrides default keep-right rule (used for obstacle scenarios)
    if forced_sign is not None:
        sign = forced_sign
    else:
        sign = -1 if own.direction > 0 else +1
    return sign * delta_i


def dlto_feasible(own: PoseLike, partner: PoseLike,
                  corridor_width: float, all_ids=None) -> bool:
    """Return False if the lower-priority vehicle has no room to offset."""
    if all_ids is None:
        all_ids = [own.aid, partner.aid]
    available_space = corridor_width / 2.0 - AGV_WIDTH / 2.0 - WALL_CLEARANCE
    return available_space >= 0.0


def passed(own: PoseLike, partner: PoseLike,
           threshold: float = PASS_LONG_THRESHOLD) -> bool:
    fx, fy = math.cos(own.theta), math.sin(own.theta)
    rel_x, rel_y = partner.x - own.x, partner.y - own.y
    longitudinal = fx * rel_x + fy * rel_y
    return longitudinal < -threshold


def find_obstacle_in_path(agv: PoseLike, obstacles: list,
                          plan_range: float = DLTO_PLAN_RANGE) -> object:
    """Return nearest Obstacle that blocks agv's forward path, or None."""
    best_obs, best_dx = None, math.inf
    for obs in obstacles:
        dx = (obs.cx - agv.x) * agv.direction
        if dx <= 0.0 or dx > plan_range:
            continue
        # Y-overlap: will the vehicle's path intersect the obstacle?
        agv_half = AGV_WIDTH / 2.0
        obs_half = obs.width / 2.0
        if abs(obs.cy - agv.y) < (agv_half + obs_half + 0.05):
            if dx < best_dx:
                best_obs, best_dx = obs, dx
    return best_obs


def obstacle_avoidance_target(agv: PoseLike, obs,
                              corridor_width: float) -> float:
    """Lateral target y to bypass a static obstacle.

    Same-side rule: both counter-direction vehicles detour to the obstacle-free side
    so they share one passing lane (serialised by priority) rather than colliding
    at the obstacle choke-point.

      obs.cy < -threshold → obstacle in lower half → BOTH go upper (+1)
      obs.cy > +threshold → obstacle in upper half → BOTH go lower (-1)
      |obs.cy| ≤ threshold → obstacle at centre → split by direction so
          right-going goes upper (+1), left-going goes lower (-1),
          keeping them on opposite sides of the centre obstacle.
    """
    available = max(0.0, corridor_width / 2.0 - AGV_WIDTH / 2.0 - WALL_CLEARANCE)
    threshold = obs.width / 2.0      # obstacle spans ±threshold from its centre
    if obs.cy < -threshold:
        sign = +1  # obstacle below → both upper
    elif obs.cy > threshold:
        sign = -1  # obstacle above → both lower
    else:
        # Centre obstacle: split so vehicles pass on opposite sides
        sign = +1 if agv.direction > 0 else -1
    return sign * available
