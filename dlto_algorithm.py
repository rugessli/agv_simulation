from __future__ import annotations
import math
from typing import Protocol

from config import (
    AGV_WIDTH, AGV_LENGTH, WALL_CLEARANCE, DETECT_RANGE, DLTO_PLAN_RANGE,
    PRIORITY_ALPHA, PRIORITY_BETA, PRIORITY_GAMMA,
    PRIORITY_EPSILON, AGV_MAX_PAYLOAD,
    D_MIN, TILT_RESERVE, D_OBSTACLE, SIM_GAP_MARGIN, D_OBS_STOP,
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
    """4-term priority: cargo, payload, proximity-to-goal, ID tie-breaker."""
    c_i    = 1.0 if agv.cargo_state else 0.0
    m_norm = agv.payload_mass / AGV_MAX_PAYLOAD
    goal_x = (agv.corridor_length - 0.5) if agv.direction > 0 else 0.5
    r_norm = 1.0 - min(1.0, abs(agv.x - goal_x) / max(agv.corridor_length, 1.0))
    id_norm = agv.aid / max(all_ids) if max(all_ids) > 0 else 0.0
    return (PRIORITY_ALPHA   * c_i
          + PRIORITY_BETA    * m_norm
          + PRIORITY_GAMMA   * r_norm
          + PRIORITY_EPSILON * (1.0 - id_norm))


def offset_bounds(corridor_width: float):
    """Return (min_off, max_off, available) for a head-on lateral offset.

    min_off : minimum magnitude that yields a >= D_MIN body gap when both
              vehicles offset symmetrically  ((AGV_WIDTH + D_MIN)/2).
    max_off : largest magnitude that still reserves TILT_RESERVE of lateral
              headroom so the rotated body never reaches the wall during the
              transient arc.  Never below min_off (D_MIN takes precedence).
    """
    available = max(0.0, corridor_width / 2.0 - AGV_WIDTH / 2.0 - WALL_CLEARANCE)
    # +GAP_BUFFER over the bare D_MIN target so a slight residual body tilt at the
    # passing instant still leaves the measured gap >= D_MIN.
    GAP_BUFFER = 0.06
    min_off = min((AGV_WIDTH + D_MIN) / 2.0 + GAP_BUFFER, available)
    max_off = max(min_off, available - TILT_RESERVE)
    return min_off, max_off, available


def dlto_target_offset(own: PoseLike, partner: PoseLike,
                       corridor_width: float,
                       all_ids=None,
                       forced_sign: int = None) -> float:
    """Compute lateral target y for `own` when negotiating with `partner`.

    Priority sets where in [min_off, max_off] each vehicle sits: the higher
    priority vehicle stays near min_off (small detour), the lower priority one
    moves further out — but both are bounded by max_off so the body always
    keeps TILT_RESERVE of clearance to the wall during the arc.
    """
    if all_ids is None:
        all_ids = [own.aid, partner.aid]

    p_i = _priority_score(own, all_ids)
    p_j = _priority_score(partner, all_ids)
    total = p_i + p_j
    # offset_coeff in [0,1]: 0 → high priority (small offset), 1 → low priority
    offset_coeff_i = p_j / total if total >= 1e-9 else 0.5

    min_off, max_off, available = offset_bounds(corridor_width)
    range_space = max(0.0, max_off - min_off)
    sym_delta   = min_off + 0.5 * range_space
    raw_i       = min_off + offset_coeff_i * range_space
    delta_i     = max(sym_delta, min(raw_i, available))

    # Guarantee the combined body gap is at least D_MIN even after the cap.
    # (When both sit at min_off the symmetric gap is exactly D_MIN.)
    delta_i     = max(min_off, min(delta_i, available))

    # Direction: forced_sign overrides default keep-right rule (obstacle lanes)
    if forced_sign is not None:
        sign = 1 if forced_sign > 0 else -1
    else:
        sign = -1 if own.direction > 0 else +1
    return sign * delta_i


# ===========================================================================
# Obstacle passing protocol (shared by DLTO / ORCA / FLC-ORCA)
# ===========================================================================
def _lane_centers(obs, corridor_width: float, clr: float):
    """Feasible AGV-centreline y for the upper / lower lane beside an obstacle.

    A lane is feasible only if a straight AGV body fits between the obstacle
    edge (plus clr) and the wall.  Returns (upper|None, lower|None, available).
    """
    available = corridor_width / 2.0 - AGV_WIDTH / 2.0 - WALL_CLEARANCE
    half = AGV_WIDTH / 2.0
    up_lo = obs.y_max + clr + half          # lowest feasible centre above obstacle
    upper = 0.5 * (up_lo + available) if up_lo <= available + 1e-9 else None
    lo_hi = obs.y_min - clr - half          # highest feasible centre below obstacle
    lower = 0.5 * (lo_hi - available) if lo_hi >= -available - 1e-9 else None
    return upper, lower, available


_STATE_RANK = {"WAIT": 0, "CREEP": 1, "SLOW": 2, "APPROACH": 3, "RUN": 4}


def slower_state(a: str, b: str) -> str:
    """Return the more conservative (slower) of two speed states."""
    return a if _STATE_RANK[a] <= _STATE_RANK[b] else b


def obstacle_traverse_state(agv: PoseLike, dist_obs: float) -> str:
    """Speed state for traversing a clear obstacle lane.

    The lane is collision-free by construction, so once the body is aligned
    with the lane there is no reason to crawl — only an un-aligned vehicle
    close to the obstacle slows down to finish easing in.
    """
    if dist_obs > DETECT_RANGE:
        return "APPROACH"
    return "SLOW" if abs(agv.target_y - agv.y) > 0.15 else "APPROACH"


def _relevant_obstacle(agv, obstacles):
    """Nearest obstacle in the encounter zone of agv (ahead within plan range
    or just behind), else None."""
    rear = AGV_LENGTH / 2.0 + 0.2
    best, best_key = None, math.inf
    for obs in obstacles:
        dx = (obs.cx - agv.x) * agv.direction
        rb = obs.length / 2.0 + rear
        if -rb < dx <= DLTO_PLAN_RANGE and abs(dx) < best_key:
            best, best_key = obs, abs(dx)
    return best


def obstacle_plan(agv, agvs, obstacles, corridor_width, prio):
    """Coordinated obstacle-passing plan for `agv`.  Returns (target_y, wait,
    dist_obs) when an obstacle governs agv's motion, otherwise None (the caller
    falls back to its normal head-on / cruise logic).

    Geometry: two opposing vehicles must pass on OPPOSITE sides.  A wall-hugging
    obstacle leaves a single free lane near x≈obstacle, so they cannot pass
    there — the bottleneck is traversed one at a time while the pass itself
    happens in the clear region.  `prio` ranks vehicles (higher = goes first);
    the lower-ranked vehicle yields.

    Crucially the contender is found at ANY range, so the side decision is made
    early (while still far from the obstacle) — the yielder heads straight to
    its pass side with full runway, never committing to the free lane first and
    then having to swap (which would dead-lock once it is stopped).
    """
    if not obstacles:
        return None
    obs = _relevant_obstacle(agv, obstacles)
    if obs is None:
        # agv itself is clear of every obstacle; it may still need to hold the
        # free side until it has crossed a contender that is AT an obstacle.
        contender = _nearest_contender(agv, agvs)
        if contender is None:
            return None
        cobs = _relevant_obstacle(contender, obstacles)
        if cobs is None or passed(agv, contender):
            return None
        obs = cobs

    contender = _nearest_contender(agv, agvs)
    dx_self = (obs.cx - agv.x) * agv.direction
    dist_obs = max(0.0, dx_self)

    min_off, _, available = offset_bounds(corridor_width)
    threshold = obs.width / 2.0
    centred = abs(obs.cy) <= threshold

    # Feasible lanes require the FULL obstacle clearance D_OBSTACLE — no
    # zero-clearance squeeze (fix 2/3).  If neither lane fits, the obstacle is
    # physically impassable for this corridor.
    upper, lower, _ = _lane_centers(obs, corridor_width, D_OBSTACLE)
    wall = corridor_width / 2.0 - WALL_CLEARANCE
    up_gap = wall - obs.y_max
    lo_gap = obs.y_min + wall

    if upper is None and lower is None:
        # Impassable: stop a safe distance in front of the obstacle and hold
        # (recorded honestly — no forced squeeze, no obstacle/vehicle contact).
        stop_at = obs.length / 2.0 + AGV_LENGTH / 2.0 + D_OBS_STOP
        return 0.0, (dx_self <= stop_at), dist_obs

    if centred:
        # right-going keeps the lower gap, left-going the upper gap → opposite.
        if agv.direction > 0:
            through = lower if lower is not None else upper
        else:
            through = upper if upper is not None else lower
        pass_side = through                     # already opposite to contender
        can_simul = (upper is not None and lower is not None
                     and up_gap >= AGV_WIDTH + SIM_GAP_MARGIN
                     and lo_gap >= AGV_WIDTH + SIM_GAP_MARGIN)
    else:
        free_sign = -1 if obs.cy > 0 else +1    # obstacle high → free lane low
        half = AGV_WIDTH / 2.0
        # +0.03 m margins so a simultaneous pass clears D_MIN (vehicle↔vehicle)
        # and D_OBSTACLE (vehicle↔obstacle) with headroom rather than exactly.
        sep   = (AGV_WIDTH + D_MIN) / 2.0 + 0.03   # half the head-on separation
        obsclr = D_OBSTACLE + 0.03
        # Centreline band on the FREE side (the region opposite the obstacle).
        if free_sign < 0:                        # top obstacle → free band below
            band_lo, band_hi = -available, obs.y_min - obsclr - half
        else:                                    # bot obstacle → free band above
            band_lo, band_hi = obs.y_max + obsclr + half, available
        if (band_hi - band_lo) >= 2.0 * sep + SIM_GAP_MARGIN:
            # Free band roomy enough for both vehicles side by side → pass
            # SIMULTANEOUSLY, the pair centred as close to the corridor centre as
            # the band allows.  This keeps both vehicles in the region opposite
            # the obstacle with the minimal necessary offset (no over-shoot to the
            # wall, no long wall-side wait).  right-going takes the lower of the
            # pair, left-going the upper, so they stay on opposite sides.
            c = min(max(0.0, band_lo + sep), band_hi - sep)
            through = (c - sep) if agv.direction > 0 else (c + sep)
            pass_side = through
            can_simul = True
        else:
            # Band too narrow for a side-by-side pass → serialise: the pass
            # happens in the clear region (full corridor width) at the minimal
            # head-on offset; only one vehicle is at the bottleneck at a time.
            through = free_sign * min_off
            pass_side = -free_sign * min_off
            can_simul = False

    if contender is None:
        return through, False, dist_obs
    if can_simul:
        return through, False, dist_obs

    must_yield = prio(agv) < prio(contender)
    if not must_yield:
        # Priority vehicle owns the free lane and stays on it until it has
        # crossed the contender (so for a bot obstacle it does not drift back
        # onto the contender's side after clearing the obstacle).
        return through, False, dist_obs

    # Yielding vehicle.
    if passed(agv, contender):
        return through, False, dist_obs         # crossed → take free lane, traverse
    # Move to the pass side first (still rolling so it can shift), then hold
    # just outside the bottleneck with runway to swing into the free lane once
    # the priority vehicle has gone by.
    reached = abs(agv.y - pass_side) < 0.20
    approach_hold = obs.length / 2.0 + AGV_LENGTH + 1.8
    wait = reached and (0.0 < dx_self < approach_hold)
    return pass_side, wait, dist_obs


def _nearest_contender(agv, agvs):
    """Nearest counter-direction, not-yet-arrived vehicle that agv has not
    already passed — the vehicle that will contest the obstacle with agv."""
    best, best_d = None, math.inf
    for b in agvs:
        if b is agv or b.arrived or b.direction == agv.direction:
            continue
        if passed(agv, b):
            continue
        d = abs(b.x - agv.x)
        if d < best_d:
            best, best_d = b, d
    return best


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


