from __future__ import annotations
import math
import copy
from typing import List, Optional, Tuple

import numpy as np

from config import (
    AGV_LENGTH, AGV_WIDTH, CRUISE_SPEED, APPROACH_SPEED, OFFSET_SPEED, CREEP_SPEED,
    WALL_CLEARANCE, CORRIDOR_LENGTH, DT, T_MAX, T_DEAD,
    D_CLEAR, D_MIN, AGV_MAX_PAYLOAD, AGV_MIN_TURN_R,
    DETECT_RANGE, DLTO_PLAN_RANGE,
)
from agv_model import (
    AGVState, integrate, steering_omega, has_arrived, corners,
)
from dlto_algorithm import (
    detect_dlto_conflict, detect_sw_conflict,
    dlto_target_offset, dlto_feasible, passed, distance,
    find_obstacle_in_path, obstacle_avoidance_target,
)
from orca_algorithm import orca_step
from flc_orca_algorithm import flc_orca_step

MAX_SIM_TIME   = T_MAX
DEADLOCK_WINDOW = T_DEAD


# ===========================================================================
# Scenario factory
# ===========================================================================
def _assign_payload(agv: AGVState, profile: str, rng) -> None:
    if profile == "unloaded":
        agv.cargo_state  = False
        agv.payload_mass = 0.0
    elif profile == "loaded":
        agv.cargo_state  = True
        agv.payload_mass = float(rng.uniform(500, AGV_MAX_PAYLOAD))
    else:  # mixed / obstacle variants: 50% loaded
        if rng.random() < 0.5:
            agv.cargo_state  = True
            agv.payload_mass = float(rng.uniform(500, AGV_MAX_PAYLOAD))
        else:
            agv.cargo_state  = False
            agv.payload_mass = 0.0


def make_scenario(n_agv: int, seed: int,
                  corridor_width: float, corridor_length: float,
                  payload_profile: str) -> List[AGVState]:
    rng = np.random.default_rng(seed)
    agvs = []
    half_right = n_agv // 2
    half_left  = n_agv - half_right

    start_gap = max(AGV_LENGTH + 0.5, 3.0)
    right_starts = [0.5 + i * start_gap for i in range(half_right)]
    left_starts  = [corridor_length - 0.5 - i * start_gap for i in range(half_left)]

    right_x_offset = float(rng.uniform(0.0, 2.0))
    left_x_offset  = float(rng.uniform(0.0, 2.0))

    y_safe = corridor_width / 2.0 - AGV_WIDTH / 2.0 - WALL_CLEARANCE - 0.05
    y_safe = max(0.0, y_safe)
    y_jitter = min(0.30, y_safe)

    aid = 0
    for i, sx in enumerate(right_starts):
        dep_delay    = float(rng.uniform(0.0, 1.0))
        speed_factor = float(rng.uniform(0.85, 1.0))
        jy = float(rng.uniform(-y_jitter, y_jitter))
        agv = AGVState(aid=aid, x=sx + right_x_offset, y=jy, theta=0.0,
                       direction=+1, corridor_length=corridor_length,
                       departure_delay=dep_delay, speed_factor=speed_factor)
        _assign_payload(agv, payload_profile, rng)
        agvs.append(agv)
        aid += 1

    for i, sx in enumerate(left_starts):
        dep_delay    = float(rng.uniform(0.0, 1.0))
        speed_factor = float(rng.uniform(0.85, 1.0))
        jy = float(rng.uniform(-y_jitter, y_jitter))
        agv = AGVState(aid=aid, x=sx - left_x_offset, y=jy, theta=math.pi,
                       direction=-1, corridor_length=corridor_length,
                       departure_delay=dep_delay, speed_factor=speed_factor)
        _assign_payload(agv, payload_profile, rng)
        agvs.append(agv)
        aid += 1

    return agvs


# ===========================================================================
# Partner search
# ===========================================================================
def find_dlto_partner(agv: AGVState, agvs: List[AGVState]) -> Optional[AGVState]:
    best, best_d = None, math.inf
    for b in agvs:
        if b is agv or b.arrived:
            continue
        if detect_dlto_conflict(agv, b):
            d = distance(agv, b)
            if d < best_d:
                best, best_d = b, d
    return best


def find_sw_blocker(agv: AGVState, agvs: List[AGVState]) -> Optional[AGVState]:
    best, best_d = None, math.inf
    for b in agvs:
        if b is agv or b.arrived:
            continue
        if detect_sw_conflict(agv, b):
            d = distance(agv, b)
            if d < best_d:
                best, best_d = b, d
    return best


# ===========================================================================
# Algorithm steps
# ===========================================================================
def _dist_based_state(dist: float) -> str:
    """Map partner distance to AGV speed state (Fix 1: four-speed ladder)."""
    if dist <= D_CLEAR:
        return "CREEP"
    if dist <= DETECT_RANGE:
        return "SLOW"
    return "APPROACH"  # dist <= DLTO_PLAN_RANGE (partner found)


def step_dlto(agvs: List[AGVState], t: float, pair_times: dict,
              corridor_width: float, obstacles: list = None) -> None:
    all_ids = [a.aid for a in agvs]
    obstacles = obstacles or []
    for a in agvs:
        if a.arrived:
            continue

        partner = find_dlto_partner(a, agvs)
        if partner is not None:
            if dlto_feasible(a, partner, corridor_width, all_ids):
                final_target = dlto_target_offset(a, partner, corridor_width, all_ids)
            else:
                p_a = _priority(a, all_ids)
                p_b = _priority(partner, all_ids)
                if p_a < p_b:
                    a.state = "WAIT"
                    continue
                final_target = dlto_target_offset(a, partner, corridor_width, all_ids)

            dist = distance(a, partner)
            if dist <= DETECT_RANGE:
                key = tuple(sorted((a.aid, partner.aid)))
                pair_times.setdefault(key, {"detect": t, "resolved": None})

            a.target_y = final_target
            a.smooth_target_y = final_target

            # Fix 1: four-speed state based on distance to partner
            a.state = _dist_based_state(dist)

            # Fix 3: wait-for-offset rule — the vehicle closer to its target
            # holds while the lagging partner completes its lateral offset.
            # Prevents mid-rotation tail sweeps causing MCD < D_MIN.
            if dist <= DETECT_RANGE:
                own_err = abs(a.target_y - a.y)
                partner_err = abs(partner.target_y - partner.y)
                if partner_err > 0.25 and own_err < partner_err - 0.05:
                    a.state = "WAIT"

        elif obstacles:
            obs = find_obstacle_in_path(a, obstacles)
            if obs is not None:
                obs_target = obstacle_avoidance_target(a, obs, corridor_width)
                a.target_y = obs_target
                a.smooth_target_y = obs_target
                dist_to_obs = (obs.cx - a.x) * a.direction
                if dist_to_obs <= D_CLEAR:
                    a.state = "CREEP"
                elif dist_to_obs <= DETECT_RANGE:
                    a.state = "SLOW"
                else:
                    a.state = "APPROACH"
            else:
                a.state = "RUN"
                a.smooth_target_y *= 0.95
                if abs(a.smooth_target_y) < 0.005:
                    a.smooth_target_y = 0.0
                a.target_y = a.smooth_target_y

        else:
            a.state = "RUN"
            a.smooth_target_y *= 0.95
            if abs(a.smooth_target_y) < 0.005:
                a.smooth_target_y = 0.0
            a.target_y = a.smooth_target_y


def _priority(agv, all_ids):
    from config import PRIORITY_ALPHA, PRIORITY_BETA, PRIORITY_GAMMA
    c_i = 1.0 if agv.cargo_state else 0.0
    m_norm = agv.payload_mass / AGV_MAX_PAYLOAD
    id_norm = agv.aid / max(all_ids) if max(all_ids) > 0 else 0.0
    return PRIORITY_ALPHA * c_i + PRIORITY_BETA * m_norm + PRIORITY_GAMMA * (1.0 - id_norm)


def step_sw(agvs: List[AGVState], t: float, pair_times: dict,
            obstacles: list = None) -> None:
    # SW only has CRUISE and STOP — no intermediate speeds
    obstacles = obstacles or []
    for a in agvs:
        if a.arrived:
            continue
        a.target_y = 0.0
        a.state = "RUN"
    for a in agvs:
        if a.arrived:
            continue
        b = find_sw_blocker(a, agvs)
        if b is not None:
            key = tuple(sorted((a.aid, b.aid)))
            pair_times.setdefault(key, {"detect": t, "resolved": None})
            if a.aid > b.aid:
                a.state = "WAIT"
        elif obstacles:
            for obs in obstacles:
                dx = (obs.cx - a.x) * a.direction
                if 0.0 < dx <= DETECT_RANGE:
                    agv_half = AGV_WIDTH / 2.0
                    if abs(obs.cy - a.y) < (agv_half + obs.width / 2.0 + 0.10):
                        a.state = "WAIT"
                        break


def _apply_following_distance(agvs: List[AGVState]) -> None:
    """Enforce minimum following gap for same-direction AGVs.

    Fix 3: larger gap required when leader is executing lateral offset
    (body rotation sweeps tail toward follower).
    Fix 4: encounter-zone serialisation forces follower to WAIT at zone
    entry when a same-direction leader is actively offsetting inside.
    """
    safe_gap  = AGV_LENGTH + D_MIN          # 2.65 m centre-to-centre minimum
    zone_lo   = CORRIDOR_LENGTH / 3.0       # ~13.3 m
    zone_hi   = 2.0 * CORRIDOR_LENGTH / 3.0 # ~26.7 m
    # Extra gap to accommodate body-rotation tail sweep during lateral offset
    offset_extra = AGV_LENGTH               # 2.35 m → total 5.0 m when offsetting

    live = [a for a in agvs if not a.arrived]
    for a in live:
        if a.state == "WAIT":
            continue

        # --- Find nearest same-direction leader (ahead of a) ---
        nearest_leader = None
        min_x_rel = math.inf
        for b in live:
            if b is a or b.arrived or b.direction != a.direction:
                continue
            x_rel = (b.x - a.x) * a.direction
            if 0 < x_rel < min_x_rel:
                nearest_leader = b
                min_x_rel = x_rel

        if nearest_leader is not None:
            b = nearest_leader
            x_rel = min_x_rel
            offsetting = abs(b.target_y) > 0.05
            gap = safe_gap + (offset_extra if offsetting else 0.0)
            if x_rel < gap:
                if x_rel < AGV_LENGTH + 0.10:
                    a.state = "WAIT"
                elif x_rel < safe_gap:
                    a.state = "WAIT" if a.state in ("CREEP",) else "CREEP"
                else:
                    # Maintain CREEP while closing the extended gap during offset
                    if a.state not in ("WAIT", "CREEP"):
                        a.state = "CREEP"
                continue

        # --- Fix 4: encounter-zone serialisation ---
        # If a same-direction leader is ACTIVELY offsetting inside the zone
        # (not itself stuck in WAIT), slow the follower at zone entry.
        # Skip if leader is WAIT — means leader is stuck waiting for its OWN
        # partner; forcing the follower to also WAIT would deadlock 4-AGV.
        for b in live:
            if b is a or b.arrived or b.direction != a.direction:
                continue
            x_rel = (b.x - a.x) * a.direction
            if x_rel <= 0:
                continue
            if (zone_lo <= b.x <= zone_hi and abs(b.target_y) > 0.05
                    and b.state not in ("WAIT",)):
                dist_to_zone = (
                    (zone_lo - a.x) if a.direction > 0 else (a.x - zone_hi))
                # Only queue at zone ENTRY (approaching from outside).
                # If follower is already inside the zone (dist_to_zone < 0),
                # it has its own crossing underway — don't restrict it.
                if 0.0 <= dist_to_zone <= AGV_LENGTH * 3:
                    if a.state not in ("WAIT",):
                        a.state = "CREEP"
                break


# ===========================================================================
# Collision detection
# ===========================================================================
def _sat_overlap(c1, c2, margin: float = 0.002) -> bool:
    polys = (c1, c2)
    for poly in polys:
        n = len(poly)
        for i in range(n):
            edge = (poly[(i+1) % n][0] - poly[i][0],
                    poly[(i+1) % n][1] - poly[i][1])
            axis = (-edge[1], edge[0])
            mag = math.hypot(*axis) or 1.0
            ax, ay = axis[0]/mag, axis[1]/mag
            p1 = [pt[0]*ax + pt[1]*ay for pt in c1]
            p2 = [pt[0]*ax + pt[1]*ay for pt in c2]
            if max(p1) < min(p2) + margin or max(p2) < min(p1) + margin:
                return False
    return True


def _would_collide(agv: AGVState, v: float, omega: float, dt: float,
                   others: List[AGVState], corridor_half_w: float,
                   obstacles: list = None) -> bool:
    test = AGVState(aid=agv.aid, x=agv.x, y=agv.y, theta=agv.theta,
                    direction=agv.direction,
                    corridor_length=agv.corridor_length)
    integrate(test, v, omega, dt, corridor_half_w)
    test_c = corners(test)
    for b in others:
        if b is agv or b.arrived:
            continue
        if _sat_overlap(test_c, corners(b)):
            return True
    if obstacles:
        for obs in obstacles:
            if _sat_overlap(test_c, obs.corners()):
                return True
    return False


def safe_motion(agv: AGVState, v_cmd: float, omega_cmd: float, dt: float,
                others: List[AGVState], corridor_half_w: float,
                obstacles: list = None) -> None:
    if not _would_collide(agv, v_cmd, omega_cmd, dt, others, corridor_half_w, obstacles):
        integrate(agv, v_cmd, omega_cmd, dt, corridor_half_w)
        return
    lo, hi = 0.0, v_cmd
    for _ in range(8):
        mid = 0.5 * (lo + hi)
        if _would_collide(agv, mid, omega_cmd, dt, others, corridor_half_w, obstacles):
            hi = mid
        else:
            lo = mid
    if lo > 0.05:
        integrate(agv, lo, omega_cmd, dt, corridor_half_w)
        return
    if not _would_collide(agv, 0.0, omega_cmd, dt, others, corridor_half_w, obstacles):
        integrate(agv, 0.0, omega_cmd, dt, corridor_half_w)
    else:
        integrate(agv, 0.0, 0.0, dt, corridor_half_w)


def pairwise_min_obb_distance(c1, c2) -> float:
    if _sat_overlap(c1, c2, margin=0.0):
        return 0.0

    def pt_seg(p, a, b):
        ax, ay = b[0] - a[0], b[1] - a[1]
        d2 = ax * ax + ay * ay or 1e-9
        t = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / d2))
        cx, cy = a[0] + t * ax, a[1] + t * ay
        return math.hypot(p[0] - cx, p[1] - cy)

    d = math.inf
    for p in c1:
        for i in range(len(c2)):
            d = min(d, pt_seg(p, c2[i], c2[(i+1) % len(c2)]))
    for p in c2:
        for i in range(len(c1)):
            d = min(d, pt_seg(p, c1[i], c1[(i+1) % len(c1)]))
    return d


# ===========================================================================
# Obstacle helpers
# ===========================================================================
def make_obstacle_set(profile: str = "obstacle_mid",
                      corridor_width: float = 3.2) -> list:
    """Create a single obstacle at x=20 m; y depends on profile and corridor_width.

    obstacle_top: y = +(corridor_width/2 - obstacle_half - WALL_CLEARANCE)  — hugs upper wall
    obstacle_mid: y = 0                                                       — corridor centre
    obstacle_bot: y = -(corridor_width/2 - obstacle_half - WALL_CLEARANCE)  — hugs lower wall
    """
    from agv_model import Obstacle
    obstacle_half = 0.25   # half-size of 0.5 m obstacle
    cx = CORRIDOR_LENGTH / 2.0  # fixed at corridor midpoint = 20.0 m
    y_hugs_wall = corridor_width / 2.0 - obstacle_half - WALL_CLEARANCE
    cy_map = {
        "obstacle_top": +y_hugs_wall,
        "obstacle_mid":  0.0,
        "obstacle_bot": -y_hugs_wall,
        "obstacle":      0.0,
    }
    cy = cy_map.get(profile, 0.0)
    return [Obstacle(x_min=cx - obstacle_half, x_max=cx + obstacle_half,
                     y_min=cy - obstacle_half, y_max=cy + obstacle_half)]


# ===========================================================================
# Main loop
# ===========================================================================
def run_scenario(n_agv: int, algorithm: str, seed: int,
                 corridor_width: float = 3.2,
                 corridor_length: float = CORRIDOR_LENGTH,
                 payload_profile: str = "unloaded",
                 record_trajectory: bool = False,
                 obstacles: list = None,
                 ) -> Tuple[List[AGVState], Optional[List], float,
                            dict, float, float, list]:

    agvs = make_scenario(n_agv, seed, corridor_width, corridor_length,
                         payload_profile)
    corridor_half_w = corridor_width / 2.0
    pair_times: dict = {}
    trajectory: list = []
    min_clearance = math.inf
    t = 0.0

    if obstacles is None and payload_profile in (
            "obstacle", "obstacle_top", "obstacle_mid", "obstacle_bot"):
        obstacles = make_obstacle_set(payload_profile, corridor_width)

    while t < MAX_SIM_TIME:
        if algorithm == "DLTO":
            step_dlto(agvs, t, pair_times, corridor_width, obstacles)
        elif algorithm == "SW":
            step_sw(agvs, t, pair_times, obstacles)
        elif algorithm == "ORCA":
            orca_step(agvs, corridor_half_w, obstacles=obstacles)
            _sync_orca_state(agvs, t, pair_times)
        elif algorithm == "FLC_ORCA":
            flc_orca_step(agvs, corridor_half_w, obstacles=obstacles)
            _sync_orca_state(agvs, t, pair_times)
        else:
            raise ValueError(f"Unknown algorithm: {algorithm!r}")

        # Resolve tracking
        by_id = {a.aid: a for a in agvs}
        for key, pt in pair_times.items():
            if pt["resolved"] is not None:
                continue
            a_lo, a_hi = by_id.get(key[0]), by_id.get(key[1])
            if a_lo is None or a_hi is None:
                continue
            if a_lo.arrived or a_hi.arrived:
                pt["resolved"] = t
                continue
            if passed(a_lo, a_hi) or passed(a_hi, a_lo):
                pt["resolved"] = t

        # Fix 3+4: enforce following distance and encounter-zone serialisation
        _apply_following_distance(agvs)

        for a in agvs:
            if a.arrived:
                a.v = 0.0
                continue
            if t < a.departure_delay:
                integrate(a, 0.0, 0.0, DT, corridor_half_w)
                continue

            # Fix 1: four-speed command based on state
            if a.state == "WAIT":
                v_cmd, omega_cmd = 0.0, 0.0
            elif a.state == "CREEP":
                v_cmd    = CREEP_SPEED
                omega_cmd = steering_omega(a)
            elif a.state == "SLOW":
                v_cmd    = OFFSET_SPEED * a.speed_factor
                omega_cmd = steering_omega(a)
            elif a.state == "APPROACH":
                v_cmd    = APPROACH_SPEED * a.speed_factor
                omega_cmd = steering_omega(a)
            else:  # RUN
                v_cmd    = CRUISE_SPEED * a.speed_factor
                omega_cmd = steering_omega(a)

            safe_motion(a, v_cmd, omega_cmd, DT, agvs, corridor_half_w, obstacles)

            if has_arrived(a):
                a.arrived = True
                a.arrival_time = t

        live = [a for a in agvs if not a.arrived]
        # MCD: counter-direction (head-on) pairs only
        for i in range(len(live)):
            for j in range(i + 1, len(live)):
                if live[i].direction == live[j].direction:
                    continue
                d = pairwise_min_obb_distance(corners(live[i]), corners(live[j]))
                if d < min_clearance:
                    min_clearance = d

        if record_trajectory:
            trajectory.append((t, [(a.x, a.y, a.theta, a.state, a.arrived,
                                    a.cargo_state, a.target_y)
                                   for a in agvs]))

        if all(a.arrived for a in agvs):
            break
        t += DT

    makespan = max((a.arrival_time for a in agvs if a.arrived), default=t)
    total_dist = sum(a.distance_travelled for a in agvs)
    if math.isinf(min_clearance):
        min_clearance = corridor_half_w

    return (agvs,
            trajectory if record_trajectory else None,
            makespan,
            pair_times,
            min_clearance,
            total_dist,
            [])


def _sync_orca_state(agvs: list, t: float, pair_times: dict) -> None:
    """Mark ORCA head-on conflicts in pair_times for metric tracking."""
    live = [a for a in agvs if not a.arrived]
    for i, a in enumerate(live):
        for b in live[i+1:]:
            if a.direction == b.direction:
                continue
            d = math.hypot(a.x - b.x, a.y - b.y)
            if d <= DETECT_RANGE:
                key = tuple(sorted((a.aid, b.aid)))
                pair_times.setdefault(key, {"detect": t, "resolved": None})
