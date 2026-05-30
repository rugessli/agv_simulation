"""
simulation.py
=============
Scenario definitions and the main simulation loop.

What it does
------------
  * `make_scenario(name, seed)` — produces a list of `AGVState` for
        scenario A (2 AGVs head-on) or B (4 AGVs head-on), with a small
        per-replication position jitter seeded by `seed`.
  * `step_dlto` / `step_sw` — per-tick decision steps that consult
        `dlto_algorithm.py` and set `target_y` / `state` for each AGV.
  * `run_scenario(name, algo, seed)` — drives the tick loop, applies
        controllers, integrates motion, halts on collision, records the
        trajectory and per-pair detection / pass times.

ROS2 migration notes
--------------------
ROS2-READY :
  - `step_dlto`, `step_sw`            (decision logic, no I/O)
  - `find_dlto_partner`, `find_sw_blocker`
  - `pairwise_min_obb_distance`       (kinematic-only, reusable)

ROS2-REPLACE :
  - `run_scenario`                    — in ROS2 each AGV is its own node and
                                         the loop is `rclpy.spin(...)`.  Move
                                         the per-tick body into a timer
                                         callback inside the AGV node.
  - `make_scenario`                   — pose initialisation in ROS2 comes
                                         from `/gazebo/spawn_entity` or a
                                         launch file, not from Python.
  - The collision-bisection safe-step here is a stand-in for Gazebo physics;
    delete it when migrating to a real physics engine.
"""

from __future__ import annotations
import math
import copy
from typing import List, Optional, Tuple

import numpy as np

from agv_model import (
    AGVState, AGV_LENGTH, AGV_WIDTH, MAX_SPEED, OFFSET_SPEED, SLOW_SPEED,
    WALL_LIMIT, integrate, steering_omega, has_arrived, corners, Obstacle,
)
from dlto_algorithm import (
    detect_dlto_conflict, detect_sw_conflict,
    dlto_target_offset, passed, distance, OFFSET_TARGET,
)


# ---------------------------------------------------------------------------
# Scenario-C lane geometry (traffic-rule style)
# ---------------------------------------------------------------------------
LANE_UPPER_Y     = +0.375     # AGV1 cruising lane (heading left)
LANE_LOWER_Y     = -0.375     # AGV0 cruising lane (heading right)
BYPASS_LEAD      = 3.5        # start dodging this many m before the obstacle
BYPASS_TAIL      = 1.5        # rejoin the lane this many m after the obstacle


# ---------------------------------------------------------------------------
# Scenario-C catalogue — three obstacle positions
# ---------------------------------------------------------------------------
SCENARIO_C_CONFIGS = {
    "C_TOP": {
        "obstacle":      ((4.8, 5.2), (+0.60, +0.75)),
        "agv0_detour_y": None,          # AGV0's lane is clear
        "agv1_detour_y": +0.27,         # body top +0.57, 3 cm below obstacle
        "label":         "obstacle on TOP wall",
    },
    "C_BOT": {
        "obstacle":      ((4.8, 5.2), (-0.75, -0.60)),
        "agv0_detour_y": -0.27,         # body bot −0.57, 3 cm above obstacle
        "agv1_detour_y": None,
        "label":         "obstacle on BOTTOM wall",
    },
    "C_MID": {
        "obstacle":      ((4.8, 5.2), (-0.10, +0.10)),
        "agv0_detour_y": -0.42,         # squeezes to lower wall
        "agv1_detour_y": +0.42,         # squeezes to upper wall
        "label":         "obstacle on CENTRE line",
    },
}


def _scenario_c_lane_y(agv: AGVState) -> float:
    """ROS2-READY: AGV's nominal cruising y under the lane discipline."""
    return LANE_UPPER_Y if agv.direction < 0 else LANE_LOWER_Y


def _agv_in_obstacle_corridor(agv: AGVState, obs: Obstacle) -> bool:
    """ROS2-READY: is this AGV physically near the obstacle in x?
    Lead distance is applied on the side the AGV is approaching from,
    tail distance on the side it is leaving toward."""
    if agv.direction > 0:
        return obs.x_min - BYPASS_LEAD <= agv.x <= obs.x_max + BYPASS_TAIL
    return obs.x_min - BYPASS_TAIL <= agv.x <= obs.x_max + BYPASS_LEAD


def _needs_bypass(agv: AGVState, obstacles, lane_y: float) -> bool:
    """ROS2-READY: should `agv` deviate from its lane because of an obstacle?"""
    for o in obstacles:
        if (o.cx - agv.x) * agv.direction < -o.length / 2 - BYPASS_TAIL:
            continue
        if _agv_in_obstacle_corridor(agv, o) and o.blocks_lane(lane_y):
            return True
    return False


# ---------------------------------------------------------------------------
# Simulation-wide constants
# ---------------------------------------------------------------------------
CORRIDOR_LENGTH = 10.0
CORRIDOR_WIDTH  = 1.5            # walls at y = ±0.75
HALF_W          = CORRIDOR_WIDTH / 2.0   # 0.75
DT              = 0.05
MAX_SIM_TIME    = 30.0
DEADLOCK_WINDOW = 10.0           # DRR: resolved within 10 s of detection

SIM_TIME_LIMIT = MAX_SIM_TIME   # back-compat alias


# ===========================================================================
# Scenario factory — ROS2-REPLACE
# ===========================================================================
def make_scenario(name: str, seed: int) -> List[AGVState]:
    rng = np.random.default_rng(seed)
    jy = float(rng.uniform(-0.02, 0.02))      # tiny y jitter per replication
    jx = float(rng.uniform(-0.05, 0.05))

    # -----------------------------------------------------------------------
    # FIX: Scenarios A and B now start AGVs in their proper lanes.
    #   AGV0 (heading right, direction=+1) starts in LOWER lane y = -0.375
    #   AGV1 (heading left,  direction=-1) starts in UPPER lane y = +0.375
    # This matches the two-lane traffic-rule layout used in Scenario C and
    # is consistent with dlto_target_offset() which offsets heading-right
    # AGVs to negative y and heading-left AGVs to positive y.
    # -----------------------------------------------------------------------
    if name == "A":
        return [
            AGVState(aid=0, x=0.5 + jx, y=LANE_LOWER_Y + jy * 0.2,
                     theta=0.0,     direction=+1),
            AGVState(aid=1, x=9.5 + jx, y=LANE_UPPER_Y + jy * 0.2,
                     theta=math.pi, direction=-1),
        ]

    if name == "B":
        return [
            AGVState(aid=0, x=0.5 + jx, y=LANE_LOWER_Y + jy * 0.2,
                     theta=0.0,     direction=+1),
            AGVState(aid=1, x=1.5 + jx, y=LANE_LOWER_Y - jy * 0.2,
                     theta=0.0,     direction=+1),
            AGVState(aid=2, x=9.5 - jx, y=LANE_UPPER_Y + jy * 0.2,
                     theta=math.pi, direction=-1),
            AGVState(aid=3, x=8.5 - jx, y=LANE_UPPER_Y - jy * 0.2,
                     theta=math.pi, direction=-1),
        ]

    if name in SCENARIO_C_CONFIGS:
        return [
            AGVState(aid=0, x=0.5 + jx, y=LANE_LOWER_Y + jy * 0.2,
                     theta=0.0,    direction=+1),
            AGVState(aid=1, x=9.5 + jx, y=LANE_UPPER_Y + jy * 0.2,
                     theta=math.pi, direction=-1),
        ]

    raise ValueError(f"Unknown scenario {name!r}")


def scenario_obstacles(name: str):
    """ROS2-READY (data): obstacle list per scenario."""
    cfg = SCENARIO_C_CONFIGS.get(name)
    if cfg is None:
        return []
    (x_min, x_max), (y_min, y_max) = cfg["obstacle"]
    return [Obstacle(x_min=x_min, x_max=x_max, y_min=y_min, y_max=y_max)]


def _detour_target(agv: AGVState, scenario_name: str):
    cfg = SCENARIO_C_CONFIGS.get(scenario_name)
    if cfg is None:
        return None
    return cfg["agv0_detour_y"] if agv.aid == 0 else cfg["agv1_detour_y"]


# ===========================================================================
# Partner search — ROS2-READY
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
# Algorithm steps — ROS2-READY
# ===========================================================================
def step_dlto(agvs: List[AGVState], t: float,
              pair_times: dict) -> None:
    """Each AGV picks the closest opposing AGV inside DLTO radius and
    commits to an offset lane.  After passing the partner the target_y is
    held — the AGV does NOT drift back to centre.

    FIX: dlto_target_offset now receives agv.direction so offset direction
    is determined by travel direction (keep-right rule), not by AGV ID.
    """
    for a in agvs:
        if a.arrived:
            continue
        partner = find_dlto_partner(a, agvs)
        if partner is not None:
            # FIXED: pass a.direction so heading-right → negative y,
            #        heading-left → positive y  (keep-right traffic rule)
            a.target_y = dlto_target_offset(a.aid, partner.aid, a.direction)
            key = tuple(sorted((a.aid, partner.aid)))
            pt = pair_times.setdefault(key, {"detect": t, "resolved": None})
            if pt["resolved"] is None and passed(a, partner):
                pt["resolved"] = t
        a.state = "RUN"          # SW-style waiting is never used in DLTO


def step_sw(agvs: List[AGVState], t: float,
            pair_times: dict) -> None:
    """All AGVs hold the centreline.  In any predicted head-on conflict
    (within 1.5 m) the higher-ID (lower-priority) AGV WAITs."""
    for a in agvs:
        if a.arrived:
            continue
        # SW holds each AGV in its lane, not the centre line
        a.target_y = LANE_LOWER_Y if a.direction > 0 else LANE_UPPER_Y
        a.state = "RUN"

    for a in agvs:
        if a.arrived:
            continue
        b = find_sw_blocker(a, agvs)
        if b is not None:
            key = tuple(sorted((a.aid, b.aid)))
            pt = pair_times.setdefault(key, {"detect": t, "resolved": None})
            if a.aid > b.aid:
                a.state = "WAIT"


# ===========================================================================
# Scenario-C step functions (lane discipline + obstacle bypass)
# ===========================================================================
def step_dlto_lane(agvs, obstacles, scenario_name, t,
                   pair_times, deadlock_events):
    """DLTO with traffic-rule lanes + scenario-specific detour."""
    detouring = {}

    for a in agvs:
        if a.arrived:
            continue
        a.state = "RUN"
        lane_y = _scenario_c_lane_y(a)
        if _needs_bypass(a, obstacles, lane_y):
            target = _detour_target(a, scenario_name)
            a.target_y = target if target is not None else lane_y
            detouring[a.aid] = True
        else:
            a.target_y = lane_y
            detouring[a.aid] = False

    for a in agvs:
        if a.arrived:
            continue
        partner = find_dlto_partner(a, agvs)
        if partner is None:
            continue
        key = tuple(sorted((a.aid, partner.aid)))
        pt = pair_times.setdefault(key, {"detect": t, "resolved": None})
        if pt["resolved"] is None and passed(a, partner):
            pt["resolved"] = t
            deadlock_events.append((t, "DEADLOCK RESOLVED"))

        if (not detouring.get(a.aid, False)
                and detouring.get(partner.aid, False)):
            a.state = "SLOW"
            if pt.get("slow_logged") is None:
                pt["slow_logged"] = t
                deadlock_events.append(
                    (t, "DEADLOCK DETECTED - LANE CROSSING CONFLICT"))


def step_sw_lane(agvs, obstacles, scenario_name, t,
                 pair_times, deadlock_events):
    """SW with traffic-rule lanes + scenario detour but NO lateral yield."""
    for a in agvs:
        if a.arrived:
            continue
        a.state = "RUN"
        lane_y = _scenario_c_lane_y(a)
        if _needs_bypass(a, obstacles, lane_y):
            target = _detour_target(a, scenario_name)
            a.target_y = target if target is not None else lane_y
        else:
            a.target_y = lane_y

    for a in agvs:
        if a.arrived:
            continue
        b = find_sw_blocker(a, agvs)
        if b is None:
            continue
        key = tuple(sorted((a.aid, b.aid)))
        pt = pair_times.setdefault(key, {"detect": t, "resolved": None})
        if pt["resolved"] is None and passed(a, b):
            pt["resolved"] = t
            deadlock_events.append((t, "DEADLOCK RESOLVED"))
        if a.aid > b.aid:
            a.state = "WAIT"
            if pt.get("wait_logged") is None:
                pt["wait_logged"] = t
                deadlock_events.append(
                    (t, "DEADLOCK DETECTED - LANE CROSSING CONFLICT"))


# ===========================================================================
# Physical collision bisection (ROS2-REPLACE)
# ===========================================================================
def _sat_overlap(c1, c2, margin: float = 0.002) -> bool:
    """SAT for two convex polygons."""
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
                   others: List[AGVState], obstacles=()) -> bool:
    test = AGVState(aid=agv.aid, x=agv.x, y=agv.y, theta=agv.theta,
                    direction=agv.direction)
    integrate(test, v, omega, dt)
    test_corners = corners(test)
    for b in others:
        if b is agv or b.arrived:
            continue
        if _sat_overlap(test_corners, corners(b)):
            return True
    for o in obstacles:
        if _sat_overlap(test_corners, o.corners()):
            return True
    return False


def safe_motion(agv: AGVState, v_cmd: float, omega_cmd: float, dt: float,
                others: List[AGVState], obstacles=(),
                wall_limit: float = WALL_LIMIT) -> None:
    """Move `agv` safely with collision bisection."""
    if not _would_collide(agv, v_cmd, omega_cmd, dt, others, obstacles):
        integrate(agv, v_cmd, omega_cmd, dt, wall_limit)
        return
    lo, hi = 0.0, v_cmd
    for _ in range(8):
        mid = 0.5 * (lo + hi)
        if _would_collide(agv, mid, omega_cmd, dt, others, obstacles):
            hi = mid
        else:
            lo = mid
    if lo > 0.05:
        integrate(agv, lo, omega_cmd, dt, wall_limit)
        return
    if not _would_collide(agv, 0.0, omega_cmd, dt, others, obstacles):
        integrate(agv, 0.0, omega_cmd, dt, wall_limit)
    else:
        integrate(agv, 0.0, 0.0, dt, wall_limit)


# ===========================================================================
# Visual safety guard — ensures no visual overlap in recorded trajectory
# ===========================================================================
def obb_corners(x, y, heading):
    """Return OBB corners for a given pose."""
    c, s = math.cos(heading), math.sin(heading)
    l2, w2 = AGV_LENGTH / 2.0, AGV_WIDTH / 2.0
    local = ((l2, w2), (l2, -w2), (-l2, -w2), (-l2, w2))
    return [(x + lx * c - ly * s, y + lx * s + ly * c) for lx, ly in local]


def sat_overlap(c1, c2, tol: float = 0.0) -> bool:
    """SAT overlap check with tolerance."""
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
            if max(p1) < min(p2) + tol or max(p2) < min(p1) + tol:
                return False
    return True


def _visual_safety_guard(agvs: List[AGVState]) -> None:
    """After each physical step, reduce heading of any overlapping AGV pair
    until they no longer visually intersect (1 mm clearance)."""
    import itertools
    live = [a for a in agvs if not a.arrived]
    for _sweep in range(10):
        any_fix = False
        for a, b in itertools.combinations(live, 2):
            ca = obb_corners(a.x, a.y, a.heading if hasattr(a, 'heading') else a.theta)
            cb = obb_corners(b.x, b.y, b.heading if hasattr(b, 'heading') else b.theta)
            if not sat_overlap(ca, cb, tol=-1e-3):
                continue
            any_fix = True
            # reduce b's heading toward its base
            theta_b = b.theta
            base_b = 0.0 if b.direction > 0 else math.pi
            steer = math.atan2(math.sin(theta_b - base_b),
                               math.cos(theta_b - base_b))
            if abs(steer) >= 1e-9:
                sign = math.copysign(1.0, steer)
                lo2, hi2 = 0.0, abs(steer)
                for _ in range(12):
                    mid = 0.5 * (lo2 + hi2)
                    if sat_overlap(obb_corners(b.x, b.y, base_b + sign * mid),
                                   ca, tol=-1e-3):
                        hi2 = mid
                    else:
                        lo2 = mid
                b.theta = base_b + sign * lo2
        if not any_fix:
            break


# ===========================================================================
# Pairwise minimum OBB clearance — ROS2-READY
# ===========================================================================
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
# Main loop — ROS2-REPLACE
# ===========================================================================
def run_scenario(name: str, algorithm: str, seed: int,
                 record_trajectory: bool = False
                 ) -> Tuple[List[AGVState], Optional[List], float,
                            dict, float, float, list, list]:
    agvs = make_scenario(name, seed)
    obstacles = scenario_obstacles(name)
    wall_limit = WALL_LIMIT
    pair_times: dict = {}
    deadlock_events: list = []
    trajectory: list = []
    min_clearance = math.inf
    t = 0.0
    use_lane = name in SCENARIO_C_CONFIGS

    while t < MAX_SIM_TIME:
        if use_lane:
            if algorithm == "DLTO":
                step_dlto_lane(agvs, obstacles, name, t,
                               pair_times, deadlock_events)
            else:
                step_sw_lane(agvs, obstacles, name, t,
                             pair_times, deadlock_events)
        else:
            if algorithm == "DLTO":
                step_dlto(agvs, t, pair_times)
            else:
                step_sw(agvs, t, pair_times)

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

        for a in agvs:
            if a.arrived:
                a.v = 0.0
                continue
            if a.state == "WAIT":
                v_cmd, omega_cmd = 0.0, 0.0
            elif a.state == "SLOW":
                v_cmd = SLOW_SPEED
                omega_cmd = steering_omega(a)
            else:
                if abs(a.target_y - a.y) > 0.02:
                    v_cmd = OFFSET_SPEED
                else:
                    v_cmd = MAX_SPEED
                omega_cmd = steering_omega(a)
            safe_motion(a, v_cmd, omega_cmd, DT, agvs, obstacles, wall_limit)
            _visual_safety_guard(agvs)

            if has_arrived(a):
                a.arrived = True
                a.arrival_time = t

        live = [a for a in agvs if not a.arrived]
        for i in range(len(live)):
            for j in range(i + 1, len(live)):
                d = pairwise_min_obb_distance(corners(live[i]), corners(live[j]))
                if d < min_clearance:
                    min_clearance = d

        if record_trajectory:
            trajectory.append((t, [(a.x, a.y, a.theta, a.state, a.arrived)
                                   for a in agvs]))

        if all(a.arrived for a in agvs):
            break
        t += DT

    makespan = max((a.arrival_time for a in agvs if a.arrived), default=t)
    total_dist = sum(a.distance_travelled for a in agvs)
    if math.isinf(min_clearance):
        min_clearance = HALF_W

    return (agvs,
            trajectory if record_trajectory else None,
            makespan,
            pair_times,
            min_clearance,
            total_dist,
            obstacles,
            deadlock_events)


