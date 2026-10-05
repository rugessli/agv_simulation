from __future__ import annotations
import math
from dataclasses import dataclass, field

from config import (
    AGV_LENGTH, AGV_WIDTH, CRUISE_SPEED, APPROACH_SPEED, OFFSET_SPEED, CREEP_SPEED,
    WALL_CLEARANCE, CORRIDOR_LENGTH, AGV_MAX_PAYLOAD, AGV_MIN_TURN_R, DT,
)

# Steering gains: KP softened (was 2.0) so a large lateral error no longer
# slams the heading over — the offset develops as a gentle arc instead.
KP_STEER  = 1.4
KH_HEAD   = 2.2
OMEGA_MAX = 0.4

_BODY_R   = math.hypot(AGV_LENGTH / 2.0, AGV_WIDTH / 2.0)   # half-diagonal
_BODY_PHI = math.atan2(AGV_WIDTH / 2.0, AGV_LENGTH / 2.0)   # diagonal angle


def theta_safe(y_abs: float, corridor_half_w: float) -> float:
    """Largest |heading| (rad) for which the rotated body still fits at lateral |y|.

    Body half-extent in y at heading θ is (L/2)|sinθ| + (W/2)|cosθ|
    = _BODY_R · sin(|θ| + _BODY_PHI).  Requiring that plus |y| to stay inside
    (corridor_half_w − WALL_CLEARANCE) bounds |θ|.
    """
    C = corridor_half_w - WALL_CLEARANCE - y_abs
    if C >= _BODY_R:
        return math.pi / 2.0          # any heading fits
    if C <= AGV_WIDTH / 2.0:
        return 0.0                    # already at the limit even when straight
    return max(0.0, math.asin(C / _BODY_R) - _BODY_PHI)


@dataclass
class AGVState:
    aid:          int
    x:            float
    y:            float
    theta:        float
    direction:    int
    corridor_length: float = CORRIDOR_LENGTH
    v:            float = 0.0
    omega:        float = 0.0
    target_y:     float = 0.0
    state:        str   = "RUN"
    arrived:      bool  = False
    arrival_time: float = math.inf
    detect_time:  float = -1.0
    pass_time:    float = -1.0
    distance_travelled: float = 0.0
    cargo_state:  bool  = False
    payload_mass: float = 0.0
    departure_delay: float = 0.0
    speed_factor:    float = 1.0
    smooth_target_y: float = 0.0

    @property
    def goal_x(self) -> float:
        return (self.corridor_length - 0.5) if self.direction > 0 else 0.5

    @property
    def remaining_dist(self) -> float:
        return abs(self.x - self.goal_x)


def steering_omega(agv: AGVState, v: float = None,
                   corridor_half_w: float = None,
                   y_target: float = None, dt: float = DT) -> float:
    """Heading-rate command for the unicycle controller.

    Three layers, applied in order:
      1. P-control on lateral error + heading error.
      2. Minimum-turn-radius clamp: |ω| ≤ v / R_min (manufacturer spec 2.6 m),
         so the path is always a physically drivable arc.
      3. Corridor tilt-safety clamp: the predicted heading is never allowed to
         exceed theta_safe(|y|), guaranteeing all four corners stay inside the
         walls.  Straightening (reducing |θ|) is always permitted.
    """
    tgt = agv.target_y if y_target is None else y_target
    base = 0.0 if agv.direction > 0 else math.pi
    # Heading deviation from the corridor axis (signed, in (-pi, pi]).
    dev = (agv.theta - base + math.pi) % (2.0 * math.pi) - math.pi
    head_err = -dev
    lateral = KP_STEER * (tgt - agv.y) * math.cos(agv.theta)
    raw = lateral + KH_HEAD * head_err

    # Layer 2: curvature limit from minimum turning radius.
    if v is not None and abs(v) > 1e-6:
        lim = min(OMEGA_MAX, abs(v) / AGV_MIN_TURN_R)
    else:
        lim = OMEGA_MAX
    omega = max(-lim, min(lim, raw))

    # Layer 3: corridor tilt safety — bound the heading *deviation* so the
    # rotated body keeps all four corners inside the walls.
    if corridor_half_w is not None:
        th_max = theta_safe(abs(agv.y), corridor_half_w)
        dev_next = dev + omega * dt
        if dev_next > th_max:
            omega = (th_max - dev) / dt
        elif dev_next < -th_max:
            omega = (-th_max - dev) / dt
        # The safety correction may need more authority than the curvature limit
        # to straighten in time, but never more than the hardware maximum.
        omega = max(-OMEGA_MAX, min(OMEGA_MAX, omega))
    return omega


def integrate(agv: AGVState, v_cmd: float, omega_cmd: float, dt: float,
              corridor_half_w: float = None) -> None:
    agv.v     = v_cmd
    agv.omega = omega_cmd
    agv.x    += v_cmd * math.cos(agv.theta) * dt
    agv.y    += v_cmd * math.sin(agv.theta) * dt
    agv.theta += omega_cmd * dt

    if corridor_half_w is None:
        corridor_half_w = AGV_WIDTH  # safe fallback

    body_half_y = (AGV_LENGTH / 2.0) * abs(math.sin(agv.theta)) \
                + (AGV_WIDTH  / 2.0) * abs(math.cos(agv.theta))
    # Include WALL_CLEARANCE so all four rotated corners stay inside the corridor
    dynamic_limit = max(0.0, corridor_half_w - body_half_y - WALL_CLEARANCE)
    agv.y = max(-dynamic_limit, min(dynamic_limit, agv.y))
    agv.distance_travelled += abs(v_cmd) * dt


def has_arrived(agv: AGVState) -> bool:
    return (agv.direction > 0 and agv.x >= agv.goal_x) \
        or (agv.direction < 0 and agv.x <= agv.goal_x)


def corners(agv: AGVState):
    c, s = math.cos(agv.theta), math.sin(agv.theta)
    l2, w2 = AGV_LENGTH / 2.0, AGV_WIDTH / 2.0
    local = ((l2, w2), (l2, -w2), (-l2, -w2), (-l2, w2))
    return [(agv.x + lx * c - ly * s,
             agv.y + lx * s + ly * c) for lx, ly in local]


@dataclass
class Obstacle:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    @property
    def cx(self):     return 0.5 * (self.x_min + self.x_max)
    @property
    def cy(self):     return 0.5 * (self.y_min + self.y_max)
    @property
    def length(self): return self.x_max - self.x_min
    @property
    def width(self):  return self.y_max - self.y_min

    def corners(self):
        return [(self.x_min, self.y_min), (self.x_max, self.y_min),
                (self.x_max, self.y_max), (self.x_min, self.y_max)]

    def blocks_lane(self, lane_y: float, agv_half_w: float = AGV_WIDTH / 2.0) -> bool:
        agv_bot = lane_y - agv_half_w
        agv_top = lane_y + agv_half_w
        return not (agv_top < self.y_min or agv_bot > self.y_max)
