from __future__ import annotations
import math
from dataclasses import dataclass, field

from config import (
    AGV_LENGTH, AGV_WIDTH, CRUISE_SPEED, APPROACH_SPEED, OFFSET_SPEED, CREEP_SPEED,
    WALL_CLEARANCE, CORRIDOR_LENGTH, AGV_MAX_PAYLOAD,
)

KP_STEER  = 2.0
KH_HEAD   = 2.5
OMEGA_MAX = 0.4


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


def steering_omega(agv: AGVState) -> float:
    base = 0.0 if agv.direction > 0 else math.pi
    head_err = (base - agv.theta + math.pi) % (2.0 * math.pi) - math.pi
    lateral = KP_STEER * (agv.target_y - agv.y) * math.cos(agv.theta)
    raw = lateral + KH_HEAD * head_err
    return max(-OMEGA_MAX, min(OMEGA_MAX, raw))


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
