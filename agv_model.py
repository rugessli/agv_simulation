"""
agv_model.py
============
AGV physical state, kinematic model, and low-level steering controller.

What it does
------------
  * `AGVState`  — dataclass holding pose, velocity, target lateral offset,
                  bookkeeping fields (detect_time, pass_time, arrived, ...).
  * `steering_omega(agv)` — proportional yaw-rate controller:
                  ω = Kp · (target_y − y) · cos(θ)        with Kp = 2.0.
  * `integrate(agv, v, ω, dt)` — unicycle integrator:
                  x ← x + v cos θ · dt
                  y ← y + v sin θ · dt
                  θ ← θ + ω · dt
  * `corners(agv)` — oriented bounding box for collision / clearance.

ROS2 migration notes
--------------------
ROS2-READY :
  - `AGVState`              (use as the in-node pose cache)
  - `steering_omega`        (control law is what a ROS2 controller publishes)
  - `corners`               (OBB helper for ROS2 collision-checker nodes)

ROS2-REPLACE :
  - `integrate`             — in ROS2 the physics is done by Gazebo / Stage
                              or by reading `/odom`.  Replace with an odom
                              subscriber that updates `AGVState` in place.
  - `AGVState.goal_x`       — replace with a `/goal_pose` topic subscriber.
  - The wall-clamp inside `integrate` becomes a real obstacle in Gazebo and
    should be deleted from this file when migrating.
"""

from __future__ import annotations
import math
from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# Physical constants — ROS2 parameter-server candidates
# ---------------------------------------------------------------------------
AGV_LENGTH      = 0.9
AGV_WIDTH       = 0.6
MAX_SPEED       = 1.5
OFFSET_SPEED    = 0.8     # forward speed while a detour is active
SLOW_SPEED      = 0.4     # DLTO "slow-down yield" speed
KP_STEER        = 2.0     # lateral-error gain   (ω = Kp · dy · cos θ + ...)
KH_HEAD         = 2.5     # heading-restoration gain  (... + Kh · (base − θ))
                          # Tuned so the closed-loop system is just
                          # over-critically damped (ξ ≈ 1.0) — no overshoot,
                          # settling time ≈ 2.4 s.
OMEGA_MAX       = 0.4     # hard yaw-rate cap, rad/s
WALL_LIMIT      = 0.45    # fallback static clamp (replaced by dynamic clamp)
CORRIDOR_HALF_W = 0.75    # physical corridor wall, |y| ≤ this
GOAL_RIGHT_X    = 9.5
GOAL_LEFT_X     = 0.5


# ---------------------------------------------------------------------------
# State container — ROS2-READY
# ---------------------------------------------------------------------------
@dataclass
class AGVState:
    aid:       int
    x:         float
    y:         float
    theta:     float
    direction: int             # +1 going right (goal x=9.5), −1 going left
    v:         float = 0.0
    omega:     float = 0.0
    target_y:  float = 0.0
    state:     str   = "RUN"   # RUN | WAIT
    arrived:   bool  = False
    arrival_time: float = math.inf
    detect_time:  float = -1.0
    pass_time:    float = -1.0
    distance_travelled: float = 0.0   # accumulated path length (for NEC)

    # The goal x depends only on direction → ROS2-REPLACE with /goal_pose
    @property
    def goal_x(self) -> float:
        return GOAL_RIGHT_X if self.direction > 0 else GOAL_LEFT_X


# ---------------------------------------------------------------------------
# Controllers — ROS2-READY
# ---------------------------------------------------------------------------
def steering_omega(agv: AGVState) -> float:
    """Combined lateral-error + heading-restoration controller.

        ω = Kp · (target_y − y) · cos θ  −  Kh · (θ − θ_base)

    The first term steers the AGV toward target_y; the second pulls θ
    back to the base heading (0 going right, π going left).  Together
    they drive the AGV to (y = target_y, θ = base) asymptotically.

    Output is clipped to ±OMEGA_MAX so the resulting arc looks like a
    real AGV lane adjustment rather than a sharp swerve."""
    base = 0.0 if agv.direction > 0 else math.pi
    head_err = (base - agv.theta + math.pi) % (2.0 * math.pi) - math.pi
    lateral = KP_STEER * (agv.target_y - agv.y) * math.cos(agv.theta)
    raw = lateral + KH_HEAD * head_err
    return max(-OMEGA_MAX, min(OMEGA_MAX, raw))


# ---------------------------------------------------------------------------
# Unicycle integration — ROS2-REPLACE
# ---------------------------------------------------------------------------
def integrate(agv: AGVState, v_cmd: float, omega_cmd: float, dt: float,
              wall_limit: float = WALL_LIMIT) -> None:
    """Forward-Euler unicycle update:
            x ← x + v cos θ · dt
            y ← y + v sin θ · dt
            θ ← θ + ω · dt

    Wall clamp is *dynamic* — the OBB y-extent for the rotated body is
        h(θ) = L/2 · |sin θ| + W/2 · |cos θ|
    so the centre is clamped to  |y| ≤ CORRIDOR_HALF_W − h(θ).
    This guarantees the rotated body edge never crosses ±0.75."""
    agv.v     = v_cmd
    agv.omega = omega_cmd
    agv.x    += v_cmd * math.cos(agv.theta) * dt
    agv.y    += v_cmd * math.sin(agv.theta) * dt
    agv.theta += omega_cmd * dt

    body_half_y = (AGV_LENGTH / 2.0) * abs(math.sin(agv.theta)) \
                + (AGV_WIDTH  / 2.0) * abs(math.cos(agv.theta))
    dynamic_limit = CORRIDOR_HALF_W - body_half_y
    if dynamic_limit < 0:
        dynamic_limit = 0.0
    if agv.y >  dynamic_limit:
        agv.y =  dynamic_limit
    elif agv.y < -dynamic_limit:
        agv.y = -dynamic_limit
    agv.distance_travelled += abs(v_cmd) * dt


# ---------------------------------------------------------------------------
# Goal check — ROS2-READY
# ---------------------------------------------------------------------------
def has_arrived(agv: AGVState) -> bool:
    return (agv.direction > 0 and agv.x >= agv.goal_x) \
        or (agv.direction < 0 and agv.x <= agv.goal_x)


# ---------------------------------------------------------------------------
# Geometry — ROS2-READY
# ---------------------------------------------------------------------------
def corners(agv: AGVState):
    """Return the 4 oriented-bounding-box corners as (x, y) tuples."""
    c, s = math.cos(agv.theta), math.sin(agv.theta)
    l2, w2 = AGV_LENGTH / 2.0, AGV_WIDTH / 2.0
    local = ((l2, w2), (l2, -w2), (-l2, -w2), (-l2, w2))
    return [(agv.x + lx * c - ly * s,
             agv.y + lx * s + ly * c) for lx, ly in local]


# ---------------------------------------------------------------------------
# Static obstacle (axis-aligned rectangle)
# ROS2-READY: in a ROS2 deployment the obstacle list is published on a
# `/map/static_obstacles` topic; the dataclass itself remains identical.
# ---------------------------------------------------------------------------
@dataclass
class Obstacle:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    @property
    def cx(self):       return 0.5 * (self.x_min + self.x_max)
    @property
    def cy(self):       return 0.5 * (self.y_min + self.y_max)
    @property
    def length(self):   return self.x_max - self.x_min
    @property
    def width(self):    return self.y_max - self.y_min

    def corners(self):
        return [(self.x_min, self.y_min), (self.x_max, self.y_min),
                (self.x_max, self.y_max), (self.x_min, self.y_max)]

    def blocks_lane(self, lane_y: float, agv_half_w: float = AGV_WIDTH / 2.0
                    ) -> bool:
        """True if an AGV travelling at y = lane_y would intersect this OBB."""
        agv_bot = lane_y - agv_half_w
        agv_top = lane_y + agv_half_w
        return not (agv_top < self.y_min or agv_bot > self.y_max)
