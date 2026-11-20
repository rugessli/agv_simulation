AGV_LENGTH       = 2.35
AGV_WIDTH        = 1.10
AGV_MAX_PAYLOAD  = 1500.0
CRUISE_SPEED     = 1.5    # no oncoming vehicle
APPROACH_SPEED   = 0.8    # oncoming detected (dist <= DLTO_PLAN_RANGE)
OFFSET_SPEED     = 0.3    # executing lateral offset (dist <= DETECT_RANGE)
CREEP_SPEED      = 0.1    # extremely close (dist <= D_CLEAR)
DETECT_RANGE     = 3.0
DLTO_PLAN_RANGE  = 11.0      # offset-planning lookahead. With min turn radius 2.6 m,
                            # an S-curve to clear ~0.7 m laterally needs ~2.7 m of
                            # forward travel; detecting the oncoming vehicle 11 m
                            # apart leaves each ~4.3 m, so the offset can be completed
                            # (and the body straightened) well before the bodies meet.
SAFETY_MARGIN    = 0.15
AGV_MIN_TURN_R   = 2.6         # minimum turning radius (m), from manufacturer spec
WALL_CLEARANCE   = 0.05
D_MIN            = 0.25        # AGV↔AGV dynamic safety gap (head-on passing)
D_OBSTACLE       = 0.15        # AGV↔static-obstacle lateral safety clearance —
                              # independent of the AGV↔AGV gap D_MIN (fix 2).
D_CLEAR          = 0.5

# --- Trajectory smoothing / physical constraints (Problem 2) ---
OFFSET_RATE      = 0.35        # max lateral-command rate (m/s); rate-limits the
                              # target so the offset develops as a smooth arc
                              # instead of a step input.
TILT_RESERVE     = 0.07        # lateral headroom (m) reserved at the offset lane so
                              # the rotated body never reaches the wall during the arc.
SIM_GAP_MARGIN   = 0.25        # extra gap width (beyond AGV_WIDTH) required before two
                              # opposite-direction AGVs may pass an obstacle simultaneously
D_OBS_STOP       = 0.30        # longitudinal command threshold for stopping in front
                              # of an impassable obstacle; leaves a measured body gap
                              # comfortably above D_OBSTACLE (fix 3)
D_LOOKAHEAD      = 1.5
T_MAX            = 120.0   # serialised obstacle passing (one vehicle waits for the
                          # other to clear a single-file bottleneck) legitimately
                          # needs more wall-clock than an open-corridor encounter.
T_DEAD           = 3.0   # deadlock confirmation time (Section 3.2); not used in the metrics
DT               = 0.1
CORRIDOR_LENGTH  = 40.0
PRIORITY_ALPHA   = 0.50   # cargo state
PRIORITY_BETA    = 0.25   # payload mass
PRIORITY_GAMMA   = 0.20   # proximity to goal
PRIORITY_EPSILON = 0.05   # ID tie-breaker

CORRIDOR_WIDTHS  = [2.8, 3.2, 3.6]
AGV_COUNTS       = [2, 3, 4]
PAYLOAD_PROFILES = ["unloaded", "mixed", "loaded"]
ALGORITHMS       = ["DLTO", "SW", "ORCA", "FLC_ORCA"]
N_REPLICATIONS   = 30

# Obstacle experiment: 3 positions × 3 widths × 4 algorithms × 30 seeds = 1,080 runs
OBSTACLE_PROFILES = ["obstacle_top", "obstacle_mid", "obstacle_bot"]
OBSTACLE_N_AGV    = [2]
OBSTACLE_SEEDS    = 30

# Wait-for-offset release tolerance (m): an AGV already in its offset lane holds
# position until the oncoming partner's lateral error falls below this value.
# Tightened from 0.25 m so that passing never starts while the partner is still
# mid-manoeuvre with a tilted body (observed MCD < D_MIN at 2.8 m, n_agv = 4).
WAIT_OFFSET_TOL  = 0.05


def hold_distance(partner_err: float) -> float:
    """Centre-to-centre distance at which an AGV already in its offset lane must
    hold while the oncoming partner completes its offset.  Physical basis: the
    partner needs about 2*sqrt(R_min * remaining_error) of forward travel to finish
    its lateral shift along arcs of the minimum turning radius, plus one body
    length so the two bodies have not yet overlapped longitudinally, plus 0.5 m.
    Never smaller than the original DETECT_RANGE trigger."""
    import math
    need = AGV_LENGTH + 2.0 * math.sqrt(AGV_MIN_TURN_R * max(partner_err, 0.0)) + 0.5
    return max(DETECT_RANGE, need)
