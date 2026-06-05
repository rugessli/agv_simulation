AGV_LENGTH       = 2.35
AGV_WIDTH        = 1.10
AGV_MAX_PAYLOAD  = 1500.0
CRUISE_SPEED     = 1.5    # no oncoming vehicle
APPROACH_SPEED   = 0.8    # oncoming detected (dist <= DLTO_PLAN_RANGE)
OFFSET_SPEED     = 0.3    # executing lateral offset (dist <= DETECT_RANGE)
CREEP_SPEED      = 0.1    # extremely close (dist <= D_CLEAR)
DETECT_RANGE     = 3.0
DLTO_PLAN_RANGE  = 7.0       # planning lookahead; larger than DETECT_RANGE to complete lateral offset before bodies close
SAFETY_MARGIN    = 0.15
AGV_MIN_TURN_R   = 2.6         # minimum turning radius (m), from manufacturer spec
WALL_CLEARANCE   = 0.05
D_MIN            = 0.25
D_CLEAR          = 0.5
D_LOOKAHEAD      = 1.5
T_MAX            = 60.0
T_DEAD           = 2.0
DT               = 0.1
CORRIDOR_LENGTH  = 40.0
PRIORITY_ALPHA   = 0.40   # cargo state
PRIORITY_BETA    = 0.20   # payload mass
PRIORITY_GAMMA   = 0.20   # proximity to goal (less disruption near destination)
PRIORITY_DELTA   = 0.15   # current speed (higher cost to reroute a faster vehicle)
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
