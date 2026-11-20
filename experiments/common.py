"""Shared helpers for the supplementary experiments (ablation and robustness).
The simulation code in the repository root is used unchanged."""
import sys, os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import config, dlto_algorithm


def set_weights(a, b, g, e):
    """Override the priority weights (alpha, beta, gamma, epsilon) at run time."""
    dlto_algorithm.PRIORITY_ALPHA, dlto_algorithm.PRIORITY_BETA = a, b
    dlto_algorithm.PRIORITY_GAMMA, dlto_algorithm.PRIORITY_EPSILON = g, e


def yield_stats(traj, agvs, w):
    """Mean of the maximum |y| reached by loaded and by unloaded vehicles in one run
    (only for runs that contain both kinds of vehicle)."""
    if not traj:
        return {}
    my = [0.0] * len(agvs)
    for _, st in traj:
        for k, s in enumerate(st):
            if not s[4]:
                my[k] = max(my[k], abs(s[1]))
    L = [my[k] for k, a in enumerate(agvs) if a.cargo_state]
    U = [my[k] for k, a in enumerate(agvs) if not a.cargo_state]
    if not L or not U:
        return {}
    return {"maxy_loaded": sum(L) / len(L), "maxy_unloaded": sum(U) / len(U)}
