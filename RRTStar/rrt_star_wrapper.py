import numpy as np
import os

from ctypes import c_double, c_int, POINTER, Structure, CDLL, byref

try:
    from py_cpp_struct import RRTStarInitialConditions, RRTStarHyperparameters,\
        RRTStarReturnValues, MAX_PATH_LENGTH
except:
    from pylot.planning.rrt_star.rrt_star_planning.RRTStar.py_cpp_struct \
        import RRTStarInitialConditions, RRTStarHyperparameters, \
        RRTStarReturnValues, MAX_PATH_LENGTH

try:
    cdll = CDLL("build/libRRTStar.so")
except:
    cdll = CDLL(
        "{}/pylot/planning/rrt_star/rrt_star_planning/"
        "build/libRRTStar.so".format(os.getcwd())
    )
_c_double_p = POINTER(c_double)

# func / return type declarations for C++ ApplyRRTStar
_apply_rrt_star = cdll.ApplyRRTStar
_apply_rrt_star.argtypes = (
    POINTER(RRTStarInitialConditions),
    POINTER(RRTStarHyperparameters),
    POINTER(RRTStarReturnValues)
)
_apply_rrt_star.restype = None


def apply_rrt_star(initial_conditions, hyperparameters):
    """ Run RRTStar given initial conditions and hyperparameters.

    Args:
        initial_conditions (dict): dict containing the following items
            start (np.ndarray): [x, y]
            end (np.ndarray): [x, y]
            obs (np.ndarray): top-down obstacles formatted as follows
                [lower left x, lower left y, upper right x, upper right y]

        hyperparameters (dict): a dict of optional hyperparameters
            step_size (float): distance to sample new points [m]
            max_iterations (int): maximum number of iterations to run
            end_dist_threshold (float): completion threshold [m]
            obstacle_clearance (float): obstacle clearance threshold [m]
            lane_width (float): road width [m]

    Returns:
        x_path (np.ndarray(float)): x positions of RRT*, if it exists
        y_path (np.ndarray(float)): y positions of RRT*, if it exists
        success (bool): whether RRT* was successful
    """
    # convert initial conditions to rrtstar format
    rrts_ic = to_rrtstar_initial_conditions(initial_conditions)

    # parse hyperparameters
    rrts_hp = _parse_hyperparameters(hyperparameters)

    # intiialize return values
    rrts_rv = RRTStarReturnValues(0)

    _apply_rrt_star(rrts_ic, rrts_hp, rrts_rv)

    success = rrts_rv.success
    x_path = np.array([rrts_rv.x_path[i] for i in range(MAX_PATH_LENGTH)])
    y_path = np.array([rrts_rv.y_path[i] for i in range(MAX_PATH_LENGTH)])

    ind = -1
    if success and np.any(np.isnan(x_path)):
        ind = np.where(np.isnan(x_path))[0][0]

    return x_path[:ind], y_path[:ind], success

def to_rrtstar_initial_conditions(initial_conditions):
    x_start = initial_conditions['start'][0]
    y_start = initial_conditions['start'][1]
    x_end = initial_conditions['end'][0]
    y_end = initial_conditions['end'][1]
    obs = initial_conditions['obs']
    o_llx = np.copy(obs[:, 0]).astype(np.float64)
    o_lly = np.copy(obs[:, 1]).astype(np.float64)
    o_urx = np.copy(obs[:, 2]).astype(np.float64)
    o_ury = np.copy(obs[:, 3]).astype(np.float64)
    return RRTStarInitialConditions(
        x_start,
        y_start,
        x_end,
        y_end,
        o_llx.ctypes.data_as(_c_double_p), # obstacles lower left x
        o_lly.ctypes.data_as(_c_double_p), # obstacles lower left y
        o_urx.ctypes.data_as(_c_double_p), # obstacles upper right x
        o_ury.ctypes.data_as(_c_double_p), # obstacles upper right y
        len(obs)
    )

def _parse_hyperparameters(hp):
    return RRTStarHyperparameters(
        hp['step_size'],
        hp['max_iterations'],
        hp['end_dist_threshold'],
        hp['obstacle_clearance'],
        hp['lane_width'],
    )