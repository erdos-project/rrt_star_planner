import ctypes
import numpy as np
from ctypes import c_double, c_int

cdll = ctypes.CDLL("build/libRRTStar.so")
_c_double_p = ctypes.POINTER(c_double)
_apply_rrt_star = cdll.ApplyRRTStar
_apply_rrt_star.argtypes = (
    c_double, c_double, c_double, c_double, c_int, c_int, _c_double_p,
    _c_double_p, _c_double_p, _c_double_p, c_int, _c_double_p, _c_double_p
)
_apply_rrt_star.restype = c_int


def apply_rrt_star(start, end, step_size, max_iterations, obs):
    """ Run RRTStar given a start, end and list of obstacles.

    Args:
        start (np.ndarray): [x, y]
        end (np.ndarray): [x, y]
        step_size (float): distance to sample new points
        max_iterations (int): maximum number of iterations to run
        obs (np.ndarray): [llx, lly, urx, ury]

    Returns:
        result_x (list(float)): x positions of rrt* path, if it exists
        result_y (list(float)): y positions of rrt* path, if it exists
        success (bool): whether rrt* found a path or not
    """
    result_x = np.zeros(100)
    result_y = np.zeros(100)
    success = _apply_rrt_star(
        c_double(start[0]), c_double(start[1]), c_double(end[0]),
        c_double(end[1]), c_int(step_size), c_int(max_iterations),
        obs[:, 0].ctypes.data_as(_c_double_p),
        obs[:, 1].ctypes.data_as(_c_double_p),
        obs[:, 2].ctypes.data_as(_c_double_p),
        obs[:, 3].ctypes.data_as(_c_double_p),
        c_int(obs.shape[0]),
        result_x.ctypes.data_as(_c_double_p),
        result_y.ctypes.data_as(_c_double_p),
    )

    ind = -1
    if success and np.any(np.isnan(result_x)):
        ind = np.where(np.isnan(result_x))[0][0]

    return result_x[:ind], result_y[:ind], success