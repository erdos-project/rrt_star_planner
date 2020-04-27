from ctypes import c_double, c_int, POINTER, Structure, CDLL

_c_double_p = POINTER(c_double)

MAX_PATH_LENGTH = 1000

class RRTStarInitialConditions(Structure):
    _fields_ = [
        ("x_start", c_double),
        ("y_start", c_double),
        ("x_end", c_double),
        ("y_end", c_double),
        ("o_llx", _c_double_p),
        ("o_lly", _c_double_p),
        ("o_urx", _c_double_p),
        ("o_ury", _c_double_p),
        ("no", c_int)
    ]

class RRTStarReturnValues(Structure):
    _fields_ = [
        ("success", c_int),
        ("x_path", c_double * MAX_PATH_LENGTH),
        ("y_path", c_double * MAX_PATH_LENGTH)
    ]

class RRTStarHyperparameters(Structure):
    _fields_ = [
        ("step_size", c_double),
        ("max_iterations", c_double),
        ("end_dist_threshold", c_double),
        ("obstacle_clearance", c_double),
        ("lane_width", c_double)
    ]