#ifndef RRTSTAR_PY_CPP_STRUCT_H
#define RRTSTAR_PY_CPP_STRUCT_H
const int MAX_PATH_LENGTH = 100;

struct RRTStarInitialConditions {
    double x_start;
    double y_start;
    double x_end;
    double y_end;
    double *o_llx;
    double *o_lly;
    double *o_urx;
    double *o_ury;
    int no;
};

struct RRTStarReturnValues {
    int success;
    double x_path[MAX_PATH_LENGTH];
    double y_path[MAX_PATH_LENGTH];
};

struct RRTStarHyperparameters {
    double step_size;
    double max_iterations;
    double end_dist_threshold;
    double obstacle_clearance;
    double lane_width;
};
#endif //RRTSTAR_PY_CPP_STRUCT_H
