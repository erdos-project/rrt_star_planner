import rrt_star_wrapper
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch


def main():
    """A debug script for the rrt star planner.

    This script will solve the rrt star problem in a
    standalone simulation and visualize the results or raise an error if a
    path is not found.
    """
    print(__file__ + " start!!")
    sim_loop = 100
    area = 20.0  # animation area length [m]
    show_animation = True

    conds = {
        'start': [150, 60],
         'end': [170, 60],
        'obstacles': [
            [158, 57,
             162, 63]
        ],
        'step_size': 0.5,
        'max_iterations': 2000
    }  # paste output from debug log

    start = conds['start']
    end = conds['end']
    max_iterations = conds['max_iterations']
    step_size = conds['step_size']
    obs = np.array(conds['obstacles'])

    total_time_taken = 0
    x, y = start
    for i in range(sim_loop):
        print("Iteration: {}".format(i))
        start_time = time.time()
        success, (result_x, result_y) = \
            rrt_star_wrapper.apply_rrt_star([x, y], end, step_size,
                                            max_iterations, obs)
        if success == 1:
            x = result_x[1]
            y = result_y[1]
        else:
            print("Failed")
        end_time = time.time() - start_time
        total_time_taken += end_time
        print("Time taken: {}".format(end_time))

        if np.hypot(x - end[0], y - end[1]) <= 1.0:
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None]
            )
            ax = plt.gca()
            for o in obs:
                rect = patch.Rectangle((o[0], o[1]),
                                       o[2] - o[0],
                                       o[3] - o[1])
                ax.add_patch(rect)
            plt.plot(start[0], start[1], "og")
            plt.plot(end[0], end[1], "or")
            if success:
                plt.plot(result_x[1:], result_y[1:], ".r")
                plt.plot(result_x[1], result_y[1], "vc")
                plt.xlim(result_x[1] - area, result_x[1] + area)
                plt.ylim(result_y[1] - area, result_y[1] + area)
            plt.xlabel("X axis")
            plt.ylabel("Y axis")
            plt.grid(True)
            plt.pause(0.1)

    print("Finish")
    print("Average time per iteration: {}".format(total_time_taken/i))
    if show_animation:  # pragma: no cover
        plt.grid(True)
        plt.pause(1)
        plt.show()


if __name__ == '__main__':
    main()
