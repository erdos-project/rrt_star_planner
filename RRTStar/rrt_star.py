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
    sim_loop = 50
    area = 20.0  # animation area length [m]
    show_animation = True

    conds = {'wx': [150.0],
             'wy': [195.0],
             'obstacle_list': [10 * [134., 194., 136., 196.]],
             'x': 120.0,
             'y': 195.0,
             }  # paste output from debug log

    # way points
    wx = np.array(conds['wx'])
    wy = np.array(conds['wy'])
    wp = np.array([wx, wy]).T

    # initial conditions
    x = conds['x']
    y = conds['y']

    # obstacle lists
    obs = np.array(conds['obstacle_list'])
    total_time_taken = 0
    for i in range(sim_loop):
        print("Iteration: {}".format(i))
        start_time = time.time()
        success, (result_x, result_y) = \
            rrt_star_wrapper.apply_rrt_star([x, y], wp[-1], 1,
                                           2000, obs)
        if success == 1:
            x = result_x[1]
            y = result_y[1]
        else:
            print("Failed")
        end_time = time.time() - start_time
        total_time_taken += end_time
        print("Time taken: {}".format(end_time))

        if np.hypot(result_x[1] - wx[-1], result_y[1] - wy[-1]) <= 2.0:
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
            rect = patch.Rectangle((obs[0, 0], obs[0, 1]),
                                   obs[0, 2] - obs[0, 0], obs[0, 3] - obs[0, 1])
            ax.add_patch(rect)
            plt.plot(wp[:, 0], wp[:, 1])
            plt.plot(result_x[1:], result_y[1:], ".r")
            plt.plot(result_x[1], result_y[1], "vc")
            plt.plot(wp[-1, 0], wp[-1, 1], "og")
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
