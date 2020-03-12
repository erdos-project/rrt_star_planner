#include "src/RRTStar/rrt_star.h"

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <map>

double SPACEDIM = 2.0;
double PI = 3.14159;

extern "C" {
    // RRT* algorithm
    //
    // Arguments:
    //      x_start: starting x position
    //      y_start: starting y position
    //      x_end: ending x position
    //      y_end: ending y position
    //      step_size: sampling distance
    //      max_iterations: maximum number of iterations to take
    //      obstacles: list of obstacles formatted as
    //          [lower_left_x, lower_left_y, upper_right_x, upper_right_y]
    //      numObstacles: number of obstacles
    // Returns:
    //      x_path: list to store x path
    //      y_path: list to store y path
    int ApplyRRTStar(double x_start, double y_start, double x_end, double y_end,
                     double step_size, int max_iterations, double* obstacles_llx,
                     double* obstacles_lly, double* obstacles_urx, double* obstacles_ury,
                     int numObstacles, double* x_path, double* y_path)
    {
        RRT* rrt = new RRT(x_start, y_start, x_end, y_end, step_size,
                max_iterations);

        // Construct obstacles
        for (int i = 0; i < numObstacles; i++) {
            rrt->addObstacle(
                    Vector2f(obstacles_llx[i], obstacles_lly[i]),
                    Vector2f(obstacles_urx[i], obstacles_ury[i])
            );
        }

        // Declare variables
        Node *q_best, *q_new, *q_nearest;
        double cost, dist, dist_best, gamma, radius;
        int reached = 0;

        // Cost to each vertex
        map<Node *, double> cost_map;
        cost_map[rrt->root] = 0;

        // Run RRT*
        gamma = 1 + pow(2, SPACEDIM) * (1 + 1 / SPACEDIM) *
                rrt->getFreeArea();
        for (int i = 0; i < rrt->max_iter; i++) {
            q_new = rrt->getRandomNode();
            q_nearest = rrt->nearest(q_new->position);
            if (rrt->distance(q_new->position, q_nearest->position) >
                rrt->step_size) {
                Vector2f newConfig = rrt->newConfig(q_new, q_nearest);
                if (rrt->isSegmentInObstacles(newConfig, q_nearest->position)) {
                    continue;
                }
                q_new = new Node(newConfig);
            }

            rrt->nearestNeighbors(q_new->position);
            q_best = q_nearest;
            dist_best = rrt->distance(q_new->position, q_nearest->position);
            radius = min(
                pow(gamma / PI * log(i + 1) / (i + 1), 1 / SPACEDIM),
                (double) rrt->step_size
            );

            // look for shortest cost path to q_new
            for (auto node : rrt->nodes) {
                // compute the cost for q_new through node
                dist = rrt->distance(q_new->position, node->position);
                cost = cost_map[node] + dist;

                // only consider proximal nodes
                if (dist > radius) {
                    break;
                }

                // if cost less than current lowest cost, check potential link
                if (cost < cost_map[q_best] + dist_best) {
                    if (rrt->isSegmentInObstacles(q_new->position,
                                                  node->position)) {
                        continue;
                    }
                    q_best = node;
                    dist_best = dist;
                }
            }

            if (rrt->isSegmentInObstacles(q_new->position, q_best->position)) {
                continue;
            }
            rrt->add(q_best, q_new, dist_best);
            cost_map[q_new] = cost_map[q_best] + dist_best;

            // rewire the tree
            for (auto node : rrt->nodes) {
                dist = rrt->distance(q_new->position, node->position);
                cost = cost_map[node] + dist;

                // only consider proximal nodes
                if (dist > radius) {
                    break;
                }

                // if cost less than current lowest cost, check potential link
                if (cost < cost_map[node]) {
                    if (rrt->isSegmentInObstacles(q_new->position,
                                                  node->position)) {
                        continue;
                    }
                    rrt->relink(node, q_new, dist);
                    cost_map[node] = cost;
                }
            }
            // early exit
            if (rrt->reached()) {
                reached = 1;
                break;
            }
        }
        Node *q;

        if (reached) {
            q = rrt->last_node;
        }
        else
        {
            // if not reached yet, then shortestPath will start from the closest
            // node to end point.
            q = rrt->nearest(rrt->end_pos);
        }
        // generate shortest path to destination.
        while (q != NULL) {
            rrt->path.push_back(q);
            q = q->parent;
        }

        // return the RRT* path
        int index = 0;
        reverse(rrt->path.begin(), rrt->path.end());
        for (auto node : rrt->path) {
            x_path[index] = node->position[0];
            y_path[index] = node->position[1];
            index += 1;
        }
        x_path[index] = NAN;
        y_path[index] = NAN;

        // free the memory
        delete rrt;
        return reached;
    }
}