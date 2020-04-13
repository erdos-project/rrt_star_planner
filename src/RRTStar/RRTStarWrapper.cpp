#include "src/RRTStar/rrt_star.h"
#include "src/RRTStar/py_cpp_struct.h"

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <map>

double SPACEDIM = 2.0;
double PI = 3.14159;

extern "C" {
    Node* getRandomNodeWithinRadius(RRT *rrt) {
        Node *q_new = rrt->getRandomNode();
        Node *q_nearest = rrt->nearest(q_new->position);

        if (RRT::distance(q_new->position, q_nearest->position) >
            rrt->step_size) {
            Vector2f newConfig = rrt->newConfig(q_new, q_nearest);
            if (rrt->isSegmentInObstacles(newConfig, q_nearest->position)) {
                return nullptr;
            }
            q_new = new Node(newConfig);
        }
        return q_new;
    }

    Node* getShortestPathToNode(RRT *rrt, Node *q_new, Node *q_best,
        double dist_best, double radius, map<Node *, double> &cost_map)
    {
        double dist, cost;
        for (auto node : rrt->nodes) {
            // compute the cost for q_new through node
            dist = RRT::distance(q_new->position, node->position);
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
        return q_best;
    }

    void rewireTree(RRT *rrt, Node *q_new, double radius, map<Node *, double>
            &cost_map) {
        double dist, cost;
        for (auto node : rrt->nodes) {
            dist = RRT::distance(q_new->position, node->position);
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
                RRT::relink(node, q_new, dist);
                cost_map[node] = cost;
            }
        }
    }


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
    void ApplyRRTStar(RRTStarInitialConditions *rrts_ic,
        RRTStarHyperparameters *rrts_hp, RRTStarReturnValues *rrts_rv)
    {
        RRT* rrt = new RRT(rrts_ic, rrts_hp);

        // Declare variables
        Node *q_best, *q_new, *q_nearest;
        double dist_best, gamma, radius;

        // Cost to each vertex
        map<Node *, double> cost_map;
        cost_map[rrt->root] = 0;

        // Run RRT*
        gamma = 1 + pow(2, SPACEDIM) * (1 + 1 / SPACEDIM) *
                rrt->getFreeArea();

        for (int i = 0; i < rrt->max_iter; i++) {
            // get a random node within the step_size radius of existing node
            q_new = getRandomNodeWithinRadius(rrt);

            if (q_new == nullptr) continue;

            q_nearest = rrt->nearest(q_new->position);

            // find nearest neighbors within radius of new node
            rrt->nearestNeighbors(q_new->position);
            q_best = q_nearest;
            dist_best = RRT::distance(q_new->position, q_nearest->position);
            radius = min(
                pow(gamma / PI * log(i + 1) / (i + 1), 1 / SPACEDIM),
                (double) rrt->step_size
            );

            // look for shortest cost path to q_new
            q_best = getShortestPathToNode(rrt, q_new, q_best, dist_best,
                    radius, cost_map);
            dist_best = RRT::distance(q_new->position, q_best->position);

            // if no collision, add to graph and update cost to new node
            if (rrt->isSegmentInObstacles(q_new->position, q_best->position)) {
                continue;
            }
            rrt->add(q_best, q_new, dist_best);
            cost_map[q_new] = cost_map[q_best] + dist_best;

            // rewire the tree
            rewireTree(rrt, q_new, radius, cost_map);

            // early exit
            if (rrt->reached()) {
                rrts_rv->success = 1;
                break;
            }
        }
        Node *q;

        if (rrts_rv->success) {
            q = rrt->last_node;
        }
        else
        {
            // if not reached yet, then shortestPath will start from the closest
            // node to end point.
            q = rrt->nearest(rrt->end_pos);
        }
        // generate shortest path to destination.
        while (q != nullptr) {
            rrt->path.push_back(q);
            q = q->parent;
        }

        // return the RRT* path
        int index = 0;
        reverse(rrt->path.begin(), rrt->path.end());
        for (auto node : rrt->path) {
            rrts_rv->x_path[index] = node->position[0];
            rrts_rv->y_path[index] = node->position[1];
            index += 1;
        }
        rrts_rv->x_path[index] = NAN;
        rrts_rv->y_path[index] = NAN;

        // free the memory
        delete rrt;
    }
}