#include "src/RRTStar/rrt.h"

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <map>

double SPACEDIM = 2.0;
double PI = 3.14159;

extern "C" {
    // RRT* algorithm
    //
    // Arguments:
    //      xStart: starting x position
    //      yStart: starting y position
    //      xEnd: ending x position
    //      yEnd: ending y position
    //      stepSize: sampling distance
    //      maxIterations: maximum number of iterations to take
    //      obstacles: list of obstacles formatted as
    //          [lower_left_x, lower_left_y, upper_right_x, upper_right_y]
    //      numObstacles: number of obstacles
    // Returns:
    //      xPath: list to store x path
    //      yPath: list to store y path
    int ApplyRRTStar(double xStart, double yStart, double xEnd, double yEnd,
            int stepSize, int maxIterations, double* obstacles_llx,
            double* obstacles_lly, double* obstacles_urx, double* obstacles_ury,
            int numObstacles, double* xPath, double* yPath)
    {
        RRT* rrt = new RRT;

        // Initialize configuration
        rrt->reset();
        rrt->setStateSpace(xStart, yStart, xEnd, yEnd);
        rrt->setStepSize(stepSize);
        rrt->setMaxIterations(maxIterations);
        rrt->initialize();

        // Construct obstacles
        for (int i = 0; i < numObstacles; i++) {
            rrt->addObstacle(
                    Vector2f(obstacles_llx[i], obstacles_lly[i]),
                    Vector2f(obstacles_urx[i], obstacles_ury[i])
            );
        }

        // Declare variables
        Node *qBest, *qNew, *qNearest;
        double cost, dist, distBest, gamma, radius;
        int reached = 0;

        // Cost to each vertex
        map<Node *, double> cost_map;
        cost_map[rrt->root] = 0;

        // Run RRT*
        gamma = 1 + pow(2, SPACEDIM) * (1 + 1 / SPACEDIM) *
                rrt->getFreeArea();
        for (int i = 0; i < rrt->max_iter; i++) {
            qNew = rrt->getRandomNode();
            qNearest = rrt->nearest(qNew->position);
            if (rrt->distance(qNew->position, qNearest->position) >
                rrt->step_size) {
                Vector2f newConfig = rrt->newConfig(qNew, qNearest);
                if (rrt->isSegmentInObstacle(newConfig, qNearest->position)) {
                    continue;
                }
                qNew = new Node;
                qNew->position = newConfig;
            }

            rrt->nearestNeighbors(qNew->position);
            qBest = qNearest;
            distBest = rrt->distance(qNew->position, qNearest->position);
            radius = min(
                pow(gamma / PI * log(i + 1) / (i + 1), 1 / SPACEDIM),
                (double) rrt->step_size
            );

            // look for shortest cost path to qNew
            for (auto node : rrt->nodes) {
                // compute the cost for qNew through node
                dist = rrt->distance(qNew->position, node->position);
                cost = cost_map[node] + dist;

                // only consider proximal nodes
                if (dist > radius) {
                    break;
                }

                // if cost less than current lowest cost, check potential link
                if (cost < cost_map[qBest] + distBest) {
                    if (rrt->isSegmentInObstacle(qNew->position,
                            node->position)) {
                        continue;
                    }
                    qBest = node;
                    distBest = dist;
                }
            }

            if (rrt->isSegmentInObstacle(qNew->position, qBest->position)) {
                continue;
            }
            rrt->add(qBest, qNew, distBest);
            cost_map[qNew] = cost_map[qBest] + distBest;

            // rewire the tree
            for (auto node : rrt->nodes) {
                dist = rrt->distance(qNew->position, node->position);
                cost = cost_map[node] + dist;

                // only consider proximal nodes
                if (dist > radius) {
                    break;
                }

                // if cost less than current lowest cost, check potential link
                if (cost < cost_map[node]) {
                    if (rrt->isSegmentInObstacle(qNew->position,
                            node->position)) {
                        continue;
                    }
                    rrt->relink(node, qNew, dist);
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
            q = rrt->lastNode;
        }
        else
        {
            // if not reached yet, then shortestPath will start from the closest
            // node to end point.
            q = rrt->nearest(rrt->endPos);
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
            xPath[index] = node->position[0];
            yPath[index] = node->position[1];
            index += 1;
        }
        xPath[index] = NAN;
        yPath[index] = NAN;

        // free the memory
        rrt->reset();
        return reached;
    }
}