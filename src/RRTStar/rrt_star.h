#ifndef RRT_H
#define RRT_H

#include <cstdlib>
#include <vector>
#include <cmath>

#include "obstacle.h"

using namespace std;
using namespace Eigen;

struct Node {
    vector<Node *> children;
    Node *parent;
    Vector2f position;
    double dist;
};

class RRT
{
public:
    vector<Obstacle *> obstacles;
    vector<Node *> nodes;
    vector<Node *> path;
    Node *root, *last_node;
    Vector2f start_pos, end_pos, origin, bounds;
    int max_iter;
    double step_size;
    RRT(double x_start, double y_start, double x_end, double y_end,
        double step_dist, int max_iterations);
    ~RRT();
    void initialize();
    void reset();
    void setStepSize(int step);
    void setMaxIterations(int iter);
    void setStartPosition(double x, double y);
    void setEndPosition(double x, double y);
    void setStateSpace(double x_start, double y_start, double x_end, double y_end);
    void addObstacle(Vector2f first_point, Vector2f second_point);
    Node* getRandomNode();
    Node* nearest(Vector2f point);
    void nearestNeighbors(Vector2f point);
    void add(Node *q_nearest, Node *q_new, double cost);
    static void relink(Node *q, Node* q_new, double dist);
    static double distance(Vector2f &p, Vector2f &q);
    bool isSegmentInObstacles(Vector2f &p1, Vector2f &p2);
    double getFreeArea();
    Vector2f newConfig(Node *q, Node *q_nearest);
    bool reached();
private:
    static void deleteNodes(Node *root);

};

#endif // RRT_H
