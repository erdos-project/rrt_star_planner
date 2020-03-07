#ifndef RRT_H
#define RRT_H

#include "obstacles.h"

#include <stdlib.h>
#include <vector>
#include <math.h>

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
    Obstacles *obstacles;
    vector<Node *> nodes;
    vector<Node *> path;
    Node *root, *lastNode;
    Vector2f startPos, endPos, origin, bounds;
    int max_iter;
    int step_size;
    RRT();
    ~RRT();
    void initialize();
    void reset();
    void setStepSize(int step);
    void setMaxIterations(int iter);
    void setStartPosition(double x, double y);
    void setEndPosition(double x, double y);
    void setStateSpace(double xStart, double yStart, double xEnd, double yEnd);
    void addObstacle(Vector2f firstPoint, Vector2f secondPoint);
    Node* getRandomNode();
    Node* nearest(Vector2f point);
    void nearestNeighbors(Vector2f point);
    void add(Node *qNearest, Node *qNew, double cost);
    void relink(Node *q, Node* qNew, double dist);
    static double distance(Vector2f &p, Vector2f &q);
    bool isSegmentInObstacle(Vector2f &p1, Vector2f &p2);
    double getFreeArea();
    Vector2f newConfig(Node *q, Node *qNearest);
    bool reached();
private:
    static void deleteNodes(Node *root);

};

#endif // RRT_H
