#include "rrt_star.h"

#include <algorithm>
#include <utility>
#include <vector>

RRT::RRT()
{
    obstacles = new Obstacles;
    setStateSpace(START_POS_X, START_POS_Y, END_POS_X, END_POS_Y);
    root = new Node;
    root->parent = nullptr;
    root->position = startPos;
    lastNode = root;
    nodes.push_back(root);
    step_size = 0.5;
    max_iter = 5000;
}

RRT::~RRT()
{
    reset();
}

/**
 * @brief Initialize root node of RRT.
 */
void RRT::initialize()
{
    root = new Node;
    root->parent = nullptr;
    root->position = startPos;
    lastNode = root;
    nodes.push_back(root);
}

/**
 * @brief Clear the RRT and free memory.
 */
void RRT::reset()
{
    obstacles->obstacles.clear();
    obstacles->obstacles.resize(0);
    deleteNodes(root);
    nodes.clear();
    nodes.resize(0);
    path.clear();
    path.resize(0);
}

void RRT::setStepSize(int step)
{
    step_size = step;
}

void RRT::setMaxIterations(int iter)
{
    max_iter = iter;
}

void RRT::setStartPosition(double x, double y) {
    startPos.x() = x;
    startPos.y() = y;
}

void RRT::setStateSpace(double xStart, double yStart, double xEnd, double yEnd) {
    setStartPosition(xStart, yStart);
    setEndPosition(xEnd, yEnd);
    // State space calculated as minimum bounding rectangle with buffer
    // origin is lower left corner, bounds is width, heigh
    origin.x() = min(startPos.x(), endPos.x()) - LANE_WIDTH;
    origin.y() = min(startPos.y(), endPos.y()) - LANE_WIDTH;
    bounds.x() = max(startPos.x(), endPos.x()) - origin.x() + LANE_WIDTH;
    bounds.y() = max(startPos.y(), endPos.y()) - origin.y() + LANE_WIDTH;
}

void RRT::setEndPosition(double x, double y) {
    endPos.x() = x;
    endPos.y() = y;
}

void RRT::addObstacle(Vector2f firstPoint, Vector2f secondPoint) {
    obstacles->addObstacle(std::move(firstPoint), std::move(secondPoint));
}

/**
 * @brief Delete all nodes using DFS technique.
 * @param root
 */
void RRT::deleteNodes(Node *root)
{
    for(auto & i : root->children) {
        deleteNodes(i);
    }
    delete root;
}

/**
 * @brief Generate a random node in the field.
 * @return
 */
Node* RRT::getRandomNode()
{
    // Try to connect with the end occasionally
    Node* ret;
    if (drand48() <= 0.1) {
        ret = new Node;
        ret->position = endPos;
    } else {
        Vector2f point(drand48() * bounds.x(),
                       drand48() * bounds.y());
        ret = new Node;
        ret->position = point + origin;
    }
    return ret;
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */
Node* RRT::nearest(Vector2f point)
{
    double minDist = 1e9;
    Node *closest = nullptr;
    for(auto & node : nodes) {
        double dist = distance(point, node->position);
        if (dist < minDist) {
            minDist = dist;
            closest = node;
        }
    }
    return closest;
}

/**
 * @brief Sort the list of nodes by distance to the given point.
 * @param point
 * @return
 */
void RRT::nearestNeighbors(Vector2f point)
{
    sort(
            nodes.begin(), nodes.end(),
            [&point](Node* lhs, Node* rhs)
            {
                return RRT::distance(point, lhs->position) <
                       RRT::distance(point,rhs->position);
            }
    );
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
double RRT::distance(Vector2f &p, Vector2f &q)
{
    Vector2f v = p - q;
    return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}
/**
 * @brief Return whether the given line segment intersects an obstacle
 */
bool RRT::isSegmentInObstacle(Vector2f &p1, Vector2f &p2) {
    return obstacles->isSegmentInObstacle(p1, p2);
}

/**
 * @brief Return the free area in the state space, considering obstacles
 */
double RRT::getFreeArea()
{
    double spaceArea = bounds.x() * bounds.y();
    double width, length;
    double obstacleArea = 0;
    for (auto obstacle : obstacles->obstacles) {
        // top right x - bottom right x
        width = obstacle.second[0] - obstacle.first[0];
        length = obstacle.second[1] - obstacle.first[1];
        obstacleArea += width * length;
    }
    return spaceArea - obstacleArea;
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to random node.
 * @param q
 * @param qNearest
 * @return
 */
Vector2f RRT::newConfig(Node *q, Node *qNearest)
{
    Vector2f to = q->position;
    Vector2f from = qNearest->position;
    Vector2f intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2f ret = from + step_size * intermediate;
    return ret;
}

/**
 * @brief Add a node to the tree.
 * @param qNearest
 * @param qNew
 */
void RRT::add(Node *qNearest, Node *qNew, double dist)
{
    qNew->dist = dist;
    qNew->parent = qNearest;
    qNearest->children.push_back(qNew);
    nodes.push_back(qNew);
    lastNode = qNew;
}

/**
 * @brief Relink q to have parent qNew.
 * @param q
 * @param qNew
 * @param dist
 */
void RRT::relink(Node *q, Node *qNew, double dist)
{
    // set the parent of q to qNew
    if (q->parent != nullptr) {
        for (int i = 0; i < (int) q->parent->children.size(); i++) {
            if (q->parent->children[i] == q) {
                q->parent->children.erase(q->parent->children.begin() + i);
                break;
            }
        }
    }
    q->dist = dist;
    q->parent = qNew;
    qNew->children.push_back(q);
}


/**
 * @brief Check if the last node is close to the end position.
 * @return
 */
bool RRT::reached()
{
    return distance(lastNode->position, endPos) < END_DIST_THRESHOLD;
}

