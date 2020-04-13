#include "rrt_star.h"

#include <algorithm>
#include <random>
#include <utility>
#include <vector>

RRT::RRT(RRTStarInitialConditions *rrts_ic_, RRTStarHyperparameters *rrts_hp_)
{
    rrts_ic = rrts_ic_;
    rrts_hp = rrts_hp_;
    setStateSpace(rrts_ic->x_start, rrts_ic->y_start, rrts_ic->x_end,
                  rrts_ic->y_end);
    setObstacles();
    initialize();
    setStepSize(rrts_hp->step_size);
    setMaxIterations(rrts_hp->max_iterations);
}

RRT::~RRT()
{
    for (auto obstacle : obstacles) {
        delete obstacle;
    }
    obstacles.clear();
    obstacles.resize(0);
    deleteNodes(root);
    nodes.clear();
    nodes.resize(0);
    path.clear();
    path.resize(0);
}

/**
 * @brief Initialize root node of RRT.
 */
void RRT::initialize()
{
    root = new Node(start_pos);
    root->parent = nullptr;
    last_node = root;
    nodes.push_back(root);
}

void RRT::setStepSize(double step)
{
    step_size = step;
}

void RRT::setMaxIterations(int iter)
{
    max_iter = iter;
}

void RRT::setStartPosition(double x, double y) {
    start_pos.x() = x;
    start_pos.y() = y;
}

void RRT::setStateSpace(double x_start, double y_start, double x_end, double y_end) {
    setStartPosition(x_start, y_start);
    setEndPosition(x_end, y_end);
    // State space calculated as minimum bounding rectangle with buffer
    // origin is lower left corner, bounds is width, height
    origin.x() = min(start_pos.x(), end_pos.x()) - rrts_hp->lane_width;
    origin.y() = min(start_pos.y(), end_pos.y()) - rrts_hp->lane_width;
    bounds.x() = max(start_pos.x(), end_pos.x()) - origin.x() + rrts_hp->lane_width;
    bounds.y() = max(start_pos.y(), end_pos.y()) - origin.y() + rrts_hp->lane_width;
}

void RRT::setEndPosition(double x, double y) {
    end_pos.x() = x;
    end_pos.y() = y;
}

void RRT::setObstacles() {
    // Construct obstacles
    vector<double> llx (rrts_ic->o_llx, rrts_ic->o_llx + rrts_ic->no);
    vector<double> lly (rrts_ic->o_lly, rrts_ic->o_lly + rrts_ic->no);
    vector<double> urx (rrts_ic->o_urx, rrts_ic->o_urx + rrts_ic->no);
    vector<double> ury (rrts_ic->o_ury, rrts_ic->o_ury + rrts_ic->no);

    for (int i = 0; i < rrts_ic->no; i++) {
        addObstacle(
            Vector2f(llx[i], lly[i]),
            Vector2f(urx[i], ury[i])
        );
    }
}

void RRT::addObstacle(Vector2f first_point, Vector2f second_point) {
    obstacles.push_back(new Obstacle(std::move(first_point),
        std::move(second_point), rrts_hp->obstacle_clearance));
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
    if (drand48() <= 0.05) {
        ret = new Node(end_pos);
    } else {
        Vector2f point(drand48() * bounds.x(),
                       drand48() * bounds.y());
        ret = new Node(point + origin);
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
    double min_dist = INFINITY;
    Node *closest = nullptr;
    for(auto & node : nodes) {
        double dist = distance(point, node->position);
        if (dist < min_dist) {
            min_dist = dist;
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
bool RRT::isSegmentInObstacles(Vector2f &p1, Vector2f &p2) {
    for (auto obstacle : obstacles) {
        if (obstacle->isSegmentInObstacle(p1, p2)) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Return the free area in the state space, considering obstacles
 */
double RRT::getFreeArea()
{
    double space_area = bounds.x() * bounds.y();
    double obstacle_area = 0;
    for (auto obstacle : obstacles) {
        obstacle_area += obstacle->getArea();
    }
    return space_area - obstacle_area;
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to random node.
 * @param q
 * @param q_nearest
 * @return
 */
Vector2f RRT::newConfig(Node *q, Node *q_nearest)
{
    Vector2f to = q->position;
    Vector2f from = q_nearest->position;
    Vector2f intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2f ret = from + step_size * intermediate;
    return ret;
}

/**
 * @brief Add a node to the tree.
 * @param q_nearest
 * @param q_new
 */
void RRT::add(Node *q_nearest, Node *q_new, double dist)
{
    q_new->dist = dist;
    q_new->parent = q_nearest;
    q_nearest->children.push_back(q_new);
    nodes.push_back(q_new);
    last_node = q_new;
}

/**
 * @brief Relink q to have parent q_new.
 * @param q
 * @param q_new
 * @param dist
 */
void RRT::relink(Node *q, Node *q_new, double dist)
{
    // set the parent of q to q_new
    if (q->parent != nullptr) {
        for (int i = 0; i < (int) q->parent->children.size(); i++) {
            if (q->parent->children[i] == q) {
                q->parent->children.erase(q->parent->children.begin() + i);
                break;
            }
        }
    }
    q->dist = dist;
    q->parent = q_new;
    q_new->children.push_back(q);
}


/**
 * @brief Check if the last node is close to the end position.
 * @return
 */
bool RRT::reached()
{
    return distance(last_node->position, end_pos) < rrts_hp->end_dist_threshold;
}

