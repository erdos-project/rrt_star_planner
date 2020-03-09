#include "obstacles.h"

#include <iostream>

using namespace std;
using namespace Eigen;

/**
 * @brief Obstacles are stored as rectangles. Rectangle is denoted by two points : topLeft and bottomRight.
 * @param first_point
 * @param second_point
 */
void Obstacles::addObstacle(Vector2f first_point, Vector2f second_point
)
{
    // Get topLeft and bottomRight points from the given points.
    Vector2f tmp;
    if (first_point.x() > second_point.x() && first_point.y() > second_point.y()) {
        tmp = first_point;
        first_point = second_point;
        second_point = tmp;
    } else if (first_point.x() < second_point.x() && first_point.y() > second_point.y()) {
        float height = first_point.y() - second_point.y();
        first_point.y() -= height;
        second_point.y() += height;
    } else if (first_point.x() > second_point.x() && first_point.y() < second_point.y()) {
        float length = first_point.x() - second_point.x();
        first_point.x() -= length;
        second_point.x() += length;
    }
    first_point.x() -= BOT_CLEARANCE;
    first_point.y() -= BOT_CLEARANCE;
    second_point.x() += BOT_CLEARANCE;
    second_point.y() += BOT_CLEARANCE;
    obstacles.emplace_back(first_point, second_point);
}

/**
 * @brief Check if a line segment intersects a rectangle.
 * @param p1
 * @param p2
 * @return
 */
bool Obstacles::isSegmentInObstacle(Vector2f &p1, Vector2f &p2)
{
    QLineF line_segment(p1.x(), p1.y(), p2.x(), p2.y());
    auto *intersect_pt = new QPointF;
    for (auto & obstacle : obstacles) {
        float length = obstacle.second.x() - obstacle.first.x();
        float breadth = obstacle.second.y() - obstacle.first.y();
        QLineF lseg1(obstacle.first.x(), obstacle.first.y(),
                obstacle.first.x() + length, obstacle.first.y());
        QLineF lseg2(obstacle.first.x(), obstacle.first.y(),
                obstacle.first.x(), obstacle.first.y() + breadth);
        QLineF lseg3(obstacle.second.x(), obstacle.second.y(),
                obstacle.second.x(), obstacle.second.y() - breadth);
        QLineF lseg4(obstacle.second.x(), obstacle.second.y(),
                obstacle.second.x() - length, obstacle.second.y());
        QLineF::IntersectType x1 = line_segment.intersect(lseg1, intersect_pt);
        QLineF::IntersectType x2 = line_segment.intersect(lseg2, intersect_pt);
        QLineF::IntersectType x3 = line_segment.intersect(lseg3, intersect_pt);
        QLineF::IntersectType x4 = line_segment.intersect(lseg4, intersect_pt);
        // check for bounded intersection. IntersectType for bounded intersection is 1.
        if (x1 == 1 || x2 == 1 || x3 == 1 || x4 == 1)
            return true;
        // check for containment
        if (obstacle.first.x() < p1.x() &&
            p1.x() < obstacle.second.x() &&
            obstacle.first.y() < p1.y() &&
            p1.x() < obstacle.second.y())
            return true;
    }
    return false;
}
