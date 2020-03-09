#include "obstacles.h"

#include <iostream>

using namespace std;
using namespace Eigen;

/**
 * @brief Obstacles are stored as rectangles. Rectangle is denoted by two points : topLeft and bottomRight.
 * @param firstPoint
 * @param secondPoint
 */
void Obstacles::addObstacle(Vector2f firstPoint, Vector2f secondPoint)
{
    // Get topLeft and bottomRight points from the given points.
    Vector2f tmp;
    if (firstPoint.x() > secondPoint.x() && firstPoint.y() > secondPoint.y()) {
        tmp = firstPoint;
        firstPoint = secondPoint;
        secondPoint = tmp;
    } else if (firstPoint.x() < secondPoint.x() && firstPoint.y() > secondPoint.y()) {
        float height = firstPoint.y() - secondPoint.y();
        firstPoint.y() -= height;
        secondPoint.y() += height;
    } else if (firstPoint.x() > secondPoint.x() && firstPoint.y() < secondPoint.y()) {
        float length = firstPoint.x() - secondPoint.x();
        firstPoint.x() -= length;
        secondPoint.x() += length;
    }
    firstPoint.x() -= BOT_CLEARANCE;
    firstPoint.y() -= BOT_CLEARANCE;
    secondPoint.x() += BOT_CLEARANCE;
    secondPoint.y() += BOT_CLEARANCE;
    obstacles.emplace_back(firstPoint, secondPoint);
}

/**
 * @brief Check if a line segment intersects a rectangle.
 * @param p1
 * @param p2
 * @return
 */
bool Obstacles::isSegmentInObstacle(Vector2f &p1, Vector2f &p2)
{
    QLineF lineSegment(p1.x(), p1.y(), p2.x(), p2.y());
    auto *intersectPt = new QPointF;
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
        QLineF::IntersectType x1 = lineSegment.intersect(lseg1, intersectPt);
        QLineF::IntersectType x2 = lineSegment.intersect(lseg2, intersectPt);
        QLineF::IntersectType x3 = lineSegment.intersect(lseg3, intersectPt);
        QLineF::IntersectType x4 = lineSegment.intersect(lseg4, intersectPt);
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
