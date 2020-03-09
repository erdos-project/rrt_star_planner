#ifndef OBSTACLES_H
#define OBSTACLES_H

#include <cassert>
#include <vector>
#include <QDebug>
#include <QLine>
#include <eigen3/Eigen/Dense>

#include "src/constants.h"

class Obstacles
{
public:
    Obstacles() = default;
    void addObstacle(Eigen::Vector2f firstPoint, Eigen::Vector2f secondPoint);
    bool isSegmentInObstacle(Eigen::Vector2f &p1, Eigen::Vector2f &p2);
    std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> obstacles;
};

#endif // OBSTACLES_H
