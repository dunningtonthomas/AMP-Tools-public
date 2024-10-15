#pragma once

#include "AMPCore.h"
#include "Eigen/Geometry"
#include "hw/HW2.h"
#include "hw/HW4.h"
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <random>

namespace amp{
    // @brief Print the vertices of an obstacle
    void printVertices(std::vector<Eigen::Vector2d>& vertices);

    // @brief Calculate the minkowski sum of two polygons
    Polygon minkowski(amp::Polygon& obstacle, amp::Polygon& negative_object);

    // @brief Calculate the angle of a side of a polygon relative to horizontal
    double angle(Eigen::Vector2d v1, Eigen::Vector2d v2);

    // @brief Sort the vertices of a polygon in counter clockwise order such that the first vertex is the smallest y coordinate
    std::vector<Eigen::Vector2d> sortVerticesCCW(std::vector<Eigen::Vector2d> vertices);

    // @brief Rotate a polygon by a given angle
    Polygon rotatePolygon(amp::Polygon& obstacle, double angle);

    // @brief Return the negative of the vertices of a polygon
    std::vector<Eigen::Vector2d> negativeVertices(std::vector<Eigen::Vector2d>& vertices);

    // @brief Collision Checker for manipulator using line segment intersection
    bool inCollision(const amp::Problem2D& problem, Eigen::Vector2d current_pos, Eigen::Vector2d next_pos);
    bool inCollision(const amp::Problem2D& problem, Eigen::Vector2d current_pos, Eigen::Vector2d next_pos, int& collision_index, amp::Obstacle2D& collision_obstacle);
    bool intersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2);
    bool onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r);
    int orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r); 

    // @brief Collision detection for point robot in polygon
    bool isInsidePolygon(const amp::Polygon& Obstacle, Eigen::Vector2d p);
    bool inCollision_point(const amp::Environment2D& env, Eigen::Vector2d point);

    // @brief generate a Eigen::Vector2d configuration in the workspace
    Eigen::Vector2d randomConfiguration(const amp::Environment2D& env);

    // @brief Calculate the distance between two points
    double distance(Eigen::Vector2d p1, Eigen::Vector2d p2);
} // namespace amp



