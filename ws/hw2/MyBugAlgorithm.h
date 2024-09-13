#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Collision Checker
        bool inCollision(const amp::Problem2D& problem, Eigen::Vector2d dir, int& collision_index, amp::Obstacle2D& collision_obstacle);
        bool intersect(Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3, Eigen::Vector2d p4);
        bool onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r); 
        int orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r); 

        // Distance Checker
        double distance(Eigen::Vector2d p1, Eigen::Vector2d p2);

        // Rotate Headings
        void rotateHeading(double angle);

        // Bug 1 Traversal
        void Bug1Traversal(amp::Path2D& path, const amp::Problem2D& problem);
        void Bug2Traversal(amp::Path2D& path, const amp::Problem2D& problem);

    private:

        double step_size = 0.1;
        Eigen::Vector2d current_position;
        Eigen::Vector2d heading;
        Eigen::Vector2d right_heading;
};