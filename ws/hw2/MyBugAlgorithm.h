#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
        bool inCollision(Eigen::Vector2d point, const amp::Problem2D& problem, double step_size);

        void moveToGoal(const Eigen::Vector2d& goal, amp::Path2D& path, const amp::Problem2D& problem);
        void handleCollision(const Eigen::Vector2d& goal, amp::Path2D& path, const amp::Problem2D& problem);
        void Bug1Traversal(amp::Path2D& path, const amp::Problem2D& problem, std::vector<Eigen::Vector2d>& vertices);

    private:
        // Add any member variables here...
};