#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW9.h"

// Include helper functions
#include "helperFuncs.h"

// Structure for the tree obstacle
struct TreeObstacle {
    Eigen::Vector2d center;
    double radius;
};

// Class for constructing the environment
class generateEnv {
    public:
        amp::KinodynamicProblem2D getEnv1();
        amp::KinodynamicProblem2D getEnv2();
        amp::KinodynamicProblem2D getEnvRandKino();
        amp::Problem2D getEnvRand();

        // @brief Create a random tree obstacle
        TreeObstacle randomTreeObstacle(double x_min, double x_max, double y_min, double y_max, double min_radius, double max_radius);

        // @brief Check if the new candidate obstacle is valid
        bool isValidObstacle(const TreeObstacle& obstacle, const std::vector<TreeObstacle>& obstacles, Eigen::Vector2d q_init, Eigen::Vector2d q_goal);

        // @brief Creates an obstacle2D object from a tree obstacle
        amp::Obstacle2D createObstacle(const TreeObstacle& obstacle);
};

