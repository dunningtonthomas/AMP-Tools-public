#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW9.h"

// Include helper functions
#include "helperFuncs.h"
#include <cmath>

// Structure for the tree obstacle
struct TreeObstacle {
    Eigen::Vector2d center;
    double radius;
};

// Structure for the agent dimensions
struct AgentProperties {
    double radius; // Range finder radius
    double length;
    double width;
};

// Class for the agent
class rangeFindingCar {
    public:
        // Default constructor
        rangeFindingCar() {
            agent_prop.length = 0.0;
            agent_prop.width = 0.0;
            agent_prop.radius = 10.0;
        }

        // Input constructor
        rangeFindingCar(double radius) {
            agent_prop.length = 0.0;
            agent_prop.width = 0.0;
            agent_prop.radius = radius;
        }

        // Propagate the state forward according to dynamics model
        void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt);
        AgentProperties agent_prop;
};

// Class for constructing the environment
class generateEnv {
    public:
        // Generate environments
        amp::KinodynamicProblem2D getEnv1();
        amp::KinodynamicProblem2D getEnv2();
        amp::KinodynamicProblem2D getEnvRandKino();
        amp::Problem2D getEnvRand();

        // @brief Given an overall problem definition, create a subproblem to adaptively plan around obstacles
        amp::Problem2D createSubproblem(const amp::Problem2D& problem, const Eigen::Vector2d& q_curr, const Eigen::Vector2d& q_intermediateGoal, const rangeFindingCar& agent);
        amp::KinodynamicProblem2D createSubproblem(const amp::KinodynamicProblem2D& problem, const Eigen::VectorXd& q_curr, const std::vector<std::pair<double, double>> q_intermediateGoal, const rangeFindingCar& agent);

        // @brief Create a random tree obstacle
        TreeObstacle randomTreeObstacle(double x_min, double x_max, double y_min, double y_max, double min_radius, double max_radius);

        // @brief Check if the new candidate obstacle is valid
        bool isValidObstacle(const TreeObstacle& obstacle, const std::vector<TreeObstacle>& obstacles, Eigen::Vector2d q_init, Eigen::Vector2d q_goal);

        // @brief Creates an obstacle2D object from a tree obstacle
        amp::Obstacle2D createObstacle(const TreeObstacle& obstacle);
};

