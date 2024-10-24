#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW8.h"

// This is file is loaded from the shared/ directory
// Overwrite with your MySamplingBasedPlanners.h and MySamplingBasedPlanners.cpp from hw7
#include "MySamplingBasedPlanners.h" 

#include <random>
#include <helperFuncs.h>


class MyCentralPlanner : public amp::CentralizedMultiAgentRRT {
    public:
        // Constructors
        MyCentralPlanner() : step_size(0.5), goal_bias(0.05), max_iterations(150000), epsilon(0.25), success(false) {}
        MyCentralPlanner(double r, double p, int n, double e) : step_size(r), goal_bias(p), max_iterations(n), epsilon(e), success(false) {}

        // @breif Use centralized planning with RRT to solve the multi-agent problem
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override; 

        // @brief generate a random configuration of the centralized planner
        Eigen::VectorXd randomConfiguration(const amp::MultiAgentProblem2D& problem);

        // @brief calculate the nearest neighbor using a standard distance metric
        amp::Node nearestNeighbor(const Eigen::VectorXd& q_rand);

        // @brief calculate the distance between two configurations
        inline double configurationDistance(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) {
            return (q1 - q2).norm();
        }

        // @brief check if the subpath between two nodes is collision free
        bool isSubpathCollisionFree(const amp::MultiAgentProblem2D& problem, const Eigen::VectorXd& q_near, const Eigen::VectorXd& q_rand);

        // @brief check if a configuration is in collision with the environment or other agents
        bool isSystemValid(const amp::MultiAgentProblem2D& problem, const Eigen::VectorXd& q);

        // @brief check if the system is in goal
        bool isSystemInGoal(const Eigen::VectorXd& q, const Eigen::VectorXd& q_goal);

        // @brief check if the disk robot is in collision with an obstacle
        bool obstacleCollision(const amp::Obstacle2D& obstacle, const Eigen::Vector2d& agent_position, double agent_radius);

        // @brief check if the disk robot is in collision with another agent
        bool agentCollision(const Eigen::Vector2d& agent_position, double agent_radius, const Eigen::Vector2d& other_agent_position, double other_agent_radius);

    private:
        // RRT Hyperparameters
        double step_size;
        double goal_bias;
        int max_iterations;
        double epsilon;

        // Node map, parent map
        std::map<amp::Node, Eigen::VectorXd> nodes;
        std::map<amp::Node, amp::Node> parents;

        // Success variable
        bool success;
};


class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT {
    public:
        // Constructors
        MyDecentralPlanner() : step_size(0.5), goal_bias(0.05), max_iterations(7500), epsilon(0.25), success(false), overall_success(true) {}
        MyDecentralPlanner(double r, double p, int n, double e) : step_size(r), goal_bias(p), max_iterations(n), epsilon(e), success(false), overall_success(true) {}

        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

        // @brief generate a random configuration of the centralized planner
        Eigen::Vector2d randomAgentConfiguration(const amp::MultiAgentProblem2D& problem);

        // @brief calculate the nearest neighbor using a standard distance metric
        amp::Node nearestNeighbor(const Eigen::VectorXd& q_rand);

        // @brief calculate the distance between two configurations
        inline double configurationDistance(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) {
            return (q1 - q2).norm();
        }

        // @brief check if the subpath between two nodes is collision free
        bool isSubpathCollisionFree(const amp::MultiAgentProblem2D& problem, const Eigen::VectorXd& q_near, const Eigen::VectorXd& q_rand, int q_rand_index);

        // @brief check if a configuration is in collision with the environment or other agents
        bool isSystemValid(const amp::MultiAgentProblem2D& problem, const Eigen::VectorXd& q, int q_rand_index);

        // @brief check if the system is in goal
        bool isSystemInGoal(const Eigen::VectorXd& q, const Eigen::VectorXd& q_goal);

        // @brief check if the disk robot is in collision with an obstacle
        bool obstacleCollision(const amp::Obstacle2D& obstacle, const Eigen::Vector2d& agent_position, double agent_radius);

        // @brief check if the disk robot is in collision with another agent
        bool agentCollision(const Eigen::Vector2d& agent_position, double agent_radius, const Eigen::Vector2d& other_agent_position, double other_agent_radius);

    private:
        // RRT Hyperparameters
        double step_size;
        double goal_bias;
        int max_iterations;
        double epsilon;

        // Node map, parent map, index map
        std::map<amp::Node, Eigen::Vector2d> nodes;
        std::map<amp::Node, amp::Node> parents;
        std::map<amp::Node, int> index_map; 
        amp::MultiAgentPath2D current_path_state;

        // Success variable
        bool success;
        bool overall_success;
};