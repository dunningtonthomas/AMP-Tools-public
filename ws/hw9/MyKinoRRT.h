#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW9.h"

// Include helper functions
#include "helperFuncs.h"

class MyKinoRRT : public amp::KinodynamicRRT {
    public:
        // Default constructor
        MyKinoRRT() : goal_bias(0.05), max_iterations(10000), epsilon(0.5), success(false), dt(0.3) {}

        // @brief Use RRT Kinodynamic planning
        virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;

        // @brief Generate a random control input
        Eigen::VectorXd randomControl(const amp::KinodynamicProblem2D& problem);

        // @brief Generate a random configuration 
        Eigen::VectorXd randomConfiguration(const amp::KinodynamicProblem2D& problem);

        // @brief Sample the goal
        Eigen::VectorXd sampleGoal(const amp::KinodynamicProblem2D& problem);

        // @brief calculate the nearest neighbor using a standard distance metric
        amp::Node nearestNeighbor(const amp::KinodynamicProblem2D& problem, const Eigen::VectorXd& q_rand);

        // @brief calculate the distance between two configurations
        double configurationDistance(const amp::KinodynamicProblem2D& problem, const Eigen::VectorXd& q1, const Eigen::VectorXd& q2);

        // @brief create a collision free subpath from the nearest node to a new node
        bool createSubpath(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent, Eigen::VectorXd& q_near, Eigen::VectorXd& q_rand, Eigen::VectorXd& q_new, Eigen::VectorXd& control);

        // @brief check if the subpath between two nodes is collision free
        bool isSubpathCollisionFree(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent, Eigen::VectorXd& q_near, Eigen::VectorXd& q_new, Eigen::VectorXd& q_rand);

        // @brief check if a configuration is in collision with the environment or other agents
        bool isSystemValid(const amp::KinodynamicProblem2D& problem, const Eigen::VectorXd& q);

        // @brief check if the system is in goal
        bool isSystemInGoal(const amp::KinodynamicProblem2D& problem, const Eigen::VectorXd& q);

        // @brief check if the disk robot is in collision with an obstacle
        bool obstacleCollision(const amp::KinodynamicProblem2D& problem, const amp::Obstacle2D& obstacle, const Eigen::VectorXd& agent_position);

    private:
        // RRT Hyperparameters
        double goal_bias;
        int max_iterations;
        double epsilon;
        bool success;
        double dt;

        // Node map and parent map, control map
        std::map<amp::Node, Eigen::VectorXd> nodes;
        std::map<amp::Node, amp::Node> parents;
        std::map<amp::Node, Eigen::VectorXd> controls;
};  

class MySingleIntegrator : public amp::DynamicAgent {
    public:
        // Default constructor
        MySingleIntegrator() {
            agent_dim.length = 0.0;
            agent_dim.width = 0.0;
        }

        // Propagate the state forward according to dynamics model
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;

};

class MyFirstOrderUnicycle : public amp::DynamicAgent {
    public:
        MyFirstOrderUnicycle() {
            agent_dim.length = 0.0;
            agent_dim.width = 0.0;
            radius = 0.25;
        }
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;

    private:
        double radius;
};

class MySecondOrderUnicycle : public amp::DynamicAgent {
    public:
        MySecondOrderUnicycle() {
            agent_dim.length = 0.0;
            agent_dim.width = 0.0;
            radius = 0.25;
        }
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;

    private:
        double radius;
};

class MySimpleCar : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {};
};