// This header file will include the class for the high level planner 
#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"
#include "hw/HW9.h"

// Include helper funtions
#include "helperFuncs.h"
#include "createEnv.h"
#include "MyRRT.h"
#include "MyKinoRRT.h"

// Include random library to get uniform sampling
#include <random>
#include <fstream>
#include <chrono>

// This class will implement the high level planner for the adaptive RRT
class adaptiveRRT {
    public:
        // @brief Planning algorithm, returns a valid path using the range finder for planning
        amp::Path2D plan(const amp::Problem2D& problem, const rangeFindingCar& agent);
        amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, const rangeFindingCar& agent);

        // @brief Return obstacles within the range of the agent
        std::vector<amp::Obstacle2D> getObstaclesInRange(const amp::Problem2D& problem, const Eigen::Vector2d& q_curr, const rangeFindingCar& agent);
        std::vector<amp::Obstacle2D> getObstaclesInRange(const amp::KinodynamicProblem2D& problem, const Eigen::VectorXd& q_curr, const rangeFindingCar& agent);

        // @brief Returns waypoints within the range of the agent
        std::vector<Eigen::Vector2d> getWaypointsInRange(std::vector<Eigen::Vector2d> curr_waypoints, const Eigen::Vector2d& q_curr, const rangeFindingCar& agent);
        std::vector<Eigen::VectorXd> getWaypointsInRange(std::vector<Eigen::VectorXd> curr_waypoints, const Eigen::VectorXd& q_curr, const rangeFindingCar& agent);

        // @brief Determines if the waypoints within the range finder radius intersect with any obstacles
        bool waypointsIntersectObstacles(const std::vector<Eigen::Vector2d>& waypoints, const std::vector<amp::Obstacle2D>& obstacles);
        bool waypointsIntersectObstacles(const std::vector<Eigen::VectorXd>& waypoints, const std::vector<amp::Obstacle2D>& obstacles);

        // @brief Write the current waypoints to a file by appending it and adding to the end row
        void writeWaypointsToFile(const std::vector<Eigen::Vector2d>& waypoints, std::ofstream& data_file);
        void writeWaypointsToFile(const std::vector<Eigen::VectorXd>& waypoints, std::ofstream& data_file);
    private:

};