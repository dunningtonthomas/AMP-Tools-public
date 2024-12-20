#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"

// Include helper funtions
#include "helperFuncs.h"

// Include Astar class
#include "MyAStar.h"

// Include random library to get uniform sampling
#include <random>

class MyPRM : public amp::PRM2D {
    public:
        // Default constructor
        MyPRM() : n(1000), r(2), success(false) {}

        // constructor to populate n and r
        MyPRM(int n, double r) : n(n), r(r), success(false) {}

        // getter functions
        std::map<amp::Node, Eigen::Vector2d> getNodes() { return nodes; }
        std::shared_ptr<amp::Graph<double>> getGraph() { return graphPtr; }
        bool getSuccess() { return success; }

        // @brief Find path using PRM
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 

        // @brief Create PRM Graph return the graph and the nodes map by reference
        void createGraph(const amp::Problem2D& problem);

        // @brief implement path smoothing by removing unnecessary waypoints
        void pathSmoothing(amp::Path2D& path, const amp::Problem2D& problem);

    private:
        // PRM Hyperparameters
        int n;
        double r;

        // Graph, node map
        std::shared_ptr<amp::Graph<double>> graphPtr;
        std::map<amp::Node, Eigen::Vector2d> nodes;

        // Success variable
        bool success;
};


class MyRRT : public amp::GoalBiasRRT2D {
    public:
        // Default constructor
        MyRRT() : step_size(0.5), goal_bias(0.05), max_iterations(5000), epsilon(0.25), success(false) {}

        // constructor to populate step_size, goal_bias, max_iterations, and epsilon
        MyRRT(double r, double p, int n, double e) : step_size(r), goal_bias(p), max_iterations(n), epsilon(e), success(false) {}

        // getter functions
        std::map<amp::Node, Eigen::Vector2d> getNodes() { return nodes; }
        std::shared_ptr<amp::Graph<double>> getGraph() { return graphPtr; }
        bool getSuccess() { return success; }

        // @brief Find path using RRT
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 

        // @brief Create the graph using RRT, return node map and parents map by reference
        void createGraph(const amp::Problem2D& problem);

        // @brief Find the nearest neighbor to a node in the graph
        amp::Node nearestNeighbor(const Eigen::Vector2d& q_rand);

    private:
        // RRT Hyperparameters
        double step_size;
        double goal_bias;
        int max_iterations;
        double epsilon;

        // Graph, node map, parent map
        std::shared_ptr<amp::Graph<double>> graphPtr;
        std::map<amp::Node, Eigen::Vector2d> nodes;
        std::map<amp::Node, amp::Node> parents;

        // Success variable
        bool success;
};


struct LookupSearchHeuristic : public amp::SearchHeuristic {
	/// @brief Get the heuristic value stored in `heuristic_values`. 
	/// @param node Node to get the heuristic value h(node) for. 
	/// @return Heuristic value
	virtual double operator()(amp::Node node) const override {
        // Check if the node is in the map
        if(heuristic_values.find(node) == heuristic_values.end()) {
            std::cout << "Node not found in heuristic map" << std::endl;
            return 0;
        } else {
            return heuristic_values.at(node);
        }
    }

    /// @brief Store the heursitic values for each node in a map
    std::map<amp::Node, double> heuristic_values; 
};