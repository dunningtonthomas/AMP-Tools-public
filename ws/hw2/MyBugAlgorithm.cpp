#include "MyBugAlgorithm.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    double step_size = 0.1;
    // path.waypoints.push_back(Eigen::Vector2d(1.0, 5.0));
    // path.waypoints.push_back(Eigen::Vector2d(3.0, 9.0));
    // path.waypoints.push_back(problem.q_goal);

    //Generate a set of primitives to represent the obstacles




    // Bug 1 Algorithm
    path.waypoints.push_back(problem.q_init);
    while(true) {
        // Move toward the goal
        Eigen::Vector2d last = path.waypoints.back();
        Eigen::Vector2d goal = problem.q_goal;
        Eigen::Vector2d dir = (goal - last).normalized();

        // Move toward the goal
        Eigen::Vector2d next = last + step_size * dir;
        path.waypoints.push_back(next);

        // Check if we are at the goal
        if((next - goal).norm() < step_size) {
            path.waypoints.push_back(problem.q_goal);
            break;
        }

        // Check if we are colliding with an obstacle
        if(inCollision(next, problem, step_size)) {
            // Execute Bug 1 obstacle traversal
            MyBugAlgorithm::Bug1Traversal(amp::Path2D& path, const amp::Problem2D& problem, std::vector<Eigen::Vector2d>& vertices);
        }
    }


    return path;
}


void MyBugAlgorithm::Bug1Traversal(amp::Path2D& path, const amp::Problem2D& problem, std::vector<Eigen::Vector2d>& vertices) {
    // Bug 1 obstacle traversal, returns the leave point of the obstacle
    std::vector<Eigen::Vector2d> untraversed_vertices;
    double min_dist = (path.waypoints.back() - problem.q_goal).norm();
    Eigen::Vector2d closest;
    bool collision_vertex = false;

    for(const auto& vertex : vertices) {
        if ((path.waypoints.back() - vertex).norm() < step_size) {
            collision_vertex = true;
        }

        if(!collision_vertex) {
            untraversed_vertices.push_back(vertex);
        } else {
            path.waypoints.push_back(vertex);

            // Calculate distance to goal
            double dist_to_goal = (vertex - problem.q_goal).norm();
        }    
    }
    
}



bool MyBugAlgorithm::inCollision(Eigen::Vector2d point, const amp::Problem2D& problem, double step_size) {
    // Return true if in a collision
    for(const auto& obs : problem.obstacles) {
        for(const auto& vertex : obs.verticesCCW()) {
            if((point - vertex).norm() < step_size) {
                return true;
            }
        }
    }
    return false;
}


/// Collision checker thoughts:
// Loop through the vertices in counter-clockwise order and create a function for the line that defines the edge
// Then quantify which side is the obstacle
