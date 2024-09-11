#include "MyBugAlgorithm.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;

    // Define the step size
    step_size = 0.1;

    // Starting Position
    current_position = problem.q_init;
    heading = (problem.q_goal - current_position).normalized();
    right_heading = Eigen::Vector2d(heading.y(), -heading.x());
    path.waypoints.push_back(current_position);

    // Bug 1 Algorithm
    while(true) {
        // Update the direction to the goal
        Eigen::Vector2d dir = (problem.q_goal - current_position).normalized();

        // Calculate the next position towards the goal
        next_position = current_position + step_size * dir;

        // Check if we are colliding with an obstacle
        // if(inCollision(problem, step_size)) {
        //     // Execute Bug 1 obstacle traversal
        //     Bug1Traversal(path, problem);
        // } else {
        //     // Next point is not a collision, add to the overall path
        //     path.waypoints.push_back(next_position);
        // }

        // Update heading
        heading = dir;
        right_heading = Eigen::Vector2d(heading.y(), -heading.x());

        // Move toward the goal
        path.waypoints.push_back(next_position);

        // Check if we are at the goal
        if((path.waypoints.back() - problem.q_goal).norm() < step_size) {
            path.waypoints.push_back(problem.q_goal);
            break;
        }
    }
    return path;
}


void MyBugAlgorithm::Bug1Traversal(amp::Path2D& path, const amp::Problem2D& problem) {
    // Bug 1 obstacle traversal, returns the leave point of the obstacle
    std::vector<Eigen::Vector2d> untraversed_vertices;
    std::vector<Eigen::Vector2d> vertices;
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



bool MyBugAlgorithm::inCollision(const amp::Problem2D& problem, double step_size) {
    return false;
    // Checks the next_position for collision with obstacles
    for(const auto& obs : problem.obstacles) {
        for(const auto& vertex : obs.verticesCCW()) {
            if((next_position - vertex).norm() < step_size) {
                return true;
            }
        }
    }
    return false;
}


/// Collision checker thoughts:
// Loop through the vertices in counter-clockwise order and create a function for the line that defines the edge
// Then quantify which side is the obstacle
