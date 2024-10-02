#include "MyGDAlgorithm.h"

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    // Create a path object to store the waypoints
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    // Gradient Descend Algorithm
    Eigen::Vector2d q_current = problem.q_init;
    Eigen::Vector2d q_next;
    double step_size = 0.05;
    double epsilon = 0.01;
    int max_iterations = 5000;
    int iterations = 0;

    // Create the potential function
    MyPotentialFunction potential_function(problem, d_star, zetta, Q_star, eta);

    // Loop until the goal is reached or the maximum number of iterations is reached
    while((q_current - problem.q_goal).norm() > epsilon && iterations < max_iterations) {
        // Calculate the gradient of the potential function at the current point
        Eigen::Vector2d gradient = potential_function.gradient(q_current);

        // Update the next point using the gradient and step size
        q_next = q_current - step_size * gradient;

        // Update the current point
        q_current = q_next;

        // Add the current point to the path
        path.waypoints.push_back(q_current);

        // Increment the iteration counter
        iterations++;
    }

    // Add the goal point to the path
    path.waypoints.push_back(problem.q_goal);

    return path;
}


double MyPotentialFunction::operator()(const Eigen::Vector2d& q) const {
    // Return the value of the potential function at the given point
    // std::vector<amp::Obstacle2D> obstacles;
    // Eigen::Vector2d goal;
    // d_star, zetta, Q_star, eta
    const Eigen::Vector2d q_goal = problem.q_goal;
    double Uatt = 0;
    Eigen::Vector2d closest_point;

    // Define Uatt
    if(distanceBetween(q, q_goal) <= d_star) {
        Uatt = d_star * zetta * distanceBetween(q, q_goal) - 0.5 * zetta * d_star * d_star;
    } else {
        Uatt = 0.5 * zetta*distanceBetween(q, q_goal) * distanceBetween(q, q_goal);
    }

    // Define Urep
    double Urep = 0;
    for(auto& obstacle : problem.obstacles) {
        double distance = distanceToObstacle(q, obstacle, closest_point);
        if(distance <= Q_star) {
            Urep += 0.5 * eta * (1/distance - 1/Q_star) * (1/distance - 1/Q_star);
        }
    }

    return Uatt + Urep;
}



// Distance between two points
double MyPotentialFunction::distanceBetween(const Eigen::Vector2d& q1, const Eigen::Vector2d& q2) const{
    return (q1 - q2).norm();
}



double MyPotentialFunction::distanceToObstacle(const Eigen::Vector2d& q, const amp::Obstacle2D& obstacle, Eigen::Vector2d& closest_point) const{
    // Find the minimum distance to an obstacle edge
    double min_distance = std::numeric_limits<double>::max();
    double distance;
    std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();

    // Loop through all vertices of the obstacle
    for(int i = 0; i < vertices.size(); i++) {
        Eigen::Vector2d A = vertices[i];
        Eigen::Vector2d B = vertices[(i+1) % vertices.size()];

        // Calculate distance to the line segment
        distance = minDistanceToLine(A, B, q, closest_point);
        if(distance < min_distance) {
            min_distance = distance;
        }
    }
    return min_distance;
}


// Got this function from Geeks4Geeks
double MyPotentialFunction::minDistanceToLine(const Eigen::Vector2d A, const Eigen::Vector2d B, const Eigen::Vector2d P, Eigen::Vector2d& closest_point) const{
    // vector AB and AP
    Eigen::Vector2d AB(B - A);
    Eigen::Vector2d AP(P - A);

    // Project AP onto AB
    double projection = AP.dot(AB) / (AB.dot(AB));

    // Clamp the projection to between 0 and 1
    projection = std::max(0.0, std::min(1.0, projection));

    // Calculate the closest point
    closest_point = A + projection * AB;

    // Return the distance
    return (P - closest_point).norm();
}

// Calculate the gradient of the potential function at a given point
Eigen::Vector2d MyPotentialFunction::gradient(const Eigen::Vector2d& q) const {
    // Calculate the gradient of the potential function at the given point
    Eigen::Vector2d gradient;
    gradient.setZero();

    // Calculate the gradient of the potential function at the given point
    const Eigen::Vector2d q_goal = problem.q_goal;

    // Calculate the gradient of Uatt
    if(distanceBetween(q, q_goal) <= d_star) {
        gradient += zetta * (q - q_goal);
    } else {
        gradient += d_star * zetta * (q - q_goal) / distanceBetween(q, q_goal);
    }

    // Calculate the gradient of Urep
    for(auto& obstacle : problem.obstacles) {
        Eigen::Vector2d closest_point;
        double distance = distanceToObstacle(q, obstacle, closest_point);
        if(distance <= Q_star) {
            Eigen::Vector2d gradient_rep = (q - closest_point) / (distance);
            gradient += eta * -1 * gradient_rep / (distance * distance) * (1/distance - 1/Q_star);
        }
    }

    return gradient;
}