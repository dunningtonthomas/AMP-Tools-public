#include "MyGDAlgorithm.h"

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    // Create a path object to store the waypoints
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    // Gradient Descent Algorithm
    Eigen::Vector2d q_current = problem.q_init;
    Eigen::Vector2d q_next;
    double step_size = 0.05;
    double epsilon = 0.25;
    int max_iterations = 20000;
    int iterations = 0;

    // Obstacle and obstacle index
    int collision_index;
    amp::Obstacle2D collision_obstacle;

    // Random walk variables
    bool rand_walk = false;

    // Create the potential function
    MyPotentialFunction potential_function(problem, d_star, zetta, Q_star, eta);

    // TODO: Step out of local minima by taking random step in a random direction that is not a collision
    // Can also try to implement a smart gradient descent that avoids obstacles by checking for collision

    // Loop until the goal is reached or the maximum number of iterations is reached
    while((q_current - problem.q_goal).norm() > epsilon && iterations < max_iterations) {
        // Calculate the gradient of the potential function at the current point
        Eigen::Vector2d gradient = potential_function.gradient(q_current);

        // Check to see if q_current is in the previous 5
        if(path.waypoints.size() > 6) {
            for(int i = 0; i < 5; i++) {
                if((q_current - path.waypoints[path.waypoints.size() - i - 2]).norm() < 0.01) {
                    // If the current point is in the previous 10 waypoints, take a random step
                    rand_walk = true;
                }
            }
        }

        // Take a random step if the gradient is too small or if rand_walk is true
        if (gradient.norm() < 0.001 || rand_walk) {
            std::cout << "Iteration: " << iterations << std::endl;
            rand_walk = false;
            // Take a random step that is not in collision
            int iter_inner = 0;
            while (iter_inner < 100) {
                Eigen::Vector2d random_step = Eigen::Vector2d::Random().normalized();
                q_next = q_current + 1 * random_step;
                if(!inCollision(problem, q_current, q_next, collision_index, collision_obstacle)) {
                    break;
                }
                iter_inner++;
            }
        } else if (gradient.norm() > 50) {
            // If the gradient is too large, take a smaller step with magnitude of 0.1
            q_next = q_current - 0.1 * gradient.normalized();
        }
        else {
            // Update the next point using the gradient and step size
            q_next = q_current - step_size * gradient; 
        }

        // Check if the next point is in collision
        if(inCollision(problem, q_current, q_next, collision_index, collision_obstacle)) {
            // If in collision take a step perpendicular to the intersected edge, COUNTER CLOCKWISE
            Eigen::Vector2d p2 = collision_obstacle.verticesCCW()[collision_index];
            Eigen::Vector2d q2 = collision_obstacle.verticesCCW()[(collision_index + 1) % collision_obstacle.verticesCCW().size()];
            Eigen::Vector2d edge = q2 - p2;
            Eigen::Vector2d normal = Eigen::Vector2d(-edge.y(), edge.x()).normalized();
            q_next = q_current + step_size * normal;
        }

        // Update the current point
        q_current = q_next;

        path.waypoints.push_back(q_current);

        // Increment the iteration counter
        iterations++;
    }

    // Add the goal point to the path
    if(iterations != max_iterations) {
        path.waypoints.push_back(problem.q_goal);
    }
    
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
        Uatt = 0.5 * zetta*distanceBetween(q, q_goal) * distanceBetween(q, q_goal);
    } else {
        Uatt = d_star * zetta * distanceBetween(q, q_goal) - 0.5 * zetta * d_star * d_star;
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

    // Calculate a tanget gradient component around the obstacles to cause circulation
    // double circulation_scale = 2.0;
    // for(auto& obstacle : problem.obstacles) {
    //     Eigen::Vector2d closest_point;
    //     double distance = distanceToObstacle(q, obstacle, closest_point);
    //     if(distance <= Q_star) {
    //         Eigen::Vector2d gradient_rep = (q - closest_point) / (distance);

    //         // Rotate the gradient_rep vector by 90 degree
    //         Eigen::Vector2d temp = gradient_rep;
    //         gradient_rep(0) = -temp(1);
    //         gradient_rep(1) = temp(0);

    //         gradient += circulation_scale * eta * gradient_rep / (distance * distance) * (1/distance - 1/Q_star);
    //     }
    // }

    return gradient;
}


Eigen::Vector2d MyPotentialFunction::getGradient(const Eigen::Vector2d& q) const {
    return gradient(q);
}