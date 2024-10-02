#include "MyGDAlgorithm.h"

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(Eigen::Vector2d(1.0, 5.0));
    path.waypoints.push_back(Eigen::Vector2d(3.0, 9.0));
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

    // Define Uatt
    if(distanceBetween(q, q_goal) <= d_star) {
        Uatt = d_star * zetta * distanceBetween(q, q_goal) - 0.5 * zetta * d_star * d_star;
    } else {
        Uatt = 0.5 * zetta*distanceBetween(q, q_goal) * distanceBetween(q, q_goal);
    }

    // Define Urep
    double Urep = 0;
    for(auto& obstacle : problem.obstacles) {
        double distance = distanceToObstacle(q, obstacle);
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



double MyPotentialFunction::distanceToObstacle(const Eigen::Vector2d& q, const amp::Obstacle2D& obstacle) const{
    // Find the minimum distance to an obstacle edge
    double min_distance = std::numeric_limits<double>::max();
    double distance;
    std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();

    // Loop through all vertices of the obstacle
    for(int i = 0; i < vertices.size(); i++) {
        Eigen::Vector2d A = vertices[i];
        Eigen::Vector2d B = vertices[(i+1) % vertices.size()];

        // Calculate distance to the line segment
        distance = minDistance(A, B, q);
        if(distance < min_distance) {
            min_distance = distance;
        }
    }
    return min_distance;
}


// Got this function from Geeks4Geeks
double MyPotentialFunction::minDistance(const Eigen::Vector2d A, const Eigen::Vector2d B, const Eigen::Vector2d E) const{
    // vector AB
    Eigen::Vector2d AB(B - A);
 
    // vector BP
    Eigen::Vector2d BE(E - B);
 
    // vector AE
    Eigen::Vector2d AE(E - A);
 
    // Variables to store dot product
    double AB_BE, AB_AE;
 
    // Calculating the dot product
    AB_BE = AB.dot(BE);
    AB_AE = AB.dot(AE);
 
    // Minimum distance from
    // point E to the line segment
    double reqAns = 0;
 
    // Case 1
    if (AB_BE > 0) {
        // Finding the magnitude
        Eigen::Vector2d C(E - B);
        reqAns = C.norm();
    }
    // Case 2
    else if (AB_AE < 0) {
        Eigen::Vector2d C(E - A);
        reqAns = C.norm();
    }
    // Case 3
    else {
        // Finding the perpendicular distance
        double x1 = AB.x();
        double y1 = AB.y();
        double x2 = AE.x();
        double y2 = AE.y();
        double mod = sqrt(x1 * x1 + y1 * y1);
        reqAns = abs(x1 * y2 - y1 * x2) / mod;
    }
    return reqAns;
}