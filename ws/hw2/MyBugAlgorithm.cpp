#include "MyBugAlgorithm.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;

    // Define the step size
    step_size = 0.1;

    // Starting Position
    current_position = problem.q_init;
    heading = (problem.q_goal - current_position).normalized() * step_size;
    right_heading = Eigen::Vector2d(heading.y(), -heading.x()) * step_size;
    path.waypoints.push_back(current_position);

    // Bug 1 Algorithm
    while(true) {
        // Update the direction to the goal
        Eigen::Vector2d dir = (problem.q_goal - current_position).normalized();

        // Calculate the next position towards the goal
        heading = step_size * dir;

        // Check if we are colliding with an obstacle
        if(inCollision(problem, heading)) {
            // Execute Bug 1 obstacle traversal
            //std::cout << "Collision detected" << std::endl;

            // Move toward the goal ANYWAY FOR NOW
            // NOte: Remove this later
            current_position += heading;
            path.waypoints.push_back(current_position);

            // Update right heading
            right_heading = Eigen::Vector2d(heading.y(), -heading.x());

            // Add point above collision point
            path.waypoints.push_back(current_position + Eigen::Vector2d(0, 0.5));


        } else {
            // Move toward the goal
            current_position += heading;
            path.waypoints.push_back(current_position);

            // Update right heading
            right_heading = Eigen::Vector2d(heading.y(), -heading.x());
        }
    
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



bool MyBugAlgorithm::inCollision(const amp::Problem2D& problem, Eigen::Vector2d dir) {
    // Check if there is a collision by checking if two line segments intersect
    // dir is a direction vector with some magnitude, check if it intersects with a line segment of an obstacle
    Eigen::Vector2d next_pos = current_position + dir;
    // std::cout << "Current pos: " << current_position.x() << " " << current_position.y() << std::endl;
    // std::cout << "Next pos: " << next_pos.x() << " " << next_pos.y() << std::endl;
    //return false;

    for(const auto& obstacle : problem.obstacles) {
        std::vector<Eigen::Vector2d> vertices = obstacle.verticesCW();   // Clockwise vertices
        for(int i = 0; i < vertices.size(); i++) {
            // Get points clockwise around the obstacle
            Eigen::Vector2d p1 = vertices[i];
            Eigen::Vector2d p2 = vertices[(i + 1) % vertices.size()];

            // Check if the heading intersects with the obstacle
            if(intersect(current_position, next_pos, p1, p2)) {
                return true;
            }
        }
    }
    return false;
}


// Given three collinear points p, q, r, the function checks if 
// point q lies on line segment 'pr' 
bool MyBugAlgorithm::onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r) 
{ 
    if (q.x() <= std::max(p.x(), r.x()) && q.x() >= std::min(p.x(), r.x()) && 
        q.y() <= std::max(p.y(), r.y()) && q.y() >= std::min(p.y(), r.y())) {
        return true;
    } 
  
    return false; 
} 

// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are collinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int MyBugAlgorithm::orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r) 
{ 
    // See https://www.geeksforgeeks.org/orientation-3-ordered-points/ 
    // for details of below formula. 
    int val = (q.y() - p.y()) * (r.x() - q.x()) - 
              (q.x() - p.x()) * (r.y() - q.y()); 
  
    if (val == 0) return 0;  // collinear 
  
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 


// The main function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
bool MyBugAlgorithm::intersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2)
{ 
    // Find the four orientations needed for general and 
    // special cases 
    int o1 = orientation(p1, q1, p2); 
    int o2 = orientation(p1, q1, q2); 
    int o3 = orientation(p2, q2, p1); 
    int o4 = orientation(p2, q2, q1); 
  
    // General case 
    if (o1 != o2 && o3 != o4) 
        return true; 
  
    // Special Cases 
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)) return true; 
  
    // p1, q1 and q2 are collinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)) return true; 
  
    // p2, q2 and p1 are collinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true; 
  
     // p2, q2 and q1 are collinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; 
  
    return false; // Doesn't fall in any of the above cases 
} 

