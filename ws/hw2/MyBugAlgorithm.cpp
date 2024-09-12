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

    // Local variables
    int collision_vertex;
    amp::Obstacle2D collision_obstacle;

    // Bug 1 Algorithm
    while(true) {
        // Set heading towards the goal
        heading = (problem.q_goal - current_position).normalized() * step_size;
        right_heading = Eigen::Vector2d(heading.y(), -heading.x());

        // Check if we are colliding with an obstacle
        if(inCollision(problem, heading, collision_vertex, collision_obstacle)) {
            // Execute Bug 1 obstacle traversal
            //std::cout << "Collision detected" << std::endl;
            Bug1Traversal(path, problem);
            break;
            
            //Bug1Traversal(path, problem);
            //current_position += heading;
            //path.waypoints.push_back(current_position);

            // Move toward the goal ANYWAY FOR NOW
            // NOte: Remove this later
            //current_position += heading;
            //path.waypoints.push_back(current_position);

            // Add point above collision point
            //path.waypoints.push_back(current_position + Eigen::Vector2d(0, 0.5));
        } else {
            // No collision, move toward the goal
            current_position += heading;
            path.waypoints.push_back(current_position);
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
    // Bug 1 Obstacle Traversal, move around the obstacle and go to the closest point to the goal
    amp::Obstacle2D collision_obstacle;
    int collision_index;
    inCollision(problem, heading, collision_index, collision_obstacle);
    std::vector<Eigen::Vector2d> obstacle_vertices = collision_obstacle.verticesCW();

    // Right heading collision objects
    int collision_index_right;
    amp::Obstacle2D collision_obstacle_right;

    // Set initial hit point
    Eigen::Vector2d hit_point = current_position;

    // Create vector of the edge of the obstacle using the collision vertices
    Eigen::Vector2d p1 = obstacle_vertices[collision_index];
    Eigen::Vector2d p2 = obstacle_vertices[(collision_index+1) % obstacle_vertices.size()];   
    Eigen::Vector2d edge = (p2 - p1).normalized();

    // Set heading to be collinear with the edge
    heading = edge * step_size;
    right_heading = Eigen::Vector2d(heading.y(), -heading.x());

    // Addition to get the next vertex of the obstacle
    int vertex_add_index = 1;

    // Navigate around the obstacle
    int max_iterations = 1000;
    int iter = 0;
    std::cout << "Entering Bug1 loop" << std::endl;
    while(true) {
        iter++;
        //std::cout << "Iter: " << iter << std::endl;
        // Update heading
        heading = edge * step_size;
        right_heading = Eigen::Vector2d(heading.y(), -heading.x()).normalized() * 1 * step_size;

        // Check if we are colliding with another obstacle
        if(inCollision(problem, heading, collision_index, collision_obstacle)) {
            // New collision, update the heading
            obstacle_vertices = collision_obstacle.verticesCW();
            p1 = obstacle_vertices[collision_index];
            p2 = obstacle_vertices[(collision_index+1) % obstacle_vertices.size()];   
            edge = (p2 - p1).normalized();
            vertex_add_index = 1;   // Reset the addition to the vertex index
            continue;   // Traverse around new obstacle
        } else {
            // No collision, move forward
            current_position += heading;
            path.waypoints.push_back(current_position);
        }

        // Move forward
        // current_position += heading;
        // path.waypoints.push_back(current_position);

        // Check if the right_heading no longer intersects with the obstacle
        if(!inCollision(problem, right_heading, collision_index_right, collision_obstacle_right)) {
            // No longer intersecting, rotate to align with the next vertex
            std::cout << "Right heading no longer intersects" << std::endl;
            p1 = obstacle_vertices[(collision_index + vertex_add_index) % obstacle_vertices.size()];
            p2 = obstacle_vertices[(collision_index + vertex_add_index + 1) % obstacle_vertices.size()];   
            edge = (p2 - p1).normalized();
            vertex_add_index++;
        }


        // Check if we are back at the first hit point
        if((path.waypoints.back() - hit_point).norm() < 0.9*step_size || iter > max_iterations) {
            vertex_add_index = 1;
            std::cout << "Exited Bug1 at " << iter << " iterations" << std::endl;
            break;
        }
    }
}

// Write a function that calculates the angle between the heading and the the edge of the obstacle
void MyBugAlgorithm::rotateHeading(double anlge) {
    // Rotate the heading by the angle
    double x = heading.x() * cos(anlge) - heading.y() * sin(anlge);
    double y = heading.x() * sin(anlge) + heading.y() * cos(anlge);
    heading = Eigen::Vector2d(x, y);

    // Set the right_heading
    right_heading = Eigen::Vector2d(heading.y(), -heading.x());
}


bool MyBugAlgorithm::inCollision(const amp::Problem2D& problem, Eigen::Vector2d dir, int& collision_index, amp::Obstacle2D& collision_obstacle) {
    // Check if there is a collision by checking if two line segments intersect
    // dir is a direction vector with some magnitude, check if it intersects with a line segment of an obstacle
    Eigen::Vector2d next_pos = current_position + dir;
    // std::cout << "Current pos: " << current_position.x() << " " << current_position.y() << std::endl;
    // std::cout << "Next pos: " << next_pos.x() << " " << next_pos.y() << std::endl;

    for(const auto& obstacle : problem.obstacles) {
        std::vector<Eigen::Vector2d> vertices = obstacle.verticesCW();   // Clockwise vertices
        for(int i = 0; i < vertices.size(); i++) {
            // Get points clockwise around the obstacle
            Eigen::Vector2d p2 = vertices[i];
            Eigen::Vector2d q2 = vertices[(i + 1) % vertices.size()];

            // Check if the heading intersects with the obstacle
            if(intersect(current_position, next_pos, p2, q2)) {
                collision_index = i;
                collision_obstacle = obstacle;
                return true;
            }
        }
    }
    return false;
}



// CHAD:
// Helper function to check the orientation of the triplet (p, q, r)
int MyBugAlgorithm::orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r) {
    // Calculate the determinant of the matrix formed by the vectors pq and qr
    double val = (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y());
    
    if (val == 0) return 0; // collinear
    return (val > 0) ? 1 : 2; // clock or counterclock wise
}

// Check if point q lies on segment pr
bool MyBugAlgorithm::onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r) {
    return q.x() <= std::max(p.x(), r.x()) && q.x() >= std::min(p.x(), r.x()) &&
           q.y() <= std::max(p.y(), r.y()) && q.y() >= std::min(p.y(), r.y());
}

// Main function to check if line segments (p1, q1) and (p2, q2) intersect
bool MyBugAlgorithm::intersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2) {
    // Find the four orientations needed for the general and special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
    
    // General case: different orientations indicate intersection
    if (o1 != o2 && o3 != o4)
        return true;
    
    // Special Cases: when points are collinear and lie on the segment
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
    
    return false; // No intersection
}



//ORIGINAL:
/*
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
*/
