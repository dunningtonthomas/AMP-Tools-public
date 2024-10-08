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

    // Max iterations
    int max_iterations = 10000;
    int iter = 1;

    // Bug 1 Algorithm
    while(true) {
        iter++;

        // Set heading towards the goal
        heading = (problem.q_goal - current_position).normalized() * step_size;
        right_heading = Eigen::Vector2d(heading.y(), -heading.x());

        // Check if we are colliding with an obstacle
        if(inCollision(problem, heading, collision_vertex, collision_obstacle)) {
            //Bug1Traversal_Distance(path, problem);
            Bug2Traversal(path, problem);
        } else {
            // No collision, move toward the goal
            current_position += heading;
            path.waypoints.push_back(current_position);
        }

        // Check if we are at the goal
        if((path.waypoints.back() - problem.q_goal).norm() < step_size || iter > max_iterations) {
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
    double heading_extension = 2;

    // Addition to get the next vertex of the obstacle
    int vertex_add_index = 1;

    // Distance
    double minimum_distance = (problem.q_goal - current_position).norm();
    double distance = minimum_distance;
    Eigen::Vector2d closest_point = current_position;

    // Vector to store the entire path, used to get back to the closest position
    std::vector<Eigen::Vector2d> path_vector;
    int minimum_distance_index = 0;

    // Navigate around the obstacle
    int max_iterations = 10000;
    int iter = 0;
    //std::cout << "Entering Bug1 loop" << std::endl;
    while(true) {
        iter++;
        //std::cout << "Iter: " << iter << std::endl;

        // Check if we are colliding with another obstacle
        if(inCollision(problem, heading_extension*heading, collision_index, collision_obstacle)) {
            // New collision, update the heading
            obstacle_vertices = collision_obstacle.verticesCW();
            p1 = obstacle_vertices[collision_index];
            p2 = obstacle_vertices[(collision_index+1) % obstacle_vertices.size()];   
            edge = (p2 - p1).normalized();
            vertex_add_index = 1;   // Reset the addition to the vertex index
        } 

        // Update heading
        heading = edge * step_size;
        right_heading = Eigen::Vector2d(heading.y(), -heading.x()).normalized() * step_size;
        
        // Move forward
        current_position += heading;
        path.waypoints.push_back(current_position);
        path_vector.push_back(current_position);

        // Update the minimum distance
        distance = (problem.q_goal - current_position).norm();
        if(distance < minimum_distance) {
            minimum_distance = distance;
            closest_point = current_position;
            minimum_distance_index = path_vector.size() - 1;
        }

        // Check if the distance from the original collision vertex is greater than the edge length
        if(distance_func(p1, current_position) > sqrt(pow((p2 - p1).norm(),2) + pow(heading_extension*step_size,2))) {
            // No longer intersecting, rotate to align with the next vertex
            //std::cout << "Right heading no longer intersects" << std::endl;
            p1 = obstacle_vertices[(collision_index + vertex_add_index) % obstacle_vertices.size()];
            p2 = obstacle_vertices[(collision_index + vertex_add_index + 1) % obstacle_vertices.size()];   
            edge = (p2 - p1).normalized();
            vertex_add_index++;

            // Update heading
            heading = edge * step_size;
            right_heading = Eigen::Vector2d(heading.y(), -heading.x()).normalized() * step_size;

            // Check if we are colliding with another obstacle
            if(inCollision(problem, heading_extension*heading, collision_index, collision_obstacle)) {
                // New collision, update the heading
                obstacle_vertices = collision_obstacle.verticesCW();
                p1 = obstacle_vertices[collision_index];
                p2 = obstacle_vertices[(collision_index+1) % obstacle_vertices.size()];   
                edge = (p2 - p1).normalized();
                vertex_add_index = 1;   // Reset the addition to the vertex index
            }
            // Update heading
            heading = edge * step_size;
            right_heading = Eigen::Vector2d(heading.y(), -heading.x()).normalized() * step_size;
            
            // Move forward
            current_position += heading;
            path.waypoints.push_back(current_position);
            path_vector.push_back(current_position);

            // Update the minimum distance
            distance = (problem.q_goal - current_position).norm();
            if(distance < minimum_distance) {
                minimum_distance = distance;
                closest_point = current_position;
                minimum_distance_index = path_vector.size() - 1;
            }
        }

        // Check if we are back at the first hit point
        if(((path.waypoints.back() - hit_point).norm() < 2*step_size && iter > 2) || iter > max_iterations) {
            vertex_add_index = 1;
            std::cout << "Exited Bug1 at " << iter << " iterations" << std::endl;
            break;
        }
    }

    // Go to the closest point to the goal, follow the path vector left or right depending on which is closer
    if(minimum_distance_index < path_vector.size() / 2.0) {
        // Go to the left
        for(int i = 0; i <= minimum_distance_index; i++) {
            current_position = path_vector[i];
            path.waypoints.push_back(current_position);
        }
    } else {
        // Go to the right
        for(int i = path_vector.size() - 1; i >= minimum_distance_index; i--) {
            current_position = path_vector[i];
            path.waypoints.push_back(current_position);
        }
    }
}


// Bug 2 Traversal
void MyBugAlgorithm::Bug2Traversal(amp::Path2D& path, const amp::Problem2D& problem) {
    // Bug 1 Obstacle Traversal, move around the obstacle and go to the closest point to the goal
    amp::Obstacle2D collision_obstacle;
    int collision_index;
    inCollision(problem, heading, collision_index, collision_obstacle);
    std::vector<Eigen::Vector2d> obstacle_vertices = collision_obstacle.verticesCW();

    // Set initial hit point
    Eigen::Vector2d hit_point = current_position;

    // Create vector of the edge of the obstacle using the collision vertices
    Eigen::Vector2d p1 = obstacle_vertices[collision_index];
    Eigen::Vector2d p2 = obstacle_vertices[(collision_index+1) % obstacle_vertices.size()];   
    Eigen::Vector2d edge = (p2 - p1).normalized();

    // Set heading to be collinear with the edge
    heading = edge * step_size;
    right_heading = Eigen::Vector2d(heading.y(), -heading.x());
    double heading_extension = 2;

    // Addition to get the next vertex of the obstacle
    int vertex_add_index = 1;

    // Distance
    double initial_distance = (problem.q_goal - current_position).norm();

    // Navigate around the obstacle
    int max_iterations = 10000;
    int iter = 0;
    while(true) {
        iter++;
        // Check if we are colliding with another obstacle
        if(inCollision(problem, heading_extension*heading, collision_index, collision_obstacle)) {
            // New collision, update the heading
            obstacle_vertices = collision_obstacle.verticesCW();
            p1 = obstacle_vertices[collision_index];
            p2 = obstacle_vertices[(collision_index+1) % obstacle_vertices.size()];   
            edge = (p2 - p1).normalized();
            vertex_add_index = 1;   // Reset the addition to the vertex index
        }

        // Update heading
        heading = edge * step_size;
        right_heading = Eigen::Vector2d(heading.y(), -heading.x()).normalized() * step_size;
        
        // Move forward
        current_position += heading;
        path.waypoints.push_back(current_position);

        // Check if we have intersected with the m-line
        if(intersect(current_position, current_position + heading_extension*heading, problem.q_init, problem.q_goal)) {
            // We have intersected with the m-line, move to the closest point to the goal
            //std::cout << "Intersected with m-line" << std::endl;
            if((problem.q_goal - current_position).norm() < initial_distance) {
                // Reached close point along the m-line, take one more step, break and return
                current_position += heading;
                path.waypoints.push_back(current_position);
                std::cout << "Exited Bug2 at " << iter << " iterations" << std::endl;
                break;
            }
        }

        // Check if the distance is large enough to switch to the next vertex
        if(distance_func(p1, current_position) > sqrt(pow((p2 - p1).norm(),2) + pow(heading_extension*step_size,2))) {
            // Switch to the next vertex of the obstacle
            p1 = obstacle_vertices[(collision_index + vertex_add_index) % obstacle_vertices.size()];
            p2 = obstacle_vertices[(collision_index + vertex_add_index + 1) % obstacle_vertices.size()];   
            edge = (p2 - p1).normalized();
            vertex_add_index++;

            // Update heading
            heading = edge * step_size;
            right_heading = Eigen::Vector2d(heading.y(), -heading.x()).normalized() * step_size;

            // Check if we are colliding with another obstacle
            if(inCollision(problem, heading_extension*heading, collision_index, collision_obstacle)) {
                // New collision, update the heading
                obstacle_vertices = collision_obstacle.verticesCW();
                p1 = obstacle_vertices[collision_index];
                p2 = obstacle_vertices[(collision_index+1) % obstacle_vertices.size()];   
                edge = (p2 - p1).normalized();
                vertex_add_index = 1;   // Reset the addition to the vertex index
            }
            // Update heading
            heading = edge * step_size;
            right_heading = Eigen::Vector2d(heading.y(), -heading.x()).normalized() * step_size;
        }

        // Check if we have intersected with the m-line
        if(intersect(current_position, current_position + heading_extension*heading, problem.q_init, problem.q_goal)) {
            // We have intersected with the m-line, move to the closest point to the goal
            //std::cout << "Intersected with m-line" << std::endl;
            if((problem.q_goal - current_position).norm() < initial_distance) {
                // Reached close point along the m-line, take one more step, break and return
                current_position += heading;
                path.waypoints.push_back(current_position);
                std::cout << "Exited Bug2 at " << iter << " iterations" << std::endl;
                break;
            }
        }

        // Check if we got back to the initial hit point
        if(((path.waypoints.back() - hit_point).norm() < 2*step_size && iter > 2) || iter > max_iterations) {
            vertex_add_index = 1;
            std::cout << "Exited Bug2 at " << iter << " iterations" << std::endl;
            break;
        }

        // Exit if reach the max iterations
        if(iter > max_iterations) {
            std::cout << "Exited Bug2 at " << iter << " iterations" << std::endl;
            break;
        }
    }
}


// Calculates the distance between two points
double MyBugAlgorithm::distance_func(Eigen::Vector2d p1, Eigen::Vector2d p2) {
    return sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2));
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


// Got the following functions from GeekforGeeks:
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

