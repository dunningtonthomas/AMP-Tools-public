#include "helperFuncs.h"

namespace amp {

    // @brief Calculate the minkowski sum of two polygons
    Polygon minkowski(amp::Polygon& obstacle, amp::Polygon& negative_object) {
        // Get the vertices of the obstacle and sort
        std::vector<Eigen::Vector2d> obstacle_vertices = sortVerticesCCW(obstacle.verticesCCW());
        std::vector<Eigen::Vector2d> negative_object_vertices = sortVerticesCCW(negative_object.verticesCCW());

        // Calculate minkowski sum
        std::vector<Eigen::Vector2d> minkowski_vertices;
        double angle1 = 0, angle2 = 0;
        int i = 0, j = 0;

        while (i < obstacle_vertices.size()+1 && j < negative_object_vertices.size()+1) {
            // Add the vertices to the minkowski sum
            minkowski_vertices.push_back(obstacle_vertices[i % obstacle_vertices.size()] + negative_object_vertices[j % negative_object_vertices.size()]);

            // Calculate the two angles for the obstacles
            angle1 = angle(obstacle_vertices[i % obstacle_vertices.size()], obstacle_vertices[(i+1) % obstacle_vertices.size()]);
            angle2 = angle(negative_object_vertices[j % negative_object_vertices.size()], negative_object_vertices[(j+1) % negative_object_vertices.size()]);

            // Increment
            if (angle1 < angle2) {
                i++;
            } else if (angle1 > angle2) {
                j++;
            } else {
                i++;
                j++;
            }
        }

        // Create the minkowski polygon
        Obstacle2D minkowski_polygon(minkowski_vertices);
        return minkowski_polygon;
    }

    // @brief Calculate the angle of a side of a polygon relative to horizontal
    double angle(Eigen::Vector2d v1, Eigen::Vector2d v2) {
        double ang = atan2(v2.y() - v1.y(), v2.x() - v1.x());
        if (ang < 0) {
            ang += 2 * M_PI;
        }
        return ang;
    }

    // @brief Sort the vertices of a polygon in counter clockwise order such that the first is the smallest y coordinate
    std::vector<Eigen::Vector2d> sortVerticesCCW(std::vector<Eigen::Vector2d> vertices) {
        // Find the vertex with the smallest y coordinate
        Eigen::Vector2d min_vertex = vertices[0];
        int min_index = 0;

        for (int i = 1; i < vertices.size(); i++) {
            if (vertices[i].y() < min_vertex.y()) {
                min_vertex = vertices[i];
                min_index = i;
            }
        }

        // Resort the vertices so that the smallest y coordinate is first and it is still ccw
        std::vector<Eigen::Vector2d> new_vertices;
        for (int i = 0; i < vertices.size(); i++) {
            new_vertices.push_back(vertices[(min_index + i) % vertices.size()]);
        }
        return new_vertices;
    }


    // @brief Print the vertices of an obstacle
    void printVertices(std::vector<Eigen::Vector2d>& vertices) {
        for (const Eigen::Vector2d& vertex : vertices) {
            std::cout << "(" << vertex.x() << ", " << vertex.y() << ")" << std::endl;
        }
    }   

    // @brief Rotate a polygon by a given angle
    Polygon rotatePolygon(amp::Polygon& obstacle, double angle) {
        std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
        std::vector<Eigen::Vector2d> rotated_vertices;
        Eigen::Rotation2Dd rot(angle);

        for (const Eigen::Vector2d& vertex : vertices) {
                
            rotated_vertices.push_back(rot * vertex);
        }

        return Polygon(rotated_vertices);
    }

    // @brief Return the negative of the vertices of a polygon
    std::vector<Eigen::Vector2d> negativeVertices(std::vector<Eigen::Vector2d>& vertices) {
        std::vector<Eigen::Vector2d> negative_vertices;
        for (const Eigen::Vector2d& vertex : vertices) {
            negative_vertices.push_back(-vertex);
        }
        return negative_vertices;
    }

    // Main function to check if line segments (p1, q1) and (p2, q2) intersect
    bool intersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2) {
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

    // Check if point q lies on segment pr
    bool onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r) {
        return q.x() <= std::max(p.x(), r.x()) && q.x() >= std::min(p.x(), r.x()) &&
            q.y() <= std::max(p.y(), r.y()) && q.y() >= std::min(p.y(), r.y());
    }

    // Helper function to check the orientation of the triplet (p, q, r)
    int orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r) {
        // Calculate the determinant of the matrix formed by the vectors pq and qr
        double val = (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y());
        
        if (val == 0) return 0; // collinear
        return (val > 0) ? 1 : 2; // clock or counterclock wise
    }

    // @brief Check if the next position is in collision with an obstacle
    bool inCollision(const amp::Problem2D& problem, Eigen::Vector2d current_pos, Eigen::Vector2d next_pos) {
        // Check if there is a collision by checking if two line segments intersect
        for(const auto& obstacle : problem.obstacles) {
            std::vector<Eigen::Vector2d> vertices = obstacle.verticesCW();   // Clockwise vertices
            for(int i = 0; i < vertices.size(); i++) {
                // Get points clockwise around the obstacle
                Eigen::Vector2d p2 = vertices[i];
                Eigen::Vector2d q2 = vertices[(i + 1) % vertices.size()];

                // Check if the heading intersects with the obstacle
                if(intersect(current_pos, next_pos, p2, q2)) {
                    return true;
                }
            }
        }
        return false;
    }

   // @brief Check if the next position is in collision with an obstacle, return the index of the obstacle and the obstacle itself
    bool inCollision(const amp::Problem2D& problem, Eigen::Vector2d current_pos, Eigen::Vector2d next_pos, int& collision_index, amp::Obstacle2D& collision_obstacle) {
        // Check if there is a collision by checking if two line segments intersect
        for(const auto& obstacle : problem.obstacles) {
            std::vector<Eigen::Vector2d> vertices = obstacle.verticesCW();   // Clockwise vertices
            for(int i = 0; i < vertices.size(); i++) {
                // Get points clockwise around the obstacle
                Eigen::Vector2d p2 = vertices[i];
                Eigen::Vector2d q2 = vertices[(i + 1) % vertices.size()];

                // Check if the heading intersects with the obstacle
                if(intersect(current_pos, next_pos, p2, q2)) {
                    collision_index = i;
                    collision_obstacle = obstacle;
                    return true;
                }
            }
        }
        return false;
    }

    // @brief Collision checker for in polygon point robot detection, from Geeks4Geeks
    bool isInsidePolygon(const amp::Polygon& Obstacle, Eigen::Vector2d p) {
        // Get the vertices
        std::vector<Eigen::Vector2d> vertices = Obstacle.verticesCCW();

        // Number of vertices
        int n = vertices.size();

        // Create a point for the ray cast (far right)
        Eigen::Vector2d extreme(10000, p.y());

        // Count intersections of the polygon edges with the ray
        int count = 0, i = 0;
        do {
            int next_index = (i + 1) % n;

            // Check if the line segment from polygon[i] to polygon[next] intersects the ray
            if (intersect(vertices[i], vertices[next_index], p, extreme)) {
                // Check if the point is collinear with a polygon edge
                if (orientation(vertices[i], p, vertices[next_index]) == 0) {
                    return onSegment(vertices[i], p, vertices[next_index]);
                }
                count++;
            }
            i = next_index;
        } while (i != 0);

        // Return true if the count is odd (point is inside), false otherwise
        return (count % 2 == 1);
    }

    // @brief Generate a random configuration in the workspace
    Eigen::Vector2d randomConfiguration(double x_min, double x_max, double y_min, double y_max) {
        // Create a random number generator
        std::random_device rd;  // Random seed
        std::mt19937 gen(rd()); // Mersenne Twister generator

        // Define the uniform distribution between -5 and 5
        std::uniform_real_distribution<double> x_rand(x_min, x_max);
        std::uniform_real_distribution<double> y_rand(y_min, y_max);

        // Sample a random point in the environment
        Eigen::Vector2d q_rand(x_rand(gen), y_rand(gen));
        return q_rand;
    }

    // @brief Generate a random number between min and max
    double randomDouble(double min, double max) {
        // Create a random number generator
        std::random_device rd;  // Random seed
        std::mt19937 gen(rd()); // Mersenne Twister generator

        // Define the uniform distribution between 0 and 1
        std::uniform_real_distribution<double> dis(min, max);

        // Sample a random number
        return dis(gen);
    }

    // @brief Function to check if a point is in collision with an obstacle
    bool inCollision_point(const amp::Environment2D& env, Eigen::Vector2d point) {
        // Loop through all the obstacles
        for(const auto& obstacle : env.obstacles) {
            // Check if the point is inside the obstacle
            if(isInsidePolygon(obstacle, point)) {
                return true;
            }
        }
        return false;
    }

    // @brief Calculates the distance between two points
    double distance(Eigen::Vector2d p1, Eigen::Vector2d p2) {
        return (p1 - p2).norm();
    }

    // @brief Calculate the minimum distance from a point to an obstacle
    double distanceToObstacle(const Eigen::Vector2d& q, const amp::Obstacle2D& obstacle, Eigen::Vector2d& closest_point) {
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

    // @brief find minimum distance from a point to a line, Got this function from Geeks4Geeks
    double minDistanceToLine(const Eigen::Vector2d A, const Eigen::Vector2d B, const Eigen::Vector2d P, Eigen::Vector2d& closest_point) {
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

} // namespace amp

