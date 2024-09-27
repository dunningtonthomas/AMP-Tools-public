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

} // namespace amp

