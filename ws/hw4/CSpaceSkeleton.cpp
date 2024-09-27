#include "CSpaceSkeleton.h"

// Override this method for returning whether or not a point is in collision

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Floor the inputs to the nearest cell
    std::pair<double, double> x0_bounds = x0Bounds();
    std::pair<double, double> x1_bounds = x1Bounds();
    std::pair<std::size_t, std::size_t> num_cells = size();

    // Get the cell size
    double x0_cell_size = (x0_bounds.second - x0_bounds.first) / num_cells.first;
    double x1_cell_size = (x1_bounds.second - x1_bounds.first) / num_cells.second;

    // Get the cell index
    std::size_t cell_x = static_cast<int>((x0 - x0_bounds.first) / x0_cell_size);
    std::size_t cell_y = static_cast<int>((x1 - x1_bounds.first) / x1_cell_size);

    return {cell_x, cell_y};
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    //std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, 0.0, 2*M_PI, 0.0, 2*M_PI);
    //std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, -M_PI, M_PI, -M_PI, M_PI);
    //std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, 0.0, 100, 0.0, 100);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;

    // Iterate through each joint configuration and check if it is in collision
    double step_size = 2 * M_PI / m_cells_per_dim;
    for(std::size_t i = 0; i < m_cells_per_dim; i++) {
        for(std::size_t j = 0; j < m_cells_per_dim; j++) {
            // Get the angle from the cell
            double x0 = i * step_size;
            double x1 = j * step_size;

            // Get the joint angles from the continuous point
            amp::ManipulatorState joint_angles(2);
            joint_angles << x0, x1;

            // Check if the manipulator is in collision
            if(inCollision_manipulator(env, manipulator, joint_angles)) {
                cspace(i, j) = true;
            } else {
                cspace(i, j) = false;
            }
        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}



// @brief Collision Checker for manipulator using line segment intersection
bool MyManipulatorCSConstructor::inCollision_manipulator(const amp::Environment2D& env, const amp::LinkManipulator2D& manipulator, amp::ManipulatorState joint_angles) {
    // Check if the manipulator intersects with any of the obstacles
    for(const auto& obstacle : env.obstacles) {
        std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();   // vertices
        for(int i = 0; i < vertices.size(); i++) {
            // Get points around the obstacle
            Eigen::Vector2d p2 = vertices[i];
            Eigen::Vector2d q2 = vertices[(i + 1) % vertices.size()];

            // Check if the manipulator intersects with the obstacle
            for(int j = 0; j < manipulator.nLinks(); j++) {
                Eigen::Vector2d p1 = manipulator.getJointLocation(joint_angles, j);
                Eigen::Vector2d q1 = manipulator.getJointLocation(joint_angles, j+1);

                if(amp::intersect(p1, q1, p2, q2)) {
                    return true;
                }
            }
        }
    }
    return false;
}