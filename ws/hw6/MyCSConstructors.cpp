#include "MyCSConstructors.h"

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    
    // Discretize the environment into a grid
    double step_size_x = (env.x_max - env.x_min) / m_cells_per_dim;
    double step_size_y = (env.y_max - env.y_min) / m_cells_per_dim;

    // Loop through each cell and see if it is in collision
    for(std::size_t i = 0; i < m_cells_per_dim; i++) {
        for(std::size_t j = 0; j < m_cells_per_dim; j++) {
            // Get the center of the cell
            double x = env.x_min + (i + 0.5) * step_size_x;
            double y = env.y_min + (j + 0.5) * step_size_y;

            // Check if the point is in collision
            if(inCollision_point(env, Eigen::Vector2d(x, y))) {
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

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    // Implement your WaveFront algorithm here
    // This path should be in continuous space
    amp::Path2D path;
    path.waypoints.push_back(q_init);
    path.waypoints.push_back(q_goal);
    if (isManipulator) {
        Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }
    return path;
}
