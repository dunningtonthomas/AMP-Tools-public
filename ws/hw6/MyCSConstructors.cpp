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


// @brief Function to check if a point is in collision with an obstacle
bool MyPointAgentCSConstructor::inCollision_point(const amp::Environment2D& env, const Eigen::Vector2d& point) {
    // Loop through all the obstacles
    for(const auto& obstacle : env.obstacles) {
        // Check if the point is inside the obstacle
        if(isInsidePolygon(obstacle, point)) {
            return true;
        }
    }
    return false;
}


amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    // Implement your WaveFront algorithm here
    // This path should be in continuous space
    amp::Path2D path;
    path.waypoints.push_back(q_init);
    path.waypoints.push_back(q_goal);

    // Get the number of cells in the C-space
    std::pair<std::size_t, std::size_t> n_cells = grid_cspace.size();
    int distanceArr[n_cells.first][n_cells.second];

    // Loop through the C-space and assign 1 to all obstacles, 0 to all free cells, and 2 to the goal cell
    for(std::size_t i = 0; i < n_cells.first; i++) {
        for(std::size_t j = 0; j < n_cells.second; j++) {
            if(grid_cspace(i, j)) {
                distanceArr[i][j] = 1;
            } else {
                distanceArr[i][j] = 0;
            }
        }
    }

    // Set the goal cell to 2
    std::pair<std::size_t, std::size_t> goal_cell = grid_cspace.getCellFromPoint(q_goal(0), q_goal(1));
    distanceArr[goal_cell.first][goal_cell.second] = 2;

    // Create a queue of cell locations
    std::queue<std::pair<std::size_t, std::size_t>> q;
    q.push(goal_cell);

    // Execute the brushfire algorithm to assign distances to each cell
    while(!q.empty()) {
        std::pair<std::size_t, std::size_t> cell = q.front();
        q.pop();

        // Populate the neighbors distances
        if(cell.first > 0 && distanceArr[cell.first-1][cell.second] == 0) {
            distanceArr[cell.first-1][cell.second] = distanceArr[cell.first][cell.second] + 1;
            q.push({cell.first-1, cell.second});
        }
        if(cell.first < n_cells.first - 1 && distanceArr[cell.first+1][cell.second] == 0) {
            distanceArr[cell.first+1][cell.second] = distanceArr[cell.first][cell.second] + 1;
            q.push({cell.first+1, cell.second});
        }
        if(cell.second > 0 && distanceArr[cell.first][cell.second-1] == 0) {
            distanceArr[cell.first][cell.second-1] = distanceArr[cell.first][cell.second] + 1;
            q.push({cell.first, cell.second-1});
        }
        if(cell.second < n_cells.second - 1 && distanceArr[cell.first][cell.second+1] == 0) {
            distanceArr[cell.first][cell.second+1] = distanceArr[cell.first][cell.second] + 1;
            q.push({cell.first, cell.second+1});
        }
    }

    // Get the initial cell
    std::pair<std::size_t, std::size_t> init_cell = grid_cspace.getCellFromPoint(q_init(0), q_init(1));

    // Get the path from the initial cell to the goal cell
    std::pair<std::size_t, std::size_t> curr_cell = init_cell;
    bool found_goal = false;

    while(!found_goal) {
        // Loop through all neigbors and push them onto a vector
        std::vector<std::pair<std::size_t, std::size_t>> neighbors;
        std::vector<double> distance;

        // Add all neighbors to the vector
        if(curr_cell.first > 0) {
            neighbors.push_back({curr_cell.first-1, curr_cell.second});
            distance.push_back(distanceArr[curr_cell.first-1][curr_cell.second]);
        }
        if(curr_cell.first < n_cells.first - 1) {
            neighbors.push_back({curr_cell.first+1, curr_cell.second});
            distance.push_back(distanceArr[curr_cell.first+1][curr_cell.second]);
        }
        if(curr_cell.second > 0) {
            neighbors.push_back({curr_cell.first, curr_cell.second-1});
            distance.push_back(distanceArr[curr_cell.first][curr_cell.second-1]);
        }
        if(curr_cell.second < n_cells.second - 1) {
            neighbors.push_back({curr_cell.first, curr_cell.second+1});
            distance.push_back(distanceArr[curr_cell.first][curr_cell.second+1]);
        }


    }



    if (isManipulator) {
        Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }
    return path;
}
