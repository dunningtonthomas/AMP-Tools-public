#include "createEnv.h"

amp::KinodynamicProblem2D generateEnv::getEnv1() {
    amp::KinodynamicProblem2D prob;
    prob.agent_type = amp::AgentType::SingleIntegrator;
    prob.isPointAgent = true;
    prob.q_init = Eigen::Vector2d(0.0, 0.0);
    prob.q_goal = {std::make_pair(290.0, 310.0), std::make_pair(290.0, 310.0)};
    prob.q_bounds = {std::make_pair(0.0, 400.0), std::make_pair(0.0, 400.0)};
    prob.u_bounds = {std::make_pair(-1.0, 1.0), std::make_pair(-1.0, 1.0)};
    prob.isDimCartesian = {true, true};

    // Set the environment bounds
    prob.x_min = 0.0;
    prob.x_max = 400.0;
    prob.y_min = 0.0;
    prob.y_max = 400.0;

    // Setup obstacles
    std::vector<Eigen::Vector2d> obstacle_vertices = {{33.0, 33.0}, {66.0, 33.0}, {66.0, 66.0}, {33.0, 66.0}};
    amp::Obstacle2D obstacle1(obstacle_vertices);
    obstacle_vertices = {{70.0, 100.0}, {130.0, 100.0}, {130.0, 130.0}, {70.0, 130.0}};
    amp::Obstacle2D obstacle2(obstacle_vertices);
    obstacle_vertices = {{210.0, 20.0}, {270.0, 20.0}, {270.0, 50.0}, {210.0, 50.0}};
    amp::Obstacle2D obstacle3(obstacle_vertices);
    obstacle_vertices = {{300.0, 100.0}, {330.0, 100.0}, {330.0, 160.0}, {300.0, 160.0}};
    amp::Obstacle2D obstacle4(obstacle_vertices);
    obstacle_vertices = {{200.0, 200.0}, {230.0, 200.0}, {230.0, 260.0}, {200.0, 260.0}};
    amp::Obstacle2D obstacle5(obstacle_vertices);
    
    prob.obstacles.push_back(obstacle1);
    prob.obstacles.push_back(obstacle2);
    prob.obstacles.push_back(obstacle3);
    prob.obstacles.push_back(obstacle4);
    prob.obstacles.push_back(obstacle5);

    return prob;
}

amp::KinodynamicProblem2D generateEnv::getEnvRandKino() {
    amp::KinodynamicProblem2D prob;
    prob.agent_type = amp::AgentType::SingleIntegrator;
    prob.isPointAgent = true;
    prob.q_init = Eigen::Vector2d(5.0, 5.0);
    prob.q_goal = {std::make_pair(290.0, 310.0), std::make_pair(290.0, 310.0)};
    prob.q_bounds = {std::make_pair(0.0, 400.0), std::make_pair(0.0, 400.0)};
    prob.u_bounds = {std::make_pair(-1.0, 1.0), std::make_pair(-1.0, 1.0)};
    prob.isDimCartesian = {true, true};

    // Set the environment bounds
    prob.x_min = 0.0;
    prob.x_max = 50.0;
    prob.y_min = 0.0;
    prob.y_max = 50.0;

    // Setup obstacles
    std::vector<Eigen::Vector2d> obstacle_vertices = {{33.0, 33.0}, {66.0, 33.0}, {66.0, 66.0}, {33.0, 66.0}};
    amp::Obstacle2D obstacle1(obstacle_vertices);
    obstacle_vertices = {{70.0, 100.0}, {130.0, 100.0}, {130.0, 130.0}, {70.0, 130.0}};


    return prob;
}

// @brief This function will create a random environment for a regular 2D problem for RRT
amp::Problem2D generateEnv::getEnvRand() {
    // Write obstacle locations to a file
    std::ofstream data_file;
    data_file.open("../../file_dump/obstacles.txt");

    // Check if the file opened correctly
    if (!data_file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
    } else {
        std::cout << "File opened successfully" << std::endl;
    }

    // Create the problem
    amp::Problem2D prob;
    prob.q_init = Eigen::Vector2d(5.0, 5.0);
    prob.q_goal = Eigen::Vector2d(95.0, 95.0);
    prob.x_min = 0.0;
    prob.x_max = 100.0;
    prob.y_min = 0.0;
    prob.y_max = 100.0;

    // Create a vector of obstacles, each obstacle has a coordinate and a radius
    int num_obstacles = 200;
    std::vector<amp::Obstacle2D> obstacles;
    std::vector<TreeObstacle> obstacle_vec;

    // Loop through and create random obstacles
    int max_iterations = 1000;
    for(int i = 0; i < num_obstacles; i++) {
        // Get a random tree obstacle
        TreeObstacle obstacle;
        bool valid = false;
        int iterations = 0;
        while(!valid && iterations < max_iterations) {
            obstacle = randomTreeObstacle(prob.x_min, prob.x_max, prob.y_min, prob.y_max, 0.4, 3.0);
            valid = isValidObstacle(obstacle, obstacle_vec, prob.q_init, prob.q_goal);
            iterations++;
        }

        // If the obstacle is valid, add it to the vector
        if(valid) {
            // Add the obstacle to the vector
            obstacle_vec.push_back(obstacle);

            // Write the obstacle to the file
            data_file << obstacle.center.x() << " " << obstacle.center.y() << " " << obstacle.radius << std::endl;

            // Create the obstacle with vertices
            amp::Obstacle2D obstacle_final = createObstacle(obstacle);

            // Add the obstacle to the vector
            prob.obstacles.push_back(obstacle_final);
        }
    }

    data_file.close();
    return prob;
}


// @brief Create a new candidate tree obstacle
TreeObstacle generateEnv::randomTreeObstacle(double x_min, double x_max, double y_min, double y_max, double min_radius, double max_radius) {
    // Create a random center
    Eigen::Vector2d center = amp::randomConfiguration(x_min, x_max, y_min, y_max);
    double radius = amp::randomDouble(min_radius, max_radius);
    TreeObstacle obstacle = {center, radius};
    return obstacle;
}

// @brief Check if the new candidate obstacle is valid
bool generateEnv::isValidObstacle(const TreeObstacle& obstacle, const std::vector<TreeObstacle>& obstacles, Eigen::Vector2d q_init, Eigen::Vector2d q_goal) {
    // Loop through all obstacles and check if the new obstacle is valid
    for(const auto& obs : obstacles) {
        double distance = (obstacle.center - obs.center).norm();
        if(distance < obstacle.radius + obs.radius) {
            return false;
        }
    }

    // Check if the obstacle is too close to the start or goal
    double distance_init = (obstacle.center - q_init).norm();
    double distance_goal = (obstacle.center - q_goal).norm();
    if(distance_init < obstacle.radius + 2.0 || distance_goal < obstacle.radius + 2.0) {
        return false;
    }

    return true;
}

// @brief Create an obstacle2D object from a tree obstacle
amp::Obstacle2D generateEnv::createObstacle(const TreeObstacle& obstacle) {
    // Create the vertices of the obstacle
    int num_vertices = 20;
    std::vector<Eigen::Vector2d> vertices;
    for(int i = 0; i < num_vertices; i++) {
        double angle = 2 * M_PI * i / num_vertices;
        Eigen::Vector2d vertex = obstacle.center + Eigen::Vector2d(obstacle.radius * cos(angle), obstacle.radius * sin(angle));
        vertices.push_back(vertex);
    }
    // Create the obstacle
    amp::Obstacle2D obs(vertices);
    return obs;
}


// @brief Given an overall problem definition, create a subproblem to adaptively plan around obstacles
amp::Problem2D generateEnv::createSubproblem(const amp::Problem2D& problem, const Eigen::Vector2d& q_curr, const Eigen::Vector2d& q_intermediateGoal, const rangeFindingCar& agent) {
    // Create the subproblem
    amp::Problem2D subproblem = problem;
    subproblem.q_init = q_curr;
    subproblem.q_goal = q_intermediateGoal;

    // Get the obstacles that are within the range finder of the agent
    std::vector<amp::Obstacle2D> new_obstacles;
    for(const auto& obs : problem.obstacles) {
        // Check if the obstacle is within the range finder
        Eigen::Vector2d closest_point;
        if(distanceToObstacle(q_curr, obs, closest_point) < agent.agent_prop.radius) {
            new_obstacles.push_back(obs);
        }
    }

    // Set the new obstacles
    subproblem.obstacles = new_obstacles;

    return subproblem;
}