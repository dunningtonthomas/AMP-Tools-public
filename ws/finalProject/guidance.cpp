// This file will include the class for the high level planner 
#include "guidance.h"


// @brief Planning algorithm, returns a valid path using the range finder for planning
amp::Path2D adaptiveRRT::plan(const amp::Problem2D& problem, const rangeFindingCar& agent) {
    // Write waypoints to a file
    std::ofstream data_file, flag_file;
    data_file.open("../../file_dump/adaptive.txt");
    flag_file.open("../../file_dump/adaptive_flag.txt");

    // Check if the file opened correctly
    if (!data_file.is_open() || !flag_file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
    } else {
        std::cout << "File opened successfully" << std::endl;
    }
    
    // Overall path found boolean
    amp::Path2D final_path;
    bool path_found = false;

    // Path propagation lookahead value
    int path_propagation_value = 4;
    
    // Get the subproblem
    generateEnv sub;
    amp::Problem2D subproblem = sub.createSubproblem(problem, problem.q_init, problem.q_goal, agent);

    // Create the RRT graph
    MyRRT rrt_init;
    amp::Path2D path_init = rrt_init.plan(subproblem);

    // Check if the path is valid
    if(!rrt_init.getSuccess()) {
        std::cout << "Initial path not found, returning empty path" << std::endl;
        return path_init;
    } else {
        std::cout << "Initial path found, moving along trajectory..." << std::endl;
    }

    // Take steps along the path and at each step, check if any obstacles intersect the path
    std::vector<Eigen::Vector2d> current_waypoints = path_init.waypoints;
    Eigen::Vector2d q_curr = problem.q_init;
    int curr_index = 0;
    int intermediate_goal_index = curr_index + 20;  // Heuristic lookahead value

    // Write initial waypoints to a file
    writeWaypointsToFile(current_waypoints, data_file);

    // Take steps along the path until we get to the goal
    while(curr_index < current_waypoints.size() - 1) {
        // Get the waypoints and obstacles within the range of the agent
        std::vector<Eigen::Vector2d> new_waypoints = getWaypointsInRange(current_waypoints, q_curr, agent);
        std::vector<amp::Obstacle2D> new_obstacles = getObstaclesInRange(problem, q_curr, agent);

        // Check if any of the obstacles within the range finder intersect the path
        if(waypointsIntersectObstacles(new_waypoints, new_obstacles)) {
            std::cout << "Obstacles intersect the path at node " << curr_index << ", recalculating new path with lookahead " << path_propagation_value << std::endl;

            // Write the flag to the file the current point where planning begins
            flag_file << q_curr.x() << " " << q_curr.y() << std::endl;

            // Path propagation, solve subproblem starting 2 ft (4 steps) ahead of the current index
            int subproblem_start;
            if(curr_index + path_propagation_value < current_waypoints.size()) {
                subproblem_start = curr_index + path_propagation_value; //look-ahead
                intermediate_goal_index = subproblem_start + 20;
            } else {
                subproblem_start = curr_index; //stop and calculate
                intermediate_goal_index = current_waypoints.size() - 1;
            }

            // Get new goal location from a heuristic look ahead
            Eigen::Vector2d new_goal;
            if(intermediate_goal_index >= current_waypoints.size()) {
                intermediate_goal_index = current_waypoints.size() - 1;
            }

            // Create a new subproblem
            amp::Problem2D new_subproblem = sub.createSubproblem(problem, current_waypoints[subproblem_start], current_waypoints[intermediate_goal_index], agent);

            // Create the RRT graph
            MyRRT rrt_new;
            auto start_time = std::chrono::high_resolution_clock::now();
            amp::Path2D path_new = rrt_new.plan(new_subproblem);
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_time = end_time - start_time;
            final_path.adaptive_times.push_back(elapsed_time.count());

            // Check if the path is valid, later you can make this into a loop and adjust the intermediate goal
            if(!rrt_new.getSuccess()) {
                std::cout << "Path not found, returning empty path" << std::endl;
                return path_new;
            }

            // Replace the path in collision with the new subpath
            current_waypoints.erase(current_waypoints.begin() + subproblem_start, current_waypoints.begin() + intermediate_goal_index);
            current_waypoints.insert(current_waypoints.begin() + subproblem_start, path_new.waypoints.begin(), path_new.waypoints.end());

            // Write the new waypoints to the file
            writeWaypointsToFile(current_waypoints, data_file);
        }

        // Move forward
        curr_index++;
        intermediate_goal_index = curr_index + 20;
        q_curr = current_waypoints[curr_index];
        std::cout << "Moving to new waypoint: " << curr_index << std::endl;
    }

    // Create the final path using the current waypoints
    final_path.waypoints = current_waypoints;

    // Close the file
    data_file.close();
    flag_file.close();

    return final_path;
}


// @brief Planning algorithm, returns a valid path using the range finder for planning
amp::KinoPath adaptiveRRT::plan(const amp::KinodynamicProblem2D& problem, const rangeFindingCar& agent) {
    // Agent factory to get the agent type
    std::unordered_map<amp::AgentType, std::function<std::shared_ptr<amp::DynamicAgent>()>> agentFactory = {
        {amp::AgentType::SingleIntegrator, []() { return std::make_shared<MySingleIntegrator>(); }},
        {amp::AgentType::FirstOrderUnicycle, []() { return std::make_shared<MyFirstOrderUnicycle>(); }},
        {amp::AgentType::SecondOrderUnicycle, []() { return std::make_shared<MySecondOrderUnicycle>(); }},
        {amp::AgentType::SimpleCar, []() { return std::make_shared<MySimpleCar>(); }}
    };

    // Write waypoints to a file
    std::ofstream data_file;
    data_file.open("../../file_dump/adaptiveKino.txt");

    // Check if the file opened correctly
    if (!data_file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
    } else {
        std::cout << "File opened successfully" << std::endl;
    }
    
    // Overall path found boolean
    bool path_found = false;
    
    // Get the subproblem
    generateEnv sub;
    amp::KinodynamicProblem2D subproblem = sub.createSubproblem(problem, problem.q_init, problem.q_goal, agent);

    // Create the RRT graph
    MyKinoRRT rrt_init;
    amp::KinoPath path_init = rrt_init.plan(subproblem, *agentFactory[subproblem.agent_type]());

    // Check if the path is valid
    if(!rrt_init.getSuccess()) {
        std::cout << "Initial path not found, returning empty path" << std::endl;
        return path_init;
    } else {
        std::cout << "Initial path found, moving along trajectory..." << std::endl;
    }

    // Take steps along the path and at each step, check if any obstacles intersect the path
    std::vector<Eigen::VectorXd> current_waypoints = path_init.waypoints;
    Eigen::VectorXd q_curr = problem.q_init;
    int curr_index = 0;
    int intermediate_goal_index = curr_index + 10;  // Heuristic lookahead value

    // Write initial waypoints to a file
    writeWaypointsToFile(current_waypoints, data_file);

    // Take steps along the path until we get to the goal
    while(curr_index < current_waypoints.size() - 1) {
        // Get the waypoints and obstacles within the range of the agent
        std::vector<Eigen::VectorXd> new_waypoints = getWaypointsInRange(current_waypoints, q_curr, agent);
        std::vector<amp::Obstacle2D> new_obstacles = getObstaclesInRange(problem, q_curr, agent);

        // Check if any of the obstacles within the range finder intersect the path
        if(waypointsIntersectObstacles(new_waypoints, new_obstacles)) {
            std::cout << "Obstacles intersect the path at node " << curr_index << ", recalculating new path" << std::endl;

            // Get new goal location from a heuristic look ahead
            std::vector<std::pair<double, double>> new_goal;
            if(intermediate_goal_index < current_waypoints.size()) {
                Eigen::VectorXd new_goal_center = current_waypoints[intermediate_goal_index];

                // Make new goal bounds
                std::pair<double, double> x_bounds(new_goal_center(0) - 0.2, new_goal_center(0) + 0.2);
                std::pair<double, double> y_bounds(new_goal_center(1) - 0.2, new_goal_center(1) + 0.2);
                std::pair<double, double> theta_bounds(new_goal_center(2) - 0.1, new_goal_center(2) + 0.1);
                new_goal = {x_bounds, y_bounds, theta_bounds};

            } else {
                intermediate_goal_index = current_waypoints.size() - 1;
                Eigen::VectorXd new_goal_center = current_waypoints[intermediate_goal_index];

                // Make new goal bounds
                std::pair<double, double> x_bounds(new_goal_center(0) - 0.2, new_goal_center(0) + 0.2);
                std::pair<double, double> y_bounds(new_goal_center(1) - 0.2, new_goal_center(1) + 0.2);
                std::pair<double, double> theta_bounds(new_goal_center(2) - 0.2, new_goal_center(2) + 0.2);
                new_goal = {x_bounds, y_bounds, theta_bounds};
            }

            // Create a new subproblem
            amp::KinodynamicProblem2D new_subproblem = sub.createSubproblem(problem, q_curr, new_goal, agent);

            // Create the RRT graph
            MyKinoRRT rrt_new;
            amp::KinoPath path_new = rrt_new.plan(new_subproblem, *agentFactory[new_subproblem.agent_type]());

            // Check if the path is valid, later you can make this into a loop and adjust the intermediate goal
            if(!rrt_new.getSuccess()) {
                std::cout << "Path not found, returning empty path" << std::endl;
                return path_new;
            }

            // Replace the path in collision with the new subpath
            current_waypoints.erase(current_waypoints.begin() + curr_index, current_waypoints.begin() + intermediate_goal_index);
            current_waypoints.insert(current_waypoints.begin() + curr_index, path_new.waypoints.begin(), path_new.waypoints.end());

            // Write the new waypoints to the file
            writeWaypointsToFile(current_waypoints, data_file);
        }

        // Move forward
        curr_index++;
        intermediate_goal_index = curr_index + 20;
        q_curr = current_waypoints[curr_index];
        std::cout << "Moving to new waypoint: " << curr_index << std::endl;
    }

    // Create the final path using the current waypoints
    amp::KinoPath final_path;
    final_path.waypoints = current_waypoints;

    // Close the file
    data_file.close();

    return final_path;
}

// @brief Return obstacles within the range of the agent
std::vector<amp::Obstacle2D> adaptiveRRT::getObstaclesInRange(const amp::Problem2D& problem, const Eigen::Vector2d& q_curr, const rangeFindingCar& agent) {
    // Get the obstacles that are within the range finder of the agent
    std::vector<amp::Obstacle2D> new_obstacles;
    for(const auto& obs : problem.obstacles) {
        // Check if the obstacle is within the range finder
        Eigen::Vector2d closest_point;
        if(amp::distanceToObstacle(q_curr, obs, closest_point) < agent.agent_prop.radius) {
            new_obstacles.push_back(obs);
        }
    }

    return new_obstacles;
}

// @brief KINODYNAMIC: Return obstacles within the range of the agent
std::vector<amp::Obstacle2D> adaptiveRRT::getObstaclesInRange(const amp::KinodynamicProblem2D& problem, const Eigen::VectorXd& q_curr, const rangeFindingCar& agent) {
    // Get the obstacles that are within the range finder of the agent
    std::vector<amp::Obstacle2D> new_obstacles;
    for(const auto& obs : problem.obstacles) {
        // Check if the obstacle is within the range finder
        Eigen::Vector2d closest_point;
        Eigen::Vector2d q_curr_pos(q_curr(0), q_curr(1));
        if(amp::distanceToObstacle(q_curr_pos, obs, closest_point) < agent.agent_prop.radius) {
            new_obstacles.push_back(obs);
        }
    }

    return new_obstacles;
}


// @brief Returns waypoints within the range of the agent
std::vector<Eigen::Vector2d> adaptiveRRT::getWaypointsInRange(std::vector<Eigen::Vector2d> curr_waypoints, const Eigen::Vector2d& q_curr, const rangeFindingCar& agent) {
    // Get the waypoints that are within the range finder of the agent
    std::vector<Eigen::Vector2d> new_waypoints;
    for(const auto& waypoint : curr_waypoints) {
        // Check if the waypoint is within the range finder
        if(amp::distance(q_curr, waypoint) < agent.agent_prop.radius) {
            new_waypoints.push_back(waypoint);
        }
    }
    return new_waypoints;
}

// @brief KINODYNAMIC: Returns waypoints within the range of the agent
std::vector<Eigen::VectorXd> adaptiveRRT::getWaypointsInRange(std::vector<Eigen::VectorXd> curr_waypoints, const Eigen::VectorXd& q_curr, const rangeFindingCar& agent) {
    // Get the waypoints that are within the range finder of the agent
    std::vector<Eigen::VectorXd> new_waypoints;
    for(const auto& waypoint : curr_waypoints) {
        // Check if the waypoint is within the range finder
        Eigen::Vector2d waypoint_pos(waypoint(0), waypoint(1));
        Eigen::Vector2d q_curr_pos(q_curr(0), q_curr(1));
        if(amp::distance(q_curr_pos, waypoint_pos) < agent.agent_prop.radius) {
            new_waypoints.push_back(waypoint);
        }
    }
    return new_waypoints;
}


// @brief Determines if the waypoints within the range finder radius intersect with any obstacles
bool adaptiveRRT::waypointsIntersectObstacles(const std::vector<Eigen::Vector2d>& waypoints, const std::vector<amp::Obstacle2D>& obstacles) {
    // Create a line between each waypoint and check if it intersects with any obstacles
    for(int i = 0; i < waypoints.size() - 1; i++) {
        Eigen::Vector2d current_pos = waypoints[i];
        Eigen::Vector2d next_pos = waypoints[i + 1];
        if(amp::inCollision(obstacles, current_pos, next_pos)) {
            return true;
        }
    }
    return false;
}

// @brief KINODYNAMIC: Determines if the waypoints within the range finder radius intersect with any obstacles
bool adaptiveRRT::waypointsIntersectObstacles(const std::vector<Eigen::VectorXd>& waypoints, const std::vector<amp::Obstacle2D>& obstacles) {
    // Create a line between each waypoint and check if it intersects with any obstacles
    for(int i = 0; i < waypoints.size() - 1; i++) {
        Eigen::VectorXd current_state = waypoints[i];
        Eigen::VectorXd next_state = waypoints[i + 1];
        Eigen::Vector2d current_pos(current_state(0), current_state(1));
        Eigen::Vector2d next_pos(next_state(0), next_state(1));
        if(amp::inCollision(obstacles, current_pos, next_pos)) {
            return true;
        }
    }
    return false;
}


// @brief Write the current waypoints to a file by appending it and adding to the end row
void adaptiveRRT::writeWaypointsToFile(const std::vector<Eigen::Vector2d>& waypoints, std::ofstream& data_file) {
    // Write the waypoints to the file
    for(const auto& waypoint : waypoints) {
        data_file << waypoint(0) << " " << waypoint(1) << " ";
    }
    data_file << std::endl;
}

// @brief KINODYNAMIC: Write the current waypoints to a file by appending it and adding to the end row
void adaptiveRRT::writeWaypointsToFile(const std::vector<Eigen::VectorXd>& waypoints, std::ofstream& data_file) {
    // Write the waypoints to the file
    for(const auto& waypoint : waypoints) {
        for (int i = 0; i < waypoint.size(); i++) {
            data_file << waypoint(i) << " ";
        }
    }
    data_file << std::endl;
}