// This file will include the class for the high level planner 
#include "guidance.h"


// @brief Planning algorithm, returns a valid path using the range finder for planning
amp::Path2D adaptiveRRT::plan(const amp::Problem2D& problem, const rangeFindingCar& agent) {
    // Overall path found boolean
    bool path_found = false;
    
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
    }

    // Take steps along the path and at each step, check if any obstacles intersect the path
    std::vector<Eigen::Vector2d> current_waypoints = path_init.waypoints;
    Eigen::Vector2d q_curr = problem.q_init;
    int curr_index = 0;
    int intermediate_goal_index = curr_index + 20;

    // Take steps along the path until we get to the goal
    while(curr_index < current_waypoints.size() - 1) {
        // Get the waypoints and obstacles within the range of the agent
        std::vector<Eigen::Vector2d> new_waypoints = getWaypointsInRange(current_waypoints, q_curr, agent);
        std::vector<amp::Obstacle2D> new_obstacles = getObstaclesInRange(problem, q_curr, agent);

        // Check if any of the obstacles within the range finder intersect the path
        if(waypointsIntersectObstacles(new_waypoints, new_obstacles)) {
            std::cout << "Obstacles intersect the path, recalculating new path" << std::endl;

            // Get new goal location from a heuristic look ahead
            Eigen::Vector2d new_goal;
            if(intermediate_goal_index < current_waypoints.size()) {
                new_goal = current_waypoints[intermediate_goal_index];
            } else {
                intermediate_goal_index = current_waypoints.size() - 1;
                new_goal = problem.q_goal;
            }

            // Create a new subproblem
            amp::Problem2D new_subproblem = sub.createSubproblem(problem, q_curr, new_goal, agent);

            // Create the RRT graph
            MyRRT rrt_new;
            amp::Path2D path_new = rrt_new.plan(new_subproblem);

            // Check if the path is valid, later you can make this into a loop and adjust the intermediate goal
            if(!rrt_new.getSuccess()) {
                std::cout << "Path not found, returning empty path" << std::endl;
                return path_new;
            }

            // Replace the path in collision with the new subpath
            current_waypoints.erase(current_waypoints.begin() + curr_index, current_waypoints.begin() + intermediate_goal_index);
            current_waypoints.insert(current_waypoints.begin() + curr_index, path_new.waypoints.begin(), path_new.waypoints.end());

            // Update the current position
            curr_index++;
            intermediate_goal_index = curr_index + 20;
            q_curr = current_waypoints[curr_index];
        } else {
            // No collision, update the current position
            curr_index++;
            intermediate_goal_index = curr_index + 20;
            q_curr = current_waypoints[curr_index];
        }
    }

    // Create the final path using the current waypoints
    amp::Path2D final_path;
    final_path.waypoints = current_waypoints;
    return final_path;
}


// @brief Return obstacles within the range of the agent
std::vector<amp::Obstacle2D> adaptiveRRT::getObstaclesInRange(const amp::Problem2D& problem, const Eigen::Vector2d& q_curr, const rangeFindingCar& agent) {
    // Get the obstacles that are within the range finder of the agent
    std::vector<amp::Obstacle2D> new_obstacles;
    for(const auto& obs : problem.obstacles) {
        // Check if the obstacle is within the range finder
        Eigen::Vector2d closest_point;
        if(distanceToObstacle(q_curr, obs, closest_point) < agent.agent_prop.radius) {
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
