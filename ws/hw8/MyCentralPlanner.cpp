#include "MyMultiAgentPlanners.h"


//////////////////////////////////////////////////////////
///////// CENTRALIZED PLANNER ////////////////////////////
//////////////////////////////////////////////////////////

amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    
    // Steps for centralized multi agent RRT planning
    // 1) Create tree rooted at qi which is the configuration of all agents at the beginning (x1_i, y1_i, x2_i, y2_i, ..., xn_i, yn_i)
    // 2) Generate random configuration
    // 3) Find the nearest configuration in the tree, need to define distance metric, probably just L2 norm for each robot added together
    // 4) Extend path from nearest neighbor to the qrand
    // 5) Check if subpath is collision free, need to discretize and check for collision in the workspace
    // 6) For each discrete step, need to check if the system is valid by
          // 1. Checking if the robot is in collision with an obstacle
          // 2. Checking if the robot is in collision with another robot
    // 7) If the path is valid, add q_new which is a step size in the direction of q_rand from q_near
    // 8) Check if q_new is close enough to the goal for all agents, within epsilon, check for all configuration in the q vector
    // 9) If q_new is close enough, back track all the agents' parent nodes to get the path for every agent

    // Overall path for all agents
    amp::MultiAgentPath2D path;

    // Create initial and goal configuration vectors which is (x1_i, y1_i, ..., xn_i, yn_i) for all agents
    Eigen::VectorXd q_init(problem.numAgents() * 2);
    Eigen::VectorXd q_goal(problem.numAgents() * 2);
    int i = 0;
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        q_init(i) = agent.q_init.x();
        q_init(i+1) = agent.q_init.y();
        q_goal(i) = agent.q_goal.x();
        q_goal(i+1) = agent.q_goal.y();
        i += 2;
    }

    // Create initial node
    amp::Node init_node = 0;
    nodes[init_node] = q_init;

    // Loop to find path
    int iteration = 0;
    while(iteration < max_iterations) {
        // Generate random sample in the environment, biased toward the goal
        Eigen::VectorXd q_rand(problem.numAgents() * 2);
        if(amp::randomDouble(0.0, 1.0) < goal_bias) {
            // Sample the goal
            q_rand = q_goal;
        } else {
            // Sample a random point in the environment
            q_rand = randomConfiguration(problem);
        }

        // Find the nearest neighbor to the random configuration
        amp::Node nearest_node = nearestNeighbor(q_rand);
        Eigen::VectorXd q_near = nodes[nearest_node];


        // Check if the subpath is collision free
        if(isSubpathCollisionFree(problem, q_near, q_rand)) {
            // Extend tree to the new node 
            // Eigen::VectorXd q_new_big = q_rand - q_near;
            // Eigen::VectorXd q_new(q_new_big.size());
            // for(int k = 0; k < q_new_big.size(); k+=2) {
            //     // Normalize each agent's step size
            //     Eigen::Vector2d q_new_big_agent(q_new_big(k), q_new_big(k+1));
            //     Eigen::Vector2d q_near_agent(q_near(k), q_near(k+1));
            //     Eigen::Vector2d q_new_agent = q_near_agent + step_size * q_new_big_agent.normalized();

            //     // Add to the new configuration
            //     q_new(k) = q_new_agent.x();
            //     q_new(k+1) = q_new_agent.y();
            // }

            Eigen::VectorXd q_new(q_rand.size());
            for(int k = 0; k < q_rand.size(); k+=2) {
                Eigen::Vector2d q_change(q_rand(k) - q_near(k), q_rand(k+1) - q_near(k+1));
                q_change = q_change.normalized();

                // Only extend the parts that are not in the goal
                Eigen::Vector2d q_near_agent(q_near(k), q_near(k+1));
                Eigen::Vector2d q_goal_agent(q_goal(k), q_goal(k+1));
                if(!isSystemInGoal(q_near_agent, q_goal_agent)) {
                    q_new(k) = q_near(k) + step_size * q_change.x();
                    q_new(k+1) = q_near(k+1) + step_size * q_change.y();
                } else {
                    q_new(k) = q_near(k);
                    q_new(k+1) = q_near(k+1);
                }
            }

            //Eigen::VectorXd q_new = q_near + step_size * (q_rand - q_near).normalized();


            amp::Node add_node = nodes.size();
            nodes[add_node] = q_new;

            // Set the parent map of the new configuration
            parents[add_node] = nearest_node;

            // Check if we are close enough to the goal
            if(isSystemInGoal(q_new, q_goal)) {
                // Add the goal to the graph
                amp::Node goal_node = nodes.size();
                nodes[goal_node] = q_goal;
                parents[goal_node] = add_node;
                std::cout << "RRT Path found in "<< iteration << " iterations, reconstructing path..." << std::endl;
                

                // Recreate the path from the parents map
                amp::Node final_node = nodes.size() - 1;
                for(int i = 0; i < problem.numAgents() * 2; i+=2) {
                    // Create the path for each agent
                    amp::Path2D agent_path;

                    amp::Node curr_node = final_node;
                    while(nodes[curr_node] != q_init) {
                        // Get the current agents configuration
                        Eigen::Vector2d agent_position(nodes[curr_node](i), nodes[curr_node](i+1));
                        agent_path.waypoints.push_back(agent_position);
                        curr_node = parents[curr_node];
                    }

                    // Add the initial configuration
                    Eigen::Vector2d agent_position(nodes[curr_node](i), nodes[curr_node](i+1));
                    agent_path.waypoints.push_back(agent_position);

                    // Reverse the path
                    std::reverse(agent_path.waypoints.begin(), agent_path.waypoints.end());

                    // Add to the overall multi agent path
                    path.agent_paths.push_back(agent_path);
                }
                // Set the success variable
                success = true;
                break;
            }
        }
        iteration++;
        std::cout << "Iteration: " << iteration << std::endl;
    }

    // If the path is not found, return the initial path
    if(!success) {
        std::cout << "RRT Path not found in " << iteration << " iterations, returning initial path" << std::endl;
        // for(const amp::CircularAgentProperties& agent : problem.agent_properties) {
        //     amp::Path2D agent_path;
        //     agent_path.waypoints = {agent.q_init, agent.q_goal};
        //     path.agent_paths.push_back(agent_path);
        // }

        // DEBUGGING: Return the path of the final node even if it is not successful
        // Recreate the path from the parents map
        amp::Node final_node = nodes.size() - 1;
        for(int i = 0; i < problem.numAgents() * 2; i+=2) {
            // Create the path for each agent
            amp::Path2D agent_path;

            amp::Node curr_node = final_node;
            while(nodes[curr_node] != q_init) {
                // Get the current agents configuration
                Eigen::Vector2d agent_position(nodes[curr_node](i), nodes[curr_node](i+1));
                agent_path.waypoints.push_back(agent_position);
                curr_node = parents[curr_node];
            }

            // Add the initial configuration
            Eigen::Vector2d agent_position(nodes[curr_node](i), nodes[curr_node](i+1));
            agent_path.waypoints.push_back(agent_position);

            // Reverse the path
            std::reverse(agent_path.waypoints.begin(), agent_path.waypoints.end());

            // Add the goal to the agent path
            Eigen::Vector2d agent_goal(q_goal(i), q_goal(i+1));
            agent_path.waypoints.push_back(agent_goal);

            // Add to the overall multi agent path
            path.agent_paths.push_back(agent_path);
        }
    }
    return path;
}

Eigen::VectorXd MyCentralPlanner::randomConfiguration(const amp::MultiAgentProblem2D& problem) {
    // Create a random number generator
    std::random_device rd;  // Random seed
    std::mt19937 gen(rd()); // Mersenne Twister generator

    // Define the uniform distribution between -5 and 5
    std::uniform_real_distribution<double> x_rand(problem.x_min, problem.x_max);
    std::uniform_real_distribution<double> y_rand(problem.y_min, problem.y_max);

    // Sample a random configuration in the environment
    Eigen::VectorXd q_rand(problem.numAgents() * 2);
    for(int i = 0; i < problem.numAgents() * 2; i+=2) {
        q_rand(i) = x_rand(gen);
        q_rand(i+1) = y_rand(gen);
    }
    return q_rand;
}


// @brief calculate the nearest neighbor using a standard distance metric
amp::Node MyCentralPlanner::nearestNeighbor(const Eigen::VectorXd& q_rand) {
    // Find the nearest node in the tree to the new configuration
    double min_distance = std::numeric_limits<double>::max();
    double distance;
    amp::Node nearest_node = 0;
    for (const auto& [node, q] : nodes) {
        distance = configurationDistance(q, q_rand);
        if (distance < min_distance && distance > 0) {
            min_distance = distance;
            nearest_node = node;
        }
    }
    return nearest_node;
}


// @brief check if the subpath between two nodes is collision free
bool MyCentralPlanner::isSubpathCollisionFree(const amp::MultiAgentProblem2D& problem, const Eigen::VectorXd& q_near, const Eigen::VectorXd& q_rand) {
    // Discretize the path between q_near and q_rand and check for collision
    int num_steps = 50;
    for (int i = 0; i < num_steps; i++) {
        // Calculate the step size
        Eigen::VectorXd q_new = q_near + (q_rand - q_near) * i * (1.0 / num_steps);

        // Check if the path is valid
        if (!isSystemValid(problem, q_new)) {
            return false;
        }
    }
    return true;
}


// @brief check if a configuration is in collision with the environment or other agents
bool MyCentralPlanner::isSystemValid(const amp::MultiAgentProblem2D& problem, const Eigen::VectorXd& q) {
    int i = 0;
    for(const auto& agent : problem.agent_properties) {
        // Check if the agent is in collision with an obstacle
        Eigen::Vector2d agent_position(q(i), q(i+1));

        // Loop through obstacles and check for collisions
        for(const auto& obstacle : problem.obstacles) {
            // Check if the agent is in collision with the obstacle
            if (obstacleCollision(obstacle, agent_position, agent.radius)) {
                return false;
            }
        }

        // Loop through the other agents and check for collisions
        int j = 0;
        for(const auto& other_agent : problem.agent_properties) {
            if (i != j) {
                Eigen::Vector2d other_agent_position(q(j), q(j+1));
                if (agentCollision(agent_position, agent.radius, other_agent_position, other_agent.radius)) {
                    return false;
                }
            }
            j += 2;
        }
        i += 2;
    }
    return true;
}

// @brief check if the system is in goal
bool MyCentralPlanner::isSystemInGoal(const Eigen::VectorXd& q, const Eigen::VectorXd& q_goal) {
    // Loop through all agents and check if they are close enough to the goal
    for(int i = 0; i < q.size(); i+=2) {
        Eigen::Vector2d agent_position(q(i), q(i+1));
        Eigen::Vector2d agent_goal(q_goal(i), q_goal(i+1));
        if(configurationDistance(agent_position, agent_goal) > epsilon) {
            return false;
        }
    }
    return true;
}


// @brief check if the disk robot is in collision with an obstacle
bool MyCentralPlanner::obstacleCollision(const amp::Obstacle2D& obstacle, const Eigen::Vector2d& agent_position, double agent_radius) {
    // Check if the point is inside the obstacle or if the disk intersects with the obstacle
    Eigen::Vector2d closest_point;
    if(isInsidePolygon(obstacle, agent_position) || distanceToObstacle(agent_position, obstacle, closest_point) < agent_radius) {
        return true;
    }
    return false;
}


// @brief check if the disk robot is in collision with another agent
bool MyCentralPlanner::agentCollision(const Eigen::Vector2d& agent_position, double agent_radius, const Eigen::Vector2d& other_agent_position, double other_agent_radius) {
    // Check if the agent is in collision with the other agent
    if ((agent_position - other_agent_position).norm() < agent_radius + other_agent_radius) {
        return true;
    }
    return false;
}





//////////////////////////////////////////////////////////
///////// DECENTRALIZED PLANNER //////////////////////////
//////////////////////////////////////////////////////////



amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    // DECENTRALIZED STEPS:
    // 1) compute collision free trajectory for the first agent and parameterize it w.r.t. time for the indices
    // 2) Compute collision free trajectory for the second agent, check for collisions with obstacles and other agents
    // 3) Continue for all other agents in the system


    // Plan for each agent
    for(const auto& agent : problem.agent_properties) {
        // Run RRT for the agent to generate a path
        amp::Path2D agent_path;

        // Clear the nodes, parents map, and index_map
        nodes.clear();
        parents.clear();
        index_map.clear();
        
        // Create the initial node
        amp::Node init_node = 0;
        nodes[init_node] = agent.q_init;
        index_map[init_node] = 0;

        // Run RRT for the agent
        int iteration = 0;
        while(iteration < max_iterations) {
            // Generate random sample in the environment, biased toward the goal
            Eigen::Vector2d q_rand;
            if(amp::randomDouble(0.0, 1.0) < goal_bias) {
                // Sample the goal
                q_rand = agent.q_goal;
            } else {
                // Sample a random point in the environment
                q_rand = randomAgentConfiguration(problem);
            }
            
            // Find the nearest neighbor to the random configuration
            amp::Node nearest_node = nearestNeighbor(q_rand);
            Eigen::Vector2d q_near = nodes[nearest_node];
            int near_index = index_map[nearest_node];

            if(isSubpathCollisionFree(problem, q_near, q_rand, near_index + 1)) {
                // Add the new node to the tree
                Eigen::Vector2d q_new = q_near + (q_rand - q_near).normalized() * step_size;
                amp::Node add_node = nodes.size();
                nodes[add_node] = q_new;
                index_map[add_node] = near_index + 1;
                parents[add_node] = nearest_node;

                // Check if the new node is close enough to the goal
                if(configurationDistance(q_new, agent.q_goal) < epsilon) {
                    // Add the goal to the graph
                    amp::Node goal_node = nodes.size();
                    nodes[goal_node] = agent.q_goal;
                    index_map[goal_node] = near_index + 1;
                    parents[goal_node] = add_node;
                    std::cout << "RRT Path found in "<< iteration << " iterations, reconstructing path..." << std::endl;

                    // Recreate the path from the parents map
                    amp::Node final_node = nodes.size() - 1;
                    amp::Node curr_node = final_node;
                    while(nodes[curr_node] != agent.q_init) {
                        agent_path.waypoints.push_back(nodes[curr_node]);
                        curr_node = parents[curr_node];
                    }
                    agent_path.waypoints.push_back(nodes[curr_node]);

                    // Reverse the path, exit the loop
                    std::reverse(agent_path.waypoints.begin(), agent_path.waypoints.end());
                    success = true;
                    break;
                }

            }
            iteration++;
        }

        // If the path is not found, return the initial path, reset success variable for next agent
        if(!success) {
            overall_success = false;
            std::cout << "RRT Path not found in " << iteration << " iterations, returning initial path" << std::endl;
            agent_path.waypoints = {agent.q_init, agent.q_goal};
        } else {
            success = false;
        }

        // Push the agent path to the overall path
        current_path_state.agent_paths.push_back(agent_path);
    }

    // Result
    if(overall_success) {
        std::cout << "RRT Path found for all agents, reconstructing path..." << std::endl;
    } else {
        std::cout << "RRT Path not found for all agents..." << std::endl;
    }

    return current_path_state;
}



Eigen::Vector2d MyDecentralPlanner::randomAgentConfiguration(const amp::MultiAgentProblem2D& problem) {
    // Create a random number generator
    std::random_device rd;  // Random seed
    std::mt19937 gen(rd()); // Mersenne Twister generator

    // Define the uniform distribution between -5 and 5
    std::uniform_real_distribution<double> x_rand(problem.x_min, problem.x_max);
    std::uniform_real_distribution<double> y_rand(problem.y_min, problem.y_max);

    // Sample a random configuration in the environment
    Eigen::Vector2d q_rand(x_rand(gen), y_rand(gen));
    return q_rand;
}


// @brief calculate the nearest neighbor using a standard distance metric
amp::Node MyDecentralPlanner::nearestNeighbor(const Eigen::VectorXd& q_rand) {
    // Find the nearest node in the tree to the new configuration
    double min_distance = std::numeric_limits<double>::max();
    double distance;
    amp::Node nearest_node = 0;
    for (const auto& [node, q] : nodes) {
        distance = configurationDistance(q, q_rand);
        if (distance < min_distance && distance > 0) {
            min_distance = distance;
            nearest_node = node;
        }
    }
    return nearest_node;
}

// @brief check if the subpath between two nodes is collision free
bool MyDecentralPlanner::isSubpathCollisionFree(const amp::MultiAgentProblem2D& problem, const Eigen::VectorXd& q_near, const Eigen::VectorXd& q_rand, int q_rand_index) {
    // Discretize the path between q_near and q_rand and check for collision
    int num_steps = 50;
    for (int i = 0; i < num_steps; i++) {
        // Calculate the step size
        Eigen::VectorXd q_new = q_near + (q_rand - q_near) * i * (1.0 / num_steps);

        // Check if the path is valid
        if (!isSystemValid(problem, q_new, q_rand_index)) {
            return false;
        }
    }
    return true;
}


// @brief check if a configuration is in collision with the environment or other agents at
bool MyDecentralPlanner::isSystemValid(const amp::MultiAgentProblem2D& problem, const Eigen::VectorXd& q, int q_rand_index) {
    // Get the agent
    const auto agent = problem.agent_properties[0];
    
    // Check for collision with obstacles for the agent
    Eigen::Vector2d agent_position(q(0), q(1));
    for(const auto& obstacle : problem.obstacles) {
        // Check if the agent is in collision with the obstacle
        if (obstacleCollision(obstacle, agent_position, agent.radius)) {
            return false;
        }
    }

    // Check if the agent is in collision with other agents at the given time step
    for(const auto& other_agent_path : current_path_state.agent_paths) {
        // Go to the next other path if the current time step is greater than the other agent's path
        if(q_rand_index >= other_agent_path.waypoints.size()) {
            continue;
        }

        // Check if the agent is in collision with the other agent
        Eigen::Vector2d other_agent_position = other_agent_path.waypoints[q_rand_index];
        if (agentCollision(agent_position, agent.radius, other_agent_position, agent.radius)) {
            return false;
        }
    }
    return true;
}


// @brief check if the disk robot is in collision with an obstacle
bool MyDecentralPlanner::obstacleCollision(const amp::Obstacle2D& obstacle, const Eigen::Vector2d& agent_position, double agent_radius) {
    // Check if the point is inside the obstacle or if the disk intersects with the obstacle
    Eigen::Vector2d closest_point;
    if(isInsidePolygon(obstacle, agent_position) || distanceToObstacle(agent_position, obstacle, closest_point) < agent_radius) {
        return true;
    }
    return false;
}


// @brief check if the disk robot is in collision with another agent
bool MyDecentralPlanner::agentCollision(const Eigen::Vector2d& agent_position, double agent_radius, const Eigen::Vector2d& other_agent_position, double other_agent_radius) {
    // Check if the agent is in collision with the other agent
    if ((agent_position - other_agent_position).norm() < agent_radius + other_agent_radius) {
        return true;
    }
    return false;
}
