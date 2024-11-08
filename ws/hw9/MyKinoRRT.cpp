#include "MyKinoRRT.h"

void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    state += dt * control;
};

void MyFirstOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    state(0) += control(0) * radius * std::cos(state(2)) * dt;
    state(1) += control(0) * radius * std::sin(state(2)) * dt;
    state(2) += control(1) * dt;
};

void MySecondOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    state(0) += state(3) * radius * std::cos(state(2)) * dt;
    state(1) += state(3) * radius * std::sin(state(2)) * dt;
    state(2) += state(4) * dt;
    state(3) += control(0) * dt;
    state(4) += control(1) * dt;
};

void MySimpleCar::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    state(0) += state(3) * std::cos(state(2)) * dt;
    state(1) += state(3) * std::sin(state(2)) * dt;
    state(2) += state(3) / agent_dim.length * std::tan(state(4)) * dt;
    state(3) += control(0) * dt;
    state(4) += control(1) * dt;
};

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    // Kinodynami RRT Variables
    amp::KinoPath path;
    Eigen::VectorXd state = problem.q_init;

    // Output the control bounds
    std::cout << "Control 1 Bounds: " << problem.u_bounds[0].first << " " << problem.u_bounds[0].second << std::endl;
    std::cout << "Control 2 Bounds: " << problem.u_bounds[1].first << " " << problem.u_bounds[1].second << std::endl;


    // If not a point robot, define the dimensions of the agent
    if(!problem.isPointAgent) {
        agent.agent_dim = problem.agent_dim;
        std::cout << "Agent Dimensions: " << agent.agent_dim.length << " " << agent.agent_dim.width << std::endl;
    }

    // Add the initial state
    //path.waypoints.push_back(state);
    nodes[0] = state;

    // Run Kinodynamic RRT
    int iteration = 0;
    while(iteration < max_iterations) {
        // Generate random sample in the environment, biased toward the goal
        Eigen::VectorXd q_rand(problem.q_bounds.size());
        if(amp::randomDouble(0.0, 1.0) < goal_bias) {
            // Sample the goal
            q_rand = sampleGoal(problem);
        } else {
            // Sample a random point in the environment
            q_rand = randomConfiguration(problem);
        }

        // Find the nearest neighbor
        amp::Node nearest_node = nearestNeighbor(problem, q_rand);
        Eigen::VectorXd q_near = nodes[nearest_node];

        // Create a collision free subpath
        Eigen::VectorXd q_new;
        Eigen::VectorXd control;
        bool subpath_bool = createSubpath(problem, agent, q_near, q_rand, q_new, control); 

        // Check if the subpath is collision free
        if(subpath_bool) {
            //std::cout << "Iteration: " << iteration << " Subpath found" << std::endl;
            // Add the new state to the tree
            amp::Node new_node = nodes.size();
            nodes[new_node] = q_new;
            parents[new_node] = nearest_node;
            controls[new_node] = control;

            // Check if the new node is close enough to the goal
            if(isSystemInGoal(problem, q_new)) {
                std::cout << "RRT Path found in "<< iteration << " iterations, reconstructing path..." << std::endl;

                // Propagate with controls again to test next state
                Eigen::VectorXd q_temp = q_new;
                agent.propagate(q_temp, control, dt);


                int i = 0;
                for(const auto& goal : problem.q_goal) {
                    std::cout << "Goal " << i << " Bounds: " << goal.first << " " << goal.second << std::endl;
                    std::cout << "Final State " << i << ": " << q_new(i) << std::endl << std::endl;
                    i++;
                }

                // Reconstruct the path
                amp::Node final_node = nodes.size() - 1;
                amp::Node curr_node = final_node;
                amp::Node control_node = parents[final_node];

                // Add zero control for the final goal node
                path.controls.push_back(Eigen::VectorXd::Zero(problem.u_bounds.size()));
                path.durations.push_back(0.0);

                // Loop until we get to the initial node
                while(nodes[curr_node] != problem.q_init) {
                    path.waypoints.push_back(nodes[curr_node]);
                    path.controls.push_back(controls[curr_node]);
                    path.durations.push_back(dt);
                    curr_node = parents[curr_node];
                }
                // Add initial node
                path.waypoints.push_back(nodes[curr_node]);

                // Reverse the path
                std::reverse(path.waypoints.begin(), path.waypoints.end());
                std::reverse(path.controls.begin(), path.controls.end());
                std::reverse(path.durations.begin(), path.durations.end());
                path.valid = true;
                return path;
            }
        }
        iteration++;
    }

    // Reconstruct the path to the most recent node
    std::cout << "RRT Path not found in " << iteration << " iterations, returning invalid final path" << std::endl;


    // CAR DEBUGGING
    // Reconstruct the path
    amp::Node final_node = nodes.size() - 1;
    amp::Node curr_node = final_node;
    amp::Node control_node = parents[final_node];

    // Add zero control for the final goal node
    path.controls.push_back(Eigen::VectorXd::Zero(problem.u_bounds.size()));
    path.durations.push_back(0.0);

    // Loop until we get to the initial node
    while(nodes[curr_node] != problem.q_init) {
        path.waypoints.push_back(nodes[curr_node]);
        path.controls.push_back(controls[curr_node]);
        path.durations.push_back(dt);
        curr_node = parents[curr_node];
    }
    // Add initial node
    path.waypoints.push_back(nodes[curr_node]);

    // Reverse the path
    std::reverse(path.waypoints.begin(), path.waypoints.end());
    std::reverse(path.controls.begin(), path.controls.end());
    std::reverse(path.durations.begin(), path.durations.end());

    // Return the path
    path.valid = true;
    return path;
}



// @brief Generate a random control input for a given agent
Eigen::VectorXd MyKinoRRT::randomControl(const amp::KinodynamicProblem2D& problem) {
    Eigen::VectorXd control = Eigen::VectorXd::Random(problem.u_bounds.size());

    // Normalize control to be within the bounds of the problem
    for(int i = 0; i < problem.u_bounds.size(); i++) {
        control(i) = control(i) * (problem.u_bounds[i].second - problem.u_bounds[i].first) / 2.0 + (problem.u_bounds[i].second + problem.u_bounds[i].first) / 2.0;
    }
    return control;
}

// @brief Generate a random configuration 
Eigen::VectorXd MyKinoRRT::randomConfiguration(const amp::KinodynamicProblem2D& problem) {
    Eigen::VectorXd configuration = Eigen::VectorXd::Random(problem.q_bounds.size());

    // Normalize configuration to be within the bounds of the problem
    for(int i = 0; i < problem.q_bounds.size(); i++) {
        configuration(i) = configuration(i) * (problem.q_bounds[i].second - problem.q_bounds[i].first) / 2.0 + (problem.q_bounds[i].second + problem.q_bounds[i].first) / 2.0;
    }
    return configuration;
}

// @brief Sample the goal
Eigen::VectorXd MyKinoRRT::sampleGoal(const amp::KinodynamicProblem2D& problem) {
    Eigen::VectorXd goal = Eigen::VectorXd(problem.q_bounds.size());
    for(int i = 0; i < problem.q_goal.size(); i++) {
        goal(i) = amp::randomDouble(problem.q_goal[i].first, problem.q_goal[i].second);
    }
    return goal;
}


// @brief calculate the nearest neighbor using a standard distance metric
amp::Node MyKinoRRT::nearestNeighbor(const amp::KinodynamicProblem2D& problem, const Eigen::VectorXd& q_rand) {
    // Find the nearest node in the tree to the new configuration
    double min_distance = std::numeric_limits<double>::max();
    double distance;
    amp::Node nearest_node = 0;
    for (const auto& [node, q] : nodes) {
        distance = configurationDistance(problem, q, q_rand);
        if (distance < min_distance && distance > 0) {
            min_distance = distance;
            nearest_node = node;
        }
    }
    return nearest_node;
}


// @brief calculate the distance between two configurations
double MyKinoRRT::configurationDistance(const amp::KinodynamicProblem2D& problem, const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) {
    // Calculate the distance between two configurations, prioritize the x, y distance
    std::vector<double> distances;
    std::vector<double> anglular_distances;
    for(int i = 0; i < problem.q_bounds.size(); i++) {
        // Only calculate the distance for Cartesian dimensions
        // if(problem.isDimCartesian[i]) {
        //     distances.push_back(std::abs(q1(i) - q2(i)));
        // } else {
        //     // Calculate the angular distance
        //     double angle_distance = std::abs(q1(i) - q2(i));
        //     if(angle_distance > M_PI) {
        //         angle_distance = 2 * M_PI - angle_distance;
        //     }
        //     anglular_distances.push_back(angle_distance);
        // }
        distances.push_back(std::abs(q1(i) - q2(i)));
    }

    // Calculate the Euclidean distance
    double distance = 0.0;
    for(const auto& dist : distances) {
        distance += dist * dist;
    }
    distance = std::sqrt(distance);

    // Add angular distance 
    // for(const auto& angle_dist : anglular_distances) {
    //     distance += angle_dist;
    // }

    return distance;
}


// @brief create a collision free subpath from the nearest node to a new node
bool MyKinoRRT::createSubpath(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent, Eigen::VectorXd& q_near, Eigen::VectorXd& q_rand, Eigen::VectorXd& q_new, Eigen::VectorXd& control) {
    // Loop through multiple random controls and determine if the path is collision free
    int num_controls = u_samples;
    bool subpath_found = false;
    double min_distance = std::numeric_limits<double>::max();
    for(int i = 0; i < num_controls; i++) {
        // Sample a random control input
        Eigen::VectorXd control_candidate = randomControl(problem);
        Eigen::VectorXd q_new_candidate;

        // Check if the path is valid
        if(isSubpathCollisionFree(problem, agent, q_near, q_new_candidate, control_candidate)) {
            // Check if the new control is closer to the goal
            double distance = configurationDistance(problem, q_new_candidate, q_rand);
            if(distance < min_distance) {
                min_distance = distance;
                q_new = q_new_candidate;
                control = control_candidate;
                subpath_found = true;
            }
        }
    }
    return subpath_found;
}


// @brief check if a local trajectory from a node with a given control and time step is collision free
bool MyKinoRRT::isSubpathCollisionFree(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent, Eigen::VectorXd& q_near, Eigen::VectorXd& q_new, Eigen::VectorXd& control) {
    // Discretize the control by 100 steps along the dt change and check for collision
    // Return boolean success and q_new by reference to update the new state
    int num_steps = 100;
    q_new = q_near;
    for(int i = 0; i < num_steps; i++) {
        // Propagate the state forward according to dynamics model
        agent.propagate(q_new, control, dt/num_steps);

        // Check if the path is valid
        if(!isSystemValid(problem, q_new)) {
            return false;
        }
    }
    return true;
}


// @brief check if a configuration is in collision with the environment
bool MyKinoRRT::isSystemValid(const amp::KinodynamicProblem2D& problem, const Eigen::VectorXd& q) {
    for(const auto& obstacle : problem.obstacles) {
        // Check if the agent is in collision with the obstacle
        if(obstacleCollision(problem, obstacle, q)) {
            return false;
        }
    }

    // Check if the state is outside the state bounds
    for(int i = 0; i < problem.q_bounds.size(); i++) {
        if(q(i) < problem.q_bounds[i].first || q(i) > problem.q_bounds[i].second) {
            return false;
        }
    }

    return true;
}

// @brief check if the configuration is in collision with an obstacle
bool MyKinoRRT::obstacleCollision(const amp::KinodynamicProblem2D& problem, const amp::Obstacle2D& obstacle, const Eigen::VectorXd& agent_configuration) {
    // Get the x,y location
    Eigen::Vector2d agent_position(agent_configuration(0), agent_configuration(1));
    Eigen::Vector2d closest_point;
    
    // Check if point robot
    if(problem.isPointAgent) {
        // In polygon collision
        if(isInsidePolygon(obstacle, agent_position) || distanceToObstacle(agent_position, obstacle, closest_point) <= 0.1) {
            return true;
        }
    } else {
        // In polygon collision and check geometry intersections as well
        // ADD GEOMETRIC CONSIDERATIONS HERE TOO
        if(polygonPolygonCollision(problem, obstacle, agent_configuration)) {
            return true;
        }
    }

    return false;
}


// @brief check if the system is in goal
bool MyKinoRRT::isSystemInGoal(const amp::KinodynamicProblem2D& problem, const Eigen::VectorXd& q) {
    // Check if the agent is within the goal bounds
    int i = 0;
    for(const auto& goal : problem.q_goal) {
        if(q(i) < goal.first || q(i) > goal.second) {
            return false;
        }
        i++;
    }
    return true;
}


// @brief check if the car robot is in collision with an obstacle
bool MyKinoRRT::polygonPolygonCollision(const amp::KinodynamicProblem2D& problem, const amp::Obstacle2D& obstacle, const Eigen::VectorXd& agent_configuration) {
    // Define vertices of the agent polygon
    Eigen::Vector2d v1(agent_configuration(0) + problem.agent_dim.width / 2.0 * std::sin(agent_configuration(2)), agent_configuration(1) - problem.agent_dim.width / 2.0 * std::cos(agent_configuration(2)));
    Eigen::Vector2d v2(v1(0) + problem.agent_dim.length * std::cos(agent_configuration(2)), v1(1) + problem.agent_dim.length * std::sin(agent_configuration(2)));
    Eigen::Vector2d v3(v2(0) - problem.agent_dim.width * std::sin(agent_configuration(2)), v2(1) + problem.agent_dim.width * std::cos(agent_configuration(2)));
    Eigen::Vector2d v4(v3(0) - problem.agent_dim.length * std::cos(agent_configuration(2)), v3(1) - problem.agent_dim.length * std::sin(agent_configuration(2)));

    // Add the vertices to the agent polygon
    std::vector<Eigen::Vector2d> agent_polygon = {v1, v2, v3, v4};

    // Use line segment intersection to see if any of the lines interesct an obstacle
    for(int i = 0; i < agent_polygon.size(); i++) {
        Eigen::Vector2d p1 = agent_polygon[i];
        Eigen::Vector2d p2 = agent_polygon[(i + 1) % agent_polygon.size()];
        
        // Check for collision
        if(inCollision(problem, p1, p2)) {
            return true;
        }
    }

    // Also check if any of the vertices are outside the bounds
    for(const auto& vertex : agent_polygon) {
        if(vertex(0) < problem.x_min || vertex(0) > problem.x_max || vertex(1) < problem.y_min || vertex(1) > problem.y_max) {
            return true;
        }
    }
    return false;
}
