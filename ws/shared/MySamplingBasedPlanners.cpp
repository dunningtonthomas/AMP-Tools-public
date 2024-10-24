#include "MySamplingBasedPlanners.h"

// @brief Execute PRM with n sampled configuartions and a connect a node to neighbors within radius r
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    // Create the PRM graph
    createGraph(problem);

    // Create the problem for the A* algorithm
    amp::ShortestPathProblem graph_problem;
    graph_problem.graph = graphPtr;
    graph_problem.init_node = 0;
    graph_problem.goal_node = 1;

    // Create a heuristic using distance to the goal
    LookupSearchHeuristic distance_heuristic;
    for(auto [node, point] : nodes) {
        // Set the init node heuristic to zero
        if(node == 0) {
            distance_heuristic.heuristic_values[node] = 0;
        } else {
            distance_heuristic.heuristic_values[node] = amp::distance(point, problem.q_goal);
        }
    }
    // for(auto node : graphPtr->nodes()) {
    //     distance_heuristic.heuristic_values[node] = amp::distance(nodes[node], problem.q_goal);
    // }

    // Run A* to find the shortest path
    MyAStarAlgo astar;
    MyAStarAlgo::GraphSearchResult astar_result = astar.search(graph_problem, distance_heuristic);

    // Set the success variable
    success = astar_result.success;

    // Create the path from the astar result
    amp::Path2D path;
    if(success) {
        for(auto node : astar_result.node_path) {
            path.waypoints.push_back(nodes[node]);
        }
    } else {
        // Just add the initial point and the goal
        path.waypoints.push_back(problem.q_init);
        path.waypoints.push_back(problem.q_goal);
    }


    // Path smoothing
    //pathSmoothing(path, problem);
    return path;
}


void MyPRM::createGraph(const amp::Problem2D& problem) {
    // Add the nodes to the graph
    amp::Node init_node = 0, goal_node = 1;
    nodes[init_node] = problem.q_init;
    nodes[goal_node] = problem.q_goal;

    // Sample n configurations
    for(int i = 0; i < n; i++) {
        // Sample a random point in the environment
        Eigen::Vector2d q_rand = amp::randomConfiguration(problem.x_min, problem.x_max, problem.y_min, problem.y_max);
        //Eigen::Vector2d q_rand = amp::randomConfiguration(-1.0, 11.0, -3.0, 3.0);

        // Check if the sampled point is valid
        if(!inCollision_point(problem, q_rand)) {
            // Add the point to the graph
            amp::Node add_node = nodes.size();
            nodes[add_node] = q_rand;
        }
    }

    // Connect the nodes in the graph
    std::vector<std::tuple<amp::Node, amp::Node, double>> edges;
    for(const auto& [node, point] : nodes) {
        for(const auto& [neighbor, neighbor_point] : nodes) {
            // Check if the neighbor is within the desired radius
            if(amp::distance(point, neighbor_point) < r && node != neighbor) {
                // Check for collision
                if(!inCollision(problem, point, neighbor_point)) {
                    // Add the edge to the graph
                    edges.push_back({node, neighbor, amp::distance(point, neighbor_point)});
                }
            }
        }
    }

    // Create the graph
    graphPtr = std::make_shared<amp::Graph<double>>();
    for (const auto& [from, to, weight] : edges) {
        graphPtr->connect(from, to, weight);
    }
}


// @brief Implement path smoothing by removing unnecessary waypoints
void MyPRM::pathSmoothing(amp::Path2D& path, const amp::Problem2D& problem) {
    // Smooth 100 times
    for(int i = 0; i < 100; i++) {
        // return if the path is too short
        if(path.waypoints.size() < 3) {
            return;
        }

        // Iterate through the path and remove unnecessary waypoints
        for(int i = 0; i < path.waypoints.size() - 2; i++) {
            // Check if the path is valid
            if(!inCollision(problem, path.waypoints[i], path.waypoints[i + 2])) {
                // Remove the middle waypoint
                path.waypoints.erase(path.waypoints.begin() + i + 1);
            }
        }
    }
}




// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    // Create the RRT graph
    createGraph(problem);

    // If a solution is found, return the path to goal
    amp::Path2D path;
    if(nodes[nodes.size() - 1] == problem.q_goal) {
        // Recreate the path from the parents map
        amp::Node curr_node = nodes.size() - 1;
        while(nodes[curr_node] != problem.q_init) {
            path.waypoints.push_back(nodes[curr_node]);
            curr_node = parents[curr_node];
        }
        path.waypoints.push_back(nodes[curr_node]);

        // Reverse the path
        std::reverse(path.waypoints.begin(), path.waypoints.end());

        // Set the success variable
        success = true;
    } else {
        // Just return the init and goal locations
        path.waypoints.push_back(problem.q_init);
        path.waypoints.push_back(problem.q_goal);
    }

    return path;
}




void MyRRT::createGraph(const amp::Problem2D& problem) {
    // Set the initial node to 0
    nodes[0] = problem.q_init;

    // Edges vector
    std::vector<std::tuple<amp::Node, amp::Node, double>> edges;

    // Iterate to find the goal
    int iteration = 0;
    while(iteration < max_iterations) {
        // Generate random sample in the environment, biased toward the goal
        Eigen::Vector2d q_rand;
        if(amp::randomDouble(0.0, 1.0) < goal_bias) {
            // Sample the goal
            q_rand = problem.q_goal;
        } else {
            // Sample a random point in the environment
            q_rand = amp::randomConfiguration(problem.x_min, problem.x_max, problem.y_min, problem.y_max);
        }

        // Check if the sampled point is valid
        if(!inCollision_point(problem, q_rand)) {
            // Find the nearest neigbor to the random point
            amp::Node nearest_node = nearestNeighbor(q_rand);

            // Generate the path taking a step size
            Eigen::Vector2d q_near = nodes[nearest_node];

            // Take a single step towards the random point
            Eigen::Vector2d q_new = q_near + step_size * (q_rand - q_near).normalized();

            // Check if the path is valid
            if(!inCollision(problem, q_near, q_new)) {
                // Add the edge to the graph
                amp::Node add_node = nodes.size();
                nodes[add_node] = q_new;
                edges.push_back({nearest_node, add_node, amp::distance(q_near, q_new)});

                // Add the parent to the parent map
                parents[add_node] = nearest_node;

                // Check if a solution is found
                if(amp::distance(q_new, problem.q_goal) < epsilon) {
                    // Add the goal to the graph
                    amp::Node goal_node = nodes.size();
                    nodes[goal_node] = problem.q_goal;
                    edges.push_back({add_node, goal_node, amp::distance(q_new, problem.q_goal)});
                    parents[goal_node] = add_node;
                    std::cout << "RRT Path found in "<< iteration << " iterations, reconstructing path..." << std::endl;
                    break;
                }
            }
        }
        iteration++;
    }

    // Create the graph
    graphPtr = std::make_shared<amp::Graph<double>>();
    for (const auto& [from, to, weight] : edges) {
        graphPtr->connect(from, to, weight);
    }
}


amp::Node MyRRT::nearestNeighbor(const Eigen::Vector2d& q_rand) {
    amp::Node nearest_node = 0;
    double min_distance = std::numeric_limits<double>::max();
    for(const auto& [node, point] : nodes) {
        double distance = amp::distance(point, q_rand);
        if(distance < min_distance && distance != 0) {
            min_distance = distance;
            nearest_node = node;
        }
    }
    return nearest_node;
}