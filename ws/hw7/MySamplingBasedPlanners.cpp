# include "MySamplingBasedPlanners.h"

// @brief Execute PRM with n sampled configuartions and a connect a node to neighbors within radius r
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    // Create the nodes map
    std::map<amp::Node, Eigen::Vector2d> nodes;

    // Create the PRM graph
    std::shared_ptr<amp::Graph<double>> graphPtr = createGraph(problem, nodes);

    // Create the problem for the A* algorithm
    amp::ShortestPathProblem graph_problem;
    graph_problem.graph = graphPtr;
    graph_problem.init_node = 0;
    graph_problem.goal_node = 1;

    // Create a heuristic using distance to the goal
    amp::SearchHeuristic distance_heuristic;
    // for(auto node : graphPtr->nodes()) {
    //     distance_heuristic.heuristic_values[node] = amp::distance(nodes[node], problem.q_goal);
    // }

    // Run A* to find the shortest path
    MyAStarAlgo astar;
    MyAStarAlgo::GraphSearchResult astar_result = astar.search(graph_problem, distance_heuristic);

    // Create the path from the astar result
    amp::Path2D path;
    for(auto node : astar_result.node_path) {
        path.waypoints.push_back(nodes[node]);
    }

    return path;
}


std::shared_ptr<amp::Graph<double>> MyPRM::createGraph(const amp::Problem2D& problem, std::map<amp::Node, Eigen::Vector2d>& nodes) {
    // Add the nodes to the graph
    amp::Node init_node = 0, goal_node = 1;
    nodes[init_node] = problem.q_init;
    nodes[goal_node] = problem.q_goal;

    // Sample n configurations
    for(int i = 0; i < n; i++) {
        // Sample a random point in the environment
        Eigen::Vector2d q_rand = amp::randomConfiguration(problem);

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
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    for (const auto& [from, to, weight] : edges) {
        graphPtr->connect(from, to, weight);
    }

    // Return the graph
    return graphPtr;
}


// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    // Create the nodes map
    std::map<amp::Node, Eigen::Vector2d> nodes;

    // Create the parents map
    std::map<amp::Node, amp::Node> parents;

    // Create the RRT graph
    createGraph(problem, nodes, parents);

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
    } else {
        // Just return the init and goal locations
        path.waypoints.push_back(problem.q_init);
        path.waypoints.push_back(problem.q_goal);
    }

    return path;
}




std::shared_ptr<amp::Graph<double>> MyRRT::createGraph(const amp::Problem2D& problem, std::map<amp::Node, Eigen::Vector2d>& nodes, std::map<amp::Node, amp::Node>& parents) {
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
            q_rand = amp::randomConfiguration(problem);
        }

        // Check if the sampled point is valid
        if(!inCollision_point(problem, q_rand)) {
            // Find the nearest neigbor to the random point
            amp::Node nearest_node = nearestNeighbor(q_rand, nodes);

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
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    for (const auto& [from, to, weight] : edges) {
        graphPtr->connect(from, to, weight);
    }

    return graphPtr;
}


amp::Node MyRRT::nearestNeighbor(const Eigen::Vector2d& q_rand, const std::map<amp::Node, Eigen::Vector2d>& nodes) {
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