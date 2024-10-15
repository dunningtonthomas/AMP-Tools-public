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

    // path.waypoints.push_back(problem.q_init);
    // path.waypoints.push_back(problem.q_goal);
  
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

// Example of creating a graph and adding nodes for visualization
// std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
// std::map<amp::Node, Eigen::Vector2d> nodes;

// std::vector<Eigen::Vector2d> points = {{3, 3}, {4, 5}, {5, 3}, {6, 5}, {5, 7}, {7, 3}}; // Points to add to the graph
// for (amp::Node i = 0; i < points.size(); ++i) nodes[i] = points[i]; // Add point-index pair to the map
// std::vector<std::tuple<amp::Node, amp::Node, double>> edges = {{0, 4, 1}, {0, 5, 1}, {4, 5, 1}, {1, 2, 1}, {1, 3, 1}, {2, 3, 1}}; // Edges to connect
// for (const auto& [from, to, weight] : edges) graphPtr->connect(from, to, weight); // Connect the edges in the graph
// graphPtr->print();


// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(problem.q_goal);
    return path;
}