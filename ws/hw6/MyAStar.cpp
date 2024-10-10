#include "MyAStar.h"

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    GraphSearchResult result = {false, {}, 0.0}; // initialize the results object
    result.path_cost = 0.0;

    // Initialize the priority queue
    std::priority_queue<std::pair<double, amp::Node>, std::vector<std::pair<double, amp::Node>>, std::greater<std::pair<double, amp::Node>>> pq;

    // Map to store the parent of each node
    std::unordered_map<amp::Node, amp::Node> parent_map;

    // Map to store the cost to reach each node and serve as the closed list
    std::unordered_map<amp::Node, double> cost_map;

    // Add the initial node to the priority queue
    pq.push({heuristic(problem.init_node), problem.init_node});
    cost_map[problem.init_node] = 0.0;

    while(!pq.empty()) {
        // Get the current node
        amp::Node curr_node = pq.top().second;
        pq.pop();

        // Check if the goal has been reached
        if(curr_node == problem.goal_node) {
            result.success = true;
            break;
        }

        // Get the path cost to the current node
        double curr_cost = cost_map[curr_node];

        // Get the neighbors of the current node and the edge costs to the neighbors
        std::vector<amp::Node> neighbors = problem.graph->children(curr_node);
        std::vector<double> edge_costs = problem.graph->outgoingEdges(curr_node);

        // Iterate through the neighbors
        for(int i = 0; i < neighbors.size(); i++) {
            // Get the neighbor and edge cost
            amp::Node neighbor = neighbors[i];
            double edge_cost = edge_costs[i];

            // Calculate the cost to move to the neighbor
            double cost = cost_map[curr_node] + edge_cost;

            // Check if the neighbor has been visited or if the cost is less than the current cost
            if(cost_map.find(neighbor) == cost_map.end() || cost < cost_map[neighbor]) {
                // Update the cost and add the neighbor to the priority queue
                cost_map[neighbor] = cost;
                pq.push({cost + heuristic(neighbor), neighbor});
                parent_map[neighbor] = curr_node;
            }
        }
    }

    if(result.success) {
        // Get the path from the goal to the initial node
        std::cout << "Path found, reconstructing path..." << std::endl;
        amp::Node curr_node = problem.goal_node;
        result.path_cost = cost_map[curr_node];
        while(curr_node != problem.init_node) {
            result.node_path.push_front(curr_node);
            curr_node = parent_map[curr_node];
        }
        result.node_path.push_front(problem.init_node);
    } else {
        std::cout << "No path found" << std::endl;
    }

    //result.print();
    return result;
}
