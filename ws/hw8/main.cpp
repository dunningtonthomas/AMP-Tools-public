// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"
#include <chrono>

using namespace amp;

void timer_example() {
    double startTime;
    amp::Timer timer("timer");
    for (int i=0; i < 5; ++i) {
        startTime = timer.now(TimeUnit::ms);  
        std::cout << "Press any key to continue...\n";
        std::cin.get();
        std::cout << "Time since last run: " << timer.now(TimeUnit::ms) - startTime << std::endl;
    }
    timer.stop();
    std::cout << "Total time elapsed: " << Profiler::getTotalProfile("timer") << std::endl;
}

int main(int argc, char** argv) {
    // Run timer example (useful for benchmarking)
    //timer_example();
    bool benchmark_central = false;
    bool benchmark_decentral = false;

    // Initialize Workspace 1 with 3 agents
    amp::RNG::seed(amp::RNG::randiUnbounded());
    MultiAgentProblem2D problem = HW8::getWorkspace1(2);
    std::vector<std::vector<Eigen::Vector2d>> collision_states;

    // Solve using a centralized approach
    // MyCentralPlanner central_planner;
    // MultiAgentPath2D path = central_planner.plan(problem);
    // bool isValid = HW8::check(path, problem, collision_states);
    // Visualizer::makeFigure(problem, path, collision_states);

    // Solve using a decentralized approach
    // MyDecentralPlanner decentral_planner;
    // MultiAgentPath2D path = decentral_planner.plan(problem);
    // collision_states = {{}};
    // bool isValid = HW8::check(path, problem, collision_states);
    // Visualizer::makeFigure(problem, path, collision_states);


    // Benchmarking central
    if(benchmark_central) {
        std::list<std::vector<double>> tree_sizes, path_times;
        std::vector<double> tree_size, path_time;
        for(int i = 0; i < 10; i++) {
            // Make object
            MyCentralPlanner central_planner_bench;

            // Time the time it takes
            auto start = std::chrono::high_resolution_clock::now();
            MultiAgentPath2D path = central_planner_bench.plan(problem);
            auto end = std::chrono::high_resolution_clock::now();

            // Calculate the time it took
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            double elapsed_seconds = elapsed.count();
            path_time.push_back(elapsed_seconds);
            tree_size.push_back(central_planner_bench.getTreeSize());
        }

        // Add 
        tree_sizes.push_back(tree_size);
        path_times.push_back(path_time);

        std::vector<std::string> labels = {"100"};
        Visualizer::makeBoxPlot(path_times, labels, "5 Agents Time", "Number of runs", "Computation Time (ms)");
        Visualizer::makeBoxPlot(tree_sizes, labels, "5 Agents Tree Size", "Number of runs", "Tree Size (#nodes)");
    }

    // Benchmarking decentral
    if(benchmark_decentral) {
        // Loop through all benchmarking number of agents
        std::vector<int> num_agents = {2, 3, 4, 5, 6};
        for(const auto& num_agent : num_agents) {
            MultiAgentProblem2D problem = HW8::getWorkspace1(num_agent);
            std::list<std::vector<double>> path_times;
            std::vector<double> path_time;
            for(int i = 0; i < 100; i++) {
                // Make object
                MyDecentralPlanner decentral_planner_bench;

                // Time the time it takes
                auto start = std::chrono::high_resolution_clock::now();
                MultiAgentPath2D path = decentral_planner_bench.plan(problem);
                auto end = std::chrono::high_resolution_clock::now();

                // Calculate the time it took
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                double elapsed_seconds = elapsed.count();
                path_time.push_back(elapsed_seconds);
            }

            // Add 
            path_times.push_back(path_time);

            std::vector<std::string> labels = {"100"};
            std::string title = std::to_string(num_agent) + " Agents Time";
            Visualizer::makeBoxPlot(path_times, labels, title, "Number of runs", "Computation Time (ms)");
        }
    }


    // Visualize and grade methods
    Visualizer::showFigures();
    //HW8::grade<MyCentralPlanner, MyDecentralPlanner>("thomas.dunnington@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}