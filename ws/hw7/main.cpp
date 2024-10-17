// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"
#include <chrono>

using namespace amp;

int main(int argc, char** argv) {
    //HW7::hint(); // Consider implementing an N-dimensional planner 
    bool benchmark_bool = true;
    bool hw5_bool = false;

    // Get the problem
    //Problem2D problem = HW2::getWorkspace1();
    Problem2D problem = HW2::getWorkspace2();
    //Problem2D problem = HW5::getWorkspace1();

    // Test PRM on Workspace1 of HW2
    MyPRM prm(200, 2);
    Path2D prm_path = prm.plan(problem);
    std::map<amp::Node, Eigen::Vector2d> prm_nodes = prm.getNodes();
    std::shared_ptr<amp::Graph<double>> prm_graph = prm.getGraph();
    Visualizer::makeFigure(problem, prm_path, *prm_graph, prm_nodes);
    std::cout << "PRM Path Length: " << prm_path.length() << std::endl; 

    // Generate a random problem and test RRT
    //HW7::generateAndCheck(rrt, path, problem);
    MyRRT rrt;
    Path2D path = rrt.plan(problem);
    std::map<amp::Node, Eigen::Vector2d> rrt_nodes = rrt.getNodes();
    std::shared_ptr<amp::Graph<double>> rrt_graph = rrt.getGraph();
    //Visualizer::makeFigure(problem, path, *rrt_graph, rrt_nodes);

    // Benchmarking
    if(benchmark_bool) {
        std::vector<std::pair<double, double>> prm_bench_values_1 = {{200, 0.5}, {200, 1}, {200, 1.5}, {200, 2}, {500, 0.5}, {500, 1}, {500, 1.5}, {500, 2}};
        std::vector<std::pair<double, double>> prm_bench_values_2 = {{200, 1}, {200, 2}, {500, 1}, {500, 2}, {1000, 1}, {1000, 2}};
        std::vector<std::string> labels_1 = {"n=200, r=0.5", "n=200, r=1", "n=200, r=1.5", "n=200, r=2", "n=500, r=0.5", "n=500, r=1", "n=500, r=1.5", "n=500, r=2"};
        std::vector<std::string> labels_2 = {"n=200, r=1", "n=200, r=2", "n=500, r=1", "n=500, r=2", "n=1000, r=1", "n=1000, r=2"};
        std::list<std::vector<double>> path_lengths, path_times;
        std::vector<double> success_rates;

        // Vaiues for if hw5 or hw2
        std::vector<std::pair<double, double>> prm_bench_values;
        if(hw5_bool) {
            prm_bench_values = prm_bench_values_1;
        } else {
            prm_bench_values = prm_bench_values_2;
        }

        std::vector<std::string> labels;
        if(hw5_bool) {
            labels = labels_1;
        } else {
            labels = labels_2;
        }
        
        // Run each case 100 times
        std::cout << "Benchmarking PRM..." << std::endl;
        for(const auto& [n, r] : prm_bench_values) {
            std::vector<double> path_length, path_time, success_rate;
            std::cout << "Running PRM with n = " << n << " and r = " << r << std::endl;
            //MyPRM prm_bench(n, r);
            for(int i = 0; i < 100; i++) {
                // Make object
                MyPRM prm_bench(n, r);

                // Time the time it takes
                auto start = std::chrono::high_resolution_clock::now();
                Path2D path = prm_bench.plan(problem);
                auto end = std::chrono::high_resolution_clock::now();

                // Calculate the time it took
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                double elapsed_seconds = elapsed.count();

                // Add path length if there is a valid path
                if(prm_bench.getSuccess()) {
                    path_length.push_back(path.length());
                    success_rate.push_back(1);
                } else {
                    success_rate.push_back(0);
                }

                // Add time and success rate
                path_time.push_back(elapsed_seconds);
            }
            
            // Add vectors to the lists
            path_lengths.push_back(path_length);
            path_times.push_back(path_time);
            success_rates.push_back(std::accumulate(success_rate.begin(), success_rate.end(), 0));
        }

        // Box Plots
        Visualizer::makeBoxPlot(path_lengths, labels, "Path Lengths", "PRM Configuration", "Path Length");
        Visualizer::makeBoxPlot(path_times, labels, "Computation Time", "PRM Configuration", "Time (ms)");
        Visualizer::makeBarGraph(success_rates, labels, "Success Rates", "PRM Configuration", "Success Rate (%)");
    } // end benchmarking

    // Grade method
    Visualizer::showFigures();
    //HW7::grade<MyPRM, MyRRT>("firstName.lastName@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}