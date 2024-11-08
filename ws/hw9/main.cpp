// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW9.h"
#include "hw/HW2.h"
#include "MyKinoRRT.h"
#include <chrono>
#include <fstream>

using namespace amp;

// Load problems and map agent for quick testing
std::vector<KinodynamicProblem2D> problems = {HW9::getStateIntProblemWS1(), HW9::getStateIntProblemWS2(), HW9::getFOUniProblemWS1(), HW9::getFOUniProblemWS2(), HW9::getSOUniProblemWS1(), HW9::getSOUniProblemWS2(), HW9::getCarProblemWS1(), HW9::getParkingProblem()};
std::unordered_map<AgentType, std::function<std::shared_ptr<amp::DynamicAgent>()>> agentFactory = {
    {AgentType::SingleIntegrator, []() { return std::make_shared<MySingleIntegrator>(); }},
    {AgentType::FirstOrderUnicycle, []() { return std::make_shared<MyFirstOrderUnicycle>(); }},
    {AgentType::SecondOrderUnicycle, []() { return std::make_shared<MySecondOrderUnicycle>(); }},
    {AgentType::SimpleCar, []() { return std::make_shared<MySimpleCar>(); }}
};

int main(int argc, char** argv) {
    // Select problem, plan, check, and visualize
    bool benchmark = false;
    int select = 7;
    KinodynamicProblem2D prob = problems[select];

    // Hardcode the control bounds if it is the car problem
    if(select == 7 || select == 6) {
        prob.u_bounds = {{-1.5, 2.0}, {-0.3, 0.3}};
    }

    MyKinoRRT kino_planner;
    auto start_park = std::chrono::high_resolution_clock::now();
    KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
    auto end_park = std::chrono::high_resolution_clock::now();
    HW9::check(path, prob);
    
    // Visualize the path
    if (path.valid) {
        Visualizer::makeFigure(prob, path, false); // Set to 'true' to render animation
        //Visualizer::makeFigure(prob, path, true); // Set to 'true' to render animation
    }

    // Calulate the path length
    double length = 0.0;
    for (int i = 1; i < path.waypoints.size(); i++) {
        length += (path.waypoints[i] - path.waypoints[i - 1]).norm();
    }
    std::cout << "Path Length: " << length << std::endl;

    // Calculate the computation time
    std::cout << "Computation Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_park - start_park).count() << " ms" << std::endl;

    // Write the controls data and the time to a data file
    std::ofstream data_file;
    data_file.open("../../file_dump/controls.txt");

    // Check if the file opened correctly
    if (!data_file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
    } else {
        std::cout << "File opened successfully" << std::endl;
        double total_time = 0.0;
        for (int i = 0; i < path.controls.size(); i++) {
            total_time += path.durations[i];
            data_file << total_time << " " << path.controls[i](0) << " " << path.controls[i](1) << std::endl;
        }
    }
    data_file.close();


    // Benchmark the planner
    if(benchmark) {
        std::cout << "Benchmarking MyKinoRRT..." << std::endl;
        int num_runs = 50;
        std::vector<int> u_samples = {1, 5, 10, 15};

        // List of path lengths and times
        std::list<std::vector<double>> path_lengths, path_times;
        std::vector<double> success_rates;

        // Run for each number of samples
        for(const auto& u : u_samples) {
            // Run each case 50 times
            std::vector<double> path_length, path_time;
            std::vector<double> valid_solutions;
            for(int i = 0; i < num_runs; i++) {
                // Make object
                MyKinoRRT kino_planner_bench(u);

                // Time the time it takes
                auto start = std::chrono::high_resolution_clock::now();
                KinoPath path = kino_planner_bench.plan(prob, *agentFactory[prob.agent_type]());
                auto end = std::chrono::high_resolution_clock::now();

                // Calculate the path length
                double length = 0.0;
                for (int i = 1; i < path.waypoints.size(); i++) {
                    length += (path.waypoints[i] - path.waypoints[i - 1]).norm();
                }

                // Push results
                path_length.push_back(length);
                path_time.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

                // Push valid
                if(path.valid) {
                    valid_solutions.push_back(1);
                } else {
                    valid_solutions.push_back(0);
                }
            }
            // Add times and lengths to list
            path_lengths.push_back(path_length);
            path_times.push_back(path_time);
            success_rates.push_back(std::accumulate(valid_solutions.begin(), valid_solutions.end(), 0));
        }

        // Visualize the results
        std::vector<std::string> labels = {"1", "5", "10", "15"};
        //Visualizer::makeBoxPlot(path_times, labels, "Path Time", "Number of Control Samples", "Computation Time (ms)");
        //Visualizer::makeBoxPlot(path_lengths, labels, "Path Length", "Number of Control Samples", "Path Length (m)");
        //Visualizer::makeBarGraph(success_rates, labels, "Success Rates", "Number of Control Samples", "Number of Valid Solutions");
    }

    Visualizer::showFigures();
    //HW9::grade<MyKinoRRT, MySingleIntegrator, MyFirstOrderUnicycle, MySecondOrderUnicycle, MySimpleCar>("thomas.dunnington@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple());
    return 0;
}