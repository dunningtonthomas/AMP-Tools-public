// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW9.h"
#include "hw/HW2.h"
#include "MyKinoRRT.h"
#include "MyRRT.h"
#include "createEnv.h"
#include "guidance.h"
#include <chrono>
#include <fstream>
#include <cmath>

using namespace amp;

std::vector<KinodynamicProblem2D> problems = {HW9::getStateIntProblemWS1(), HW9::getStateIntProblemWS2(), HW9::getFOUniProblemWS1(), HW9::getFOUniProblemWS2(), HW9::getSOUniProblemWS1(), HW9::getSOUniProblemWS2(), HW9::getCarProblemWS1(), HW9::getParkingProblem()};
std::unordered_map<AgentType, std::function<std::shared_ptr<amp::DynamicAgent>()>> agentFactory = {
    {AgentType::SingleIntegrator, []() { return std::make_shared<MySingleIntegrator>(); }},
    {AgentType::FirstOrderUnicycle, []() { return std::make_shared<MyFirstOrderUnicycle>(); }},
    {AgentType::SecondOrderUnicycle, []() { return std::make_shared<MySecondOrderUnicycle>(); }},
    {AgentType::SimpleCar, []() { return std::make_shared<MySimpleCar>(); }}
};


int main(int argc, char** argv) {
    // Create the problem
    rangeFindingCar agent;
    generateEnv obj;
    KinodynamicProblem2D prob = obj.getEnvRandKino();
    bool benchmark_RRT = false;
    bool find_two_replan = false;
    bool benchmark_RRT_total = true;

    // REGULAR RRT
    // Get a regular 2D problem
    Problem2D prob2D = obj.getEnvRand();
    rangeFindingCar agent_basic(5.0);
    //Visualizer::makeFigure(prob2D);

    // Use adaptive RRT to find a path
    adaptiveRRT adaptive_rrt;
    Path2D adaptive_path = adaptive_rrt.plan(prob2D, agent_basic);
    Visualizer::makeFigure(prob2D, adaptive_path);
    //std::cout << "Adaptive RRT Path Length: " << adaptive_path.length() << std::endl;
    // Output the adaptive times
    std::cout << "Adaptive RRT Times: " << std::endl;
    for(int i = 0; i < adaptive_path.adaptive_times.size(); i++) {
        std::cout << adaptive_path.adaptive_times[i] << std::endl;
    }


    // Get a path that has two replans
    if(find_two_replan) {
        int num_replans = 100;

        for(int i = 0; i < num_replans; i++) {
            // Get a regular 2D problem
            Problem2D prob2D_replan = obj.getEnvRand();
            rangeFindingCar agent_basic_replan(5.0);

            // Use adaptive RRT to find a path
            adaptiveRRT adaptive_rrt_replan;
            Path2D adaptive_path_replan = adaptive_rrt_replan.plan(prob2D_replan, agent_basic_replan);

            // See how many replans were needed
            if(adaptive_path_replan.adaptive_times.size() == 6) {
                // Break out of the loop
                std::cout << "Two replans found" << std::endl;
                break;
            }
        }
    }

    // Benchmark the RRT algorithm and write times to a file
    if(benchmark_RRT) {
        std::cout << "Benchmarking Online RRT" << std::endl;
        std::ofstream time_file;
        time_file.open("../../file_dump/adaptive_times_online.txt");
        // Check if the file opened correctly
        if (!time_file.is_open()) {
            std::cerr << "Error opening file" << std::endl;
            return -1;
        } else {
            std::cout << "File opened successfully" << std::endl;
        }
        int num_bench = 100;
        for(int i = 0; i < num_bench; i++) {
            adaptiveRRT adaptive_rrt_bench;
            Path2D adaptive_path_bench = adaptive_rrt_bench.plan(prob2D, agent_basic);
            for(int j = 0; j < adaptive_path_bench.adaptive_times.size(); j++) {
                time_file << adaptive_path_bench.adaptive_times[j] << " ";
            }   
            time_file << std::endl;
        }
        time_file.close();
    }

    // Benchmark the RRT algorithm and write times to a file
    if(benchmark_RRT_total) {
        std::cout << "Benchmarking RRT Offline vs Online" << std::endl;
        std::ofstream time_file;
        time_file.open("../../file_dump/offline_online_times.txt");
        // Check if the file opened correctly
        if (!time_file.is_open()) {
            std::cerr << "Error opening file" << std::endl;
            return -1;
        } else {
            std::cout << "File opened successfully" << std::endl;
        }

        int num_bench = 1000;
        for(int i = 0; i < num_bench; i++) {
            // Get a regular 2D problem
            Problem2D prob2D_bench = obj.getEnvRand();
            MyRRT rrt_bench;
            adaptiveRRT adaptive_rrt_bench;

            // Time both RRT methods
            auto start_time = std::chrono::high_resolution_clock::now();
            Path2D rrt_path_bench = rrt_bench.plan(prob2D_bench);
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_time = end_time - start_time;
            time_file << elapsed_time.count() << " ";

            // Time the adaptive RRT method
            start_time = std::chrono::high_resolution_clock::now();
            Path2D adaptive_path_bench = adaptive_rrt_bench.plan(prob2D_bench, agent_basic);
            end_time = std::chrono::high_resolution_clock::now();
            elapsed_time = end_time - start_time;
            time_file << elapsed_time.count() << " "; 
            time_file << std::endl;
        }
        time_file.close();
    }


    // MyKinoRRT kino_planner;
    // auto start_park = std::chrono::high_resolution_clock::now();
    // KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
    // auto end_park = std::chrono::high_resolution_clock::now();
    // HW9::check(path, prob);
    
    // // Visualize the path
    // if (path.valid) {
    //     Visualizer::makeFigure(prob, path, false); // Set to 'true' to render animation
    //     //Visualizer::makeFigure(prob, path, true); // Set to 'true' to render animation
    // }

    // Write the controls, time, and state data to a data file
    // std::ofstream data_file;
    // data_file.open("../../file_dump/controls.txt");

    // // Check if the file opened correctly
    // if (!data_file.is_open()) {
    //     std::cerr << "Error opening file" << std::endl;
    // } else {
    //     std::cout << "File opened successfully" << std::endl;
    //     double total_time = 0.0;
    //     for (int i = 0; i < path.controls.size(); i++) {
    //         total_time += path.durations[i];
    //         data_file << total_time << " " << path.controls[i](0) << " " << path.controls[i](1) << std::endl;
    //     }
    // }
    // data_file.close();


    Visualizer::showFigures();
    return 0;
}