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
    bool benchmark_RRT = true;

    // Find a path in the new environment
    // MyKinoRRT kino_planner_test;
    // KinoPath path = kino_planner_test.plan(prob, *agentFactory[prob.agent_type]());

    // Use the guidance level to plan
    // adaptiveRRT adaptive_rrt_kino;
    // KinoPath adaptive_path_kino = adaptive_rrt_kino.plan(prob, agent);

    // // // Visualize the path
    // if (adaptive_path_kino.valid) {
    //     Visualizer::makeFigure(prob, adaptive_path_kino, false); // Set to 'true' to render animation
    //     //Visualizer::makeFigure(prob, path, true); // Set to 'true' to render animation
    // }


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

    // Benchmark the RRT algorithm and write times to a file
    if(benchmark_RRT) {
        std::cout << "Benchmarking RRT" << std::endl;
        std::ofstream time_file;
        time_file.open("../../file_dump/adaptive_times.txt");
        int num_bench = 1000;
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