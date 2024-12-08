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
    //KinodynamicProblem2D prob;
    //KinodynamicProblem2D prob = problems[0];
    // TODO: Create a problem with buildings as obstacles

    // Get the problem
    // generateEnv obj;
    // KinodynamicProblem2D prob = obj.getEnv1();
    // Visualizer::makeFigure(prob);

    // // Find a path in the new environment
    // MyKinoRRT kino_planner_test;
    // KinoPath path = kino_planner_test.plan(prob, *agentFactory[prob.agent_type]());

    // // Visualize the path
    // if (path.valid) {
    //     Visualizer::makeFigure(prob, path, false); // Set to 'true' to render animation
    //     //Visualizer::makeFigure(prob, path, true); // Set to 'true' to render animation
    // }

    // Get a regular 2D problem
    generateEnv obj;
    Problem2D prob2D = obj.getEnvRand();
    Visualizer::makeFigure(prob2D);

    // Use RRT to find a path
    MyRRT rrt;
    Path2D path2D = rrt.plan(prob2D);
    std::map<amp::Node, Eigen::Vector2d> rrt_nodes = rrt.getNodes();
    std::shared_ptr<amp::Graph<double>> rrt_graph = rrt.getGraph();
    Visualizer::makeFigure(prob2D, path2D, *rrt_graph, rrt_nodes);
    std::cout << "RRT Path Length: " << path2D.length() << std::endl;

    // Use adaptive RRT to find a path
    rangeFindingCar agent;
    adaptiveRRT adaptive_rrt;
    Path2D adaptive_path = adaptive_rrt.plan(prob2D, agent);
    Visualizer::makeFigure(prob2D, adaptive_path);
    std::cout << "Adaptive RRT Path Length: " << adaptive_path.length() << std::endl;


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