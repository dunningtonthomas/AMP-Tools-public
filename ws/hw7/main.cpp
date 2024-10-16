// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"

using namespace amp;

int main(int argc, char** argv) {
    //HW7::hint(); // Consider implementing an N-dimensional planner 

    // Get the problem
    //Problem2D problem = HW2::getWorkspace1();
    Problem2D problem = HW5::getWorkspace1();

    // Test PRM on Workspace1 of HW2
    int n = 300;
    double r = 1.5;
    MyPRM prm(n, r);
    //Visualizer::makeFigure(problem, prm.plan(problem), *graphPtr, nodes);

    // Test the PRM graph
    std::map<amp::Node, Eigen::Vector2d> prm_nodes;
    std::shared_ptr<amp::Graph<double>> prm_graph = prm.createGraph(problem, prm_nodes);
    Visualizer::makeFigure(problem, prm.plan(problem), *prm_graph, prm_nodes);

    // Generate a random problem and test RRT
    Path2D path;
    MyRRT rrt;
    std::map<amp::Node, Eigen::Vector2d> rrt_nodes;
    std::map<amp::Node, amp::Node> rrt_parents;
    std::shared_ptr<amp::Graph<double>> rrt_graph = rrt.createGraph(problem, rrt_nodes, rrt_parents);
    //HW7::generateAndCheck(rrt, path, problem);
    Visualizer::makeFigure(problem, rrt.plan(problem), *rrt_graph, rrt_nodes);
    
    // Grade method
    Visualizer::showFigures();
    //HW7::grade<MyPRM, MyRRT>("firstName.lastName@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}