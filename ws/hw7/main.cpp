// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"

using namespace amp;

int main(int argc, char** argv) {
    //HW7::hint(); // Consider implementing an N-dimensional planner 

    // Get the problem
    Problem2D problem = HW2::getWorkspace1();
    //Problem2D problem = HW2::getWorkspace2();
    //Problem2D problem = HW5::getWorkspace1();

    // Test PRM on Workspace1 of HW2
    int n = 200;
    double r = 1;
    MyPRM prm(n, r);
    Path2D prm_path = prm.plan(problem);
    std::map<amp::Node, Eigen::Vector2d> prm_nodes = prm.getNodes();
    std::shared_ptr<amp::Graph<double>> prm_graph = prm.getGraph();
    Visualizer::makeFigure(problem, prm_path, *prm_graph, prm_nodes);

    // Generate a random problem and test RRT
    //HW7::generateAndCheck(rrt, path, problem);
    MyRRT rrt;
    Path2D path = rrt.plan(problem);
    std::map<amp::Node, Eigen::Vector2d> rrt_nodes = rrt.getNodes();
    std::shared_ptr<amp::Graph<double>> rrt_graph = rrt.getGraph();
    Visualizer::makeFigure(problem, path, *rrt_graph, rrt_nodes);
    
    // Grade method
    Visualizer::showFigures();
    //HW7::grade<MyPRM, MyRRT>("firstName.lastName@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}