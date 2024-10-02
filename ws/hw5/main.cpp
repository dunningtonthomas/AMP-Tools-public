// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

// Helper functions
#include "helperFuncs.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Test your gradient descent algorithm on a random problem.
    //double d_star = 2.0, zetta = 3.0, Q_star = 1.0, eta = 1.5;
    double d_star = 2.0, zetta = 0.9, Q_star = 0.5, eta = 1.5;
    MyGDAlgorithm algo(d_star, zetta, Q_star, eta);

    // Test on a random problem
    // amp::Path2D path;
    // amp::Problem2D prob;
    // bool success = HW5::generateAndCheck(algo, path, prob);
    // Visualizer::makeFigure(prob, path);

    // Test on the first workspace
    // amp::Problem2D prob = HW5::getWorkspace1();
    // amp::Path2D path = algo.plan(prob);

    // Test hw2 workspace
    amp::Problem2D prob = HW2::getWorkspace1();
    amp::Path2D path = algo.plan(prob);

    // Test hw2 workspace
    // amp::Problem2D prob = HW2::getWorkspace2();
    // amp::Path2D path = algo.plan(prob);

    // Visualize your potential function
    amp::Visualizer::makeFigure(prob, path);
    amp::Visualizer::makeFigure(MyPotentialFunction{prob, d_star, zetta, Q_star, eta}, prob.x_min, prob.x_max, prob.y_min, prob.y_max, 500);
    Visualizer::showFigures();
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    //HW5::grade<MyGDAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv, d_star, zetta, Q_star, eta);
    return 0;
}