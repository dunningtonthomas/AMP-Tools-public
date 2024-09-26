// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

// Include the header of the shared class
#include "HelpfulClass.h"

// Include the header of the shared helper functions
#include "helperFuncs.h"



using namespace amp;

int main(int argc, char** argv) {
    ////////////////////////////////////////////////////////////////////////////
    // Problem 1
    ////////////////////////////////////////////////////////////////////////////
    // Get the triangle obstacle
    Obstacle2D obstacle = HW4::getEx1TriangleObstacle();

    // Calculate negative object
    std::vector<Eigen::Vector2d> negative_vertices = negativeVertices(obstacle.verticesCCW());

    // Create the robot
    Obstacle2D robot(negative_vertices);

    // Calculate the minkowski sum for the robot and the obstacle
    Obstacle2D c_space_obj = minkowski(obstacle, robot);

    // Create 12 evenly space angles from 0 to 2pi
    std::vector<double> angles;
    for (int i = 0; i < 12; i++) {
        angles.push_back(i * M_PI / 6);
    }

    // Create 12 orientations of the robot
    std::vector<Obstacle2D> robot_objects;
    for (double angle : angles) {
        robot_objects.push_back(rotatePolygon(obstacle, angle));
    }

    // Create 12 C-space objects
    std::vector<Obstacle2D> c_space_objects;
    for (Obstacle2D& robot_obj : robot_objects) {
        // Calculate negative object
        std::vector<Eigen::Vector2d> negative_vertices = negativeVertices(robot_obj.verticesCCW());

        // Create the robot
        Obstacle2D robot_neg(negative_vertices);

        // Calculate the minkowski sum for the robot and the obstacle
        Obstacle2D c_space_obj_ang = minkowski(obstacle, robot_neg);
        c_space_objects.push_back(c_space_obj_ang);
    }

    // Create a vector of strings for the labels
    std::vector<std::string> labels;
    labels.push_back("Obstacle");
    labels.push_back("Negative Robot");

    // Visualize
    // Visualizer::makeFigure({obstacle, robot}, labels);
    // Visualizer::makeFigure(robot_objects);
    // Visualizer::makeFigure({c_space_obj});
    // Visualizer::makeFigure(c_space_objects, angles);
    // Visualizer::showFigures();


    ////////////////////////////////////////////////////////////////////////////
    // Problem 2
    ////////////////////////////////////////////////////////////////////////////


    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Create a manipulator with 3 links
    std::vector<double> link_lengths_a = {0.5, 1.0, 0.5};
    std::vector<double> link_lengths_b = {1.0, 0.5, 1.0};

    MyManipulator2D manipulator(link_lengths_a);

    // You can visualize your manipulator given an angle state like so:
    amp::ManipulatorState test_state(3);
    test_state << M_PI/6, M_PI/3, 7*M_PI/4;
    //test_state.setZero();

    // Get the location of the end effector
    Eigen::Vector2d end_effector = manipulator.getJointLocation(test_state, 3);
    std::cout << "End effector location: " << end_effector.transpose() << std::endl;

    // The visualizer uses your implementation of forward kinematics to show the joint positions so you can use that to test your FK algorithm
    Visualizer::makeFigure(manipulator, test_state); 
    Visualizer::showFigures();






    ////////////////////////////////////////////////////////////////////////////
    // Problem 3
    ////////////////////////////////////////////////////////////////////////////

    // Create the collision space constructor
    // std::size_t n_cells = 5;
    // MyManipulatorCSConstructor cspace_constructor(n_cells);

    // // Create the collision space using a given manipulator and environment
    // std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator, HW4::getEx3Workspace1());

    // // You can visualize your cspace 
    // Visualizer::makeFigure(*cspace);

    // Visualizer::showFigures();
    // // Grade method
    // amp::HW4::grade<MyManipulator2D>(cspace_constructor, "nonhuman.biologic@myspace.edu", argc, argv);

    return 0;
}