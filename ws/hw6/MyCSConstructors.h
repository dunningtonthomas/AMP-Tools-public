#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW4.h"
#include "hw/HW6.h"

// Include the Cspace constructor from HW4
#include "CSpaceSkeleton.h"

// Include the helper functions
#include "helperFuncs.h"

// Derive the PointAgentCSConstructor class and override the missing method
class MyPointAgentCSConstructor : public amp::PointAgentCSConstructor {
    public:
        // To make things easy, add the number of cells as a ctor param so you can easily play around with it
        MyPointAgentCSConstructor(std::size_t cells_per_dim) : m_cells_per_dim(cells_per_dim) {}

        // Override this method for computing all of the boolean collision values for each cell in the cspace
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::Environment2D& env) override;

        // Function to check if a point is in collision
        bool inCollision_point(const amp::Environment2D& env, const Eigen::Vector2d& point);

    private:
        std::size_t m_cells_per_dim;
};

class MyWaveFrontAlgorithm : public amp::WaveFrontAlgorithm {
    public:
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) override;

};

