#include "ManipulatorSkeleton.h"


MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1.0, 1.0}) // Default to a 2-link with all links of 1.0 length
{}

// Override this method for implementing forward kinematics
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    // Implement forward kinematics to calculate the joint position given the manipulator state (angles)
    // m_link_lengths contains the lengths of each link getLinkLengths()
    std::vector<Eigen::Vector2d> joint_positions;
    std::vector<Eigen::Matrix3d> rot_mats;
    std::vector<double> links = getLinkLengths();

    // Add the base location
    joint_positions.push_back(Eigen::Vector2d(0.0, 0.0));

    double rotation_angle = 0.0;
    Eigen::Vector2d joint_position(0.0, 0.0);
    for(int i = 0; i < state.size(); i++) {
        // Increment the rotation angle
        rotation_angle = rotation_angle + state[i];

        // Update the joint position
        joint_position = joint_position + Eigen::Vector2d(links[i] * cos(rotation_angle), links[i] * sin(rotation_angle));
        
        // Push onto joint position vector
        joint_positions.push_back(joint_position);
    }

    //std::vector<Eigen::Vector2d> joint_positions = {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(1.0, 1.0)};
    return joint_positions[joint_index];
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here
    amp::ManipulatorState joint_angles(nLinks());
    std::vector<double> links = getLinkLengths();
    joint_angles.setZero();
    
    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    if (nLinks() == 2) {
        // Orientation to the end effector
        double alpha = acos((-1*end_effector_location.squaredNorm() + links[0]*links[0] + links[1]*links[1]) / (2 * links[0] * links[1]));
        double beta = asin(links[1] * sin(alpha) / end_effector_location.norm());

        // Calculate the joint angles theta1 and theta2
        joint_angles[0] = atan2(end_effector_location.y(), end_effector_location.x()) - beta;
        joint_angles[1] = M_PI - alpha;

        return joint_angles;
    } else if (nLinks() == 3) {
        // Orientation to the end effector
        double gamma = atan2(end_effector_location.y(), end_effector_location.x());

        // Define location of the joint before the end effector
        Eigen::Vector2d p3 = end_effector_location - links[2] * Eigen::Vector2d(cos(gamma), sin(gamma));

        // Calculate the angle of the joint before the end effector
        double alpha = acos((-1*p3.squaredNorm() + links[0]*links[0] + links[1]*links[1]) / (2 * links[0] * links[1]));
        double beta = asin(links[1] * sin(alpha) / p3.norm());

        // Calculate the joint angles theta1 and theta2
        joint_angles[0] = atan2(p3.y(), p3.x()) - beta;
        joint_angles[1] = M_PI - alpha;

        // Calculate the final joint angle 
        joint_angles[2] = gamma - joint_angles[0] - joint_angles[1];
        
        return joint_angles;
    } else {
        // I don know bro
        return joint_angles;
    }

    return joint_angles;
}