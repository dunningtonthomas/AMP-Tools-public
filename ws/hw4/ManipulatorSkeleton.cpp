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

        // Account for quadrant errors
        if (end_effector_location.y() < 0) {
            joint_angles[0] = -joint_angles[0];
        }

        // Check for NaN values
        if (std::isnan(joint_angles[0]) || std::isnan(joint_angles[1]) || std::isnan(joint_angles[2])) {
            // Set the joint angles to zero
            joint_angles.setZero();
        }

        return joint_angles;
    } else if (nLinks() == 3) {

        // Loop through angles to the end effector location
        int num_angles = 20;
        double angle_step = 2*M_PI / num_angles;
        for(int i = 0; i < num_angles; i++) {
            // Orientation to the end effector
            double angle = i * angle_step;

            // Define location of the joint before the end effector
            Eigen::Vector2d p3 = end_effector_location - links[2] * Eigen::Vector2d(cos(angle), sin(angle));

            // Calculate first two angles for 2D arm case
            joint_angles[1] = acos(1 / (2*links[0]*links[1]) * (p3.squaredNorm() - (links[0]*links[0] + links[1]*links[1])));
            joint_angles[0] = acos(1 / (p3.squaredNorm()) * (p3.x() * (links[0] + links[1] * cos(joint_angles[1])) + p3.y() * links[1] * sqrt(1 - cos(joint_angles[1])*cos(joint_angles[1]))));

            // Account for quadrant errors
            if (p3.y() < 0) {
                joint_angles[0] = -joint_angles[0];
            }

            // Calculate the final joint angle 
            joint_angles[2] = angle - joint_angles[0] - joint_angles[1];

            // Use forward kinematics to check if the joint angles are correct
            Eigen::Vector2d end_effector = getJointLocation(joint_angles, 3);
            if ((end_effector - end_effector_location).norm() < 0.01) {
                //std::cout << "Found the correct joint angles!" << std::endl;
                return joint_angles;
            }
        }

        // Try new angle if none of the others worked
        // Orientation to the end effector
        double gamma = atan2(end_effector_location.y(), end_effector_location.x());

        // Get angle between 0 and 2pi
        if (gamma < 0) {
            gamma = gamma + 2*M_PI;
        }

        // Define location of the joint before the end effector
        Eigen::Vector2d p3 = end_effector_location - links[2] * Eigen::Vector2d(cos(gamma), sin(gamma));

        // Calculate first two angles for 2D arm case
        joint_angles[1] = acos(1 / (2*links[0]*links[1]) * (p3.squaredNorm() - (links[0]*links[0] + links[1]*links[1])));
        joint_angles[0] = acos(1 / (p3.squaredNorm()) * (p3.x() * (links[0] + links[1] * cos(joint_angles[1])) + p3.y() * links[1] * sqrt(1 - cos(joint_angles[1])*cos(joint_angles[1]))));

        // Account for quadrant errors
        if (p3.y() < 0) {
            joint_angles[0] = -joint_angles[0] + 2*M_PI;
        }

        // // Calculate the final joint angle 
        joint_angles[2] = gamma - joint_angles[0] - joint_angles[1];

        // Check for NaN values
        if (std::isnan(joint_angles[0]) || std::isnan(joint_angles[1]) || std::isnan(joint_angles[2])) {
            // Set the joint angles to zero
            joint_angles.setZero();
        }

        return joint_angles;
    } else {
        // I don know bro
        return joint_angles;
    }

    return joint_angles;
}