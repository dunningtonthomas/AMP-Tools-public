#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"


class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// Consider defining a class constructor to easily tune parameters, for example: 
		MyGDAlgorithm(double d_star, double zetta, double Q_star, double eta) :
			d_star(d_star),
			zetta(zetta),
			Q_star(Q_star),
			eta(eta) {}

		// Override this method to solve a given problem.
		virtual amp::Path2D plan(const amp::Problem2D& problem) override;
	private:
		double d_star, zetta, Q_star, eta;
		// Add additional member variables here...
};

class MyPotentialFunction : public amp::PotentialFunction2D {
    public:
	    // Create a constructor that takes in the parameters for the potential function
		MyPotentialFunction(amp::Problem2D problem, double d_star, double zetta, double Q_star, double eta) :
		problem(problem),
		d_star(d_star),
		zetta(zetta),
		Q_star(Q_star),
		eta(eta) {}

		// Create a default constructor
		MyPotentialFunction() = default;


		// Returns the potential function value (height) for a given 2D point. 
        virtual double operator()(const Eigen::Vector2d& q) const override;

		// Distance to goal function
		double distanceBetween(const Eigen::Vector2d& q1, const Eigen::Vector2d& q2) const;

		// Distance to obstacle function
		double distanceToObstacle(const Eigen::Vector2d& q, const amp::Obstacle2D& obstacle) const;

		// Distance from point to a line segment
		double minDistance(const Eigen::Vector2d A, const Eigen::Vector2d B, const Eigen::Vector2d E) const;

	private:
		amp::Problem2D problem;
		double d_star, zetta, Q_star, eta;
};