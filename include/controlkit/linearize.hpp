#pragma once

#include <Eigen/Dense>
#include "controlkit/model.hpp"

struct LinearModel {
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
};

// Input: a model, a point (x,u) to straigten around, and the time step.
// returns: A, B for that point
LinearModel linearize(const Model& model, const Eigen::VectorXd& state, const Eigen::VectorXd& control, double time_step);