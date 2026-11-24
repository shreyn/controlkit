#pragma once

#include <functional>
#include <Eigen/Dense>


// dynamics(state, control). Returns rate of change of state
using DynamicsFunction = std::function<Eigen::VectorXd(const Eigen::VectorXd& state, const Eigen::VectorXd& control)>;

struct Model {
    int state_dim;
    int control_dim;
    DynamicsFunction dynamics;
};


