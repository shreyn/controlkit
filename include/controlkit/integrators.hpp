#pragma once

#include "controlkit/model.hpp"

Eigen::VectorXd euler_step(cost Model& model, const Eigen::VectorXd& state, const Eigen::VectorXd& control, double time_step);

Eigen::VectorXd rk4_step(cost Model& model, const Eigen::VectorXd& state, const Eigen::VectorXd& control, double time_step);
