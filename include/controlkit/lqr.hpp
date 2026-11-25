#pragma once

#include <Eigen/Dense>

//Input: A, B, Q, R
// returns: K (control_dim x state_dim)
Eigen::MatrixXd lqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);