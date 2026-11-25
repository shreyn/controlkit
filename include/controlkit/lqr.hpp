#pragma once

#include <Eigen/Dense>

Eigen::MatrixXd solve_riccati(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);

Eigen::MatrixXd lqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);