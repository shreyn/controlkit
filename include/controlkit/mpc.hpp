#pragma once

#include <vector>
#include <Eigen/Dense>
#include <osqp.h>

const double no_limit = 1e30;

struct Mpc {
    int state_dim;
    int control_dim;
    int horizon;
    Eigen::VectorXd goal_state;
    Eigen::VectorXd goal_control;
    Eigen::MatrixXd prediction_from_state;
    Eigen::MatrixXd gradient_from_state;
    Eigen::VectorXd push_lower;
    Eigen::VectorXd push_upper;
    Eigen::VectorXd state_lower;
    Eigen::VectorXd state_upper;
    std::vector<OSQPFloat> hessian_values;
    std::vector<OSQPInt> hessian_rows;
    std::vector<OSQPInt> hessian_column_starts;
    std::vector<OSQPFloat> constraint_values;
    std::vector<OSQPInt> constraint_rows;
    std::vector<OSQPInt> constraint_column_starts;
    OSQPSolver* solver;
};

//builds the controller once. Use no_limit / -no_limit for "no limit on this entry"
Mpc make_mpc(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& terminal_cost, int horizon, const Eigen::VectorXd& goal_state, const Eigen::VectorXd& goal_control, const Eigen::VectorXd& control_min, const Eigen::VectorXd& control_max, const Eigen::VectorXd& state_min, const Eigen::VectorXd& state_max);

//one tick: takes current state, returns control to apply
Eigen::VectorXd mpc_control(Mpc& mpc, const Eigen::VectorXd& state);

//frees the OSQP solver
void destroy_mpc(Mpc& mpc);