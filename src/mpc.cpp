#include "controlkit/mpc.hpp"
#include <iostream>
#include <string>

static double shift_bound(double bound, double shift) {
    if (bound > 1e29 || bound < -1e29) {
        return bound;
    }
    return bound - shift;
}

static void to_csc(const Eigen::MatrixXd& dense, bool upper_triangle_only, std::vector<OSQPFloat>& values, std::vector<OSQPInt>& row_indices, std::vector<OSQPInt>& column_starts) {
    column_starts.push_back(0);
    for (int column = 0; column < dense.cols(); column++) {
        for (int row = 0; row < dense.rows(); row++) {
            if (upper_triangle_only && row > column) {
                continue;
            }
            if (dense(row, column) == 0.0) {
                continue;
            }
            values.push_back(dense(row, column));
            row_indices.push_back(row);
        }
        column_starts.push_back(values.size());
    }
}

Mpc make_mpc(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& terminal_cost, int horizon, const Eigen::VectorXd& goal_state, const Eigen::VectorXd& goal_control, const Eigen::VectorXd& control_min, const Eigen::VectorXd& control_max, const Eigen::VectorXd& state_min, const Eigen::VectorXd& state_max) {
    static_assert(sizeof(OSQPFloat) == sizeof(double), "OSQP must be built with double precision");

    Mpc mpc;
    mpc.state_dim = A.rows();
    mpc.control_dim = B.cols();
    mpc.horizon = horizon;
    mpc.goal_state = goal_state;
    mpc.goal_control = goal_control;
    mpc.solver = nullptr;

    int state_dim = mpc.state_dim;
    int control_dim = mpc.control_dim;
    int num_push_rows = horizon * control_dim;
    int num_state_rows = horizon * state_dim;
    int num_constraints = num_push_rows + num_state_rows;

    // powers of A: A^0, A^1, ..., A^horizon
    std::vector<Eigen::MatrixXd> A_powers(horizon + 1);
    A_powers[0] = Eigen::MatrixXd::Identity(state_dim, state_dim);
    for (int k = 1; k <= horizon; k++) {
        A_powers[k] = A * A_powers[k - 1];
    }

    // prediction: X = prediction_from_state * state_error + prediction_from_pushes * U
    mpc.prediction_from_state = Eigen::MatrixXd::Zero(num_state_rows, state_dim);
    Eigen::MatrixXd prediction_from_pushes = Eigen::MatrixXd::Zero(num_state_rows, num_push_rows);
    for (int i = 0; i < horizon; i++) {
        mpc.prediction_from_state.block(i * state_dim, 0, state_dim, state_dim) = A_powers[i + 1];
        for (int j = 0; j <= i; j++) {
            prediction_from_pushes.block(i * state_dim, j * control_dim, state_dim, control_dim) = A_powers[i - j] * B;
        }
    }

    // stacked costs: Q for each error, the terminal cost for the last error, R for each push
    Eigen::MatrixXd Q_stacked = Eigen::MatrixXd::Zero(num_state_rows, num_state_rows);
    for (int i = 0; i < horizon - 1; i++) {
        Q_stacked.block(i * state_dim, i * state_dim, state_dim, state_dim) = Q;
    }
    Q_stacked.block((horizon - 1) * state_dim, (horizon - 1) * state_dim, state_dim, state_dim) = terminal_cost;
    Eigen::MatrixXd R_stacked = Eigen::MatrixXd::Zero(num_push_rows, num_push_rows);
    for (int i = 0; i < horizon; i++) {
        R_stacked.block(i * control_dim, i * control_dim, control_dim, control_dim) = R;
    }

    // cost = 0.5 * U' * hessian * U + gradient' * U, where gradient = gradient_from_state * state_error
    Eigen::MatrixXd hessian = 2.0 * (prediction_from_pushes.transpose() * Q_stacked * prediction_from_pushes + R_stacked);
    mpc.gradient_from_state = 2.0 * prediction_from_pushes.transpose() * Q_stacked * mpc.prediction_from_state;

    // limits, written in terms of errors from the goal
    mpc.push_lower.resize(num_push_rows);
    mpc.push_upper.resize(num_push_rows);
    mpc.state_lower.resize(num_state_rows);
    mpc.state_upper.resize(num_state_rows);
    for (int k = 0; k < horizon; k++) {
        for (int i = 0; i < control_dim; i++) {
            mpc.push_lower(k * control_dim + i) = shift_bound(control_min(i), goal_control(i));
            mpc.push_upper(k * control_dim + i) = shift_bound(control_max(i), goal_control(i));
        }
        for (int i = 0; i < state_dim; i++) {
            mpc.state_lower(k * state_dim + i) = shift_bound(state_min(i), goal_state(i));
            mpc.state_upper(k * state_dim + i) = shift_bound(state_max(i), goal_state(i));
        }
    }

    // constraint matrix: first rows are the pushes themselves, then the predicted errors
    Eigen::MatrixXd constraint_matrix(num_constraints, num_push_rows);
    constraint_matrix.topRows(num_push_rows) = Eigen::MatrixXd::Identity(num_push_rows, num_push_rows);
    constraint_matrix.bottomRows(num_state_rows) = prediction_from_pushes;

    // convert to OSQP's sparse format (the arrays stay stored inside mpc)
    to_csc(hessian, true, mpc.hessian_values, mpc.hessian_rows, mpc.hessian_column_starts);
    to_csc(constraint_matrix, false, mpc.constraint_values, mpc.constraint_rows, mpc.constraint_column_starts);

    Eigen::VectorXd initial_gradient = Eigen::VectorXd::Zero(num_push_rows);
    Eigen::VectorXd initial_lower(num_constraints);
    Eigen::VectorXd initial_upper(num_constraints);
    initial_lower.head(num_push_rows) = mpc.push_lower;
    initial_lower.tail(num_state_rows) = mpc.state_lower;
    initial_upper.head(num_push_rows) = mpc.push_upper;
    initial_upper.tail(num_state_rows) = mpc.state_upper;

    OSQPCscMatrix* hessian_csc = OSQPCscMatrix_new(num_push_rows, num_push_rows, mpc.hessian_values.size(), mpc.hessian_values.data(), mpc.hessian_rows.data(), mpc.hessian_column_starts.data());
    OSQPCscMatrix* constraint_csc = OSQPCscMatrix_new(num_constraints, num_push_rows, mpc.constraint_values.size(), mpc.constraint_values.data(), mpc.constraint_rows.data(), mpc.constraint_column_starts.data());
    OSQPSettings* settings = OSQPSettings_new();
    settings->verbose = 0;
    settings->eps_abs = 1e-6;
    settings->eps_rel = 1e-6;

    OSQPInt exit_flag = osqp_setup(&mpc.solver, hessian_csc, initial_gradient.data(), constraint_csc, initial_lower.data(), initial_upper.data(), num_constraints, num_push_rows, settings);
    if (exit_flag != 0) {
        std::cerr << "OSQP setup failed, code " << exit_flag << "\n";
    }

    OSQPCscMatrix_free(hessian_csc);
    OSQPCscMatrix_free(constraint_csc);
    OSQPSettings_free(settings);

    return mpc;
}

Eigen::VectorXd mpc_control(Mpc& mpc, const Eigen::VectorXd& state) {
    Eigen::VectorXd state_error = state - mpc.goal_state;

    Eigen::VectorXd gradient = mpc.gradient_from_state * state_error;
    Eigen::VectorXd free_motion = mpc.prediction_from_state * state_error;

    int num_push_rows = mpc.horizon * mpc.control_dim;
    int num_state_rows = mpc.horizon * mpc.state_dim;
    Eigen::VectorXd lower(num_push_rows + num_state_rows);
    Eigen::VectorXd upper(num_push_rows + num_state_rows);
    lower.head(num_push_rows) = mpc.push_lower;
    upper.head(num_push_rows) = mpc.push_upper;
    for (int k = 0; k < num_state_rows; k++) {
        lower(num_push_rows + k) = shift_bound(mpc.state_lower(k), free_motion(k));
        upper(num_push_rows + k) = shift_bound(mpc.state_upper(k), free_motion(k));
    }

    osqp_update_data_vec(mpc.solver, gradient.data(), lower.data(), upper.data());
    osqp_solve(mpc.solver);

    std::string status = mpc.solver->info->status;
    if (status != "solved" && status != "solved inaccurate") {
        std::cerr << "OSQP status: " << status << " (applying the goal control)\n";
        return mpc.goal_control;
    }

    Eigen::VectorXd control = mpc.goal_control;
    for (int i = 0; i < mpc.control_dim; i++) {
        control(i) += mpc.solver->solution->x[i];
    }
    return control;
}

void destroy_mpc(Mpc& mpc) {
    osqp_cleanup(mpc.solver);
    mpc.solver = nullptr;
}