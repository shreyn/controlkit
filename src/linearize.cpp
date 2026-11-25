#include "controlkit/linearize.hpp"
#include "controlkit/integrators.hpp"

LinearModel linearize(const Model& model, const Eigen::VectorXd& state, const Eigen::VectorXd& control, double time_step) {
    double epsilon = 1e-6;

    LinearModel linear_model;
    linear_model.A.resize(model.state_dim, model.state_dim);
    linear_model.B.resize(model.state_dim, model.control_dim);

    //first A (state):
    for (int j =0; j < model.state_dim; j++) { //across cols
        Eigen::VectorXd state_plus = state;
        Eigen::VectorXd state_minus = state;
        state_plus(j) += epsilon;
        state_minus(j) -= epsilon;
        Eigen::VectorXd next_state_plus = rk4_step(model, state_plus, control, time_step);
        Eigen::VectorXd next_state_minus = rk4_step(model, state_minus, control, time_step);
        linear_model.A.col(j) = (next_state_plus-next_state_minus) / (2.0*epsilon); //central difference, then store vector as col j
    }

    // identical, for B (control):
    for (int j=0; j < model.control_dim; j++) {
        Eigen::VectorXd control_plus = control;
        Eigen::VectorXd control_minus = control;
        control_plus(j) += epsilon;
        control_minus(j) -= epsilon;
        Eigen::VectorXd next_state_plus = rk4_step(model, state, control_plus, time_step);
        Eigen::VectorXd next_state_minus = rk4_step(model, state, control_minus, time_step);
        linear_model.B.col(j) = (next_state_plus-next_state_minus) / (2.0 *epsilon);
    }

    return linear_model; //both matrices
}