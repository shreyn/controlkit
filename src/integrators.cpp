#include "controlkit/integrators.hpp"

Eigen::VectorXd euler_step(const Model& model, const Eigen::VectorXd& state, const Eigen::VectorXd& control, double time_step) {
    Eigen::VectorXd state_derivative = model.dynamics(state, control);
    Eigen::VectorXd next_state = state + time_step*state_derivative;
    return next_state;
}

Eigen::VectorXd rk4_step(const Model& model, const Eigen::VectorXd& state, const Eigen::VectorXd& control, double time_step) {
    Eigen::VectorXd slope_1 = model.dynamics(state, control);
    Eigen::VectorXd slope_2 = model.dynamics(state + 0.5*time_step*slope_1, control);
    Eigen::VectorXd slope_3 = model.dynamics(state + 0.5*time_step*slope_2, control);
    Eigen::VectorXd slope_4 = model.dynamics(state + time_step*slope_3, control);

    Eigen::VectorXd average_slope = (slope_1 + 2.0*slope_2 + 2.0*slope_3 + slope_4) / 6.0;
    Eigen::VectorXd next_state = state + time_step*average_slope;
    return next_state;
}