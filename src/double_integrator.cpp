#include "controlkit/double_integrator.hpp"

//how fast is state changing? 
Eigen::VectorXd ball_dynamics(const Eigen::VectorXd& state, const Eigen::VectorXd& control) {
    double velocity = state(1);
    double force = control(0);

    Eigen::VectorXd state_derivative(2);
    state_derivative << velocity, force; //in order
    return state_derivative;
}

Model make_double_integrator() {
    Model model;
    model.state_dim = 2;
    model.control_dim = 1;
    model.dynamics = ball_dynamics;
    return model;
}