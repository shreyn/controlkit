#include <fstream>
#include <iostream>
#include "controlkit/double_integrator.hpp"
#include "controlkit/integrators.hpp"
#include "controlkit/linearize.hpp"
#include "controlkit/lqr.hpp"

int main() {
    Model model = make_double_integrator();
    double time_step = 0.1;

    Eigen::VectorXd goal_state(2);
    goal_state << 0.0, 0.0;
    Eigen::VectorXd goal_control(1);
    goal_control << 0.0;

    LinearModel linear_model = linearize(model, goal_state, goal_control, time_step);

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);

    Eigen::MatrixXd K = lqr(linear_model.A, linear_model.B, Q, R);

    std::cout << "A =\n" << linear_model.A << "\n";
    std::cout << "B =\n" << linear_model.B << "\n";
    std::cout << "K =\n" << K << "\n";

    std::ofstream csv_file("ball.csv");
    csv_file << "tick,position,velocity,push\n";

    Eigen::VectorXd state(2);
    state << 3.0, 0.0;

    for (int tick = 0; tick <= 100; tick++) {
        Eigen::VectorXd error = state - goal_state;
        Eigen::VectorXd control = goal_control - K * error;

        csv_file << tick << "," << state(0) << "," << state(1) << "," << control(0) << "\n";

        if (tick < 5 || tick % 10 == 0) {
            std::cout << "tick " << tick << "  position " << state(0) << "  velocity " << state(1) << "  push " << control(0) << "\n";
        }

        state = rk4_step(model, state, control, time_step);
    }

    return 0;
}