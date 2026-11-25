#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include "controlkit/double_integrator.hpp"
#include "controlkit/integrators.hpp"
#include "controlkit/linearize.hpp"
#include "controlkit/lqr.hpp"
#include "controlkit/mpc.hpp"

void run_case(const std::string& case_name, const std::string& csv_name, double speed_limit) {
    Model model = make_double_integrator();
    double time_step = 0.1;

    Eigen::VectorXd goal_state(2);
    goal_state << 0.0, 0.0;
    Eigen::VectorXd goal_control(1);
    goal_control << 0.0;

    LinearModel linear_model = linearize(model, goal_state, goal_control, time_step);

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);
    Eigen::MatrixXd terminal_cost = solve_riccati(linear_model.A, linear_model.B, Q, R);

    Eigen::VectorXd control_min(1);
    control_min << -0.5;
    Eigen::VectorXd control_max(1);
    control_max << 0.5;
    Eigen::VectorXd state_min(2);
    state_min << -no_limit, -speed_limit;
    Eigen::VectorXd state_max(2);
    state_max << no_limit, speed_limit;

    int horizon = 20;
    Mpc mpc = make_mpc(linear_model.A, linear_model.B, Q, R, terminal_cost, horizon, goal_state, goal_control, control_min, control_max, state_min, state_max);

    std::ofstream csv_file(csv_name);
    csv_file << "tick,position,velocity,push\n";

    Eigen::VectorXd state(2);
    state << 10.0, 0.0;

    int settled_tick = -1;
    double min_position = state(0);
    double max_speed = 0.0;

    std::cout << "=== " << case_name << " ===\n";
    for (int tick = 0; tick < 300; tick++) {
        Eigen::VectorXd control = mpc_control(mpc, state);

        csv_file << tick << "," << state(0) << "," << state(1) << "," << control(0) << "\n";
        if (tick < 3 || tick % 30 == 0) {
            std::cout << "tick " << tick << "  position " << state(0) << "  velocity " << state(1) << "  push " << control(0) << "\n";
        }

        if (settled_tick < 0 && std::sqrt(state(0) * state(0) + state(1) * state(1)) < 0.05) {
            settled_tick = tick;
        }
        if (state(0) < min_position) {
            min_position = state(0);
        }
        if (std::abs(state(1)) > max_speed) {
            max_speed = std::abs(state(1));
        }

        state = rk4_step(model, state, control, time_step);
    }

    std::cout << "settled at tick " << settled_tick << ", lowest position " << min_position << ", top speed " << max_speed << "\n\n";

    destroy_mpc(mpc);
}

int main() {
    run_case("push limit 0.5", "ball_mpc_push_only.csv", no_limit);
    run_case("push limit 0.5 and speed limit 1.0", "ball_mpc_speed.csv", 1.0);

    int plot_status = std::system("python python_plotting/animate_ball_mpc.py");
    if (plot_status != 0) {
        std::cout << "plotting failed: run from the controlkit folder, with python, numpy and matplotlib available\n";
    }

    return 0;
}