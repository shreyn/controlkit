#include "controlkit/lqr.hpp"

Eigen::MatrixXd solve_riccati(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    int state_dim = A.rows();

    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(state_dim, state_dim);

    for (int iteration = 0; iteration < 10000; iteration++) {
        Eigen::MatrixXd S = R + B.transpose() * P * B;
        Eigen::MatrixXd G = B.transpose() * P * A;
        Eigen::MatrixXd K = S.ldlt().solve(G);

        Eigen::MatrixXd new_P = Q + A.transpose() * P * (A - B * K);

        double change = (new_P - P).norm();
        P = new_P;
        if (change < 1e-10) {
            break;
        }
    }

    return P;
}

Eigen::MatrixXd lqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    Eigen::MatrixXd P = solve_riccati(A, B, Q, R);

    Eigen::MatrixXd S = R + B.transpose() * P * B;
    Eigen::MatrixXd G = B.transpose() * P * A;
    Eigen::MatrixXd K = S.ldlt().solve(G);
    return K;
}