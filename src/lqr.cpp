#include "controlkit/lqr.hpp"

Eigen::MatrixXd lqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    int state_dim = A.rows();
    int control_dim = B.cols();

    //P is initialized to all zeros:
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(state_dim, state_dim); 

    for (int iter=0; iter < 1000; iter++) {
        Eigen::MatrixXd S = R + B.transpose() * P * B;
        Eigen::MatrixXd G = B.transpose() * P * A;
        Eigen::MatrixXd K = S.ldlt().solve(G); //solves SK = G => K = S^-1G

        Eigen::MatrixXd new_P = Q + A.transpose() * P * (A - B*K);

        double change = (new_P - P).norm();
        P = new_P; //update before break, so newest val is used below
        if (change < 1e-10) { //stop when P stops changing
            break;
        }     
    }

    //now recompute K with last updated P
    Eigen::MatrixXd S = R + B.transpose() * P * B;
    Eigen::MatrixXd G = B.transpose() * P * A;
    Eigen::MatrixXd K = S.ldlt().solve(G);
    return K;
}