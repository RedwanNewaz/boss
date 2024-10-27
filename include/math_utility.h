#pragma once 

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cmath>

using namespace Eigen;



// Function to compute trajectory using sparse matrices
inline MatrixXd computeTrajectory(VectorXd state, const VectorXd& control, double DT, double T) {

    // Define constants
    const int num_steps = static_cast<int>(T / DT);  // Number of steps
   // Define the state transition matrix F for [x, y, theta, v, omega]
    state(2) += control(1) * DT;
    state(2) = fmod(state(2) + M_PI, 2 * M_PI) - M_PI;
    Eigen::MatrixXd F(5, 5);
    F << 1.0, 0, 0, DT * std::cos(state(2)), 0,
        0, 1.0, 0, DT * std::sin(state(2)), 0,
        0, 0, 1.0, 0, DT,
        0, 0, 0, 1.0, 0,
        0, 0, 0, 0, 1.0;

    // Define the control input matrix B
    Eigen::MatrixXd B(5, 2);
    B << 0, 0,
        0, 0,
        0, DT,
        DT, 0,
        0, DT;

    // Initialize trajectory matrix
    MatrixXd trajectory(num_steps, state.size());
    trajectory.row(0) = state.transpose();

    // Compute trajectory over time steps
    for (int i = 1; i < num_steps; ++i) {
        state = F * state + B * control;
        state(2) = fmod(state(2) + M_PI, 2 * M_PI) - M_PI;
        trajectory.row(i) = state.transpose();
    }

    return trajectory;
}


/* usage 

    Traj traj; 
    Eigen::Map<Eigen::VectorXd> state(x.data(), x.size());
    Eigen::Vector2d control;
    control << v, y;
    Eigen::MatrixXd matrix = computeTrajectory(state, control, config.dt, config.predict_time);

    // Iterate over each row of the Eigen matrix
    for (int i = 0; i < matrix.rows(); ++i) {
        std::array<double, 5> rowArray;
        for (int j = 0; j < matrix.cols() && distance >  config.goal_radius; ++j) {
            rowArray[j] = matrix(i, j);
        }
        double dx = goal[0] - rowArray[0];
        double dy = goal[1] - rowArray[1];
        distance = sqrt(dx * dx + dy * dy);
        traj.push_back(rowArray);
    }

*/
   