#include <iostream>

#include "laser_manager.h"
#include "tools.h"
#include "unscented_kalman_filter.h"

LaserManager::LaserManager(UnscentedKalmanFilter& ukf) : ukf_(ukf), debug_(false)
{
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.10;
    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.10;
}

void LaserManager::Update(MeasurementPackage meas_package)
{
    // Use lidar data to update the belief about the object's
    // position. Modify the state vector, x_, and covariance, P_.
    // You'll also need to calculate the lidar NIS.

    auto n_z = 2;
    VectorXd z_pred = VectorXd(n_z);
    MatrixXd S = MatrixXd(n_z, n_z);
    MatrixXd Zsig = MatrixXd(n_z, 2 * ukf_.GetAugmentedStateSize() + 1);

    PredictMeasurement(n_z, z_pred, S, Zsig);
    ukf_.UpdateState(n_z, z_pred, S, Zsig, meas_package.raw_measurements_);
}

void LaserManager::PredictMeasurement(int n_z, VectorXd& z_pred, MatrixXd& S, MatrixXd& Zsig)
{
    auto augement_state_size = ukf_.GetAugmentedStateSize();
    auto weights = ukf_.GetWeights();
    auto predicted_sigma_points = ukf_.GetPredictedSigmaPoints();

    // transform sigma points into measurement space
    for (int i = 0; i < 2 * augement_state_size + 1; i++)
    {  // 2n+1 sigma points

        // extract values for better readability
        double p_x = predicted_sigma_points(0, i);
        double p_y = predicted_sigma_points(1, i);

        // measurement model
        Zsig(0, i) = p_x;  // x
        Zsig(1, i) = p_y;  // y
    }

    // mean predicted measurement
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * augement_state_size + 1; i++)
    {
        z_pred = z_pred + weights(i) * Zsig.col(i);
    }

    // measurement covariance matrix S
    S.fill(0.0);
    for (int i = 0; i < 2 * augement_state_size + 1; i++)
    {  // 2n+1 sigma points
       // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        Tools::NormAngle(z_diff(1));

        S = S + weights(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;
    S = S + R;

    // print result
    if (debug_)
    {
        std::cout << "z_pred: " << std::endl << z_pred << std::endl;
        std::cout << "S: " << std::endl << S << std::endl;
    }
}
