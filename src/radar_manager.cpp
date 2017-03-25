#include <iostream>

#include "radar_manager.h"
#include "unscented_kalman_filter.h"

#include "tools.h"

RadarManager::RadarManager(UnscentedKalmanFilter& ukf, bool use_debug_outputs_)
        : ukf_(ukf), use_debug_outputs_(use_debug_outputs_)
{
    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;
    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.0175;
    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.1;
}

void RadarManager::Update(MeasurementPackage meas_package)
{
    // Use radar data to update the belief about the object's
    // position. Modify the state vector, x_, and covariance, P_.
    // You'll also need to calculate the radar NIS.

    auto n_z = 3;
    VectorXd z_pred = VectorXd(n_z);
    MatrixXd S = MatrixXd(n_z, n_z);
    MatrixXd Zsig = MatrixXd(n_z, 2 * ukf_.GetAugmentedStateSize() + 1);

    PredictMeasurement(n_z, z_pred, S, Zsig);
    ukf_.UpdateState(n_z, z_pred, S, Zsig, meas_package.raw_measurements_);
}

void RadarManager::PredictMeasurement(int n_z, VectorXd& z_pred, MatrixXd& S, MatrixXd& Zsig)
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
        double v = predicted_sigma_points(2, i);
        double yaw = predicted_sigma_points(3, i);

        double v1 = cos(yaw) * v;
        double v2 = sin(yaw) * v;

        // measurement model
        Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);                          // r
        Zsig(1, i) = atan2(p_y, p_x);                                      // phi
        Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y);  // r_dot
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
    R << std_radr_ * std_radr_, 0, 0, 0, std_radphi_ * std_radphi_, 0, 0, 0, std_radrd_ * std_radrd_;
    S = S + R;

    // print result
    if (use_debug_outputs_)
    {
        std::cout << "z_pred: " << std::endl << z_pred << std::endl;
        std::cout << "S: " << std::endl << S << std::endl;
    }
}