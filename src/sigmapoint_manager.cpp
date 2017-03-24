#include <iostream>

#include "sigmapoint_manager.h"
#include "unscented_kalman_filter.h"

SigmapointManager::SigmapointManager(UnscentedKalmanFilter& ukf) : ukf_(ukf), debug_(false)
{
}

void SigmapointManager::GenerateAugmentedSigmaPoints(MatrixXd& Xsig_aug)
{
    // create augmented mean vector
    VectorXd x_aug = VectorXd(7);

    // create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);

    // create augmented mean state
    x_aug.head(5) = ukf_.GetState();
    x_aug(5) = 0;
    x_aug(6) = 0;

    // create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5, 5) = ukf_.GetCovariance();
    P_aug(5, 5) = ukf_.GetStdDevAcceleration() * ukf_.GetStdDevAcceleration();
    P_aug(6, 6) = ukf_.GetStdDevYawAcceleration() * ukf_.GetStdDevYawAcceleration();

    // create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    // create augmented sigma points
    Xsig_aug.col(0) = x_aug;
    auto augmented_state_size = ukf_.GetAugmentedStateSize();
    for (int i = 0; i < augmented_state_size; i++)
    {
        Xsig_aug.col(i + 1) = x_aug + sqrt(ukf_.GetLambda() + augmented_state_size) * L.col(i);
        Xsig_aug.col(i + 1 + augmented_state_size) = x_aug - sqrt(ukf_.GetLambda() + augmented_state_size) * L.col(i);
    }

    // print result
    if (debug_)
    {
        std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;
    }
}

void SigmapointManager::PredictSigmaPoints(const MatrixXd& Xsig_aug, double delta_t)
{
    auto& Xsig_pred = ukf_.GetPredictedSigmaPoints();

    // predict sigma points
    for (int i = 0; i < 2 * ukf_.GetAugmentedStateSize() + 1; i++)
    {
        // extract values for better readability
        double p_x = Xsig_aug(0, i);
        double p_y = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        // predicted state values
        double px_p, py_p;

        // avoid division by zero
        if (fabs(yawd) > 0.001)
        {
            px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
            py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
        }
        else
        {
            px_p = p_x + v * delta_t * cos(yaw);
            py_p = p_y + v * delta_t * sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd * delta_t;
        double yawd_p = yawd;

        // add noise
        px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
        py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
        v_p = v_p + nu_a * delta_t;

        yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
        yawd_p = yawd_p + nu_yawdd * delta_t;

        // write predicted sigma point into right column
        Xsig_pred(0, i) = px_p;
        Xsig_pred(1, i) = py_p;
        Xsig_pred(2, i) = v_p;
        Xsig_pred(3, i) = yaw_p;
        Xsig_pred(4, i) = yawd_p;
    }

    // print result
    if (debug_)
    {
        std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;
    }
}
