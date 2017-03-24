#pragma once

#include <vector>

#include "Eigen/Dense"

#include "measurement_package.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UnscentedKalmanFilter;

class SigmapointManager
{
  public:
    /// @brief Construct with given ukf reference
    SigmapointManager(UnscentedKalmanFilter& ukf);

    /// @brief Generates a set of sigma points for an augmented state.
    void GenerateAugmentedSigmaPoints(MatrixXd& Xsig_out);

    /// @brief Sigma points gets predicted to a new state.
    void PredictSigmaPoints(const MatrixXd& Xsig_aug, double delta_t);

  private:
    /// @brief Reference to the Kalman filter
    UnscentedKalmanFilter& ukf_;

    /// @brief Sigma point spreading parameter
    double lambda_;

    /// @brief Shall make debug outputs
    bool debug_;
};
