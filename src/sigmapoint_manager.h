#pragma once

#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UnscentedKalmanFilter;

class SigmapointManager
{
  public:
    /// @brief Construct with given ukf reference
    SigmapointManager(UnscentedKalmanFilter& ukf, bool use_debug_outputs);

    /// @brief Generates a set of sigma points for an augmented state.
    void GenerateAugmentedSigmaPoints(MatrixXd& Xsig_out);

    /// @brief Sigma points gets predicted to a new state.
    void PredictSigmaPoints(const MatrixXd& Xsig_aug, double delta_t);

  private:
    /// @brief Reference to the Kalman filter
    UnscentedKalmanFilter& ukf_;

    /// @brief Shall make debug outputs
    bool use_debug_outputs_;
};
