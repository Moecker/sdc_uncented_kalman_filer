#pragma once

#include <vector>
#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;

/// @brief Tool class used to compute Jacobian and RMSE
class Tools
{
  public:
    /// @brief A helper method to calculate RMSE.
    static VectorXd CalculateRMSE(const std::vector<VectorXd>& estimations, const std::vector<VectorXd>& ground_truth);
};
