#pragma once

#include <vector>
#include "Eigen/Dense"

#include "measurement_package.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class ISensorMeasurement
{
  public:
    /// Updates the state and the state covariance matrix using a sensor measurement
    /// @param meas_package The measurement at k+1
    virtual void Update(MeasurementPackage meas_package) = 0;

    /// @brief A sensor measurement is predicted to a new state
    virtual void PredictMeasurement(const int n_z, VectorXd& z_pred, MatrixXd& S, MatrixXd& Zsig) = 0;
};