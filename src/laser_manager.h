#pragma once

#include "isensor_measurement.h"

class UnscentedKalmanFilter;

class LaserManager : public ISensorMeasurement
{
  public:
    /// @brief Construct with given ukf reference
    LaserManager(UnscentedKalmanFilter& ukf);

    /// @copydoc ISensorMeasurement::Update
    void Update(MeasurementPackage meas_package);

    /// @copydoc ISensorMeasurement::PredictMeasurement
    void PredictMeasurement(const int n_z, VectorXd& z_pred, MatrixXd& S, MatrixXd& Zsig);

  private:
    /// @brief Reference to the Kalman filter
    UnscentedKalmanFilter& ukf_;

    /// Laser measurement noise standard deviation position1 in m
    double std_laspx_;
    /// Laser measurement noise standard deviation position2 in m
    double std_laspy_;

    /// @brief Shall make debug outputs
    bool debug_;
};
