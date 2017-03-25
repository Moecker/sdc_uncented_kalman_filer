#pragma once

#include "isensor_measurement.h"

class UnscentedKalmanFilter;

class RadarManager : public ISensorMeasurement
{
  public:
    /// @brief Construct with given ukf reference
    RadarManager(UnscentedKalmanFilter& ukf, bool use_debug_outputs_);

    /// @copydoc ISensorMeasurement::Update
    void Update(MeasurementPackage meas_package);

    /// @copydoc ISensorMeasurement::PredictMeasurement
    void PredictMeasurement(const int n_z, VectorXd& z_pred, MatrixXd& S, MatrixXd& Zsig);

  private:
    /// @brief Reference to the Kalman filter
    UnscentedKalmanFilter& ukf_;

    /// Radar measurement noise standard deviation radius in m
    double std_radr_;
    /// Radar measurement noise standard deviation angle in rad
    double std_radphi_;
    /// Radar measurement noise standard deviation radius change in m/s
    double std_radrd_;

    /// @brief Shall make debug outputs
    bool use_debug_outputs_;
};
