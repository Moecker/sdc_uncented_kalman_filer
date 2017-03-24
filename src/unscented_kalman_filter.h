#pragma once

#include <vector>

#include "Eigen/Dense"

#include "measurement_package.h"
#include "sigmapoint_manager.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UnscentedKalmanFilter
{
  public:
    /// Shall make debug outputs
    bool debug;
    ///* State dimension
    int n_x_;
    ///* Augmented state dimension
    int n_aug_;
    ///* Sigma point spreading parameter
    double lambda_;

    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd x_;
    ///* state covariance matrix
    MatrixXd P_;
    ///* predicted sigma points matrix
    MatrixXd Xsig_pred_;
    ///* Weights of sigma points
    VectorXd weights_;

    ///* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;
    ///* if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;
    ///* if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;

    ///* time when the state is true, in us
    long long time_us_;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;
    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    ///* Laser measurement noise standard deviation position1 in m
    double std_laspx_;
    ///* Laser measurement noise standard deviation position2 in m
    double std_laspy_;
    ///* Radar measurement noise standard deviation radius in m
    double std_radr_;
    ///* Radar measurement noise standard deviation angle in rad
    double std_radphi_;
    ///* Radar measurement noise standard deviation radius change in m/s
    double std_radrd_;

    ///* the current NIS for radar
    double NIS_radar_;
    ///* the current NIS for laser
    double NIS_laser_;

    UnscentedKalmanFilter();

    /// @brief Processes a measurement package
    /// @param meas_package The latest measurement data of either radar or laser
    void ProcessMeasurement(MeasurementPackage meas_package);

    /// @brief PredictionStep Predicts sigma points, the state, and the state covariance matrix
    /// @param delta_t Time between k and k+1 in s
    void PredictionStep(double delta_t);

    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateLidar(MeasurementPackage meas_package);

    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateRadar(MeasurementPackage meas_package);

    /// @brief Initializes the Kalman filter with a first given measurement
    void InitializeWithFirstMasurement(const MeasurementPackage& meas_package);

    /// @brief Generates a set of sigma points for an augmented state.
    void GenerateAugmentedSigmaPoints(MatrixXd& Xsig_out);

    /// @brief Sigma points gets predicted to a new state.
    void PredictSigmaPoints(const MatrixXd& Xsig_aug, double delta_t);

    /// @brief Based on the predicted sigma points the mean and covariance is predicted.
    void PredictMeanAndCovariance();

    /// @brief A Laser measurement is predicted to a new state
    void PredictLaserMeasurement(const int n_z, VectorXd& z_pred, MatrixXd& S, MatrixXd& Zsig);

    /// @brief A Radar measurement is predicted to a new state
    void PredictRadarMeasurement(const int n_z, VectorXd& z_pred, MatrixXd& S, MatrixXd& Zsig);

    /// @brief The state is updated using the predicted measurements
    void UpdateState(const int n_z, const VectorXd& z_pred, const MatrixXd& S, const MatrixXd& Zsig, const VectorXd& z);

    VectorXd& GetState() { return x_; }
    MatrixXd& GetCovariance() { return P_; }

    double GetStdDevAcceleration() { return std_a_; }
    double GetStdDevYawAcceleration() { return std_yawdd_; }

    int GetStateSize() { return n_x_; }
    int GetAugmentedStateSize() { return n_aug_; }

    MatrixXd& GetPredictedSigmaPoints() { return Xsig_pred_; }

  private:
    SigmapointManager sigma_manager_;

    void InitializeProcessNoise();
    void InitialzeMeasurementNoise();
    void InitializeCovariance();

    void NormAngle(double& angle);
    void SetupWeights();
};
