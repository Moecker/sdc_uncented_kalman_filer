#include <iostream>

#include "tools.h"
#include "unscented_kalman_filter.h"

UnscentedKalmanFilter::UnscentedKalmanFilter()
        : use_debug_outputs_(false),
          use_laser_(true),
          use_radar_(true),
          sigma_manager_(*this, use_debug_outputs_),
          laser_manager_(*this, use_debug_outputs_),
          radar_manager_(*this, use_debug_outputs_),
          n_x_(5),
          n_aug_(7),
          lambda_(3 - n_aug_),
          x_(VectorXd(n_x_)),
          P_(MatrixXd(n_x_, n_x_)),
          Xsig_pred_(MatrixXd(n_x_, 2 * n_aug_ + 1)),
          weights_(VectorXd(2 * n_aug_ + 1)),
          time_us_(0),
          is_initialized_(false)
{
    InitializeProcessNoise();
    InitializeCovariance();
    SetupWeights();
}

void UnscentedKalmanFilter::InitializeProcessNoise()
{
    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 0.2;
    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.4;
}

void UnscentedKalmanFilter::InitializeCovariance()
{
    const double kUncertain = 1000;
    const double kPx = 1.0;
    const double kPy = 1.0;
    const double kV = kUncertain;
    const double kPsi = kUncertain;
    const double kPsiDot = 100.0;

    // clang-format off
    P_ << kPx, 0, 0, 0, 0,
        0, kPy, 0, 0, 0,
        0, 0, kV, 0, 0,
        0, 0, 0, kPsi, 0,
        0, 0, 0, kPsiDot, 0;
    // clang-format on
}

void UnscentedKalmanFilter::ProcessMeasurement(MeasurementPackage meas_package)
{
    // Initialization
    if (!is_initialized_)
    {
        InitializeWithFirstMasurement(meas_package);
        return;
    }

    // Compute the time elapsed between the current and previous measurements
    double delta_time = (meas_package.timestamp_ - time_us_) / 1000000.0;
    if (use_debug_outputs_)
    {
        std::cout << "Elapsed delta time: " << delta_time << "s" << std::endl;
    }

    // Update state time to measurement time
    time_us_ = meas_package.timestamp_;

    // Prediction for elapsed time duration
    PredictionStep(delta_time);

    // Update
    if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
        laser_manager_.Update(meas_package);
    }
    else if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
        radar_manager_.Update(meas_package);
    }

    // Print the current state and covariance
    if (use_debug_outputs_)
    {
        std::cout << "New Mean x_: \n" << x_ << std::endl;
        std::cout << "New Covariance P_: \n" << P_ << std::endl;
    }
}

void UnscentedKalmanFilter::PredictionStep(double delta_t)
{
    // Estimate the object's location. Modify the state
    // vector, x_. Predict sigma points, the state, and the state covariance matrix.

    // Create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    // Call prediction chain
    sigma_manager_.GenerateAugmentedSigmaPoints(Xsig_aug);
    sigma_manager_.PredictSigmaPoints(Xsig_aug, delta_t);

    PredictMeanAndCovariance();
}

void UnscentedKalmanFilter::InitializeWithFirstMasurement(const MeasurementPackage& meas_package)
{
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
        // Converts radar measurement from polar into euclidean coordinate system
        double ro = meas_package.raw_measurements_(0);
        double phi = meas_package.raw_measurements_(1);
        auto x = ro * cos(phi);
        auto y = ro * sin(phi);
        x_ << x, y, 0, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
        x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
    }

    // Update timestamp with current measurement timestamp
    time_us_ = meas_package.timestamp_;
    std::cout << "UnscentedKalmanFilter is initialized to: \n" << x_ << std::endl;

    // Done initializing, no need to predict or update
    is_initialized_ = true;
}

void UnscentedKalmanFilter::PredictMeanAndCovariance()
{
    // predicted state mean
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {  // iterate over sigma points
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }

    // predicted state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {  // iterate over sigma points

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        Tools::NormAngle(x_diff(3));

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    }

    // print result
    if (use_debug_outputs_)
    {
        std::cout << "Predicted state" << std::endl;
        std::cout << x_ << std::endl;
        std::cout << "Predicted covariance matrix" << std::endl;
        std::cout << P_ << std::endl;
    }
}

void UnscentedKalmanFilter::UpdateState(const int n_z,
                                        const VectorXd& z_pred,
                                        const MatrixXd& S,
                                        const MatrixXd& Zsig,
                                        const VectorXd& z)
{
    // create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);

    // calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {  // 2n+1 sigma points

        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        Tools::NormAngle(z_diff(1));

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        Tools::NormAngle(x_diff(3));

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = z - z_pred;

    Tools::NormAngle(z_diff(1));

    // update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    // print result
    if (use_debug_outputs_)
    {
        std::cout << "Updated state x: " << std::endl << x_ << std::endl;
        std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;
    }
}

void UnscentedKalmanFilter::SetupWeights()
{
    double weight_0 = lambda_ / (lambda_ + n_aug_);
    weights_(0) = weight_0;
    for (int i = 1; i < 2 * n_aug_ + 1; i++)
    {
        double weight = 0.5 / (n_aug_ + lambda_);
        weights_(i) = weight;
    }
}