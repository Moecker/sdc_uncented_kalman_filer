#include <iostream>

#include "ukf.h"

UKF::UKF()
        : n_x_(5),
          n_aug_(7),
          lambda_(3 - n_aug_),
          x_(VectorXd(n_x_)),
          P_(MatrixXd(n_x_, n_x_)),
          Xsig_pred_(MatrixXd(n_x_, 2 * n_aug_ + 1)),
          weights_(VectorXd(2 * n_aug_ + 1)),
          time_us_(0),
          is_initialized_(false),
          use_laser_(false),
          use_radar_(true)
{
    InitializeProcessNoise();
    InitialzeMeasurementNoise();
    InitializeCovariance();
}

void UKF::InitializeProcessNoise()
{
    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 0.2;
    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.2;
}

void UKF::InitialzeMeasurementNoise()
{
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;
    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;
    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.0175;
    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.1;
}

void UKF::InitializeCovariance()
{
    const double kPx = 1.0;
    const double kPy = 1.0;
    const double kV = 10.0;
    const double kPsi = 5.0;
    const double kPsiDot = 2.0;

    // clang-format off
    P_ << kPx, 0, 0, 0, 0,
        0, kPy, 0, 0, 0,
        0, 0, kV, 0, 0,
        0, 0, 0, kPsi, 0,
        0, 0, 0, kPsiDot, 0;
    // clang-format on
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
    // Initialization
    if (!is_initialized_)
    {
        InitializeWithFirstMasurement(meas_package);
        return;
    }

    // Compute the time elapsed between the current and previous measurements
    double delta_time = (meas_package.timestamp_ - time_us_) / 1000000.0;
    std::cout << "Elapsed delta time: " << delta_time << "s" << std::endl;

    // Update state time to measurement time
    time_us_ = meas_package.timestamp_;

    // Prediction for elapsed time duration
    Prediction(delta_time);

    // Update
    if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
        UpdateLidar(meas_package);
    }
    else if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
        UpdateRadar(meas_package);
    }

    // Print the current state and covariance
    std::cout << "New Mean x_: \n" << x_ << std::endl;
    std::cout << "New Covariance P_: \n" << P_ << std::endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t)
{
    /**
    TODO:

    Complete this function! Estimate the object's location. Modify the state
    vector, x_. Predict sigma points, the state, and the state covariance matrix.
    */
    // Create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    // Call prediction chain
    GenerateAugmentedSigmaPoints(Xsig_aug);
    PredictSigmaPoints(Xsig_aug, delta_t);
    PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package)
{
    /**
    TODO:

    Complete this function! Use lidar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the lidar NIS.
    */
    auto n_z = 2;
    VectorXd z_pred = VectorXd(n_z);
    MatrixXd S = MatrixXd(n_z, n_z);
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    PredictLaserMeasurement(n_z, z_pred, S, Zsig);
    UpdateState(n_z, z_pred, S, Zsig, meas_package.raw_measurements_);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package)
{
    /**
    TODO:

    Complete this function! Use radar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the radar NIS.
    */
    auto n_z = 3;
    VectorXd z_pred = VectorXd(n_z);
    MatrixXd S = MatrixXd(n_z, n_z);
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    PredictRadarMeasurement(n_z, z_pred, S, Zsig);
    UpdateState(n_z, z_pred, S, Zsig, meas_package.raw_measurements_);
}

void UKF::InitializeWithFirstMasurement(const MeasurementPackage& meas_package)
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
    // UPdate timestamp with current measurement timestamp
    time_us_ = meas_package.timestamp_;
    std::cout << "UKF is initialized to: \n" << x_ << std::endl;

    // Done initializing, no need to predict or update
    is_initialized_ = true;
}

void UKF::GenerateAugmentedSigmaPoints(MatrixXd& Xsig_aug)
{
    // create augmented mean vector
    VectorXd x_aug = VectorXd(7);

    // create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);

    // create augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;

    // create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5, 5) = P_;
    P_aug(5, 5) = std_a_ * std_a_;
    P_aug(6, 6) = std_yawdd_ * std_yawdd_;

    // create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    // create augmented sigma points
    Xsig_aug.col(0) = x_aug;
    for (int i = 0; i < n_aug_; i++)
    {
        Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
        Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
    }

    // print result
    std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;
}

void UKF::PredictSigmaPoints(const MatrixXd& Xsig_aug, double delta_t)
{
    // predict sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; i++)
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
        Xsig_pred_(0, i) = px_p;
        Xsig_pred_(1, i) = py_p;
        Xsig_pred_(2, i) = v_p;
        Xsig_pred_(3, i) = yaw_p;
        Xsig_pred_(4, i) = yawd_p;
    }

    // print result
    std::cout << "Xsig_pred = " << std::endl << Xsig_pred_ << std::endl;
}

void UKF::PredictMeanAndCovariance()
{
    SetupWeights();

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
        // angle normalization
        while (x_diff(3) > M_PI)
            x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI)
            x_diff(3) += 2. * M_PI;

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    }

    // print result
    std::cout << "Predicted state" << std::endl;
    std::cout << x_ << std::endl;
    std::cout << "Predicted covariance matrix" << std::endl;
    std::cout << P_ << std::endl;
}

void UKF::PredictLaserMeasurement(int n_z, VectorXd& z_pred, MatrixXd& S, MatrixXd& Zsig)
{
    n_z;
    z_pred.fill(0.01);
    S.fill(0.01);
    Zsig.fill(0.01);
}

void UKF::PredictRadarMeasurement(int n_z, VectorXd& z_pred, MatrixXd& S, MatrixXd& Zsig)
{
    SetupWeights();

    // transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {  // 2n+1 sigma points

        // extract values for better readability
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        double v = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);

        double v1 = cos(yaw) * v;
        double v2 = sin(yaw) * v;

        // measurement model
        Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);                          // r
        Zsig(1, i) = atan2(p_y, p_x);                                      // phi
        Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y);  // r_dot
    }

    // mean predicted measurement
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    // measurement covariance matrix S
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {  // 2n+1 sigma points
       // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // angle normalization
        while (z_diff(1) > M_PI)
            z_diff(1) -= 2. * M_PI;
        while (z_diff(1) < -M_PI)
            z_diff(1) += 2. * M_PI;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_radr_ * std_radr_, 0, 0, 0, std_radphi_ * std_radphi_, 0, 0, 0, std_radrd_ * std_radrd_;
    S = S + R;

    // print result
    std::cout << "z_pred: " << std::endl << z_pred << std::endl;
    std::cout << "S: " << std::endl << S << std::endl;
}

void UKF::SetupWeights()
{
    double weight_0 = lambda_ / (lambda_ + n_aug_);
    weights_(0) = weight_0;
    for (int i = 1; i < 2 * n_aug_ + 1; i++)
    {
        double weight = 0.5 / (n_aug_ + lambda_);
        weights_(i) = weight;
    }
}

void UKF::UpdateState(const int n_z, const VectorXd& z_pred, const MatrixXd& S, const MatrixXd& Zsig, const VectorXd& z)
{
    SetupWeights();

    // create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);

    // calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {  // 2n+1 sigma points

        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // angle normalization
        while (z_diff(1) > M_PI)
            z_diff(1) -= 2. * M_PI;
        while (z_diff(1) < -M_PI)
            z_diff(1) += 2. * M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // angle normalization
        while (x_diff(3) > M_PI)
            x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI)
            x_diff(3) += 2. * M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = z - z_pred;

    // angle normalization
    while (z_diff(1) > M_PI)
        z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
        z_diff(1) += 2. * M_PI;

    // update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    // print result
    std::cout << "Updated state x: " << std::endl << x_ << std::endl;
    std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;
}