#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "Eigen/Dense"

#include "main_utils.h"
#include "tools.h"

#include "measurement_package.h"
#include "ground_truth_package.h"
#include "udacity.h"
#include "ukf.h"

using namespace std;
using std::vector;

using Eigen::MatrixXd;
using Eigen::VectorXd;

void CallQuizMethods()
{
    MatrixXd Xsig = MatrixXd(11, 5);
    MatrixXd Xsig_aug = MatrixXd(15, 7);
    MatrixXd Xsig_pred = MatrixXd(15, 5);
    VectorXd x_pred = VectorXd(5);
    MatrixXd P_pred = MatrixXd(5, 5);
    VectorXd z_out = VectorXd(3);
    MatrixXd S_out = MatrixXd(3, 3);
    VectorXd x_out = VectorXd(5);
    MatrixXd P_out = MatrixXd(5, 5);

    udacity::GenerateSigmaPoints(&Xsig);
    udacity::AugmentedSigmaPoints(&Xsig_aug);
    udacity::SigmaPointPrediction(&Xsig_pred);
    udacity::PredictMeanAndCovariance(&x_pred, &P_pred);
    udacity::PredictRadarMeasurement(&z_out, &S_out);
    udacity::UpdateState(&x_out, &P_out);
}

int main(int argc, char* argv[])
{
    CheckArguments(argc, argv);

    string in_file_name_ = argv[1];
    ifstream in_file_(in_file_name_.c_str(), ifstream::in);

    string out_file_name_ = argv[2];
    ofstream out_file_(out_file_name_.c_str(), ofstream::out);

    CheckFiles(in_file_, in_file_name_, out_file_, out_file_name_);

    vector<MeasurementPackage> measurement_pack_list;
    vector<GroundTruthPackage> gt_pack_list;

    string line;

    // prep the measurement packages (each line represents a measurement at a
    // timestamp)
    auto kMax = 10;
    auto counter = 0;

    while (getline(in_file_, line))
    {
        counter++;
        if (counter >= kMax)
            break;

        string sensor_type;
        MeasurementPackage meas_package;
        GroundTruthPackage gt_package;
        istringstream iss(line);

        // reads first element from the current line
        iss >> sensor_type;
        if (sensor_type.compare("L") == 0)
        {
            ReadLaserMeasurement(meas_package, iss, measurement_pack_list);
        }
        else if (sensor_type.compare("R") == 0)
        {
            ReadRadarMeasurement(meas_package, iss, measurement_pack_list);
        }

        ReadGroundTruth(iss, gt_package, gt_pack_list);
    }

    // Create a UKF instance
    UKF ukf;

    // Used to compute the RMSE later
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    // Call the UKF-based fusion
    size_t N = measurement_pack_list.size();
    for (size_t k = 0; k < N; ++k)
    {
        std::cout << "Cycle: " << k + 1 << std::endl;
        // Start filtering from the second frame (the speed is unknown in the first frame)
        ukf.ProcessMeasurement(measurement_pack_list[k]);

        // Write estimations to output file
        OutputEstimations(out_file_, ukf, measurement_pack_list, k, gt_pack_list);

        // Store ground truth and current Kalman state
        estimations.push_back(ukf.x_);
        ground_truth.push_back(gt_pack_list[k].gt_values_);
    }

    // Compute the accuracy (RMSE)
    std::cout << "Accuracy - RMSE: \n" << Tools::CalculateRMSE(estimations, ground_truth) << std::endl;

    // Close files
    if (out_file_.is_open())
    {
        out_file_.close();
    }

    if (in_file_.is_open())
    {
        in_file_.close();
    }

    return 0;
}

int main2(int argc, char* argv[])
{
    CallQuizMethods();

    CheckArguments(argc, argv);

    string in_file_name_ = argv[1];
    ifstream in_file_(in_file_name_.c_str(), ifstream::in);

    string out_file_name_ = argv[2];
    ofstream out_file_(out_file_name_.c_str(), ofstream::out);

    CheckFiles(in_file_, in_file_name_, out_file_, out_file_name_);

    /**********************************************
     *  Set Measurements                          *
     **********************************************/

    vector<MeasurementPackage> measurement_pack_list;
    string line;

    // prep the measurement packages (each line represents a measurement at a
    // timestamp)
    auto kMax = 10;
    auto counter = 0;
    while (getline(in_file_, line))
    {
        counter++;
        if (counter >= kMax)
            break;

        string sensor_type;
        MeasurementPackage meas_package;
        istringstream iss(line);
        long long timestamp;

        // reads first element from the current line
        iss >> sensor_type;

        if (sensor_type.compare("L") == 0)
        {
            // laser measurement

            // read measurements at this timestamp
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float px;
            float py;
            iss >> px;
            iss >> py;
            meas_package.raw_measurements_ << px, py;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        }
        else if (sensor_type.compare("R") == 0)
        {
            // radar measurement

            // read measurements at this timestamp
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro;
            float theta;
            float ro_dot;
            iss >> ro;
            iss >> theta;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro, theta, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        }
    }

    // Create a UKF instance
    UKF ukf;

    size_t number_of_measurements = measurement_pack_list.size();

    // start filtering from the second frame (the speed is unknown in the first
    // frame)
    for (size_t k = 0; k < number_of_measurements; ++k)
    {
        // Call the UKF-based fusion
        ukf.ProcessMeasurement(measurement_pack_list[k]);

        // output the estimation
        out_file_ << ukf.x_(0) << "\t";  // pos1 - est
        out_file_ << ukf.x_(1) << "\t";  // pos2 - est
        out_file_ << ukf.x_(2) << "\t";  // vel_abs -est
        out_file_ << ukf.x_(3) << "\t";  // yaw_angle -est
        out_file_ << ukf.x_(4) << "\t";  // yaw_rate -est

        // output the measurements
        if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER)
        {
            // output the estimation

            // p1 - meas
            out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";

            // p2 - meas
            out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
        }
        else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR)
        {
            // output the estimation in the cartesian coordinates
            double ro = measurement_pack_list[k].raw_measurements_(0);
            double phi = measurement_pack_list[k].raw_measurements_(1);
            out_file_ << ro * cos(phi) << "\t";  // p1_meas
            out_file_ << ro * sin(phi) << "\t";  // p2_meas
        }

        out_file_ << "\n";
    }

    // close files
    if (out_file_.is_open())
    {
        out_file_.close();
    }

    if (in_file_.is_open())
    {
        in_file_.close();
    }

    cout << "Done!" << endl;
    return 0;
}
