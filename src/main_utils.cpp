#include "main_utils.h"

void ReadLaserMeasurement(MeasurementPackage& meas_package,
                          istringstream& iss,
                          vector<MeasurementPackage>& measurement_pack_list)
{
    // LASER MEASUREMENT

    // read measurements at this timestamp
    meas_package.sensor_type_ = MeasurementPackage::LASER;
    meas_package.raw_measurements_ = VectorXd(2);
    float x;
    float y;
    long long timestamp;

    iss >> x;
    iss >> y;
    meas_package.raw_measurements_ << x, y;

    iss >> timestamp;
    meas_package.timestamp_ = timestamp;

    measurement_pack_list.push_back(meas_package);
}

void ReadRadarMeasurement(MeasurementPackage& meas_package,
                          istringstream& iss,
                          vector<MeasurementPackage>& measurement_pack_list)
{
    // RADAR MEASUREMENT

    // read measurements at this timestamp
    meas_package.sensor_type_ = MeasurementPackage::RADAR;
    meas_package.raw_measurements_ = VectorXd(3);
    float ro;
    float theta;
    float ro_dot;
    long long timestamp;

    iss >> ro;
    iss >> theta;
    iss >> ro_dot;
    meas_package.raw_measurements_ << ro, theta, ro_dot;

    iss >> timestamp;
    meas_package.timestamp_ = timestamp;

    measurement_pack_list.push_back(meas_package);
}

void ReadGroundTruth(istringstream& iss, GroundTruthPackage& gt_package, vector<GroundTruthPackage>& gt_pack_list)
{
    // read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;

    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;

    gt_package.gt_values_ = VectorXd(4);
    gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;

    gt_pack_list.push_back(gt_package);
}

void OutputEstimations(ofstream& out_file_,
                       UnscentedKalmanFilter& ukf,
                       vector<MeasurementPackage>& measurement_pack_list,
                       size_t k,
                       vector<GroundTruthPackage>& gt_pack_list)
{
    // Output the estimation
    const auto& state = ukf.GetState();
    out_file_ << state(0) << "\t";
    out_file_ << state(1) << "\t";
    out_file_ << state(2) << "\t";
    out_file_ << state(3) << "\t";

    // Output the measurements
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER)
    {
        // Output the estimation
        out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
        out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
    }
    else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR)
    {
        // Output the estimation in the Cartesian coordinates
        double ro = measurement_pack_list[k].raw_measurements_(0);
        double phi = measurement_pack_list[k].raw_measurements_(1);
        out_file_ << ro * cos(phi) << "\t";  // p1_meas
        out_file_ << ro * sin(phi) << "\t";  // ps_meas
    }

    // Output the ground truth packages
    out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(3) << "\n";
}

void CheckArguments(int argc, char* argv[])
{
    string usage_instructions = "Usage instructions: ";
    usage_instructions += argv[0];
    usage_instructions += " path/to/input.txt output.txt";

    bool has_valid_args = false;

    // make sure the user has provided input and output files
    if (argc == 1)
    {
        cerr << usage_instructions << endl;
    }
    else if (argc == 2)
    {
        cerr << "Please include an output file.\n" << usage_instructions << endl;
    }
    else if (argc == 3)
    {
        has_valid_args = true;
    }
    else if (argc > 3)
    {
        cerr << "Too many arguments.\n" << usage_instructions << endl;
    }

    if (!has_valid_args)
    {
        exit(EXIT_FAILURE);
    }
}

void CheckFiles(ifstream& in_file, string& in_name, ofstream& out_file, string& out_name)
{
    if (!in_file.is_open())
    {
        cerr << "Cannot open input file: " << in_name << endl;
        exit(EXIT_FAILURE);
    }

    if (!out_file.is_open())
    {
        cerr << "Cannot open output file: " << out_name << endl;
        exit(EXIT_FAILURE);
    }
}