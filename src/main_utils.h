#pragma once

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <sstream>

#include "Eigen/Dense"

#include "ukf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"

using namespace std;
using Eigen::VectorXd;

void ReadLaserMeasurement(MeasurementPackage& meas_package,
                          istringstream& iss,
                          vector<MeasurementPackage>& measurement_pack_list);

void ReadRadarMeasurement(MeasurementPackage& meas_package,
                          istringstream& iss,
                          vector<MeasurementPackage>& measurement_pack_list);

void ReadGroundTruth(istringstream& iss, GroundTruthPackage& gt_package, vector<GroundTruthPackage>& gt_pack_list);

void OutputEstimations(ofstream& out_file_,
                       UKF& ufk,
                       vector<MeasurementPackage>& measurement_pack_list,
                       size_t k,
                       vector<GroundTruthPackage>& gt_pack_list);

void CheckArguments(int argc, char* argv[]);

void CheckFiles(ifstream& in_file, string& in_name, ofstream& out_file, string& out_name);