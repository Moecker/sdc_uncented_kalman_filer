#pragma once

#include <vector>

#include "Eigen/Dense"
#include "measurement_package.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace udacity
{

void GenerateSigmaPoints(MatrixXd* Xsig_out);
void AugmentedSigmaPoints(MatrixXd* Xsig_out);
void SigmaPointPrediction(MatrixXd* Xsig_out);
void PredictMeanAndCovariance(VectorXd* x_pred, MatrixXd* P_pred);
void PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out);
void UpdateState(VectorXd* x_out, MatrixXd* P_out);
}
