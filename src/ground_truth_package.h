#pragma once

#include "Eigen/Dense"

using Eigen::VectorXd;

class GroundTruthPackage
{
  public:
    long long timestamp_;

    enum SensorType
    {
        LASER,
        RADAR
    } sensor_type_;

    VectorXd gt_values_;
};
