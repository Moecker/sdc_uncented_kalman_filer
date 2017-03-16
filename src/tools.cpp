#include <iostream>

#include "tools.h"

VectorXd Tools::CalculateRMSE(const std::vector<VectorXd>& estimations, const std::vector<VectorXd>& ground_truth)
{
    VectorXd rmse(2);
    rmse << 0, 0;

    // Check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != ground_truth.size() || estimations.size() == 0)
    {
        std::cout << "Invalid estimation or ground_truth data" << std::endl;
        return rmse;
    }

    // Accumulate squared residuals
    for (unsigned int i = 0; i < estimations.size(); ++i)
    {
        VectorXd pos = ground_truth[i].head(2);
        VectorXd est = estimations[i].head(2);

        VectorXd residual = est - pos;

        // Coefficient-wise multiplication
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    // Calculate the mean
    rmse = rmse / static_cast<double>(estimations.size());

    // Calculate the squared root
    rmse = rmse.array().sqrt();

    return rmse;
}
