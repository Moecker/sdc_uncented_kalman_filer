#include <iostream>

#include "tools.h"

VectorXd Tools::CalculateRMSE(const std::vector<VectorXd>& estimations, const std::vector<VectorXd>& ground_truth)
{
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

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
        // Get current items
        auto estimation = estimations[i];
        auto truth = ground_truth[i];

        // Get the estimated position directly from the state vector
        VectorXd estimated_position = estimation.head(2);

        // The velocity cannot be taken directly from the state vector but must be derived via psi and v
        auto v = estimation(2);
        auto psi = estimation(3);
        auto v_x = cos(psi) * v;
        auto v_y = sin(psi) * v;

        VectorXd actual = VectorXd(4);
        actual << estimated_position, v_x, v_y;

        VectorXd residual = actual - truth;

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
