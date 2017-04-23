#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    
  /**************************************************************************
  * Calculate RMSE
  ***************************************************************************/

    int n = estimations.size();
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // accumulate squared residuals
    for(int i = 0; i < n; ++i){
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse = rmse.array() + residual.array();
    }

    // calculate the mean
    rmse = rmse / n;

    // calculate the squared root
    rmse = rmse.array().sqrt();

    //return rmse
    return rmse;
}
