#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  /**************************************************************************
  * Calculate RMSE
  ***************************************************************************/
  int n = estimations.size();
  vector<VectorXd>residuals;
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  VectorXd mean(4);

  if (n == 0) {
    std::cout << "Estimation vecor size is zero" << std::endl;
    return rmse;
  }

  for(int i = 0; i < n; ++i){
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse = rmse.array() + residual.array();
  }
  
  rmse = rmse / n;
  rmse = rmse.array().sqrt();
  return rmse;
}