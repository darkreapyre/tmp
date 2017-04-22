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
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  for(int i = 0; i < estimations.size(); i++){
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }
  
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

double Tools::NormalizeAngles(const double radians) {
  const double max = M_PI;
  const double min = -M_PI;
  return radians < min ? max + std::fmod(radians - min, max - min) : std::fmod(radians - min, max - min) + min;
}
