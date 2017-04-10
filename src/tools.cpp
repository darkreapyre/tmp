#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // Calculate the RMSE
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

//  if ((estimations.size() == 0) || (estimations.size() != ground_truth.size())) {
//    cout << "ERROR: invalid estimations or ground_truth" << endl;
//    return rmse;
//  }

  assert(estimations.size() > 0);

  assert(estimations.size() == ground_truth.size());

  VectorXd squared_residuals(estimations.size());
  for(int i = 0; i < estimations.size(); ++i){
    VextorXd residual = estimations[i] - ground_truth[i];
    squared_residuals = residual.array() * residual.array();
    rmse += squared_residuals;
  }

//  for(unsigned int i = 0; i < estimations.size(); ++i){
//    VectorXd residual = estimations[i] - ground_truth[i];
//    residual = residual.array() * residual.array();
//    rmse += residual;
//  }
  
  VectorXd mean = rmse / estimations.size();

  rmse = mean.array().sqrt();

//  rmse = rmse / estimations.size();
//  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  // Calculate Jacobian
    MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //TODO: YOUR CODE HERE
    double c1 = px*px + py*py;
    double c2 = sqrt(c1);
    //check division by zero
    if (c1<0.0001)
    {
        std::cout << "Division by zero ERROR!" << std::endl;
        return Hj;
    }
    //compute the Jacobian matrix
    double dp_dpx = px/c2;
    double dp_dpy = py/c2;
    double dp_dvx = 0;
    double dp_dvy = 0;

    double dphi_dpx = -py/c1;
    double dphi_dpy = px/c1;
    double dphi_dvx = 0;
    double dphi_dvy = 0;

    double dpdot_dpx = py*(vx*py - vy*px) / pow(c2,3);
    double dpdot_dpy = px*(vy*px - vx*py) / pow(c2,3);
    double dpdot_dvx = dp_dpx;
    double dpdot_dvy = dp_dpy;

    Hj <<   dp_dpx, dp_dpy, dp_dvx, dp_dvy,
            dphi_dpx, dphi_dpy, dphi_dvx, dphi_dvy,
            dpdot_dpx, dpdot_dpy, dpdot_dvx, dpdot_dvy;

    return Hj;

}