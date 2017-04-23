#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

    int est_size = estimations.size();
    vector<VectorXd> residuals;
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    VectorXd mean(4);

    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (est_size == 0) {
        std::cout << "Estimantions vector size is 0" << std::endl;
        return rmse;
    } else if (est_size != ground_truth.size()) {
        std::cout << "Estimantions vector size is not equal to Ground Truth vector size" << std::endl;
        return rmse;
    }

    //accumulate squared residuals
    for(int i=0; i < est_size; ++i){
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse = rmse.array() + residual.array();
    }
    //calculate the mean
    rmse = rmse / est_size;
    //calculate the squared root
    rmse = rmse.array().sqrt();


    //return the result
    return rmse;
}
