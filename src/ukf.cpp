#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...

  */

  /*************************************************************************
  * Complete initialization
  **************************************************************************/

  is_initialized_ = false; //Set to false

  n_x_ = 5; //State dimention

  n_aug_ = 7; //Augmented state dimension

  lambda_ = 3 - n_x_; //Sigma point spreading parameter

  Xsig_pred_  = MatrixXd(n_x_, 2 * n_aug_ + 1); //Predicted sigma points matrix

  weights_ = VectoXd(2 * n_aug_ + 1); //Sigma point weights

  VectorXd x_aug_ = VectorXd(7); //Initialize augmented mean vector

  MatrixXd P_aug_ = MatrixXd(7, 7); //Initialize augmented state covariance matrix

  MatrixXd Xsig_aug_ = MatrixXd(n_aug, 2 * n_aug_ + 1); //Initialize augmented sigma points matrix

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  /*******************************************************************************
  * Initial Measurements (RADAR and LIDAR)
  ********************************************************************************/

  if (!is_initialized_) {
    // Initial LIDAR measurement
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      float px = meas_package.raw_measurements_(0);
      float py = meas_package.raw_measuments_(1);
      x_ << px, py, 0, 0, 0;
      is_initialized_ = true;
    } else if (mease_package.sensor_type_ = MeasuementPackage::RADAR) { // Initial RADAR measurement
      float px = meas_package.raw_measurements_(0) * cos(mease_package.raw_measurements_(1));
      float py = meas_package.raw_measurements_(0) * sin(mease_package.raw_measurements_(1));
      float v = mease_package.raw_measurements_(2);
      float yaw = meas_package.raw_measurements_(1);
      x_ << px, py, 0, 0, 0;
      in_initialized_ = true;
    }
    time_us_ = meas_package.timestamp_;
    return;
  }

  /***********************************************************************************
  * Subsequent Mesurements (RADAR and LIDAR)
  ************************************************************************************/

  double delta_t_ = (mease_package.timestamp_ - time_us_) / 1000000.0;
  UKF::Prediction(delta_t_);

  if (mease_package.sesor_type_ == MeasurementPackage::LASER && use_laser_) {
    UKF::UpdateLidar(mease_package);
  } else if (mease_package.sensor_type_ == MeasurementPackage::RADAR ** use_radar_) {
    UKF::UpdateRadar(mease_package);
  }
  time_us_ = meas_package.timestamp_;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  /**********************************************************************************
  * Augmented sigma points
  ***********************************************************************************/

  //Augmented mean state
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;

  //Augmented covariance matrix
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(5, 5) = P_;
  P_aug_(5, 5) = std_a_ * std_a_;
  P_aug_(6, 6) = std_yawdd_ * std_yawdd_;

  //Square root matrix
  MatrixXd L = P_aug_.llt().matrixL();

  //Augmented sigma points
  Xsig_aug_.col(0) = x_aug_;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug_.col(i + 1) = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  /***********************************************************************************
  * Predict sigma points
  ************************************************************************************/

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double px = Xsig_aug_(0, i);
    double py = Xsig_aug_(1, i);
    double v = Xsig_aug_(2, i);
    double yaw = Xsig_aug_(3, i);
    double yawd = Xsig_aug_(4, i); //yaw rate
    double nu_a = Xsig_aug_(5, i); //acceleration noise
    double nu_yawdd = Xsig_aug_(6, i); //yaw acceleration noise
    double px_pred; 
    double py_pred;
    double v_pred = v;
    double yaw_pred = yaw + yawd * delta_t;
    double yawd_pred = yawd;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_pred = px + v / yawd * (sin (yaw + yawd * delta_t) - sin(yaw));
        py_pred = py + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else {
        px_pred = px + v * delta_t * cos(yaw);
        py_pred = py + v * delta_t * sin(yaw);
    }

    //add noise
    px_pred = px_pred + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_pred = px_pred + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_pred = v_pred + nu_a * delta_t;

    yaw_pred = yaw_pred + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_pred = yawd_pred + nu_yawdd * delta_t;

    //predicted sigma
    Xsig_pred_(0, i) = px_pred;
    Xsig_pred_(1, i) = py_pred;
    Xsig_pred_(2, i) = v_pred;
    Xsig_pred_(3, i) = yaw_pred;
    Xsig_pred_(4, i) = yawd_pred;
  }

  /*************************************************************************************
  * Predicted state
  **************************************************************************************/

  //set weights
  weights_(0) = lambda_ / (lambda_ + n_aug_); //Eleviate declaring weight_0 variable
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {
    weights_(i) = 0.5 / ( n_aug_ + lambda_); //Eleviate declaring weight variable
  }

  //predicted state
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    x_ = x_ + weights(i) * Xsig_pred_.col(i);
  }

  /**************************************************************************************
  * Predicted state covariance matrix
  ***************************************************************************************/

  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    //state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) <- M_PI) x_diff(3) += 2. * M_PI;

    P_ = P_ + weights(i) * x_diff * x_diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  /******************************************************************************
  * Transform sigma points into measurement space
  *******************************************************************************/

  int n_z = 2; //measurement dimensions for lidar
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double px_pred = Xsig_pred_(0, i);
    double py_pred = Xsig_pred_(1, i);
//    double v = Xsig_pred(2, i);
//    double yaw = Xsig_pred(3, i);
//    double v1 = cos(yaw) * v;
//    double v2 = sin(yaw) * v;

    // measurement model
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1); //create matrix for sigma points in measurement space
    Zsig(0, i) = px_pred;
//    Zsig(0, i) = sqrt(px_pred * px_pred + py_pred * py_pred);
    Zsig(1, i) = py_pred;
//    Zsig(1, i) = atan2(py_pred, px_pred);
//    Zsig(2, i) = (px_pred * v1 + py_pred * v2) / sqrt(px_pred * px_pred + py_pred * py_pred);
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) <- M_PI) z_diff(1) += 2. * M_PI;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  //measurement noise and covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_ * std_laspx_, 0,
       0, std_laspy_ * std_laspy_;
  
  S = S + R;

  /******************************************************************************
  * Modify state and covariance matrix
  *******************************************************************************/

  //cross correlation matrix
  MatrixXd Tc = MatrixXd(n_z, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ +1; i++) {
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while (z-diff(1) <- M_PI) z_diff(1) += 2. * M_PI;

    //state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    //angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) <- M_PI) x_diff(3)) += 2. * M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K
  MatrixXd K = Tc * S.inverse();

  //residual
  VextorXd z_diff = mease_package.raw_measurements_ - z_pred;
  //angle normalization <- TBD
  while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
  while (z_diff(1) <- M_PI) z_diff += 2. * M_PI;

  //update state and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  /*****************************************************************************
  * lidar NIS
  ******************************************************************************/

  NIS_laser_ = z_diff.trabspose() * S.inverse() * z_diff;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  /**********************************************************************************
  * Transform sigma points into measurement space
  ***********************************************************************************/
}
