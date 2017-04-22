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

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.8;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.53;

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

  lambda_ = 3 - n_aug_; //Sigma point spreading parameter
  
  n_z_ = 3; //dimnesion space for RADAR

  n_l_ = 2; //dimnesion space for LASER

  x_ = VectorXd(n_x_); //initialize x_
  x_ << 1, 1, 0, 0, 0;

  x_aug_ = VectorXd(n_aug_); //augmented state vector

  P_aug_ = MatrixXd(n_aug_, n_aug_); //augmented covariance matrix

  Q_ = MatrixXd(2, 2);
  Q_ << std_a_ * std_a_; 0.0,
        0.0, std_yadd_ * std_yawdd_;
  
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1) ; //Augmented sigma points matrix

//  Xsig_pred_  = MatrixXd(n_x_, 2 * n_aug_ + 1); //Predicted sigma points matrix

  P_ = MatrixXd(n_x_, n_x_); //Updte covariance matrix
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;
  
  T_ = VectorXd(5); //Transition vector

  N_ = VectorXd(5); //Noise vector 

  weights_ = VectorXd(2 * n_aug_ + 1); //Sigma point weights
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }

//  VectorXd x_aug_ = VectorXd(7); //Initialize augmented mean vector

//  MatrixXd P_aug_ = MatrixXd(7, 7); //Initialize augmented state covariance matrix

//  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1); //Initialize augmented sigma points matrix

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
  float px = 0.0; //Initialize
  float py = 0.0; //Initialize

  if (!is_initialized_) {
    x_.fill(0.0);
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      float rho = meas_package.raw_mesurements_(0);
      float phi = mease_package.raw_measurements_(1);
      px = rho * cos(phi);
      py = rho * sin(phi);
      is_initialized = true;
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      px = meas_package.raw_measurements_(0);
      py = meas_package.raw_measurements_(1);
      is_initialized = true;
    }

    //avoid division by zero
    if (fabs(px) > 0.001 || fabs(py) > 0.001) { // <-- TBC on py
      x_(0) = px;
      x_(1) = py;
    } else {
      x_(0) = 0.001;
      x_(1) = 0.001;
    }
    time_us_ = meas_package.timestamp_;
    return;

//    time_us_ = meas_package.timestamp_;
//    // Initial LIDAR measurement
//    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
//      float px = meas_package.raw_measurements_(0);
//      float py = meas_package.raw_measurements_(1);
//      x_ << px, py, 0, 0, 0;
//      is_initialized_ = true;
//    } else if (meas_package.sensor_type_ = MeasurementPackage::RADAR) { // Initial RADAR measurement
//      float px = meas_package.raw_measurements_(0) * cos(meas_package.raw_measurements_(1));
//      float py = meas_package.raw_measurements_(0) * sin(meas_package.raw_measurements_(1));
//      float v = meas_package.raw_measurements_(2);
//      float yaw = meas_package.raw_measurements_(1);
//      x_ << px, py, 0, 0, 0;
//      is_initialized_ = true;
//    }
//    return;
  }

  /***********************************************************************************
  * Subsequent Mesurements (RADAR and LIDAR)
  ************************************************************************************/

  if (meas_package.sensor_type_ == MesurementPackage::RADAR && use_radar_) {
    double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;

    while (dt > 0.1) {
      const double delta_t = 0.05;
      UKF::Prediction(delta_t);
      dt -= delta_t;
    }

    UKF::Predition(dt);
    UKF::UpdateRadar(meas_package);

  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time)us_ = meas_package.timestamp_;

    while (dt > 0.1) {
      const double delta_t = 0.05;
      UKF::Prediction(delta_t);
      dt -= delta_t;
    }

    UKF::Prediction(dt);
    UKF::UpdateLidar(meas_package);

  }

//  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
//  Prediction(dt);

//  if ( (meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_) {
//    UpdateLidar(meas_package);
//  } else if ( (meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_) {
//    UpdateRadar(meas_package);
//  }
//  time_us_ = meas_package.timestamp_;
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
  x_aug_ << x_, 0, 0;
  P_aug_.fill(0.0);

  //Augmented covariance matrix
  P_aug_.topLeftCorner(5, 5) = P_;
  P_aug_.bottomRightCorner(2, 2) = Q_;

  //Square root Matrix L
  MatrixXd L = P_aug_.llt().matrixL();

  //Augmented sigma points
  Xsig_aug_.col(0) = x_aug_;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug_.col(i + 1) = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(i);
  }

/*
  //Augmented mean state
  VectorXd x_aug = VectorXd(7);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //Augmented covariance matrix
  MatrixXd P_aug = MatrixXd(7, 7);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  //Square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //Augmented sigma points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug_.col(i + 1) = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug_.col(n_aug_ + i + 1) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }
*/

  /***********************************************************************************
  * Predict sigma points
  ************************************************************************************/

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double p_x = Xsig_aug_(0, i);
    double p_y = Xsig_aug_(1, i);
    double v = Xsig_aug_(2, i);
    double yaw = Xsig_aug_(3, i);
    double yawd = Xsig_aug_(4, i);
    double nu_a = Xsig_aug_(5, i);
    double nu_yawdd = Xsig_aug_(6, i);
    double px_p, py_p;
    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    } else {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }

    //add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;
    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    //predict sigma points
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

/*  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double px = Xsig_aug(0, i);
    double py = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i); //yaw rate
    double nu_a = Xsig_aug(5, i); //acceleration noise
    double nu_yawd = Xsig_aug(6, i); //yaw acceleration noise
    double delta_t_squared = delta_t * delta_t;
    double px_pred; 
    double py_pred;
    double v_pred;
    double yaw_pred;
    double yawd_pred;

    //avoid division by zero
    yaw_pred = yaw + yawd * delta_t;
    if (fabs(yawd) < 0.001) {
      px_pred = px + (v * cos(yaw) * delta_t);
      py_pred = py + (v * sin(yaw) * delta_t);
//        px_pred = px + (v / yawd * (sin (yaw + yawd * delta_t) - sin(yaw));
//        py_pred = py + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else {
      px_pred = px + ((v / yawd) * (sin(yaw_pred)-sin(yaw)));
      py_pred = py + ((v / yawd) * (-cos(yaw_pred)+cos(yaw)));
//        px_pred = px + v * delta_t * cos(yaw);
//        py_pred = py + v * delta_t * sin(yaw);
    }

    //add noise
    px_pred += 0.5 * delta_t_squared * cos(yaw) * nu_a;
    py_pred += 0.5 * delta_t_squared * sin(yaw) * nu_a;
    yaw_pred += 0.5 * delta_t_squared * nu_yawd;
    yawd_pred = yawd + delta_t * nu_yawd;
    v_pred = v + delta_t * nu_a;
//    px_pred = px_pred + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
//    py_pred = px_pred + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
//    v_pred = v_pred + nu_a * delta_t;
//    yaw_pred = yaw_pred + 0.5 * nu_yawdd * delta_t * delta_t;
//    yawd_pred = yawd_pred + nu_yawdd * delta_t;

    //predicted sigma
    Xsig_pred_(0, i) = px_pred;
    Xsig_pred_(1, i) = py_pred;
    Xsig_pred_(2, i) = v_pred;
    Xsig_pred_(3, i) = yaw_pred;
    Xsig_pred_(4, i) = yawd_pred;
  }
*/
  /*************************************************************************************
  * Predicted state
  **************************************************************************************/

  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  /**************************************************************************************
  * Predicted state covariance matrix
  ***************************************************************************************/

  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    //state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
//    x_diff(3) = tools_.NormalizeAngles(x_diff(3));
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
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

//  int n_z = 2; //measurement dimensions for lidar
  MatrixXd Zsig = MatrixXd(n_l_, 2 * n_aug_ + 1); //create matrix for sigma points in measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);

    if (px == 0 && py == 0) {
      Zsig.col(i) << 0, 0;
    } else {
      Zsig.col(i) << px, py;
    }

//    // measurement model
//    Zsig(0, i) = px_pred;
//    Zsig(1, i) = py_pred;
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_l_);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_l_, n_l_);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //measurement noise and covariance matrix
  MatrixXd R = MatrixXd(n_l_, n_l_);
  R << std_laspx_ * std_laspx_, 0,
       0, std_laspx_ * std_laspx_;
  
  S = S + R;

  /******************************************************************************
  * Modify state and covariance matrix
  *******************************************************************************/

  //cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_l_);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ +1; i++) {
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    //state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
//    x_diff(3) = tools_.NormalizeAngles(x_diff(3));
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z = mease_package.raw_measurement_;
  VectorXd z_diff = z - z_pred;
  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  /*****************************************************************************
  * lidar NIS
  ******************************************************************************/

  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
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

  /******************************************************************************
  * Transform sigma points into measurement space
  *******************************************************************************/

  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1); //create matrix for sigma points in measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);
    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;
    double rho = sqrt(px * px + py * py);
    double phi = atan2(py, px);
    double rho_dot = (px * v1 + py * v2) / rho;

    if (px == 0 && py ==0) {
      Zsig.col(i) << 0, 0, 0;
    } else {
      Zsig.col(i) << rho, phi, rho_dot;
    }

//    // measurement model
//    Zsig(0, i) = rho;
//    Zsig(1, i) = phi;
//    Zsig(2, i) = rho_dot;

  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_, n_z_);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
//    z_diff(1) = tools_.NormalizeAngles(z_diff(1));
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //measurement noise and covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0,
       0, std_radphi_ * std_radphi_, 0,
       0, 0, std_radrd_ * std_radrd_;
  
  S = S + R;

  /******************************************************************************
  * Modify state and covariance matrix
  *******************************************************************************/

  //cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z_);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ +1; i++) {
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
//    z_diff(1) = tools_.NormalizeAngles(z_diff(1));
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    //state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
//    x_diff(3) = tools_.NormalizeAngles(x_diff(3));
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z = meas_package.raw_measurements_;
  VectorXd z_diff = z - z_pred;
  //angle normalization
//  z_diff(1) = tools_.NormalizeAngles(z_diff(1));
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  /*****************************************************************************
  * radar NIS
  ******************************************************************************/

  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}
