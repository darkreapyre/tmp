#include <iostream>
#include "ukf.h"

using namespace std;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    ///* initially set to false, set to true in first call of ProcessMeasurement
    is_initialized_ = false;

    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;


    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 0.8; // 0.3

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

    /*************************************************************************
    * Complete initialization
    **************************************************************************/

    ///* State dimension
    n_x_ = 5;

    ///* Augmentation State diemnsion
    n_aug_ = 7;
    
    ///* Radar measurement dimensions
    n_z_ = 3;
    
    ///* Lidar measurement dimensions
    n_l_ = 2;

    ///* Sigma points spreading parameter
    lambda_ = 3 - n_aug_;

    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    x_ = VectorXd(n_x_);
    x_ << 1, 1, 0, 0, 0;

    ///* state covariance matrix
    P_ = MatrixXd(n_x_, n_x_);
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

    ///* Augmented state vector
    x_aug_ = VectorXd(n_aug_);

    ///* Augmented covariance matrix
    P_aug_ = MatrixXd(n_aug_, n_aug_);

    ///* Noise
    Q_ = MatrixXd(2, 2);
    Q_ << std_a_*std_a_, 0.0,
          0.0, std_yawdd_*std_yawdd_;

    ///* sigma point augmentation matrix
    Xsig_aug_ = MatrixXd(n_aug_, n_aug_ * 2 + 1);

    ///* Transition vector
    T_ = VectorXd(5);

    ///* Noise vector
    N_ = VectorXd(5);

    ///* Predicted sigma point matrix
    Xsig_pred_ = MatrixXd(n_x_, n_aug_ * 2 + 1);

    ///* Sigma point weights
    weights_ = VectorXd(2 * n_aug_ + 1);
    double weight_0 = lambda_ / (lambda_ + n_aug_);
    weights_(0) = weight_0;
    for (int i=1; i<2 * n_aug_ + 1; i++) {
        double weight = 0.5 / (n_aug_ + lambda_);
        weights_(i) = weight;
    }
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

    /*******************************************************************************
    * Initial Measurements (RADAR and LIDAR)
    ********************************************************************************/

    float px = 0.0; //initial state px
    float py = 0.0; //initial state py

    if (!is_initialized_) {

        x_.fill(0.0);

        if (meas_package.sensor_type_ == MeasurementPackage::RADAR and use_radar_) {
            float ro = meas_package.raw_measurements_(0);
            float phi = meas_package.raw_measurements_(1);
            px = ro * cos(phi);
            py = ro * sin(phi);
            is_initialized_ = true;
        } else if (meas_package.sensor_type_ == MeasurementPackage::LASER and use_laser_) {
            px = meas_package.raw_measurements_(0);
            py = meas_package.raw_measurements_(1);
            is_initialized_ = true;
        }

        //when both measurements px and py are 0
        if (fabs(px) > 0.001 or fabs(px) > 0.001) {
            x_(0) = px;
            x_(1) = py;
        } else {
            x_(0) = 0.001;
            x_(1) = 0.001;
        }
        time_us_ = meas_package.timestamp_;
        return;
    }

    /***********************************************************************************
    * Subsequent Mesurements (RADAR and LIDAR)
    ************************************************************************************/

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR and use_radar_) {
        double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
        time_us_ = meas_package.timestamp_;

        while (dt > 0.1)
        {
            const double delta_t = 0.05;
            UKF::Prediction(delta_t);
            dt -= delta_t;
        }

        UKF::Prediction(dt);
        UKF::UpdateRadar(meas_package);

    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER and use_laser_) {
        double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;  //dt - expressed in seconds
        time_us_ = meas_package.timestamp_;

        while (dt > 0.1)
        {
            const double delta_t = 0.05;
            UKF::Prediction(delta_t);
            dt -= delta_t;
        }

        UKF::Prediction(dt);
        UKF::UpdateLidar(meas_package);

    }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

    /**********************************************************************************
    * Augmented sigma points
    ***********************************************************************************/
    
    //create augmented mean state
    x_aug_ << x_, 0, 0;
    P_aug_.fill(0.0);

    //create augmented covariance matrix
    P_aug_.topLeftCorner(5, 5) = P_;
    P_aug_.bottomRightCorner(2, 2) = Q_;

    //create square root matrix
    MatrixXd L = P_aug_.llt().matrixL();

    //create augmented sigma points
    Xsig_aug_.col(0) = x_aug_;
    for (int colnr=0; colnr < n_aug_; colnr++) {
        Xsig_aug_.col(colnr + 1)          = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(colnr);
        Xsig_aug_.col(colnr + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(colnr);
    }
    
    /***********************************************************************************
    * Predict sigma points
    ************************************************************************************/
    for (int i = 0; i<2*n_aug_+1; i++) {
        double p_x = Xsig_aug_(0,i);
        double p_y = Xsig_aug_(1,i);
        double v = Xsig_aug_(2,i);
        double yaw = Xsig_aug_(3,i);
        double yawd = Xsig_aug_(4,i);
        double nu_a = Xsig_aug_(5,i);
        double nu_yawdd = Xsig_aug_(6,i);

        //predicted state values
        double px_p, py_p;

        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
        }
        else {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
        py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
        v_p = v_p + nu_a*delta_t;

        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;

        //write predicted sigma point into right column
        Xsig_pred_(0, i) = px_p;
        Xsig_pred_(1, i) = py_p;
        Xsig_pred_(2, i) = v_p;
        Xsig_pred_(3, i) = yaw_p;
        Xsig_pred_(4, i) = yawd_p;
    }
    
    /*************************************************************************************
    * Predicted state
    **************************************************************************************/
    
    x_.fill(0.0);
    for (int i=0; i < 2 * n_aug_ + 1; i++) {
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }
    
    /**************************************************************************************
    * Predicted state covariance matrix
    ***************************************************************************************/

    P_.fill(0.0);
    for (int i=0; i < 2 * n_aug_ + 1; i++) {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        //angle normalization 
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
    
    /******************************************************************************
    * Transform sigma points into measurement space
    *******************************************************************************/

    MatrixXd Zsig = MatrixXd(n_l_, 2 * n_aug_ + 1);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        double px = Xsig_pred_(0, i);
        double py = Xsig_pred_(1, i);

        //measurement model
        if (px == 0 && py == 0) {
            Zsig.col(i) << 0, 0;
        } else {
            Zsig.col(i) << px, py;
        }
    }

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_l_);
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; i++) {
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
    MatrixXd R = MatrixXd(n_l_,n_l_);
    R << std_laspx_*std_laspx_, 0,
         0, std_laspx_*std_laspx_;

    S = S + R;
    
    /******************************************************************************
    * Modify state and covariance matrix
    *******************************************************************************/

    //cross correlation matrix
    MatrixXd Tc = MatrixXd(n_x_, n_l_);
    Tc.fill(0.0);
    for (int i=0; i < 2 * n_aug_ + 1; i++) {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        // measurement difference
        VectorXd z_diff = Zsig.col(i) - z_pred;

        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //calculate Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    VectorXd z = meas_package.raw_measurements_;
    // measurement difference for latest prediction and latest measurement
    VectorXd z_diff = z - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    //update state mean and covariance matrix
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
    
    /******************************************************************************
    * Transform sigma points into measurement space
    *******************************************************************************/
    
    MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        double px = Xsig_pred_(0, i);
        double py = Xsig_pred_(1, i);
        double v = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);
        double v1 = cos(yaw) * v;
        double v2 = sin(yaw) * v;

        //transform sigma points into measurement space
        double ro = sqrt(px*px + py*py);
        double fi = atan2(py, px);
        double rod = (px * v1 + py * v2) / ro;

        //calculate mean predicted measurement
        if (px == 0 && py == 0) {
            Zsig.col(i) << 0, 0, 0;
        } else {
            Zsig.col(i) << ro, fi, rod;
        }
    }

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z_);
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z_, n_z_);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        // measurement difference
        VectorXd z_diff = Zsig.col(i) - z_pred;

        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }
    
    //measurement noise and covariance matrix
    MatrixXd R = MatrixXd(n_z_,n_z_);
    R << std_radr_*std_radr_, 0, 0,
         0, std_radphi_*std_radphi_, 0,
         0, 0,std_radrd_*std_radrd_;

    S = S + R;
    
    /******************************************************************************
    * Modify state and covariance matrix
    *******************************************************************************/

    //cross correlation matrix
    MatrixXd Tc = MatrixXd(n_x_, n_z_);
    Tc.fill(0.0);
    for (int i=0; i < 2 * n_aug_ + 1; i++) {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        // measurement difference
        VectorXd z_diff = Zsig.col(i) - z_pred;

        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //calculate Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    VectorXd z = meas_package.raw_measurements_;
    // measurement difference for latest prediction and latest measurement
    VectorXd z_diff = z - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();
    
    /*****************************************************************************
    * radar NIS
    ******************************************************************************/
  
    NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}
