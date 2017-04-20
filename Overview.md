The project has two files that you will need to modify: ukf.cpp and tools.cpp

UKF.cpp
The ukf.cpp provides a template for your unscented Kalman filter code. The first step will be to initialize your parameters and matrices Here is a snippet from that part of the code:

```cpp
/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

. . . . .
```

## Tuning Process Noise
We have provided parameter values for measurement noise as well as process noise. The measurement noise values should not be changed; these are provided by the sensor manufacturer.

The values for the process noise std_a_ and std_yawdd_ are both set to 30. These will need to be adjusted in order to get your Kalman filter working. Think about what a standard deviation of 30 means. For a Gaussian distribution, we expect the acceleration to be between −60
​s
​2
​​ 
​
​m
​​  and +60
​s
​2
​​ 
​
​m
​​  or −60
​s
​2
​​ 
​
​rad
​​  and +60
​s
​2
​​ 
​
​rad
​​  ninety-five percent of the time .

That seems quite high! To put those values in perspective, the fastest measured linear acceleration for a street legal sports car is currently 0 to 60 mph in 2.2 seconds. 0 to 60 mph in 2.2 seconds is about 12
​s
​2
​​ 
​
​m
​​ . The bike simulation probably tends to have even lower acceleration.

Once your unscented Kalman filter is coded, you'll have to experiment with different process noise values to try and lower RMSE.

Initializing Variables
You will need to initialize other variables besides the ones given in the ukf.cpp template. We have defined all of the variables that you will need in ukf.h. You can look at ukf.h to see what those variables are called, but there is no need to modify ukf.h.

Pay special attention to how you initialize x and P. For more information go back to the unscented Kalman filter lectures notes titled "What to Expect from the Project".
Prediction and Update
The rest of the code template contains functions for running the prediction and update steps:

```cpp
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
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
```

As a reminder, the ProcessMeasurement() function gets called in main.cpp. The main.cpp code contains a for loop that iterates through the data file one line at a time. For each line in the data file, ProcessMeasurement() gets called sending the sensor data to ukf.cpp
tools.cpp
The tools.cpp file is similar to the EKF tools file. For this project, you only need to calculate RMSE.

```cpp
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
}
```

## EKF Versus UKF Repositories
The EKF and UKF repositories are similar, but they also have small differences.

In the EKF project, there was a separate KalmanFilter class for storing variables and calling the predict and update steps. In this project all of the Kalman filter code will go in the ukf.cpp file.

Another difference is that main.cpp will output NIS (recall that NIS is a metric that helps with parameter tuning). Here is the code from main.cpp that outputs NIS:

```cpp

    // output the NIS values

    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
      out_file_ << ukf.NIS_laser_ << "\n";
    } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      out_file_ << ukf.NIS_radar_ << "\n";
    }
```

Therefore, as part of your code, you will need to store laser and radar NIS. The ukf.cpp starter code shows where to calculate NIS. The NIS values should be stored in the NIS_radar_ and NIS_laser_ variables. You can see how these variables are defined in the ukf.h file.




$$\sigma^2_a$$

